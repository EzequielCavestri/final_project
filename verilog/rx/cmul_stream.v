`timescale 1ns/1ps
`default_nettype none

// ============================================================
// cmul_pbfdaf  —  Bloque CMUL del ecualizador PBFDAF
//
// Calcula en streaming (bin a bin):
//
//   Y[k] = W0[k]·X_curr[k] + W1[k]·X_old[k]
//
// donde:
//   X_curr[k] = frame FFT actual        (del history_buffer)
//   X_old[k]  = frame FFT de hace K bloques (del history_buffer)
//   W0[k], W1[k] = coeficientes del filtro adaptivo
//
// --- Estado actual (sin LMS) ---
//   W0[k] = 1.0  para todo k  → Y = X_curr  (identidad)
//   W1[k] = 0.0  para todo k
//   Con esta inicialización: IFFT(Y) = IFFT(FFT(x)) = x ✓
//
// --- Cuando se conecte el UPDATE LMS ---
//   El puerto de escritura i_we/i_wk/i_wsel/i_W_re/i_W_im
//   se conecta al bloque UPDATE_LMS. El resto del sistema
//   NO necesita ningún cambio.
//
// --- Interfaz ---
//   Entrada : streaming bin a bin, 1 ciclo por bin, NFFT bins por frame.
//   Latencia: 1 ciclo (registro de salida).
//   Nombre de puertos de salida compatible con cmul_stream:
//     o_valid, o_start, o_yI (=o_Y_re), o_yQ (=o_Y_im)
//
// --- Contador de bin k ---
//   eff_samp = índice del bin actual (0..NFFT-1).
//   Se sincroniza con i_start igual que en history_buffer y cmul_stream.
//   En el ciclo con i_start=1 lee W[0], en el siguiente W[1], etc.
//
// --- Pesos W (RAM interna) ---
//   Implementados como arreglos de registros (style = distributed RAM).
//   Puerto de lectura: 1 ciclo combinacional (sin latencia adicional).
//   Puerto de escritura: síncrono, cualquier ciclo.
//   Política read-before-write: si LMS escribe en k=k0 en el mismo
//   ciclo en que se lee k0, la salida de ese ciclo usa el valor anterior.
//   El LMS recibe los pesos actualizados en el ciclo siguiente. ✓
//
// --- Fix respecto a cmul_pbfdaf original ---
//   - o_start corregido: o_start <= i_valid && i_start
//     (evita start espúreo si i_start queda alto con i_valid=0)
//   - Nombre de puertos de salida unificado con cmul_stream
//     (o_yI / o_yQ en lugar de o_Y_re / o_Y_im)
// ============================================================

module cmul_pbfdaf #(
    parameter integer NB_W  = 17,    // ancho de dato Q(NB_W, NBF_W)
    parameter integer NBF_W = 10,    // bits fraccionarios
    parameter integer NFFT  = 32     // bins por frame
)(
    input  wire                      clk,
    input  wire                      rst,

    // -------------------------------------------------------
    // Entrada streaming — del history_buffer
    // -------------------------------------------------------
    input  wire                      i_valid,
    input  wire                      i_start,
    input  wire signed [NB_W-1:0]   i_X0_re,   // X_curr Re
    input  wire signed [NB_W-1:0]   i_X0_im,   // X_curr Im
    input  wire signed [NB_W-1:0]   i_X1_re,   // X_old  Re
    input  wire signed [NB_W-1:0]   i_X1_im,   // X_old  Im

    // -------------------------------------------------------
    // Puerto de escritura de pesos — desde UPDATE_LMS
    // Dejar sin conectar (flotantes = 0) mientras no haya LMS.
    // -------------------------------------------------------
    input  wire                      i_we,      // write enable
    input  wire [$clog2(NFFT)-1:0]  i_wk,      // índice bin k
    input  wire                      i_wsel,    // 0=W0, 1=W1
    input  wire signed [NB_W-1:0]   i_W_re,    // peso Re a escribir
    input  wire signed [NB_W-1:0]   i_W_im,    // peso Im a escribir

    // -------------------------------------------------------
    // Salida — hacia la IFFT
    // Nombres compatibles con cmul_stream para no cambiar top_global
    // -------------------------------------------------------
    output reg                       o_valid,
    output reg                       o_start,
    output reg  signed [NB_W-1:0]   o_yI,      // Y Re
    output reg  signed [NB_W-1:0]   o_yQ,      // Y Im

    // -------------------------------------------------------
    // Diagnóstico
    // -------------------------------------------------------
    output wire [$clog2(NFFT)-1:0]  o_samp_idx  // bin actual
);

    localparam integer KW    = $clog2(NFFT);
    localparam signed [NB_W-1:0] ONE_FX = (1 << NBF_W);  // 1.0 en Q(NBF_W)

    // ============================================================
    // Banco de pesos W0[k] y W1[k]
    //   W0: coeficiente para X_curr (frame actual)
    //   W1: coeficiente para X_old  (frame anterior)
    //   Inicialización: W0=identidad (1+0j), W1=cero
    // ============================================================
    reg signed [NB_W-1:0] W0_re [0:NFFT-1];
    reg signed [NB_W-1:0] W0_im [0:NFFT-1];
    reg signed [NB_W-1:0] W1_re [0:NFFT-1];
    reg signed [NB_W-1:0] W1_im [0:NFFT-1];

    integer init_i;
    initial begin
        for (init_i = 0; init_i < NFFT; init_i = init_i + 1) begin
            W0_re[init_i] = ONE_FX;  // 1.0
            W0_im[init_i] = {NB_W{1'b0}};
            W1_re[init_i] = {NB_W{1'b0}};
            W1_im[init_i] = {NB_W{1'b0}};
        end
    end

    // ============================================================
    // Puerto de escritura de pesos (síncrono)
    // ============================================================
    always @(posedge clk) begin
        if (i_we) begin
            if (i_wsel == 1'b0) begin
                W0_re[i_wk] <= i_W_re;
                W0_im[i_wk] <= i_W_im;
            end else begin
                W1_re[i_wk] <= i_W_re;
                W1_im[i_wk] <= i_W_im;
            end
        end
    end

    // ============================================================
    // Contador de bin k — igual que en cmul_stream
    //   eff_samp: índice del bin que se presenta en este ciclo.
    //   En el ciclo con i_start=1 → bin 0.
    //   samp_cnt se actualiza al SIGUIENTE ciclo.
    //   eff_samp = i_start ? 0 : samp_cnt resuelve el desfase.
    // ============================================================
    reg [KW-1:0] samp_cnt;

    wire [KW-1:0] eff_samp = (i_valid && i_start) ? {KW{1'b0}} : samp_cnt;

    always @(posedge clk) begin
        if (rst) begin
            samp_cnt <= {KW{1'b0}};
        end else if (i_valid) begin
            samp_cnt <= (eff_samp == (NFFT-1)) ? {KW{1'b0}}
                                               : (eff_samp + 1'b1);
        end
    end

    assign o_samp_idx = eff_samp;

    // ============================================================
    // Lectura de pesos para el bin actual
    // ============================================================
    wire signed [NB_W-1:0] w0_re_rd = W0_re[eff_samp];
    wire signed [NB_W-1:0] w0_im_rd = W0_im[eff_samp];
    wire signed [NB_W-1:0] w1_re_rd = W1_re[eff_samp];
    wire signed [NB_W-1:0] w1_im_rd = W1_im[eff_samp];

    // ============================================================
    // M0 = W0 · X_curr  (combinacional)
    // ============================================================
    wire signed [NB_W-1:0] M0_re, M0_im;

    complex_mult #(
        .NB_W (NB_W),
        .NBF_W(NBF_W)
    ) u_cm0 (
        .i_aI(i_X0_re),   .i_aQ(i_X0_im),
        .i_bI(w0_re_rd),  .i_bQ(w0_im_rd),
        .o_yI(M0_re),     .o_yQ(M0_im)
    );

    // ============================================================
    // M1 = W1 · X_old  (combinacional)
    // ============================================================
    wire signed [NB_W-1:0] M1_re, M1_im;

    complex_mult #(
        .NB_W (NB_W),
        .NBF_W(NBF_W)
    ) u_cm1 (
        .i_aI(i_X1_re),   .i_aQ(i_X1_im),
        .i_bI(w1_re_rd),  .i_bQ(w1_im_rd),
        .o_yI(M1_re),     .o_yQ(M1_im)
    );

    // ============================================================
    // Suma Y = M0 + M1
    //   Se extiende 1 bit para capturar el carry de la suma.
    //   sat_trunc recorta de NB_W+1 a NB_W sin cambio de punto fijo.
    // ============================================================
    wire signed [NB_W:0] sum_re = $signed(M0_re) + $signed(M1_re);
    wire signed [NB_W:0] sum_im = $signed(M0_im) + $signed(M1_im);

    wire signed [NB_W-1:0] Y_re_sat;
    wire signed [NB_W-1:0] Y_im_sat;

    sat_trunc #(
        .NB_XI(NB_W+1), .NBF_XI(NBF_W),
        .NB_XO(NB_W),   .NBF_XO(NBF_W),
        .ROUND_EVEN(0)
    ) u_sat_re (
        .i_data(sum_re),
        .o_data(Y_re_sat)
    );

    sat_trunc #(
        .NB_XI(NB_W+1), .NBF_XI(NBF_W),
        .NB_XO(NB_W),   .NBF_XO(NBF_W),
        .ROUND_EVEN(0)
    ) u_sat_im (
        .i_data(sum_im),
        .o_data(Y_im_sat)
    );

    // ============================================================
    // Registro de salida — 1 ciclo de latencia
    // ============================================================
    always @(posedge clk) begin
        if (rst) begin
            o_valid <= 1'b0;
            o_start <= 1'b0;
            o_yI    <= {NB_W{1'b0}};
            o_yQ    <= {NB_W{1'b0}};
        end else begin
            o_valid <= i_valid;
            o_start <= i_valid && i_start;  // fix: condicionado a i_valid
            o_yI    <= i_valid ? Y_re_sat : {NB_W{1'b0}};
            o_yQ    <= i_valid ? Y_im_sat : {NB_W{1'b0}};
        end
    end

endmodule

`default_nettype wire
