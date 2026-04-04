`timescale 1ns/1ps
`default_nettype none

// ============================================================
// history_buffer
//
// Propósito en PBFDAF:
//   Almacena los K bloques FFT anteriores para construir el
//   vector de observación extendido [X_curr, X_k-1, ..., X_k-K].
//
// Interfaz:
//   - Entrada: salida STREAMING de la FFT (NB_W bits, 32 muestras/frame)
//   - Salida:  MISMO streaming sincronizado + bloque pasado (delay K frames)
//
// Funcionamiento:
//   * Cada frame de 32 muestras que entra se escribe en un banco
//     circular de K+1 entradas (RAM de frames).
//   * Simultáneamente se lee el banco que tiene K frames de antigüedad
//     -> eso es X_old = X[n-K].
//   * o_X_curr: muestra actual del frame presente.
//   * o_X_old : muestra del frame que lleva K bloques en memoria.
//   * o_valid_data se activa 1 ciclo después de i_valid (latencia RAM).
//   * o_start_out se activa 1 ciclo después de i_start (alineado).
//
// Restricciones de diseño:
//   * Se necesita que K >= 1.
//   * NFFT = 32 fijo (puede parametrizarse después).
//   * El sistema no puede escribir un nuevo frame mientras el mismo
//     banco aún está siendo leído: con streaming continuo a 32 muestras
//     por frame esto no sucede porque wr y rd son el mismo índice.
//
// Nota sobre coeficientes W0/W1:
//   En esta etapa de integración NO se pasan coeficientes por aquí.
//   El ecualizador los recibirá desde su propia ROM/RAM en una etapa
//   posterior. Mantener coeficientes en el history_buffer mezcla
//   responsabilidades y dificulta el debug. Se eliminaron.
// ============================================================

module history_buffer #(
    parameter integer NB_W = 17,    // Ancho del dato (igual que salida FFT)
    parameter integer NFFT = 32,    // Muestras por frame
    parameter integer K    = 1      // Bloques pasados a conservar (>= 1)
)(
    input  wire                      clk,
    input  wire                      rst,

    // --- Entrada: streaming de la FFT ---
    input  wire                      i_valid,   // 1 ciclo por muestra
    input  wire                      i_start,   // alto en muestra 0 del frame
    input  wire signed [NB_W-1:0]    i_xI,
    input  wire signed [NB_W-1:0]    i_xQ,

    // --- Salida: streaming sincronizado (1 ciclo de latencia) ---
    output reg                       o_valid,   // retrasado 1 ciclo (latencia RAM)
    output reg                       o_start,   // retrasado 1 ciclo
    output reg  signed [NB_W-1:0]    o_X_curr_re, // muestra actual (frame n)
    output reg  signed [NB_W-1:0]    o_X_curr_im,
    output reg  signed [NB_W-1:0]    o_X_old_re,  // muestra de hace K frames (frame n-K)
    output reg  signed [NB_W-1:0]    o_X_old_im,

    // --- Diagnóstico ---
    output wire [$clog2(K+1)-1:0]   o_wr_bank,  // banco donde se escribe actualmente
    output wire [$clog2(NFFT)-1:0]  o_samp_idx  // índice de muestra dentro del frame
);

    // ============================================================
    // Parámetros derivados
    // ============================================================
    localparam integer DEPTH    = K + 1;          // bancos circulares (K pasados + 1 actual)
    localparam integer BANK_W   = $clog2(DEPTH);  // bits para indexar bancos
    localparam integer SAMP_W   = $clog2(NFFT);   // bits para indexar muestras

    // ============================================================
    // RAM de frames: DEPTH bancos x NFFT muestras
    // Formato: cada entrada = {im[NB_W-1:0], re[NB_W-1:0]}
    // ============================================================
    reg signed [NB_W-1:0] ram_re [0:DEPTH*NFFT-1];
    reg signed [NB_W-1:0] ram_im [0:DEPTH*NFFT-1];

    integer ii;
    initial begin
        for (ii = 0; ii < DEPTH*NFFT; ii = ii+1) begin
            ram_re[ii] = 0;
            ram_im[ii] = 0;
        end
    end

    // ============================================================
    // Control de índices
    // ============================================================
    reg [BANK_W-1:0] wr_bank;   // banco en escritura (frame n)
    reg [SAMP_W-1:0] samp_cnt;  // índice muestra dentro del frame

    // Banco de lectura = banco con K frames de antigüedad
    // wr_bank avanzó wr_bank veces desde reset, por lo que
    // el banco "más viejo disponible" con K bloques de diferencia es:
    //   rd_bank = (wr_bank - K + DEPTH) % DEPTH
    // Se calcula combinacionalmente para ser usado en el mismo ciclo
    // de escritura (read-before-write no es necesario porque
    // o_X_old es registrado -> latencia 1 ciclo es correcta).

    // ============================================================
    // Corrección de timing para el ciclo i_start=1:
    //
    // Problema: samp_cnt usa <= (non-blocking). Cuando llega i_start=1
    // (muestra 0 del nuevo frame), samp_cnt TODAVÍA vale NFFT-1 del
    // frame anterior. El reset a 0 ocurre AL FINAL del ciclo de start.
    //
    // Igualmente, wr_bank debe apuntar al banco NUEVO desde la muestra 0.
    // Si rotamos wr_bank con <= en el ciclo de start, en ese mismo ciclo
    // wr_bank todavía es el viejo (non-blocking). El nuevo valor está
    // disponible recién en el ciclo siguiente.
    //
    // Solución: calcular combinacionalmente el banco y muestra efectivos
    // para las direcciones RAM. Estos son los valores "post-update":
    //   - En i_start: eff_samp = 0, eff_wr_bank = banco rotado
    //   - En !i_start: eff_samp = samp_cnt, eff_wr_bank = wr_bank
    // ============================================================
    wire [SAMP_W-1:0]  eff_samp    = (i_valid && i_start) ? {SAMP_W{1'b0}} : samp_cnt;

    wire [BANK_W-1:0]  next_wr_bank = (wr_bank == (DEPTH-1)) ? {BANK_W{1'b0}}
                                                               : (wr_bank + 1'b1);
    wire [BANK_W-1:0]  eff_wr_bank  = (i_valid && i_start) ? next_wr_bank : wr_bank;

    // rd_bank se calcula sobre eff_wr_bank para mantener la diferencia K
    wire [BANK_W-1:0]  eff_rd_bank  = (eff_wr_bank >= K[BANK_W-1:0])
                                      ? (eff_wr_bank - K[BANK_W-1:0])
                                      : (eff_wr_bank + DEPTH[BANK_W-1:0] - K[BANK_W-1:0]);

    // Dirección de acceso a la RAM
    wire [$clog2(DEPTH*NFFT)-1:0] wr_addr = eff_wr_bank * NFFT + eff_samp;
    wire [$clog2(DEPTH*NFFT)-1:0] rd_addr = eff_rd_bank * NFFT + eff_samp;

    // ============================================================
    // Lógica de escritura + lectura sincronizada
    // ============================================================
    always @(posedge clk) begin
        if (rst) begin
            wr_bank  <= {BANK_W{1'b0}};
            samp_cnt <= {SAMP_W{1'b0}};
            o_valid      <= 1'b0;
            o_start      <= 1'b0;
            o_X_curr_re  <= {NB_W{1'b0}};
            o_X_curr_im  <= {NB_W{1'b0}};
            o_X_old_re   <= {NB_W{1'b0}};
            o_X_old_im   <= {NB_W{1'b0}};
        end else begin

            // Defaults: sin salida válida
            o_valid <= 1'b0;
            o_start <= 1'b0;

            if (i_valid) begin
                // DIAGNOSTIC: print eff_samp on start to verify correct file loaded
                if (i_start)
                    $display("[HB_HW] i_start wr_bank=%0d eff_wr_bank=%0d eff_samp=%0d samp_cnt=%0d",
                              wr_bank, eff_wr_bank, eff_samp, samp_cnt);
                // -------------------------------------------------
                // A) Escritura del frame actual en wr_bank
                // -------------------------------------------------
                ram_re[wr_addr] <= i_xI;
                ram_im[wr_addr] <= i_xQ;

                // -------------------------------------------------
                // B) Lectura del frame pasado (rd_bank)
                //    Latencia 1 ciclo de registro -> o_X_old sale
                //    en el ciclo siguiente, igual que o_X_curr.
                // -------------------------------------------------
                o_X_old_re  <= ram_re[rd_addr];
                o_X_old_im  <= ram_im[rd_addr];

                // -------------------------------------------------
                // C) Paso del dato actual (registrado = mismo delay)
                // -------------------------------------------------
                o_X_curr_re <= i_xI;
                o_X_curr_im <= i_xQ;

                // -------------------------------------------------
                // D) Valid y start retrasados 1 ciclo
                // -------------------------------------------------
                o_valid <= 1'b1;
                o_start <= i_start;

                // -------------------------------------------------
                // E) Avance de registros de índice
                // wr_bank y samp_cnt se sincronizan con eff_wr_bank/eff_samp
                // para que en el PRÓXIMO ciclo los registros ya reflejen
                // el estado correcto.
                // -------------------------------------------------
                wr_bank  <= eff_wr_bank;
                samp_cnt <= (eff_samp == (NFFT-1)) ? {SAMP_W{1'b0}}
                                                   : (eff_samp + 1'b1);
            end
        end
    end

    // Diagnóstico
    assign o_wr_bank  = eff_wr_bank;
    assign o_samp_idx = eff_samp;

endmodule

`default_nettype wire