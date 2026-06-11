`timescale 1us/1ns
`default_nettype none

// ============================================================
// block_power.v  —  Estimación de potencia de bloque para NLMS
//
//   P_frame = Σ_k ( Xre[k]² + Xim[k]² )  sobre NFFT bins
//   P_ema   = EMA(P_frame)  con factor 1/2^BETA   (suaviza varianza)
//
//   Salida: o_mu_extra = clamp( msb(P_ema) - P_REF_MSB, 0, MU_EXTRA_MAX )
//
//   Ese valor se SUMA al shift base del LMS:
//       mu_eff = mu_base / 2^o_mu_extra
//   → replica el  mu/(pwr+eps)  del Python (NLMS) en potencia de 2.
//
//   Como la potencia del canal es ~estacionaria, o_mu_extra es
//   cuasi-estático: NO requiere alineación fina con el gradiente.
//   El update_lms lo latchea por frame.
//
//   CALIBRACIÓN (único parámetro a ajustar):
//     Correr una vez, leer [BPOW] en el log → anotar 'msb'.
//     Poner P_REF_MSB = ese msb del canal/escenario de referencia
//     en el que el shift base solo (sin extra) ya andaba bien.
//     A mayor potencia que la de referencia → extra>0 → paso menor.
// ============================================================
module block_power #(
    parameter integer NB_W         = 17,   // ancho de X (Q17.10)
    parameter integer NFFT         = 32,
    parameter integer ACC_W        = 48,   // ancho del acumulador Σ|X|²
    parameter integer BETA         = 4,    // EMA: factor 1/2^BETA (~16 frames)
    parameter integer P_REF_MSB    = 30,   // CALIBRAR con el log [BPOW]
    parameter integer MU_EXTRA_MAX = 8     // tope del shift extra
)(
    input  wire                    clk,
    input  wire                    rst,
    input  wire                    i_valid,
    input  wire                    i_start,
    input  wire signed [NB_W-1:0]  i_xre,
    input  wire signed [NB_W-1:0]  i_xim,
    output reg  [4:0]              o_mu_extra
);
    localparam integer KW = $clog2(NFFT);

    // |X[k]|² = xre² + xim²
    wire signed [2*NB_W-1:0] xre2 = i_xre * i_xre;
    wire signed [2*NB_W-1:0] xim2 = i_xim * i_xim;
    wire        [2*NB_W:0]   mag2 = xre2[2*NB_W-1:0] + xim2[2*NB_W-1:0];

    // Acumulador de frame y contador de muestra
    reg  [ACC_W-1:0] acc;
    reg  [KW-1:0]    samp;
    wire [KW-1:0]    eff_samp = (i_valid && i_start) ? {KW{1'b0}} : samp;

    // Suma total del frame (acc previo + muestra actual)
    wire [ACC_W-1:0] frame_sum = acc + mag2;

    // EMA de la potencia
    reg  [ACC_W-1:0] p_ema;

    // MSB de p_ema → floor(log2)
    integer b;
    reg [5:0] msb_pos;
    always @(*) begin
        msb_pos = 6'd0;
        for (b = 0; b < ACC_W; b = b + 1)
            if (p_ema[b]) msb_pos = b[5:0];
    end

    // extra = clamp(msb - P_REF_MSB, 0, MU_EXTRA_MAX)
    wire signed [7:0] diff = $signed({2'b00, msb_pos}) - $signed(P_REF_MSB[7:0]);
    wire [5:0] extra_raw =
        (diff <= 0)              ? 6'd0 :
        (diff > MU_EXTRA_MAX)    ? MU_EXTRA_MAX[5:0] :
                                   diff[5:0];

    always @(posedge clk) begin
        if (rst) begin
            acc        <= {ACC_W{1'b0}};
            samp       <= {KW{1'b0}};
            p_ema      <= {ACC_W{1'b0}};
            o_mu_extra <= 5'd0;
        end else if (i_valid) begin
            // acumular |X|² en el frame
            acc <= i_start ? mag2 : frame_sum;

            // al cerrar el frame: actualizar EMA y recalcular extra
            if (eff_samp == NFFT-1) begin
                // FORMA ESTÁNDAR UNSIGNED (Sin $signed)
                p_ema      <= p_ema - (p_ema >> BETA) + (frame_sum >> BETA);
                
                o_mu_extra <= extra_raw[4:0];
                $display("[BPOW] frame_sum=%0d  p_ema=%0d  msb=%0d  mu_extra=%0d",
                         frame_sum, p_ema, msb_pos, extra_raw);
            end

            samp <= (eff_samp == NFFT-1) ? {KW{1'b0}} : (eff_samp + 1'b1);
        end
    end
endmodule

`default_nettype wire