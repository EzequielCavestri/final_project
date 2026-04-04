`timescale 1ns/1ps
`default_nettype none

// ============================================================
// slicer_qpsk
//
// Propósito:
//   Implementa el SLICER y el cálculo de ERROR del PBFDAF.
//
// Operación (por muestra, mismo ciclo que i_valid):
//
//   1) SLICER: decisión dura QPSK
//      yhat_I = (y_blk_I >= 0) ? +QPSK_A : -QPSK_A
//      yhat_Q = (y_blk_Q >= 0) ? +QPSK_A : -QPSK_A
//
//   2) ERROR:
//      e_I = yhat_I - y_blk_I
//      e_Q = yhat_Q - y_blk_Q
//
// Punto fijo:
//   Entrada  y_blk : Q(NB_W=9, NBF_W=7)  -> WN=9 bits, misma escala que IFFT
//   QPSK_A         : 1/sqrt(2) en Q(9,7) = 91 (integer)
//                    -> 91/128 = 0.7109 ~ 1/sqrt(2) ✓
//
//   Rango del error:
//     yhat ∈ {-91, +91}  (integer Q9.7)
//     y    ∈ [-128, 127] (integer Q9.7, 9 bits signed)
//     e    ∈ [-91-127, 91+127] = [-218, 218]
//     218 < 256 = 2^8  -> e cabe en NB_W=9 bits signed ✓
//     (con saturación por si acaso)
//
// Latencia: 1 ciclo (registro de salida).
//
// Salidas:
//   o_yhat_I/Q : símbolo decidido (±QPSK_A), para referencia/debug
//   o_e_I/Q    : error  e = yhat - y, entrada al bloque FFT_ERROR
//   o_valid    : alineado con las salidas (= i_valid retrasado 1 ciclo)
//   o_start    : alineado con o_valid
// ============================================================

module slicer_qpsk #(
    parameter integer NB_W  = 9,    // ancho de dato Q(NB_W, NBF_W)
    parameter integer NBF_W = 7,    // bits fraccionarios
    parameter integer NFFT  = 32    // muestras por frame (para o_samp_idx)
)(
    input  wire                    clk,
    input  wire                    rst,

    // --- Entrada: y_blk del discard_n ---
    input  wire                    i_valid,
    input  wire                    i_start,
    input  wire signed [NB_W-1:0]  i_yI,   // y_blk Re
    input  wire signed [NB_W-1:0]  i_yQ,   // y_blk Im

    // --- Salida: símbolo decidido ---
    output reg                     o_valid,
    output reg                     o_start,
    output reg  signed [NB_W-1:0]  o_yhat_I,  // yhat Re = ±QPSK_A
    output reg  signed [NB_W-1:0]  o_yhat_Q,  // yhat Im = ±QPSK_A

    // --- Salida: error e = yhat - y ---
    output reg  signed [NB_W-1:0]  o_e_I,     // error Re
    output reg  signed [NB_W-1:0]  o_e_Q      // error Im
);

    // ============================================================
    // QPSK_A en Q(NB_W=9, NBF_W=7):
    //   1/sqrt(2) * 2^7 = 0.70711 * 128 = 90.51 -> round_even -> 91
    //   91 / 128 = 0.7109 (error < 0.5 LSB)
    // ============================================================
    localparam signed [NB_W-1:0] QPSK_A     =  9'sd91;
    localparam signed [NB_W-1:0] QPSK_A_NEG = -9'sd91;

    // ============================================================
    // Saturación del error en NB_W bits
    //   e = yhat - y, máximo posible = 91 + 127 = 218
    //   218 < 256 = 2^(NB_W-1) -> no satura nunca en Q9.7
    //   Se incluye sat_trunc por seguridad y consistencia con el resto
    // ============================================================
    // Suma con 1 bit extra para capturar el signo de la resta
    wire signed [NB_W:0] e_I_ext;
    wire signed [NB_W:0] e_Q_ext;

    // Slicer combinacional
    wire signed [NB_W-1:0] yhat_I_comb = (i_yI >= 0) ? QPSK_A : QPSK_A_NEG;
    wire signed [NB_W-1:0] yhat_Q_comb = (i_yQ >= 0) ? QPSK_A : QPSK_A_NEG;

    // Error combinacional (extendido a NB_W+1 para no perder signo)
    assign e_I_ext = $signed({yhat_I_comb[NB_W-1], yhat_I_comb})
                - $signed({i_yI[NB_W-1],        i_yI});
    assign e_Q_ext = $signed({yhat_Q_comb[NB_W-1], yhat_Q_comb})
                - $signed({i_yQ[NB_W-1],        i_yQ});

    // Saturación de NB_W+1 a NB_W bits (sin cambio de punto fijo)
    wire signed [NB_W-1:0] e_I_sat;
    wire signed [NB_W-1:0] e_Q_sat;

    sat_trunc #(
        .NB_XI(NB_W+1), .NBF_XI(NBF_W),
        .NB_XO(NB_W),   .NBF_XO(NBF_W),
        .ROUND_EVEN(0)
    ) u_sat_eI (
        .i_data(e_I_ext),
        .o_data(e_I_sat)
    );

    sat_trunc #(
        .NB_XI(NB_W+1), .NBF_XI(NBF_W),
        .NB_XO(NB_W),   .NBF_XO(NBF_W),
        .ROUND_EVEN(0)
    ) u_sat_eQ (
        .i_data(e_Q_ext),
        .o_data(e_Q_sat)
    );

    // ============================================================
    // Registro de salida — 1 ciclo de latencia
    // ============================================================
    always @(posedge clk) begin
        if (rst) begin
            o_valid   <= 1'b0;
            o_start   <= 1'b0;
            o_yhat_I  <= {NB_W{1'b0}};
            o_yhat_Q  <= {NB_W{1'b0}};
            o_e_I     <= {NB_W{1'b0}};
            o_e_Q     <= {NB_W{1'b0}};
        end else begin
            o_valid  <= i_valid;
            o_start  <= i_valid && i_start;
            if (i_valid) begin
                o_yhat_I <= yhat_I_comb;
                o_yhat_Q <= yhat_Q_comb;
                o_e_I    <= e_I_sat;
                o_e_Q    <= e_Q_sat;
            end else begin
                o_yhat_I <= {NB_W{1'b0}};
                o_yhat_Q <= {NB_W{1'b0}};
                o_e_I    <= {NB_W{1'b0}};
                o_e_Q    <= {NB_W{1'b0}};
            end
        end
    end

endmodule

`default_nettype wire
