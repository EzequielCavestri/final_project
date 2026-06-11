`timescale 1us/1ns
`default_nettype none

// ============================================================
// keep_first_n.v  —  PROYECCION del gradiente PBFDAF-LMS
//
// Propósito:
//   Implementa la proyección del gradiente temporal phi(t)
//   sobre el subespacio causal de un filtro de N taps.
//
//   La IFFT del gradiente produce 2N muestras (phi[0..2N-1]).
//   Solo las primeras N (índices 0..N-1) son válidas para
//   actualizar un filtro FIR causal de longitud N.
//   Las últimas N muestras (N..2N-1) se descartan.
//
//   DIFERENCIA clave con discard_n:
//     discard_n  → guarda muestras [N .. 2N-1]  (segunda mitad)
//     keep_first_n → guarda muestras [0 .. N-1]  (primera mitad)  ← este módulo
//
// Operación:
//   - Cuenta muestras dentro del frame con i_start/i_valid.
//   - Genera o_valid=1 solo para muestras con índice < N (NFFT/2).
//   - o_start=1 en la primera muestra válida (índice 0).
//   - Latencia: 1 ciclo (registro de salida).
//
// Parámetros:
//   NB_W  = ancho de dato
//   NBF_W = bits fraccionarios
//   NFFT  = tamaño total del frame (2N, default 32)
//   → N = NFFT/2 = 16 muestras útiles por frame
// ============================================================

module keep_first_n #(
    parameter integer NB_W  = 17,
    parameter integer NBF_W = 10,
    parameter integer NFFT  = 32
)(
    input  wire                    clk,
    input  wire                    rst,

    // --- Entrada: streaming del IFFT_GRAD ---
    input  wire                    i_valid,
    input  wire                    i_start,   // alto en muestra 0 de cada frame
    input  wire signed [NB_W-1:0]  i_yI,
    input  wire signed [NB_W-1:0]  i_yQ,

    // --- Salida: solo la 1ra mitad del frame (muestras 0..N-1) ---
    output reg                     o_valid,   // 1 solo para muestras 0..N-1
    output reg                     o_start,   // 1 en la muestra 0 (primera útil)
    output reg  signed [NB_W-1:0]  o_yI,
    output reg  signed [NB_W-1:0]  o_yQ,

    // --- Diagnóstico ---
    output wire [$clog2(NFFT)-1:0] o_samp_idx
);

    // ============================================================
    // Parámetros derivados
    // ============================================================
    localparam integer N  = NFFT / 2;
    localparam integer KW = $clog2(NFFT);

    // N en ancho de contador (evita bit-select sobre integer, ilegal en Vivado)
    localparam [KW-1:0] N_W = N[KW-1:0];

    // ============================================================
    // Contador de muestra dentro del frame (0..NFFT-1)
    // eff_samp = 0 en el ciclo exacto en que llega i_start
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
    // Lógica de selección:
    //   - Muestra válida solo si eff_samp < N  (primera mitad)
    //   - o_start = 1 en la muestra 0 (eff_samp == 0)
    // ============================================================
    wire samp_valid = (eff_samp < N_W);
    wire samp_start = (eff_samp == {KW{1'b0}});  // índice 0

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
            o_valid <= i_valid && samp_valid;
            o_start <= i_valid && samp_start;
            o_yI    <= (i_valid && samp_valid) ? i_yI : {NB_W{1'b0}};
            o_yQ    <= (i_valid && samp_valid) ? i_yQ : {NB_W{1'b0}};
        end
    end

endmodule

`default_nettype wire
