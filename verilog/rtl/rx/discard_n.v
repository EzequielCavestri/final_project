`timescale 1ns/1ps
`default_nettype none

// ============================================================
// discard_n
//
// Propósito:
//   Implementa el paso "DISCARD N" del overlap-save.
//
//   La IFFT produce frames de 2N=NFFT muestras. Las primeras N
//   muestras (índices 0..N-1) son el transitorio del overlap y
//   deben descartarse. Solo las N muestras finales (N..2N-1)
//   son la salida útil del filtro: y_blk[n].
//
// Operación:
//   - Cuenta muestras dentro del frame usando i_start/i_valid.
//   - Genera o_valid=1 solo para muestras con índice >= N (NFFT/2).
//   - o_start=1 en la primera muestra válida de cada frame (índice N).
//   - Latencia: 0 ciclos (lógica combinacional + registro de control).
//
// Parámetros:
//   NB_W  = ancho de dato (igual que salida IFFT, WN=9 bits)
//   NFFT  = tamaño total del frame FFT (2N, default 32)
//   -> N = NFFT/2 = 16 muestras útiles por frame
//
// Punto fijo:
//   Datos en Q(NB_W, NBF_W). No se modifica el valor,
//   solo se filtra el flujo de muestras.
// ============================================================

module discard_n #(
    parameter integer NB_W  = 9,     // ancho de dato (igual que IFFT out)
    parameter integer NBF_W = 7,     // bits fraccionarios (Q9.7)
    parameter integer NFFT  = 32     // tamaño total frame (2N)
)(
    input  wire                    clk,
    input  wire                    rst,

    // --- Entrada: streaming de la IFFT ---
    input  wire                    i_valid,
    input  wire                    i_start,   // alto en muestra 0 de cada frame
    input  wire signed [NB_W-1:0]  i_yI,
    input  wire signed [NB_W-1:0]  i_yQ,

    // --- Salida: solo la 2da mitad del frame (muestras N..2N-1) ---
    output reg                     o_valid,   // 1 solo para muestras N..2N-1
    output reg                     o_start,   // 1 en la muestra N (primera útil)
    output reg  signed [NB_W-1:0]  o_yI,
    output reg  signed [NB_W-1:0]  o_yQ,

    // --- Diagnóstico ---
    output wire [$clog2(NFFT)-1:0] o_samp_idx  // índice actual dentro del frame
);

    // ============================================================
    // Parámetros derivados
    // ============================================================
    localparam integer N    = NFFT / 2;       // muestras a descartar (primera mitad)
    localparam integer KW   = $clog2(NFFT);

    // Wire para N en el ancho correcto del contador (evita bit-select variable
    // sobre localparam integer, que es ilegal en Verilog-2001 y falla en Vivado)
    localparam [KW-1:0] N_W = N[KW-1:0];

    // ============================================================
    // Contador de muestra dentro del frame (0..NFFT-1)
    //   Igual que en cmul_pbfdaf: eff_samp resuelve el bin 0
    //   en el mismo ciclo que llega i_start.
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
    // Lógica de descarte:
    //   - Muestra válida solo si eff_samp >= N
    //   - o_start = 1 en la primera muestra válida (eff_samp == N)
    // ============================================================
    wire samp_valid = (eff_samp >= N_W);
    wire samp_start = (eff_samp == N_W);

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
