`timescale 1ns/1ps
`default_nettype none

// ============================================================
// tabla_w.v
//
// Banco de registros Q(17,10) de NFFT=32 posiciones complejas.
// Almacena W[k] = FFT([w|0..0]) — coeficientes en frecuencia.
//
// Puerto escritura — fft_pesos (una vez cada ~5 frames):
//   i_wr_valid/start → wr_cnt sube 0..31
//   Escribe W_new[k] en cada ciclo válido.
//
// Puerto lectura — cmul (cada frame de señal):
//   i_rd_valid/start → rd_cnt sube 0..31
//   Lectura COMBINACIONAL (assign): o_W_re/im en el mismo ciclo.
//   Latencia 0 ciclos adicionales.
//
// Sin ping-pong:
//   cmul termina de leer en 32 ciclos.
//   fft_pesos empieza a escribir ~160 ciclos después.
//   Sin colisión → banco simple.
//
// Inicialización a cero: W[k]=0 → CMUL produce Y=0 hasta
// que llega el primer frame del LMS. Convergencia rápida
// por el error inicial grande.
//
// Vivado: 32×17×2 = 1088 bits → LUTRAM (no BRAM).
// ============================================================

module tabla_w #(
    parameter integer NB_W  = 17,
    parameter integer NBF_W = 10,
    parameter integer NFFT  = 32
)(
    input  wire                    clk,
    input  wire                    rst,

    // Puerto escritura: desde fft_pesos
    input  wire                    i_wr_valid,
    input  wire                    i_wr_start,
    input  wire signed [NB_W-1:0]  i_W_re,
    input  wire signed [NB_W-1:0]  i_W_im,

    // Puerto lectura: sincronizado con hb_out_valid/start
    input  wire                    i_rd_valid,
    input  wire                    i_rd_start,

    // Lectura combinacional
    output wire signed [NB_W-1:0]  o_W_re,
    output wire signed [NB_W-1:0]  o_W_im
);

    localparam integer KW  = $clog2(NFFT);
    localparam [KW-1:0] N1 = NFFT - 1;

    // Banco de registros
    reg signed [NB_W-1:0] w_re [0:NFFT-1];
    reg signed [NB_W-1:0] w_im [0:NFFT-1];

    integer ii;
    initial begin
        for (ii = 0; ii < NFFT; ii = ii + 1) begin
            w_re[ii] = {NB_W{1'b0}};
            w_im[ii] = {NB_W{1'b0}};
        end
    end

    // Contador escritura
    reg [KW-1:0] wr_cnt;
    wire [KW-1:0] eff_wr = (i_wr_valid && i_wr_start) ? {KW{1'b0}} : wr_cnt;

    always @(posedge clk) begin
        if (rst) begin
            wr_cnt <= {KW{1'b0}};
        end else if (i_wr_valid) begin
            w_re[eff_wr] <= i_W_re;
            w_im[eff_wr] <= i_W_im;
            wr_cnt <= (eff_wr == N1) ? {KW{1'b0}} : (eff_wr + 1'b1);
        end
    end

    // Contador lectura
    reg [KW-1:0] rd_cnt;
    wire [KW-1:0] eff_rd = (i_rd_valid && i_rd_start) ? {KW{1'b0}} : rd_cnt;

    always @(posedge clk) begin
        if (rst) begin
            rd_cnt <= {KW{1'b0}};
        end else if (i_rd_valid) begin
            rd_cnt <= (eff_rd == N1) ? {KW{1'b0}} : (eff_rd + 1'b1);
        end
    end

    // Lectura combinacional
    assign o_W_re = w_re[eff_rd];
    assign o_W_im = w_im[eff_rd];

endmodule

`default_nettype wire
