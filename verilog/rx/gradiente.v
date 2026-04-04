`timescale 1ns/1ps
`default_nettype none

// ============================================================
// gradiente.v
//
// Calcula el gradiente espectral del PBFDAF-LMS:
//
//   PHI_k = conj(X_hist_k) * E_k
//
// Siendo:
//   X_hist_k  = salida de xhist_delay (X_hist retrasado 118 ciclos)
//   E_k       = salida de fft_error   (espectro del error)
//
// Matemática compleja:
//   conj(Xr + jXi) * (Er + jEi) = (Xr - jXi)(Er + jEi)
//
//   PHI_re = Xr*Er + Xi*Ei
//   PHI_im = Xr*Ei - Xi*Er
//
// Aritmética: Q(NB_W, NBF_W) = Q(17,10)
//   Producto: 34 bits Q(34,20)
//   Truncar:  shift right NBF_W → Q(24,10)
//   Saturar:  17 bits con detección de overflow
//
// Pipeline 2 ciclos:
//   Ciclo 1: registrar las 4 multiplicaciones
//   Ciclo 2: sumar/restar + truncar + saturar → salida
//
// Control:
//   i_valid = ffte_out_valid  (ffte define el ritmo)
//   i_start = ffte_out_start  (primer sample del frame)
//   Cuando i_valid=1 los datos de xhd y ffte son válidos y sincronizados.
//
// Latencia: 2 ciclos desde i_valid hasta o_valid
// ============================================================

module gradiente #(
    parameter integer NB_W  = 17,
    parameter integer NBF_W = 10
)(
    input  wire                    clk,
    input  wire                    rst,

    // --- Control (usar señales de ffte_out) ---
    input  wire                    i_valid,  // ffte_out_valid
    input  wire                    i_start,  // ffte_out_start

    // --- X_hist del xhist_delay ---
    input  wire signed [NB_W-1:0]  i_xre,   // X_hist parte real
    input  wire signed [NB_W-1:0]  i_xim,   // X_hist parte imaginaria

    // --- E_k de la FFT_ERROR ---
    input  wire signed [NB_W-1:0]  i_ere,   // E_k parte real
    input  wire signed [NB_W-1:0]  i_eim,   // E_k parte imaginaria

    // --- PHI_k = conj(X_hist) * E_k ---
    output reg                     o_valid,  // válido 2 ciclos después
    output reg                     o_start,  // inicio de frame
    output reg  signed [NB_W-1:0]  o_phi_re, // PHI_k parte real
    output reg  signed [NB_W-1:0]  o_phi_im  // PHI_k parte imaginaria
);

    // Ancho del producto antes de truncar
    localparam integer NB_PROD = 2 * NB_W;          // 34 bits
    localparam integer NB_TRUNC = NB_PROD - NBF_W;  // 24 bits

    // ============================================================
    // CICLO 1 — 4 multiplicaciones registradas
    // ============================================================
    reg signed [NB_PROD-1:0] p_rr;  // Xr * Er
    reg signed [NB_PROD-1:0] p_ii;  // Xi * Ei
    reg signed [NB_PROD-1:0] p_ri;  // Xr * Ei
    reg signed [NB_PROD-1:0] p_ir;  // Xi * Er

    // Retardo de valid y start — 1 registro, salida en ciclo 2
    reg valid_d1, start_d1;

    // ============================================================
    // CICLO 2 — sumas + truncado + saturación
    // ============================================================
    // Suma de productos (1 bit extra para no perder carry)
    wire signed [NB_TRUNC:0]   sum_re_w; // p_rr + p_ii truncado
    wire signed [NB_TRUNC:0]   sum_im_w; // p_ri - p_ir truncado

    assign sum_re_w = $signed(p_rr[NB_PROD-1:NBF_W]) + $signed(p_ii[NB_PROD-1:NBF_W]);
    assign sum_im_w = $signed(p_ri[NB_PROD-1:NBF_W]) - $signed(p_ir[NB_PROD-1:NBF_W]);

    // Detección de saturación
    // Si los bits de overflow (bits superiores a NB_W-1) no son todos
    // iguales al bit de signo, hay overflow → saturar
    wire ovf_re = (sum_re_w[NB_TRUNC:NB_W-1] != {(NB_TRUNC-NB_W+2){sum_re_w[NB_TRUNC]}});
    wire ovf_im = (sum_im_w[NB_TRUNC:NB_W-1] != {(NB_TRUNC-NB_W+2){sum_im_w[NB_TRUNC]}});

    wire signed [NB_W-1:0] sat_re = ovf_re ?
        (sum_re_w[NB_TRUNC] ? {1'b1, {(NB_W-1){1'b0}}} : {1'b0, {(NB_W-1){1'b1}}}) :
        sum_re_w[NB_W-1:0];

    wire signed [NB_W-1:0] sat_im = ovf_im ?
        (sum_im_w[NB_TRUNC] ? {1'b1, {(NB_W-1){1'b0}}} : {1'b0, {(NB_W-1){1'b1}}}) :
        sum_im_w[NB_W-1:0];

    // ============================================================
    // Registros
    // ============================================================
    always @(posedge clk) begin
        if (rst) begin
            p_rr     <= {NB_PROD{1'b0}};
            p_ii     <= {NB_PROD{1'b0}};
            p_ri     <= {NB_PROD{1'b0}};
            p_ir     <= {NB_PROD{1'b0}};
            valid_d1 <= 1'b0;
            start_d1 <= 1'b0;
            o_valid  <= 1'b0;
            o_start  <= 1'b0;
            o_phi_re <= {NB_W{1'b0}};
            o_phi_im <= {NB_W{1'b0}};
        end else begin

            // ---- CICLO 1: 4 multiplicaciones registradas ----
            if (i_valid) begin
                p_rr <= i_xre * i_ere;
                p_ii <= i_xim * i_eim;
                p_ri <= i_xre * i_eim;
                p_ir <= i_xim * i_ere;
            end
            valid_d1 <= i_valid;
            start_d1 <= i_start;

            // ---- CICLO 2: sumas combinacionales + saturación + salida ----
            // sat_re y sat_im son wires combinacionales desde p_rr/p_ii
            // que ya están registrados → timing limpio
            if (valid_d1) begin
                o_phi_re <= sat_re;
                o_phi_im <= sat_im;
            end
            o_valid <= valid_d1;   // exactamente 2 ciclos desde i_valid
            o_start <= start_d1;   // exactamente 2 ciclos desde i_start

        end
    end

endmodule

`default_nettype wire
