`timescale 1ns/1ps
`default_nettype none

// ============================================================
// xhist_delay
//
// Propósito:
//   Retarda X_hist (o_X_old del history buffer) exactamente
//   DELAY ciclos para sincronizarlo con E_k (ffte_out) antes
//   de entrar al módulo GRADIENTE.
//
//   Sin este retardo, cuando E_k del frame N llega al GRADIENTE,
//   X_hist del frame N ya pasó hace ~3.7 frames. Este módulo
//   mantiene X_hist "esperando" hasta que E_k esté listo.
//
// Implementación:
//   Shift register de DELAY posiciones. Cada ciclo de reloj
//   todos los elementos avanzan una posición. La salida es
//   siempre la entrada retrasada DELAY ciclos exactos.
//
//   Vivado sintetiza esto automáticamente como SRL (Shift
//   Register LUT) — no requiere BRAM ni IP externos.
//
// Timing:
//   Entrada en ciclo T  →  Salida en ciclo T + DELAY
//   Delay exacto = DELAY ciclos (fijo, constante)
//
//   Las señales i_valid e i_start se retrasan igual que los
//   datos — el GRADIENTE usa o_valid para saber cuándo
//   hay datos útiles disponibles.
//
// Parámetros:
//   NB_W  = ancho del dato (17 bits, Q17.10)
//   DELAY = ciclos de retardo (118, medido en simulación)
// ============================================================

module xhist_delay #(
    parameter integer NB_W  = 17,
    parameter integer DELAY = 118
)(
    input  wire                    clk,
    input  wire                    rst,

    // --- Entrada: X_hist del history buffer ---
    input  wire                    i_valid,  // hb_out_valid
    input  wire                    i_start,  // hb_out_start
    input  wire signed [NB_W-1:0]  i_xre,   // hb_out_X_old_re
    input  wire signed [NB_W-1:0]  i_xim,   // hb_out_X_old_im

    // --- Salida: X_hist retrasado DELAY ciclos ---
    output wire                    o_valid,  // sincronizado con ffte_out_valid
    output wire                    o_start,  // sincronizado con ffte_out_start
    output wire signed [NB_W-1:0]  o_xre,   // listo para GRADIENTE
    output wire signed [NB_W-1:0]  o_xim
);

    // ============================================================
    // Shift register — DELAY posiciones
    // Vivado infiere SRL automáticamente
    // ============================================================
    reg signed [NB_W-1:0] sr_re [0:DELAY];
    reg signed [NB_W-1:0] sr_im [0:DELAY];
    reg                   sr_v  [0:DELAY];
    reg                   sr_s  [0:DELAY];

    integer k;

    always @(posedge clk) begin
        if (rst) begin
            for (k=0; k<=DELAY; k=k+1) begin
                sr_re[k] <= {NB_W{1'b0}};
                sr_im[k] <= {NB_W{1'b0}};
                sr_v [k] <= 1'b0;
                sr_s [k] <= 1'b0;
            end
        end else begin
            sr_re[0] <= i_xre;
            sr_im[0] <= i_xim;
            sr_v [0] <= i_valid;
            sr_s [0] <= i_start;
            for (k=1; k<=DELAY; k=k+1) begin
                sr_re[k] <= sr_re[k-1];
                sr_im[k] <= sr_im[k-1];
                sr_v [k] <= sr_v [k-1];
                sr_s [k] <= sr_s [k-1];
            end
        end
    end

    assign o_xre   = sr_re[DELAY];
    assign o_xim   = sr_im[DELAY];
    assign o_valid = sr_v [DELAY];
    assign o_start = sr_s [DELAY];

endmodule

`default_nettype wire
