`timescale 1ns/1ps
`default_nettype none

// ============================================================
// sat_trunc (combinacional)
// - Alinea punto fijo (ajuste de fracción)
// - Round-to-even opcional
// - Saturación a NB_XO
//
// FIX importante para Vivado + `default_nettype none:
//   declarar explícitamente wire en puertos.
// ============================================================

module sat_trunc #(
    parameter integer NB_XI   = 17,
    parameter integer NBF_XI  = 10,

    parameter integer NB_XO   = 9,
    parameter integer NBF_XO  = 7,

    parameter integer ROUND_EVEN = 1   // 1: round-to-even, 0: trunc
)(
    input  wire signed [NB_XI-1:0] i_data,
    output wire signed [NB_XO-1:0] o_data
);

    localparam integer NBI_XO  = NB_XO - NBF_XO;

    localparam integer K_DROP  = (NBF_XI > NBF_XO) ? (NBF_XI - NBF_XO) : 0;
    localparam integer K_ADD   = (NBF_XI < NBF_XO) ? (NBF_XO - NBF_XI) : 0;

    wire signed [NB_XI-1:0] y_shift = (K_DROP > 0) ? (i_data >>> K_DROP) : i_data;

    wire guard  = (K_DROP > 0) ? i_data[K_DROP-1] : 1'b0;
    wire sticky = (K_DROP > 1) ? (|i_data[K_DROP-2:0]) : 1'b0;

    wire inc_even = guard & (sticky | y_shift[0]);

    wire signed [NB_XI-1:0] y_round =
        ((ROUND_EVEN != 0) && (K_DROP > 0)) ? (y_shift + (inc_even ? 1 : 0)) : y_shift;

    wire signed [NB_XI-1:0] data_adj = (K_ADD > 0) ? (y_round <<< K_ADD) : y_round;

    wire [NBF_XO-1:0] frac_out = data_adj[NBF_XO-1:0];

    localparam integer NBI_ADJ = NB_XI - NBF_XO;

    wire signed [NB_XO-1:0] sat_max = {1'b0, {(NB_XO-1){1'b1}}};
    wire signed [NB_XO-1:0] sat_min = {1'b1, {(NB_XO-1){1'b0}}};

    wire overflow_ok =
        (NBI_ADJ > NBI_XO) ?
            (data_adj[(NB_XI-1) -: (NBI_ADJ-NBI_XO+1)] == {(NBI_ADJ-NBI_XO+1){data_adj[NB_XI-1]}}) :
            1'b1;

    wire signed [NB_XO-1:0] packed = { data_adj[NBF_XO +: NBI_XO], frac_out };

    assign o_data = overflow_ok ? packed : (data_adj[NB_XI-1] ? sat_min : sat_max);

endmodule

`default_nettype wire