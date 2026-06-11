`timescale 1us/1ns
`default_nettype none

module sat_trunc #(
    parameter integer NB_XI   = 17,
    parameter integer NBF_XI  = 10,
    parameter integer NB_XO   = 9,
    parameter integer NBF_XO  = 7,
    parameter integer ROUND_EVEN = 1 
)(
    input  wire signed [NB_XI-1:0] i_data,
    output wire signed [NB_XO-1:0] o_data
);

    // --- Parámetros Internos ---
    localparam integer NBI_XO  = NB_XO - NBF_XO;
    localparam integer K_DROP  = (NBF_XI > NBF_XO) ? (NBF_XI - NBF_XO) : 0;
    localparam integer K_ADD   = (NBF_XI < NBF_XO) ? (NBF_XO - NBF_XI) : 0;
    localparam integer NBI_ADJ = NB_XI - NBF_XO;

    // --- Declaración de Cables ---
    wire signed [NB_XI-1:0] y_shift;
    wire guard;
    wire sticky;
    wire [NB_XI-1:0] sticky_mask;
    wire inc_even;
    wire signed [NB_XI-1:0] y_round;
    wire signed [NB_XI-1:0] data_adj;
    wire signed [NB_XO-1:0] sat_max;
    wire signed [NB_XO-1:0] sat_min;
    wire overflow_ok;
    wire [NB_XO-1:0] packed_val;

    // --- 1. Alineación de punto fijo y Redondeo ---
    assign y_shift = (K_DROP > 0) ? (i_data >>> K_DROP) : i_data;
    assign guard   = (K_DROP > 0) ? i_data[K_DROP-1] : 1'b0;
    
    // Máscara para el sticky bit (evita errores de rango negativo)
    assign sticky_mask = (K_DROP > 1) ? ((1 << (K_DROP-1)) - 1) : {NB_XI{1'b0}};
    assign sticky  = |(i_data & sticky_mask);
    
    // Lógica Round-to-even
    assign inc_even = guard & (sticky | y_shift[0]);
    assign y_round = ((ROUND_EVEN != 0) && (K_DROP > 0)) ? (y_shift + $signed({1'b0, inc_even})) : y_shift;

    // Ajuste si el destino tiene más bits de fracción
    assign data_adj = (K_ADD > 0) ? (y_round <<< K_ADD) : y_round;

    // --- 2. Definición de Límites de Saturación ---
    assign sat_max = {1'b0, {(NB_XO-1){1'b1}}};
    assign sat_min = {1'b1, {(NB_XO-1){1'b0}}};

    // --- 3. Detección de Overflow ---
    assign overflow_ok = (NBI_ADJ <= NBI_XO) ? 1'b1 :
                         (data_adj[NB_XI-1 : NBF_XO + NBI_XO - 1] == {(NBI_ADJ-NBI_XO+1){data_adj[NB_XI-1]}});

    //assign packed_val = { data_adj[NBF_XO + NBI_XO - 1], data_adj[NBF_XO +: (NB_XO-1)] };
    assign packed_val = data_adj[NB_XO-1:0];

    // --- 4. Selección de Salida ---
    assign o_data = (overflow_ok == 1'b1) ? $signed(packed_val) : 
                    (data_adj[NB_XI-1] == 1'b1) ? sat_min : sat_max;

endmodule

`default_nettype wire