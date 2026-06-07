`timescale 1ns/1ps

module filtro_fir #(
    // --------------------------
    // Parámetros principales
    // --------------------------
    parameter  H  = 13,   // taps
    parameter  W  = 9,   // ancho datos din/dout
    parameter  CW = 9,    // ancho coeficientes

    // --------------------------
    // Punto fijo explícito
    // --------------------------
    parameter  DATA_F = 7, // fraccionales de din/dout
    parameter  COEF_F = 7, // fraccionales del coef

    // --------------------------
    // Opciones de salida
    // --------------------------
    parameter SATURATE_EN = 1,   // 1=con saturación, 0=truncamiento
    parameter ROUND_EN    = 0,   // 1=redondeo simple antes del shift

    // --------------------------
    // Coeficientes empaquetados
    // COEFFS_VECTOR = {c0, c1, ..., c(H-1)}
    // taps_coeffs[0] = c0
    // --------------------------
    parameter [H*CW-1:0] COEFFS_VECTOR = {
        // Default: delta en tap 0 (1.0 en Q7 -> 128)
        9'sd128,
        { (H-1){ 9'sd0 } }
    }
)(
    input  wire                 clk,
    input  wire                 rst,
    input  wire signed [W-1:0]  din,
    output reg  signed [W-1:0]  dout
);

    // ================================================================
    // Función clog2 (para GUARD bits) - Verilog
    // ================================================================
    function integer clog2;
        input integer value;
        integer v;
        integer i;
        begin
            v = value - 1;
            i = 0;
            while (v > 0) begin
                v = v >> 1;
                i = i + 1;
            end
            clog2 = i;
        end
    endfunction

    // ================================================================
    // 1) DESEMPAQUETAR COEFICIENTES
    // taps_coeffs[0] toma el primer coeficiente del {...}
    // ================================================================
    wire signed [CW-1:0] taps_coeffs [0:H-1];

    genvar k;
    generate
        for (k = 0; k < H; k = k + 1) begin : unpack_coeffs
            // COEFFS_VECTOR = {c0, c1, ..., c(H-1)}
            // slice: taps_coeffs[k] = c[k]
            assign taps_coeffs[k] = COEFFS_VECTOR[(H-k)*CW - 1 : (H-k-1)*CW];
        end
    endgenerate

    // ================================================================
    // 2) LINEA DE RETARDO
    // ================================================================
    integer i;
    reg signed [W-1:0] shift_reg [0:H-1];

    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < H; i = i + 1)
                shift_reg[i] <= {W{1'b0}};
        end else begin
            shift_reg[0] <= din;
            for (i = 1; i < H; i = i + 1)
                shift_reg[i] <= shift_reg[i-1];
        end
    end

    // ================================================================
    // 3) MULTIPLICACIÓN
    // prod: S(W+CW, DATA_F+COEF_F)
    // ================================================================
    reg signed [W+CW-1:0] mult_res [0:H-1];

    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < H; i = i + 1)
                mult_res[i] <= {(W+CW){1'b0}};
        end else begin
            for (i = 0; i < H; i = i + 1)
                mult_res[i] <= shift_reg[i] * taps_coeffs[i];
        end
    end

    // ================================================================
    // 4) ACUMULADOR (con guard bits ~ log2(H))
    // ================================================================
    localparam integer GUARD = (H <= 1) ? 0 : clog2(H);
    localparam integer ACC_W = (W + CW) + GUARD + 1;

    reg signed [ACC_W-1:0] sum_temp;
    integer j;

    always @(*) begin
        sum_temp = {ACC_W{1'b0}};
        for (j = 0; j < H; j = j + 1) begin
            // Extensión de signo de mult_res[j] hasta ACC_W
            sum_temp = sum_temp + {{(ACC_W-(W+CW)){mult_res[j][W+CW-1]}}, mult_res[j]};
        end
    end

    // ================================================================
    // 5) ESCALADO (volver de Q(DATA_F+COEF_F) a Q(DATA_F))
    // shift por COEF_F. Redondeo simple opcional.
    // ================================================================
    reg  signed [ACC_W-1:0] sum_round;
    reg  signed [ACC_W-1:0] off;
    wire signed [ACC_W-1:0] sum_scaled;

    always @(*) begin
        // default
        sum_round = sum_temp;
        off       = {ACC_W{1'b0}};

        if (ROUND_EN && (COEF_F > 0)) begin
            off = ({{(ACC_W-1){1'b0}}, 1'b1} <<< (COEF_F-1));
            if (sum_temp >= 0)
                sum_round = sum_temp + off;
            else
                sum_round = sum_temp - off;
        end
    end

    assign sum_scaled = (COEF_F > 0) ? (sum_round >>> COEF_F) : sum_round;

    // ================================================================
    // 6) SATURACIÓN a W bits (opcional)
    // ================================================================
    localparam signed [W-1:0] MAX_POS = {1'b0, {(W-1){1'b1}}};
    localparam signed [W-1:0] MAX_NEG = {1'b1, {(W-1){1'b0}}};

    function [W-1:0] sat_to_W;
        input [ACC_W-1:0] x;
        begin
            // Overflow si los bits altos no son extensión del signo de x[W-1]
            if (| (x[ACC_W-1:W] ^ { (ACC_W-W){x[W-1]} })) begin
                sat_to_W = x[ACC_W-1] ? MAX_NEG : MAX_POS;
            end else begin
                sat_to_W = x[W-1:0];
            end
        end
    endfunction

    always @(posedge clk) begin
        if (rst) begin
            dout <= {W{1'b0}};
        end else begin
            if (SATURATE_EN)
                dout <= sat_to_W(sum_scaled);
            else
                dout <= sum_scaled[W-1:0]; // truncamiento
        end
    end

endmodule
