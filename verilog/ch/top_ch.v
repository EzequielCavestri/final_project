`timescale 1ns/1ps

module top_ch #(
    // ================================================================
    // Datos y coeficientes (SIN integer)
    // ================================================================
    parameter DWIDTH      = 9,  
    parameter L_CH        = 13,  
    parameter CWIDTH      = 9,   
    parameter DATA_F      = 7,   
    parameter COEF_F      = 7,   

    // ================================================================
    // Ruido
    // ================================================================
    parameter NOISE_WIDTH = 16,
    parameter SNR_WIDTH   = 11,
    parameter SIGMA_F     = 10,  

    // ================================================================
    // DEFINICIÓN DEL CANAL
    // ================================================================
  
        // ================================================================
    // CANAL ISI ASIMETRICO — 5 taps, ratio frecuencial 124x
    // h[5]=50, h[6]=90, h[7]=70, h[8]=40, h[9]=20  (en Q7/x128)
    // BER sin ecualizar estimado ~12% a sigma_scale=8 (SNR~15dB)
    // Todos los taps dentro del CP (N=16) — sin IBI entre bloques OFDM
    // ================================================================
        // ================================================================
    // CANAL ISI 3 taps — tap principal h[6]=110, laterales h[5]=h[7]=45
    // ratio |H|max/|H|min = 10x — ISI moderada, manejable en Q17.10
    // BER sin ecualizar ~10% a sigma_scale=8 (SNR~15dB)
    // ================================================================
        // Canal delta puro: h = delta[n-6] (delay 6 ciclos en clk_low)
    // Para verificacion del lag del pipeline sin ISI
        // Canal ISI 3 taps: h[5]=45, h[6]=110 (tap principal), h[7]=45 en Q7
    // Ratio |H|max/|H|min = 10x — ISI moderada, manejable en Q17.10
    parameter [L_CH*CWIDTH-1:0] H_REAL_INIT = {
    9'sd0, 9'sd0, 9'sd0, 9'sd0, 9'sd0, 9'sd45, 9'sd110,
    9'sd45, 9'sd0, 9'sd0, 9'sd0, 9'sd0, 9'sd0
    },


    parameter [L_CH*CWIDTH-1:0] H_IMAG_INIT = {L_CH{9'sd0}}

)(
    input  wire                        clk,
    input  wire                        rst,
    input  wire signed [DWIDTH-1:0]    In_I,
    input  wire signed [DWIDTH-1:0]    In_Q,
    input  wire signed [SNR_WIDTH-1:0] sigma_scale,
    output wire signed [DWIDTH-1:0]    Out_I,
    output wire signed [DWIDTH-1:0]    Out_Q
);

    // ================================================================
    // 1) LOS 4 FIR (canal complejo)
    // ================================================================
    wire signed [DWIDTH-1:0] y_ii, y_qq, y_iq, y_qi; // salidas internas de los FIR

    // I -> I : I*hR
    filtro_fir #(
        .H(L_CH), .W(DWIDTH), .CW(CWIDTH),
        .DATA_F(DATA_F), .COEF_F(COEF_F),
        .SATURATE_EN(1'b1), .ROUND_EN(1'b0),
        .COEFFS_VECTOR(H_REAL_INIT)
    ) u_fir_ii (
        .clk(clk), .rst(rst), .din(In_I), .dout(y_ii)
    );

    // Q -> Q : Q*hR
    filtro_fir #(
        .H(L_CH), .W(DWIDTH), .CW(CWIDTH),
        .DATA_F(DATA_F), .COEF_F(COEF_F),
        .SATURATE_EN(1'b1), .ROUND_EN(1'b0),
        .COEFFS_VECTOR(H_REAL_INIT)
    ) u_fir_qq (
        .clk(clk), .rst(rst), .din(In_Q), .dout(y_qq)
    );

    // I -> Q : I*hI
    filtro_fir #(
        .H(L_CH), .W(DWIDTH), .CW(CWIDTH),
        .DATA_F(DATA_F), .COEF_F(COEF_F),
        .SATURATE_EN(1'b1), .ROUND_EN(1'b0),
        .COEFFS_VECTOR(H_IMAG_INIT)
    ) u_fir_iq (
        .clk(clk), .rst(rst), .din(In_I), .dout(y_iq)
    );

    // Q -> I : Q*hI
    filtro_fir #(
        .H(L_CH), .W(DWIDTH), .CW(CWIDTH),
        .DATA_F(DATA_F), .COEF_F(COEF_F),
        .SATURATE_EN(1'b1), .ROUND_EN(1'b0),
        .COEFFS_VECTOR(H_IMAG_INIT)
    ) u_fir_qi (
        .clk(clk), .rst(rst), .din(In_Q), .dout(y_qi)
    );

    // ================================================================
    // 2) COMBINACIÓN COMPLEJA
    // yI = I*hR - Q*hI
    // yQ = I*hI + Q*hR
    // ================================================================
    reg signed [DWIDTH-1:0] channel_out_I;
    reg signed [DWIDTH-1:0] channel_out_Q;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            channel_out_I <= {DWIDTH{1'b0}};
            channel_out_Q <= {DWIDTH{1'b0}};
        end else begin
            channel_out_I <= y_ii - y_qi;  // I*hR - Q*hI
            channel_out_Q <= y_iq + y_qq;  // I*hI + Q*hR
        end
    end

    // ================================================================
    // GENERADORES DE RUIDO GNG (OpenCores)
    // ================================================================
    wire signed [NOISE_WIDTH-1:0] raw_noise_I;
    wire signed [NOISE_WIDTH-1:0] raw_noise_Q;

    // Generador para el Canal I
    // Usamos las semillas por defecto 
    gng #(
        .INIT_Z1(64'h123456789ABCDEF0),
        .INIT_Z2(64'h0FEDCBA987654321),
        .INIT_Z3(64'hA5A5A5A55A5A5A5A)
    ) u_noise_gen_I (
        .clk        (clk),
        .rstn      (!rst),      // Ojo: Tu rst es activo alto, el IP pide activo bajo
        .ce         (1'b1),      // Siempre habilitado
        .valid_out  (),          // No lo necesitamos si asumimos flujo continuo
        .data_out   (raw_noise_I)
    );

    // Generador para el Canal Q
    gng #(
        .INIT_Z1(64'h876543210FEDCBA9), 
        .INIT_Z2(64'h1029384756AFBECD), 
        .INIT_Z3(64'hF0F0F0F00F0F0F0F)  
    ) u_noise_gen_Q (
        .clk        (clk),
        .rstn      (!rst),
        .ce         (1'b1),
        .valid_out  (),
        .data_out   (raw_noise_Q)
    );

    wire signed [NOISE_WIDTH + SNR_WIDTH - 1 : 0] mult_res_I;
    wire signed [NOISE_WIDTH + SNR_WIDTH - 1 : 0] mult_res_Q;

    assign mult_res_I = raw_noise_I * sigma_scale;
    assign mult_res_Q = raw_noise_Q * sigma_scale;

    // se guarda el ruido con ancho de producto 
    wire signed [NOISE_WIDTH + SNR_WIDTH - 1 : 0] noise_scaled_I;
    wire signed [NOISE_WIDTH + SNR_WIDTH - 1 : 0] noise_scaled_Q;

    assign noise_scaled_I = mult_res_I >>> SIGMA_F;
    assign noise_scaled_Q = mult_res_Q >>> SIGMA_F;

    // Saturación del ruido a DWIDTH antes de sumarlo (evita wrap)
    localparam signed [DWIDTH-1:0] MAX_POS = {1'b0, {(DWIDTH-1){1'b1}}};
    localparam signed [DWIDTH-1:0] MAX_NEG = {1'b1, {(DWIDTH-1){1'b0}}};

    function [DWIDTH-1:0] sat_to_DWIDTH;
        input signed [NOISE_WIDTH + SNR_WIDTH - 1 : 0] x;
        begin
            // Overflow si los bits altos no son extensión del signo de x[DWIDTH-1]
            if (| ( x[NOISE_WIDTH+SNR_WIDTH-1:DWIDTH] ^
                    {(NOISE_WIDTH+SNR_WIDTH-DWIDTH){x[DWIDTH-1]}} )) begin
                sat_to_DWIDTH = x[NOISE_WIDTH+SNR_WIDTH-1] ? MAX_NEG : MAX_POS;
            end else begin
                sat_to_DWIDTH = x[DWIDTH-1:0];
            end
        end
    endfunction

    wire signed [DWIDTH-1:0] final_noise_I;
    wire signed [DWIDTH-1:0] final_noise_Q;

    assign final_noise_I = sat_to_DWIDTH(noise_scaled_I);
    assign final_noise_Q = sat_to_DWIDTH(noise_scaled_Q);

    // Suma final con saturación
    wire signed [DWIDTH:0] sum_temp_I;
    wire signed [DWIDTH:0] sum_temp_Q;

    assign sum_temp_I = {channel_out_I[DWIDTH-1], channel_out_I} + {final_noise_I[DWIDTH-1], final_noise_I};
    assign sum_temp_Q = {channel_out_Q[DWIDTH-1], channel_out_Q} + {final_noise_Q[DWIDTH-1], final_noise_Q};

    assign Out_I = (sum_temp_I[DWIDTH] ^ sum_temp_I[DWIDTH-1]) ?
                   (sum_temp_I[DWIDTH] ? MAX_NEG : MAX_POS) :
                   sum_temp_I[DWIDTH-1:0];

    assign Out_Q = (sum_temp_Q[DWIDTH] ^ sum_temp_Q[DWIDTH-1]) ?
                   (sum_temp_Q[DWIDTH] ? MAX_NEG : MAX_POS) :
                   sum_temp_Q[DWIDTH-1:0];

endmodule
