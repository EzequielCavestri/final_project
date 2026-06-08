`timescale 1ns/1ps
`default_nettype none

// ============================================================
// update_lms.v  —  Actualización de pesos PBFDAF-LMS
//
// Fórmula:
//   w_new[k] = sat( w_old[k] + (grad_t[k] >>> mu_sh_eff) )
//
// mu_sh_eff conmuta automáticamente:
//   frames 0 .. N_SWITCH-1  →  i_mu_sh_init   (convergencia rápida)
//   frames N_SWITCH .. inf  →  i_mu_sh_final  (estado estable)
//
// Arquitectura:
//   - N=16 registros complejos Q(17,10), inicializados a cero
//   - Read combinacional → Vivado infiere LUTRAM
//   - Read-before-write garantizado en sim y síntesis
//   - Latencia de salida: 1 ciclo
//   - frame_cnt ampliado a 16 bits para control desde VIO
// ============================================================

module update_lms #(
    parameter integer NB_W        = 17,
    parameter integer NBF_W       = 10,
    parameter integer N           = 16
    // ELIMINADOS: MU_SH_INIT, MU_SH_FINAL, y N_SWITCH 
    // Ahora entran por los puertos dinámicos.
)(
    input  wire                    clk,
    input  wire                    rst,

    // Entrada: grad_t de PROYECCION (N muestras por frame)
    input  wire                    i_valid,
    input  wire                    i_start,
    input  wire signed [NB_W-1:0]  i_gI,
    input  wire signed [NB_W-1:0]  i_gQ,

    // Entradas de control en vivo (Desde el VIO en top_fpga)
    input  wire [3:0]              i_mu_sh_init,
    input  wire [3:0]              i_mu_sh_final,
    input  wire [15:0]             i_n_switch,

    // Salida: w_new hacia ZERO_PAD_PESOS (N muestras por frame)
    output reg                     o_valid,
    output reg                     o_start,
    output reg  signed [NB_W-1:0]  o_wI,
    output reg  signed [NB_W-1:0]  o_wQ,

    // Debug: estado del mu_switch 
    output wire                    o_switched,
    output wire [15:0]             o_frame_cnt  // AMPLIADO a 16 bits
);

    // ============================================================
    // Parámetros derivados
    // ============================================================
    localparam integer KW  = $clog2(N);
    localparam [KW-1:0] N1 = N - 1;

    // ============================================================
    // Banco de pesos  (LUTRAM en Vivado para N=16)
    // ============================================================
    reg signed [NB_W-1:0] w_re [0:N-1];
    reg signed [NB_W-1:0] w_im [0:N-1];

    integer ii;
    initial begin
        for (ii = 0; ii < N; ii = ii + 1) begin
            w_re[ii] = (ii == 7) ? 17'sd1024 : {NB_W{1'b0}};
            w_im[ii] = {NB_W{1'b0}};
        end
    end

    // ============================================================
    // mu_switch
    // ============================================================
    reg [15:0] frame_cnt;  // AMPLIADO a 16 bits
    reg        switched;

    assign o_switched  = switched;
    assign o_frame_cnt = frame_cnt;

    // ============================================================
    // Contador de muestra dentro del frame
    // eff_samp = 0 cuando llega i_start
    // ============================================================
    reg [KW-1:0] samp;
    wire [KW-1:0] eff_samp = (i_valid && i_start) ? {KW{1'b0}} : samp;

    // ============================================================
    // Shift de mu: pre-calcular ambas versiones y seleccionar
    // Vivado sintetiza esto como un mux, no como shift variable
    // Costo: ~17 LUT2
    // ============================================================
    // AHORA USA LOS INPUTS (i_mu_sh_init e i_mu_sh_final) EN VEZ DE PARAMETERS
    wire signed [NB_W-1:0] mu_gI_fast = i_gI[NB_W-1] ?
        -($signed(-i_gI) >>> i_mu_sh_init) : ($signed(i_gI) >>> i_mu_sh_init);
        
    wire signed [NB_W-1:0] mu_gI_slow = i_gI[NB_W-1] ?
        -($signed(-i_gI) >>> i_mu_sh_final) : ($signed(i_gI) >>> i_mu_sh_final);
        
    wire signed [NB_W-1:0] mu_gQ_fast = i_gQ[NB_W-1] ?
        -($signed(-i_gQ) >>> i_mu_sh_init) : ($signed(i_gQ) >>> i_mu_sh_init);
        
    wire signed [NB_W-1:0] mu_gQ_slow = i_gQ[NB_W-1] ?
        -($signed(-i_gQ) >>> i_mu_sh_final) : ($signed(i_gQ) >>> i_mu_sh_final);

    wire signed [NB_W-1:0] mu_gI = switched ? mu_gI_slow : mu_gI_fast;
    wire signed [NB_W-1:0] mu_gQ = switched ? mu_gQ_slow : mu_gQ_fast;

    // ============================================================
    // Suma con 1 bit de guardia para detectar overflow
    // ============================================================
    wire signed [NB_W:0] sum_re = {w_re[eff_samp][NB_W-1], w_re[eff_samp]}
                                + {mu_gI[NB_W-1],           mu_gI};
    wire signed [NB_W:0] sum_im = {w_im[eff_samp][NB_W-1], w_im[eff_samp]}
                                + {mu_gQ[NB_W-1],           mu_gQ};

    // Overflow si bit de guardia != bit de signo del resultado
    wire ovf_re = (sum_re[NB_W] != sum_re[NB_W-1]);
    wire ovf_im = (sum_im[NB_W] != sum_im[NB_W-1]);

    // Saturación: MAX = 0_1111...1, MIN = 1_0000...0
    wire signed [NB_W-1:0] new_wI = ovf_re ?
        (sum_re[NB_W] ? {1'b1,{(NB_W-1){1'b0}}} : {1'b0,{(NB_W-1){1'b1}}}) :
        sum_re[NB_W-1:0];

    wire signed [NB_W-1:0] new_wQ = ovf_im ?
        (sum_im[NB_W] ? {1'b1,{(NB_W-1){1'b0}}} : {1'b0,{(NB_W-1){1'b1}}}) :
        sum_im[NB_W-1:0];

    // ============================================================
    // Lógica síncrona
    // ============================================================
    always @(posedge clk) begin
        if (rst) begin
            samp      <= {KW{1'b0}};
            frame_cnt <= 16'd0; // AMPLIADO a 16 bits
            switched  <= 1'b0;
            o_valid   <= 1'b0;
            o_start   <= 1'b0;
            o_wI      <= {NB_W{1'b0}};
            o_wQ      <= {NB_W{1'b0}};
        end else if (i_valid) begin

            // Actualizar banco de pesos
            w_re[eff_samp] <= new_wI;
            w_im[eff_samp] <= new_wQ;

            // Emitir peso actualizado (latencia 1 ciclo)
            o_wI    <= new_wI;
            o_wQ    <= new_wQ;
            o_start <= i_start;
            o_valid <= 1'b1;
            
            // Debug
            if (eff_samp < 4) begin
                $display("[LMS] samp=%0d  grad=(%0d,%0d)  mu_g=(%0d,%0d)  w_new=(%0d,%0d)",
                        eff_samp, i_gI, i_gQ, mu_gI, mu_gQ, new_wI, new_wQ);
            end
            
            // Avanzar contador de muestras
            samp <= (eff_samp == N1) ? {KW{1'b0}} : (eff_samp + 1'b1);

            // mu_switch: contar frames (se detecta al final de cada frame)
            if (eff_samp == N1 && !switched) begin
                // AHORA USA >= Y EL INPUT i_n_switch
                if (frame_cnt >= i_n_switch - 1)
                    switched <= 1'b1;
                else
                    frame_cnt <= frame_cnt + 16'd1;
            end

        end else begin
            o_valid <= 1'b0;
            o_start <= 1'b0;
        end
    end

endmodule

`default_nettype wire