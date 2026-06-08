`timescale 1ns / 1ps
`default_nettype none

module top_fpga (
    input wire sys_clk_p,
    input wire sys_clk_n
);

    // ============================================================
    // 1. Generador de Relojes (Clocking Wizard)
    // ============================================================
    wire clk_fast;
    wire locked;
    wire clk_low;   

    // El Clocking Wizard configurado como 'Differential' ya contiene el IBUFDS.
    // No agregues otro buffer externo para no tener errores de conexión.
    clk_wiz_0 u_clocks (
    .clk_in1_p (sys_clk_p),
    .clk_in1_n (sys_clk_n),
    .clk_out1  (clk_fast),   // ← clk_out1 = 100 MHz
    .clk_out2  (clk_low),    // ← clk_out2 = 50 MHz
    .reset     (1'b0),
    .locked    (locked)
    );

    // ============================================================
    // 2. VIO (Panel de Control)
    // ============================================================
    wire        vio_rst;
    wire [10:0] vio_sigma_scale;
    wire [3:0]  vio_mu_init;
    wire [3:0]  vio_mu_final;
    wire [15:0] vio_n_switch;
    
    vio_0 u_vio (
        .clk        (clk_fast),
        .probe_out0 (vio_rst),
        .probe_out1 (vio_sigma_scale),
        .probe_out2 (vio_mu_init),
        .probe_out3 (vio_mu_final),
        .probe_out4 (vio_n_switch)
    );

    // ============================================================
    // 3. Sincronizador de Reset
    // ============================================================
    reg [1:0] rst_sync;
    wire      sys_rst_safe = rst_sync[1];

    always @(posedge clk_fast) begin
        if (!locked || vio_rst) begin
            rst_sync <= 2'b11;
        end else begin
            rst_sync <= {rst_sync[0], 1'b0};
        end
    end

    // ============================================================
    // 4. Núcleo DSP (top_global)
    // ============================================================
    wire        dn_out_valid;
    wire        dn_out_start;
    wire signed [8:0] dn_out_I;
    wire signed [8:0] dn_out_Q;
    wire        sl_out_valid;
    wire        sl_out_start;
    wire signed [8:0] sl_out_yhat_I;
    wire signed [8:0] sl_out_yhat_Q;
    wire signed [8:0] sl_out_e_I;
    wire signed [8:0] sl_out_e_Q;

    top_global #(
        .N_OS(16), 
        .WN(9), 
        .NFFT(32)
    ) u_dsp_core (
        .clk_fast     (clk_fast),
        .rst          (sys_rst_safe),
        .sigma_scale  (vio_sigma_scale),
        .clk_low      (clk_low), 
        .dn_out_valid (dn_out_valid),
        .dn_out_start (dn_out_start),
        .dn_out_I     (dn_out_I),
        .dn_out_Q     (dn_out_Q),
        .sl_out_valid (sl_out_valid),
        .sl_out_start (sl_out_start),
        .sl_out_yhat_I(sl_out_yhat_I),
        .sl_out_yhat_Q(sl_out_yhat_Q),
        .sl_out_e_I   (sl_out_e_I),
        .sl_out_e_Q   (sl_out_e_Q),
        .mu_sh_init   (vio_mu_init),
        .mu_sh_final  (vio_mu_final),
        .n_switch     (vio_n_switch)
    );

    // ============================================================
    // 5. ILA (Osciloscopio)
    // ============================================================
    ila_0 u_ila (
        .clk    (clk_fast),
        .probe0 (dn_out_valid), 
        .probe1 (dn_out_start), 
        .probe2 (dn_out_I),    
        .probe3 (dn_out_Q),    
        .probe4 (sl_out_e_I),  
        .probe5 (sl_out_e_Q)
    );

endmodule
`default_nettype wire