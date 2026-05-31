`timescale 1ns/1ps
`default_nettype none

// ============================================================
// top_global_all  v14 (Loop LMS cerrado de procesamiento limpio)
// ============================================================

module top_global #(
    parameter integer N_OS      = 16,
    parameter integer WN        = 9,
    parameter integer FIFO_AW   = 8,

    parameter integer NFFT      = 32,
    parameter integer LOGN      = 5,
    parameter integer NB_INT    = 17,
    parameter integer NBF_INT   = 10,
    parameter integer REORDER_BITREV = 1,

    parameter integer CH_GAIN_SH = 0,
    parameter integer K_HIST     = 1
)(
    // --- Relojes y Control Esenciales ---
    input  wire                 clk_fast,
    input  wire                 rst,
    input  wire                 enable_div,
    input  wire [10:0]          sigma_scale,
    input  wire [3:0]           mu_sh_init,
    input  wire [3:0]           mu_sh_final,
    input  wire [15:0]          n_switch,

    output wire                 clk_low,

    // --- SALIDAS ÚTILES DEL SISTEMA (Data Plane) ---
    // Muestras N..2N-1 ecualizadas útiles del frame (Salida del Filtro)
    output wire                 dn_out_valid,
    output wire                 dn_out_start,
    output wire signed [WN-1:0] dn_out_I,
    output wire signed [WN-1:0] dn_out_Q,

    // Símbolos Decididos y Error (Salida del Slicer QPSK)
    output wire                 sl_out_valid,   
    output wire                 sl_out_start,   
    output wire signed [WN-1:0] sl_out_yhat_I,  // Símbolo Re (±QPSK_A)
    output wire signed [WN-1:0] sl_out_yhat_Q,  // Símbolo Im (±QPSK_A)
    output wire signed [WN-1:0] sl_out_e_I,     // Error Re
    output wire signed [WN-1:0] sl_out_e_Q      // Error Im
);

    // ============================================================
    // WIRES INTERNOS (Toda la telemetría vieja ahora es interna)
    // ============================================================
    // TX & CH
    wire signed [WN-1:0] sI_tx, sQ_tx;
    wire signed [WN-1:0] sI_ch, sQ_ch;

    // OS Buffer
    wire                 os_overflow;
    wire                 os_start;
    wire                 os_valid;
    wire signed [WN-1:0] os_I;
    wire signed [WN-1:0] os_Q;

    // FIFO
    wire                 fifo_full;
    wire                 fifo_empty;
    wire                 fifo_overflow;
    wire [FIFO_AW:0]     fifo_count;
    
    // Entrada / Salida FFT Principal
    wire                 fft_in_valid;
    wire                 fft_in_start;
    wire signed [WN-1:0] fft_in_I;
    wire signed [WN-1:0] fft_in_Q;
    wire                 fft_out_valid;
    wire                 fft_out_start;
    wire signed [NB_INT-1:0] fft_out_I;
    wire signed [NB_INT-1:0] fft_out_Q;

    // History Buffer
    wire                     hb_out_valid;
    wire                     hb_out_start;
    wire signed [NB_INT-1:0] hb_out_curr_I;
    wire signed [NB_INT-1:0] hb_out_curr_Q;
    wire signed [NB_INT-1:0] hb_out_old_I;
    wire signed [NB_INT-1:0] hb_out_old_Q;

    // CMUL
    wire                     cmul_out_valid;
    wire                     cmul_out_start;
    wire signed [NB_INT-1:0] cmul_out_I;
    wire signed [NB_INT-1:0] cmul_out_Q;

    // IFFT Principal
    wire                 ifft_out_valid;
    wire                 ifft_out_start;
    wire signed [WN-1:0] ifft_out_I;
    wire signed [WN-1:0] ifft_out_Q;

    // Zero Pad Error & FFT Error
    wire                 zpe_out_valid;  
    wire                 zpe_out_start;  
    wire signed [WN-1:0] zpe_out_eI;     
    wire signed [WN-1:0] zpe_out_eQ;     
    wire                 ffte_out_valid; 
    wire                 ffte_out_start; 
    wire signed [NB_INT-1:0] ffte_out_I;     
    wire signed [NB_INT-1:0] ffte_out_Q;     

    // XHist Delay & Gradiente
    wire                 xhd_out_valid;  
    wire                 xhd_out_start;  
    wire signed [NB_INT-1:0] xhd_out_re;     
    wire signed [NB_INT-1:0] xhd_out_im;     
    wire                 grad_out_valid;  
    wire                 grad_out_start;  
    wire signed [NB_INT-1:0] grad_out_re;     
    wire signed [NB_INT-1:0] grad_out_im;     

    // IFFT Gradiente & Proyección
    wire                 ifft_grad_valid;
    wire                 ifft_grad_start;
    wire signed [NB_INT-1:0] ifft_grad_I;
    wire signed [NB_INT-1:0] ifft_grad_Q;
    wire                 grad_t_valid;
    wire                 grad_t_start;
    wire signed [NB_INT-1:0] grad_t_I;
    wire signed [NB_INT-1:0] grad_t_Q;

    // LMS Update & Zero Pad Pesos
    wire                 lms_w_valid;   
    wire                 lms_w_start;   
    wire signed [NB_INT-1:0] lms_w_I;       
    wire signed [NB_INT-1:0] lms_w_Q;       
    wire                 lms_switched;  
    wire [7:0]           lms_frame_cnt; 
    wire                 zpp_out_valid;
    wire                 zpp_out_start;
    wire signed [NB_INT-1:0] zpp_out_wI;
    wire signed [NB_INT-1:0] zpp_out_wQ;

    // FFT Pesos
    wire                 fft_w_valid;
    wire                 fft_w_start;
    wire signed [NB_INT-1:0] fft_w_I;
    wire signed [NB_INT-1:0] fft_w_Q;

    // ============================================================
    // ESTRUCTURA INTERNA
    // ============================================================
    
    clock_div2 u_div2 (
        .i_clk_fast(clk_fast),
        .i_enable  (enable_div),
        .o_clk_low (clk_low)
    );

    reg i_en_tx;
    always @(posedge clk_low) begin
        if (rst) i_en_tx <= 1'b0;
        else     i_en_tx <= 1'b1;
    end

    top_tx u_tx (
        .clk   (clk_low),
        .reset (rst),
        .i_en  (i_en_tx),
        .sI_out(sI_tx),
        .sQ_out(sQ_tx)
    );

    top_ch u_ch (
        .clk        (clk_low),
        .rst        (rst),
        .In_I       (sI_tx),
        .In_Q       (sQ_tx),
        .sigma_scale(sigma_scale),
        .Out_I      (sI_ch),
        .Out_Q      (sQ_ch)
    );

    function signed [WN-1:0] sat_wn;
        input signed [WN+7:0] x;
        reg signed [WN-1:0] maxv, minv;
        begin
            maxv = {1'b0, {(WN-1){1'b1}}};
            minv = {1'b1, {(WN-1){1'b0}}};
            if      (x > $signed(maxv)) sat_wn = maxv;
            else if (x < $signed(minv)) sat_wn = minv;
            else                        sat_wn = x[WN-1:0];
        end
    endfunction

    wire signed [WN+7:0] sI_ch_ext = {{8{sI_ch[WN-1]}}, sI_ch};
    wire signed [WN+7:0] sQ_ch_ext = {{8{sQ_ch[WN-1]}}, sQ_ch};
    wire signed [WN+7:0] sI_ch_sh  = (CH_GAIN_SH > 0) ? (sI_ch_ext <<< CH_GAIN_SH) : sI_ch_ext;
    wire signed [WN+7:0] sQ_ch_sh  = (CH_GAIN_SH > 0) ? (sQ_ch_ext <<< CH_GAIN_SH) : sQ_ch_ext;
    wire signed [WN-1:0] sI_ch_os  = sat_wn(sI_ch_sh);
    wire signed [WN-1:0] sQ_ch_os  = sat_wn(sQ_ch_sh);

    reg i_valid_low;
    always @(posedge clk_low) begin
        if (rst) i_valid_low <= 1'b0;
        else     i_valid_low <= 1'b1;
    end

    os_buffer #(
        .N (N_OS),
        .WN(WN)
    ) u_os (
        .i_clk_low (clk_low),
        .i_clk_fast(clk_fast),
        .i_rst     (rst),
        .i_valid   (i_valid_low),
        .i_i       (sI_ch_os),
        .i_q       (sQ_ch_os),
        .o_overflow(os_overflow),
        .o_start   (os_start),
        .o_valid   (os_valid),
        .o_i       (os_I),
        .o_q       (os_Q)
    );

    wire [2*WN:0] fifo_din  = {os_start, os_I, os_Q};
    wire [2*WN:0] fifo_dout;
    wire fifo_rd_en = !fifo_empty;

    fifo #(
        .DATA_WIDTH(2*WN + 1),
        .ADDR_WIDTH(FIFO_AW)
    ) u_fifo (
        .clk       (clk_fast),
        .rst       (rst),
        .din       (fifo_din),
        .wr_en     (os_valid),
        .rd_en     (fifo_rd_en),
        .dout      (fifo_dout),
        .full      (fifo_full),
        .empty     (fifo_empty),
        .valid     (/* no usado */),
        .overflow  (fifo_overflow),
        .data_count(fifo_count)
    );

    wire rd_fire = fifo_rd_en && !fifo_empty;
    reg  rd_fire_q;
    always @(posedge clk_fast) begin
        if (rst) rd_fire_q <= 1'b0;
        else     rd_fire_q <= rd_fire;
    end

    assign fft_in_valid = rd_fire_q;
    assign fft_in_start = fifo_dout[2*WN];
    assign fft_in_I     = fifo_dout[2*WN-1:WN];
    assign fft_in_Q     = fifo_dout[WN-1:0];

    wire fft_rdy;
    fft_ifft_stream #(
        .NFFT(NFFT), .LOGN(LOGN),
        .NB_IN(WN),      .NBF_IN(7),
        .NB_W(NB_INT),   .NBF_W(NBF_INT),
        .NB_OUT(NB_INT), .NBF_OUT(NBF_INT),
        .BF_SCALE(0),
        .REORDER_BITREV(REORDER_BITREV)
    ) u_fft (
        .i_clk    (clk_fast),
        .i_rst    (rst),
        .i_valid  (fft_in_valid),
        .i_start  (fft_in_start),
        .i_xI     (fft_in_I),
        .i_xQ     (fft_in_Q),
        .i_inverse(1'b0),
        .o_in_ready(fft_rdy),
        .o_start  (fft_out_start),
        .o_valid  (fft_out_valid),
        .o_yI     (fft_out_I),
        .o_yQ     (fft_out_Q)
    );

    wire [$clog2(K_HIST+1)-1:0] hb_wr_bank_dbg;
    wire [$clog2(NFFT)-1:0]     hb_samp_dbg;

    history_buffer #(
        .NB_W(NB_INT),
        .NFFT(NFFT),
        .K   (K_HIST)
    ) u_hb (
        .clk        (clk_fast),
        .rst        (rst),
        .i_valid    (fft_out_valid),
        .i_start    (fft_out_start),
        .i_xI       (fft_out_I),
        .i_xQ       (fft_out_Q),
        .o_valid    (hb_out_valid),
        .o_start    (hb_out_start),
        .o_X_curr_re(hb_out_curr_I),
        .o_X_curr_im(hb_out_curr_Q),
        .o_X_old_re (hb_out_old_I),
        .o_X_old_im (hb_out_old_Q),
        .o_wr_bank  (hb_wr_bank_dbg),
        .o_samp_idx (hb_samp_dbg)
    );

    reg  [$clog2(NFFT)-1:0] cmul_wr_cnt;
    wire [$clog2(NFFT)-1:0] cmul_eff_wk = (fft_w_valid && fft_w_start)
                                          ? {$clog2(NFFT){1'b0}}
                                          : cmul_wr_cnt;

    always @(posedge clk_fast) begin
        if (rst) begin
            cmul_wr_cnt <= {$clog2(NFFT){1'b0}};
        end else if (fft_w_valid) begin
            cmul_wr_cnt <= (cmul_eff_wk == (NFFT-1))
                         ? {$clog2(NFFT){1'b0}}
                         : (cmul_eff_wk + 1'b1);
        end
    end

    wire [$clog2(NFFT)-1:0] cmul_samp_dbg;
    cmul_pbfdaf #(
        .NB_W (NB_INT),
        .NBF_W(NBF_INT),
        .NFFT (NFFT)
    ) u_cmul (
        .clk       (clk_fast),
        .rst       (rst),
        .i_valid   (hb_out_valid),
        .i_start   (hb_out_start),
        .i_X0_re   (hb_out_curr_I),
        .i_X0_im   (hb_out_curr_Q),
        .i_X1_re   (hb_out_old_I),
        .i_X1_im   (hb_out_old_Q),
        .i_we      (fft_w_valid), 
        .i_wk      (cmul_eff_wk),
        .i_wsel    (1'b0),          
        .i_W_re    (fft_w_I),
        .i_W_im    (fft_w_Q),
        .o_valid   (cmul_out_valid),
        .o_start   (cmul_out_start),
        .o_yI      (cmul_out_I),
        .o_yQ      (cmul_out_Q),
        .o_samp_idx(cmul_samp_dbg)
    );

    wire ifft_rdy;
    fft_ifft_stream #(
        .NFFT(NFFT), .LOGN(LOGN),
        .NB_IN(NB_INT),  .NBF_IN(NBF_INT),
        .NB_W(NB_INT),   .NBF_W(NBF_INT),
        .NB_OUT(WN),     .NBF_OUT(7),
        .BF_SCALE(0),
        .REORDER_BITREV(REORDER_BITREV)
    ) u_ifft (
        .i_clk    (clk_fast),
        .i_rst    (rst),
        .i_valid  (cmul_out_valid),
        .i_start  (cmul_out_start),
        .i_xI     (cmul_out_I),
        .i_xQ     (cmul_out_Q),
        .i_inverse(1'b1),
        .o_in_ready(ifft_rdy),
        .o_start  (ifft_out_start),
        .o_valid  (ifft_out_valid),
        .o_yI     (ifft_out_I),
        .o_yQ     (ifft_out_Q)
    );

    discard_n #(
        .NB_W (WN),
        .NBF_W(7),
        .NFFT (NFFT)
    ) u_dn (
        .clk       (clk_fast),
        .rst       (rst),
        .i_valid   (ifft_out_valid),
        .i_start   (ifft_out_start),
        .i_yI      (ifft_out_I),
        .i_yQ      (ifft_out_Q),
        .o_valid   (dn_out_valid),
        .o_start   (dn_out_start),
        .o_yI      (dn_out_I),
        .o_yQ      (dn_out_Q),
        .o_samp_idx()
    );

    slicer_qpsk #(
        .NB_W (WN),
        .NBF_W(7),
        .NFFT (NFFT)
    ) u_slicer (
        .clk       (clk_fast),
        .rst       (rst),
        .i_valid   (dn_out_valid),
        .i_start   (dn_out_start),
        .i_yI      (dn_out_I),
        .i_yQ      (dn_out_Q),
        .o_valid   (sl_out_valid),
        .o_start   (sl_out_start),
        .o_yhat_I  (sl_out_yhat_I),
        .o_yhat_Q  (sl_out_yhat_Q),
        .o_e_I     (sl_out_e_I),
        .o_e_Q     (sl_out_e_Q)
    );

    zero_pad_error #(
        .NB_W (WN),
        .NFFT (NFFT)
    ) u_zpe (
        .clk     (clk_fast),
        .rst     (rst),
        .i_valid (sl_out_valid),
        .i_start (sl_out_start),
        .i_eI    (sl_out_e_I),
        .i_eQ    (sl_out_e_Q),
        .o_valid (zpe_out_valid),
        .o_start (zpe_out_start),
        .o_eI    (zpe_out_eI),
        .o_eQ    (zpe_out_eQ)
    );

    fft_ifft_stream #(
        .NFFT(NFFT), .LOGN(LOGN),
        .NB_IN(WN),      .NBF_IN(7),
        .NB_W(NB_INT),   .NBF_W(NBF_INT),
        .NB_OUT(NB_INT), .NBF_OUT(NBF_INT),
        .BF_SCALE(0),
        .REORDER_BITREV(REORDER_BITREV)
    ) u_fft_error (
        .i_clk    (clk_fast),
        .i_rst    (rst),
        .i_valid  (zpe_out_valid),
        .i_start  (zpe_out_start),
        .i_xI     (zpe_out_eI),
        .i_xQ     (zpe_out_eQ),
        .i_inverse(1'b0),
        .o_in_ready(),
        .o_start  (ffte_out_start),
        .o_valid  (ffte_out_valid),
        .o_yI     (ffte_out_I),
        .o_yQ     (ffte_out_Q)
    );

    xhist_delay #(
        .NB_W (NB_INT),
        .DELAY(118)
    ) u_xhd (
        .clk    (clk_fast),
        .rst    (rst),
        .i_valid(hb_out_valid),
        .i_start(hb_out_start),
        .i_xre  (hb_out_curr_I),
        .i_xim  (hb_out_curr_Q),
        .o_valid(xhd_out_valid),
        .o_start(xhd_out_start),
        .o_xre  (xhd_out_re),
        .o_xim  (xhd_out_im)
    );

    gradiente #(
        .NB_W (NB_INT),
        .NBF_W(NBF_INT)
    ) u_grad (
        .clk    (clk_fast),
        .rst    (rst),
        .i_valid(ffte_out_valid),
        .i_start(ffte_out_start),
        .i_xre  (xhd_out_re),
        .i_xim  (xhd_out_im),
        .i_ere  (ffte_out_I),
        .i_eim  (ffte_out_Q),
        .o_valid(grad_out_valid),
        .o_start(grad_out_start),
        .o_phi_re(grad_out_re),
        .o_phi_im(grad_out_im)
    );

    wire ifft_grad_rdy;
    fft_ifft_stream #(
        .NFFT(NFFT), .LOGN(LOGN),
        .NB_IN(NB_INT),  .NBF_IN(NBF_INT),
        .NB_W(NB_INT),   .NBF_W(NBF_INT),
        .NB_OUT(NB_INT), .NBF_OUT(NBF_INT),
        .BF_SCALE(0),
        .REORDER_BITREV(REORDER_BITREV)
    ) u_ifft_grad (
        .i_clk    (clk_fast),
        .i_rst    (rst),
        .i_valid  (grad_out_valid),
        .i_start  (grad_out_start),
        .i_xI     (grad_out_re),
        .i_xQ     (grad_out_im),
        .i_inverse(1'b1),
        .o_in_ready(ifft_grad_rdy),
        .o_start  (ifft_grad_start),
        .o_valid  (ifft_grad_valid),
        .o_yI     (ifft_grad_I),
        .o_yQ     (ifft_grad_Q)
    );

    keep_first_n #(
        .NB_W (NB_INT),
        .NBF_W(NBF_INT),
        .NFFT (NFFT)
    ) u_proy (
        .clk    (clk_fast),
        .rst    (rst),
        .i_valid(ifft_grad_valid),
        .i_start(ifft_grad_start),
        .i_yI   (ifft_grad_I),
        .i_yQ   (ifft_grad_Q),
        .o_valid(grad_t_valid),
        .o_start(grad_t_start),
        .o_yI   (grad_t_I),
        .o_yQ   (grad_t_Q),
        .o_samp_idx()
    );

    update_lms #(
        .NB_W       (NB_INT),
        .NBF_W      (NBF_INT),
        .N          (NFFT / 2)        
    ) u_lms (
        .clk        (clk_fast),
        .rst        (rst),
        .i_valid    (grad_t_valid),
        .i_start    (grad_t_start),
        .i_gI       (grad_t_I),
        .i_gQ       (grad_t_Q),
        .i_mu_sh_init  (mu_sh_init),
        .i_mu_sh_final (mu_sh_final),
        .i_n_switch    (n_switch),
        .o_valid    (lms_w_valid),
        .o_start    (lms_w_start),
        .o_wI       (lms_w_I),
        .o_wQ       (lms_w_Q),
        .o_switched (lms_switched),
        .o_frame_cnt(lms_frame_cnt)
    );

    zero_pad_pesos #(
        .NB_W (NB_INT),
        .NBF_W(NBF_INT),
        .NFFT (NFFT)
    ) u_zpp (
        .clk    (clk_fast),
        .rst    (rst),
        .i_valid(lms_w_valid),
        .i_start(lms_w_start),
        .i_wI   (lms_w_I),
        .i_wQ   (lms_w_Q),
        .o_valid(zpp_out_valid),
        .o_start(zpp_out_start),
        .o_wI   (zpp_out_wI),
        .o_wQ   (zpp_out_wQ)
    );

    wire fft_w_rdy;
    fft_ifft_stream #(
        .NFFT(NFFT), .LOGN(LOGN),
        .NB_IN (NB_INT), .NBF_IN (NBF_INT),
        .NB_W  (NB_INT), .NBF_W  (NBF_INT),
        .NB_OUT(NB_INT), .NBF_OUT(NBF_INT),
        .BF_SCALE(0),
        .REORDER_BITREV(REORDER_BITREV)
    ) u_fft_pesos (
        .i_clk    (clk_fast),
        .i_rst    (rst),
        .i_valid  (zpp_out_valid),
        .i_start  (zpp_out_start),
        .i_xI     (zpp_out_wI),
        .i_xQ     (zpp_out_wQ),
        .i_inverse(1'b0),
        .o_in_ready(fft_w_rdy),
        .o_valid  (fft_w_valid),
        .o_start  (fft_w_start),
        .o_yI     (fft_w_I),
        .o_yQ     (fft_w_Q)
    );

endmodule
`default_nettype wire