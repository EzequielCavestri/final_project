`timescale 1us/1ns
`default_nettype none

// ============================================================
// top_global_all  v15 (PBFDAF K=2 particiones: W0 + W1 adaptivos)
//
// Cadena completa:
//   TX -> CH -> OS -> FIFO -> FFT -> HB -> CMUL -> IFFT -> DN -> SLICER -> ZPE -> FFT_ERROR
//                              └──── xhist_delay ────┘             │
//                                                              GRADIENTE
//                                                         PHI_k = conj(X_hist)*E_k
//
// NUEVO en v11:
//   - Instancia gradiente conectado a xhd_out y ffte_out
//   - Control: i_valid=ffte_out_valid, i_start=ffte_out_start
//   - Latencia: 2 ciclos desde ffte_out hasta grad_out
//   - Puertos nuevos: grad_out_valid, grad_out_start, grad_out_re/im
// NUEVO en v12:
//   - Instancia update_lms conectado a grad_t_*
//   - Puertos nuevos: lms_w_valid/start/I/Q, lms_switched, lms_frame_cnt
// NUEVO en v13:
//   - Instancia zero_pad_pesos: [w|0..0] → 32 muestras para FFT
//   - Instancia fft_pesos (fft_ifft_stream reutilizado): W_k = FFT([w|0])
//   - Puertos nuevos: zpp_out_*, fft_w_*
// NUEVO en v14:
//   - fft_pesos conectado al puerto de escritura del cmul_pbfdaf
//   - Contador wr_cnt convierte fft_w_valid/start → i_we/i_wk
//   - Loop LMS completamente cerrado (1 partición)
// NUEVO en v15:
//   - SEGUNDA PARTICIÓN (X_old): rama de update espejo
//       u_xhd_old → u_grad1 → u_ifft_grad1 → u_proy1 → u_lms1
//                 → u_zpp1 → u_fft_pesos1  →  W1 del CMUL
//   - cmul_pbfdaf con DOS puertos de escritura (W0 y W1 concurrentes)
//   - update_lms parametrizado: u_lms (impulso tap0) / u_lms1 (cero)
//   - W0 y W1 salen en lockstep → comparten cmul_eff_wk / cmul_wr_cnt
// ============================================================

module top_global_all #(
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
    input  wire                 clk_fast,
    input  wire                 rst,
    input  wire                 enable_div,
    input  wire [10:0]          sigma_scale,

    output wire                 clk_low,

    // --- Debug TX/CH ---
    output wire signed [WN-1:0] tx_I_dbg,
    output wire signed [WN-1:0] tx_Q_dbg,
    output wire signed [WN-1:0] ch_I_dbg,
    output wire signed [WN-1:0] ch_Q_dbg,

    // --- OS buffer ---
    output wire                 os_overflow,
    output wire                 os_start,
    output wire                 os_valid,
    output wire signed [WN-1:0] os_I,
    output wire signed [WN-1:0] os_Q,

    // --- FIFO ---
    output wire                 fifo_full,
    output wire                 fifo_empty,
    output wire                 fifo_overflow,
    output wire [FIFO_AW:0]     fifo_count,

    // --- Entrada a la FFT (debug) ---
    output wire                 fft_in_valid,
    output wire                 fft_in_start,
    output wire signed [WN-1:0] fft_in_I,
    output wire signed [WN-1:0] fft_in_Q,

    // --- Salida de la FFT ---
    output wire                 fft_out_valid,
    output wire                 fft_out_start,
    output wire signed [NB_INT-1:0] fft_out_I,
    output wire signed [NB_INT-1:0] fft_out_Q,

    // --- History Buffer ---
    output wire                     hb_out_valid,
    output wire                     hb_out_start,
    output wire signed [NB_INT-1:0] hb_out_curr_I,
    output wire signed [NB_INT-1:0] hb_out_curr_Q,
    output wire signed [NB_INT-1:0] hb_out_old_I,
    output wire signed [NB_INT-1:0] hb_out_old_Q,

    // --- CMUL ---
    output wire                     cmul_out_valid,
    output wire                     cmul_out_start,
    output wire signed [NB_INT-1:0] cmul_out_I,
    output wire signed [NB_INT-1:0] cmul_out_Q,

    // --- IFFT (cruda, para debug) ---
    output wire                 ifft_out_valid,
    output wire                 ifft_out_start,
    output wire signed [WN-1:0] ifft_out_I,
    output wire signed [WN-1:0] ifft_out_Q,

    // --- DISCARD_N: y_blk (muestras N..2N-1 del frame) ---
    output wire                 dn_out_valid,
    output wire                 dn_out_start,
    output wire signed [WN-1:0] dn_out_I,
    output wire signed [WN-1:0] dn_out_Q,

    // --- SLICER_QPSK ---
    output wire                 sl_out_valid,   // 1 durante N muestras por frame
    output wire                 sl_out_start,   // 1 en la primera muestra del frame
    output wire signed [WN-1:0] sl_out_yhat_I,  // símbolo decidido Re  (±QPSK_A)
    output wire signed [WN-1:0] sl_out_yhat_Q,  // símbolo decidido Im  (±QPSK_A)
    output wire signed [WN-1:0] sl_out_e_I,     // error Re  e = yhat - y
    output wire signed [WN-1:0] sl_out_e_Q,     // error Im  e = yhat - y

    // --- ZERO_PAD_ERROR ---
    output wire                 zpe_out_valid,  // 1 durante 2N muestras por frame
    output wire                 zpe_out_start,  // 1 en la primera muestra (primer cero)
    output wire signed [WN-1:0] zpe_out_eI,     // Re: 0 (primera mitad) | e (segunda mitad)
    output wire signed [WN-1:0] zpe_out_eQ,     // Im: 0 (primera mitad) | e (segunda mitad)

    // --- FFT_ERROR ---
    output wire                       ffte_out_valid, // 1 durante 2N muestras por frame
    output wire                       ffte_out_start, // 1 en la primera muestra del frame
    output wire signed [NB_INT-1:0]   ffte_out_I,     // espectro del error Re  E_k
    output wire signed [NB_INT-1:0]   ffte_out_Q,     // espectro del error Im  E_k

    // --- XHIST_DELAY ---
    // X_hist retrasado 118 ciclos — sincronizado con ffte_out
    // VERIFICAR: xhd_out_start debe coincidir con ffte_out_start
    output wire                        xhd_out_valid,  // 1 cuando hay dato válido
    output wire                        xhd_out_start,  // 1 en primera muestra del frame
    output wire signed [NB_INT-1:0]    xhd_out_re,     // X_hist Re retrasado
    output wire signed [NB_INT-1:0]    xhd_out_im,     // X_hist Im retrasado

    // --- GRADIENTE ---
    // PHI_k = conj(X_hist) * E_k  — gradiente espectral del PBFDAF-LMS
    // Latencia 2 ciclos desde ffte_out.i_we (1'b0)
    output wire                        grad_out_valid,  // 1 cuando PHI_k es válido
    output wire                        grad_out_start,  // 1 en primera muestra del frame
    output wire signed [NB_INT-1:0]    grad_out_re,     // PHI_k parte real
    output wire signed [NB_INT-1:0]    grad_out_im,     // PHI_k parte imaginaria

    // --- IFFT_GRAD ---
    output wire                        ifft_grad_valid,
    output wire                        ifft_grad_start,
    output wire signed [NB_INT-1:0]    ifft_grad_I,
    output wire signed [NB_INT-1:0]    ifft_grad_Q,

    // --- PROYECCION (discard_n de phi(t)) ---
    output wire                        grad_t_valid,
    output wire                        grad_t_start,
    output wire signed [NB_INT-1:0]    grad_t_I,
    output wire signed [NB_INT-1:0]    grad_t_Q,

    // --- UPDATE_LMS ---
    // w_new[k] = sat(w_old[k] + grad_t[k] >>> MU_SH_eff)
    // Latencia: 1 ciclo desde grad_t
    output wire                        lms_w_valid,   // 1 durante N muestras/frame
    output wire                        lms_w_start,   // 1 en k=0
    output wire signed [NB_INT-1:0]    lms_w_I,       // w_new Re
    output wire signed [NB_INT-1:0]    lms_w_Q,       // w_new Im
    output wire                        lms_switched,  // 1 cuando ya usa MU_SH_FINAL
    output wire [7:0]                  lms_frame_cnt, // frame counter (satura en N_SWITCH)

    // --- ZERO_PAD_PESOS ---
    // [w[0..N-1] | 0..0]  — 32 muestras/frame para FFT_PESOS
    output wire                        zpp_out_valid,
    output wire                        zpp_out_start,
    output wire signed [NB_INT-1:0]    zpp_out_wI,
    output wire signed [NB_INT-1:0]    zpp_out_wQ,

    // --- FFT_PESOS (u_fft_pesos) — W0 ---
    // W_k = FFT([w|0..0]) — coeficientes en frecuencia
    output wire                        fft_w_valid,
    output wire                        fft_w_start,
    output wire signed [NB_INT-1:0]    fft_w_I,
    output wire signed [NB_INT-1:0]    fft_w_Q,

    // --- FFT_PESOS1 (u_fft_pesos1) — W1 (partición X_old) ---
    output wire                        fft_w1_valid,
    output wire                        fft_w1_start,
    output wire signed [NB_INT-1:0]    fft_w1_I,
    output wire signed [NB_INT-1:0]    fft_w1_Q
);

    // ============================================================
    // Clock divider
    // ============================================================
    clock_div2 u_div2 (
        .i_clk_fast(clk_fast),
        .i_enable  (enable_div),
        .o_clk_low (clk_low)
    );

    // ============================================================
    // TX  (clk_low)
    // ============================================================
    reg i_en_tx;
    always @(posedge clk_low) begin
        if (rst) i_en_tx <= 1'b0;
        else     i_en_tx <= 1'b1;
    end

    wire signed [WN-1:0] sI_tx, sQ_tx;

    top_tx u_tx (
        .clk   (clk_low),
        .reset (rst),
        .i_en  (i_en_tx),
        .sI_out(sI_tx),
        .sQ_out(sQ_tx)
    );

    assign tx_I_dbg = sI_tx;
    assign tx_Q_dbg = sQ_tx;

    // ============================================================
    // Canal  (clk_low)
    // ============================================================
    wire signed [WN-1:0] sI_ch, sQ_ch;

    top_ch u_ch (
        .clk        (clk_low),
        .rst        (rst),
        .In_I       (sI_tx),
        .In_Q       (sQ_tx),
        .sigma_scale(sigma_scale),
        .Out_I      (sI_ch),
        .Out_Q      (sQ_ch)
    );

    assign ch_I_dbg = sI_ch;
    assign ch_Q_dbg = sQ_ch;

    // ============================================================
    // Ganancia opcional CH -> OS
    // ============================================================
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

    // ============================================================
    // OS Buffer  (clk_low -> clk_fast)
    // ============================================================
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

    // ============================================================
    // FIFO  (clk_fast)
    // ============================================================
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

    // ============================================================
    // FFT  (clk_fast)
    // ============================================================
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

    // ============================================================
    // HISTORY BUFFER  (clk_fast)
    // ============================================================
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

    // ============================================================
    // Wires de la PARTICIÓN 1 (rama de update de W1, fuente X_old)
    // ============================================================
    wire                     xhd_old_valid, xhd_old_start;
    wire signed [NB_INT-1:0] xhd_old_re,    xhd_old_im;
    wire                     grad1_valid,   grad1_start;
    wire signed [NB_INT-1:0] grad1_re,      grad1_im;
    wire                     ifft_grad1_valid, ifft_grad1_start, ifft_grad1_rdy;
    wire signed [NB_INT-1:0] ifft_grad1_I,     ifft_grad1_Q;
    wire                     grad1_t_valid, grad1_t_start;
    wire signed [NB_INT-1:0] grad1_t_I,     grad1_t_Q;
    wire                     lms1_w_valid,  lms1_w_start;
    wire signed [NB_INT-1:0] lms1_w_I,      lms1_w_Q;
    wire                     zpp1_out_valid, zpp1_out_start;
    wire signed [NB_INT-1:0] zpp1_out_wI,    zpp1_out_wQ;
    wire                     fft_w1_rdy;

    // ============================================================
    // Contador wr_cnt para escribir W_new en cmul_pbfdaf
    //
    //   fft_pesos (W0) y fft_pesos1 (W1) emiten en LOCKSTEP:
    //   misma latencia, mismo bin, mismo ciclo. Por eso un único
    //   contador alcanza para indexar AMBOS puertos de escritura.
    //     fft_w_start → cmul_eff_wk=0, luego incrementa con fft_w_valid.
    // ============================================================
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

    // ============================================================
    // CMUL_PBFDAF  (clk_fast)
    //   Y[k] = W0[k]·X_curr[k] + W1[k]·X_old[k]
    //   v15: W0 y W1 se actualizan frame a frame desde fft_pesos/fft_pesos1
    //   Dos puertos de escritura concurrentes (arrays separados).
    // ============================================================
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
        // Puerto W0 — partición X_curr (fft_pesos)
        .i_we0     (fft_w_valid),
        .i_wk0     (cmul_eff_wk),
        .i_W0_re   (fft_w_I),
        .i_W0_im   (fft_w_Q),
        // Puerto W1 — partición X_old (fft_pesos1), lockstep con W0
        .i_we1     (fft_w1_valid),
        .i_wk1     (cmul_eff_wk),
        .i_W1_re   (fft_w1_I),
        .i_W1_im   (fft_w1_Q),
        .o_valid   (cmul_out_valid),
        .o_start   (cmul_out_start),
        .o_yI      (cmul_out_I),
        .o_yQ      (cmul_out_Q),
        .o_samp_idx(cmul_samp_dbg)
    );

    // ============================================================
    // IFFT  (clk_fast)
    // ============================================================
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

    // ============================================================
    // DISCARD_N  (clk_fast)
    //
    //   Recibe el stream IFFT de NFFT=32 muestras por frame.
    //   Descarta los primeros N=16 (transitorio overlap-save).
    //   Emite los últimos N=16 como y_blk: la salida útil del filtro.
    //
    //   Latencia añadida: 1 ciclo (registro de salida).
    //
    //   Próximo en cadena: SLICER_QPSK  (dn_out_* → slicer)
    // ============================================================
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
        .o_samp_idx(/* debug no expuesto */)
    );

    // ============================================================
    // SLICER_QPSK  (u_slicer)
    //
    //   Entrada:  y_blk  = dn_out_*  (las N muestras útiles del frame)
    //   Salida:   yhat   = decisión dura QPSK  →  ±QPSK_A = ±91 en Q(9,7)
    //             e      = yhat - y            →  error para el LMS
    //
    //   Latencia: 1 ciclo (registro de salida).
    // ============================================================
    slicer_qpsk #(
        .NB_W (WN),
        .NBF_W(7),
        .NFFT (NFFT)
    ) u_slicer (
        .clk      (clk_fast),
        .rst      (rst),
        .i_valid  (dn_out_valid),
        .i_start  (dn_out_start),
        .i_yI     (dn_out_I),
        .i_yQ     (dn_out_Q),
        .o_valid  (sl_out_valid),
        .o_start  (sl_out_start),
        .o_yhat_I (sl_out_yhat_I),
        .o_yhat_Q (sl_out_yhat_Q),
        .o_e_I    (sl_out_e_I),
        .o_e_Q    (sl_out_e_Q)
    );

    // ============================================================
    // ZERO_PAD_ERROR  (u_zpe)
    //
    //   Entrada:  e_blk  = sl_out_e_*  (N errores del slicer)
    //   Salida:   frame  = [0...0 | e_blk]  (2N muestras para FFT_ERROR)
    // ============================================================
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

    // ============================================================
    // FFT_ERROR  (u_fft_error)
    //
    //   Entrada:  zpe_out_*  (frame 2N = [0..0 | e_blk])
    //   Salida:   E_k        (espectro del error, único, compartido
    //                         por AMBAS particiones del gradiente)
    // ============================================================
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

    // ============================================================
    // BLOCK_POWER  (u_bpow) — estimación de potencia para NLMS
    //   P = Σ|X|² sobre la salida de la FFT principal (fft_out).
    //   o_mu_extra → shift extra que normaliza el paso de AMBAS
    //   particiones (la potencia del canal es común a las dos).
    //   CALIBRAR P_REF_MSB con el log [BPOW] (ver block_power.v).
    // ============================================================
    wire [4:0] nlms_mu_extra;

    block_power #(
        .NB_W      (NB_INT),
        .NFFT      (NFFT),
        .BETA      (4),
        .P_REF_MSB (30),       // ← CALIBRAR con [BPOW]
        .MU_EXTRA_MAX(8)
    ) u_bpow (
        .clk     (clk_fast),
        .rst     (rst),
        .i_valid (fft_out_valid),
        .i_start (fft_out_start),
        .i_xre   (fft_out_I),
        .i_xim   (fft_out_Q),
        .o_mu_extra(nlms_mu_extra)
    );

    // ============================================================
    // ===================  PARTICIÓN 0  (W0, X_curr)  ============
    // ============================================================

    // ============================================================
    // XHIST_DELAY  (u_xhd) — X_curr retrasado 118 ciclos
    // ============================================================
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

    // ============================================================
    // GRADIENTE  (u_grad) — PHI0 = conj(X_curr) * E_k
    // ============================================================
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

    // ============================================================
    // IFFT_GRAD  (u_ifft_grad) — PHI0_k → phi0(t)
    // ============================================================
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

    // ============================================================
    // PROYECCION  (u_proy) — keep_first_n  [0..N-1]
    // ============================================================
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

    // ============================================================
    // UPDATE_LMS  (u_lms) — partición 0
    //   Inicialización: impulso centrado en tap 15 (INIT_TAP=15, INIT_VAL=1024)
    // ============================================================
    update_lms #(
        .NB_W       (NB_INT),
        .NBF_W      (NBF_INT),
        .N          (NFFT / 2),    // N=16
        .MU_SH_INIT (8),
        .MU_SH_FINAL(12),
        .N_SWITCH   (300),
        .INIT_TAP (7),     // era 15 — sacarlo de la costura P0/P1
        .INIT_VAL (17'sd1024)    // impulso 1.0 en el centro → pre/post-cursor habilitados
    ) u_lms (
        .clk        (clk_fast),
        .rst        (rst),
        .i_valid    (grad_t_valid),
        .i_start    (grad_t_start),
        .i_gI       (grad_t_I),
        .i_gQ       (grad_t_Q),
        .i_mu_extra (nlms_mu_extra),
        .o_valid    (lms_w_valid),
        .o_start    (lms_w_start),
        .o_wI       (lms_w_I),
        .o_wQ       (lms_w_Q),
        .o_switched (lms_switched),
        .o_frame_cnt(lms_frame_cnt)
    );

    // ============================================================
    // ZERO_PAD_PESOS  (u_zpp) — partición 0
    // ============================================================
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

    // ============================================================
    // FFT_PESOS  (u_fft_pesos) — W0 = FFT([w0|0..0])
    // ============================================================
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

    // ============================================================
    // ===================  PARTICIÓN 1  (W1, X_old)  =============
    //   Rama de update espejo de la partición 0, cambiando:
    //     - fuente de X: hb_out_old (en lugar de hb_out_curr)
    //     - destino: puerto W1 del CMUL (i_we1/i_wk1/i_W1_*)
    //     - init de pesos: CERO (INIT_VAL=0)
    //   Mismo DELAY=118: X_old y X_curr salen del HB el mismo ciclo,
    //   ambos deben alinearse con el mismo E_k.
    // ============================================================

    // --- XHIST_DELAY1: X_old retrasado 118 ---
    xhist_delay #(
        .NB_W (NB_INT),
        .DELAY(118)
    ) u_xhd_old (
        .clk    (clk_fast),
        .rst    (rst),
        .i_valid(hb_out_valid),
        .i_start(hb_out_start),
        .i_xre  (hb_out_old_I),
        .i_xim  (hb_out_old_Q),
        .o_valid(xhd_old_valid),
        .o_start(xhd_old_start),
        .o_xre  (xhd_old_re),
        .o_xim  (xhd_old_im)
    );

    // --- GRADIENTE1: PHI1 = conj(X_old) * E_k (mismo E_k) ---
    gradiente #(
        .NB_W (NB_INT),
        .NBF_W(NBF_INT)
    ) u_grad1 (
        .clk    (clk_fast),
        .rst    (rst),
        .i_valid(ffte_out_valid),
        .i_start(ffte_out_start),
        .i_xre  (xhd_old_re),
        .i_xim  (xhd_old_im),
        .i_ere  (ffte_out_I),
        .i_eim  (ffte_out_Q),
        .o_valid(grad1_valid),
        .o_start(grad1_start),
        .o_phi_re(grad1_re),
        .o_phi_im(grad1_im)
    );

    // --- IFFT_GRAD1: PHI1_k → phi1(t) ---
    fft_ifft_stream #(
        .NFFT(NFFT), .LOGN(LOGN),
        .NB_IN(NB_INT),  .NBF_IN(NBF_INT),
        .NB_W(NB_INT),   .NBF_W(NBF_INT),
        .NB_OUT(NB_INT), .NBF_OUT(NBF_INT),
        .BF_SCALE(0),
        .REORDER_BITREV(REORDER_BITREV)
    ) u_ifft_grad1 (
        .i_clk    (clk_fast),
        .i_rst    (rst),
        .i_valid  (grad1_valid),
        .i_start  (grad1_start),
        .i_xI     (grad1_re),
        .i_xQ     (grad1_im),
        .i_inverse(1'b1),
        .o_in_ready(ifft_grad1_rdy),
        .o_start  (ifft_grad1_start),
        .o_valid  (ifft_grad1_valid),
        .o_yI     (ifft_grad1_I),
        .o_yQ     (ifft_grad1_Q)
    );

    // --- PROYECCION1: keep_first_n [0..N-1] ---
    keep_first_n #(
        .NB_W (NB_INT),
        .NBF_W(NBF_INT),
        .NFFT (NFFT)
    ) u_proy1 (
        .clk    (clk_fast),
        .rst    (rst),
        .i_valid(ifft_grad1_valid),
        .i_start(ifft_grad1_start),
        .i_yI   (ifft_grad1_I),
        .i_yQ   (ifft_grad1_Q),
        .o_valid(grad1_t_valid),
        .o_start(grad1_t_start),
        .o_yI   (grad1_t_I),
        .o_yQ   (grad1_t_Q),
        .o_samp_idx()
    );

    // --- UPDATE_LMS1: partición 1, arranca en CERO ---
    update_lms #(
        .NB_W       (NB_INT),
        .NBF_W      (NBF_INT),
        .N          (NFFT / 2),
        .MU_SH_INIT (8),
        .MU_SH_FINAL(12),
        .N_SWITCH   (300),
        .INIT_VAL   (17'sd0)        // ← sin impulso: todos los taps en 0
    ) u_lms1 (
        .clk        (clk_fast),
        .rst        (rst),
        .i_valid    (grad1_t_valid),
        .i_start    (grad1_t_start),
        .i_gI       (grad1_t_I),
        .i_gQ       (grad1_t_Q),
        .i_mu_extra (nlms_mu_extra),
        .o_valid    (lms1_w_valid),
        .o_start    (lms1_w_start),
        .o_wI       (lms1_w_I),
        .o_wQ       (lms1_w_Q),
        .o_switched (),
        .o_frame_cnt()
    );

    // --- ZERO_PAD_PESOS1: [w1|0..0] ---
    zero_pad_pesos #(
        .NB_W (NB_INT),
        .NBF_W(NBF_INT),
        .NFFT (NFFT)
    ) u_zpp1 (
        .clk    (clk_fast),
        .rst    (rst),
        .i_valid(lms1_w_valid),
        .i_start(lms1_w_start),
        .i_wI   (lms1_w_I),
        .i_wQ   (lms1_w_Q),
        .o_valid(zpp1_out_valid),
        .o_start(zpp1_out_start),
        .o_wI   (zpp1_out_wI),
        .o_wQ   (zpp1_out_wQ)
    );

    // --- FFT_PESOS1: W1 = FFT([w1|0..0]) ---
    fft_ifft_stream #(
        .NFFT(NFFT), .LOGN(LOGN),
        .NB_IN (NB_INT), .NBF_IN (NBF_INT),
        .NB_W  (NB_INT), .NBF_W  (NBF_INT),
        .NB_OUT(NB_INT), .NBF_OUT(NBF_INT),
        .BF_SCALE(0),
        .REORDER_BITREV(REORDER_BITREV)
    ) u_fft_pesos1 (
        .i_clk    (clk_fast),
        .i_rst    (rst),
        .i_valid  (zpp1_out_valid),
        .i_start  (zpp1_out_start),
        .i_xI     (zpp1_out_wI),
        .i_xQ     (zpp1_out_wQ),
        .i_inverse(1'b0),
        .o_in_ready(fft_w1_rdy),
        .o_valid  (fft_w1_valid),
        .o_start  (fft_w1_start),
        .o_yI     (fft_w1_I),
        .o_yQ     (fft_w1_Q)
    );

    // ============================================================
    // DEBUG PIPELINE — señales clave para diagnóstico
    // Borrar antes de síntesis
    // ============================================================

    // --- Dominio clk_low: canal y TX ---
    always @(posedge clk_low) begin
        if (i_valid_low)
            $display("[CH_OUT] t=%0t  txI=%0d txQ=%0d  chI=%0d chQ=%0d",
                $time, sI_tx, sQ_tx, sI_ch, sQ_ch);
    end

    // --- Dominio clk_fast: pipeline completo ---
    always @(posedge clk_fast) begin

        // OS buffer → FIFO: primer sample de cada bloque
        if (os_valid && os_start)
            $display("[OS_OUT]   t=%0t  I=%0d Q=%0d",
                $time, os_I, os_Q);

        // FIFO → FFT: primer sample de cada bloque
        if (fft_in_valid && fft_in_start)
            $display("[FFT_IN]   t=%0t  I=%0d Q=%0d",
                $time, fft_in_I, fft_in_Q);

        // FFT salida: primer bin de cada frame
        if (fft_out_valid && fft_out_start)
            $display("[FFT_OUT]  t=%0t  I=%0d Q=%0d",
                $time, fft_out_I, fft_out_Q);

        // HB salida: primer bin
        if (hb_out_valid && hb_out_start)
            $display("[HB_OUT]   t=%0t  currI=%0d currQ=%0d  oldI=%0d oldQ=%0d",
                $time, hb_out_curr_I, hb_out_curr_Q,
                       hb_out_old_I,  hb_out_old_Q);

        // CMUL salida: primer bin
        if (cmul_out_valid && cmul_out_start)
            $display("[CMUL_OUT] t=%0t  I=%0d Q=%0d",
                $time, cmul_out_I, cmul_out_Q);

        // IFFT salida: primer sample
        if (ifft_out_valid && ifft_out_start)
            $display("[IFFT_OUT] t=%0t  I=%0d Q=%0d",
                $time, ifft_out_I, ifft_out_Q);

        // discard_n salida: primer sample útil
        if (dn_out_valid && dn_out_start)
            $display("[DN_OUT]   t=%0t  I=%0d Q=%0d",
                $time, dn_out_I, dn_out_Q);

        // Slicer: yhat y error
        if (sl_out_valid && sl_out_start)
            $display("[SL_START] t=%0t  yhatI=%0d yhatQ=%0d  eI=%0d eQ=%0d",
                $time, sl_out_yhat_I, sl_out_yhat_Q,
                       sl_out_e_I, sl_out_e_Q);

        // FFT_ERROR: primer bin del espectro del error
        if (ffte_out_valid && ffte_out_start)
            $display("[FFTE_OUT] t=%0t  I=%0d Q=%0d",
                $time, ffte_out_I, ffte_out_Q);

        // XHD (P0): primer bin retrasado
        if (xhd_out_valid && xhd_out_start)
            $display("[XHD_OUT]  t=%0t  re=%0d im=%0d",
                $time, xhd_out_re, xhd_out_im);

        // XHD (P1): primer bin X_old retrasado — verificar lockstep con XHD_OUT
        if (xhd_old_valid && xhd_old_start)
            $display("[XHD_OLD]  t=%0t  re=%0d im=%0d",
                $time, xhd_old_re, xhd_old_im);

        // Gradiente P0: primer bin PHI0_k
        if (grad_out_valid && grad_out_start)
            $display("[GRAD_OUT] t=%0t  phiRe=%0d phiIm=%0d",
                $time, grad_out_re, grad_out_im);

        // Gradiente P1: primer bin PHI1_k
        if (grad1_valid && grad1_start)
            $display("[GRAD1]    t=%0t  phiRe=%0d phiIm=%0d",
                $time, grad1_re, grad1_im);

        // LMS P0: peso w0[0]
        if (lms_w_valid && lms_w_start)
            $display("[LMS_W0]   t=%0t  w0=%0d",
                $time, lms_w_I);

        // LMS P1: peso w1[0]
        if (lms1_w_valid && lms1_w_start)
            $display("[LMS_W1]   t=%0t  w1=%0d",
                $time, lms1_w_I);

        // FFT_PESOS P0: primer bin W0 escrito en CMUL
        if (fft_w_valid && fft_w_start)
            $display("[FFT_W]    t=%0t  W0re=%0d W0im=%0d",
                $time, fft_w_I, fft_w_Q);

        // FFT_PESOS P1: primer bin W1 escrito en CMUL — debe coincidir en t con FFT_W
        if (fft_w1_valid && fft_w1_start)
            $display("[FFT_W1]   t=%0t  W1re=%0d W1im=%0d",
                $time, fft_w1_I, fft_w1_Q);

    end

endmodule

`default_nettype wire