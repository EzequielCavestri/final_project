`timescale 1us/1ns
`default_nettype none

// ============================================================
// tb_top_global_all  v14+BER (loop LMS cerrado + contador BER)
//
// Verifica la cadena completa:
//   TX->CH->OS->FIFO->FFT->HB->CMUL->IFFT->DN->SLICER->ZPE->FFT_ERROR
//                           └── xhist_delay ──┘
//                                         GRADIENTE->IFFT_GRAD->PROY
//                                         UPDATE_LMS->ZPP->FFTW->CMUL(W)
//
// BLOQUES DE VERIFICACIÓN
// -----------------------
// [BLOQUE 1]  OS
// [BLOQUE 2]  FIFO
// [BLOQUE 3]  FFT
// [BLOQUE 4]  HB
// [BLOQUE 5]  CMUL
// [BLOQUE 6]  IFFT
// [BLOQUE 7]  DISCARD_N
// [BLOQUE 8]  SLICER_QPSK
// [BLOQUE 9]  ZERO_PAD_ERROR
// [BLOQUE 10] FFT_ERROR
// [BLOQUE 11] XHIST_DELAY
// [BLOQUE 12] GRADIENTE
// [BLOQUE 13] IFFT_GRAD + PROYECCION
// [BLOQUE 14] UPDATE_LMS
// [BLOQUE 15] ZERO_PAD_PESOS
// [BLOQUE 16] FFT_PESOS
// [BLOQUE 17] CONVERGENCIA (err_avg por frame)
// [BLOQUE 18] BER COUNTER  <-- NUEVO
//
// VERIFICACION CLAVE del BLOQUE 11:
//   xhd_out_start y ffte_out_start deben aparecer en el MISMO ciclo.
//   Si hay diferencia, el GRADIENTE va a multiplicar muestras equivocadas.
//
//   X1) xhd_out_start == ffte_out_start ciclo a ciclo (sin desfasaje)
//   X2) xhd_out_valid == ffte_out_valid ciclo a ciclo (misma duración)
//   X3) exactamente NFFT=32 muestras válidas por frame en xhd_out
//   Estrategia — delay line directa (igual que Bloque 7):
//     slicer tiene 1 ciclo de latencia. sl_out[T] == f(dn_out[T-1]).
//     Se registra dn_out 1 ciclo (dn_d1) y se verifica ciclo a ciclo.
//   S1) sl_out_yhat ∈ {+91, -91}
//   S2) signo(sl_out_yhat[T]) == signo(dn_d1[T])
//   S3) sl_out_e[T] == sl_out_yhat[T] - dn_d1[T]  (saturado 9 bits)
//   S4) Exactamente N=16 válidos por frame
//   S5) Exactamente 1 start por frame
// ============================================================

module tb_top_global_all;

    // ============================================================
    // Parámetros
    // ============================================================
    localparam integer WN        = 9;
    localparam integer NFFT      = 32;
    localparam integer N_HALF    = NFFT / 2;   // 16
    localparam integer NB_INT    = 17;
    localparam integer FIFO_AW   = 8;
    localparam integer K_HIST    = 1;

    localparam integer TOL              = 3;
    localparam integer VERBOSE          = 0;
    localparam integer FRAMES_TO_CHECK  = 2000;

    localparam integer QDEPTH = 128;
    localparam integer QMEM   = QDEPTH * NFFT;

    // ============================================================
    // Relojes y reset
    // ============================================================
    reg clk_fast;
    initial begin clk_fast = 1'b0; forever #5 clk_fast = ~clk_fast; end

    reg rst;
    initial begin
        rst = 1'b1;
        repeat(20) @(posedge clk_fast);
        rst = 1'b0;
    end

    reg enable_div;
    initial enable_div = 1'b1;

    reg [10:0] sigma_scale;
    // Opciones:
    //initial sigma_scale = 11'd6;
    initial sigma_scale = 11'd4;   // SNR ~3 dB (ruido fuerte)
    

    // ============================================================
    // Señales DUT
    // ============================================================
    wire clk_low;
    wire signed [WN-1:0]     tx_I_dbg, tx_Q_dbg;
    wire signed [WN-1:0]     ch_I_dbg, ch_Q_dbg;

    wire                     os_overflow, os_start, os_valid;
    wire signed [WN-1:0]     os_I, os_Q;

    wire                     fifo_full, fifo_empty, fifo_overflow;
    wire [FIFO_AW:0]         fifo_count;

    wire                     fft_in_valid, fft_in_start;
    wire signed [WN-1:0]     fft_in_I, fft_in_Q;

    wire                     fft_out_valid, fft_out_start;
    wire signed [NB_INT-1:0] fft_out_I, fft_out_Q;

    wire                     hb_out_valid, hb_out_start;
    wire signed [NB_INT-1:0] hb_out_curr_I, hb_out_curr_Q;
    wire signed [NB_INT-1:0] hb_out_old_I,  hb_out_old_Q;

    wire                     cmul_out_valid, cmul_out_start;
    wire signed [NB_INT-1:0] cmul_out_I, cmul_out_Q;

    wire                     ifft_out_valid, ifft_out_start;
    wire signed [WN-1:0]     ifft_out_I, ifft_out_Q;

    // DISCARD_N (nuevo en v5)
    wire                     dn_out_valid, dn_out_start;
    wire signed [WN-1:0]     dn_out_I, dn_out_Q;

    // SLICER_QPSK (nuevo v6)
    wire                     sl_out_valid, sl_out_start;
    wire signed [WN-1:0]     sl_out_yhat_I, sl_out_yhat_Q;
    wire signed [WN-1:0]     sl_out_e_I, sl_out_e_Q;

    // --- ZERO_PAD_ERROR ---
    wire                     zpe_out_valid, zpe_out_start;
    wire signed [WN-1:0]     zpe_out_eI, zpe_out_eQ;

    // --- FFT_ERROR ---
    wire                          ffte_out_valid, ffte_out_start;
    wire signed [NB_INT-1:0]      ffte_out_I, ffte_out_Q;

    // --- XHIST_DELAY ---
    wire                          xhd_out_valid, xhd_out_start;
    wire signed [NB_INT-1:0]      xhd_out_re, xhd_out_im;
    // --- GRADIENTE ---
    wire                          grad_out_valid, grad_out_start;
    wire signed [NB_INT-1:0]      grad_out_re, grad_out_im;
    // --- IFFT_GRAD ---
    wire                          ifft_grad_valid, ifft_grad_start;
    wire signed [NB_INT-1:0]      ifft_grad_I, ifft_grad_Q;
    // --- PROYECCION ---
    wire                          grad_t_valid, grad_t_start;
    wire signed [NB_INT-1:0]      grad_t_I, grad_t_Q;

    // ============================================================
    // DUT
    // ============================================================
    top_global_all #(
        .N_OS   (16),
        .WN     (WN),
        .FIFO_AW(FIFO_AW),
        .NFFT   (NFFT),
        .NB_INT (NB_INT),
        .K_HIST (K_HIST)
    ) dut (
        .clk_fast      (clk_fast),
        .rst           (rst),
        .enable_div    (enable_div),
        .sigma_scale   (sigma_scale),
        .clk_low       (clk_low),
        .tx_I_dbg      (tx_I_dbg),
        .tx_Q_dbg      (tx_Q_dbg),
        .ch_I_dbg      (ch_I_dbg),
        .ch_Q_dbg      (ch_Q_dbg),
        .os_overflow   (os_overflow),
        .os_start      (os_start),
        .os_valid      (os_valid),
        .os_I          (os_I),
        .os_Q          (os_Q),
        .fifo_full     (fifo_full),
        .fifo_empty    (fifo_empty),
        .fifo_overflow (fifo_overflow),
        .fifo_count    (fifo_count),
        .fft_in_valid  (fft_in_valid),
        .fft_in_start  (fft_in_start),
        .fft_in_I      (fft_in_I),
        .fft_in_Q      (fft_in_Q),
        .fft_out_valid (fft_out_valid),
        .fft_out_start (fft_out_start),
        .fft_out_I     (fft_out_I),
        .fft_out_Q     (fft_out_Q),
        .hb_out_valid  (hb_out_valid),
        .hb_out_start  (hb_out_start),
        .hb_out_curr_I (hb_out_curr_I),
        .hb_out_curr_Q (hb_out_curr_Q),
        .hb_out_old_I  (hb_out_old_I),
        .hb_out_old_Q  (hb_out_old_Q),
        .cmul_out_valid(cmul_out_valid),
        .cmul_out_start(cmul_out_start),
        .cmul_out_I    (cmul_out_I),
        .cmul_out_Q    (cmul_out_Q),
        .ifft_out_valid(ifft_out_valid),
        .ifft_out_start(ifft_out_start),
        .ifft_out_I    (ifft_out_I),
        .ifft_out_Q    (ifft_out_Q),
        .dn_out_valid  (dn_out_valid),
        .dn_out_start  (dn_out_start),
        .dn_out_I      (dn_out_I),
        .dn_out_Q      (dn_out_Q),
        // SLICER_QPSK (nuevo v6)
        .sl_out_valid  (sl_out_valid),
        .sl_out_start  (sl_out_start),
        .sl_out_yhat_I (sl_out_yhat_I),
        .sl_out_yhat_Q (sl_out_yhat_Q),
        .sl_out_e_I    (sl_out_e_I),
        .sl_out_e_Q    (sl_out_e_Q),
        // --- ZERO_PAD_ERROR ---
        .zpe_out_valid (zpe_out_valid),
        .zpe_out_start (zpe_out_start),
        .zpe_out_eI    (zpe_out_eI),
        .zpe_out_eQ    (zpe_out_eQ),
        // --- FFT_ERROR ---
        .ffte_out_valid (ffte_out_valid),
        .ffte_out_start (ffte_out_start),
        .ffte_out_I     (ffte_out_I),
        .ffte_out_Q     (ffte_out_Q),
        // --- XHIST_DELAY ---
        .xhd_out_valid  (xhd_out_valid),
        .xhd_out_start  (xhd_out_start),
        .xhd_out_re     (xhd_out_re),
        .xhd_out_im     (xhd_out_im),
        // --- GRADIENTE ---
        .grad_out_valid (grad_out_valid),
        .grad_out_start (grad_out_start),
        .grad_out_re    (grad_out_re),
        .grad_out_im    (grad_out_im),
        // --- IFFT_GRAD ---
        .ifft_grad_valid(ifft_grad_valid),
        .ifft_grad_start(ifft_grad_start),
        .ifft_grad_I    (ifft_grad_I),
        .ifft_grad_Q    (ifft_grad_Q),
        // --- PROYECCION ---
        .grad_t_valid   (grad_t_valid),
        .grad_t_start   (grad_t_start),
        .grad_t_I       (grad_t_I),
        .grad_t_Q       (grad_t_Q)
    );

    // ============================================================
    // Funciones auxiliares
    // ============================================================
    function integer iabs;
        input integer v;
        begin iabs = (v < 0) ? -v : v; end
    endfunction

    function integer wrap_inc;
        input integer ptr;
        begin wrap_inc = (ptr == (QDEPTH-1)) ? 0 : (ptr + 1); end
    endfunction

    // ============================================================
    // Contador de ciclos global
    // ============================================================
    integer cyc;
    initial cyc = 0;
    always @(posedge clk_fast) begin
        if (rst) cyc <= 0;
        else     cyc <= cyc + 1;
    end

    // ============================================================
    // BLOQUE 1 — OS Monitor
    // ============================================================
    integer last_os_start_cyc, os_warns;
    initial begin last_os_start_cyc = -1; os_warns = 0; end

    always @(posedge clk_fast) begin
        if (rst) begin last_os_start_cyc <= -1; os_warns <= 0; end
        else if (os_valid && os_start) begin
            if (last_os_start_cyc >= 0 && (cyc - last_os_start_cyc) != NFFT) begin
                os_warns <= os_warns + 1;
                $display("[%0t] WARN OS: periodo=%0d (exp %0d)",
                    $time, cyc-last_os_start_cyc, NFFT);
            end
            last_os_start_cyc <= cyc;
        end
    end

    // ============================================================
    // BLOQUE 2 — FIFO Monitor
    // ============================================================
    integer fifo_max_count;
    initial fifo_max_count = 0;
    always @(posedge clk_fast) begin
        if (rst) fifo_max_count <= 0;
        else if ($signed(fifo_count) > fifo_max_count)
            fifo_max_count <= fifo_count;
    end

    // ============================================================
    // BLOQUE 3 — FFT Monitor
    // ============================================================
    integer fft_frames_out, last_fft_start_cyc, fft_period_warns;
    initial begin fft_frames_out=0; last_fft_start_cyc=-1; fft_period_warns=0; end

    always @(posedge clk_fast) begin
        if (rst) begin fft_frames_out<=0; last_fft_start_cyc<=-1; fft_period_warns<=0; end
        else if (fft_out_valid && fft_out_start) begin
            fft_frames_out <= fft_frames_out + 1;
            if (last_fft_start_cyc >= 0 && (cyc - last_fft_start_cyc) != NFFT) begin
                fft_period_warns <= fft_period_warns + 1;
                $display("[%0t] WARN FFT: periodo=%0d", $time, cyc-last_fft_start_cyc);
            end
            last_fft_start_cyc <= cyc;
        end
    end

    // ============================================================
    // BLOQUE 4 — History Buffer Monitor
    // ============================================================
    localparam integer DLY_DEPTH = K_HIST * NFFT + 1;

    reg signed [NB_INT-1:0] fft_dly_I [0:DLY_DEPTH-1];
    reg signed [NB_INT-1:0] fft_dly_Q [0:DLY_DEPTH-1];
    integer fft_dly_fill;

    reg signed [NB_INT-1:0] fft_q_I [0:QMEM-1];
    reg signed [NB_INT-1:0] fft_q_Q [0:QMEM-1];
    integer fft_wr_ptr, fft_q_count, fft_in_samp;

    integer hb_total_curr_errs, hb_total_old_errs;
    integer hb_frame_errs_curr, hb_frame_errs_old;
    integer hb_frame_total;

    reg signed [NB_INT-1:0] fft_prev_I,  fft_prev_Q;
    reg                      fft_prev_valid;

    integer dly_ii;
    initial begin
        for (dly_ii=0; dly_ii<DLY_DEPTH; dly_ii=dly_ii+1) begin
            fft_dly_I[dly_ii]=0; fft_dly_Q[dly_ii]=0;
        end
        fft_dly_fill=0; fft_wr_ptr=0; fft_q_count=0; fft_in_samp=0;
        hb_total_curr_errs=0; hb_total_old_errs=0;
        hb_frame_errs_curr=0; hb_frame_errs_old=0; hb_frame_total=0;
        fft_prev_valid=0; fft_prev_I=0; fft_prev_Q=0;
    end

    integer sh_i;
    always @(posedge clk_fast) begin : hb_dly
        integer qi_wr;
        if (rst) begin
            fft_dly_fill<=0; fft_wr_ptr<=0; fft_q_count<=0; fft_in_samp<=0;
            fft_prev_valid<=0;
        end else begin
            fft_prev_valid <= fft_out_valid;
            fft_prev_I     <= fft_out_I;
            fft_prev_Q     <= fft_out_Q;

            if (fft_out_valid) begin
                for (sh_i=DLY_DEPTH-1; sh_i>0; sh_i=sh_i-1) begin
                    fft_dly_I[sh_i] <= fft_dly_I[sh_i-1];
                    fft_dly_Q[sh_i] <= fft_dly_Q[sh_i-1];
                end
                fft_dly_I[0] <= fft_out_I;
                fft_dly_Q[0] <= fft_out_Q;
                if (fft_dly_fill < DLY_DEPTH) fft_dly_fill <= fft_dly_fill + 1;

                if (fft_out_start) fft_in_samp <= 0;
                qi_wr = (fft_wr_ptr % QDEPTH)*NFFT + fft_in_samp;
                fft_q_I[qi_wr] <= fft_out_I;
                fft_q_Q[qi_wr] <= fft_out_Q;
                if (fft_in_samp == NFFT-1) begin
                    fft_wr_ptr  <= wrap_inc(fft_wr_ptr);
                    fft_q_count <= (fft_q_count < QDEPTH) ? fft_q_count+1 : fft_q_count;
                    fft_in_samp <= 0;
                end else fft_in_samp <= fft_in_samp + 1;
            end
        end
    end

    always @(posedge clk_fast) begin : hb_chk
    integer dIc, dQc, dIo, dQo;
    if (rst) begin
        hb_total_curr_errs<=0; hb_total_old_errs<=0;
        hb_frame_errs_curr<=0; hb_frame_errs_old<=0; hb_frame_total<=0;
    end else if (hb_out_valid) begin
        if (hb_out_start) begin
            if (hb_frame_total>0) begin
                if (VERBOSE && hb_frame_errs_curr==0 && hb_frame_errs_old==0)
                    $display("[HB][FRAME %0d] PASS  curr_errs=0  old_errs=0",
                        hb_frame_total-1);
                if (hb_frame_errs_curr!=0 || hb_frame_errs_old!=0)
                    $display("[HB][FRAME %0d] FAIL  curr_errs=%0d  old_errs=%0d",
                        hb_frame_total-1, hb_frame_errs_curr, hb_frame_errs_old);
                // ← sin else acá
            end
            hb_frame_errs_curr<=0; hb_frame_errs_old<=0;
            hb_frame_total<=hb_frame_total+1;
        end else begin
            dIc = $signed(hb_out_curr_I) - $signed(fft_dly_I[0]);
            dQc = $signed(hb_out_curr_Q) - $signed(fft_dly_Q[0]);
            if (iabs(dIc)>TOL || iabs(dQc)>TOL) begin
                hb_frame_errs_curr<=hb_frame_errs_curr+1;
                hb_total_curr_errs<=hb_total_curr_errs+1;
            end
            if (fft_dly_fill >= DLY_DEPTH) begin
                dIo = $signed(hb_out_old_I) - $signed(fft_dly_I[K_HIST*NFFT]);
                dQo = $signed(hb_out_old_Q) - $signed(fft_dly_Q[K_HIST*NFFT]);
                if (iabs(dIo)>TOL || iabs(dQo)>TOL) begin
                    hb_frame_errs_old<=hb_frame_errs_old+1;
                    hb_total_old_errs<=hb_total_old_errs+1;
                end
            end
        end
    end
end

    // ============================================================
    // BLOQUE 5 — CMUL Monitor
    // ============================================================
    reg signed [NB_INT-1:0] hb_prev_I, hb_prev_Q;
    reg                      hb_prev_valid, hb_prev_start;
    integer cmul_total_errs, cmul_frames_checked;
    integer cmul_frame_errs, cmul_start_misalign;

    initial begin
        hb_prev_I=0; hb_prev_Q=0; hb_prev_valid=0; hb_prev_start=0;
        cmul_total_errs=0; cmul_frames_checked=0;
        cmul_frame_errs=0; cmul_start_misalign=0;
    end

    always @(posedge clk_fast) begin : cmul_monitor
        integer dIcm, dQcm;
        if (rst) begin
            hb_prev_I<=0; hb_prev_Q<=0; hb_prev_valid<=0; hb_prev_start<=0;
            cmul_total_errs<=0; cmul_frames_checked<=0;
            cmul_frame_errs<=0; cmul_start_misalign<=0;
        end else begin
            hb_prev_valid <= hb_out_valid;
            hb_prev_start <= hb_out_start;
            hb_prev_I     <= hb_out_curr_I;
            hb_prev_Q     <= hb_out_curr_Q;

            if (cmul_out_valid) begin
                if (cmul_out_start && !hb_prev_start) begin
                    cmul_start_misalign <= cmul_start_misalign + 1;
                    $display("[CMUL][WARN] start desalineado frame %0d", cmul_frames_checked);
                end
                if (cmul_out_start) begin
                    if (cmul_frames_checked>0) begin
                        if (cmul_frame_errs==0)
                            $display("[CMUL][FRAME %0d] PASS  errs=0", cmul_frames_checked-1);
                        else
                            $display("[CMUL][FRAME %0d] FAIL  errs=%0d",
                                cmul_frames_checked-1, cmul_frame_errs);
                    end
                    cmul_frame_errs<=0;
                    cmul_frames_checked<=cmul_frames_checked+1;
                end
                // Comparacion de datos deshabilitada en v14+BER:
                // W != identidad (LMS activo), CMUL ya no debe pasar prueba Y=X.
                // Solo se verifica timing (start alignment y frame count).
            end
        end
    end

    // ============================================================
    // BLOQUE 6 — IFFT identidad check
    // ============================================================
    reg signed [WN-1:0] in_mem_I [0:QMEM-1];
    reg signed [WN-1:0] in_mem_Q [0:QMEM-1];

    integer wr_ptr, rd_ptr, qcount;
    integer in_samp;
    integer checking, out_samp;
    integer checked_frames, frame_errs, total_errs;
    integer idx_in, idx_out, dI, dQ;

    initial begin
        wr_ptr=0; rd_ptr=0; qcount=0; in_samp=0;
        checking=0; out_samp=0;
        checked_frames=0; frame_errs=0; total_errs=0;
    end

    always @(posedge clk_fast) begin
        if (rst) begin wr_ptr<=0; qcount<=0; in_samp<=0; end
        else if (fft_in_valid) begin
            idx_in = fft_in_start ? 0 : in_samp;
            in_mem_I[wr_ptr*NFFT + idx_in] <= fft_in_I;
            in_mem_Q[wr_ptr*NFFT + idx_in] <= fft_in_Q;
            if (idx_in == (NFFT-1)) begin
                if (qcount < QDEPTH) begin
                    wr_ptr <= wrap_inc(wr_ptr);
                    qcount <= qcount + 1;
                end else begin
                    $display("[%0t] ERROR: input queue overflow", $time); $finish;
                end
                in_samp <= 0;
            end else in_samp <= idx_in + 1;
        end
    end

    always @(posedge clk_fast) begin
        if (rst) begin
            rd_ptr<=0; checking<=0; out_samp<=0;
            checked_frames<=0; frame_errs<=0; total_errs<=0;
        end else if (ifft_out_valid) begin
            idx_out = ifft_out_start ? 0 : out_samp;
            if (ifft_out_start) begin
                if (checking) begin
                    if (frame_errs==0)
                        $display("[IFFT][FRAME %0d] PASS", checked_frames-1);
                    else
                        $display("[IFFT][FRAME %0d] FAIL errs=%0d", checked_frames-1, frame_errs);
                end
                if (checked_frames < FRAMES_TO_CHECK) begin
                    if (qcount==0) begin checking<=0; end
                    else begin checking<=1; frame_errs<=0; checked_frames<=checked_frames+1; end
                end else checking<=0;
            end
            if (checking) begin
                dI = $signed(ifft_out_I) - $signed(in_mem_I[rd_ptr*NFFT + idx_out]);
                dQ = $signed(ifft_out_Q) - $signed(in_mem_Q[rd_ptr*NFFT + idx_out]);
                if (iabs(dI)>TOL || iabs(dQ)>TOL) begin
                    frame_errs<=frame_errs+1; total_errs<=total_errs+1;
                end
                if (idx_out==(NFFT-1)) begin
                    rd_ptr<=wrap_inc(rd_ptr); qcount<=qcount-1; out_samp<=0;
                end else out_samp<=idx_out+1;
            end else begin
                if (idx_out==(NFFT-1)) out_samp<=0; else out_samp<=idx_out+1;
            end
        end
    end

    // ============================================================
    // BLOQUE 7 — DISCARD_N Monitor
    //
    // Verificaciones:
    //   P1) Exactamente N_HALF=16 salidas válidas por frame
    //   P2) Exactamente 1 pulso dn_out_start por frame
    //   P4) dn_out[j] == ifft_out[N_HALF+j]  (discard_n no modifica el dato)
    //
    // Estrategia — delay line directa sobre ifft_out (sin for en always):
    //
    //   discard_n tiene 1 ciclo de latencia (registro de salida).
    //   Por tanto:  dn_out en ciclo T  ==  ifft_out en ciclo T-1
    //
    //   Se registra ifft_out un ciclo: ifft_d1_I/Q.
    //   Se registra también ifft_out_valid/start un ciclo: ifft_d1_valid/start.
    //
    //   Luego, en cada ciclo donde dn_out_valid=1, se compara:
    //     dn_out_I/Q  vs  ifft_d1_I/Q   (valor de 1 ciclo atrás)
    //
    //   Esta comparación es directa ciclo a ciclo, sin necesidad de
    //   ningún snapshot ni for-loop. La lógica de P1/P2 cuenta
    //   válidos y starts por frame usando dn_out_start como separador.
    //
    //   Condición de arranque: solo comparar cuando el delay line está
    //   caliente (dn_d1_armed=1, se activa al primer dn_out_valid).
    // ============================================================

    // Delay 1 ciclo de ifft_out (para alinear con dn_out)
    reg signed [WN-1:0] ifft_d1_I, ifft_d1_Q;
    reg                  ifft_d1_valid, ifft_d1_start;

    always @(posedge clk_fast) begin
        if (rst) begin
            ifft_d1_I     <= {WN{1'b0}};
            ifft_d1_Q     <= {WN{1'b0}};
            ifft_d1_valid <= 1'b0;
            ifft_d1_start <= 1'b0;
        end else begin
            ifft_d1_I     <= ifft_out_I;
            ifft_d1_Q     <= ifft_out_Q;
            ifft_d1_valid <= ifft_out_valid;
            ifft_d1_start <= ifft_out_start;
        end
    end

    // Contadores DISCARD_N
    integer dn_total_errs;
    integer dn_frames_checked;
    integer dn_frame_errs;
    integer dn_frame_valid_cnt;
    integer dn_frame_start_cnt;
    integer dn_p1_errs;
    integer dn_p2_errs;
    integer dn_p4_errs;
    integer dn_armed;       // 1 tras el primer dn_out_valid (delay caliente)

    integer dI_dn, dQ_dn;

    initial begin
        dn_total_errs=0; dn_frames_checked=0; dn_frame_errs=0;
        dn_frame_valid_cnt=0; dn_frame_start_cnt=0;
        dn_p1_errs=0; dn_p2_errs=0; dn_p4_errs=0;
        dn_armed=0;
    end

    always @(posedge clk_fast) begin : dn_monitor
        if (rst) begin
            dn_total_errs<=0; dn_frames_checked<=0; dn_frame_errs<=0;
            dn_frame_valid_cnt<=0; dn_frame_start_cnt<=0;
            dn_p1_errs<=0; dn_p2_errs<=0; dn_p4_errs<=0;
            dn_armed<=0;
        end else begin

            // Armar el delay tras el primer dn_out_valid
            if (dn_out_valid) dn_armed <= 1;

            // ---- Conteo, reset y comparación — un solo if/else para evitar
            //      conflicto de non-blocking en el ciclo de start.
            //
            //      Cuando dn_out_start=1 (= primer válido del nuevo frame):
            //        · Reportar frame anterior
            //        · Resetear contadores a 1 (ya cuenta esta muestra)
            //        · Comparar P4 normalmente
            //
            //      Cuando dn_out_valid=1 y NO es start:
            //        · Incrementar contadores
            //        · Comparar P4
            // ----
            if (dn_out_valid) begin

                if (dn_out_start) begin
                    // -- Reporte del frame anterior --
                    if (dn_frames_checked > 0) begin
                        if (dn_frame_valid_cnt !== N_HALF) begin
                            dn_p1_errs    <= dn_p1_errs + 1;
                            dn_total_errs <= dn_total_errs + 1;
                            $display("[DN][FRAME %0d] FAIL P1: valid_cnt=%0d exp=%0d",
                                dn_frames_checked-1, dn_frame_valid_cnt, N_HALF);
                        end
                        if (dn_frame_start_cnt !== 1) begin
                            dn_p2_errs    <= dn_p2_errs + 1;
                            dn_total_errs <= dn_total_errs + 1;
                            $display("[DN][FRAME %0d] FAIL P2: start_cnt=%0d exp=1",
                                dn_frames_checked-1, dn_frame_start_cnt);
                        end
                        if (dn_frame_errs == 0)
                            $display("[DN][FRAME %0d] PASS  valid=%0d  errs=0",
                                dn_frames_checked-1, dn_frame_valid_cnt);
                        else
                            $display("[DN][FRAME %0d] FAIL  valid=%0d  errs=%0d",
                                dn_frames_checked-1, dn_frame_valid_cnt, dn_frame_errs);
                    end
                    // -- Reset contadores: start ya cuenta como 1 válido y 1 start --
                    dn_frame_valid_cnt <= 1;
                    dn_frame_start_cnt <= 1;
                    dn_frame_errs      <= 0;
                    dn_frames_checked  <= dn_frames_checked + 1;

                end else begin
                    // -- Muestra normal: solo incrementar --
                    dn_frame_valid_cnt <= dn_frame_valid_cnt + 1;
                end

                // -- P4: comparar dato (aplica en start y en el resto) --
                if (dn_armed) begin
                    dI_dn = $signed(dn_out_I) - $signed(ifft_d1_I);
                    dQ_dn = $signed(dn_out_Q) - $signed(ifft_d1_Q);
                    if (iabs(dI_dn) > 0 || iabs(dQ_dn) > 0) begin
                        dn_frame_errs <= dn_frame_errs + 1;
                        dn_p4_errs    <= dn_p4_errs + 1;
                        dn_total_errs <= dn_total_errs + 1;
                        $display("[DN][FRAME %0d] FAIL P4: dn_I=%0d dn_Q=%0d ifft_d1_I=%0d ifft_d1_Q=%0d",
                            dn_frames_checked-1,
                            $signed(dn_out_I), $signed(dn_out_Q),
                            $signed(ifft_d1_I), $signed(ifft_d1_Q));
                    end
                end

            end
        end
    end

    // ============================================================
    // BLOQUE 8 — SLICER_QPSK Monitor  (NUEVO en v6)
    //
    // Estrategia — delay line directa sobre dn_out:
    //   slicer_qpsk tiene 1 ciclo de latencia (registro de salida).
    //   sl_out en ciclo T  ==  f(dn_out en ciclo T-1)
    //   Se registra dn_out 1 ciclo → dn_d1_I/Q.
    //
    // Propiedades verificadas ciclo a ciclo cuando sl_out_valid=1:
    //   S1) sl_out_yhat ∈ {+91, -91}
    //   S2) signo(sl_out_yhat[T]) == signo(dn_d1[T])
    //   S3) sl_out_e[T] == sl_out_yhat[T] - dn_d1[T]  (saturado NB_W bits)
    //
    // Propiedades de frame (usando sl_out_start como separador):
    //   S4) Exactamente N=16 válidos por frame
    //   S5) Exactamente 1 start por frame
    // ============================================================

    localparam signed [WN-1:0] QPSK_A     =  9'sd91;
    localparam signed [WN-1:0] QPSK_A_NEG = -9'sd91;
    localparam integer SL_SAT_MAX =  (1 << (WN-1)) - 1;  //  255
    localparam integer SL_SAT_MIN = -(1 << (WN-1));       // -256

    // Delay 1 ciclo de dn_out
    reg signed [WN-1:0] dn_d1_I, dn_d1_Q;
    reg                  dn_d1_valid;

    // Variables auxiliares del monitor SLICER (nivel modulo — XSim no soporta reg en named blocks)
    reg signed [WN+1:0] exp_e_ext_I, exp_e_ext_Q;
    reg signed [WN-1:0] exp_e_I, exp_e_Q;
    reg signed [WN-1:0] exp_yhat_I, exp_yhat_Q;

    always @(posedge clk_fast) begin
        if (rst) begin
            dn_d1_I     <= {WN{1'b0}};
            dn_d1_Q     <= {WN{1'b0}};
            dn_d1_valid <= 1'b0;
        end else begin
            dn_d1_I     <= dn_out_I;
            dn_d1_Q     <= dn_out_Q;
            dn_d1_valid <= dn_out_valid;
        end
    end

    // Contadores SLICER
    integer sl_total_errs;
    integer sl_frames_checked;
    integer sl_frame_errs;
    integer sl_frame_valid_cnt;
    integer sl_frame_start_cnt;
    integer sl_p1_errs;   // S4: valid count
    integer sl_p2_errs;   // S5: start count
    integer sl_p3_errs;   // S1+S2: yhat fuera de rango o signo incorrecto
    integer sl_p4_errs;   // S3: error incorrecto
    integer sl_armed;     // 1 tras el primer sl_out_valid

    integer dI_sl, dQ_sl;

    initial begin
        sl_total_errs=0; sl_frames_checked=0; sl_frame_errs=0;
        sl_frame_valid_cnt=0; sl_frame_start_cnt=0;
        sl_p1_errs=0; sl_p2_errs=0; sl_p3_errs=0; sl_p4_errs=0;
        sl_armed=0;
    end

    always @(posedge clk_fast) begin : sl_monitor
        if (rst) begin
            sl_total_errs<=0; sl_frames_checked<=0; sl_frame_errs<=0;
            sl_frame_valid_cnt<=0; sl_frame_start_cnt<=0;
            sl_p1_errs<=0; sl_p2_errs<=0; sl_p3_errs<=0; sl_p4_errs<=0;
            sl_armed<=0;
        end else begin

            if (sl_out_valid) dn_d1_valid <= 1;  // armar delay
            if (sl_out_valid) sl_armed <= 1;

            if (sl_out_valid) begin

                // ---- Separador de frame ----
                if (sl_out_start) begin
                    // Reporte del frame anterior
                    if (sl_frames_checked > 0) begin
                        if (sl_frame_valid_cnt !== N_HALF) begin
                            sl_p1_errs    <= sl_p1_errs + 1;
                            sl_total_errs <= sl_total_errs + 1;
                            $display("[SL][FRAME %0d] FAIL S4: valid_cnt=%0d exp=%0d",
                                sl_frames_checked-1, sl_frame_valid_cnt, N_HALF);
                        end
                        if (sl_frame_start_cnt !== 1) begin
                            sl_p2_errs    <= sl_p2_errs + 1;
                            sl_total_errs <= sl_total_errs + 1;
                            $display("[SL][FRAME %0d] FAIL S5: start_cnt=%0d exp=1",
                                sl_frames_checked-1, sl_frame_start_cnt);
                        end
                        if (sl_frame_errs == 0)
                            $display("[SL][FRAME %0d] PASS  valid=%0d  errs=0",
                                sl_frames_checked-1, sl_frame_valid_cnt);
                        else
                            $display("[SL][FRAME %0d] FAIL  valid=%0d  errs=%0d",
                                sl_frames_checked-1, sl_frame_valid_cnt, sl_frame_errs);
                    end
                    // Reset contadores — la muestra actual ya cuenta como 1
                    sl_frame_valid_cnt <= 1;
                    sl_frame_start_cnt <= 1;
                    sl_frame_errs      <= 0;
                    sl_frames_checked  <= sl_frames_checked + 1;

                end else begin
                    sl_frame_valid_cnt <= sl_frame_valid_cnt + 1;
                end

                // ---- Verificaciones ciclo a ciclo (delay caliente) ----
                if (sl_armed) begin
                    // S1 + S2: yhat debe ser ±91 con signo correcto
                    exp_yhat_I = (dn_d1_I >= 0) ? QPSK_A : QPSK_A_NEG;
                    exp_yhat_Q = (dn_d1_Q >= 0) ? QPSK_A : QPSK_A_NEG;

                    if ($signed(sl_out_yhat_I) !== $signed(exp_yhat_I)) begin
                        sl_frame_errs <= sl_frame_errs + 1;
                        sl_p3_errs    <= sl_p3_errs + 1;
                        sl_total_errs <= sl_total_errs + 1;
                        $display("[SL][FRAME %0d] FAIL S1/S2 I: yhat=%0d exp=%0d (dn_d1_I=%0d)",
                            sl_frames_checked-1,
                            $signed(sl_out_yhat_I), $signed(exp_yhat_I), $signed(dn_d1_I));
                    end
                    if ($signed(sl_out_yhat_Q) !== $signed(exp_yhat_Q)) begin
                        sl_frame_errs <= sl_frame_errs + 1;
                        sl_p3_errs    <= sl_p3_errs + 1;
                        sl_total_errs <= sl_total_errs + 1;
                        $display("[SL][FRAME %0d] FAIL S1/S2 Q: yhat=%0d exp=%0d (dn_d1_Q=%0d)",
                            sl_frames_checked-1,
                            $signed(sl_out_yhat_Q), $signed(exp_yhat_Q), $signed(dn_d1_Q));
                    end

                    // S3: e = yhat - y  (saturado a WN bits)
                    exp_e_ext_I = $signed({{3{exp_yhat_I[WN-1]}}, exp_yhat_I})
                                - $signed({{3{dn_d1_I[WN-1]}},    dn_d1_I});
                    exp_e_ext_Q = $signed({{3{exp_yhat_Q[WN-1]}}, exp_yhat_Q})
                                - $signed({{3{dn_d1_Q[WN-1]}},    dn_d1_Q});

                    exp_e_I = (exp_e_ext_I > SL_SAT_MAX) ?  SL_SAT_MAX[WN-1:0] :
                              (exp_e_ext_I < SL_SAT_MIN) ?  SL_SAT_MIN[WN-1:0] :
                               exp_e_ext_I[WN-1:0];
                    exp_e_Q = (exp_e_ext_Q > SL_SAT_MAX) ?  SL_SAT_MAX[WN-1:0] :
                              (exp_e_ext_Q < SL_SAT_MIN) ?  SL_SAT_MIN[WN-1:0] :
                               exp_e_ext_Q[WN-1:0];

                    dI_sl = iabs($signed(sl_out_e_I) - $signed(exp_e_I));
                    dQ_sl = iabs($signed(sl_out_e_Q) - $signed(exp_e_Q));

                    if (dI_sl > 0 || dQ_sl > 0) begin
                        sl_frame_errs <= sl_frame_errs + 1;
                        sl_p4_errs    <= sl_p4_errs + 1;
                        sl_total_errs <= sl_total_errs + 1;
                        $display("[SL][FRAME %0d] FAIL S3: e_I=%0d exp=%0d  e_Q=%0d exp=%0d",
                            sl_frames_checked-1,
                            $signed(sl_out_e_I), $signed(exp_e_I),
                            $signed(sl_out_e_Q), $signed(exp_e_Q));
                    end
                end
            end
        end
    end

    // ============================================================
    // ============================================================
    // BLOQUE 9 — ZERO_PAD_ERROR Monitor  v2 (delay-line)
    // ============================================================
    //
    // Verificación por delay line fijo:
    //
    //   ZPE recibe e[j] en ciclo A+j (sl_out_valid=1)
    //   ZPE v5 (ping-pong): emite e[j] en ciclo A+32+j
    //   → delay constante = 2N = 32 ciclos
    //
    //   Un shift register de profundidad 32 retrasa sl_out_e exactamente
    //   ese tiempo. En la zona de error, zpe_out debe coincidir con
    //   e_dly[ZPE_DELAY] siempre que e_dly_v[ZPE_DELAY]=1.
    //
    //   No se necesitan snapshots, buffers ni sincronización manual.
    //
    // Propiedades:
    //   Z1) primeras N salidas del frame = 0
    //   Z2) últimas N salidas = e_dly[ZPE_DELAY] (sl_out_e retrasado 32 ciclos)
    //   Z3) exactamente 1 start por frame
    //   Z4) exactamente 2N muestras válidas por frame
    // ============================================================

    localparam integer ZPE_DELAY = 2*N_HALF + 1;  // 33

    // Shift register para sl_out_e (desplaza cada ciclo de reloj)
    reg signed [WN-1:0] e_dly_I [0:ZPE_DELAY];
    reg signed [WN-1:0] e_dly_Q [0:ZPE_DELAY];
    reg                 e_dly_v [0:ZPE_DELAY];
    integer             zpe_dly_k;

    always @(posedge clk_fast) begin : zpe_delay_line
        integer k;
        if (rst) begin
            for (k=0; k<=ZPE_DELAY; k=k+1) begin
                e_dly_I[k] <= 0;
                e_dly_Q[k] <= 0;
                e_dly_v[k] <= 0;
            end
        end else begin
            e_dly_I[0] <= sl_out_e_I;
            e_dly_Q[0] <= sl_out_e_Q;
            e_dly_v[0] <= sl_out_valid;
            for (k=1; k<=ZPE_DELAY; k=k+1) begin
                e_dly_I[k] <= e_dly_I[k-1];
                e_dly_Q[k] <= e_dly_Q[k-1];
                e_dly_v[k] <= e_dly_v[k-1];
            end
        end
    end

    // Contadores del monitor
    integer zpe_total_errs;
    integer zpe_frames_checked;
    integer zpe_frame_errs;
    integer zpe_frame_valid_cnt;
    integer zpe_frame_start_cnt;
    integer zpe_z1_errs;
    integer zpe_z2_errs;
    integer zpe_z3_errs;
    integer zpe_z4_errs;
    integer zpe_out_idx;
    integer zpe_armed;

    initial begin
        zpe_total_errs=0; zpe_frames_checked=0; zpe_frame_errs=0;
        zpe_frame_valid_cnt=0; zpe_frame_start_cnt=0;
        zpe_z1_errs=0; zpe_z2_errs=0; zpe_z3_errs=0; zpe_z4_errs=0;
        zpe_out_idx=0; zpe_armed=0;
    end

    always @(posedge clk_fast) begin : zpe_monitor
        if (rst) begin
            zpe_total_errs      <= 0; zpe_frames_checked <= 0;
            zpe_frame_errs      <= 0; zpe_frame_valid_cnt<= 0;
            zpe_frame_start_cnt <= 0; zpe_out_idx        <= 0;
            zpe_z1_errs <= 0; zpe_z2_errs <= 0;
            zpe_z3_errs <= 0; zpe_z4_errs <= 0;
            zpe_armed   <= 0;
        end else begin
            if (zpe_out_valid) begin
                zpe_armed <= 1;

                if (zpe_out_start) begin
                    // Cerrar frame anterior
                    if (zpe_armed) begin
                        if (zpe_frame_valid_cnt !== NFFT) begin
                            zpe_z4_errs    <= zpe_z4_errs + 1;
                            zpe_total_errs <= zpe_total_errs + 1;
                            $display("[ZPE][FRAME %0d] FAIL Z4: valid_cnt=%0d exp=%0d",
                                zpe_frames_checked, zpe_frame_valid_cnt, NFFT);
                        end
                        if (zpe_frame_start_cnt !== 1) begin
                            zpe_z3_errs    <= zpe_z3_errs + 1;
                            zpe_total_errs <= zpe_total_errs + 1;
                            $display("[ZPE][FRAME %0d] FAIL Z3: start_cnt=%0d exp=1",
                                zpe_frames_checked, zpe_frame_start_cnt);
                        end
                        if (zpe_frame_errs == 0)
                            $display("[ZPE][FRAME %0d] PASS  valid=%0d  errs=0",
                                zpe_frames_checked, zpe_frame_valid_cnt);
                        else
                            $display("[ZPE][FRAME %0d] FAIL  errs=%0d",
                                zpe_frames_checked, zpe_frame_errs);
                        zpe_frames_checked <= zpe_frames_checked + 1;
                    end
                    zpe_frame_valid_cnt  <= 1;
                    zpe_frame_start_cnt  <= 1;
                    zpe_frame_errs       <= 0;
                    zpe_out_idx          <= 1;
                end else begin
                    zpe_frame_valid_cnt <= zpe_frame_valid_cnt + 1;
                    zpe_out_idx         <= zpe_out_idx + 1;
                end

                // Z1: primera mitad debe ser 0
                if (zpe_out_idx < N_HALF) begin
                    if ($signed(zpe_out_eI) !== 0 || $signed(zpe_out_eQ) !== 0) begin
                        zpe_frame_errs <= zpe_frame_errs + 1;
                        zpe_z1_errs    <= zpe_z1_errs + 1;
                        zpe_total_errs <= zpe_total_errs + 1;
                        $display("[ZPE][FRAME %0d] FAIL Z1: idx=%0d eI=%0d eQ=%0d (exp 0)",
                            zpe_frames_checked, zpe_out_idx,
                            $signed(zpe_out_eI), $signed(zpe_out_eQ));
                    end
                end

                // Z2: segunda mitad = sl_out_e retrasado ZPE_DELAY ciclos
                if (zpe_out_idx >= N_HALF && zpe_out_idx < NFFT && e_dly_v[ZPE_DELAY]) begin
                    if ($signed(zpe_out_eI) !== $signed(e_dly_I[ZPE_DELAY]) ||
                        $signed(zpe_out_eQ) !== $signed(e_dly_Q[ZPE_DELAY])) begin
                        zpe_frame_errs <= zpe_frame_errs + 1;
                        zpe_z2_errs    <= zpe_z2_errs + 1;
                        zpe_total_errs <= zpe_total_errs + 1;
                        $display("[ZPE][FRAME %0d] FAIL Z2: idx=%0d eI=%0d exp=%0d  eQ=%0d exp=%0d",
                            zpe_frames_checked, zpe_out_idx - N_HALF,
                            $signed(zpe_out_eI), $signed(e_dly_I[ZPE_DELAY]),
                            $signed(zpe_out_eQ), $signed(e_dly_Q[ZPE_DELAY]));
                    end
                end

            end
        end
    end

    // ============================================================
    // ============================================================
    // BLOQUE 10 — FFT_ERROR Monitor  (NUEVO en v8)
    // ============================================================
    //
    // La FFT_ERROR es un reuso de fft_ifft_stream con i_inverse=0.
    // No tenemos referencia matemática del resultado, así que solo
    // verificamos estructura y timing:
    //
    //   F1) exactamente 2N muestras válidas por frame
    //   F2) exactamente 1 start por frame
    //   F3) o_valid continuo dentro del frame (sin gaps)
    //
    // Estrategia: igual que el monitor de la FFT principal (BLOQUE 3)
    // ============================================================

    integer ffte_total_errs;
    integer ffte_frames_checked;
    integer ffte_frame_errs;
    integer ffte_frame_valid_cnt;
    integer ffte_frame_start_cnt;
    integer ffte_f1_errs;
    integer ffte_f2_errs;
    integer ffte_f3_errs;
    reg     ffte_armed;
    reg     ffte_prev_valid;

    initial begin
        ffte_total_errs=0; ffte_frames_checked=0; ffte_frame_errs=0;
        ffte_frame_valid_cnt=0; ffte_frame_start_cnt=0;
        ffte_f1_errs=0; ffte_f2_errs=0; ffte_f3_errs=0;
        ffte_armed=0; ffte_prev_valid=0;
    end

    always @(posedge clk_fast) begin : ffte_monitor
        if (rst) begin
            ffte_total_errs     <= 0; ffte_frames_checked <= 0;
            ffte_frame_errs     <= 0; ffte_frame_valid_cnt<= 0;
            ffte_frame_start_cnt<= 0;
            ffte_f1_errs <= 0; ffte_f2_errs <= 0; ffte_f3_errs <= 0;
            ffte_armed   <= 0; ffte_prev_valid <= 0;
        end else begin

            if (ffte_out_valid) begin
                ffte_armed <= 1;

                if (ffte_out_start) begin
                    // Cerrar frame anterior
                    if (ffte_armed) begin
                        // F1: conteo válidos
                        if (ffte_frame_valid_cnt !== NFFT) begin
                            ffte_f1_errs    <= ffte_f1_errs + 1;
                            ffte_total_errs <= ffte_total_errs + 1;
                            $display("[FFTE][FRAME %0d] FAIL F1: valid_cnt=%0d exp=%0d",
                                ffte_frames_checked, ffte_frame_valid_cnt, NFFT);
                        end
                        // F2: start count
                        if (ffte_frame_start_cnt !== 1) begin
                            ffte_f2_errs    <= ffte_f2_errs + 1;
                            ffte_total_errs <= ffte_total_errs + 1;
                            $display("[FFTE][FRAME %0d] FAIL F2: start_cnt=%0d exp=1",
                                ffte_frames_checked, ffte_frame_start_cnt);
                        end
                        if (ffte_frame_errs == 0)
                            $display("[FFTE][FRAME %0d] PASS  valid=%0d  errs=0",
                                ffte_frames_checked, ffte_frame_valid_cnt);
                        else
                            $display("[FFTE][FRAME %0d] FAIL  errs=%0d",
                                ffte_frames_checked, ffte_frame_errs);
                        ffte_frames_checked <= ffte_frames_checked + 1;
                    end
                    ffte_frame_valid_cnt  <= 1;
                    ffte_frame_start_cnt  <= 1;
                    ffte_frame_errs       <= 0;
                end else begin
                    ffte_frame_valid_cnt <= ffte_frame_valid_cnt + 1;
                    // F3: gap detection — si el ciclo anterior no tenía valid y no es start
                    if (ffte_armed && !ffte_prev_valid) begin
                        ffte_frame_errs <= ffte_frame_errs + 1;
                        ffte_f3_errs    <= ffte_f3_errs + 1;
                        ffte_total_errs <= ffte_total_errs + 1;
                        $display("[FFTE][FRAME %0d] FAIL F3: gap detectado en valid",
                            ffte_frames_checked);
                    end
                end
            end

            ffte_prev_valid <= ffte_out_valid;
        end
    end


    // ============================================================
    // ============================================================
    // BLOQUE 11 — XHIST_DELAY Monitor  v2 (NUEVO en v9)
    // ============================================================
    //
    // Verificación del sincronismo entre xhd_out y ffte_out.
    //
    // IMPORTANTE — por qué xhd_valid ≠ ffte_valid entre frames:
    //   HB emite frames CONTINUOS (32 ciclos, sin gaps)
    //   FFTE tiene gaps entre frames (ZPE tiene fase RECV=16 ciclos)
    //   → Durante esos 16 ciclos de gap: xhd_valid=1 pero ffte_valid=0
    //   → Esto es NORMAL y esperado — el GRADIENTE solo opera cuando
    //     ffte_valid=1, así que esos ciclos extra de xhd se ignoran
    //
    // Propiedades verificadas:
    //
    //   X1) SINCRONISMO DE START (la más importante):
    //       Cuando ffte_start=1 → xhd_start debe ser 1 TAMBIÉN
    //       Si ffte_start=1 pero xhd_start=0 → desfasaje → ajustar DELAY
    //       (Los arranques de xhd sin ffte son normales — son los frames
    //        extra del HB continuo, se ignoran)
    //
    //   X2) XHD ACTIVO CUANDO FFTE ACTIVO:
    //       Cuando ffte_valid=1 → xhd_valid debe ser 1
    //       Si ffte_valid=1 pero xhd_valid=0 → problema grave
    //       (El caso xhd=1 cuando ffte=0 es normal y no se reporta)
    //
    //   X3) CONTEO DE MUESTRAS POR FRAME FFTE:
    //       Cuando ffte emite un frame, xhd debe emitir exactamente
    //       NFFT=32 muestras válidas al mismo tiempo
    //
    // Cómo interpretar los resultados:
    //   X1=0, X2=0 → GRADIENTE puede implementarse  ✓
    //   X1>0       → ajustar DELAY en top_global.v
    //   X2>0       → problema grave, xhd no alcanza cuando ffte necesita
    // ============================================================

    integer xhd_total_errs;
    integer xhd_frames_checked;
    integer xhd_frame_errs;
    integer xhd_frame_valid_cnt;
    integer xhd_x1_errs;
    integer xhd_x2_errs;
    integer xhd_x3_errs;
    reg     xhd_armed;

    initial begin
        xhd_total_errs=0; xhd_frames_checked=0; xhd_frame_errs=0;
        xhd_frame_valid_cnt=0;
        xhd_x1_errs=0; xhd_x2_errs=0; xhd_x3_errs=0;
        xhd_armed=0;
    end

    always @(posedge clk_fast) begin : xhd_monitor
        if (rst) begin
            xhd_total_errs    <= 0; xhd_frames_checked <= 0;
            xhd_frame_errs    <= 0; xhd_frame_valid_cnt<= 0;
            xhd_x1_errs <= 0; xhd_x2_errs <= 0; xhd_x3_errs <= 0;
            xhd_armed   <= 0;
        end else begin

            // ---- X1: cuando ffte dispara, xhd debe disparar también ----
            // (No reportamos cuando xhd dispara sin ffte — eso es normal)
            if (ffte_out_start && !xhd_out_start) begin
                xhd_x1_errs    <= xhd_x1_errs + 1;
                xhd_frame_errs <= xhd_frame_errs + 1;
                xhd_total_errs <= xhd_total_errs + 1;
                $display("[XHD] FAIL X1: ffte_start=1 pero xhd_start=0 — DELAY demasiado grande");
            end

            // ---- X2: cuando ffte es válido, xhd debe ser válido también ----
            // (No reportamos cuando ffte=0 y xhd=1 — eso es el gap de ZPE, normal)
            if (ffte_out_valid && !xhd_out_valid) begin
                xhd_x2_errs    <= xhd_x2_errs + 1;
                xhd_frame_errs <= xhd_frame_errs + 1;
                xhd_total_errs <= xhd_total_errs + 1;
                $display("[XHD] FAIL X2: ffte_valid=1 pero xhd_valid=0 — problema grave");
            end

            // ---- X3: contar muestras simultáneas (ffte && xhd válidos) ----
            if (ffte_out_valid) begin
                xhd_armed <= 1;

                if (ffte_out_start) begin
                    // Cerrar frame anterior
                    if (xhd_armed) begin
                        if (xhd_frame_valid_cnt !== NFFT) begin
                            xhd_x3_errs    <= xhd_x3_errs + 1;
                            xhd_total_errs <= xhd_total_errs + 1;
                            $display("[XHD][FRAME %0d] FAIL X3: muestras_simultáneas=%0d exp=%0d",
                                xhd_frames_checked, xhd_frame_valid_cnt, NFFT);
                        end
                        if (xhd_frame_errs == 0)
                            $display("[XHD][FRAME %0d] PASS  muestras_sync=%0d  start_sync=OK",
                                xhd_frames_checked, xhd_frame_valid_cnt);
                        else
                            $display("[XHD][FRAME %0d] FAIL  errs=%0d",
                                xhd_frames_checked, xhd_frame_errs);
                        xhd_frames_checked <= xhd_frames_checked + 1;
                    end
                    xhd_frame_valid_cnt <= 1;
                    xhd_frame_errs      <= 0;
                end else begin
                    xhd_frame_valid_cnt <= xhd_frame_valid_cnt + 1;
                end
            end

        end
    end

    // ============================================================
    // Declaraciones BLOQUE 12 — deben ir antes del RESUMEN FINAL
    // ============================================================
    integer grad_total_errs;
    integer grad_frames_checked;
    integer grad_frame_valid_cnt;
    integer grad_frame_errs;
    integer grad_g1_errs;
    integer grad_g2_errs;
    integer grad_g3_errs;
    integer grad_g4_errs;
    integer grad_armed;
    integer grad_samp_cnt;

    reg signed [16:0] xr_d1, xr_d2;
    reg signed [16:0] xi_d1, xi_d2;
    reg signed [16:0] er_d1, er_d2;
    reg signed [16:0] ei_d1, ei_d2;
    reg               fv_d1, fv_d2;
    reg               fs_d1, fs_d2;

    reg signed [33:0] gref_p_rr, gref_p_ii, gref_p_ri, gref_p_ir;
    reg signed [24:0] gref_sum_re, gref_sum_im;
    reg signed [16:0] gref_phi_re, gref_phi_im;

    // ============================================================
    // Declaraciones BLOQUE 13 — antes de los always que las usan
    // ============================================================
    integer ig_total_errs,      ig_frames_checked;
    integer ig_frame_valid_cnt, ig_frame_errs;
    integer ig_i1_errs, ig_i2_errs, ig_armed;

    integer proy_total_errs,      proy_frames_checked;
    integer proy_frame_valid_cnt, proy_frame_errs;
    integer proy_p1_errs, proy_p2_errs, proy_armed;

    // ============================================================
    // ============================================================
    // Declaraciones BLOQUE 18 - BER con buffer diagnostico
    // ============================================================
    // Para encontrar el offset correcto, guardamos los primeros
    // 200 bits del slicer en un buffer y correlacionamos con TX.
    // ============================================================
    // ============================================================
    // BER — ventana unica post-convergencia
    // ============================================================
    // Metodologia:
    //   N_ADAPT frames descartados (adaptacion del LMS)
    //   luego medicion sobre FRAMES_TO_CHECK - N_ADAPT frames
    //   Un solo BER por corrida → curva BER vs Es/N0 limpia
    //
    // Para cambiar el punto de la curva: solo cambiar sigma_scale
    // ============================================================
    localparam integer N_ADAPT         = 200;  // frames descartados (convergencia LMS)
    localparam integer BER_TX_NBUF     = 33000;
    localparam integer BER_PIPE_DELAY  = 9;    // medido: tx_wr - sym_cnt al final
    localparam integer BER_RX_DIAG     = 2000; // bits para correlacion del lag

    // Buffer TX
    reg  tx_ber_I [0:BER_TX_NBUF-1];
    reg  tx_ber_Q [0:BER_TX_NBUF-1];
    integer tx_ber_wr;

    // Buffer RX diagnostico (primeros BER_RX_DIAG simbolos POST-adaptacion)
    reg  rx_diag_I [0:BER_RX_DIAG-1];
    reg  rx_diag_Q [0:BER_RX_DIAG-1];

    // Contador global de simbolos slicer
    integer ber_sym_cnt;

    // Acumulador BER — una sola ventana post-convergencia
    integer ber_bits, ber_ei, ber_eq;

    // Variables para correlacion y calculo
    integer ber_tx_idx;
    integer b_lag, b_k, b_errs_i, b_best_lag, b_best_errs;
    integer b_best_lag_q, b_best_errs_q;

    // Deteccion posedge clk_low
    reg  ber_clk_low_d;
    wire ber_clklow_r = clk_low & ~ber_clk_low_d;

    // Demapper: MSB del simbolo decidido = bit original
    wire ber_rx_I = sl_out_yhat_I[WN-1];
    wire ber_rx_Q = sl_out_yhat_Q[WN-1];

    // RESUMEN FINAL  — disparado cuando IFFT termina sus 30 frames
    // ============================================================
    always @(posedge clk_fast) begin
        // Esperar que IFFT principal, IFFT_GRAD y PROYECCION hayan procesado frames
        if (!rst && ifft_out_valid && checked_frames == FRAMES_TO_CHECK
            && out_samp == (NFFT-1)
            && ig_frames_checked >= 5
            && proy_frames_checked >= 5) begin

            if (frame_errs==0)
                $display("[IFFT][FRAME %0d] PASS", checked_frames-1);
            else
                $display("[IFFT][FRAME %0d] FAIL errs=%0d", checked_frames-1, frame_errs);

            $display("");
            $display("========================================");
            $display("[TB] RESUMEN FINAL  v14+BER (loop LMS cerrado + contador BER)");
            $display("========================================");
            $display("[OS]   %s  overflow=%0d  period_warns=%0d",
                (os_overflow||os_warns!=0) ? "FAIL":"PASS", os_overflow, os_warns);
            $display("[FIFO] %s  overflow=%0d  max_count=%0d (~%0d frames)",
                fifo_overflow?"FAIL":"PASS",
                fifo_overflow, fifo_max_count, (fifo_max_count+NFFT-1)/NFFT);
            $display("[FFT]  frames_out=%0d  period_warns=%0d  %s",
                fft_frames_out, fft_period_warns,
                fft_period_warns?"WARN":"OK");
            $display("[HB]   frames_checked=%0d  curr_errs=%0d  old_errs=%0d  => %s",
                hb_frame_total, hb_total_curr_errs, hb_total_old_errs,
                (hb_total_curr_errs==0&&hb_total_old_errs==0)?"PASS":"FAIL");
            $display("[CMUL] frames_checked=%0d  misalign=%0d  => %s (data check OFF: W!=I esperado)",
                cmul_frames_checked, cmul_start_misalign,
                (cmul_start_misalign==0)?"PASS: timing OK (W0 adaptivo desde LMS)":"FAIL: start desalineado");
            $display("[IFFT] checked_frames=%0d  total_errs=%0d  tol=%0d  => %s",
                FRAMES_TO_CHECK, total_errs, TOL,
                total_errs==0?"PASS":"FAIL (esperado: W!=I, filtrado activo)");
            $display("[DN]   frames_checked=%0d  total_errs=%0d  => %s",
                dn_frames_checked, dn_total_errs,
                dn_total_errs==0 ? "PASS: dn_out=ifft[N..2N-1]" : "FAIL");
            $display("  [DN] P1(valid_cnt) errs=%0d  P2(start_cnt) errs=%0d  P4(datos) errs=%0d",
                dn_p1_errs, dn_p2_errs, dn_p4_errs);
            $display("[SL]   frames_checked=%0d  total_errs=%0d  => %s",
                sl_frames_checked, sl_total_errs,
                sl_total_errs==0 ? "PASS: yhat=+-91  e=yhat-y" : "FAIL");
            $display("  [SL] S4(valid_cnt) errs=%0d  S5(start_cnt) errs=%0d  S1/S2(yhat) errs=%0d  S3(error) errs=%0d",
                sl_p1_errs, sl_p2_errs, sl_p3_errs, sl_p4_errs);
            $display("[ZPE]  frames_checked=%0d  total_errs=%0d  => %s",
                zpe_frames_checked, zpe_total_errs,
                zpe_total_errs==0 ? "PASS: [0..0|e_blk] OK" : "FAIL");
            $display("  [ZPE] Z1(ceros) errs=%0d  Z2(datos) errs=%0d  Z3(start) errs=%0d  Z4(valid) errs=%0d",
                zpe_z1_errs, zpe_z2_errs, zpe_z3_errs, zpe_z4_errs);
            $display("[FFTE] frames_checked=%0d  total_errs=%0d  => %s",
                ffte_frames_checked, ffte_total_errs,
                ffte_total_errs==0 ? "PASS: FFT_ERROR timing OK" : "FAIL");
            $display("  [FFTE] F1(valid_cnt) errs=%0d  F2(start_cnt) errs=%0d  F3(gaps) errs=%0d",
                ffte_f1_errs, ffte_f2_errs, ffte_f3_errs);
            $display("[XHD]  frames_checked=%0d  total_errs=%0d  => %s",
                xhd_frames_checked, xhd_total_errs,
                xhd_total_errs==0 ? "PASS: xhist_delay sincronizado con ffte_out" : "FAIL — ajustar DELAY");
            $display("  [XHD] X1(start_sync) errs=%0d  X2(valid_sync) errs=%0d  X3(valid_cnt) errs=%0d",
                xhd_x1_errs, xhd_x2_errs, xhd_x3_errs);
            $display("  [XHD] INTERPRETACION:");
            if (xhd_x1_errs == 0 && xhd_x2_errs == 0)
                $display("        X1=0 X2=0 => GRADIENTE puede implementarse");
            else if (xhd_x1_errs > 0)
                $display("        X1>0 => start desincronizado, ajustar DELAY en xhist_delay.v");
            else
                $display("        X2>0 => valid desincronizado, revisar cadena de valid");
            $display("[GRAD] frames_checked=%0d  total_errs=%0d  => %s",
                grad_frames_checked, grad_total_errs,
                grad_total_errs==0 ? "PASS: gradiente OK — PHI=conj(X)*E verificado" : "FAIL");
            $display("  [GRAD] G1(valid_cnt) errs=%0d  G2(start_cnt) errs=%0d  G3(latencia) errs=%0d  G4(math) errs=%0d",
                grad_g1_errs, grad_g2_errs, grad_g3_errs, grad_g4_errs);
            $display("[IFFT_GRAD] frames_checked=%0d  total_errs=%0d  => %s",
                ig_frames_checked, ig_total_errs,
                ig_total_errs==0 ? "PASS: IFFT_GRAD timing OK (32 valid/frame)" : "FAIL");
            $display("  [IFFT_GRAD] I1(valid_cnt) errs=%0d  I2(start_cnt) errs=%0d",
                ig_i1_errs, ig_i2_errs);
            $display("[PROY] frames_checked=%0d  total_errs=%0d  => %s",
                proy_frames_checked, proy_total_errs,
                proy_total_errs==0 ? "PASS: grad_t = phi_t[N..2N-1] (16 valid/frame)" : "FAIL");
            $display("  [PROY] P1(valid_cnt) errs=%0d  P2(start_cnt) errs=%0d",
                proy_p1_errs, proy_p2_errs);
            // ---- BER post-convergencia (ventana unica) ----
            begin
                // Correlacion I para confirmar lag
                b_best_errs = BER_RX_DIAG + 1;
                b_best_lag  = 0;
                for (b_lag = 0; b_lag < 200; b_lag = b_lag + 1) begin
                    b_errs_i = 0;
                    for (b_k = 0; b_k < BER_RX_DIAG; b_k = b_k + 1) begin
                        if ((N_ADAPT*N_HALF + b_k + b_lag) < tx_ber_wr) begin
                            if (rx_diag_I[b_k] !== tx_ber_I[N_ADAPT*N_HALF + b_k - b_lag])
                                b_errs_i = b_errs_i + 1;
                        end
                    end
                    if (b_errs_i < b_best_errs) begin
                        b_best_errs = b_errs_i;
                        b_best_lag  = b_lag;
                    end
                end

                $display("[BER]  sigma_scale=%0d  sym_cnt=%0d  tx_wr=%0d  PIPE_DELAY=%0d",
                    sigma_scale, ber_sym_cnt, tx_ber_wr, BER_PIPE_DELAY);
                $display("[BER]  Corr lag=%0d  errs=%0d/%0d  BER=0.%03d (confirma offset)",
                    b_best_lag, b_best_errs, BER_RX_DIAG,
                    (b_best_errs*1000)/BER_RX_DIAG);
                $display("[BER]  ventana=[frame %0d..%0d]  bits=%0d",
                    N_ADAPT, FRAMES_TO_CHECK-1, ber_bits);

                // Resultado final
                if (ber_bits > 0) begin
                    $display("[BER]  errs_I=%0d  errs_Q=%0d  BER=0.%03d (x1e-3)",
                        ber_ei, ber_eq,
                        ((ber_ei + ber_eq)*1000) / ber_bits);
                    // Mostrar BER por canal también
                    $display("[BER]  BER_I=0.%03d  BER_Q=0.%03d",
                        (ber_ei*1000) / (ber_bits/2),
                        (ber_eq*1000) / (ber_bits/2));
                end else
                    $display("[BER]  sin datos — aumentar FRAMES_TO_CHECK");
            end
            $display("========================================");
            $finish;
        end
    end

    // ============================================================
    // ============================================================
    // BLOQUE 12 — GRADIENTE Monitor  (NUEVO en v10)
    // ============================================================
    //
    //   G1) exactamente NFFT=32 muestras válidas por frame
    //   G2) exactamente 1 start por frame
    //   G3) latencia exacta = 2 ciclos desde ffte_out_start
    //   G4) verificación matemática: PHI_re/im coincide con modelo
    //
    // Para cada frame imprime los primeros 4 samples:
    //   [GRAD][Fn][s=k] Xr=.. Xi=.. Er=.. Ei=.. | ref=(re,im) hw=(re,im) OK/MISMATCH
    // ============================================================

    initial begin
        grad_total_errs=0; grad_frames_checked=0;
        grad_frame_valid_cnt=0; grad_frame_errs=0;
        grad_g1_errs=0; grad_g2_errs=0;
        grad_g3_errs=0; grad_g4_errs=0;
        grad_armed=0; grad_samp_cnt=0;
        xr_d1=0; xr_d2=0; xi_d1=0; xi_d2=0;
        er_d1=0; er_d2=0; ei_d1=0; ei_d2=0;
        fv_d1=0; fv_d2=0; fs_d1=0; fs_d2=0;
    end

    always @(posedge clk_fast) begin : grad_monitor
        if (rst) begin
            grad_total_errs      <= 0; grad_frames_checked <= 0;
            grad_frame_valid_cnt <= 0; grad_frame_errs     <= 0;
            grad_g1_errs <= 0; grad_g2_errs <= 0;
            grad_g3_errs <= 0; grad_g4_errs <= 0;
            grad_armed   <= 0; grad_samp_cnt <= 0;
            xr_d1<=0; xr_d2<=0; xi_d1<=0; xi_d2<=0;
            er_d1<=0; er_d2<=0; ei_d1<=0; ei_d2<=0;
            fv_d1<=0; fv_d2<=0; fs_d1<=0; fs_d2<=0;
        end else begin

            // ---- Pipeline de entradas (2 ciclos) ----
            xr_d1 <= xhd_out_re;    xr_d2 <= xr_d1;
            xi_d1 <= xhd_out_im;    xi_d2 <= xi_d1;
            er_d1 <= ffte_out_I;    er_d2 <= er_d1;
            ei_d1 <= ffte_out_Q;    ei_d2 <= ei_d1;
            fv_d1 <= ffte_out_valid; fv_d2 <= fv_d1;
            fs_d1 <= ffte_out_start; fs_d2 <= fs_d1;

            // ---- G3: latencia exacta = 2 ciclos ----
            if (fs_d2 && !grad_out_start) begin
                grad_g3_errs    <= grad_g3_errs + 1;
                grad_frame_errs <= grad_frame_errs + 1;
                grad_total_errs <= grad_total_errs + 1;
                $display("[GRAD] FAIL G3: ffte_start hace 2 ciclos pero grad_start=0 — latencia rota");
            end

            // ---- G4 + display matemático ----
            if (fv_d2) begin
                // Calcular referencia con entradas de hace 2 ciclos
                // PHI = conj(X)*E:  PHI_re = Xr*Er + Xi*Ei
                //                   PHI_im = Xr*Ei - Xi*Er
                gref_p_rr = $signed(xr_d2) * $signed(er_d2);  // Xr*Er
                gref_p_ii = $signed(xi_d2) * $signed(ei_d2);  // Xi*Ei
                gref_p_ri = $signed(xr_d2) * $signed(ei_d2);  // Xr*Ei
                gref_p_ir = $signed(xi_d2) * $signed(er_d2);  // Xi*Er

                // Truncar a Q17.10: tomar bits [33:10] = dividir por 1024
                gref_sum_re = $signed(gref_p_rr[33:10]) + $signed(gref_p_ii[33:10]);
                gref_sum_im = $signed(gref_p_ri[33:10]) - $signed(gref_p_ir[33:10]);

                // Saturar a 17 bits con signo
                if      (gref_sum_re >  65535) gref_phi_re = 17'sd65535;
                else if (gref_sum_re < -65536) gref_phi_re = -17'sd65536;
                else                            gref_phi_re = gref_sum_re[16:0];

                if      (gref_sum_im >  65535) gref_phi_im = 17'sd65535;
                else if (gref_sum_im < -65536) gref_phi_im = -17'sd65536;
                else                            gref_phi_im = gref_sum_im[16:0];

                // Imprimir primeros 4 samples del frame
                if (grad_samp_cnt < 4) begin
                    $display("[GRAD][F%0d][s=%0d] Xr=%0d Xi=%0d Er=%0d Ei=%0d | ref=(%0d,%0d) hw=(%0d,%0d) %s",
                        grad_frames_checked, grad_samp_cnt,
                        $signed(xr_d2), $signed(xi_d2),
                        $signed(er_d2), $signed(ei_d2),
                        $signed(gref_phi_re), $signed(gref_phi_im),
                        $signed(grad_out_re),  $signed(grad_out_im),
                        (gref_phi_re === $signed(grad_out_re) &&
                         gref_phi_im === $signed(grad_out_im)) ? "OK" : "MISMATCH");
                end

                // Verificar coincidencia matemática
                if (gref_phi_re !== $signed(grad_out_re) ||
                    gref_phi_im !== $signed(grad_out_im)) begin
                    grad_g4_errs    <= grad_g4_errs + 1;
                    grad_frame_errs <= grad_frame_errs + 1;
                    grad_total_errs <= grad_total_errs + 1;
                end
            end

            // ---- G1 y G2: contar válidos y starts por frame ----
            if (grad_out_valid) begin
                grad_armed <= 1;
                if (grad_out_start) begin
                    // Cerrar frame anterior
                    if (grad_armed) begin
                        if (grad_frame_valid_cnt !== NFFT) begin
                            grad_g1_errs    <= grad_g1_errs + 1;
                            grad_total_errs <= grad_total_errs + 1;
                            $display("[GRAD][FRAME %0d] FAIL G1: valid_cnt=%0d exp=%0d",
                                grad_frames_checked, grad_frame_valid_cnt, NFFT);
                        end
                        if (grad_frame_errs == 0)
                            $display("[GRAD][FRAME %0d] PASS  valid=%0d  latencia=2  start=OK",
                                grad_frames_checked, grad_frame_valid_cnt);
                        else
                            $display("[GRAD][FRAME %0d] FAIL  errs=%0d",
                                grad_frames_checked, grad_frame_errs);
                        grad_frames_checked <= grad_frames_checked + 1;
                    end
                    grad_frame_valid_cnt <= 1;
                    grad_frame_errs      <= 0;
                    grad_samp_cnt        <= 0;  // reset contador display al inicio del frame
                end else begin
                    grad_frame_valid_cnt <= grad_frame_valid_cnt + 1;
                    grad_samp_cnt        <= grad_samp_cnt + 1;
                end
            end

        end
    end

    // Timeout
    initial begin
        @(negedge rst);
        #3_000_000;
        $display("[TB] TIMEOUT  checked_frames=%0d  ig_frames=%0d  proy_frames=%0d",
            checked_frames, ig_frames_checked, proy_frames_checked);
        $finish;
    end

    // ============================================================
    // BLOQUE 13a — IFFT_GRAD Monitor
    // ============================================================
    //
    //   IFFT_GRAD = reuso de fft_ifft_stream con i_inverse=1
    //   Entrada: grad_out (PHI_k, 32 muestras/frame frecuencia)
    //   Salida:  ifft_grad (phi(t), 32 muestras/frame tiempo)
    //
    //   I1) exactamente NFFT=32 muestras válidas por frame
    //   I2) exactamente 1 start por frame
    //   I3) o_valid continuo dentro del frame (sin gaps)
    // ============================================================
    initial begin
        ig_total_errs=0; ig_frames_checked=0;
        ig_frame_valid_cnt=0; ig_frame_errs=0;
        ig_i1_errs=0; ig_i2_errs=0; ig_armed=0;
    end

    always @(posedge clk_fast) begin : ig_monitor
        if (rst) begin
            ig_total_errs      <= 0; ig_frames_checked  <= 0;
            ig_frame_valid_cnt <= 0; ig_frame_errs      <= 0;
            ig_i1_errs <= 0; ig_i2_errs <= 0; ig_armed <= 0;
        end else begin
            if (ifft_grad_valid) begin
                ig_armed <= 1;

                if (ifft_grad_start) begin
                    // Cerrar frame anterior
                    if (ig_armed) begin
                        if (ig_frame_valid_cnt !== NFFT) begin
                            ig_i1_errs    <= ig_i1_errs + 1;
                            ig_total_errs <= ig_total_errs + 1;
                            $display("[IFFT_GRAD][FRAME %0d] FAIL I1: valid_cnt=%0d exp=%0d",
                                ig_frames_checked, ig_frame_valid_cnt, NFFT);
                        end
                        if (ig_frame_errs == 0)
                            $display("[IFFT_GRAD][FRAME %0d] PASS  valid=%0d",
                                ig_frames_checked, ig_frame_valid_cnt);
                        else
                            $display("[IFFT_GRAD][FRAME %0d] FAIL  errs=%0d",
                                ig_frames_checked, ig_frame_errs);
                        ig_frames_checked <= ig_frames_checked + 1;
                    end
                    ig_frame_valid_cnt <= 1;
                    ig_frame_errs      <= 0;
                end else begin
                    ig_frame_valid_cnt <= ig_frame_valid_cnt + 1;
                end
            end
        end
    end

    // ============================================================
    // BLOQUE 13b — PROYECCION Monitor
    // ============================================================
    //
    //   PROYECCION = discard_n aplicado a phi(t)
    //   Descarta primeras N=16 muestras, emite las últimas N=16
    //   = grad_t: el gradiente temporal útil para el LMS
    //
    //   P1) exactamente N_HALF=16 muestras válidas por frame
    //   P2) exactamente 1 start por frame
    // ============================================================
    initial begin
        proy_total_errs=0; proy_frames_checked=0;
        proy_frame_valid_cnt=0; proy_frame_errs=0;
        proy_p1_errs=0; proy_p2_errs=0; proy_armed=0;
    end

    always @(posedge clk_fast) begin : proy_monitor
        if (rst) begin
            proy_total_errs      <= 0; proy_frames_checked  <= 0;
            proy_frame_valid_cnt <= 0; proy_frame_errs      <= 0;
            proy_p1_errs <= 0; proy_p2_errs <= 0; proy_armed <= 0;
        end else begin
            if (grad_t_valid) begin
                proy_armed <= 1;

                if (grad_t_start) begin
                    // Cerrar frame anterior
                    if (proy_armed) begin
                        if (proy_frame_valid_cnt !== N_HALF) begin
                            proy_p1_errs    <= proy_p1_errs + 1;
                            proy_total_errs <= proy_total_errs + 1;
                            $display("[PROY][FRAME %0d] FAIL P1: valid_cnt=%0d exp=%0d",
                                proy_frames_checked, proy_frame_valid_cnt, N_HALF);
                        end
                        if (proy_frame_errs == 0)
                            $display("[PROY][FRAME %0d] PASS  valid=%0d",
                                proy_frames_checked, proy_frame_valid_cnt);
                        else
                            $display("[PROY][FRAME %0d] FAIL  errs=%0d",
                                proy_frames_checked, proy_frame_errs);
                        proy_frames_checked <= proy_frames_checked + 1;
                    end
                    proy_frame_valid_cnt <= 1;
                    proy_frame_errs      <= 0;
                end else begin
                    proy_frame_valid_cnt <= proy_frame_valid_cnt + 1;
                end
            end
        end
    end

    // ============================================================
    // ============================================================
    // ============================================================
    // BLOQUE 18 — BER ventana unica post-convergencia
    // ============================================================
    // Simbolos [N_ADAPT*N_HALF .. FRAMES_TO_CHECK*N_HALF):
    //   descarta la adaptacion inicial, mide en regimen

    integer ber_init_j;
    initial begin
        tx_ber_wr  = 0;
        ber_sym_cnt = 0;
        ber_bits   = 0;
        ber_ei     = 0;
        ber_eq     = 0;
        ber_tx_idx = 0;
        b_best_lag = 0; b_best_errs = 0;
        ber_clk_low_d = 1'b0;
        for (ber_init_j = 0; ber_init_j < BER_TX_NBUF; ber_init_j = ber_init_j + 1)
            begin tx_ber_I[ber_init_j] = 1'b0; tx_ber_Q[ber_init_j] = 1'b0; end
        for (ber_init_j = 0; ber_init_j < BER_RX_DIAG; ber_init_j = ber_init_j + 1)
            begin rx_diag_I[ber_init_j] = 1'b0; rx_diag_Q[ber_init_j] = 1'b0; end
    end

    // Captura TX
    always @(posedge clk_fast) begin : ber_clk_det
        if (rst) ber_clk_low_d <= 1'b0;
        else     ber_clk_low_d <= clk_low;
    end

    always @(posedge clk_fast) begin : ber_tx_cap
        if (rst) begin
            tx_ber_wr <= 0;
        end else if (ber_clklow_r && tx_ber_wr < BER_TX_NBUF) begin
            tx_ber_I[tx_ber_wr] <= tx_I_dbg[WN-1];
            tx_ber_Q[tx_ber_wr] <= tx_Q_dbg[WN-1];
            tx_ber_wr <= tx_ber_wr + 1;
        end
    end

    // Captura RX y acumulacion BER
    always @(posedge clk_fast) begin : ber_count
        if (rst) begin
            ber_sym_cnt <= 0;
            ber_bits    <= 0;
            ber_ei      <= 0;
            ber_eq      <= 0;
        end else if (sl_out_valid) begin
            // Buffer diagnostico: primeros BER_RX_DIAG simbolos post-adaptacion
            if (ber_sym_cnt >= N_ADAPT*N_HALF &&
                ber_sym_cnt <  N_ADAPT*N_HALF + BER_RX_DIAG) begin
                rx_diag_I[ber_sym_cnt - N_ADAPT*N_HALF] <= ber_rx_I;
                rx_diag_Q[ber_sym_cnt - N_ADAPT*N_HALF] <= ber_rx_Q;
            end

            // Ventana de medicion: simbolos [N_ADAPT*N_HALF .. fin)
            ber_tx_idx = ber_sym_cnt - BER_PIPE_DELAY;
            if (ber_sym_cnt >= N_ADAPT*N_HALF &&
                ber_tx_idx >= 0 && ber_tx_idx < BER_TX_NBUF) begin
                ber_bits <= ber_bits + 2;
                if (ber_rx_I !== tx_ber_I[ber_tx_idx]) ber_ei <= ber_ei + 1;
                if (ber_rx_Q !== tx_ber_Q[ber_tx_idx]) ber_eq <= ber_eq + 1;
            end
            ber_sym_cnt <= ber_sym_cnt + 1;
        end
    end
    // ============================================================
    // fin BLOQUE 18
    // ============================================================

endmodule

`default_nettype wire
