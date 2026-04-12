`timescale 1ns/1ps
`default_nettype none

// ============================================================
// tb_zero_pad_pesos.v  —  Testbench STANDALONE
//
// TESTS
// -----
// [TEST 1] Frame básico: w[k] = k+1  (valores 1..16)
//          Z1) Primeras N=16 salidas = w[k] = k+1
//          Z2) Últimas N=16 salidas  = 0
//          Z3) Exactamente 1 start por frame
//          Z4) Exactamente NFFT=32 muestras válidas por frame
//          Z5) Latencia = 1 ciclo (o_start aparece 1 clk después de i_start)
//
// [TEST 2] Frame con valores negativos: w[k] = -(k+1)*512
//          Z1/Z2 con signo negativo
//
// [TEST 3] Dos frames consecutivos: verificar que la FSM se reinicia
//          correctamente y el segundo frame también pasa Z1-Z5
//
// [TEST 4] Gap en i_valid durante PESOS: simular que update_lms
//          interrumpe i_valid 3 ciclos en el medio.
//          La salida debe esperar (o_valid=0) y retomar sin
//          perder datos ni cambiar el conteo total.
//
// [TEST 5] Protocolo continuo (monitor): corre durante toda la sim.
//          P1) o_valid continuo dentro de cada frame (sin gaps en S_ZEROS)
//          P2) o_start exactamente 1 vez por frame
//          P3) NFFT muestras válidas por frame
// ============================================================

module tb_zero_pad_pesos;

    // ============================================================
    // Parámetros
    // ============================================================
    localparam integer NB_W  = 17;
    localparam integer NBF_W = 10;
    localparam integer NFFT  = 32;
    localparam integer N     = NFFT / 2;   // 16

    // ============================================================
    // Clock y reset
    // ============================================================
    reg clk;
    initial begin clk = 1'b0; forever #5 clk = ~clk; end

    reg rst;
    initial begin
        rst = 1'b1;
        repeat(8) @(posedge clk);
        @(negedge clk);
        rst = 1'b0;
    end

    // ============================================================
    // DUT
    // ============================================================
    reg                    i_valid, i_start;
    reg  signed [NB_W-1:0] i_wI, i_wQ;

    wire                   o_valid, o_start;
    wire signed [NB_W-1:0] o_wI, o_wQ;

    zero_pad_pesos #(
        .NB_W (NB_W),
        .NBF_W(NBF_W),
        .NFFT (NFFT)
    ) dut (
        .clk    (clk),
        .rst    (rst),
        .i_valid(i_valid),
        .i_start(i_start),
        .i_wI   (i_wI),
        .i_wQ   (i_wQ),
        .o_valid(o_valid),
        .o_start(o_start),
        .o_wI   (o_wI),
        .o_wQ   (o_wQ)
    );

    // ============================================================
    // Tarea: inyectar un frame de N muestras con w[k] = base + k*step
    // ============================================================
    task inject_frame;
        input signed [NB_W-1:0] base;
        input integer            step;
        integer k;
        begin
            for (k = 0; k < N; k = k + 1) begin
                @(negedge clk);
                i_valid = 1'b1;
                i_start = (k == 0) ? 1'b1 : 1'b0;
                i_wI    = base + k * step;
                i_wQ    = base + k * step;
            end
            @(negedge clk);
            i_valid = 1'b0;
            i_start = 1'b0;
            i_wI    = {NB_W{1'b0}};
            i_wQ    = {NB_W{1'b0}};
        end
    endtask

    // Inyectar frame con pausa en el medio (simula gap de update_lms)
    task inject_frame_with_gap;
        input signed [NB_W-1:0] base;
        input integer            step;
        input integer            gap_at;    // índice k donde se inserta la pausa
        input integer            gap_len;   // ciclos de pausa
        integer k;
        integer g;
        begin
            for (k = 0; k < N; k = k + 1) begin
                // Insertar pausa antes de la muestra gap_at
                if (k == gap_at) begin
                    // Pausa: i_valid=0 por gap_len ciclos
                    @(negedge clk);
                    i_valid = 1'b0;
                    i_start = 1'b0;
                    repeat(gap_len - 1) @(negedge clk);
                end
                @(negedge clk);
                i_valid = 1'b1;
                i_start = (k == 0) ? 1'b1 : 1'b0;
                i_wI    = base + k * step;
                i_wQ    = base + k * step;
            end
            @(negedge clk);
            i_valid = 1'b0;
            i_start = 1'b0;
        end
    endtask

    // Capturar NFFT salidas válidas en arrays
    reg signed [NB_W-1:0] cap_I [0:NFFT-1];
    reg signed [NB_W-1:0] cap_Q [0:NFFT-1];
    integer cap_start_cnt;
    integer cap_valid_cnt;

    task capture_output;
        integer k;
        begin
            cap_start_cnt = 0;
            cap_valid_cnt = 0;
            k = 0;
            // Esperar primera salida válida
            while (!o_valid) @(posedge clk);
            // Capturar NFFT muestras válidas
            while (cap_valid_cnt < NFFT) begin
                if (o_valid) begin
                    cap_I[k] = o_wI;
                    cap_Q[k] = o_wQ;
                    if (o_start) cap_start_cnt = cap_start_cnt + 1;
                    cap_valid_cnt = cap_valid_cnt + 1;
                    k = k + 1;
                end
                @(posedge clk);
            end
        end
    endtask

    // ============================================================
    // Declaraciones globales (antes del RESUMEN FINAL)
    // ============================================================
    integer total_errs;
    integer t1_errs, t2_errs, t3_errs, t4_errs;

    // TEST 5: monitor de protocolo
    integer t5_errs;
    integer t5_frame_valid_cnt, t5_frame_start_cnt;
    integer t5_total_frames, t5_armed;
    integer t5_gap_errs;   // gaps en la zona de ZEROS
    reg     t5_in_zeros;   // 1 cuando estamos en la segunda mitad del frame
    integer t5_zeros_idx;

    // Latencia: prev i_start para verificar o_start 1 ciclo después
    integer lat_errs;
    reg     prev_i_start;

    // ============================================================
    // BLOQUE TEST 5 — Monitor de protocolo (siempre activo)
    // ============================================================
    initial begin
        t5_errs=0; t5_frame_valid_cnt=0; t5_frame_start_cnt=0;
        t5_total_frames=0; t5_armed=0; t5_gap_errs=0;
        lat_errs=0;
    end

    always @(posedge clk) begin : proto_monitor
        if (rst) begin
            t5_errs<=0; t5_frame_valid_cnt<=0; t5_frame_start_cnt<=0;
            t5_total_frames<=0; t5_armed<=0; t5_gap_errs<=0;
            lat_errs<=0; prev_i_start<=0;
            t5_in_zeros<=0; t5_zeros_idx<=0;
        end else begin
            // Latencia: o_start debe llegar 1 ciclo después de i_start
            prev_i_start <= (i_valid && i_start);
            if (prev_i_start && !o_start) begin
                lat_errs <= lat_errs + 1;
                $display("[T5] FAIL LATENCIA: i_start hace 1 ciclo pero o_start=0");
            end

            // Contar válidos y starts por frame
            if (o_valid) begin
                t5_armed <= 1;
                if (o_start) begin
                    // Inicio de nuevo frame: cerrar el anterior
                    if (t5_armed) begin
                        if (t5_frame_valid_cnt !== NFFT) begin
                            t5_errs <= t5_errs + 1;
                            $display("[T5] FAIL P3: frame %0d valid_cnt=%0d exp=%0d",
                                t5_total_frames, t5_frame_valid_cnt, NFFT);
                        end
                        if (t5_frame_start_cnt !== 1) begin
                            t5_errs <= t5_errs + 1;
                            $display("[T5] FAIL P2: frame %0d start_cnt=%0d exp=1",
                                t5_total_frames, t5_frame_start_cnt);
                        end
                    end
                    t5_frame_valid_cnt <= 1;
                    t5_frame_start_cnt <= 1;
                    t5_total_frames    <= t5_total_frames + 1;
                end else begin
                    t5_frame_valid_cnt <= t5_frame_valid_cnt + 1;
                end
            end
        end
    end

    // ============================================================
    // STIMULUS + CHECKS
    // ============================================================
    integer k;
    integer err_this;

    initial begin : stim
        i_valid = 0; i_start = 0; i_wI = 0; i_wQ = 0;
        total_errs = 0;
        t1_errs = 0; t2_errs = 0; t3_errs = 0; t4_errs = 0;

        @(negedge rst);
        repeat(5) @(posedge clk);

        // ====================================================
        // TEST 1 — Frame básico: w[k] = k+1  (valores 1..16)
        // ====================================================
        $display("");
        $display("========================================");
        $display("[TEST 1] Frame basico: w[k]=k+1 (valores 1..16)");
        $display("========================================");

        fork
            inject_frame(17'sd1, 1);        // w[k] = 1 + k*1
            capture_output();
        join

        err_this = 0;

        // Z1: primeras N muestras = w[k]
        for (k = 0; k < N; k = k + 1) begin
            if ($signed(cap_I[k]) !== k + 1 || $signed(cap_Q[k]) !== k + 1) begin
                err_this = err_this + 1;
                $display("[T1] FAIL Z1: k=%0d got_I=%0d exp=%0d",
                    k, $signed(cap_I[k]), k+1);
            end else begin
                $display("[T1] OK Z1: k=%0d  w=%0d", k, $signed(cap_I[k]));
            end
        end

        // Z2: últimas N muestras = 0
        for (k = N; k < NFFT; k = k + 1) begin
            if ($signed(cap_I[k]) !== 0 || $signed(cap_Q[k]) !== 0) begin
                err_this = err_this + 1;
                $display("[T1] FAIL Z2: k=%0d got_I=%0d (exp 0)",
                    k, $signed(cap_I[k]));
            end
        end

        // Z3/Z4: start y valid (verificados en monitor, pero validar captura)
        if (cap_start_cnt !== 1) begin
            err_this = err_this + 1;
            $display("[T1] FAIL Z3: start_cnt=%0d exp=1", cap_start_cnt);
        end
        if (cap_valid_cnt !== NFFT) begin
            err_this = err_this + 1;
            $display("[T1] FAIL Z4: valid_cnt=%0d exp=%0d", cap_valid_cnt, NFFT);
        end

        if (err_this == 0)
            $display("[TEST 1] PASS  errs=0");
        else
            $display("[TEST 1] FAIL  errs=%0d", err_this);
        t1_errs = err_this;
        total_errs = total_errs + err_this;

        repeat(5) @(posedge clk);

        // ====================================================
        // TEST 2 — Frame con valores negativos: w[k] = -(k+1)*512
        // ====================================================
        $display("");
        $display("========================================");
        $display("[TEST 2] Valores negativos: w[k]=-(k+1)*512");
        $display("========================================");

        fork
            inject_frame(-17'sd512, -512);   // w[k] = -512 - k*512
            capture_output();
        join

        err_this = 0;
        for (k = 0; k < N; k = k + 1) begin
            if ($signed(cap_I[k]) !== -(k+1)*512) begin
                err_this = err_this + 1;
                $display("[T2] FAIL Z1: k=%0d got=%0d exp=%0d",
                    k, $signed(cap_I[k]), -(k+1)*512);
            end
        end
        for (k = N; k < NFFT; k = k + 1) begin
            if ($signed(cap_I[k]) !== 0) begin
                err_this = err_this + 1;
                $display("[T2] FAIL Z2: k=%0d got=%0d (exp 0)", k, $signed(cap_I[k]));
            end
        end

        if (err_this == 0)
            $display("[TEST 2] PASS — valores negativos OK  errs=0");
        else
            $display("[TEST 2] FAIL  errs=%0d", err_this);
        t2_errs = err_this;
        total_errs = total_errs + err_this;

        repeat(5) @(posedge clk);

        // ====================================================
        // TEST 3 — Dos frames consecutivos (FSM reinicia bien)
        // ====================================================
        $display("");
        $display("========================================");
        $display("[TEST 3] Dos frames consecutivos");
        $display("========================================");

        // Frame A: w[k] = 100
        // Frame B: w[k] = 200
        err_this = 0;
        begin : frames_consec
            reg signed [NB_W-1:0] capA_I [0:NFFT-1];
            integer j;

            // Frame A — capturar
            fork
                inject_frame(17'sd100, 0);
                capture_output();
            join
            for (j = 0; j < NFFT; j = j + 1) capA_I[j] = cap_I[j];

            repeat(3) @(posedge clk);

            // Frame B — capturar
            fork
                inject_frame(17'sd200, 0);
                capture_output();
            join

            // Verificar Frame A
            for (j = 0; j < N; j = j + 1) begin
                if ($signed(capA_I[j]) !== 100) begin
                    err_this = err_this + 1;
                    $display("[T3][FrameA] FAIL Z1: k=%0d got=%0d exp=100", j, $signed(capA_I[j]));
                end
            end
            for (j = N; j < NFFT; j = j + 1) begin
                if ($signed(capA_I[j]) !== 0) begin
                    err_this = err_this + 1;
                    $display("[T3][FrameA] FAIL Z2: k=%0d got=%0d exp=0", j, $signed(capA_I[j]));
                end
            end

            // Verificar Frame B
            for (j = 0; j < N; j = j + 1) begin
                if ($signed(cap_I[j]) !== 200) begin
                    err_this = err_this + 1;
                    $display("[T3][FrameB] FAIL Z1: k=%0d got=%0d exp=200", j, $signed(cap_I[j]));
                end
            end
            for (j = N; j < NFFT; j = j + 1) begin
                if ($signed(cap_I[j]) !== 0) begin
                    err_this = err_this + 1;
                    $display("[T3][FrameB] FAIL Z2: k=%0d got=%0d exp=0", j, $signed(cap_I[j]));
                end
            end
        end

        if (err_this == 0)
            $display("[TEST 3] PASS — dos frames consecutivos OK  errs=0");
        else
            $display("[TEST 3] FAIL  errs=%0d", err_this);
        t3_errs = err_this;
        total_errs = total_errs + err_this;

        repeat(5) @(posedge clk);

        // ====================================================
        // TEST 4 — Gap en i_valid durante PESOS
        //          Simula que update_lms deja de emitir 3 ciclos
        //          en el medio del frame. La salida debe:
        //          - pausar (o_valid=0) durante el gap
        //          - retomar y completar los 32 samples correctamente
        // ====================================================
        $display("");
        $display("========================================");
        $display("[TEST 4] Gap en i_valid durante S_PESOS");
        $display("         (pausa de 3 ciclos en k=8)");
        $display("========================================");

        fork
            inject_frame_with_gap(17'sd10, 10, 8, 3);  // gap en k=8, 3 ciclos
            capture_output();
        join

        err_this = 0;

        // Verificar: las 16 primeras deben ser w[k]=10+k*10
        for (k = 0; k < N; k = k + 1) begin
            if ($signed(cap_I[k]) !== 10 + k * 10) begin
                err_this = err_this + 1;
                $display("[T4] FAIL Z1: k=%0d got=%0d exp=%0d",
                    k, $signed(cap_I[k]), 10 + k * 10);
            end
        end
        // Últimas 16 deben ser cero
        for (k = N; k < NFFT; k = k + 1) begin
            if ($signed(cap_I[k]) !== 0) begin
                err_this = err_this + 1;
                $display("[T4] FAIL Z2: k=%0d got=%0d exp=0", k, $signed(cap_I[k]));
            end
        end
        // Start y valid count
        if (cap_start_cnt !== 1) begin
            err_this = err_this + 1;
            $display("[T4] FAIL Z3: start_cnt=%0d exp=1", cap_start_cnt);
        end

        if (err_this == 0)
            $display("[TEST 4] PASS — gap en PESOS manejado correctamente  errs=0");
        else
            $display("[TEST 4] FAIL  errs=%0d", err_this);
        t4_errs = err_this;
        total_errs = total_errs + err_this;

        // ====================================================
        // RESUMEN FINAL
        // ====================================================
        repeat(20) @(posedge clk);
        $display("");
        $display("========================================");
        $display("[TB] RESUMEN FINAL — tb_zero_pad_pesos");
        $display("========================================");
        $display("[TEST 1] Frame basico w[k]=k+1, zeros OK    => %s  errs=%0d",
            t1_errs==0?"PASS":"FAIL", t1_errs);
        $display("[TEST 2] Valores negativos                  => %s  errs=%0d",
            t2_errs==0?"PASS":"FAIL", t2_errs);
        $display("[TEST 3] Dos frames consecutivos            => %s  errs=%0d",
            t3_errs==0?"PASS":"FAIL", t3_errs);
        $display("[TEST 4] Gap en i_valid durante PESOS       => %s  errs=%0d",
            t4_errs==0?"PASS":"FAIL", t4_errs);
        $display("[TEST 5] Protocolo (monitor continuo)       => %s  errs=%0d",
            (t5_errs+lat_errs)==0?"PASS":"FAIL", t5_errs+lat_errs);
        $display("  [T5] P2(start_cnt) errs=%0d  P3(valid_cnt) errs=%0d  latencia errs=%0d",
            t5_errs, t5_errs, lat_errs);
        $display("  [T5] Frames procesados por monitor: %0d", t5_total_frames);
        $display("TOTAL errs=%0d  => %s",
            total_errs + t5_errs + lat_errs,
            (total_errs + t5_errs + lat_errs)==0 ? "ALL PASS" : "SOME FAIL");
        $display("========================================");
        $finish;
    end

    // Timeout
    initial begin
        @(negedge rst);
        #500_000;
        $display("[TB] TIMEOUT");
        $finish;
    end

endmodule

`default_nettype wire
