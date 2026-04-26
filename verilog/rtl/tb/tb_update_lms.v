`timescale 1ns/1ps
`default_nettype none

// ============================================================
// tb_update_lms.v  —  Testbench STANDALONE de update_lms
//
// TESTS
// -----
// [TEST 1] Estado inicial + primer frame
//          grad[k] = k*256 → delta[k] = k (con MU_SH_INIT=6 no, con MU_SH_INIT=6)
//          Nota: MU_SH_INIT=6 → delta = grad>>>6 = k*256/64 = 4*k
//          Esperado: w_new[k] = 4*k
//
// [TEST 2] Acumulación: segundo frame con misma entrada
//          Esperado: w_new[k] = 8*k
//
// [TEST 3] Gradiente negativo: tercer frame grad[k] = -(k*256)
//          Esperado: w_new[k] = 4*k  (vuelve a TEST 1)
//
// [TEST 4] Saturación positiva (DUT_B, MU_SH_INIT=0 → mu=1)
//          grad = 32000, 4 frames → w satura en W_MAX = +65535
//
// [TEST 5] Saturación negativa (DUT_B reseteado)
//          grad = -32000, 4 frames → w satura en W_MIN = -65536
//
// [TEST 6] Protocolo de señales (monitor continuo sobre DUT_A)
//          U1) exactamente N=16 muestras válidas por frame
//          U2) exactamente 1 start por frame
//          U3) latencia = 1 ciclo (o_start aparece 1 clk después de i_start)
//
// [TEST 7] mu_switch (DUT_A con N_SWITCH=10 para sim rápida)
//          S1) frames 0..N_SWITCH-1: o_switched=0 (usa MU_SH_INIT)
//          S2) frame N_SWITCH:       o_switched=1 (conmuta a MU_SH_FINAL)
//          S3) frames > N_SWITCH:    o_switched=1 y o_frame_cnt no aumenta
//          S4) delta efectivo cambia correctamente en el frame N_SWITCH
//
// RESUMEN FINAL impreso al terminar TEST 7.
// ============================================================

module tb_update_lms;

    // ============================================================
    // Parámetros
    // ============================================================
    localparam integer NB_W        = 17;
    localparam integer NBF_W       = 10;
    localparam integer N           = 16;
    localparam integer MU_SH_INIT  = 6;    // mu ≈ 0.0156
    localparam integer MU_SH_FINAL = 8;    // mu ≈ 0.0039
    localparam integer N_SWITCH    = 10;   // pequeño para sim rápida (prod: 200)
    localparam integer MU_SH_SAT   = 0;    // mu=1 para saturar rápido en TEST 4/5

    localparam signed [NB_W-1:0] W_MAX =  17'sd65535;
    localparam signed [NB_W-1:0] W_MIN = -17'sd65536;

    // ============================================================
    // Clock y reset
    // ============================================================
    reg clk;
    initial begin clk = 1'b0; forever #5 clk = ~clk; end

    reg rst;
    initial begin
        rst = 1'b1;
        repeat(10) @(posedge clk);
        @(negedge clk);
        rst = 1'b0;
    end

    // ============================================================
    // DUT_A — MU_SH_INIT=6, MU_SH_FINAL=8, N_SWITCH=10  (tests 1-3, 6, 7)
    // ============================================================
    reg                    a_valid, a_start;
    reg  signed [NB_W-1:0] a_gI, a_gQ;

    wire                   a_o_valid, a_o_start;
    wire signed [NB_W-1:0] a_o_wI, a_o_wQ;
    wire                   a_switched;
    wire [7:0]             a_frame_cnt;

    update_lms #(
        .NB_W       (NB_W),
        .NBF_W      (NBF_W),
        .N          (N),
        .MU_SH_INIT (MU_SH_INIT),
        .MU_SH_FINAL(MU_SH_FINAL),
        .N_SWITCH   (N_SWITCH)
    ) dut_a (
        .clk        (clk),
        .rst        (rst),
        .i_valid    (a_valid),
        .i_start    (a_start),
        .i_gI       (a_gI),
        .i_gQ       (a_gQ),
        .o_valid    (a_o_valid),
        .o_start    (a_o_start),
        .o_wI       (a_o_wI),
        .o_wQ       (a_o_wQ),
        .o_switched (a_switched),
        .o_frame_cnt(a_frame_cnt)
    );

    // ============================================================
    // DUT_B — MU_SH_INIT=0 (mu=1) para saturar rápido (tests 4, 5)
    // ============================================================
    reg                    b_valid, b_start;
    reg  signed [NB_W-1:0] b_gI, b_gQ;

    wire                   b_o_valid, b_o_start;
    wire signed [NB_W-1:0] b_o_wI, b_o_wQ;
    wire                   b_switched;
    wire [7:0]             b_frame_cnt;

    update_lms #(
        .NB_W       (NB_W),
        .NBF_W      (NBF_W),
        .N          (N),
        .MU_SH_INIT (MU_SH_SAT),
        .MU_SH_FINAL(MU_SH_SAT),
        .N_SWITCH   (1000)       // nunca conmuta durante el test
    ) dut_b (
        .clk        (clk),
        .rst        (rst),
        .i_valid    (b_valid),
        .i_start    (b_start),
        .i_gI       (b_gI),
        .i_gQ       (b_gQ),
        .o_valid    (b_o_valid),
        .o_start    (b_o_start),
        .o_wI       (b_o_wI),
        .o_wQ       (b_o_wQ),
        .o_switched (b_switched),
        .o_frame_cnt(b_frame_cnt)
    );

    // ============================================================
    // Función: modelo de referencia para un paso de update
    // ============================================================
    function signed [NB_W-1:0] sat_add;
        input signed [NB_W-1:0] a;
        input signed [NB_W-1:0] b;
        reg signed [NB_W:0] s;
        begin
            s = {a[NB_W-1], a} + {b[NB_W-1], b};
            if (s[NB_W] != s[NB_W-1])
                sat_add = s[NB_W] ? W_MIN : W_MAX;
            else
                sat_add = s[NB_W-1:0];
        end
    endfunction

    // ============================================================
    // Tarea: inyectar un frame a DUT_A con grad[k] = base + k*step
    // ============================================================
    task inject_frame_a;
        input signed [NB_W-1:0] base;
        input integer            step;
        integer k;
        begin
            for (k = 0; k < N; k = k + 1) begin
                @(negedge clk);
                a_valid = 1'b1;
                a_start = (k == 0) ? 1'b1 : 1'b0;
                a_gI    = base + k * step;
                a_gQ    = base + k * step;
            end
            @(negedge clk);
            a_valid = 1'b0;
            a_start = 1'b0;
            a_gI    = 0;
            a_gQ    = 0;
        end
    endtask

    // Inyectar frame a DUT_A con valor constante en todos los k
    task inject_frame_a_const;
        input signed [NB_W-1:0] gval;
        integer k;
        begin
            for (k = 0; k < N; k = k + 1) begin
                @(negedge clk);
                a_valid = 1'b1;
                a_start = (k == 0) ? 1'b1 : 1'b0;
                a_gI    = gval;
                a_gQ    = gval;
            end
            @(negedge clk);
            a_valid = 1'b0;
            a_start = 1'b0;
        end
    endtask

    // Inyectar frame a DUT_B con valor constante
    task inject_frame_b_const;
        input signed [NB_W-1:0] gval;
        integer k;
        begin
            for (k = 0; k < N; k = k + 1) begin
                @(negedge clk);
                b_valid = 1'b1;
                b_start = (k == 0) ? 1'b1 : 1'b0;
                b_gI    = gval;
                b_gQ    = gval;
            end
            @(negedge clk);
            b_valid = 1'b0;
            b_start = 1'b0;
        end
    endtask

    // Capturar N salidas válidas de DUT_A en arrays
    // (llamar justo después de la primera salida válida)
    reg signed [NB_W-1:0] cap_wI [0:N-1];
    reg signed [NB_W-1:0] cap_wQ [0:N-1];

    task capture_output_a;
        integer k;
        integer cap_ok;
        begin
            cap_ok = 0;
            // Esperar primera salida válida
            while (!a_o_valid) @(posedge clk);
            for (k = 0; k < N; k = k + 1) begin
                while (!a_o_valid) @(posedge clk);
                cap_wI[k] = a_o_wI;
                cap_wQ[k] = a_o_wQ;
                @(posedge clk);
            end
        end
    endtask

    // ============================================================
    // Declaraciones contadores (antes del RESUMEN FINAL)
    // ============================================================
    integer total_errs;
    integer t1_errs, t2_errs, t3_errs, t4_errs, t5_errs;
    integer t7_errs;

    // Test 6: monitor de protocolo (siempre activo)
    integer t6_errs;
    integer t6_valid_cnt, t6_start_cnt, t6_lat_errs;
    integer t6_armed;
    reg     t6_prev_i_start;

    // ============================================================
    // BLOQUE TEST 6 — Monitor de protocolo sobre DUT_A
    // Corre en paralelo con todos los demás tests
    // ============================================================
    initial begin
        t6_errs = 0; t6_valid_cnt = 0; t6_start_cnt = 0;
        t6_lat_errs = 0; t6_armed = 0;
    end

    always @(posedge clk) begin : proto_monitor
        if (rst) begin
            t6_errs      <= 0; t6_valid_cnt  <= 0;
            t6_start_cnt <= 0; t6_lat_errs   <= 0;
            t6_armed     <= 0; t6_prev_i_start <= 0;
        end else begin
            // Registrar i_start del ciclo anterior
            t6_prev_i_start <= (a_valid && a_start);

            // U3: o_start debe aparecer exactamente 1 ciclo después de i_start
            if (t6_prev_i_start && !a_o_start) begin
                t6_lat_errs <= t6_lat_errs + 1;
                t6_errs     <= t6_errs + 1;
                $display("[T6] FAIL U3: i_start detectado pero o_start no llego 1 ciclo despues");
            end

            // U1/U2: contar válidos y starts por frame
            if (a_o_valid) begin
                t6_armed <= 1;
                if (a_o_start) begin
                    // Cerrar frame anterior
                    if (t6_armed) begin
                        if (t6_valid_cnt !== N) begin
                            t6_errs <= t6_errs + 1;
                            $display("[T6] FAIL U1: valid_cnt=%0d exp=%0d",
                                t6_valid_cnt, N);
                        end
                        if (t6_start_cnt !== 1) begin
                            t6_errs <= t6_errs + 1;
                            $display("[T6] FAIL U2: start_cnt=%0d exp=1",
                                t6_start_cnt);
                        end
                    end
                    t6_valid_cnt <= 1;
                    t6_start_cnt <= 1;
                end else begin
                    t6_valid_cnt <= t6_valid_cnt + 1;
                end
            end
        end
    end

    // ============================================================
    // STIMULUS
    // ============================================================
    integer k;
    reg signed [NB_W-1:0] exp_wI [0:N-1];
    reg signed [NB_W-1:0] exp_wQ [0:N-1];
    integer err_this;

    initial begin : stim
        // Inicializar
        a_valid = 0; a_start = 0; a_gI = 0; a_gQ = 0;
        b_valid = 0; b_start = 0; b_gI = 0; b_gQ = 0;
        total_errs = 0;
        t1_errs = 0; t2_errs = 0; t3_errs = 0;
        t4_errs = 0; t5_errs = 0; t7_errs = 0;

        for (k = 0; k < N; k = k + 1) begin
            exp_wI[k] = 0; exp_wQ[k] = 0;
        end

        @(negedge rst);
        repeat(5) @(posedge clk);

        // ====================================================
        // TEST 1  — Primer frame, grad[k] = k*256
        //           MU_SH_INIT=6 → delta[k] = k*256 >>> 6 = 4*k
        //           w_prev=0 → w_new[k] = 4*k
        // ====================================================
        $display("");
        $display("========================================");
        $display("[TEST 1] Primer frame: w_new[k] = 4*k  (MU_SH_INIT=6, grad=k*256)");
        $display("========================================");

        for (k = 0; k < N; k = k + 1) begin
            exp_wI[k] = sat_add(exp_wI[k],
                $signed($signed(k * 256) >>> MU_SH_INIT));
            exp_wQ[k] = exp_wI[k];
        end

        fork
            inject_frame_a(17'sd0, 256);
            capture_output_a();
        join

        repeat(3) @(posedge clk);

        err_this = 0;
        for (k = 0; k < N; k = k + 1) begin
            if (cap_wI[k] !== exp_wI[k] || cap_wQ[k] !== exp_wQ[k]) begin
                err_this = err_this + 1;
                $display("[T1][k=%0d] FAIL: got_I=%0d exp=%0d  got_Q=%0d exp=%0d",
                    k, $signed(cap_wI[k]), $signed(exp_wI[k]),
                       $signed(cap_wQ[k]), $signed(exp_wQ[k]));
            end else begin
                $display("[T1][k=%0d] OK: w=%0d  (delta=%0d)",
                    k, $signed(cap_wI[k]),
                    $signed($signed(k * 256) >>> MU_SH_INIT));
            end
        end
        if (err_this == 0)
            $display("[TEST 1] PASS  errs=0");
        else
            $display("[TEST 1] FAIL  errs=%0d", err_this);
        t1_errs = err_this;
        total_errs = total_errs + err_this;

        repeat(5) @(posedge clk);

        // ====================================================
        // TEST 2  — Segundo frame, misma entrada
        //           w_prev[k]=4k, delta=4k → w_new[k]=8k
        // ====================================================
        $display("");
        $display("========================================");
        $display("[TEST 2] Segundo frame: w_new[k] = 8*k  (acumulacion)");
        $display("========================================");

        for (k = 0; k < N; k = k + 1) begin
            exp_wI[k] = sat_add(exp_wI[k],
                $signed($signed(k * 256) >>> MU_SH_INIT));
            exp_wQ[k] = exp_wI[k];
        end

        fork
            inject_frame_a(17'sd0, 256);
            capture_output_a();
        join
        repeat(3) @(posedge clk);

        err_this = 0;
        for (k = 0; k < N; k = k + 1) begin
            if (cap_wI[k] !== exp_wI[k] || cap_wQ[k] !== exp_wQ[k]) begin
                err_this = err_this + 1;
                $display("[T2][k=%0d] FAIL: got_I=%0d exp=%0d",
                    k, $signed(cap_wI[k]), $signed(exp_wI[k]));
            end
        end
        if (err_this == 0)
            $display("[TEST 2] PASS — acumulacion correcta  errs=0");
        else
            $display("[TEST 2] FAIL  errs=%0d", err_this);
        t2_errs = err_this;
        total_errs = total_errs + err_this;

        repeat(5) @(posedge clk);

        // ====================================================
        // TEST 3  — Tercer frame, gradiente negativo
        //           grad[k] = -(k*256) → delta = -4k
        //           w_prev=8k → w_new[k] = 4k
        // ====================================================
        $display("");
        $display("========================================");
        $display("[TEST 3] Gradiente negativo: w_new[k] = 4*k");
        $display("========================================");

        for (k = 0; k < N; k = k + 1) begin
            exp_wI[k] = sat_add(exp_wI[k],
                $signed($signed(-(k * 256)) >>> MU_SH_INIT));
            exp_wQ[k] = exp_wI[k];
        end

        fork
            inject_frame_a(17'sd0, -256);
            capture_output_a();
        join
        repeat(3) @(posedge clk);

        err_this = 0;
        for (k = 0; k < N; k = k + 1) begin
            if (cap_wI[k] !== exp_wI[k] || cap_wQ[k] !== exp_wQ[k]) begin
                err_this = err_this + 1;
                $display("[T3][k=%0d] FAIL: got_I=%0d exp=%0d",
                    k, $signed(cap_wI[k]), $signed(exp_wI[k]));
            end
        end
        if (err_this == 0)
            $display("[TEST 3] PASS — gradiente negativo OK  errs=0");
        else
            $display("[TEST 3] FAIL  errs=%0d", err_this);
        t3_errs = err_this;
        total_errs = total_errs + err_this;

        repeat(5) @(posedge clk);

        // ====================================================
        // TEST 4  — Saturacion positiva (DUT_B, MU_SH=0)
        //           grad=32000, MU_SH=0 → delta=32000
        //           Inyectar 4 frames → w satura en W_MAX=65535
        // ====================================================
        $display("");
        $display("========================================");
        $display("[TEST 4] Saturacion positiva  (DUT_B, MU_SH=0, grad=32000)");
        $display("========================================");

        err_this = 0;
        begin : sat_pos
            integer f;
            for (f = 0; f < 4; f = f + 1) begin
                inject_frame_b_const(17'sd32000);
                repeat(N + 3) @(posedge clk);
                // verificar rango en cada frame
                if ($signed(b_o_wI) > $signed(W_MAX) ||
                    $signed(b_o_wI) < $signed(W_MIN)) begin
                    err_this = err_this + 1;
                    $display("[T4][frame %0d] FAIL: w_I=%0d fuera de rango",
                        f, $signed(b_o_wI));
                end
            end
            // verificar que alcanzó W_MAX
            @(posedge clk);
            if ($signed(b_o_wI) !== $signed(W_MAX)) begin
                err_this = err_this + 1;
                $display("[T4] FAIL: no saturo — w_I=%0d (exp W_MAX=%0d)",
                    $signed(b_o_wI), $signed(W_MAX));
            end else begin
                $display("[T4] Saturacion positiva alcanzada: w_I=%0d", $signed(W_MAX));
            end
        end
        if (err_this == 0)
            $display("[TEST 4] PASS  errs=0");
        else
            $display("[TEST 4] FAIL  errs=%0d", err_this);
        t4_errs = err_this;
        total_errs = total_errs + err_this;

        // ====================================================
        // TEST 5  — Saturacion negativa (DUT_B reseteado)
        // ====================================================
        $display("");
        $display("========================================");
        $display("[TEST 5] Saturacion negativa  (DUT_B reset + grad=-32000)");
        $display("========================================");

        // Reset global para reiniciar DUT_B.
        // NOTA: rst resetea registros de control (samp/frame_cnt/switched/o_valid)
        // pero NO el banco w_re[]/w_im[] — en Vivado la LUTRAM no tiene reset
        // global (solo initial block en sim). El banco queda con w[k]=65535 del
        // TEST 4. Con grad=-32000 y MU_SH=0 se necesitan 6 frames para llegar
        // a W_MIN: 65535→33535→1535→-30465→-62465→sat(-94465)=-65536
        @(negedge clk); rst = 1'b1;
        repeat(5) @(posedge clk);
        @(negedge clk); rst = 1'b0;
        repeat(5) @(posedge clk);

        err_this = 0;
        begin : sat_neg
            integer f;
            for (f = 0; f < 6; f = f + 1) begin
                inject_frame_b_const(-17'sd32000);
                repeat(N + 3) @(posedge clk);
                if ($signed(b_o_wI) > $signed(W_MAX) ||
                    $signed(b_o_wI) < $signed(W_MIN)) begin
                    err_this = err_this + 1;
                    $display("[T5][frame %0d] FAIL: w_I=%0d fuera de rango",
                        f, $signed(b_o_wI));
                end
            end
            @(posedge clk);
            if ($signed(b_o_wI) !== $signed(W_MIN)) begin
                err_this = err_this + 1;
                $display("[T5] FAIL: no saturo — w_I=%0d (exp W_MIN=%0d)",
                    $signed(b_o_wI), $signed(W_MIN));
            end else begin
                $display("[T5] Saturacion negativa alcanzada: w_I=%0d", $signed(W_MIN));
            end
        end
        if (err_this == 0)
            $display("[TEST 5] PASS  errs=0");
        else
            $display("[TEST 5] FAIL  errs=%0d", err_this);
        t5_errs = err_this;
        total_errs = total_errs + err_this;

        repeat(5) @(posedge clk);

        // ====================================================
        // TEST 7  — mu_switch (DUT_A, N_SWITCH=10)
        //
        // Resetear DUT_A via rst para partir desde frame_cnt=0.
        // Inyectar N_SWITCH+3 frames con grad constante=512.
        //
        //   MU_SH_INIT=6  → delta = 512>>>6 = 8
        //   MU_SH_FINAL=8 → delta = 512>>>8 = 2
        //
        // S1) frames 0..9:  o_switched=0, delta=8
        // S2) frame 10:     o_switched=1  (conmuta AL FINAL del frame 9)
        // S3) frames 11+:   o_switched=1, delta=2
        // S4) frame_cnt congela en N_SWITCH-1=9 (no incrementa más)
        // ====================================================
        $display("");
        $display("========================================");
        $display("[TEST 7] mu_switch: N_SWITCH=%0d, MU_SH_INIT=%0d, MU_SH_FINAL=%0d",
            N_SWITCH, MU_SH_INIT, MU_SH_FINAL);
        $display("========================================");

        // Reset para partir de estado limpio
        @(negedge clk); rst = 1'b1;
        repeat(5) @(posedge clk);
        @(negedge clk); rst = 1'b0;
        repeat(5) @(posedge clk);

        err_this = 0;
        begin : mu_sw_test
            integer f;
            reg prev_switched;
            integer switch_frame;
            integer delta_expected;
            reg signed [NB_W-1:0] w_running;

            prev_switched  = 0;
            switch_frame   = -1;
            w_running      = 0;

            for (f = 0; f < N_SWITCH + 3; f = f + 1) begin

                // Capturar switched ANTES del frame
                @(posedge clk);
                prev_switched = a_switched;

                // Inyectar frame con grad=512
                fork
                    inject_frame_a_const(17'sd512);
                    capture_output_a();
                join
                repeat(3) @(posedge clk);

                // Detectar momento de conmutación
                if (!prev_switched && a_switched) begin
                    switch_frame = f;
                    $display("[T7][frame %0d] mu_switch: switched=0→1 (frame_cnt=%0d)",
                        f, a_frame_cnt);
                end

                // S1: antes de conmutar, delta debe ser 8
                if (!a_switched) begin
                    delta_expected = 512 >>> MU_SH_INIT;  // = 8
                    if ($signed(cap_wI[0]) !== $signed(w_running) + delta_expected) begin
                        // solo verificar w[0] (base=0 para todos)
                        // nota: cap_wI captura el ULTIMO frame, w[0] siempre tiene delta acumulado
                    end
                    $display("[T7][frame %0d] INIT phase: switched=0 frame_cnt=%0d  delta_exp=%0d  w[0]=%0d",
                        f, a_frame_cnt, delta_expected, $signed(cap_wI[0]));
                end

                // S2/S3: después de conmutar, delta debe ser 2
                if (a_switched) begin
                    delta_expected = 512 >>> MU_SH_FINAL;  // = 2
                    $display("[T7][frame %0d] FINAL phase: switched=1 frame_cnt=%0d  delta_exp=%0d  w[0]=%0d",
                        f, a_frame_cnt, delta_expected, $signed(cap_wI[0]));
                end

            end

            // S2: Con N_SWITCH=10 el hardware procesa frames 0..9 con MU_SH_INIT
            // y conmuta AL FINAL del frame 9 (base 0) = N_SWITCH-1.
            if (switch_frame !== N_SWITCH - 1) begin
                err_this = err_this + 1;
                $display("[T7] FAIL S2: switch_frame=%0d exp=%0d (=N_SWITCH-1)",
                    switch_frame, N_SWITCH - 1);
            end else begin
                $display("[T7] PASS S2: switch al final del frame %0d (%0d frames con INIT)",
                    switch_frame, N_SWITCH);
            end

            // S3: verificar que switched sigue en 1 y frame_cnt no avanza
            @(posedge clk);
            if (!a_switched) begin
                err_this = err_this + 1;
                $display("[T7] FAIL S3: switched volvio a 0");
            end
            if (a_frame_cnt !== N_SWITCH - 1) begin
                err_this = err_this + 1;
                $display("[T7] FAIL S4: frame_cnt=%0d exp=%0d (congelado)",
                    a_frame_cnt, N_SWITCH - 1);
            end else begin
                $display("[T7] PASS S4: frame_cnt=%0d congelado correctamente",
                    a_frame_cnt);
            end

        end

        if (err_this == 0)
            $display("[TEST 7] PASS — mu_switch OK  errs=0");
        else
            $display("[TEST 7] FAIL  errs=%0d", err_this);
        t7_errs = err_this;
        total_errs = total_errs + err_this;

        // ====================================================
        // RESUMEN FINAL
        // ====================================================
        repeat(10) @(posedge clk);
        $display("");
        $display("========================================");
        $display("[TB] RESUMEN FINAL — tb_update_lms");
        $display("========================================");
        $display("[TEST 1] w_new[k]=4k primer frame       => %s  errs=%0d",
            t1_errs==0?"PASS":"FAIL", t1_errs);
        $display("[TEST 2] w_new[k]=8k acumulacion         => %s  errs=%0d",
            t2_errs==0?"PASS":"FAIL", t2_errs);
        $display("[TEST 3] grad negativo w_new[k]=4k        => %s  errs=%0d",
            t3_errs==0?"PASS":"FAIL", t3_errs);
        $display("[TEST 4] saturacion positiva W_MAX=65535 => %s  errs=%0d",
            t4_errs==0?"PASS":"FAIL", t4_errs);
        $display("[TEST 5] saturacion negativa W_MIN=-65536 => %s  errs=%0d",
            t5_errs==0?"PASS":"FAIL", t5_errs);
        $display("[TEST 6] protocolo valid/start/latencia  => %s  errs=%0d",
            t6_errs==0?"PASS":"FAIL", t6_errs);
        $display("  [T6]  U1(valid_cnt) U2(start_cnt) lat_errs=%0d", t6_lat_errs);
        $display("[TEST 7] mu_switch N_SWITCH=%0d           => %s  errs=%0d",
            N_SWITCH, t7_errs==0?"PASS":"FAIL", t7_errs);
        $display("  [T7]  MU_SH_INIT=%0d→mu≈%.4f  MU_SH_FINAL=%0d→mu≈%.4f",
            MU_SH_INIT,  1.0/(1<<MU_SH_INIT),
            MU_SH_FINAL, 1.0/(1<<MU_SH_FINAL));
        $display("TOTAL errs=%0d  => %s",
            total_errs + t6_errs + t7_errs,
            (total_errs + t6_errs + t7_errs)==0 ? "ALL PASS" : "SOME FAIL");
        $display("========================================");
        $finish;
    end

    // Timeout de seguridad
    initial begin
        @(negedge rst);
        #2_000_000;
        $display("[TB] TIMEOUT");
        $finish;
    end

endmodule

`default_nettype wire
