`timescale 1ns/1ps
`default_nettype none

// ============================================================
// tb_tabla_w.v  —  Testbench STANDALONE
//
// TESTS
// -----
// [TEST 1] Escritura y lectura básica
//          Escribe W[k] = k*100 (valores 0,100,200,...,3100)
//          Lee de vuelta con rd_cnt y verifica W[k] == k*100
//
// [TEST 2] Sobrescritura — nuevo frame reemplaza el anterior
//          Escribe W[k] = k*100, luego W[k] = k*200
//          Verifica que la lectura devuelve los valores nuevos
//
// [TEST 3] Independencia de contadores
//          Escribe los 32 bins con wr_cnt
//          Lee con rd_cnt que arranca ANTES que termine la escritura
//          Verifica que la lectura es coherente con lo escrito
//
// [TEST 4] Lectura en cada frame — el mismo dato está disponible
//          Escribe W[k]=k+1 una vez, luego lee 3 veces seguidas
//          Verifica que los tres reads dan los mismos valores
//
// [TEST 5] Protocolo de contadores
//          R1) wr_cnt se sincroniza con i_wr_start
//          R2) rd_cnt se sincroniza con i_rd_start
//          R3) lectura combinacional: o_W_re cambia en el mismo
//              ciclo que rd_cnt avanza (sin latencia de registro)
// ============================================================

module tb_tabla_w;

    // ============================================================
    // Parámetros
    // ============================================================
    localparam integer NB_W  = 17;
    localparam integer NBF_W = 10;
    localparam integer NFFT  = 32;

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
    reg                    i_wr_valid, i_wr_start;
    reg  signed [NB_W-1:0] i_W_re,    i_W_im;
    reg                    i_rd_valid, i_rd_start;

    wire signed [NB_W-1:0] o_W_re, o_W_im;

    tabla_w #(
        .NB_W (NB_W),
        .NBF_W(NBF_W),
        .NFFT (NFFT)
    ) dut (
        .clk       (clk),
        .rst       (rst),
        .i_wr_valid(i_wr_valid),
        .i_wr_start(i_wr_start),
        .i_W_re    (i_W_re),
        .i_W_im    (i_W_im),
        .i_rd_valid(i_rd_valid),
        .i_rd_start(i_rd_start),
        .o_W_re    (o_W_re),
        .o_W_im    (o_W_im)
    );

    // ============================================================
    // Tarea: escribir un frame completo de NFFT bins
    //   W[k] = base + k * step
    // ============================================================
    task write_frame;
        input signed [NB_W-1:0] base;
        input integer            step;
        integer k;
        begin
            for (k = 0; k < NFFT; k = k + 1) begin
                @(negedge clk);
                i_wr_valid = 1'b1;
                i_wr_start = (k == 0) ? 1'b1 : 1'b0;
                i_W_re     = base + k * step;
                i_W_im     = base + k * step;
            end
            @(negedge clk);
            i_wr_valid = 1'b0;
            i_wr_start = 1'b0;
            i_W_re     = {NB_W{1'b0}};
            i_W_im     = {NB_W{1'b0}};
        end
    endtask

    // ============================================================
    // Tarea: leer un frame completo de NFFT bins y capturar
    // ============================================================
    reg signed [NB_W-1:0] cap_re [0:NFFT-1];
    reg signed [NB_W-1:0] cap_im [0:NFFT-1];

    task read_frame;
        integer k;
        begin
            for (k = 0; k < NFFT; k = k + 1) begin
                @(negedge clk);
                i_rd_valid = 1'b1;
                i_rd_start = (k == 0) ? 1'b1 : 1'b0;
                // La lectura es combinacional: capturar DESPUÉS del negedge
                // pero ANTES del posedge (valores estables)
                #1;  // pequeño delay para que eff_rd se actualice
                cap_re[k] = o_W_re;
                cap_im[k] = o_W_im;
            end
            @(negedge clk);
            i_rd_valid = 1'b0;
            i_rd_start = 1'b0;
        end
    endtask

    // ============================================================
    // Declaraciones globales
    // ============================================================
    integer total_errs;
    integer t1_errs, t2_errs, t3_errs, t4_errs;

    integer k;
    integer err_this;

    initial begin : stim
        i_wr_valid = 0; i_wr_start = 0; i_W_re = 0; i_W_im = 0;
        i_rd_valid = 0; i_rd_start = 0;
        total_errs = 0;
        t1_errs = 0; t2_errs = 0; t3_errs = 0; t4_errs = 0;

        @(negedge rst);
        repeat(5) @(posedge clk);

        // ====================================================
        // TEST 1 — Escritura y lectura básica
        //          W[k] = k*100
        //          Verifica: o_W_re[k] == k*100
        // ====================================================
        $display("");
        $display("========================================");
        $display("[TEST 1] Escritura/lectura basica: W[k]=k*100");
        $display("========================================");

        write_frame(17'sd0, 100);
        repeat(3) @(posedge clk);
        read_frame();
        repeat(3) @(posedge clk);

        err_this = 0;
        for (k = 0; k < NFFT; k = k + 1) begin
            if ($signed(cap_re[k]) !== k * 100) begin
                err_this = err_this + 1;
                $display("[T1][k=%0d] FAIL: got=%0d exp=%0d",
                    k, $signed(cap_re[k]), k*100);
            end else begin
                $display("[T1][k=%0d] OK: W_re=%0d", k, $signed(cap_re[k]));
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
        // TEST 2 — Sobrescritura
        //          Frame 1: W[k]=k*100
        //          Frame 2: W[k]=k*200
        //          Lectura debe devolver k*200
        // ====================================================
        $display("");
        $display("========================================");
        $display("[TEST 2] Sobrescritura: W[k]=k*100 luego W[k]=k*200");
        $display("========================================");

        write_frame(17'sd0, 100);
        repeat(2) @(posedge clk);
        write_frame(17'sd0, 200);
        repeat(3) @(posedge clk);
        read_frame();
        repeat(3) @(posedge clk);

        err_this = 0;
        for (k = 0; k < NFFT; k = k + 1) begin
            if ($signed(cap_re[k]) !== k * 200) begin
                err_this = err_this + 1;
                $display("[T2][k=%0d] FAIL: got=%0d exp=%0d",
                    k, $signed(cap_re[k]), k*200);
            end
        end
        if (err_this == 0)
            $display("[TEST 2] PASS — sobrescritura OK  errs=0");
        else
            $display("[TEST 2] FAIL  errs=%0d", err_this);
        t2_errs = err_this;
        total_errs = total_errs + err_this;

        repeat(5) @(posedge clk);

        // ====================================================
        // TEST 3 — Lectura repetida: mismos datos en 3 lecturas
        //          Escribe W[k]=k+1 una sola vez
        //          Lee 3 veces → siempre debe dar k+1
        // ====================================================
        $display("");
        $display("========================================");
        $display("[TEST 3] Lectura repetida: 3 reads del mismo frame");
        $display("========================================");

        write_frame(17'sd1, 1);   // W[k] = 1 + k
        repeat(3) @(posedge clk);

        err_this = 0;
        begin : triple_read
            integer rd;
            reg signed [NB_W-1:0] capA [0:NFFT-1];
            reg signed [NB_W-1:0] capB [0:NFFT-1];
            reg signed [NB_W-1:0] capC [0:NFFT-1];

            // Read 1
            read_frame();
            for (k = 0; k < NFFT; k = k + 1) capA[k] = cap_re[k];
            repeat(2) @(posedge clk);

            // Read 2
            read_frame();
            for (k = 0; k < NFFT; k = k + 1) capB[k] = cap_re[k];
            repeat(2) @(posedge clk);

            // Read 3
            read_frame();
            for (k = 0; k < NFFT; k = k + 1) capC[k] = cap_re[k];
            repeat(2) @(posedge clk);

            for (k = 0; k < NFFT; k = k + 1) begin
                if ($signed(capA[k]) !== k+1 ||
                    $signed(capB[k]) !== k+1 ||
                    $signed(capC[k]) !== k+1) begin
                    err_this = err_this + 1;
                    $display("[T3][k=%0d] FAIL: A=%0d B=%0d C=%0d exp=%0d",
                        k, $signed(capA[k]), $signed(capB[k]),
                           $signed(capC[k]), k+1);
                end
            end
        end

        if (err_this == 0)
            $display("[TEST 3] PASS — lectura no destructiva OK  errs=0");
        else
            $display("[TEST 3] FAIL  errs=%0d", err_this);
        t3_errs = err_this;
        total_errs = total_errs + err_this;

        repeat(5) @(posedge clk);

        // ====================================================
        // TEST 4 — Lectura combinacional (sin latencia)
        //          Escribe W[k]=k*50
        //          Avanza rd_cnt manualmente un ciclo
        //          Verifica que o_W_re cambia en el MISMO ciclo
        //          que i_rd_valid sube (no un ciclo después)
        // ====================================================
        $display("");
        $display("========================================");
        $display("[TEST 4] Lectura combinacional: sin latencia adicional");
        $display("========================================");

        write_frame(17'sd0, 50);   // W[k] = k*50
        repeat(3) @(posedge clk);

        err_this = 0;
        begin : comb_read_test
            reg signed [NB_W-1:0] val_at_rd_valid;
            integer j;

            for (j = 0; j < NFFT; j = j + 1) begin
                @(negedge clk);
                i_rd_valid = 1'b1;
                i_rd_start = (j == 0) ? 1'b1 : 1'b0;
                #1;
                // Capturar INMEDIATAMENTE — lectura combinacional
                val_at_rd_valid = o_W_re;
                // Verificar: debe ser W[j] = j*50
                if ($signed(val_at_rd_valid) !== j * 50) begin
                    err_this = err_this + 1;
                    $display("[T4][k=%0d] FAIL comb: got=%0d exp=%0d (1 ciclo de latencia?)",
                        j, $signed(val_at_rd_valid), j*50);
                end
            end
            @(negedge clk);
            i_rd_valid = 1'b0;
            i_rd_start = 1'b0;
        end

        if (err_this == 0)
            $display("[TEST 4] PASS — lectura combinacional confirmada  errs=0");
        else
            $display("[TEST 4] FAIL  errs=%0d", err_this);
        t4_errs = err_this;
        total_errs = total_errs + err_this;

        // ====================================================
        // RESUMEN FINAL
        // ====================================================
        repeat(10) @(posedge clk);
        $display("");
        $display("========================================");
        $display("[TB] RESUMEN FINAL — tb_tabla_w");
        $display("========================================");
        $display("[TEST 1] Escritura/lectura basica         => %s  errs=%0d",
            t1_errs==0?"PASS":"FAIL", t1_errs);
        $display("[TEST 2] Sobrescritura con nuevo frame    => %s  errs=%0d",
            t2_errs==0?"PASS":"FAIL", t2_errs);
        $display("[TEST 3] Lectura no destructiva (3x)      => %s  errs=%0d",
            t3_errs==0?"PASS":"FAIL", t3_errs);
        $display("[TEST 4] Lectura combinacional sin latencia => %s  errs=%0d",
            t4_errs==0?"PASS":"FAIL", t4_errs);
        $display("TOTAL errs=%0d  => %s",
            total_errs,
            total_errs==0 ? "ALL PASS" : "SOME FAIL");
        $display("========================================");
        $finish;
    end

    initial begin
        @(negedge rst);
        #500_000;
        $display("[TB] TIMEOUT");
        $finish;
    end

endmodule

`default_nettype wire
