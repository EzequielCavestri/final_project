`timescale 1ns/1ps
`default_nettype none

// ============================================================
// tb_slicer_qpsk
//
// Verifica el módulo slicer_qpsk de forma aislada.
//
// Parámetros:  NB_W=9, NBF_W=7, QPSK_A=91, latencia=1 ciclo
//
// CASOS:
//   C1) Decisión: 4 cuadrantes + casos borde (y=0, max, min, símbolo perfecto)
//   C2) 4 frames completos de N=16 muestras LFSR consecutivos
//   C3) Gap (i_valid=0): salidas deben ser cero, nada avanza
//   C4) Reset en medio de frame: post-reset todo en 0, luego frame limpio
//
// VERIFICACIONES (monitor permanente + checks en task):
//   V1) o_yhat_I/Q ∈ {+91, -91}  siempre que o_valid=1
//   V2) signo(o_yhat) == signo(i_y del ciclo anterior)
//   V3) o_e = o_yhat - i_y[T-1]  (con saturación a NB_W bits)
//   V4) o_valid = i_valid retrasado 1 ciclo
//   V5) Cuando o_valid=0 → o_yhat=0, o_e=0
// ============================================================

module tb_slicer_qpsk;

    localparam integer NB_W  = 9;
    localparam integer NBF_W = 7;
    localparam integer NFFT  = 32;
    localparam integer N     = NFFT / 2;   // 16
    localparam signed [NB_W-1:0] QPSK_A     =  9'sd91;
    localparam signed [NB_W-1:0] QPSK_A_NEG = -9'sd91;
    localparam CLK_HALF = 5;

    // ---- Puertos DUT ----
    reg                    clk, rst;
    reg                    i_valid, i_start;
    reg  signed [NB_W-1:0] i_yI, i_yQ;
    wire                   o_valid, o_start;
    wire signed [NB_W-1:0] o_yhat_I, o_yhat_Q;
    wire signed [NB_W-1:0] o_e_I, o_e_Q;

    slicer_qpsk #(.NB_W(NB_W), .NBF_W(NBF_W), .NFFT(NFFT)) u_dut (
        .clk(clk), .rst(rst),
        .i_valid(i_valid), .i_start(i_start),
        .i_yI(i_yI), .i_yQ(i_yQ),
        .o_valid(o_valid), .o_start(o_start),
        .o_yhat_I(o_yhat_I), .o_yhat_Q(o_yhat_Q),
        .o_e_I(o_e_I), .o_e_Q(o_e_Q)
    );

    initial clk = 0;
    always #CLK_HALF clk = ~clk;

    // ---- Delay line 1 ciclo (para comparar salida con entrada anterior) ----
    reg signed [NB_W-1:0] d1_yI, d1_yQ;
    reg                   d1_valid;
    always @(posedge clk) begin
        d1_yI   <= i_yI;
        d1_yQ   <= i_yQ;
        d1_valid <= i_valid;
    end

    // ---- Contadores ----
    integer total_errs;
    integer mon_errs;

    function integer iabs;
        input integer x;
        iabs = (x < 0) ? -x : x;
    endfunction

    // ============================================================
    // MONITOR PERMANENTE — corre cada ciclo sin depender de los casos
    // ============================================================
    always @(posedge clk) begin : monitor
        if (!rst) begin
            if (o_valid) begin
                // V1: yhat debe ser exactamente ±91
                if ($signed(o_yhat_I) !== QPSK_A && $signed(o_yhat_I) !== QPSK_A_NEG) begin
                    $display("[MON] FAIL V1: o_yhat_I=%0d no es +/-91", $signed(o_yhat_I));
                    mon_errs = mon_errs + 1;
                end
                if ($signed(o_yhat_Q) !== QPSK_A && $signed(o_yhat_Q) !== QPSK_A_NEG) begin
                    $display("[MON] FAIL V1: o_yhat_Q=%0d no es +/-91", $signed(o_yhat_Q));
                    mon_errs = mon_errs + 1;
                end
                // V2: signo de yhat debe coincidir con signo de i_y del ciclo anterior
                if (d1_yI >= 0 && $signed(o_yhat_I) !== QPSK_A) begin
                    $display("[MON] FAIL V2: d1_yI=%0d>=0 pero o_yhat_I=%0d (exp +91)",
                        $signed(d1_yI), $signed(o_yhat_I));
                    mon_errs = mon_errs + 1;
                end
                if (d1_yI <  0 && $signed(o_yhat_I) !== QPSK_A_NEG) begin
                    $display("[MON] FAIL V2: d1_yI=%0d<0  pero o_yhat_I=%0d (exp -91)",
                        $signed(d1_yI), $signed(o_yhat_I));
                    mon_errs = mon_errs + 1;
                end
                if (d1_yQ >= 0 && $signed(o_yhat_Q) !== QPSK_A) begin
                    $display("[MON] FAIL V2: d1_yQ=%0d>=0 pero o_yhat_Q=%0d (exp +91)",
                        $signed(d1_yQ), $signed(o_yhat_Q));
                    mon_errs = mon_errs + 1;
                end
                if (d1_yQ <  0 && $signed(o_yhat_Q) !== QPSK_A_NEG) begin
                    $display("[MON] FAIL V2: d1_yQ=%0d<0  pero o_yhat_Q=%0d (exp -91)",
                        $signed(d1_yQ), $signed(o_yhat_Q));
                    mon_errs = mon_errs + 1;
                end
                // V3: o_e debe ser yhat - d1_y  (saturado a NB_W)
                begin : v3_check
                    reg signed [NB_W+1:0] exp_eI_ext, exp_eQ_ext;
                    reg signed [NB_W-1:0] exp_eI, exp_eQ;
                    exp_eI_ext = $signed({{3{o_yhat_I[NB_W-1]}}, o_yhat_I})
                               - $signed({{3{d1_yI[NB_W-1]}},    d1_yI});
                    exp_eQ_ext = $signed({{3{o_yhat_Q[NB_W-1]}}, o_yhat_Q})
                               - $signed({{3{d1_yQ[NB_W-1]}},    d1_yQ});
                    exp_eI = (exp_eI_ext >  255) ?  255 :
                             (exp_eI_ext < -256) ? -256 : exp_eI_ext[NB_W-1:0];
                    exp_eQ = (exp_eQ_ext >  255) ?  255 :
                             (exp_eQ_ext < -256) ? -256 : exp_eQ_ext[NB_W-1:0];
                    if ($signed(o_e_I) !== $signed(exp_eI)) begin
                        $display("[MON] FAIL V3: o_e_I=%0d exp=%0d (yhat_I=%0d d1_yI=%0d)",
                            $signed(o_e_I), $signed(exp_eI),
                            $signed(o_yhat_I), $signed(d1_yI));
                        mon_errs = mon_errs + 1;
                    end
                    if ($signed(o_e_Q) !== $signed(exp_eQ)) begin
                        $display("[MON] FAIL V3: o_e_Q=%0d exp=%0d (yhat_Q=%0d d1_yQ=%0d)",
                            $signed(o_e_Q), $signed(exp_eQ),
                            $signed(o_yhat_Q), $signed(d1_yQ));
                        mon_errs = mon_errs + 1;
                    end
                end
            end else begin
                // V5: cuando o_valid=0 todo debe ser 0
                if (o_yhat_I !== 0 || o_yhat_Q !== 0 || o_e_I !== 0 || o_e_Q !== 0) begin
                    $display("[MON] FAIL V5: o_valid=0 pero salidas no cero yhat=(%0d,%0d) e=(%0d,%0d)",
                        $signed(o_yhat_I),$signed(o_yhat_Q),
                        $signed(o_e_I),   $signed(o_e_Q));
                    mon_errs = mon_errs + 1;
                end
            end
        end
    end

    // ============================================================
    // TAREAS AUXILIARES
    // ============================================================

    // Pulso de reset limpio
    task do_reset;
        begin
            rst=1; i_valid=0; i_start=0; i_yI=0; i_yQ=0;
            @(posedge clk); #1;
            @(posedge clk); #1;
            rst=0; @(posedge clk); #1;
        end
    endtask

    // Enviar una muestra y esperar que salga (con gap posterior)
    task send_one;
        input signed [NB_W-1:0] yI, yQ;
        input is_start;
        begin
            i_yI=yI; i_yQ=yQ; i_valid=1; i_start=is_start;
            @(posedge clk); #1;
            i_valid=0; i_start=0; i_yI=0; i_yQ=0;
            @(posedge clk); #1;   // ciclo de salida — monitor verifica
            @(posedge clk); #1;   // ciclo de idle  — monitor verifica V5
        end
    endtask

    // ============================================================
    // CASO 1 — Decisión QPSK: 4 cuadrantes + casos borde
    // ============================================================
    integer c1_errs_pre;
    task run_caso1;
        begin
            c1_errs_pre = mon_errs + total_errs;
            $display("--- CASO 1: Decision QPSK (4 cuadrantes + borde) ---");
            do_reset;
            // Q1: ++
            send_one( 9'sd60,   9'sd45,  1);
            // Q2: -+
            send_one(-9'sd60,   9'sd45,  0);
            // Q3: --
            send_one(-9'sd60,  -9'sd45,  0);
            // Q4: +-
            send_one( 9'sd60,  -9'sd45,  0);
            // Borde: y=0 → positivo (>=0)
            send_one(  9'sd0,    9'sd0,  0);
            // Máximo positivo
            send_one( 9'sd127,  9'sd127, 0);
            // Mínimo negativo: -128 en Q9 signed
            send_one(-9'sd128, -9'sd128, 0);
            // Símbolo perfecto: e debe ser 0
            send_one( 9'sd91,   9'sd91,  0);
            send_one(-9'sd91,  -9'sd91,  0);

            if (mon_errs + total_errs == c1_errs_pre)
                $display("[CASO 1] errs=0 => PASS");
            else
                $display("[CASO 1] FAIL errs=%0d", mon_errs+total_errs-c1_errs_pre);
        end
    endtask

    // ============================================================
    // CASO 2 — 4 frames completos LFSR continuos
    // ============================================================
    reg [15:0] lfsr;
    integer    c2_errs_pre;
    integer    c2_start_cnt, c2_valid_cnt;
    integer    f, j;
    reg [7:0]  tmp_i, tmp_q;

    task run_caso2;
        begin
            $display("--- CASO 2: 4 frames consecutivos (LFSR) ---");
            lfsr = 16'hACE1;
            do_reset;

            for (f=0; f<4; f=f+1) begin
                c2_errs_pre  = mon_errs + total_errs;
                c2_start_cnt = 0;
                c2_valid_cnt = 0;

                for (j=0; j<N; j=j+1) begin
                    // Avanzar LFSR
                    lfsr = {lfsr[14:0], lfsr[15]^lfsr[13]^lfsr[12]^lfsr[10]};
                    tmp_i = lfsr[7:0];
                    tmp_q = lfsr[15:8];
                    i_yI   = {{(NB_W-7){tmp_i[7]}}, tmp_i[7:1]};
                    i_yQ   = {{(NB_W-7){tmp_q[7]}}, tmp_q[7:1]};
                    i_valid = 1;
                    i_start = (j == 0);
                    @(posedge clk); #1;
                    // Contar salida de este ciclo (output del input j, latencia 1)
                    if (o_valid) c2_valid_cnt = c2_valid_cnt + 1;
                    if (o_start) c2_start_cnt = c2_start_cnt + 1;
                end
                // Flush: sacar la última muestra del frame
                i_valid=0; i_start=0; i_yI=0; i_yQ=0;
                @(posedge clk); #1;
                // o_valid=0 aquí (i_valid=0 en el ciclo anterior) → no contar

                // Verificar conteos P1/P2
                if (c2_valid_cnt !== N) begin
                    $display("  [F%0d] FAIL P1: valid_cnt=%0d exp=%0d", f, c2_valid_cnt, N);
                    total_errs = total_errs + 1;
                end
                if (c2_start_cnt !== 1) begin
                    $display("  [F%0d] FAIL P2: start_cnt=%0d exp=1", f, c2_start_cnt);
                    total_errs = total_errs + 1;
                end
                if (mon_errs + total_errs == c2_errs_pre)
                    $display("  [F%0d] valid=%0d  start=%0d  errs=0  => PASS", f, c2_valid_cnt, c2_start_cnt);
                else
                    $display("  [F%0d] FAIL acum_errs=%0d", f, mon_errs+total_errs-c2_errs_pre);
            end

            if (total_errs == 0 && mon_errs == 0)
                $display("[CASO 2] total_errs=0 => PASS");
            else
                $display("[CASO 2] FAIL");
        end
    endtask

    // ============================================================
    // CASO 3 — Gap de i_valid=0: salidas permanecen en cero
    // ============================================================
    integer c3_errs_pre, gap;
    task run_caso3;
        begin
            c3_errs_pre = mon_errs + total_errs;
            $display("--- CASO 3: Gap de valid=0 (20 ciclos) ---");
            do_reset;
            // Una muestra válida
            i_yI=9'sd50; i_yQ=9'sd30; i_valid=1; i_start=1;
            @(posedge clk); #1;
            // Gap largo: el monitor verifica V5 cada ciclo
            i_valid=0; i_start=0; i_yI=0; i_yQ=0;
            repeat(20) @(posedge clk); #1;
            // Reanudar
            i_yI=-9'sd50; i_yQ=-9'sd30; i_valid=1; i_start=1;
            @(posedge clk); #1;
            i_valid=0; @(posedge clk); #1;

            if (mon_errs + total_errs == c3_errs_pre)
                $display("[CASO 3] errs=0 => PASS");
            else
                $display("[CASO 3] FAIL errs=%0d", mon_errs+total_errs-c3_errs_pre);
        end
    endtask

    // ============================================================
    // CASO 4 — Reset en medio de frame
    // ============================================================
    integer c4_errs_pre;
    task run_caso4;
        integer jj;
        begin
            c4_errs_pre = mon_errs + total_errs;
            $display("--- CASO 4: Reset en medio de frame ---");
            do_reset;
            // Enviar 6 muestras y resetear a mitad
            for (jj=0; jj<6; jj=jj+1) begin
                i_yI=9'sd40; i_yQ=-9'sd20; i_valid=1; i_start=(jj==0);
                @(posedge clk); #1;
            end
            rst=1; i_valid=0; i_start=0; i_yI=0; i_yQ=0;
            @(posedge clk); #1;
            @(posedge clk); #1;
            // Post-reset: verificar que todo es 0
            if (o_valid !== 0 || o_yhat_I !== 0 || o_yhat_Q !== 0
                || o_e_I !== 0 || o_e_Q !== 0) begin
                $display("[CASO 4] FAIL: salidas no cero post-reset");
                total_errs = total_errs + 1;
            end
            // Reanudar: frame con símbolos perfectos (e debe ser 0)
            rst=0; @(posedge clk); #1;
            for (jj=0; jj<N; jj=jj+1) begin
                i_yI = (jj < N/2) ?  9'sd91 : -9'sd91;
                i_yQ = (jj < N/2) ?  9'sd91 : -9'sd91;
                i_valid=1; i_start=(jj==0);
                @(posedge clk); #1;
            end
            i_valid=0; @(posedge clk); #1;

            if (mon_errs + total_errs == c4_errs_pre)
                $display("[CASO 4] post-reset valid=%0d  errs=0  => PASS", N);
            else
                $display("[CASO 4] FAIL errs=%0d", mon_errs+total_errs-c4_errs_pre);
        end
    endtask

    // ============================================================
    // MAIN
    // ============================================================
    initial begin
        total_errs = 0;
        mon_errs   = 0;
        i_valid=0; i_start=0; i_yI=0; i_yQ=0; rst=1;
        @(posedge clk); #1;

        run_caso1; #20;
        run_caso2; #20;
        run_caso3; #20;
        run_caso4; #20;

        $display("");
        $display("========================================");
        $display("[TB_SLICER_QPSK] RESUMEN FINAL");
        $display("========================================");
        $display("[TOTAL]  errs=%0d  => %s",
            total_errs + mon_errs,
            (total_errs+mon_errs == 0) ? "PASS: slicer_qpsk OK" : "FAIL");
        $display("========================================");
        $finish;
    end

    initial begin
        #500_000;
        $display("[TB_SLICER_QPSK] TIMEOUT");
        $finish;
    end

endmodule

`default_nettype wire
