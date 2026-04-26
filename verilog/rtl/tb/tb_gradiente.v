`timescale 1ns/1ps
`default_nettype none

// ============================================================
// tb_gradiente.v
//
// Verifica gradiente.v — PHI_k = conj(X_hist) * E_k
//
// Señales que confirman funcionamiento correcto:
//
//   o_valid  → debe subir exactamente 2 ciclos después de i_valid
//   o_start  → debe subir exactamente 2 ciclos después de i_start
//   o_phi_re → PHI_re = Xr*Er + Xi*Ei  (Q17.10 saturado)
//   o_phi_im → PHI_im = Xr*Ei - Xi*Er  (Q17.10 saturado)
//
// CASOS:
//   C1) Casos conocidos simples — verificación matemática exacta
//   C2) Frame completo (NFFT=32 muestras) — verificar o_valid/o_start
//   C3) Datos aleatorios LFSR — verificar con modelo Python
//   C4) Saturación — entradas máximas, verificar clamp
//   C5) Reset — verificar que salidas vuelven a 0
// ============================================================

module tb_gradiente;

    localparam integer NB_W    = 17;
    localparam integer NBF_W   = 10;
    localparam integer NFFT    = 32;
    localparam integer CLK_HALF = 5;

    // SCALE = 2^NBF_W = 1024 (para convertir flotante a fixed)
    localparam integer SCALE = 1024;

    // ---- DUT ----
    reg                    clk, rst;
    reg                    i_valid, i_start;
    reg  signed [NB_W-1:0] i_xre, i_xim;
    reg  signed [NB_W-1:0] i_ere, i_eim;
    wire                   o_valid, o_start;
    wire signed [NB_W-1:0] o_phi_re, o_phi_im;

    gradiente #(.NB_W(NB_W), .NBF_W(NBF_W)) u_dut (
        .clk(clk), .rst(rst),
        .i_valid(i_valid), .i_start(i_start),
        .i_xre(i_xre), .i_xim(i_xim),
        .i_ere(i_ere), .i_eim(i_eim),
        .o_valid(o_valid), .o_start(o_start),
        .o_phi_re(o_phi_re), .o_phi_im(o_phi_im)
    );

    initial clk = 0;
    always #CLK_HALF clk = ~clk;

    integer total_errs;

    // ============================================================
    // Función de referencia: mul saturado Q(17,10)
    // ============================================================
    // Nota: en Verilog usamos tareas en vez de funciones para
    // resultados de múltiples valores

    // Modelo del gradiente en punto fijo
    // Retorna phi_re y phi_im como enteros
    reg signed [2*NB_W-1:0] ref_p_rr, ref_p_ii, ref_p_ri, ref_p_ir;
    reg signed [NB_W:0]     ref_sum_re, ref_sum_im;
    reg signed [NB_W-1:0]   ref_phi_re, ref_phi_im;

    localparam signed [NB_W-1:0] MAX_VAL =  {1'b0, {(NB_W-1){1'b1}}};
    localparam signed [NB_W-1:0] MIN_VAL =  {1'b1, {(NB_W-1){1'b0}}};

    task calc_ref;
        input signed [NB_W-1:0] xre, xim, ere, eim;
        reg signed [NB_W-1:0] trunc_rr, trunc_ii, trunc_ri, trunc_ir;
        begin
            // 4 productos Q(34,20)
            ref_p_rr = xre * ere;
            ref_p_ii = xim * eim;
            ref_p_ri = xre * eim;
            ref_p_ir = xim * ere;

            // Truncar a Q(24,10) tomando bits [33:10]
            trunc_rr = $signed(ref_p_rr[2*NB_W-1:NBF_W]);
            trunc_ii = $signed(ref_p_ii[2*NB_W-1:NBF_W]);
            trunc_ri = $signed(ref_p_ri[2*NB_W-1:NBF_W]);
            trunc_ir = $signed(ref_p_ir[2*NB_W-1:NBF_W]);

            // Sumas con 1 bit extra para detectar overflow
            ref_sum_re = $signed({trunc_rr[NB_W-1], trunc_rr}) +
                         $signed({trunc_ii[NB_W-1], trunc_ii});
            ref_sum_im = $signed({trunc_ri[NB_W-1], trunc_ri}) -
                         $signed({trunc_ir[NB_W-1], trunc_ir});

            // Saturar a NB_W bits
            if (ref_sum_re > $signed({1'b0, MAX_VAL}))
                ref_phi_re = MAX_VAL;
            else if (ref_sum_re < $signed({1'b1, MIN_VAL}))
                ref_phi_re = MIN_VAL;
            else
                ref_phi_re = ref_sum_re[NB_W-1:0];

            if (ref_sum_im > $signed({1'b0, MAX_VAL}))
                ref_phi_im = MAX_VAL;
            else if (ref_sum_im < $signed({1'b1, MIN_VAL}))
                ref_phi_im = MIN_VAL;
            else
                ref_phi_im = ref_sum_im[NB_W-1:0];
        end
    endtask

    // ============================================================
    // Tarea: aplicar 1 muestra y verificar después de 2 ciclos
    // ============================================================
    task apply_and_check;
        input signed [NB_W-1:0] xre, xim, ere, eim;
        input integer fid;
        input integer sample_idx;
        input integer is_start;
        begin
            // Aplicar entrada
            i_valid = 1; i_start = is_start;
            i_xre = xre; i_xim = xim;
            i_ere = ere; i_eim = eim;
            @(posedge clk); #1;
            i_start = 0;

            // Esperar 2 ciclos de latencia
            @(posedge clk); #1;
            @(posedge clk); #1;

            // Calcular referencia
            calc_ref(xre, xim, ere, eim);

            // Verificar
            if ($signed(o_phi_re) !== ref_phi_re || $signed(o_phi_im) !== ref_phi_im) begin
                $display("[C%0d][s=%0d] FAIL: phi_re=%0d exp=%0d  phi_im=%0d exp=%0d",
                    fid, sample_idx,
                    $signed(o_phi_re), ref_phi_re,
                    $signed(o_phi_im), ref_phi_im);
                total_errs = total_errs + 1;
            end
        end
    endtask

    // ============================================================
    // CASO 1 — Casos conocidos simples
    // Verificación matemática exacta con valores controlados
    // ============================================================
    task run_caso1;
        integer pre;
        begin
            $display("--- CASO 1: Casos matematicos conocidos ---");
            $display("  Verifica PHI = conj(X)*E con valores simples");
            pre = total_errs;

            // Reset limpio
            rst=1; i_valid=0; i_start=0;
            i_xre=0; i_xim=0; i_ere=0; i_eim=0;
            repeat(3) @(posedge clk); #1;
            rst=0; @(posedge clk); #1;

            // [A] X=1+0j, E=1+0j → PHI = 1+0j
            // PHI_re = 1*1 + 0*0 = 1.0 = 1024 en Q17.10
            // PHI_im = 1*0 - 0*1 = 0.0 = 0
            $display("  [A] X=1+0j, E=1+0j → exp PHI=1+0j");
            apply_and_check(SCALE, 0, SCALE, 0, 1, 0, 1);

            // [B] X=0+1j, E=1+0j → PHI = conj(0+1j)*(1+0j) = (0-1j)*(1+0j) = 0-1j
            // PHI_re = 0*1 + 1*0 = 0
            // PHI_im = 0*0 - 1*1 = -1 = -1024
            $display("  [B] X=0+1j, E=1+0j → exp PHI=0-1j");
            apply_and_check(0, SCALE, SCALE, 0, 1, 1, 1);

            // [C] X=0.5+0.5j, E=0.5+0.5j → PHI = conj(0.5+0.5j)*(0.5+0.5j)
            // = (0.5-0.5j)*(0.5+0.5j) = 0.25+0.25j-0.25j+0.25 = 0.5+0j
            // PHI_re = 0.5*0.5 + 0.5*0.5 = 0.5 = 512
            // PHI_im = 0.5*0.5 - 0.5*0.5 = 0
            $display("  [C] X=0.5+0.5j, E=0.5+0.5j → exp PHI=0.5+0j");
            apply_and_check(512, 512, 512, 512, 1, 2, 1);

            // [D] X=-1+0.5j, E=0.5-0.5j
            // PHI_re = (-1)*0.5 + 0.5*(-0.5) = -0.5-0.25 = -0.75 = -768
            // PHI_im = (-1)*(-0.5) - 0.5*0.5 = 0.5-0.25 = 0.25 = 256
            $display("  [D] X=-1+0.5j, E=0.5-0.5j → exp PHI=-0.75+0.25j");
            apply_and_check(-SCALE, 512, 512, -512, 1, 3, 1);

            if (total_errs == pre)
                $display("[CASO 1] PASS  errs=0");
            else
                $display("[CASO 1] FAIL  errs=%0d", total_errs-pre);
        end
    endtask

    // ============================================================
    // CASO 2 — Frame completo: verificar o_valid y o_start
    // Manda NFFT=32 muestras y verifica que:
    //   - Se reciben exactamente NFFT salidas válidas
    //   - o_start aparece exactamente 1 vez
    //   - La latencia total entre i_start y o_start es 2 ciclos
    // ============================================================
    integer valid_cnt_in, valid_cnt_out;
    integer start_cnt_in, start_cnt_out;
    integer latency_errs;
    integer start_in_cycle, start_out_cycle, cycle_cnt;

    task run_caso2;
        integer j, pre;
        begin
            $display("--- CASO 2: Frame completo - verificar o_valid y o_start ---");
            $display("  Envia %0d muestras, verifica timing de control", NFFT);
            pre = total_errs;

            rst=1; i_valid=0; i_start=0; i_xre=0; i_xim=0; i_ere=0; i_eim=0;
            repeat(3) @(posedge clk); #1;
            rst=0; @(posedge clk); #1;

            valid_cnt_in=0; valid_cnt_out=0;
            start_cnt_out=0; cycle_cnt=0;
            start_in_cycle=-1; start_out_cycle=-1;

            // Enviar frame completo de NFFT muestras
            for (j=0; j<NFFT; j=j+1) begin
                i_valid = 1;
                i_start = (j == 0);
                i_xre = j + 1;
                i_xim = -(j+1);
                i_ere = j + 1;
                i_eim = j + 1;
                @(posedge clk); #1;
                valid_cnt_in = valid_cnt_in + 1;
                if (i_start) start_in_cycle = cycle_cnt;
                // Capturar salida
                if (o_valid) valid_cnt_out = valid_cnt_out + 1;
                if (o_start) begin
                    start_cnt_out = start_cnt_out + 1;
                    start_out_cycle = cycle_cnt;
                end
                i_start = 0;
                cycle_cnt = cycle_cnt + 1;
            end

            // Drenar pipeline (2 ciclos extra)
            i_valid=0;
            repeat(4) begin
                @(posedge clk); #1;
                if (o_valid) valid_cnt_out = valid_cnt_out + 1;
                if (o_start) begin
                    start_cnt_out = start_cnt_out + 1;
                    start_out_cycle = cycle_cnt;
                end
                cycle_cnt = cycle_cnt + 1;
            end

            // Verificar conteo de valids
            if (valid_cnt_out !== NFFT) begin
                $display("  [C2] FAIL valid_cnt: out=%0d exp=%0d", valid_cnt_out, NFFT);
                total_errs = total_errs + 1;
            end

            // Verificar exactamente 1 start de salida
            if (start_cnt_out !== 1) begin
                $display("  [C2] FAIL start_cnt: out=%0d exp=1", start_cnt_out);
                total_errs = total_errs + 1;
            end

            // Verificar latencia = 2 ciclos
            if (start_in_cycle >= 0 && start_out_cycle >= 0) begin
                if (start_out_cycle - start_in_cycle !== 2) begin
                    $display("  [C2] FAIL latencia: start_in=%0d start_out=%0d delta=%0d exp=2",
                        start_in_cycle, start_out_cycle, start_out_cycle-start_in_cycle);
                    total_errs = total_errs + 1;
                end
            end

            if (total_errs == pre)
                $display("[CASO 2] PASS  valid_out=%0d  start_out=%0d  latencia=2ciclos",
                    valid_cnt_out, start_cnt_out);
            else
                $display("[CASO 2] FAIL  errs=%0d", total_errs-pre);
        end
    endtask

    // ============================================================
    // CASO 3 — Datos aleatorios LFSR
    // Verifica que el módulo calcula bien con valores variados
    // usando el modelo de referencia en Verilog
    // ============================================================
    reg [15:0] lfsr;

    task run_caso3;
        integer j, pre;
        reg signed [NB_W-1:0] xre_t, xim_t, ere_t, eim_t;
        reg [7:0] tmp;
        begin
            $display("--- CASO 3: 4 frames con datos LFSR ---");
            $display("  Compara salida del DUT contra modelo de referencia");
            lfsr = 16'hACE1;
            pre = total_errs;

            rst=1; i_valid=0; i_start=0;
            repeat(3) @(posedge clk); #1;
            rst=0; @(posedge clk); #1;

            for (j=0; j<4*NFFT; j=j+1) begin
                // Generar 4 valores con LFSR
                lfsr = {lfsr[14:0], lfsr[15]^lfsr[13]^lfsr[12]^lfsr[10]};
                tmp  = lfsr[7:0];
                xre_t = {{(NB_W-8){tmp[7]}}, tmp};

                lfsr = {lfsr[14:0], lfsr[15]^lfsr[13]^lfsr[12]^lfsr[10]};
                tmp  = lfsr[7:0];
                xim_t = {{(NB_W-8){tmp[7]}}, tmp};

                lfsr = {lfsr[14:0], lfsr[15]^lfsr[13]^lfsr[12]^lfsr[10]};
                tmp  = lfsr[7:0];
                ere_t = {{(NB_W-8){tmp[7]}}, tmp};

                lfsr = {lfsr[14:0], lfsr[15]^lfsr[13]^lfsr[12]^lfsr[10]};
                tmp  = lfsr[7:0];
                eim_t = {{(NB_W-8){tmp[7]}}, tmp};

                i_valid = 1;
                i_start = (j % NFFT == 0);
                i_xre = xre_t; i_xim = xim_t;
                i_ere = ere_t; i_eim = eim_t;
                @(posedge clk); #1;
                i_start = 0;

                // Calcular referencia para este sample
                calc_ref(xre_t, xim_t, ere_t, eim_t);

                // Esperar 2 ciclos de latencia
                @(posedge clk); #1;
                @(posedge clk); #1;

                if ($signed(o_phi_re) !== ref_phi_re || $signed(o_phi_im) !== ref_phi_im) begin
                    $display("  [C3][s=%0d] FAIL: phi_re=%0d exp=%0d  phi_im=%0d exp=%0d",
                        j, $signed(o_phi_re), ref_phi_re,
                        $signed(o_phi_im), ref_phi_im);
                    total_errs = total_errs + 1;
                end
            end

            if (total_errs == pre)
                $display("[CASO 3] PASS  errs=0  frames_lfsr=4");
            else
                $display("[CASO 3] FAIL  errs=%0d", total_errs-pre);
        end
    endtask

    // ============================================================
    // CASO 4 — Saturación
    // Entradas máximas → verificar que la salida no desborda
    // ============================================================
    task run_caso4;
        integer pre;
        begin
            $display("--- CASO 4: Saturacion con entradas maximas ---");
            $display("  MAX*MAX deberia saturar al maximo representable");
            pre = total_errs;

            rst=1; i_valid=0; i_start=0;
            repeat(3) @(posedge clk); #1;
            rst=0; @(posedge clk); #1;

            // MAX * MAX en Q17.10: 65535 * 65535 >> 10 → overflow seguro
            i_valid=1; i_start=1;
            i_xre = 17'sd32767;  i_xim = 17'sd32767;
            i_ere = 17'sd32767;  i_eim = 17'sd32767;
            @(posedge clk); #1;
            i_valid=0; i_start=0;

            // Esperar salida
            @(posedge clk); #1;
            @(posedge clk); #1;

            // PHI_re = MAX*MAX + MAX*MAX = overflow → MAX_VAL = 65535 (2^16-1 para Q17)
            // PHI_im = MAX*MAX - MAX*MAX = 0
            if (o_phi_re !== 17'sd65535) begin
                $display("  [C4] FAIL phi_re=%0d exp=65535 (saturacion MAX Q17)",
                    $signed(o_phi_re));
                total_errs = total_errs + 1;
            end
            if (o_phi_im !== 17'sd0) begin
                $display("  [C4] FAIL phi_im=%0d exp=0", $signed(o_phi_im));
                total_errs = total_errs + 1;
            end

            // MIN * MAX: (-65536) * 65535 + (-65536) * 65535 → MIN_VAL = -65536
            i_valid=1; i_start=1;
            i_xre = -17'sd65536; i_xim = -17'sd65536;
            i_ere =  17'sd65535; i_eim =  17'sd65535;
            @(posedge clk); #1;
            i_valid=0;
            @(posedge clk); #1;
            @(posedge clk); #1;

            if (o_phi_re !== -17'sd65536) begin
                $display("  [C4] FAIL phi_re=%0d exp=-65536 (saturacion MIN Q17)",
                    $signed(o_phi_re));
                total_errs = total_errs + 1;
            end

            if (total_errs == pre)
                $display("[CASO 4] PASS  saturacion OK  phi_re_max=32767 phi_re_min=-32768");
            else
                $display("[CASO 4] FAIL  errs=%0d", total_errs-pre);
        end
    endtask

    // ============================================================
    // CASO 5 — Reset
    // Aplica datos, luego resetea, verifica que las salidas son 0
    // ============================================================
    task run_caso5;
        integer pre;
        begin
            $display("--- CASO 5: Reset en medio de operacion ---");
            pre = total_errs;

            // Enviar algunos datos
            i_valid=1; i_start=1;
            i_xre=500; i_xim=300; i_ere=200; i_eim=100;
            @(posedge clk); #1;
            i_start=0;
            repeat(3) @(posedge clk); #1;

            // Reset
            rst=1; i_valid=0;
            @(posedge clk); #1; @(posedge clk); #1;

            // Verificar salidas en 0
            if (o_valid !== 0 || o_phi_re !== 0 || o_phi_im !== 0) begin
                $display("  [C5] FAIL: post-reset o_valid=%0d phi_re=%0d phi_im=%0d",
                    o_valid, $signed(o_phi_re), $signed(o_phi_im));
                total_errs = total_errs + 1;
            end else
                $display("  [C5] post-reset: o_valid=0 phi_re=0 phi_im=0 OK");

            rst=0; @(posedge clk); #1;

            // Frame limpio post-reset
            i_valid=1; i_start=1;
            i_xre=SCALE; i_xim=0; i_ere=SCALE; i_eim=0;
            @(posedge clk); #1;
            i_valid=0; i_start=0;
            @(posedge clk); #1; @(posedge clk); #1;

            // Debe dar PHI = 1+0j = 1024+0j
            if ($signed(o_phi_re) !== SCALE || $signed(o_phi_im) !== 0) begin
                $display("  [C5] FAIL post-reset: phi_re=%0d exp=%0d  phi_im=%0d exp=0",
                    $signed(o_phi_re), SCALE, $signed(o_phi_im));
                total_errs = total_errs + 1;
            end

            if (total_errs == pre)
                $display("[CASO 5] PASS  reset y recuperacion OK");
            else
                $display("[CASO 5] FAIL  errs=%0d", total_errs-pre);
        end
    endtask

    // ============================================================
    // MAIN
    // ============================================================
    initial begin
        total_errs = 0;
        i_valid=0; i_start=0;
        i_xre=0; i_xim=0; i_ere=0; i_eim=0;
        rst=1;
        @(posedge clk); #1;

        run_caso1; #20;
        run_caso2; #20;
        run_caso3; #20;
        run_caso4; #20;
        run_caso5; #20;

        $display("");
        $display("========================================");
        $display("[TB_GRADIENTE] RESUMEN FINAL");
        $display("========================================");
        $display("  o_valid  → 2 ciclos de latencia desde i_valid");
        $display("  o_start  → 2 ciclos de latencia desde i_start");
        $display("  o_phi_re → Xr*Er + Xi*Ei  (Q17.10 saturado)");
        $display("  o_phi_im → Xr*Ei - Xi*Er  (Q17.10 saturado)");
        $display("========================================");
        $display("[TOTAL]  errs=%0d  => %s",
            total_errs,
            (total_errs==0) ? "PASS: gradiente OK" : "FAIL");
        $display("========================================");
        $finish;
    end

    initial begin
        #500_000;
        $display("[TB_GRADIENTE] TIMEOUT");
        $finish;
    end

endmodule

`default_nettype wire
