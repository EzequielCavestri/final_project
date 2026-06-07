`timescale 1ns/1ps
`default_nettype none

// ============================================================
// tb_xhist_delay  v2
// Testbench con mensajes claros en cada paso
// ============================================================

module tb_xhist_delay;

    localparam integer NB_W     = 17;
    localparam integer DELAY    = 118;
    localparam integer NFFT     = 32;
    localparam integer CLK_HALF = 5;

    // ---- DUT ----
    reg                    clk, rst;
    reg                    i_valid, i_start;
    reg  signed [NB_W-1:0] i_xre, i_xim;
    wire                   o_valid, o_start;
    wire signed [NB_W-1:0] o_xre, o_xim;

    xhist_delay #(.NB_W(NB_W), .DELAY(DELAY)) u_dut (
        .clk(clk), .rst(rst),
        .i_valid(i_valid), .i_start(i_start),
        .i_xre(i_xre), .i_xim(i_xim),
        .o_valid(o_valid), .o_start(o_start),
        .o_xre(o_xre), .o_xim(o_xim)
    );

    initial clk = 0;
    always #CLK_HALF clk = ~clk;

    integer total_errs;

    // ============================================================
    // Modelo de referencia — shift register igual al DUT
    // Se usa para saber qué debería salir en cada ciclo
    // ============================================================
    reg signed [NB_W-1:0] ref_re [0:DELAY];
    reg signed [NB_W-1:0] ref_im [0:DELAY];
    reg                   ref_v  [0:DELAY];
    reg                   ref_s  [0:DELAY];
    integer cycle_cnt;
    integer ref_k;  // índice para el for loop del modelo

    always @(posedge clk) begin
        if (rst) begin
            for (ref_k=0; ref_k<=DELAY; ref_k=ref_k+1) begin
                ref_re[ref_k]<=0; ref_im[ref_k]<=0;
                ref_v[ref_k] <=0; ref_s[ref_k] <=0;
            end
            cycle_cnt <= 0;
        end else begin
            ref_re[0] <= i_xre;   ref_im[0] <= i_xim;
            ref_v [0] <= i_valid; ref_s [0] <= i_start;
            for (ref_k=1; ref_k<=DELAY; ref_k=ref_k+1) begin
                ref_re[ref_k] <= ref_re[ref_k-1]; ref_im[ref_k] <= ref_im[ref_k-1];
                ref_v [ref_k] <= ref_v [ref_k-1]; ref_s [ref_k] <= ref_s [ref_k-1];
            end
            cycle_cnt <= cycle_cnt + 1;
        end
    end

    // ============================================================
    // Monitor permanente — compara DUT contra modelo ciclo a ciclo
    // Solo activo después de que el shift register se llenó
    // ============================================================
    integer mon_errs;

    always @(posedge clk) begin : monitor
        if (!rst && cycle_cnt > DELAY) begin
            if (o_xre   !== ref_re[DELAY] ||
                o_xim   !== ref_im[DELAY] ||
                o_valid !== ref_v [DELAY] ||
                o_start !== ref_s [DELAY]) begin
                $display("  [ERROR ciclo %0d] salida: re=%0d im=%0d valid=%0d start=%0d",
                    cycle_cnt,
                    $signed(o_xre), $signed(o_xim), o_valid, o_start);
                $display("  [ERROR ciclo %0d] expect: re=%0d im=%0d valid=%0d start=%0d",
                    cycle_cnt,
                    $signed(ref_re[DELAY]), $signed(ref_im[DELAY]),
                    ref_v[DELAY], ref_s[DELAY]);
                mon_errs = mon_errs + 1;
            end
        end
    end

    // ============================================================
    // TAREA: reset limpio
    // ============================================================
    task do_reset;
        begin
            rst=1; i_valid=0; i_start=0; i_xre=0; i_xim=0;
            repeat(3) @(posedge clk); #1;
            rst=0;
            @(posedge clk); #1;
        end
    endtask

    // ============================================================
    // CASO 1 — Ramp conocida
    // Envía valores simples (1,2,3...) para que sea fácil
    // verificar a mano si algo falla
    // ============================================================
    task run_caso1;
        integer j, pre;
        begin
            $display("");
            $display("--- CASO 1: Ramp conocida ---");
            $display("  Enviando %0d frames con valores re=1,2,3... im=-1,-2,-3...", 6);
            $display("  El shift register tiene %0d posiciones", DELAY);
            $display("  Los datos deben salir exactamente %0d ciclos despues", DELAY);
            do_reset;
            pre = mon_errs;

            // Enviar 6 frames de datos conocidos
            for (j=0; j<6*NFFT; j=j+1) begin
                i_xre   = j + 1;
                i_xim   = -(j + 1);
                i_valid = 1;
                i_start = (j % NFFT == 0);
                @(posedge clk); #1;
            end
            i_valid=0; i_start=0; i_xre=0; i_xim=0;

            $display("  Esperando %0d ciclos para que los datos salgan...", DELAY);
            repeat(DELAY + 10) @(posedge clk); #1;

            if (mon_errs == pre) begin
                $display("[CASO 1] PASS => delay de %0d ciclos verificado correctamente", DELAY);
            end else begin
                $display("[CASO 1] FAIL => %0d errores detectados", mon_errs - pre);
                total_errs = total_errs + (mon_errs - pre);
            end
        end
    endtask

    // ============================================================
    // CASO 2 — 4 frames LFSR
    // Datos pseudo-aleatorios para cubrir más combinaciones
    // ============================================================
    reg [15:0] lfsr;

    task run_caso2;
        integer j, f, pre;
        reg [7:0] tmp_re, tmp_im;
        begin
            $display("");
            $display("--- CASO 2: 4 frames con datos LFSR (pseudo-aleatorios) ---");
            $display("  Verifica que el delay funciona con datos variables");
            lfsr = 16'hACE1;
            do_reset;
            pre = mon_errs;

            for (f=0; f<4; f=f+1) begin
                $display("  Enviando frame %0d...", f);
                for (j=0; j<NFFT; j=j+1) begin
                    lfsr    = {lfsr[14:0], lfsr[15]^lfsr[13]^lfsr[12]^lfsr[10]};
                    tmp_re  = lfsr[7:0];
                    tmp_im  = lfsr[15:8];
                    i_xre   = {{(NB_W-8){tmp_re[7]}}, tmp_re};
                    i_xim   = {{(NB_W-8){tmp_im[7]}}, tmp_im};
                    i_valid = 1;
                    i_start = (j==0);
                    @(posedge clk); #1;
                end
            end
            i_valid=0; i_start=0; i_xre=0; i_xim=0;

            $display("  Esperando que los datos salgan...");
            repeat(DELAY + 10) @(posedge clk); #1;

            if (mon_errs == pre) begin
                $display("[CASO 2] PASS => 4 frames LFSR correctamente retrasados");
            end else begin
                $display("[CASO 2] FAIL => %0d errores detectados", mon_errs - pre);
                total_errs = total_errs + (mon_errs - pre);
            end
        end
    endtask

    // ============================================================
    // CASO 3 — Reset en medio
    // Verifica que despues de un reset todo vuelve a 0
    // ============================================================
    task run_caso3;
        integer j, pre;
        begin
            $display("");
            $display("--- CASO 3: Reset en medio de operacion ---");
            $display("  Enviando 20 muestras, luego reset, luego frame limpio");
            pre = mon_errs;

            // Enviar algunos datos
            $display("  Enviando 20 muestras...");
            for (j=0; j<20; j=j+1) begin
                i_xre=j; i_xim=-j; i_valid=1; i_start=(j==0);
                @(posedge clk); #1;
            end

            // Aplicar reset
            $display("  Aplicando reset...");
            rst=1; i_valid=0; i_xre=0; i_xim=0;
            @(posedge clk); #1;
            @(posedge clk); #1;

            // Verificar que la salida es 0
            if (o_valid !== 0 || o_xre !== 0 || o_xim !== 0) begin
                $display("  [ERROR] Salida no es cero despues del reset");
                $display("         o_valid=%0d o_xre=%0d o_xim=%0d",
                    o_valid, $signed(o_xre), $signed(o_xim));
                total_errs = total_errs + 1;
            end else begin
                $display("  Salida = 0 post-reset: OK");
            end

            // Reanudar con frame limpio
            $display("  Enviando frame limpio post-reset...");
            rst=0; @(posedge clk); #1;
            for (j=0; j<NFFT; j=j+1) begin
                i_xre=j*2; i_xim=-j; i_valid=1; i_start=(j==0);
                @(posedge clk); #1;
            end
            i_valid=0; i_start=0; i_xre=0; i_xim=0;

            repeat(DELAY+10) @(posedge clk); #1;

            if (mon_errs == pre) begin
                $display("[CASO 3] PASS => reset y recuperacion correctos");
            end else begin
                $display("[CASO 3] FAIL => %0d errores detectados", mon_errs - pre);
                total_errs = total_errs + (mon_errs - pre);
            end
        end
    endtask

    // ============================================================
    // MAIN
    // ============================================================
    initial begin
        total_errs = 0;
        mon_errs   = 0;
        i_valid=0; i_start=0; i_xre=0; i_xim=0; rst=1;
        @(posedge clk); #1;

        $display("========================================");
        $display("[TB] xhist_delay — DELAY=%0d ciclos", DELAY);
        $display("========================================");

        run_caso1;
        run_caso2;
        run_caso3;

        $display("");
        $display("========================================");
        $display("[TB_XHIST_DELAY] RESUMEN FINAL");
        $display("========================================");
        if (total_errs == 0)
            $display("[TOTAL] errs=0 => PASS: xhist_delay OK");
        else
            $display("[TOTAL] errs=%0d => FAIL", total_errs);
        $display("========================================");
        $finish;
    end

    initial begin
        #2_000_000;
        $display("[TB_XHIST_DELAY] TIMEOUT — algo se colgó");
        $finish;
    end

endmodule

`default_nettype wire
