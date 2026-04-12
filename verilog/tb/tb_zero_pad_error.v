`timescale 1ns/1ps
`default_nettype none

// ============================================================
// tb_zero_pad_error  v3  (ping-pong)
//
// Con ping-pong el módulo emite el frame N MIENTRAS recibe el
// frame N+1. Por eso el TB envía frames continuamente y captura
// la salida en paralelo con un monitor permanente.
//
// CASOS:
//   C1) 6 frames ramp consecutivos — verificar estructura y datos
//   C2) 4 frames LFSR — verificar datos con valores aleatorios
//   C3) Reset en medio — verificar recuperación
// ============================================================

module tb_zero_pad_error;

    localparam integer NB_W     = 9;
    localparam integer NFFT     = 32;
    localparam integer N        = NFFT / 2;
    localparam integer CLK_HALF = 5;

    reg                    clk, rst;
    reg                    i_valid, i_start;
    reg  signed [NB_W-1:0] i_eI, i_eQ;
    wire                   o_valid, o_start;
    wire signed [NB_W-1:0] o_eI, o_eQ;

    zero_pad_error #(.NB_W(NB_W), .NFFT(NFFT)) u_dut (
        .clk(clk), .rst(rst),
        .i_valid(i_valid), .i_start(i_start),
        .i_eI(i_eI), .i_eQ(i_eQ),
        .o_valid(o_valid), .o_start(o_start),
        .o_eI(o_eI), .o_eQ(o_eQ)
    );

    initial clk = 0;
    always #CLK_HALF clk = ~clk;

    integer total_errs;

    // ============================================================
    // Memoria de referencia — guardar lo que entró para comparar
    // con lo que sale NFFT ciclos después
    // Usamos una FIFO de frames: guardamos los frames enviados
    // ============================================================
    localparam integer MAX_FRAMES = 16;

    reg signed [NB_W-1:0] ref_fifo_I [0:MAX_FRAMES-1][0:N-1];
    reg signed [NB_W-1:0] ref_fifo_Q [0:MAX_FRAMES-1][0:N-1];
    integer ref_wr;   // próximo frame a escribir en la FIFO
    integer ref_rd;   // próximo frame a leer de la FIFO

    // ============================================================
    // Monitor permanente — verifica cada frame de salida
    // ============================================================
    integer mon_out_idx;    // posición dentro del frame de salida (0..2N-1)
    integer mon_start_cnt;
    integer mon_frame;
    integer mon_errs;
    integer armed;

    integer j;

    always @(posedge clk) begin : monitor
        if (rst) begin
            mon_out_idx  <= 0;
            mon_start_cnt<= 0;
            mon_frame    <= 0;
            armed        <= 0;
        end else if (o_valid) begin
            armed <= 1;

            if (o_start) begin
                mon_out_idx   <= 1;
                mon_start_cnt <= mon_start_cnt + 1;
            end else begin
                mon_out_idx <= mon_out_idx + 1;
            end

            // V1: primera mitad = 0
            if (mon_out_idx < N) begin
                if ($signed(o_eI) !== 0 || $signed(o_eQ) !== 0) begin
                    $display("[MON][F%0d][j=%0d] FAIL V1: eI=%0d eQ=%0d (exp 0)",
                        mon_frame, mon_out_idx,
                        $signed(o_eI), $signed(o_eQ));
                    mon_errs = mon_errs + 1;
                end
            end

            // V2: segunda mitad = ref_fifo[ref_rd]
            if (mon_out_idx >= N && mon_out_idx < NFFT) begin
                if (ref_rd < ref_wr) begin
                    j = mon_out_idx - N;
                    if ($signed(o_eI) !== $signed(ref_fifo_I[ref_rd % MAX_FRAMES][j]) ||
                        $signed(o_eQ) !== $signed(ref_fifo_Q[ref_rd % MAX_FRAMES][j])) begin
                        $display("[MON][F%0d][j=%0d] FAIL V2: eI=%0d exp=%0d  eQ=%0d exp=%0d",
                            mon_frame, j,
                            $signed(o_eI), $signed(ref_fifo_I[ref_rd % MAX_FRAMES][j]),
                            $signed(o_eQ), $signed(ref_fifo_Q[ref_rd % MAX_FRAMES][j]));
                        mon_errs = mon_errs + 1;
                    end
                end
            end

            // Fin de frame
            if (mon_out_idx == NFFT-1) begin
                if (mon_errs == 0 && armed)
                    $display("[MON][F%0d] PASS  zeros=%0d errors=%0d",
                        mon_frame, N, N);
                mon_frame <= mon_frame + 1;
                ref_rd    <= ref_rd + 1;
                mon_out_idx <= 0;
            end
        end
    end

    // ============================================================
    // TAREA: reset limpio
    // ============================================================
    task do_reset;
        integer k, m;
        begin
            rst=1; i_valid=0; i_start=0; i_eI=0; i_eQ=0;
            ref_wr=0; ref_rd=0; mon_out_idx=0;
            mon_start_cnt=0; mon_frame=0; mon_errs=0; armed=0;
            repeat(3) @(posedge clk); #1;
            rst=0; @(posedge clk); #1;
        end
    endtask

    // ============================================================
    // TAREA: enviar un frame y guardarlo en ref_fifo
    // ============================================================
    reg signed [NB_W-1:0] cur_I [0:N-1];
    reg signed [NB_W-1:0] cur_Q [0:N-1];
    integer fi;

    task send_frame;
        integer jj;
        begin
            // Guardar en ref_fifo para el monitor
            for (jj=0; jj<N; jj=jj+1) begin
                ref_fifo_I[ref_wr % MAX_FRAMES][jj] = cur_I[jj];
                ref_fifo_Q[ref_wr % MAX_FRAMES][jj] = cur_Q[jj];
            end
            ref_wr = ref_wr + 1;

            // Enviar al DUT
            for (jj=0; jj<N; jj=jj+1) begin
                i_eI    = cur_I[jj];
                i_eQ    = cur_Q[jj];
                i_valid = 1;
                i_start = (jj == 0);
                @(posedge clk); #1;
            end
            i_valid=0; i_start=0;

            // Gap de N ciclos — imita el slicer real
            // El slicer emite N muestras cada NFFT=2N ciclos,
            // dejando N ciclos de silencio entre frames.
            // Sin este gap el ZPE no puede mantener el ritmo
            // porque emitir un frame toma 2N ciclos.
            repeat(N) @(posedge clk); #1;
        end
    endtask

    // ============================================================
    // CASO 1 — 6 frames ramp consecutivos
    // ============================================================
    integer pre1;
    task run_caso1;
        integer jj, ff;
        begin
            $display("--- CASO 1: 6 frames ramp consecutivos ---");
            do_reset;
            pre1 = mon_errs;

            for (ff=0; ff<6; ff=ff+1) begin
                for (jj=0; jj<N; jj=jj+1) begin
                    cur_I[jj] = ff*N + jj + 1;
                    cur_Q[jj] = -(ff*N + jj + 1);
                end
                send_frame;
            end

            // Esperar que salgan todos los frames
            repeat(NFFT*2 + 10) @(posedge clk); #1;

            total_errs = total_errs + mon_errs - pre1;
            if (mon_errs == pre1)
                $display("[CASO 1] PASS  frames_verificados=%0d", mon_frame);
            else
                $display("[CASO 1] FAIL  errs=%0d", mon_errs-pre1);
        end
    endtask

    // ============================================================
    // CASO 2 — 4 frames LFSR
    // ============================================================
    reg [15:0] lfsr;
    reg [7:0]  tmp_i, tmp_q;
    integer pre2;

    task run_caso2;
        integer jj, ff;
        begin
            $display("--- CASO 2: 4 frames LFSR ---");
            lfsr = 16'hACE1;
            do_reset;
            pre2 = mon_errs;

            for (ff=0; ff<4; ff=ff+1) begin
                for (jj=0; jj<N; jj=jj+1) begin
                    lfsr  = {lfsr[14:0], lfsr[15]^lfsr[13]^lfsr[12]^lfsr[10]};
                    tmp_i = lfsr[7:0];
                    tmp_q = lfsr[15:8];
                    cur_I[jj] = {{(NB_W-7){tmp_i[7]}}, tmp_i[7:1]};
                    cur_Q[jj] = {{(NB_W-7){tmp_q[7]}}, tmp_q[7:1]};
                end
                send_frame;
            end

            repeat(NFFT*2 + 10) @(posedge clk); #1;

            total_errs = total_errs + mon_errs - pre2;
            if (mon_errs == pre2)
                $display("[CASO 2] PASS  frames_verificados=%0d", mon_frame);
            else
                $display("[CASO 2] FAIL  errs=%0d", mon_errs-pre2);
        end
    endtask

    // ============================================================
    // CASO 3 — Reset en medio
    // ============================================================
    integer pre3;
    task run_caso3;
        integer jj;
        begin
            $display("--- CASO 3: Reset en medio ---");
            pre3 = mon_errs;

            // Enviar 2 frames, resetear, luego 3 frames limpios
            for (jj=0; jj<N; jj=jj+1) begin
                cur_I[jj] = 9'sd50; cur_Q[jj] = -9'sd50;
            end
            send_frame;
            send_frame;

            rst=1; i_valid=0; i_eI=0; i_eQ=0;
            @(posedge clk); #1; @(posedge clk); #1;

            if (o_valid !== 0 || o_eI !== 0 || o_eQ !== 0) begin
                $display("[CASO 3] FAIL: salidas no cero post-reset");
                total_errs = total_errs + 1;
            end else
                $display("[CASO 3] salida=0 post-reset: OK");

            rst=0; @(posedge clk); #1;
            // Reinicializar contadores del monitor
            ref_wr=0; ref_rd=0; mon_out_idx=0;
            mon_frame=0; armed=0;

            for (jj=0; jj<N; jj=jj+1) begin
                cur_I[jj] = jj+1; cur_Q[jj] = -(jj+1);
            end
            send_frame;
            send_frame;
            send_frame;

            repeat(NFFT*2 + 10) @(posedge clk); #1;

            total_errs = total_errs + mon_errs - pre3;
            if (mon_errs == pre3)
                $display("[CASO 3] PASS  post-reset frames_verificados=%0d", mon_frame);
            else
                $display("[CASO 3] FAIL  errs=%0d", mon_errs-pre3);
        end
    endtask

    // ============================================================
    // MAIN
    // ============================================================
    initial begin
        total_errs = 0;
        mon_errs   = 0;
        ref_wr=0; ref_rd=0;
        i_valid=0; i_start=0; i_eI=0; i_eQ=0; rst=1;
        @(posedge clk); #1;

        run_caso1; #20;
        run_caso2; #20;
        run_caso3; #20;

        $display("");
        $display("========================================");
        $display("[TB_ZERO_PAD_ERROR] RESUMEN FINAL  v3");
        $display("========================================");
        $display("[TOTAL]  errs=%0d  => %s",
            total_errs,
            (total_errs==0) ? "PASS: zero_pad_error OK" : "FAIL");
        $display("========================================");
        $finish;
    end

    initial begin
        #500_000;
        $display("[TB_ZERO_PAD_ERROR] TIMEOUT");
        $finish;
    end

endmodule

`default_nettype wire
