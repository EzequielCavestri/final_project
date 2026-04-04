`timescale 1ns/1ps
`default_nettype none

// ============================================================
// tb_discard_n   —   Testbench unitario de discard_n
//
// El módulo recibe un stream de NFFT=32 muestras por frame.
// Debe descartar las primeras N=16 (índices 0..15) y pasar
// las últimas N=16 (índices 16..31) sin modificar su valor.
//
// CASOS
// -----
// CASO 1 — Ramp conocida (1 frame)
//   Entrada: I = índice (0..31),  Q = índice - 16 (-16..15)
//   Verifica:
//     * Exactamente N=16 salidas válidas
//     * o_start=1 solo en la primera salida válida (idx=16)
//     * Cada salida o_yI[j] == (N+j), o_yQ[j] == (j) para j=0..15
//
// CASO 2 — 4 frames consecutivos sin gap (datos LFSR)
//   Verifica que en frames continuos cada frame produce
//   exactamente N salidas y un solo o_start.
//
// CASO 3 — Gap de 20 ciclos entre frames (i_valid=0)
//   Verifica que el contador interno no avanza durante el gap
//   y el frame siguiente se procesa correctamente.
//
// CASO 4 — Reset en medio de un frame
//   Verifica recuperación limpia: post-reset el primer frame
//   completo produce exactamente N salidas correctas.
//
// ESTRATEGIA: secuencial pura (igual que tb_cmul_pbfdaf).
//   Loop: presentar muestra[n], avanzar @(posedge clk), leer #1.
// ============================================================

module tb_discard_n;

    // ----------------------------------------------------------
    // Parámetros
    // ----------------------------------------------------------
    localparam integer NB_W = 9;
    localparam integer NBF_W = 7;
    localparam integer NFFT  = 32;
    localparam integer N     = NFFT / 2;   // 16
    localparam integer KW    = 5;          // $clog2(32)

    // ----------------------------------------------------------
    // Clock 100 MHz
    // ----------------------------------------------------------
    reg clk;
    initial begin clk = 1'b0; forever #5 clk = ~clk; end

    // ----------------------------------------------------------
    // Reset
    // ----------------------------------------------------------
    reg rst;
    initial begin
        rst = 1'b1;
        repeat(10) @(posedge clk);
        @(posedge clk); #1;
        rst = 1'b0;
    end

    // ----------------------------------------------------------
    // Puertos DUT
    // ----------------------------------------------------------
    reg                    i_valid;
    reg                    i_start;
    reg  signed [NB_W-1:0] i_yI;
    reg  signed [NB_W-1:0] i_yQ;

    wire                   o_valid;
    wire                   o_start;
    wire signed [NB_W-1:0] o_yI;
    wire signed [NB_W-1:0] o_yQ;
    wire        [KW-1:0]   o_samp_idx;

    // ----------------------------------------------------------
    // DUT
    // ----------------------------------------------------------
    discard_n #(
        .NB_W (NB_W),
        .NBF_W(NBF_W),
        .NFFT (NFFT)
    ) dut (
        .clk       (clk),
        .rst       (rst),
        .i_valid   (i_valid),
        .i_start   (i_start),
        .i_yI      (i_yI),
        .i_yQ      (i_yQ),
        .o_valid   (o_valid),
        .o_start   (o_start),
        .o_yI      (o_yI),
        .o_yQ      (o_yQ),
        .o_samp_idx(o_samp_idx)
    );

    // ----------------------------------------------------------
    // Buffers de trabajo
    // ----------------------------------------------------------
    reg signed [NB_W-1:0] in_I   [0:NFFT-1];
    reg signed [NB_W-1:0] in_Q   [0:NFFT-1];
    reg signed [NB_W-1:0] cap_I  [0:N-1];
    reg signed [NB_W-1:0] cap_Q  [0:N-1];
    reg                   cap_st [0:N-1];

    // Contador de errores global
    integer total_errs;

    // LFSR 16 bits
    reg [15:0] lfsr;

    task lfsr_step;
        reg [15:0] t;
        begin t = lfsr ^ (lfsr << 3) ^ (lfsr >> 13); lfsr = t; end
    endtask

    task fill_lfsr;
        integer k;
        reg signed [7:0] tmp_i, tmp_q;
        begin
            for (k = 0; k < NFFT; k = k + 1) begin
                lfsr_step();
                tmp_i = $signed(lfsr[7:0]);
                in_I[k] = {{(NB_W-8){tmp_i[7]}}, tmp_i[7:1]};  // sign-extend + >>1
                lfsr_step();
                tmp_q = $signed(lfsr[7:0]);
                in_Q[k] = {{(NB_W-8){tmp_q[7]}}, tmp_q[7:1]};
            end
        end
    endtask

    // ----------------------------------------------------------
    // Tarea send_frame
    //   Envía un frame completo (NFFT muestras) y captura las
    //   salidas válidas. Retorna conteos y errores.
    //
    //   Timing (latencia 1 ciclo del DUT):
    //     Presentar muestra[n] en flanco T  →
    //     Leer salida de muestra[n] en flanco T+1, leída #1 tarde.
    // ----------------------------------------------------------
    task send_frame;
        input  integer fid;        // frame id para mensajes
        output integer out_cnt;    // salidas válidas capturadas
        output integer start_cnt;  // pulsos o_start
        output integer errs;       // errores de este frame
        integer n, dI, dQ;
        begin
            out_cnt   = 0;
            start_cnt = 0;
            errs      = 0;

            // --- Muestra 0 (i_start=1) ---
            @(posedge clk);
            i_valid = 1'b1; i_start = 1'b1;
            i_yI = in_I[0]; i_yQ = in_Q[0];

            // --- Muestras 1 .. NFFT-1: presentar y leer anterior ---
            for (n = 1; n < NFFT; n = n + 1) begin
                @(posedge clk); #1;
                if (o_valid) begin
                    cap_I [out_cnt] = o_yI;
                    cap_Q [out_cnt] = o_yQ;
                    cap_st[out_cnt] = o_start;
                    if (o_start) start_cnt = start_cnt + 1;
                    out_cnt = out_cnt + 1;
                end
                i_valid = 1'b1; i_start = 1'b0;
                i_yI = in_I[n]; i_yQ = in_Q[n];
            end

            // --- Ciclo extra: capturar última muestra ---
            @(posedge clk); #1;
            i_valid = 1'b0; i_start = 1'b0;
            if (o_valid) begin
                cap_I [out_cnt] = o_yI;
                cap_Q [out_cnt] = o_yQ;
                cap_st[out_cnt] = o_start;
                if (o_start) start_cnt = start_cnt + 1;
                out_cnt = out_cnt + 1;
            end

            // --- Verificaciones ---

            // V1: exactamente N salidas válidas
            if (out_cnt !== N) begin
                $display("[DN][F%0d] FAIL V1 valid_cnt=%0d esperado=%0d",
                    fid, out_cnt, N);
                errs = errs + 1; total_errs = total_errs + 1;
            end

            // V2: exactamente 1 pulso o_start
            if (start_cnt !== 1) begin
                $display("[DN][F%0d] FAIL V2 start_cnt=%0d esperado=1", fid, start_cnt);
                errs = errs + 1; total_errs = total_errs + 1;
            end

            // V3: o_start en la PRIMERA salida válida
            if (out_cnt > 0 && cap_st[0] !== 1'b1) begin
                $display("[DN][F%0d] FAIL V3 cap_st[0]=%0b esperado=1", fid, cap_st[0]);
                errs = errs + 1; total_errs = total_errs + 1;
            end

            // V4: datos == segunda mitad de la entrada
            if (out_cnt == N) begin
                for (n = 0; n < N; n = n + 1) begin
                    dI = $signed(cap_I[n]) - $signed(in_I[N+n]);
                    dQ = $signed(cap_Q[n]) - $signed(in_Q[N+n]);
                    if (dI !== 0 || dQ !== 0) begin
                        $display("[DN][F%0d][j=%0d] FAIL V4 I=%0d Q=%0d expI=%0d expQ=%0d",
                            fid, n,
                            $signed(cap_I[n]),  $signed(cap_Q[n]),
                            $signed(in_I[N+n]), $signed(in_Q[N+n]));
                        errs = errs + 1; total_errs = total_errs + 1;
                    end
                end
            end
        end
    endtask

    // ----------------------------------------------------------
    // Variables de loop
    // ----------------------------------------------------------
    integer n_v, f_v;
    integer oc, sc, ce, ct;

    // ==========================================================
    // SECUENCIA PRINCIPAL
    // ==========================================================
    initial begin
        i_valid = 1'b0; i_start = 1'b0;
        i_yI = {NB_W{1'b0}}; i_yQ = {NB_W{1'b0}};
        total_errs = 0;
        lfsr = 16'hACE1;

        @(negedge rst);
        repeat(5) @(posedge clk);

        // ==============================================================
        // CASO 1 — Ramp conocida
        // ==============================================================
        $display("\n--- CASO 1: Ramp conocida ---");
        for (n_v = 0; n_v < NFFT; n_v = n_v + 1) begin
            in_I[n_v] = n_v[NB_W-1:0];
            in_Q[n_v] = $signed(n_v) - 16;
        end
        send_frame(0, oc, sc, ce);

        // Verificación visual de los bins (solo imprime desajustes)
        if (oc == N) begin
            for (n_v = 0; n_v < N; n_v = n_v + 1) begin
                if ($signed(cap_I[n_v]) !== (N + n_v) ||
                    $signed(cap_Q[n_v]) !== n_v) begin
                    $display("  [C1] bin[%0d] I=%0d(exp %0d) Q=%0d(exp %0d)",
                        n_v, $signed(cap_I[n_v]), N+n_v,
                        $signed(cap_Q[n_v]), n_v);
                end
            end
        end
        $display("[CASO 1] valid=%0d/%0d  start=%0d  errs=%0d  => %s",
            oc, N, sc, ce, ce==0 ? "PASS" : "FAIL");

        repeat(5) @(posedge clk);

        // ==============================================================
        // CASO 2 — 4 frames consecutivos (LFSR)
        // ==============================================================
        $display("\n--- CASO 2: 4 frames consecutivos (LFSR) ---");
        ct = 0;
        for (f_v = 0; f_v < 4; f_v = f_v + 1) begin
            fill_lfsr();
            send_frame(f_v, oc, sc, ce);
            ct = ct + ce;
            $display("  [F%0d] valid=%0d  start=%0d  errs=%0d  => %s",
                f_v, oc, sc, ce, ce==0 ? "PASS" : "FAIL");
        end
        $display("[CASO 2] total_case_errs=%0d  => %s", ct, ct==0 ? "PASS" : "FAIL");

        repeat(5) @(posedge clk);

        // ==============================================================
        // CASO 3 — Gap de 20 ciclos entre frames
        // ==============================================================
        $display("\n--- CASO 3: Gap de 20 ciclos ---");
        ct = 0;

        fill_lfsr();
        send_frame(0, oc, sc, ce);
        ct = ct + ce;
        $display("  [Frame A] valid=%0d  errs=%0d", oc, ce);

        repeat(20) @(posedge clk);   // gap: i_valid ya está en 0

        fill_lfsr();
        send_frame(1, oc, sc, ce);
        ct = ct + ce;
        $display("  [Frame B] valid=%0d  errs=%0d", oc, ce);

        $display("[CASO 3] total_case_errs=%0d  => %s", ct, ct==0 ? "PASS" : "FAIL");

        repeat(5) @(posedge clk);

        // ==============================================================
        // CASO 4 — Reset en medio de un frame
        // ==============================================================
        $display("\n--- CASO 4: Reset en medio de frame ---");
        ct = 0;

        // Enviar 8 muestras y luego resetear
        fill_lfsr();
        @(posedge clk);
        i_valid = 1'b1; i_start = 1'b1; i_yI = in_I[0]; i_yQ = in_Q[0];
        repeat(7) begin
            @(posedge clk);
            i_valid = 1'b1; i_start = 1'b0;
        end
        // Forzar reset
        @(posedge clk);
        rst = 1'b1; i_valid = 1'b0; i_start = 1'b0;
        repeat(8) @(posedge clk);
        rst = 1'b0;
        repeat(4) @(posedge clk);

        // Frame completo post-reset
        fill_lfsr();
        send_frame(0, oc, sc, ce);
        ct = ct + ce;
        $display("[CASO 4] post-reset valid=%0d/%0d  start=%0d  errs=%0d  => %s",
            oc, N, sc, ce, ct==0 ? "PASS" : "FAIL");

        repeat(5) @(posedge clk);

        // ==============================================================
        // RESUMEN FINAL
        // ==============================================================
        $display("\n========================================");
        $display("[TB_DISCARD_N] RESUMEN FINAL");
        $display("========================================");
        $display("[TOTAL]  total_errs=%0d  => %s",
            total_errs,
            total_errs==0 ? "PASS: discard_n OK" : "FAIL: revisar DUT");
        $display("========================================");
        $finish;
    end

    // Watchdog
    initial begin
        #2_000_000;
        $display("[TB_DISCARD_N] TIMEOUT sin terminar");
        $finish;
    end

endmodule

`default_nettype wire
