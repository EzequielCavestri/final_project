`timescale 1ns/1ps
`default_nettype none

// ============================================================
// tb_cmul_pbfdaf — Testbench unitario de cmul_pbfdaf
//
// Verifica: Y[k] = W0[k]·X0[k] + W1[k]·X1[k]
//
// CASOS DE PRUEBA
// ---------------
// CASO 1: W0=1+0j, W1=0  (inicialización por defecto)
//   Y = X_curr  (identidad sobre X0, X1 ignorado)
//
// CASO 2: W0=0, W1=1+0j
//   Y = X_old   (identidad sobre X1, X0 ignorado)
//
// CASO 3: W0=0.5+0j, W1=0.5+0j
//   Y = 0.5·X_curr + 0.5·X_old  (promedio real de ambos taps)
//
// CASO 4: W0=0+1j, W1=0-1j
//   Y_re = -X0_im + X1_im
//   Y_im =  X0_re - X1_re
//   Verifica que Re e Im de cada tap se mezclan correctamente.
//
// CASO 5: Puerto LMS — escritura de pesos via i_we
//   Frame A: W0=1+0j, W1=0 (default) → Y = X_curr
//   Escribir W0[k]=0 para todo k mediante i_we durante idle
//   Frame B: Y = 0  (pesos cero verifican que LMS escribió)
//
// ESTRATEGIA: secuencial pura (igual que tb_cmul verificado).
//   Pre-genera in_mem_X0 e in_mem_X1.
//   En el loop de envío: al ciclo N presenta muestra N,
//   lee con #1 la salida del ciclo N (= resultado de muestra N-1).
//   Un ciclo extra captura la última muestra.
//
// FORMATO PUNTO FIJO: Q(NB_W=17, NBF_W=10)
//   1.0 = 1024, 0.5 = 512, -1.0 = -1024
//   Tolerancia: 2 LSB (truncamiento en sat_trunc)
// ============================================================

module tb_cmul_pbfdaf;

    // -------------------------------------------------------
    // Parámetros
    // -------------------------------------------------------
    localparam integer NB_W  = 17;
    localparam integer NBF_W = 10;
    localparam integer NFFT  = 32;
    localparam integer KW    = 5;   // $clog2(32)

    localparam signed [NB_W-1:0] ONE  =  17'sd1024;
    localparam signed [NB_W-1:0] MONE = -17'sd1024;
    localparam signed [NB_W-1:0] HALF =  17'sd512;
    localparam signed [NB_W-1:0] ZERO =  17'sd0;
    localparam signed [NB_W-1:0] IONE =  17'sd1024;  // 0+1j: parte imag = 1.0

    localparam integer TOL             = 2;
    localparam integer FRAMES_PER_CASE = 4;
    localparam integer TOTAL_SAMP      = NFFT * FRAMES_PER_CASE;

    // -------------------------------------------------------
    // Clock y reset
    // -------------------------------------------------------
    reg clk;
    initial begin clk = 1'b0; forever #5 clk = ~clk; end

    reg rst;
    initial begin
        rst = 1'b1;
        repeat(10) @(posedge clk);
        rst = 1'b0;
    end

    // -------------------------------------------------------
    // Señales DUT
    // -------------------------------------------------------
    reg                      i_valid;
    reg                      i_start;
    reg  signed [NB_W-1:0]  i_X0_re, i_X0_im;   // X_curr
    reg  signed [NB_W-1:0]  i_X1_re, i_X1_im;   // X_old

    reg                      i_we;
    reg  [KW-1:0]            i_wk;
    reg                      i_wsel;
    reg  signed [NB_W-1:0]  i_W_re, i_W_im;

    wire                     o_valid;
    wire                     o_start;
    wire signed [NB_W-1:0]  o_yI, o_yQ;
    wire [KW-1:0]            o_samp_idx;

    // -------------------------------------------------------
    // DUT
    // -------------------------------------------------------
    cmul_pbfdaf #(
        .NB_W (NB_W),
        .NBF_W(NBF_W),
        .NFFT (NFFT)
    ) dut (
        .clk      (clk),
        .rst      (rst),
        .i_valid  (i_valid),
        .i_start  (i_start),
        .i_X0_re  (i_X0_re), .i_X0_im(i_X0_im),
        .i_X1_re  (i_X1_re), .i_X1_im(i_X1_im),
        .i_we     (i_we),
        .i_wk     (i_wk),
        .i_wsel   (i_wsel),
        .i_W_re   (i_W_re),
        .i_W_im   (i_W_im),
        .o_valid  (o_valid),
        .o_start  (o_start),
        .o_yI     (o_yI),
        .o_yQ     (o_yQ),
        .o_samp_idx(o_samp_idx)
    );

    // -------------------------------------------------------
    // Funciones auxiliares
    // -------------------------------------------------------
    function integer iabs;
        input integer v;
        begin iabs = (v < 0) ? -v : v; end
    endfunction

    // Replica sat_trunc: producto en 2*NBF_W fraccionarios → NB_W bits
    function integer sat_q;
        input integer full_val;
        integer shifted, maxv, minv;
        begin
            shifted = full_val >>> NBF_W;
            maxv =  (1 << (NB_W-1)) - 1;
            minv = -(1 << (NB_W-1));
            if      (shifted >  maxv) sat_q =  maxv;
            else if (shifted <  minv) sat_q =  minv;
            else                      sat_q =  shifted;
        end
    endfunction

    // Suma y satura a NB_W bits (para el acumulador M0+M1)
    function integer sat_sum;
        input integer a, b;
        integer s, maxv, minv;
        begin
            s    = a + b;
            maxv =  (1 << (NB_W-1)) - 1;
            minv = -(1 << (NB_W-1));
            if      (s >  maxv) sat_sum =  maxv;
            else if (s <  minv) sat_sum =  minv;
            else                sat_sum =  s;
        end
    endfunction

    // -------------------------------------------------------
    // Memorias de entrada (dos canales: X0=curr, X1=old)
    // -------------------------------------------------------
    reg signed [NB_W-1:0] mem_X0_re [0:TOTAL_SAMP-1];
    reg signed [NB_W-1:0] mem_X0_im [0:TOTAL_SAMP-1];
    reg signed [NB_W-1:0] mem_X1_re [0:TOTAL_SAMP-1];
    reg signed [NB_W-1:0] mem_X1_im [0:TOTAL_SAMP-1];

    // Pesos activos para cálculo del expected
    reg signed [NB_W-1:0] act_w0_re, act_w0_im;
    reg signed [NB_W-1:0] act_w1_re, act_w1_im;

    // Contadores
    integer total_errs, case_errs, case_num;

    // -------------------------------------------------------
    // load_weights: carga W0 y W1 iguales para todos los bins
    //   (los pesos por bin se pueden variar con i_we si se necesita)
    // -------------------------------------------------------
    task load_weights;
        input signed [NB_W-1:0] w0r, w0i, w1r, w1i;
        integer k;
        begin
            for (k = 0; k < NFFT; k = k + 1) begin
                dut.W0_re[k] = w0r;  dut.W0_im[k] = w0i;
                dut.W1_re[k] = w1r;  dut.W1_im[k] = w1i;
            end
            act_w0_re = w0r;  act_w0_im = w0i;
            act_w1_re = w1r;  act_w1_im = w1i;
        end
    endtask

    // -------------------------------------------------------
    // gen_data: llena mem_X0 y mem_X1 con dos LFSR distintos
    //   (seeds diferentes para X0 y X1 → datos independientes)
    // -------------------------------------------------------
    task gen_data;
        integer n, l0, l1, t;
        begin
            l0 = 16'hACE1;
            l1 = 16'h1234;
            for (n = 0; n < TOTAL_SAMP; n = n + 1) begin
                t  = l0 ^ (l0 << 3) ^ (l0 >> 5); l0 = t & 16'hFFFF;
                mem_X0_re[n] = ((l0 % 512) - 256);
                t  = l0 ^ (l0 << 7) ^ (l0 >> 2); l0 = t & 16'hFFFF;
                mem_X0_im[n] = ((l0 % 512) - 256);

                t  = l1 ^ (l1 << 3) ^ (l1 >> 5); l1 = t & 16'hFFFF;
                mem_X1_re[n] = ((l1 % 512) - 256);
                t  = l1 ^ (l1 << 7) ^ (l1 >> 2); l1 = t & 16'hFFFF;
                mem_X1_im[n] = ((l1 % 512) - 256);
            end
        end
    endtask

    // -------------------------------------------------------
    // expected: calcula Y esperado para la muestra n
    //   Y[k] = W0·X0 + W1·X1
    //   = (w0r+j·w0i)·(x0r+j·x0i) + (w1r+j·w1i)·(x1r+j·x1i)
    //
    // M0_re = sat(w0r*x0r - w0i*x0i)
    // M0_im = sat(w0r*x0i + w0i*x0r)
    // M1_re = sat(w1r*x1r - w1i*x1i)
    // M1_im = sat(w1r*x1i + w1i*x1r)
    // Y_re  = sat_sum(M0_re, M1_re)
    // Y_im  = sat_sum(M0_im, M1_im)
    // -------------------------------------------------------
    task expected;
        input  integer n;
        output integer eyI, eyQ;
        integer x0r, x0i, x1r, x1i;
        integer m0r, m0i, m1r, m1i;
        begin
            x0r = $signed(mem_X0_re[n]);
            x0i = $signed(mem_X0_im[n]);
            x1r = $signed(mem_X1_re[n]);
            x1i = $signed(mem_X1_im[n]);

            m0r = sat_q($signed(act_w0_re)*x0r - $signed(act_w0_im)*x0i);
            m0i = sat_q($signed(act_w0_re)*x0i + $signed(act_w0_im)*x0r);
            m1r = sat_q($signed(act_w1_re)*x1r - $signed(act_w1_im)*x1i);
            m1i = sat_q($signed(act_w1_re)*x1i + $signed(act_w1_im)*x1r);

            eyI = sat_sum(m0r, m1r);
            eyQ = sat_sum(m0i, m1i);
        end
    endtask

    // -------------------------------------------------------
    // run_case: envía TOTAL_SAMP muestras y verifica salidas.
    //
    // Mismo esquema que tb_cmul verificado:
    //   ciclo N → presenta muestra N
    //   ciclo N+1 (con #1) → lee salida de muestra N-1
    //   +1 ciclo extra para capturar última muestra
    // -------------------------------------------------------
    task run_case;
        input integer cnum;
        integer n, out_ptr;
        integer eyI, eyQ, dI, dQ;
        begin
            case_num  = cnum;
            case_errs = 0;
            out_ptr   = 0;

            // Muestra 0: primer flanco
            @(posedge clk);
            i_valid = 1'b1;
            i_start = 1'b1;
            i_X0_re = mem_X0_re[0]; i_X0_im = mem_X0_im[0];
            i_X1_re = mem_X1_re[0]; i_X1_im = mem_X1_im[0];

            // Muestras 1 .. TOTAL_SAMP-1
            for (n = 1; n < TOTAL_SAMP; n = n + 1) begin
                @(posedge clk); #1;

                // Leer salida de muestra n-1
                if (o_valid) begin
                    expected(out_ptr, eyI, eyQ);
                    dI = $signed(o_yI) - eyI;
                    dQ = $signed(o_yQ) - eyQ;
                    if (iabs(dI) > TOL || iabs(dQ) > TOL) begin
                        $display("[CMUL_PBFDAF][C%0d][ERR] ptr=%0d  got_yI=%0d got_yQ=%0d  exp_yI=%0d exp_yQ=%0d  dI=%0d dQ=%0d",
                            case_num, out_ptr,
                            $signed(o_yI), $signed(o_yQ), eyI, eyQ, dI, dQ);
                        case_errs  = case_errs  + 1;
                        total_errs = total_errs + 1;
                    end
                    out_ptr = out_ptr + 1;
                end

                // Presentar muestra n
                i_valid = 1'b1;
                i_start = ((n % NFFT) == 0) ? 1'b1 : 1'b0;
                i_X0_re = mem_X0_re[n]; i_X0_im = mem_X0_im[n];
                i_X1_re = mem_X1_re[n]; i_X1_im = mem_X1_im[n];
            end

            // Ciclo extra: capturar última muestra
            @(posedge clk); #1;
            i_valid = 1'b0;
            i_start = 1'b0;
            if (o_valid) begin
                expected(out_ptr, eyI, eyQ);
                dI = $signed(o_yI) - eyI;
                dQ = $signed(o_yQ) - eyQ;
                if (iabs(dI) > TOL || iabs(dQ) > TOL) begin
                    $display("[CMUL_PBFDAF][C%0d][ERR] ptr=%0d  got_yI=%0d got_yQ=%0d  exp_yI=%0d exp_yQ=%0d  dI=%0d dQ=%0d",
                        case_num, out_ptr,
                        $signed(o_yI), $signed(o_yQ), eyI, eyQ, dI, dQ);
                    case_errs  = case_errs  + 1;
                    total_errs = total_errs + 1;
                end
                out_ptr = out_ptr + 1;
            end

            $display("[CASO %0d] W0=(%0d+%0dj) W1=(%0d+%0dj)  ver=%0d/%0d  errs=%0d  => %s",
                case_num,
                $signed(act_w0_re), $signed(act_w0_im),
                $signed(act_w1_re), $signed(act_w1_im),
                out_ptr, TOTAL_SAMP, case_errs,
                (case_errs == 0 && out_ptr == TOTAL_SAMP) ? "PASS" : "FAIL");
        end
    endtask

    // -------------------------------------------------------
    // write_all_weights: escribe el mismo W en todos los bins
    // vía el puerto i_we (simula lo que haría el UPDATE_LMS)
    // -------------------------------------------------------
    task write_all_weights;
        input integer wsel;
        input signed [NB_W-1:0] wr, wi;
        integer k;
        begin
            for (k = 0; k < NFFT; k = k + 1) begin
                @(posedge clk);
                i_we   = 1'b1;
                i_wk   = k[KW-1:0];
                i_wsel = wsel[0];
                i_W_re = wr;
                i_W_im = wi;
            end
            @(posedge clk);
            i_we = 1'b0;
        end
    endtask

    // -------------------------------------------------------
    // Secuencia principal
    // -------------------------------------------------------
    initial begin
        // Init señales
        i_valid = 1'b0; i_start = 1'b0;
        i_X0_re = 0; i_X0_im = 0;
        i_X1_re = 0; i_X1_im = 0;
        i_we    = 1'b0; i_wk = 0; i_wsel = 0;
        i_W_re  = 0; i_W_im = 0;
        total_errs = 0; case_errs = 0; case_num = 0;

        @(negedge rst);
        repeat(5) @(posedge clk);

        // Generar datos de entrada (mismos para todos los casos)
        gen_data();

        // ====================================================
        // CASO 1: W0=1+0j, W1=0  → Y = X_curr
        // ====================================================
        $display("\n--- CASO 1: W0=1+0j  W1=0  (Y = X_curr) ---");
        load_weights(ONE, ZERO, ZERO, ZERO);
        repeat(3) @(posedge clk);
        run_case(1);
        repeat(5) @(posedge clk);

        // ====================================================
        // CASO 2: W0=0, W1=1+0j  → Y = X_old
        // ====================================================
        $display("\n--- CASO 2: W0=0  W1=1+0j  (Y = X_old) ---");
        load_weights(ZERO, ZERO, ONE, ZERO);
        repeat(3) @(posedge clk);
        run_case(2);
        repeat(5) @(posedge clk);

        // ====================================================
        // CASO 3: W0=0.5+0j, W1=0.5+0j → Y = 0.5·X_curr + 0.5·X_old
        // ====================================================
        $display("\n--- CASO 3: W0=0.5+0j  W1=0.5+0j  (promedio real) ---");
        load_weights(HALF, ZERO, HALF, ZERO);
        repeat(3) @(posedge clk);
        run_case(3);
        repeat(5) @(posedge clk);

        // ====================================================
        // CASO 4: W0=0+1j, W1=0-1j → mezcla Im↔Re con signo
        //   Y_re = -X0_im + X1_im
        //   Y_im =  X0_re - X1_re
        // ====================================================
        $display("\n--- CASO 4: W0=0+1j  W1=0-1j  (mezcla Im<->Re) ---");
        load_weights(ZERO, IONE, ZERO, MONE);
        repeat(3) @(posedge clk);
        run_case(4);
        repeat(5) @(posedge clk);

        // ====================================================
        // CASO 5: Puerto LMS — escribir W0=0 via i_we
        //   Frame A: W0=1+0j (default) → Y = X_curr
        //   Escribir W0[k]=0 para todo k
        //   Frame B: Y = 0
        // ====================================================
        $display("\n--- CASO 5: Puerto LMS (write via i_we) ---");

        // Sub-caso 5a: antes de escribir, W0=1+0j → Y = X_curr
        $display("  [5a] Antes de write_all: W0=1+0j → Y=X_curr");
        load_weights(ONE, ZERO, ZERO, ZERO);
        repeat(3) @(posedge clk);
        run_case(5);
        repeat(5) @(posedge clk);

        // Escribir W0=0 vía puerto i_we
        $display("  [LMS] Escribiendo W0=0 via i_we para todos los bins...");
        write_all_weights(0, ZERO, ZERO);  // wsel=0 → W0
        // Actualizar expected: ahora W0=0, W1=0 → Y=0
        act_w0_re = ZERO; act_w0_im = ZERO;
        act_w1_re = ZERO; act_w1_im = ZERO;
        repeat(3) @(posedge clk);

        // Sub-caso 5b: después de escribir, W0=0, W1=0 → Y = 0
        $display("  [5b] Despues de write_all: W0=0 W1=0 → Y=0");
        case_num  = 55;
        case_errs = 0;
        run_case(55);
        repeat(5) @(posedge clk);

        // ====================================================
        // Resumen
        // ====================================================
        $display("\n========================================");
        $display("[TB_CMUL_PBFDAF] RESUMEN FINAL");
        $display("========================================");
        $display("[TOTAL]  total_errs=%0d  => %s",
            total_errs,
            (total_errs == 0) ? "PASS: cmul_pbfdaf OK" : "FAIL: revisar DUT");
        $display("========================================");
        $finish;
    end

    initial begin
        #1000000;
        $display("[TB_CMUL_PBFDAF] TIMEOUT");
        $finish;
    end

endmodule

`default_nettype wire
