`timescale 1ns/1ps
`default_nettype none

// ============================================================
// zero_pad_error  v5  (ping-pong, bugs corregidos)
//
// Fixes respecto a v4:
//
//   Bug 1 — Off-by-1 en posición:
//     En el ciclo i_start, recv_armed era 0 (non-blocking),
//     así que el sample 0 no se escribía. El buffer recibía
//     samples 1..16 en posiciones 0..15.
//     Fix: condición de escritura = (recv_armed || i_start)
//          En i_start se escribe sample 0 y recv_cnt queda en 1.
//
//   Bug 2 — Frame skipping:
//     wr_buf alternaba al completar el buffer, pero podía apuntar
//     a un buffer EMITTING cuando llegaba el próximo i_start.
//     Fix: en cada i_start se busca el buffer FREE sin importar
//          el valor de wr_buf. recv_cnt se inicializa en 1 porque
//          ya se escribió sample 0 en ese mismo ciclo.
// ============================================================

module zero_pad_error #(
    parameter integer NB_W = 9,
    parameter integer NFFT = 32
)(
    input  wire                    clk,
    input  wire                    rst,
    input  wire                    i_valid,
    input  wire                    i_start,
    input  wire signed [NB_W-1:0]  i_eI,
    input  wire signed [NB_W-1:0]  i_eQ,
    output reg                     o_valid,
    output reg                     o_start,
    output reg  signed [NB_W-1:0]  o_eI,
    output reg  signed [NB_W-1:0]  o_eQ
);

    localparam integer N  = NFFT / 2;
    localparam integer KW = $clog2(N);

    localparam [1:0] FREE     = 2'd0,
                     READY    = 2'd1,
                     EMITTING = 2'd2;

    localparam [1:0] ST_IDLE  = 2'd0,
                     ST_ZEROS = 2'd1,
                     ST_ERROR = 2'd2;

    reg signed [NB_W-1:0] buf0_I [0:N-1];
    reg signed [NB_W-1:0] buf0_Q [0:N-1];
    reg signed [NB_W-1:0] buf1_I [0:N-1];
    reg signed [NB_W-1:0] buf1_Q [0:N-1];

    reg [1:0] buf0_st;
    reg [1:0] buf1_st;

    reg [KW:0] recv_cnt;
    reg        wr_buf;
    reg        recv_armed;

    reg [1:0]  emit_st;
    reg [KW:0] emit_cnt;
    reg        rd_buf;

    integer k;

    always @(posedge clk) begin
        if (rst) begin
            buf0_st    <= FREE;
            buf1_st    <= FREE;
            recv_cnt   <= 0;
            wr_buf     <= 1'b0;
            recv_armed <= 1'b0;
            emit_st    <= ST_IDLE;
            emit_cnt   <= 0;
            rd_buf     <= 1'b0;
            o_valid    <= 1'b0;
            o_start    <= 1'b0;
            o_eI       <= {NB_W{1'b0}};
            o_eQ       <= {NB_W{1'b0}};
            for (k=0; k<N; k=k+1) begin
                buf0_I[k] <= {NB_W{1'b0}};
                buf0_Q[k] <= {NB_W{1'b0}};
                buf1_I[k] <= {NB_W{1'b0}};
                buf1_Q[k] <= {NB_W{1'b0}};
            end
        end else begin

            // ================================================
            // RECEPTOR
            // ================================================
            if (i_valid) begin

                if (i_start) begin
                    // Buscar buffer FREE — ignora wr_buf actual
                    if (buf0_st == FREE) begin
                        wr_buf     <= 1'b0;
                        // Escribir sample 0 inmediatamente
                        buf0_I[0]  <= i_eI;
                        buf0_Q[0]  <= i_eQ;
                        recv_cnt   <= 1;      // próximo sample va en pos 1
                        recv_armed <= 1'b1;
                    end else if (buf1_st == FREE) begin
                        wr_buf     <= 1'b1;
                        buf1_I[0]  <= i_eI;
                        buf1_Q[0]  <= i_eQ;
                        recv_cnt   <= 1;
                        recv_armed <= 1'b1;
                    end
                    // else: ambos ocupados, frame perdido (no debería pasar)

                end else if (recv_armed) begin
                    // Samples 1..N-1
                    if (wr_buf == 1'b0) begin
                        buf0_I[recv_cnt[KW-1:0]] <= i_eI;
                        buf0_Q[recv_cnt[KW-1:0]] <= i_eQ;
                        if (recv_cnt == N-1) begin
                            buf0_st    <= READY;
                            recv_armed <= 1'b0;
                        end else begin
                            recv_cnt <= recv_cnt + 1'b1;
                        end
                    end else begin
                        buf1_I[recv_cnt[KW-1:0]] <= i_eI;
                        buf1_Q[recv_cnt[KW-1:0]] <= i_eQ;
                        if (recv_cnt == N-1) begin
                            buf1_st    <= READY;
                            recv_armed <= 1'b0;
                        end else begin
                            recv_cnt <= recv_cnt + 1'b1;
                        end
                    end
                end
            end

            // ================================================
            // EMISOR FSM
            // ================================================
            case (emit_st)

                ST_IDLE: begin
                    o_valid <= 1'b0;
                    o_start <= 1'b0;
                    o_eI    <= {NB_W{1'b0}};
                    o_eQ    <= {NB_W{1'b0}};
                    if (buf0_st == READY) begin
                        buf0_st  <= EMITTING;
                        rd_buf   <= 1'b0;
                        emit_st  <= ST_ZEROS;
                        emit_cnt <= 0;
                    end else if (buf1_st == READY) begin
                        buf1_st  <= EMITTING;
                        rd_buf   <= 1'b1;
                        emit_st  <= ST_ZEROS;
                        emit_cnt <= 0;
                    end
                end

                ST_ZEROS: begin
                    o_valid <= 1'b1;
                    o_start <= (emit_cnt == 0);
                    o_eI    <= {NB_W{1'b0}};
                    o_eQ    <= {NB_W{1'b0}};
                    if (emit_cnt == N-1) begin
                        emit_st  <= ST_ERROR;
                        emit_cnt <= 0;
                    end else begin
                        emit_cnt <= emit_cnt + 1'b1;
                    end
                end

                ST_ERROR: begin
                    o_valid <= 1'b1;
                    o_start <= 1'b0;
                    if (rd_buf == 1'b0) begin
                        o_eI <= buf0_I[emit_cnt[KW-1:0]];
                        o_eQ <= buf0_Q[emit_cnt[KW-1:0]];
                    end else begin
                        o_eI <= buf1_I[emit_cnt[KW-1:0]];
                        o_eQ <= buf1_Q[emit_cnt[KW-1:0]];
                    end
                    if (emit_cnt == N-1) begin
                        // Liberar buffer actual
                        if (rd_buf == 1'b0) buf0_st <= FREE;
                        else                buf1_st <= FREE;
                        emit_cnt <= 0;
                        // Transición directa al siguiente buffer si ya está READY
                        // → elimina el ciclo IDLE entre frames
                        // → latencia constante = 32 ciclos para todos los frames
                        if (rd_buf == 1'b0 && buf1_st == READY) begin
                            buf1_st <= EMITTING;
                            rd_buf  <= 1'b1;
                            emit_st <= ST_ZEROS;
                        end else if (rd_buf == 1'b1 && buf0_st == READY) begin
                            buf0_st <= EMITTING;
                            rd_buf  <= 1'b0;
                            emit_st <= ST_ZEROS;
                        end else begin
                            emit_st <= ST_IDLE;
                        end
                    end else begin
                        emit_cnt <= emit_cnt + 1'b1;
                    end
                end

                default: emit_st <= ST_IDLE;
            endcase

        end
    end

endmodule

`default_nettype wire
