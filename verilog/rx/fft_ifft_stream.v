`timescale 1ns/1ps
`default_nettype none

module fft_ifft_stream #(
    parameter integer NFFT           = 32,
    parameter integer LOGN           = 5,

    parameter integer NB_IN          = 9,
    parameter integer NBF_IN         = 7,

    parameter integer NB_W           = 17,
    parameter integer NBF_W          = 10,

    parameter integer NB_OUT         = 9,
    parameter integer NBF_OUT        = 7,

    parameter integer REORDER_BITREV = 1,
    parameter integer BF_SCALE       = 0   // para identidad: 0
)(
    input  wire                     i_clk,
    input  wire                     i_rst,

    input  wire                     i_valid,
    input  wire                     i_start,    // resync opcional
    input  wire signed [NB_IN-1:0]  i_xI,
    input  wire signed [NB_IN-1:0]  i_xQ,
    input  wire                     i_inverse,  // 0=FFT, 1=IFFT

    output wire                     o_in_ready,

    output reg                      o_start,
    output reg                      o_valid,
    output reg  signed [NB_OUT-1:0] o_yI,
    output reg  signed [NB_OUT-1:0] o_yQ
);

    assign o_in_ready = 1'b1;

    // narrow->wide
    wire signed [NB_W-1:0] inI_w = ($signed({{(NB_W-NB_IN){i_xI[NB_IN-1]}}, i_xI})) <<< (NBF_W - NBF_IN);
    wire signed [NB_W-1:0] inQ_w = ($signed({{(NB_W-NB_IN){i_xQ[NB_IN-1]}}, i_xQ})) <<< (NBF_W - NBF_IN);

    // ---------------- INPUT ping-pong ----------------
    reg [4:0] in_idx;
    reg in_wbank;
    reg in_full0, in_full1;

    reg signed [NB_W*32-1:0] in0I, in0Q;
    reg signed [NB_W*32-1:0] in1I, in1Q;

    wire [9:0] in_base = in_idx * NB_W;

    reg take0, take1;

    always @(posedge i_clk) begin
        if (i_rst) begin
            in_idx   <= 0;
            in_wbank <= 0;
            in_full0 <= 0;
            in_full1 <= 0;
            in0I <= 0; in0Q <= 0;
            in1I <= 0; in1Q <= 0;
        end else begin
            if (take0) in_full0 <= 1'b0;
            if (take1) in_full1 <= 1'b0;

            if (i_valid) begin
                if (i_start) in_idx <= 0;

                if (in_wbank == 1'b0) begin
                    in0I[in_base +: NB_W] <= inI_w;
                    in0Q[in_base +: NB_W] <= inQ_w;
                end else begin
                    in1I[in_base +: NB_W] <= inI_w;
                    in1Q[in_base +: NB_W] <= inQ_w;
                end

                if (in_idx == 5'd31) begin
                    if (in_wbank == 1'b0) in_full0 <= 1'b1;
                    else                  in_full1 <= 1'b1;
                    in_wbank <= ~in_wbank;
                    in_idx   <= 0;
                end else begin
                    in_idx <= in_idx + 1'b1;
                end
            end
        end
    end

    // ---------------- PIPELINE compute (5 ciclos/frame) ----------------
    reg comp_busy;
    reg [2:0] comp_phase;

    reg signed [NB_W*32-1:0] s0I, s0Q;
    reg signed [NB_W*32-1:0] s1I, s1Q;
    reg signed [NB_W*32-1:0] s2I, s2Q;
    reg signed [NB_W*32-1:0] s3I, s3Q;
    reg signed [NB_W*32-1:0] s4I, s4Q;

    wire signed [NB_W*32-1:0] c1I, c1Q;
    wire signed [NB_W*32-1:0] c2I, c2Q;
    wire signed [NB_W*32-1:0] c3I, c3Q;
    wire signed [NB_W*32-1:0] c4I, c4Q;
    wire signed [NB_W*32-1:0] c5I, c5Q;

    fft32_stage_dit #(.NB_W(NB_W),.NBF_W(NBF_W),.LOGN(LOGN),.STAGE(0),.BF_SCALE(BF_SCALE)) st0(.i_inverse(i_inverse), .i_xI(s0I), .i_xQ(s0Q), .o_yI(c1I), .o_yQ(c1Q));
    fft32_stage_dit #(.NB_W(NB_W),.NBF_W(NBF_W),.LOGN(LOGN),.STAGE(1),.BF_SCALE(BF_SCALE)) st1(.i_inverse(i_inverse), .i_xI(s1I), .i_xQ(s1Q), .o_yI(c2I), .o_yQ(c2Q));
    fft32_stage_dit #(.NB_W(NB_W),.NBF_W(NBF_W),.LOGN(LOGN),.STAGE(2),.BF_SCALE(BF_SCALE)) st2(.i_inverse(i_inverse), .i_xI(s2I), .i_xQ(s2Q), .o_yI(c3I), .o_yQ(c3Q));
    fft32_stage_dit #(.NB_W(NB_W),.NBF_W(NBF_W),.LOGN(LOGN),.STAGE(3),.BF_SCALE(BF_SCALE)) st3(.i_inverse(i_inverse), .i_xI(s3I), .i_xQ(s3Q), .o_yI(c4I), .o_yQ(c4Q));
    fft32_stage_dit #(.NB_W(NB_W),.NBF_W(NBF_W),.LOGN(LOGN),.STAGE(4),.BF_SCALE(BF_SCALE)) st4(.i_inverse(i_inverse), .i_xI(s4I), .i_xQ(s4Q), .o_yI(c5I), .o_yQ(c5Q));

    // ---------------- OUTPUT ping-pong ----------------
    reg out_full0, out_full1;
    reg signed [NB_W*32-1:0] out0I, out0Q;
    reg signed [NB_W*32-1:0] out1I, out1Q;

    reg out_wr_req;
    reg out_wr_bank;
    reg signed [NB_W*32-1:0] out_wrI, out_wrQ;

    always @(posedge i_clk) begin
        if (i_rst) begin
            comp_busy  <= 1'b0;
            comp_phase <= 0;
            s0I <= 0; s0Q <= 0;
            s1I <= 0; s1Q <= 0;
            s2I <= 0; s2Q <= 0;
            s3I <= 0; s3Q <= 0;
            s4I <= 0; s4Q <= 0;
            take0 <= 0; take1 <= 0;
            out_wr_req <= 0;
            out_wr_bank <= 0;
            out_wrI <= 0; out_wrQ <= 0;
        end else begin
            take0 <= 1'b0;
            take1 <= 1'b0;
            out_wr_req <= 1'b0;

            if (!comp_busy) begin
                if (in_full0 && !(out_full0 && out_full1)) begin
                    comp_busy  <= 1'b1;
                    comp_phase <= 0;
                    s0I <= in0I; s0Q <= in0Q;
                    take0 <= 1'b1;
                end else if (in_full1 && !(out_full0 && out_full1)) begin
                    comp_busy  <= 1'b1;
                    comp_phase <= 0;
                    s0I <= in1I; s0Q <= in1Q;
                    take1 <= 1'b1;
                end
            end else begin
                if (comp_phase == 0) begin
                    s1I <= c1I; s1Q <= c1Q;
                    comp_phase <= 1;
                end else if (comp_phase == 1) begin
                    s2I <= c2I; s2Q <= c2Q;
                    comp_phase <= 2;
                end else if (comp_phase == 2) begin
                    s3I <= c3I; s3Q <= c3Q;
                    comp_phase <= 3;
                end else if (comp_phase == 3) begin
                    s4I <= c4I; s4Q <= c4Q;
                    comp_phase <= 4;
                end else begin
                    // listo: escribir frame crudo (bit-reversed) a salida
                    out_wr_req  <= 1'b1;
                    out_wr_bank <= (!out_full0) ? 1'b0 : 1'b1;
                    out_wrI     <= c5I;
                    out_wrQ     <= c5Q;
                    comp_busy   <= 1'b0;
                    comp_phase  <= 0;
                end
            end
        end
    end

    // ---------------- SEND streaming + gestión out_full (UNA sola always) ----------------
    reg sending;
    reg send_bank;
    reg [4:0] send_idx;

    function [LOGN-1:0] bitrev;
        input [LOGN-1:0] x;
        integer b;
        begin
            for (b=0; b<LOGN; b=b+1)
                bitrev[b] = x[LOGN-1-b];
        end
    endfunction

    wire [4:0] rd_idx = (REORDER_BITREV!=0) ? bitrev(send_idx) : send_idx;
    wire [9:0] out_base = rd_idx * NB_W;

    wire signed [NB_W-1:0] curI = (send_bank==1'b0) ? out0I[out_base +: NB_W] : out1I[out_base +: NB_W];
    wire signed [NB_W-1:0] curQ = (send_bank==1'b0) ? out0Q[out_base +: NB_W] : out1Q[out_base +: NB_W];

    function signed [NB_W-1:0] rshift_round_even;
        input signed [NB_W-1:0] x;
        input integer sh;
        reg signed [NB_W-1:0] y;
        reg guard;
        reg sticky;
        integer t;
        begin
            if (sh <= 0) begin
                rshift_round_even = x;
            end else begin
                y     = x >>> sh;
                guard = x[sh-1];
                sticky = 1'b0;
                for (t=0; t<sh-1; t=t+1)
                    sticky = sticky | x[t];
                rshift_round_even = y + (guard & (sticky | y[0]));
            end
        end
    endfunction

    wire signed [NB_W-1:0] scI = (i_inverse) ? rshift_round_even(curI, LOGN) : curI;
    wire signed [NB_W-1:0] scQ = (i_inverse) ? rshift_round_even(curQ, LOGN) : curQ;

    wire signed [NB_OUT-1:0] yI_n;
    wire signed [NB_OUT-1:0] yQ_n;

    sat_trunc #(.NB_XI(NB_W),.NBF_XI(NBF_W),.NB_XO(NB_OUT),.NBF_XO(NBF_OUT),.ROUND_EVEN(1)) u_outI(.i_data(scI), .o_data(yI_n));
    sat_trunc #(.NB_XI(NB_W),.NBF_XI(NBF_W),.NB_XO(NB_OUT),.NBF_XO(NBF_OUT),.ROUND_EVEN(1)) u_outQ(.i_data(scQ), .o_data(yQ_n));

    always @(posedge i_clk) begin
        if (i_rst) begin
            out_full0 <= 1'b0;
            out_full1 <= 1'b0;
            out0I <= 0; out0Q <= 0;
            out1I <= 0; out1Q <= 0;

            sending   <= 1'b0;
            send_bank <= 1'b0;
            send_idx  <= 0;

            o_valid <= 1'b0;
            o_start <= 1'b0;
            o_yI    <= 0;
            o_yQ    <= 0;
        end else begin
            // captura escritura de frame a salida
            if (out_wr_req) begin
                if (out_wr_bank == 1'b0) begin
                    out0I <= out_wrI;
                    out0Q <= out_wrQ;
                    out_full0 <= 1'b1;
                end else begin
                    out1I <= out_wrI;
                    out1Q <= out_wrQ;
                    out_full1 <= 1'b1;
                end
            end

            o_valid <= 1'b0;
            o_start <= 1'b0;

            if (!sending) begin
                if (out_full0) begin
                    sending   <= 1'b1;
                    send_bank <= 1'b0;
                    send_idx  <= 0;
                end else if (out_full1) begin
                    sending   <= 1'b1;
                    send_bank <= 1'b1;
                    send_idx  <= 0;
                end
            end else begin
                o_valid <= 1'b1;
                o_start <= (send_idx == 0);
                o_yI    <= yI_n;
                o_yQ    <= yQ_n;

                if (send_idx == 5'd31) begin
                    // libero banco
                    if (send_bank==1'b0) out_full0 <= 1'b0;
                    else                 out_full1 <= 1'b0;

                    // continuo si el otro está listo
                    if (send_bank==1'b0 && out_full1) begin
                        send_bank <= 1'b1;
                        send_idx  <= 0;
                        sending   <= 1'b1;
                    end else if (send_bank==1'b1 && out_full0) begin
                        send_bank <= 1'b0;
                        send_idx  <= 0;
                        sending   <= 1'b1;
                    end else begin
                        sending  <= 1'b0;
                        send_idx <= 0;
                    end
                end else begin
                    send_idx <= send_idx + 1'b1;
                end
            end
        end
    end

endmodule

`default_nettype wire