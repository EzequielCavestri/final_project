module top_tx (
    input  wire clk,
    input  wire reset,
    input  wire i_en,
    output wire signed [8:0] sI_out,
    output wire signed [8:0] sQ_out
);
    wire bI, bQ;

    prbs9 prbs_i (
        .clk (clk),
        .rst (reset),
        .en  (i_en),
        .seed(9'h17F),
        .bit (bI)
    );

    prbs9 prbs_q (
        .clk (clk),
        .rst (reset),
        .en  (i_en),
        .seed(9'h11D),
        .bit (bQ)
    );

    qpsk_mapper mapper_qpsk (
        .bit_I(bI),
        .bit_Q(bQ),
        .sym_I(sI_out),
        .sym_Q(sQ_out)
    );
endmodule
