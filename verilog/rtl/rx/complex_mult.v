module complex_mult #(
    parameter integer NB_W   = 17,
    parameter integer NBF_W  = 10
)(
    input  signed [NB_W-1:0] i_aI,
    input  signed [NB_W-1:0] i_aQ,
    input  signed [NB_W-1:0] i_bI,
    input  signed [NB_W-1:0] i_bQ,
    output signed [NB_W-1:0] o_yI,
    output signed [NB_W-1:0] o_yQ
);

    wire signed [(2*NB_W)-1:0] p1 = $signed(i_aI) * $signed(i_bI);
    wire signed [(2*NB_W)-1:0] p2 = $signed(i_aQ) * $signed(i_bQ);
    wire signed [(2*NB_W)-1:0] p3 = $signed(i_aI) * $signed(i_bQ);
    wire signed [(2*NB_W)-1:0] p4 = $signed(i_aQ) * $signed(i_bI);

    wire signed [(2*NB_W)-1:0] yI_full = p1 - p2;
    wire signed [(2*NB_W)-1:0] yQ_full = p3 + p4;

    // Full frac = 2*NBF_W
    sat_trunc #(
        .NB_XI(2*NB_W), .NBF_XI(2*NBF_W),
        .NB_XO(NB_W),   .NBF_XO(NBF_W),
        .ROUND_EVEN(1)
    ) u_qI (
        .i_data(yI_full),
        .o_data(o_yI)
    );

    sat_trunc #(
        .NB_XI(2*NB_W), .NBF_XI(2*NBF_W),
        .NB_XO(NB_W),   .NBF_XO(NBF_W),
        .ROUND_EVEN(1)
    ) u_qQ (
        .i_data(yQ_full),
        .o_data(o_yQ)
    );

endmodule
