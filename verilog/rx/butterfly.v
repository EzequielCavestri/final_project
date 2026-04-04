module butterfly #(
    parameter integer NB_W   = 17,
    parameter integer NBF_W  = 10,
    parameter integer SCALE  = 1   // 1: >>1, 0: no scale
)(
    input  signed [NB_W-1:0] i_uI,
    input  signed [NB_W-1:0] i_uQ,
    input  signed [NB_W-1:0] i_vI,
    input  signed [NB_W-1:0] i_vQ,
    input  signed [NB_W-1:0] i_wI,
    input  signed [NB_W-1:0] i_wQ,
    output signed [NB_W-1:0] o_aI,
    output signed [NB_W-1:0] o_aQ,
    output signed [NB_W-1:0] o_bI,
    output signed [NB_W-1:0] o_bQ
);

    // t = v * w
    wire signed [NB_W-1:0] tI;
    wire signed [NB_W-1:0] tQ;

    complex_mult #(.NB_W(NB_W), .NBF_W(NBF_W)) u_cm (
        .i_aI(i_vI), .i_aQ(i_vQ),
        .i_bI(i_wI), .i_bQ(i_wQ),
        .o_yI(tI),   .o_yQ(tQ)
    );

    // a=u+t, b=u-t (use NB_W+1)
    wire signed [NB_W:0] aI_big = $signed(i_uI) + $signed(tI);
    wire signed [NB_W:0] aQ_big = $signed(i_uQ) + $signed(tQ);
    wire signed [NB_W:0] bI_big = $signed(i_uI) - $signed(tI);
    wire signed [NB_W:0] bQ_big = $signed(i_uQ) - $signed(tQ);

    wire signed [NB_W:0] aI_s = (SCALE!=0) ? (aI_big >>> 1) : aI_big;
    wire signed [NB_W:0] aQ_s = (SCALE!=0) ? (aQ_big >>> 1) : aQ_big;
    wire signed [NB_W:0] bI_s = (SCALE!=0) ? (bI_big >>> 1) : bI_big;
    wire signed [NB_W:0] bQ_s = (SCALE!=0) ? (bQ_big >>> 1) : bQ_big;

    function signed [NB_W-1:0] satW;
        input signed [NB_W:0] x;
        reg signed [NB_W-1:0] maxv;
        reg signed [NB_W-1:0] minv;
        begin
            maxv = {1'b0, {(NB_W-1){1'b1}}};
            minv = {1'b1, {(NB_W-1){1'b0}}};
            if (x > $signed(maxv))      satW = maxv;
            else if (x < $signed(minv)) satW = minv;
            else                        satW = x[NB_W-1:0];
        end
    endfunction

    assign o_aI = satW(aI_s);
    assign o_aQ = satW(aQ_s);
    assign o_bI = satW(bI_s);
    assign o_bQ = satW(bQ_s);

endmodule
