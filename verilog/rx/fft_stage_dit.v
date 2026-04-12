`timescale 1ns/1ps
`default_nettype none

// ============================================================
// fft32_stage_dit
//  - 1 etapa DIT de FFT/IFFT de 32 puntos
//  - 16 butterflies en paralelo
//  - Twiddle index: k = j << (LOGN-1-STAGE)
//  - i_inverse: 0=FFT, 1=IFFT (conjugado)
//  - BF_SCALE controla SCALE del butterfly (0 => identidad)
// ============================================================

module fft32_stage_dit #(
    parameter integer NB_W     = 17,
    parameter integer NBF_W    = 10,
    parameter integer LOGN     = 5,
    parameter integer STAGE    = 0,
    parameter integer BF_SCALE = 0
)(
    input  wire                      i_inverse,
    input  wire signed [NB_W*32-1:0]  i_xI,
    input  wire signed [NB_W*32-1:0]  i_xQ,
    output wire signed [NB_W*32-1:0]  o_yI,
    output wire signed [NB_W*32-1:0]  o_yQ
);

    localparam integer SPAN  = (1 << STAGE);
    localparam integer STEP  = (SPAN << 1);
    localparam integer NG    = (32 / STEP);
    localparam integer SHIFT = (LOGN-1-STAGE);

    genvar g, j;
    generate
      for (g=0; g<NG; g=g+1) begin : G_G
        for (j=0; j<SPAN; j=j+1) begin : G_J
          localparam integer U = (g*STEP) + j;
          localparam integer V = U + SPAN;
          localparam [4:0] TW  = (j << SHIFT);

          wire signed [NB_W-1:0] uI = i_xI[U*NB_W +: NB_W];
          wire signed [NB_W-1:0] uQ = i_xQ[U*NB_W +: NB_W];
          wire signed [NB_W-1:0] vI = i_xI[V*NB_W +: NB_W];
          wire signed [NB_W-1:0] vQ = i_xQ[V*NB_W +: NB_W];

          wire signed [NB_W-1:0] w_re;
          wire signed [NB_W-1:0] w_im_fft; // = -sin()

          twiddle_rom #(.NB_W(NB_W), .NBF_W(NBF_W)) u_tw (
            .i_k(TW),
            .o_re(w_re),
            .o_im_fft(w_im_fft)
          );

          wire signed [NB_W-1:0] wI = w_re;
          wire signed [NB_W-1:0] wQ = (i_inverse) ? -w_im_fft : w_im_fft;

          wire signed [NB_W-1:0] aI, aQ, bI, bQ;

          // OJO: butterfly.v usa complex_mult adentro => complex_mult.v ES necesario
          butterfly #(.NB_W(NB_W), .NBF_W(NBF_W), .SCALE(BF_SCALE)) u_bf (
            .i_uI(uI), .i_uQ(uQ),
            .i_vI(vI), .i_vQ(vQ),
            .i_wI(wI), .i_wQ(wQ),
            .o_aI(aI), .o_aQ(aQ),
            .o_bI(bI), .o_bQ(bQ)
          );

          assign o_yI[U*NB_W +: NB_W] = aI;
          assign o_yQ[U*NB_W +: NB_W] = aQ;
          assign o_yI[V*NB_W +: NB_W] = bI;
          assign o_yQ[V*NB_W +: NB_W] = bQ;
        end
      end
    endgenerate

endmodule

`default_nettype wire