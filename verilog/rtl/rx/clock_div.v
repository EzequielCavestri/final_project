module clock_div2 (
  input  wire i_clk_fast,
  input  wire i_enable,
  output reg  o_clk_low
);
  initial o_clk_low = 1'b0;

  always @(posedge i_clk_fast) begin
    if (i_enable)
      o_clk_low <= ~o_clk_low;
  end
endmodule