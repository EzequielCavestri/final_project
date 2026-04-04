`timescale 1ns/1ps
module prbs9(
  input  wire clk,
  input  wire rst,
  input  wire en,          // avanzar 1 bit
  input  wire [8:0] seed,  // 9 bits, no nulo
  output wire bit
);
  reg [8:0] s;
  wire fb = s[8] ^ s[4];   // x^9 + x^5 + 1
  always @(posedge clk) begin
    if (rst) s <= (seed!=9'd0) ? seed : 9'h1;
    else if (en) s <= {s[7:0], fb};
  end
  assign bit = s[8];
endmodule
