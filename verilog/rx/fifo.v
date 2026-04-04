`timescale 1ns/1ps
`default_nettype none

module fifo #(
    parameter integer DATA_WIDTH = 18,
    parameter integer ADDR_WIDTH = 8
)(
    input  wire                   clk,
    input  wire                   rst,
    input  wire [DATA_WIDTH-1:0]  din,
    input  wire                   wr_en,
    input  wire                   rd_en,
    output reg  [DATA_WIDTH-1:0]  dout,
    output wire                   full,
    output wire                   empty,
    output reg                    valid,
    output reg                    overflow,
    output wire [ADDR_WIDTH:0]    data_count
);

    localparam integer DEPTH = 1 << ADDR_WIDTH;

    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];
    reg [ADDR_WIDTH-1:0] wr_ptr;
    reg [ADDR_WIDTH-1:0] rd_ptr;
    reg [ADDR_WIDTH:0]   count;

    assign empty      = (count == 0);
    assign full       = (count == DEPTH);
    assign data_count = count;

    wire wr_fire = wr_en && !full;
    wire rd_fire = rd_en && !empty;

    reg rd_fire_d;

    always @(posedge clk) begin
        if (rst) begin
            wr_ptr    <= {ADDR_WIDTH{1'b0}};
            rd_ptr    <= {ADDR_WIDTH{1'b0}};
            count     <= {(ADDR_WIDTH+1){1'b0}};
            dout      <= {DATA_WIDTH{1'b0}};
            valid     <= 1'b0;
            rd_fire_d <= 1'b0;
            overflow  <= 1'b0;
        end else begin
            // WRITE
            if (wr_en) begin
                if (!full) begin
                    mem[wr_ptr] <= din;
                    wr_ptr      <= wr_ptr + 1'b1;
                end else begin
                    overflow <= 1'b1; // sticky
                end
            end

            // READ
            if (rd_fire) begin
                dout   <= mem[rd_ptr];
                rd_ptr <= rd_ptr + 1'b1;
            end

            // valid 1 ciclo después del rd_fire
            rd_fire_d <= rd_fire;
            valid     <= rd_fire_d;

            // COUNT
            case ({wr_fire, rd_fire})
                2'b10: count <= count + 1'b1;
                2'b01: count <= count - 1'b1;
                default: count <= count;
            endcase
        end
    end

endmodule

`default_nettype wire
