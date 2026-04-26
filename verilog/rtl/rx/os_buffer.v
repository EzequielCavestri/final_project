`timescale 1ns/1ps
`default_nettype none

module os_buffer #(
  parameter integer N  = 16,
  parameter integer WN = 9
)(
  input  wire                 i_clk_low,
  input  wire                 i_clk_fast,
  input  wire                 i_rst,

  input  wire                 i_valid,
  input  wire signed [WN-1:0] i_i,
  input  wire signed [WN-1:0] i_q,

  output reg                  o_overflow,

  output reg                  o_start,
  output reg                  o_valid,
  output reg  signed [WN-1:0] o_i,
  output reg  signed [WN-1:0] o_q
);

  function integer clog2;
    input integer value;
    integer k;
    begin
      clog2 = 0;
      for (k = value - 1; k > 0; k = k >> 1)
        clog2 = clog2 + 1;
    end
  endfunction

  localparam integer CNTW = (N <= 1)   ? 1 : clog2(N);
  localparam integer IDXW = (2*N <= 1) ? 1 : clog2(2*N);

  integer j;
  integer idx_int;

  // overlap (N)
  reg signed [WN-1:0] overlap_i [0:N-1];
  reg signed [WN-1:0] overlap_q [0:N-1];

  // two banks for "new" (N each)
  reg signed [WN-1:0] new0_i [0:N-1];
  reg signed [WN-1:0] new0_q [0:N-1];
  reg signed [WN-1:0] new1_i [0:N-1];
  reg signed [WN-1:0] new1_q [0:N-1];

  // -------------------------
  // LOW domain state
  // -------------------------
  reg [CNTW-1:0] fill_cnt;
  reg            wr_bank;     // which bank we write next (0/1)
  reg            ready0;
  reg            ready1;

  reg            req_tog_low;
  reg            req_bank_low;

  reg            ovf_low;     // sticky in low

  // Ack from FAST (toggle + bank)
  reg ack_seen_low;
  reg ack_tog_fast;
  reg ack_bank_fast;

  wire ack_event_low = (ack_tog_fast ^ ack_seen_low);
  wire clr0_low      = ack_event_low && (ack_bank_fast == 1'b0);
  wire clr1_low      = ack_event_low && (ack_bank_fast == 1'b1);

  wire ready0_eff    = ready0 & ~clr0_low;
  wire ready1_eff    = ready1 & ~clr1_low;

  // -------------------------
  // FAST domain state
  // -------------------------
  reg req_seen_fast;
  reg pending;
  reg pending_bank;

  reg sending;
  reg active_bank;
  reg [IDXW-1:0] send_idx;

  // -------------------------
  // LOW: collect N samples
  // -------------------------
  always @(posedge i_clk_low) begin
    if (i_rst) begin
      fill_cnt     <= {CNTW{1'b0}};
      wr_bank      <= 1'b0;
      ready0       <= 1'b0;
      ready1       <= 1'b0;

      req_tog_low  <= 1'b0;
      req_bank_low <= 1'b0;

      ovf_low      <= 1'b0;
      ack_seen_low <= 1'b0;

      for (j = 0; j < N; j = j + 1) begin
        overlap_i[j] <= 0;
        overlap_q[j] <= 0;
        new0_i[j]    <= 0;
        new0_q[j]    <= 0;
        new1_i[j]    <= 0;
        new1_q[j]    <= 0;
      end

    end else begin
      // apply ack clear (effective this cycle via ready*_eff)
      if (ack_event_low) begin
        ack_seen_low <= ack_tog_fast;
        if (ack_bank_fast == 1'b0) ready0 <= 1'b0;
        else                       ready1 <= 1'b0;
      end

      if (i_valid && !ovf_low) begin
        // write attempt: if chosen bank still full -> real overflow
        if (wr_bank == 1'b0) begin
          if (ready0_eff) begin
            ovf_low <= 1'b1;
          end else begin
            new0_i[fill_cnt] <= i_i;
            new0_q[fill_cnt] <= i_q;

            if (fill_cnt == (N-1)) begin
              ready0       <= 1'b1;
              req_bank_low <= 1'b0;
              req_tog_low  <= ~req_tog_low;
              fill_cnt     <= {CNTW{1'b0}};
              wr_bank      <= 1'b1; // switch for next block
            end else begin
              fill_cnt <= fill_cnt + 1'b1;
            end
          end
        end else begin
          if (ready1_eff) begin
            ovf_low <= 1'b1;
          end else begin
            new1_i[fill_cnt] <= i_i;
            new1_q[fill_cnt] <= i_q;

            if (fill_cnt == (N-1)) begin
              ready1       <= 1'b1;
              req_bank_low <= 1'b1;
              req_tog_low  <= ~req_tog_low;
              fill_cnt     <= {CNTW{1'b0}};
              wr_bank      <= 1'b0; // switch for next block
            end else begin
              fill_cnt <= fill_cnt + 1'b1;
            end
          end
        end
      end
    end
  end

  // -------------------------
  // FAST: request capture + send 2N
  // -------------------------
  always @(posedge i_clk_fast) begin
    if (i_rst) begin
      o_overflow   <= 1'b0;

      o_start      <= 1'b0;
      o_valid      <= 1'b0;
      o_i          <= 0;
      o_q          <= 0;

      req_seen_fast<= 1'b0;
      pending      <= 1'b0;
      pending_bank <= 1'b0;

      sending      <= 1'b0;
      active_bank  <= 1'b0;
      send_idx     <= {IDXW{1'b0}};

      ack_tog_fast <= 1'b0;
      ack_bank_fast<= 1'b0;

    end else begin
      o_start <= 1'b0;

      // propagate sticky overflow from low
      if (ovf_low)
        o_overflow <= 1'b1;

      // detect new request (toggle from low)
      if (req_tog_low ^ req_seen_fast) begin
        req_seen_fast <= req_tog_low;

        // allow ONE pending even while sending
        if (pending) begin
          o_overflow <= 1'b1;
        end else begin
          pending      <= 1'b1;
          pending_bank <= req_bank_low;
        end
      end

      // send logic
      if (!sending) begin
        if (pending) begin
          sending     <= 1'b1;
          active_bank <= pending_bank;
          pending     <= 1'b0;

          // emit sample 0 now (start aligned with valid)
          o_valid     <= 1'b1;
          o_start     <= 1'b1;
          o_i         <= overlap_i[0];
          o_q         <= overlap_q[0];

          send_idx    <= {{(IDXW-1){1'b0}}, 1'b1}; // next = 1
        end else begin
          o_valid <= 1'b0;
        end

      end else begin
        o_valid <= 1'b1;

        if (send_idx < N) begin
          idx_int = send_idx;
          o_i <= overlap_i[idx_int];
          o_q <= overlap_q[idx_int];
        end else begin
          idx_int = send_idx - N;
          if (active_bank == 1'b0) begin
            o_i <= new0_i[idx_int];
            o_q <= new0_q[idx_int];
          end else begin
            o_i <= new1_i[idx_int];
            o_q <= new1_q[idx_int];
          end
        end

        if (send_idx == (2*N-1)) begin
          sending  <= 1'b0;
          send_idx <= {IDXW{1'b0}};

          // overlap <= active new bank
          for (j = 0; j < N; j = j + 1) begin
            if (active_bank == 1'b0) begin
              overlap_i[j] <= new0_i[j];
              overlap_q[j] <= new0_q[j];
            end else begin
              overlap_i[j] <= new1_i[j];
              overlap_q[j] <= new1_q[j];
            end
          end

          // ack bank consumed
          ack_bank_fast <= active_bank;
          ack_tog_fast  <= ~ack_tog_fast;

        end else begin
          send_idx <= send_idx + 1'b1;
        end
      end
    end
  end

endmodule

`default_nettype wire