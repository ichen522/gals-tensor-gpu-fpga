// stat_cdc.v  (110_3)
//
// Consistent snapshot of a tile's counters for host_if.v (READ_ACT,
// READ_CHK, READ_STAT). Same idea as v7's host_read_cdc.v:
//
//   1. host_if flips req_tog (clk_mem)
//   2. tile side: 2-flop sync + edge -> in ONE tile-clock cycle copy
//      data_in = {activity_count, blk_done, chk} into `snap` and flip snap_tog
//   3. clk_mem side: 2-flop sync of snap_tog + edge -> data_out <= snap
//      (snap has been frozen for >= 2 tile cycles + 2 clk_mem cycles)
//   4. ONE clk_mem cycle later ack_tog flips -- so whoever sees ack_tog ==
//      req_tog (host_if.v, also through the same spine register stage as
//      data_out) is guaranteed to see the new data_out.
//
// blk_done and chk change in the same SM cycle (sm_core.v), so the
// snapshot pairs them correctly.

module stat_cdc #(
  parameter integer W = 128
)(
  input  wire          req_clk,
  input  wire          req_rst_n,
  input  wire          req_tog,
  output reg           ack_tog,
  output reg  [W-1:0]  data_out,

  input  wire          tile_clk,
  input  wire          tile_srst,      // synchronous, tile domain
  input  wire [W-1:0]  data_in
);

  // ---- tile side ----
  (* ASYNC_REG = "TRUE" *) reg [1:0] rs = 2'b00;
  reg rp = 1'b0;
  reg snap_tog = 1'b0;
  reg [W-1:0] snap = {W{1'b0}};
  wire req_edge = rs[1] ^ rp;
  always @(posedge tile_clk) begin
    rs <= {rs[0], req_tog};
    rp <= rs[1];
    if (tile_srst)     snap_tog <= 1'b0;
    else if (req_edge) snap_tog <= ~snap_tog;
  end
  // data register: enable only, no reset (its own block -- Vivado
  // [Synth 37-111] RESET-3 if it shares the reset branch above)
  always @(posedge tile_clk)
    if (req_edge) snap <= data_in;

  // ---- clk_mem side ----
  (* ASYNC_REG = "TRUE" *) reg [1:0] as = 2'b00;
  reg ap = 1'b0;
  always @(posedge req_clk or negedge req_rst_n) begin
    if (!req_rst_n) begin
      as       <= 2'b00;
      ap       <= 1'b0;
      ack_tog  <= 1'b0;
      data_out <= {W{1'b0}};
    end else begin
      as <= {as[0], snap_tog};
      ap <= as[1];
      if (as[1] ^ ap) data_out <= snap;
      ack_tog <= ap;              // one cycle after data_out took the new snap
    end
  end

endmodule
