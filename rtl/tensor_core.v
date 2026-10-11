// tensor_core.v  (10_6; was mac_sub.v in 110_3)
//
// The tensor core of one SM sub-partition (NVIDIA: every SM has 4
// sub-partitions, each with one tensor core): a ROWS x COLS grid of
// mac_cell.v (DSP48E1) -- 2 x 8 = 16 MACs in the 10_6 build,
// output-stationary (every DSP keeps its own C[i][j] in its P register
// for a whole block, like the accumulator in NVIDIA's tensor memory).
//
// Every enabled k-step, ROWS values of A (one column of A, this
// sub-partition's rows) and COLS values of B (one row of B, shared by all
// sub-partitions) come in; cell (r, c) does  P[r][c] += A[r] * B[c].
// A value is reused COLS times, a B value ROWS times.
//
// Pipeline (cycle numbers relative to the k-step's issue cycle t, see
// sm_core.v -- a_in / b_in / tags arrive at t+3, from the BRAM DO_REG):
//   t+4  a_x / b_x       fabric register next to the BRAMs (KEEP)
//   t+5  a_r (per row)   one copy of each A value per row  -> 8 DSPs
//        b_y (per sub)   one copy of B per sub-partition   -> ROWS DSPs
//   t+6  DSP A/B regs
//   t+7  DSP M reg; P updated at the end of t+7 (v7 / f7 = CEP / load)
//   t+8  capl: the block's last product is in P -> copy all P into the
//        readout chain at the end of t+8 (P is free for the next block
//        the very next cycle -- the double buffer NVIDIA gets from TMEM)
//
// Readout chain: ROWS*COLS registers of 48 bit, entry j = r*COLS + c.
// Loaded in parallel by capl, otherwise shifts one entry toward `out`
// every cycle (zeros enter at the top, so an idle chain stops toggling).
// `out` = entry 0, so C[0][0], C[0][1], ... come out on consecutive
// cycles t+9, t+10, ... (sm_core.v keeps count).

module tensor_core #(
  parameter integer ROWS = 2,
  parameter integer COLS = 8
)(
  input  wire                 clk,
  input  wire [ROWS*16-1:0]   a_in,   // lane r = A[row r of this sub-partition][k]
  input  wire [COLS*16-1:0]   b_in,   // lane c = B[k][c]
  input  wire                 tv,     // valid  (this is a real k-step)
  input  wire                 tf,     // first  (k == 0: load instead of accumulate)
  input  wire                 tl,     // last   (k == K-1: block finished after this)
  output wire [47:0]          out
);

  localparam integer NC = ROWS * COLS;

  // ---- t+4: fabric registers next to the BRAMs --------------------------
  (* keep = "true" *) reg [ROWS*16-1:0] a_x = {ROWS*16{1'b0}};
  (* keep = "true" *) reg [COLS*16-1:0] b_x = {COLS*16{1'b0}};
  reg tv_x = 1'b0, tf_x = 1'b0, tl_x = 1'b0;
  always @(posedge clk) begin
    a_x  <= a_in;
    b_x  <= b_in;
    tv_x <= tv;
    tf_x <= tf;
    tl_x <= tl;
  end

  // ---- t+5: B copy for this sub-partition ---------------------------------
  (* keep = "true" *) reg [COLS*16-1:0] b_y = {COLS*16{1'b0}};
  always @(posedge clk) b_y <= b_x;

  wire [NC*48-1:0] p_flat;
  wire [ROWS-1:0]  capl_w;

  genvar r, c, j;
  generate
    for (r = 0; r < ROWS; r = r + 1) begin : ROW
      (* keep = "true" *) reg [15:0] a_r = 16'd0;           // t+5
      reg v5 = 1'b0, f5 = 1'b0, l5 = 1'b0;                  // t+5
      reg v6 = 1'b0, f6 = 1'b0, l6 = 1'b0;                  // t+6
      (* keep = "true" *) reg v7 = 1'b0;                    // t+7: DSP CEP of this row
      (* keep = "true" *) reg f7 = 1'b0;                    // t+7: DSP load of this row
      reg l7 = 1'b0;
      (* keep = "true", max_fanout = 64 *) reg capl = 1'b0; // t+8: load this row's chain entries
      always @(posedge clk) begin
        a_r  <= a_x[r*16 +: 16];
        v5   <= tv_x;  f5 <= tf_x;  l5 <= tl_x;
        v6   <= v5;    f6 <= f5;    l6 <= l5;
        v7   <= v6;    f7 <= f6;    l7 <= l6;
        capl <= v7 & l7;
      end
      assign capl_w[r] = capl;

      for (c = 0; c < COLS; c = c + 1) begin : COL
        mac_cell u_mac (
          .clk  (clk),
          .a    (a_r),
          .b    (b_y[c*16 +: 16]),
          .ce_p (v7),
          .ld   (f7),
          .p    (p_flat[(r*COLS + c)*48 +: 48])
        );
      end
    end

    // ---- readout chain ------------------------------------------------------
    wire [(NC+1)*48-1:0] chain;
    assign chain[NC*48 +: 48] = 48'd0;            // zeros shift in at the top
    for (j = 0; j < NC; j = j + 1) begin : CAP
      reg [47:0] q = 48'd0;
      always @(posedge clk)
        q <= capl_w[j / COLS] ? p_flat[j*48 +: 48] : chain[(j+1)*48 +: 48];
      assign chain[j*48 +: 48] = q;
    end
    assign out = chain[47:0];
  endgenerate

endmodule
