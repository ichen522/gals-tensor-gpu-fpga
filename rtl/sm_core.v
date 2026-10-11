// sm_core.v  (10_6, build 0x90; was tile_core.v in 110_3)
//
// The compute part of one SM -- the FPGA version of one NVIDIA SM working
// on one GEMM output tile with all 4 sub-partitions together (like a
// Hopper warpgroup MMA, wgmma: the 4 warps / sub-partitions split the M
// rows of the tile, B is shared from shared memory):
//
//     C (8 x 8) = A (8 x K) x B (K x 8)          16-bit signed in, 48-bit out
//     (N_SUB = 4 sub-partitions x ROWS = 2 rows, COLS = 8 columns)
//
//   NVIDIA SM                          this module
//   ---------------------------------- --------------------------------------
//   shared memory: A / B tiles         A: one BRAM per sub-partition (ROWS x 16 bit)
//                                      B: one BRAM, shared (COLS x 16 bit)
//   TMA streams the next K-slice       the controller walks one BRAM address per k-step
//   4 sub-partitions, 1 tensor core    4 x tensor_core.v, 2 x 8 DSP48E1 each = 64 MACs
//   accumulators in tensor memory      the DSPs' P registers (output stationary)
//   epilogue: TMEM -> regs, ReLU,      readout chains -> ReLU -> checksum,
//     write back                         result memory (one-shot blocks only)
//
// ---- what one k-step does ----------------------------------------------
// Block b, step k (0 <= k < K) reads BRAM address  a = (b*K + k) mod 512:
//     A column  : SUBP[s].mem[a][r*16 +: 16] = A[s*ROWS + r][k]   (s = sub, r = row)
//     B row     : bmem[a][c*16 +: 16]         = B[k][c]
// and all N_SUB*ROWS*COLS DSPs do  P[i][j] += A[i][k] * B[k][j]  in the same
// cycle. After K steps every P holds C[i][j] = sum_k A[i][k] * B[k][j]. The
// address keeps counting across blocks (wraps at 512), so with K = 256
// two different (A, B) slices alternate; TILE_CFG resets it to 0.
//
// ---- the enable (power experiment) --------------------------------------
// step = (en | one-shot pending) & not flushing. en comes from en_gen.v (the
// duty-cycle FSM: 100%..80% per 1 ms window). A cycle without a step reads
// no BRAM and changes no register in the datapath, so a stopped SM only
// burns clock power.
//
// ---- readout / checksum (the "epilogue") ---------------------------------
// When a block's last step reaches the DSPs, all P values are copied into
// the 4 readout chains (tensor_core.v) and shifted out 4 per cycle over
// ROWS*COLS (16) cycles, while the DSPs already accumulate the next block.
// Each value goes through ReLU into a running 64-bit checksum; when the
// whole block is in, `chk` and `blk_done` update together (so a snapshot of
// the two is always consistent):
//     chk = sum over finished blocks of sum_ij max(C_b[i][j], 0)   mod 2^64
// The host checks chk / blk_done against host/tile_model.py. The adders
// are split so no carry chain is longer than 33 bits (400 MHz target).
// A block started with RUN_ONE (os_pulse) also writes its raw C values
// into the result memory (LUTRAM; read from clk_mem by host_if READ_RES)
// and flips os_ack_tog when done. Continuous blocks do not write it.
//
// Needs K >= K_MIN (40): the readout of one block must finish before the
// next block's results are captured.
//
// ---- clear (TILE_CFG) ----------------------------------------------------
// clr_pulse: latch K from cfg_k, address / k / checksum / blk_done -> 0,
// then HOLD cycles with no steps so every in-flight tag drains, then
// clr_ack_tog flips.
//
// Pipeline (issue cycle t): BRAM ren t+1, read latch t+2, DO_REG t+3,
// tensor_core.v t+4..t+8 (P updated end of t+7, captured end of t+8),
// results out t+9 .., checksum committed ~t+31.

module sm_core #(
  parameter integer N_SUB     = 4,      // sub-partitions (fixed 4: checksum adder tree)
  parameter integer ROWS      = 2,      // rows of C per sub-partition
  parameter integer COLS      = 8,      // columns of C
  parameter integer AW        = 9,      // BRAM address bits (depth 512)
  parameter integer KW        = 10,     // K register width (K <= 1023)
  parameter integer K_MIN     = 40,
  parameter integer K_DEFAULT = 256,
  parameter integer HOLD      = 32,
  parameter [31:0]  SEED_A    = 32'hA5A50000,   // power-on A contents (per SM)
  parameter [31:0]  SEED_B    = 32'h5A5A1234,   // power-on B contents (same in every SM)
  // plain parameters (Vivado [Synth 8-196] history): do not override
  parameter integer NCH       = ROWS * COLS,              // results per sub-partition (16)
  parameter integer NR        = N_SUB * ROWS * COLS,      // results per block (64)
  parameter integer RAW       = $clog2(N_SUB * ROWS * COLS),
  parameter integer CHW       = $clog2(ROWS * COLS)
)(
  input  wire                     clk,          // this SM's clock
  input  wire                     rst_n,        // synchronized to clk, held >= 64 cycles
  input  wire                     en,           // en_gen.v: this cycle is a working cycle

  input  wire                     clr_pulse,    // TILE_CFG (from tog_cdc)
  input  wire [KW-1:0]            cfg_k,        // quasi-static (clk_mem), sampled on clr_pulse
  output reg                      clr_ack_tog,
  input  wire                     os_pulse,     // RUN_ONE (from tog_cdc)
  output reg                      os_ack_tog,

  // host writes: write ports of the dual-clock BRAMs (wr_clk = clk_mem)
  input  wire                     wr_clk,
  input  wire                     wr_stb_a,
  input  wire                     wr_stb_b,
  input  wire [AW-1:0]            wr_addr,
  input  wire [N_SUB*ROWS*16-1:0] wr_data,      // A: all N_SUB*ROWS*16 bits; B: low COLS*16 bits

  // result memory read port (asynchronous read; used from clk_mem while idle)
  input  wire [RAW-1:0]           res_raddr,    // r = row*COLS + col
  output wire [47:0]              res_rdata,

  output reg  [31:0]              blk_done,
  output reg  [63:0]              chk,
  output wire                     busy
);

  localparam integer DEPTH = 1 << AW;
  localparam integer AWID  = ROWS * 16;
  localparam integer BWID  = COLS * 16;

  initial if (N_SUB != 4) $display("sm_core: ERROR -- the checksum adder tree is written for N_SUB = 4");

  // ---- power-on BRAM contents (constant functions -> Vivado can evaluate
  //      them; host/tile_model.py computes the same numbers) ---------------
  function [31:0] xs32;
    input [31:0] x_in;
    reg   [31:0] x;
    begin
      x = x_in;
      x = x ^ (x << 13);
      x = x ^ (x >> 17);
      x = x ^ (x << 5);
      xs32 = x;
    end
  endfunction

  function [31:0] hash3;
    input [31:0] seed, a, b;
    reg   [31:0] x;
    begin
      x = seed ^ (a * 32'h9E3779B9) ^ (b * 32'h85EBCA6B) ^ 32'h1;
      if (x == 32'd0) x = 32'h1;
      x = xs32(x);
      x = xs32(x);
      hash3 = xs32(x);
    end
  endfunction

  function [AWID-1:0] arow;
    input [31:0] seed, row;
    integer q;
    begin
      arow = {AWID{1'b0}};
      for (q = 0; q < ROWS; q = q + 1)
        arow[q*16 +: 16] = hash3(seed, row, q);
    end
  endfunction

  function [BWID-1:0] brow;
    input [31:0] seed, row;
    integer q;
    begin
      brow = {BWID{1'b0}};
      for (q = 0; q < COLS; q = q + 1)
        brow[q*16 +: 16] = hash3(seed, row, q);
    end
  endfunction

  // ---- synchronous reset (rst_n already synchronized, see sm_top.v) ------
  (* max_fanout = 32 *) reg srst = 1'b1;
  always @(posedge clk) srst <= ~rst_n;

  // =======================================================================
  // controller: k counter, block boundaries, one-shot, clear
  // =======================================================================
  reg [KW-1:0] kk        = {KW{1'b0}};
  reg [KW-1:0] k_m1      = K_DEFAULT - 1;
  reg          os_active = 1'b0;
  reg [5:0]    hold      = HOLD;
  reg          clr_pend  = 1'b0;

  wire flush    = (hold != 6'd0);
  wire step     = (en | os_active) & ~flush & ~clr_pulse & ~srst;
  wire is_first = (kk == {KW{1'b0}});
  wire is_last  = (kk == k_m1);
  localparam [KW-1:0] KMIN_V = K_MIN;
  wire [KW-1:0] k_req = (cfg_k < KMIN_V) ? KMIN_V : cfg_k;

  initial begin clr_ack_tog = 1'b0; os_ack_tog = 1'b0; end

  always @(posedge clk) begin
    if (srst) begin
      kk          <= {KW{1'b0}};
      k_m1        <= K_DEFAULT - 1;
      os_active   <= 1'b0;
      hold        <= HOLD;
      clr_pend    <= 1'b0;
      clr_ack_tog <= 1'b0;
    end else if (clr_pulse) begin
      kk        <= {KW{1'b0}};
      k_m1      <= k_req - 1'b1;
      os_active <= 1'b0;
      hold      <= HOLD;
      clr_pend  <= 1'b1;
    end else begin
      if (flush) begin
        hold <= hold - 1'b1;
        if (hold == 6'd1 && clr_pend) begin
          clr_pend    <= 1'b0;
          clr_ack_tog <= ~clr_ack_tog;
        end
      end
      if (step)
        kk <= is_last ? {KW{1'b0}} : kk + 1'b1;
      if (step && is_last) os_active <= os_pulse;   // one-shot block done (or the next one queued)
      else if (os_pulse)   os_active <= 1'b1;
    end
  end

  // tags: valid / first / last / one-shot, aligned with the BRAM DO_REG (t+3)
  reg [3:0] tg1 = 4'd0, tg2 = 4'd0, tg3 = 4'd0;      // {o, l, f, v}
  always @(posedge clk) begin
    tg1 <= {step & is_last & os_active, step & is_last, step & is_first, step};
    tg2 <= tg1;
    tg3 <= tg2;
  end

  // =======================================================================
  // B memory (shared by the 4 sub-partitions = "shared memory" B tile)
  // =======================================================================
  (* ram_style = "block" *) reg [BWID-1:0] bmem [0:DEPTH-1];
  integer bi;
  initial
    for (bi = 0; bi < DEPTH; bi = bi + 1)
      bmem[bi] = brow(SEED_B, bi);

  (* keep = "true" *) reg [BWID-1:0] b_wd = {BWID{1'b0}};
  (* keep = "true" *) reg [AW-1:0]   b_wa = {AW{1'b0}};
  (* keep = "true" *) reg            b_we = 1'b0;
  always @(posedge wr_clk) begin
    b_wd <= wr_data[BWID-1:0];
    b_wa <= wr_addr;
    b_we <= wr_stb_b;
  end
  always @(posedge wr_clk)
    if (b_we) bmem[b_wa] <= b_wd;

  (* keep = "true" *) reg          b_ren = 1'b0;
  (* keep = "true" *) reg [AW-1:0] b_ra  = {AW{1'b0}};
  reg [BWID-1:0] b_r1 = {BWID{1'b0}}, b_r2 = {BWID{1'b0}};
  always @(posedge clk) begin
    b_ren <= step;
    if (srst | clr_pulse) b_ra <= {AW{1'b0}};
    else if (b_ren)       b_ra <= b_ra + 1'b1;
    if (b_ren) b_r1 <= bmem[b_ra];
    b_r2 <= b_r1;
  end

  // =======================================================================
  // A memories + sub-partitions (one tensor core each)
  // =======================================================================
  wire [N_SUB*48-1:0] sub_out;

  genvar s;
  generate
    for (s = 0; s < N_SUB; s = s + 1) begin : SUBP
      localparam [31:0] SA = SEED_A ^ ((s + 1) * 32'h7F4A7C15);

      (* ram_style = "block" *) reg [AWID-1:0] mem [0:DEPTH-1];
      integer i;
      initial
        for (i = 0; i < DEPTH; i = i + 1)
          mem[i] = arow(SA, i);

      (* keep = "true" *) reg [AWID-1:0] wd = {AWID{1'b0}};
      (* keep = "true" *) reg [AW-1:0]   wa = {AW{1'b0}};
      (* keep = "true" *) reg            we = 1'b0;
      always @(posedge wr_clk) begin
        wd <= wr_data[s*AWID +: AWID];
        wa <= wr_addr;
        we <= wr_stb_a;
      end
      always @(posedge wr_clk)
        if (we) mem[wa] <= wd;

      (* keep = "true" *) reg          ren = 1'b0;
      (* keep = "true" *) reg [AW-1:0] ra  = {AW{1'b0}};
      reg [AWID-1:0] r1 = {AWID{1'b0}}, r2 = {AWID{1'b0}};
      always @(posedge clk) begin
        ren <= step;
        if (srst | clr_pulse) ra <= {AW{1'b0}};
        else if (ren)         ra <= ra + 1'b1;
        if (ren) r1 <= mem[ra];
        r2 <= r1;
      end

      tensor_core #(.ROWS(ROWS), .COLS(COLS)) u_tc (
        .clk  (clk),
        .a_in (r2),
        .b_in (b_r2),
        .tv   (tg3[0]),
        .tf   (tg3[1]),
        .tl   (tg3[2]),
        .out  (sub_out[s*48 +: 48])
      );
    end
  endgenerate

  // =======================================================================
  // readout control (same timing as the chains inside tensor_core.v)
  // =======================================================================
  reg [3:0] dv = 4'd0, dl = 4'd0, dos = 4'd0;   // t+4 .. t+7
  reg       capev = 1'b0, cap_os = 1'b0;        // t+8
  always @(posedge clk) begin
    dv     <= {dv[2:0],  tg3[0]};
    dl     <= {dl[2:0],  tg3[2]};
    dos    <= {dos[2:0], tg3[3]};
    capev  <= dv[3] & dl[3];
    cap_os <= dos[3];
  end

  reg [CHW:0]   rd_cnt = {(CHW+1){1'b0}};
  reg [CHW-1:0] rd_j   = {CHW{1'b0}};
  reg           rd_os  = 1'b0;
  always @(posedge clk) begin
    if (srst | flush) begin
      rd_cnt <= {(CHW+1){1'b0}};
      rd_j   <= {CHW{1'b0}};
      rd_os  <= 1'b0;
    end else if (capev) begin
      rd_cnt <= NCH;
      rd_j   <= {CHW{1'b0}};
      rd_os  <= cap_os;
    end else if (rd_cnt != 0) begin
      rd_cnt <= rd_cnt - 1'b1;
      rd_j   <= rd_j + 1'b1;
    end
  end
  wire o_v     = (rd_cnt != 0);
  wire o_first = o_v & (rd_j == {CHW{1'b0}});
  wire o_last  = o_v & (rd_cnt == 1);

  // ---- result memory: raw C values of one-shot blocks ------------------
  wire [N_SUB*48-1:0] rm_out;
  generate
    for (s = 0; s < N_SUB; s = s + 1) begin : RES
      (* ram_style = "distributed" *) reg [47:0] rm [0:NCH-1];
      integer i;
      initial for (i = 0; i < NCH; i = i + 1) rm[i] = 48'd0;
      always @(posedge clk)
        if (o_v & rd_os) rm[rd_j] <= sub_out[s*48 +: 48];
      assign rm_out[s*48 +: 48] = rm[res_raddr[CHW-1:0]];
    end
  endgenerate
  assign res_rdata = rm_out[res_raddr[RAW-1:CHW]*48 +: 48];

  // =======================================================================
  // checksum: ReLU, split into bits [23:0] and [46:24], add up the block,
  // then add the block total into chk -- every adder <= 33 bits
  // =======================================================================
  integer q;
  reg              c1_v = 1'b0, c1_f = 1'b0, c1_l = 1'b0, c1_o = 1'b0;
  reg [N_SUB*24-1:0] lo1 = {N_SUB*24{1'b0}};
  reg [N_SUB*23-1:0] hi1 = {N_SUB*23{1'b0}};
  always @(posedge clk) begin
    c1_v <= o_v & ~flush & ~srst;
    c1_f <= o_first;
    c1_l <= o_last;
    c1_o <= rd_os;
    for (q = 0; q < N_SUB; q = q + 1) begin
      lo1[q*24 +: 24] <= sub_out[q*48 + 47] ? 24'd0 : sub_out[q*48      +: 24];
      hi1[q*23 +: 23] <= sub_out[q*48 + 47] ? 23'd0 : sub_out[q*48 + 24 +: 23];
    end
  end

  reg        c2_v = 1'b0, c2_f = 1'b0, c2_l = 1'b0, c2_o = 1'b0;
  reg [24:0] lo01 = 25'd0, lo23 = 25'd0;
  reg [23:0] hi01 = 24'd0, hi23 = 24'd0;
  always @(posedge clk) begin
    c2_v <= c1_v & ~flush;  c2_f <= c1_f;  c2_l <= c1_l;  c2_o <= c1_o;
    lo01 <= {1'b0, lo1[0*24 +: 24]} + {1'b0, lo1[1*24 +: 24]};
    lo23 <= {1'b0, lo1[2*24 +: 24]} + {1'b0, lo1[3*24 +: 24]};
    hi01 <= {1'b0, hi1[0*23 +: 23]} + {1'b0, hi1[1*23 +: 23]};
    hi23 <= {1'b0, hi1[2*23 +: 23]} + {1'b0, hi1[3*23 +: 23]};
  end

  reg        c3_v = 1'b0, c3_f = 1'b0, c3_l = 1'b0, c3_o = 1'b0;
  reg [25:0] lo4 = 26'd0;
  reg [24:0] hi4 = 25'd0;
  always @(posedge clk) begin
    c3_v <= c2_v & ~flush;  c3_f <= c2_f;  c3_l <= c2_l;  c3_o <= c2_o;
    lo4  <= {1'b0, lo01} + {1'b0, lo23};
    hi4  <= {1'b0, hi01} + {1'b0, hi23};
  end

  reg [31:0] blk_lo = 32'd0, blk_hi = 32'd0;    // this block's ReLU sum, split
  reg        c4_l = 1'b0, c4_o = 1'b0;
  always @(posedge clk) begin
    c4_l <= c3_v & c3_l & ~flush;
    c4_o <= c3_o;
    if (c3_v) begin
      blk_lo <= (c3_f ? 32'd0 : blk_lo) + {6'd0, lo4};
      blk_hi <= (c3_f ? 32'd0 : blk_hi) + {7'd0, hi4};
    end
  end

  reg [32:0] s_mid = 33'd0;                     // block sum S = s_mid * 2^24 + s_lo
  reg [23:0] s_lo  = 24'd0;
  reg        c5_l = 1'b0, c5_o = 1'b0;
  always @(posedge clk) begin
    c5_l <= c4_l & ~flush;
    c5_o <= c4_o;
    if (c4_l) begin
      s_mid <= {1'b0, blk_hi} + {25'd0, blk_lo[31:24]};
      s_lo  <= blk_lo[23:0];
    end
  end

  reg [32:0] lo_sum = 33'd0;
  reg [24:0] s_hi   = 25'd0;
  reg        c6_l = 1'b0, c6_o = 1'b0;
  always @(posedge clk) begin
    c6_l <= c5_l & ~flush;
    c6_o <= c5_o;
    if (c5_l) begin
      lo_sum <= {1'b0, chk[31:0]} + {1'b0, s_mid[7:0], s_lo};
      s_hi   <= s_mid[32:8];
    end
  end

  initial begin chk = 64'd0; blk_done = 32'd0; end
  always @(posedge clk) begin
    if (srst) begin
      chk        <= 64'd0;
      blk_done   <= 32'd0;
      os_ack_tog <= 1'b0;
    end else if (clr_pulse) begin
      chk        <= 64'd0;
      blk_done   <= 32'd0;
    end else if (c6_l & ~flush) begin
      chk        <= {chk[63:32] + {7'd0, s_hi} + {31'd0, lo_sum[32]}, lo_sum[31:0]};
      blk_done   <= blk_done + 1'b1;
      if (c6_o) os_ack_tog <= ~os_ack_tog;
    end
  end

  assign busy = (kk != {KW{1'b0}}) | os_active | o_v | flush;

endmodule
