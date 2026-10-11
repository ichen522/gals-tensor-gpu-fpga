// win_ctrl.v  (10_6, build 0x90: v7.3 window engine; 24-bit N_k records,
//              per-SM pseudo-random power-on sequences)
//
// Window controller for the duty-cycle experiment. Lives in clk_mem
// (200 MHz) and is shared by all tiles:
//
//   1. Generates the sample-window boundaries: one window = WIN_LEN
//      clk_mem cycles (default 200,000 = 1 ms = the professor's 1 kS/s).
//      Using clk_mem (fixed) instead of each tile's own clock means every
//      tile's window covers the SAME 1 ms of real time, whatever frequency
//      that tile happens to run at -- which is what lets all tiles' N_k be
//      lined up against one board-level power reading.
//   2. Holds each tile's duty SEQUENCE: SEQ_LEN entries per tile, one OFF
//      value per window (see en_gen.v), looped forever. The host can
//      rewrite it over UART (OP_SEQ_WR). 10_6 power-up default: every SM
//      gets its own 256-entry pseudo-random sequence in PAIRS (x, 20 - x),
//      x = hash(SM, pair) mod 21, so
//         * every window is 80..100% active (the meeting's range),
//         * every adjacent pair averages exactly 90% (flat temperature),
//         * the 24 SMs are practically uncorrelated (the regression can
//           separate 24 W's -- the v7 20-entry pattern only has 20 distinct
//           phases, too few for 24 SMs).
//      host/gals_host.py default_sequences() computes the same values.
//   3. Collects every tile's N_k (en_gen.v nk_hold) half a window after
//      each boundary and pushes one record per window into a FIFO the host
//      drains with OP_READ_WIN:
//         record = { window_index[31:0], N_sm0[NK_W-1:0], ..., N_sm(T-1)[NK_W-1:0] }
//      N_k is NK_W = 24 bits (saturating at 2^24-1 = 41 ms at 400 MHz):
//      76 bytes per window for 24 SMs, which 921,600 baud can carry every
//      1 ms (92 bytes); 32-bit values (100 bytes) could not.
//      window_index counts from 0 at every start; the FIFO holds
//      REC_DEPTH records (1024 = ~1 s at 1 ms windows) and sets `overflow`
//      (sticky) instead of overwriting if the host falls behind.
//
// Run control: running = cfg run flag (OP_SEQ_CFG) OR hw_run (DIP switch
// 4). On start: sequence pointer -> 0, window index -> 0, window length
// and sequence length are latched (changes while running take effect on
// the next start).
//
// Window timing in clk_mem cycles (cnt = 0 is the boundary cycle):
//     cnt == 0        : win_toggle flips  -> each tile closes its window
//     cnt == SAFE     : load next window's duty values into duty_off_flat
//     cnt == WIN_LEN/2: push the record for the window that just closed
// duty_off_flat therefore never changes within SAFE (64) cycles of a
// toggle, which is what makes it safe for en_gen.v to sample it without a
// per-bit synchronizer (see that file).

module win_ctrl #(
  parameter integer NUM_TILES       = 4,
  parameter integer SEQ_DEPTH       = 256,      // entries per tile (power of 2)
  parameter integer DUTY_W          = 8,
  parameter integer DUTY_DEN        = 100,
  parameter integer WIN_LEN_DEFAULT = 200_000,  // clk_mem cycles per window
  parameter integer SEQ_LEN_DEFAULT = SEQ_DEPTH,
  parameter integer REC_DEPTH       = 1024,     // power of 2
  parameter integer NK_W            = 24,       // bits per N_k in a record
  parameter integer REC_W           = 32 + NK_W*NUM_TILES
)(
  input  wire                         clk,          // clk_mem
  input  wire                         rst_n,

  // ---- configuration from host_if.v (all 1-cycle pulses + data) ------
  input  wire                         cfg_wr,
  input  wire [7:0]                   cfg_flags,    // bit0 = run, bit1 = clear FIFO/overflow
  input  wire [15:0]                  cfg_seq_len,  // 1..SEQ_DEPTH (0 = keep current)
  input  wire [31:0]                  cfg_win_len,  // clk_mem cycles, >= MIN_WIN (0 = keep current)
  input  wire                         hw_run,       // async: DIP switch override

  input  wire                         seq_wr,
  input  wire [7:0]                   seq_wr_tile,
  input  wire [15:0]                  seq_wr_idx,
  input  wire [DUTY_W-1:0]            seq_wr_val,

  // ---- to every tile's en_gen.v -----------------------------------------
  output reg                          seq_run,
  output reg                          win_toggle,
  output reg  [NUM_TILES*DUTY_W-1:0]  duty_off_flat,

  // ---- from every tile's en_gen.v (quasi-static) -------------------------
  input  wire [NUM_TILES*32-1:0]      nk_hold_flat,

  // ---- record FIFO, drained by host_if.v ----------------------------------
  input  wire                         rec_pop,      // 1-cycle pulse: drop head
  output reg  [REC_W-1:0]             rec_head,     // valid >= 3 cycles after a push/pop
  output reg  [15:0]                  rec_count,
  output reg                          overflow,
  output wire                         running,
  output reg  [31:0]                  win_count     // windows started since last start
);

  localparam integer SEQ_AW  = $clog2(SEQ_DEPTH);
  localparam integer TIDX_W  = (NUM_TILES <= 1) ? 1 : $clog2(NUM_TILES);
  localparam integer REC_AW  = $clog2(REC_DEPTH);
  localparam integer SAFE    = 64;
  localparam integer MIN_WIN = 1024;

  // ---- configuration registers --------------------------------------------
  reg        cfg_run;
  reg [15:0] seq_len_cfg;
  reg [31:0] win_len_cfg;

  (* ASYNC_REG = "TRUE" *) reg [1:0] hw_run_s;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) hw_run_s <= 2'b00;
    else        hw_run_s <= {hw_run_s[0], hw_run};
  end
  assign running = cfg_run | hw_run_s[1];

  wire do_clear = cfg_wr & cfg_flags[1];

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      cfg_run     <= 1'b0;
      seq_len_cfg <= SEQ_LEN_DEFAULT;
      win_len_cfg <= WIN_LEN_DEFAULT;
    end else if (cfg_wr) begin
      cfg_run <= cfg_flags[0];
      if (cfg_seq_len != 16'd0)
        seq_len_cfg <= (cfg_seq_len > SEQ_DEPTH) ? SEQ_DEPTH : cfg_seq_len;
      if (cfg_win_len != 32'd0)
        win_len_cfg <= (cfg_win_len < MIN_WIN) ? MIN_WIN : cfg_win_len;
    end
  end

  // ---- duty sequence memory: NUM_TILES x SEQ_DEPTH entries ----------------
  reg [DUTY_W-1:0] seq_mem [0:NUM_TILES*SEQ_DEPTH-1];

  // Power-on sequences: constant functions (Vivado evaluates them for the
  // RAM init; a reg array filled in an initial block from non-constant data
  // would be ignored). Same xorshift hash as sm_core.v.
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

  // OFF value of SM t, window i: pairs (x, 20 - x), x = hash mod 21
  function [DUTY_W-1:0] def_off;
    input [31:0] t, i;
    reg   [31:0] x;
    begin
      x = hash3(32'h5EC0D0FF, t, i >> 1) % 21;
      def_off = i[0] ? (20 - x) : x;
    end
  endfunction

  integer ti, si;
  initial
    for (ti = 0; ti < NUM_TILES; ti = ti + 1)
      for (si = 0; si < SEQ_DEPTH; si = si + 1)
        seq_mem[ti*SEQ_DEPTH + si] = def_off(ti, si);

  always @(posedge clk)
    if (seq_wr && (seq_wr_idx < SEQ_DEPTH) && (seq_wr_tile < NUM_TILES))
      seq_mem[seq_wr_tile[TIDX_W-1:0]*SEQ_DEPTH + seq_wr_idx[SEQ_AW-1:0]] <= seq_wr_val;

  reg  [TIDX_W-1:0]  rd_tile;
  reg  [SEQ_AW-1:0]  rd_idx;
  reg  [DUTY_W-1:0]  seq_rdata;
  always @(posedge clk)
    seq_rdata <= seq_mem[rd_tile*SEQ_DEPTH + rd_idx];

  // ---- window state machine -----------------------------------------------
  localparam [2:0] S_IDLE = 3'd0, S_LOAD = 3'd1, S_PRE = 3'd2, S_RUN = 3'd3;
  reg  [2:0]  state;
  reg  [31:0] cnt;
  reg  [31:0] win_len_l;
  reg  [15:0] seq_len_l;
  reg  [15:0] ptr;            // sequence index of the window being prepared
  reg  [31:0] ticks;          // window boundaries issued since start
  reg         load_go;        // start a duty load for `ptr`
  reg         load_busy;
  reg  [TIDX_W:0] load_t;     // tile being loaded (one extra state for read latency)
  reg         load_v;         // seq_rdata is valid for tile load_t-1

  wire [31:0] cap_point = {1'b0, win_len_l[31:1]};

  // record FIFO
  reg [REC_W-1:0] rec_mem [0:REC_DEPTH-1];
  reg [REC_AW-1:0] wr_ptr, rd_ptr;
  reg [REC_AW:0]   count;
  reg              push;
  reg [REC_W-1:0]  push_data;

  // records are sent SM 0 first, right after the window index; each N_k
  // saturates at NK_W bits
  wire [NK_W*NUM_TILES-1:0] nk_tile0_first;
  localparam [31:0] NK_MAX = (32'd1 << NK_W) - 32'd1;
  genvar gt;
  generate
    for (gt = 0; gt < NUM_TILES; gt = gt + 1) begin : NK_ORDER
      wire [31:0] nk = nk_hold_flat[gt*32 +: 32];
      assign nk_tile0_first[(NUM_TILES-1-gt)*NK_W +: NK_W] = (nk > NK_MAX) ? NK_MAX[NK_W-1:0] : nk[NK_W-1:0];
    end
  endgenerate

  integer di;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state         <= S_IDLE;
      seq_run       <= 1'b0;
      win_toggle    <= 1'b0;
      duty_off_flat <= {(NUM_TILES*DUTY_W){1'b0}};
      cnt           <= 32'd0;
      win_len_l     <= WIN_LEN_DEFAULT;
      seq_len_l     <= SEQ_LEN_DEFAULT;
      ptr           <= 16'd0;
      ticks         <= 32'd0;
      win_count     <= 32'd0;
      load_go       <= 1'b0;
      load_busy     <= 1'b0;
      load_t        <= {(TIDX_W+1){1'b0}};
      load_v        <= 1'b0;
      rd_tile       <= {TIDX_W{1'b0}};
      rd_idx        <= {SEQ_AW{1'b0}};
      push          <= 1'b0;
      push_data     <= {REC_W{1'b0}};
    end else begin
      load_go <= 1'b0;
      push    <= 1'b0;

      // ---- duty loader: read seq_mem[t][ptr] for t = 0..NUM_TILES-1 ----
      if (load_go) begin
        load_busy <= 1'b1;
        load_t    <= {(TIDX_W+1){1'b0}};
        load_v    <= 1'b0;
        rd_tile   <= {TIDX_W{1'b0}};
        rd_idx    <= ptr[SEQ_AW-1:0];
      end else if (load_busy) begin
        // seq_rdata (registered read) now holds the entry addressed last cycle
        if (load_v)
          duty_off_flat[(load_t-1)*DUTY_W +: DUTY_W] <= seq_rdata;
        if (load_t == NUM_TILES) begin
          load_busy <= 1'b0;
          load_v    <= 1'b0;
        end else begin
          load_v  <= 1'b1;
          load_t  <= load_t + 1'b1;
          rd_tile <= load_t[TIDX_W-1:0] + 1'b1;
        end
      end

      case (state)
        S_IDLE: begin
          seq_run <= 1'b0;
          if (running) begin
            win_len_l <= win_len_cfg;
            seq_len_l <= seq_len_cfg;
            ptr       <= 16'd0;
            ticks     <= 32'd0;
            win_count <= 32'd0;
            cnt       <= 32'd0;
            seq_run   <= 1'b1;
            load_go   <= 1'b1;            // duty for window 0
            state     <= S_LOAD;
          end
        end

        S_LOAD: begin                     // wait for the loader, then SAFE cycles
          if (!running) state <= S_IDLE;
          else if (!load_go && !load_busy) begin
            cnt   <= 32'd0;
            state <= S_PRE;
          end
        end

        S_PRE: begin
          if (!running) state <= S_IDLE;
          else if (cnt == SAFE) begin     // first boundary: window 0 starts
            win_toggle <= ~win_toggle;
            ticks      <= 32'd1;
            win_count  <= 32'd1;
            cnt        <= 32'd0;
            state      <= S_RUN;
          end else begin
            cnt <= cnt + 32'd1;
          end
        end

        S_RUN: begin
          if (!running) begin
            state <= S_IDLE;
          end else begin
            if (cnt == win_len_l - 32'd1) begin
              cnt        <= 32'd0;
              win_toggle <= ~win_toggle;  // window (ticks-1) ends, window ticks starts
              ticks      <= ticks + 32'd1;
              win_count  <= win_count + 32'd1;
            end else begin
              cnt <= cnt + 32'd1;
            end

            if (cnt == SAFE) begin        // prepare the NEXT window's duty values
              ptr     <= (ptr + 16'd1 >= seq_len_l) ? 16'd0 : ptr + 16'd1;
              load_go <= 1'b1;
            end

            if (cnt == cap_point && ticks >= 32'd2) begin
              push      <= 1'b1;
              push_data <= {ticks - 32'd2, nk_tile0_first};
            end
          end
        end

        default: state <= S_IDLE;
      endcase
    end
  end

  // ptr is updated in the same cycle load_go is raised; the loader samples
  // ptr one cycle later (on load_go), so it always sees the new value.

  // ---- record FIFO (BRAM, registered head read) -----------------------
  // v7.2: + the BRAM's output register (rec_q -> rec_head). Without it the
  // BRAM clock-to-out (1.8 ns) plus the route into host_if's reply buffer
  // was 4.49 ns of a 5 ns cycle (-0.044 ns). host_if waits thousands of
  // cycles (a whole UART reply) between pops, so one more cycle is free.
  reg [REC_W-1:0] rec_q;
  always @(posedge clk) begin
    if (push && (count < REC_DEPTH)) rec_mem[wr_ptr] <= push_data;
    rec_q    <= rec_mem[rd_ptr];
    rec_head <= rec_q;
  end

  wire do_push = push && (count < REC_DEPTH);
  wire do_pop  = rec_pop && (count != 0);

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      wr_ptr    <= {REC_AW{1'b0}};
      rd_ptr    <= {REC_AW{1'b0}};
      count     <= {(REC_AW+1){1'b0}};
      overflow  <= 1'b0;
      rec_count <= 16'd0;
    end else if (do_clear) begin
      wr_ptr    <= {REC_AW{1'b0}};
      rd_ptr    <= {REC_AW{1'b0}};
      count     <= {(REC_AW+1){1'b0}};
      overflow  <= 1'b0;
      rec_count <= 16'd0;
    end else begin
      if (do_push) wr_ptr <= wr_ptr + 1'b1;
      if (do_pop)  rd_ptr <= rd_ptr + 1'b1;
      if (push && !(count < REC_DEPTH)) overflow <= 1'b1;
      case ({do_push, do_pop})
        2'b10:   count <= count + 1'b1;
        2'b01:   count <= count - 1'b1;
        default: count <= count;
      endcase
      rec_count <= {{(16-REC_AW-1){1'b0}}, count};
    end
  end

endmodule
