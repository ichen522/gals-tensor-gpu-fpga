// tb_win_ctrl.v
//
// Window engine end to end, without the (slow-to-simulate) MAC tiles:
// win_ctrl.v on a 200 MHz clk_mem + four en_gen.v instances on four
// different tile clocks (466.67 / 400 / 300 / 250 MHz). Checks:
//   1. Records come out with consecutive window indices from 0.
//   2. Every record's N for every tile equals an independent count of
//      that tile's en=1 cycles between its own window ticks.
//   3. Every N matches the POWER-ON duty sequence (10_6: per-SM hash, pairs
//      (x, 20 - x)): tile t, window k uses OFF = def_off(t, k), so
//      N ~= cycles_in_window*(100-OFF)/100. Also prints the per-tile
//      20-window average activity (exactly 90%: 10 pairs).
//      Records carry 24-bit N_k (NK_W).
//   4. All tiles' windows cover the same real time (cycle counts scale
//      with each tile's own frequency).
//   5. OP_SEQ_WR path: rewriting tile 0's sequence changes its N_k.
//   6. FIFO overflow flag + clear (REC_DEPTH = 16 here).
`timescale 1ns/1ps

module tb_win_ctrl;
  localparam NUM_TILES = 4;
  localparam WIN       = 2000;        // clk_mem cycles per window (10 us) for speed
  localparam REC_DEPTH = 16;
  localparam NK_W      = 24;
  localparam REC_W     = 32 + NK_W*NUM_TILES;

  reg clk_mem = 0, rst_n = 0;
  always #2.5 clk_mem = ~clk_mem;

  reg [NUM_TILES-1:0] tclk = 0;
  always #1.0715 tclk[0] = ~tclk[0];   // 466.67 MHz
  always #1.25   tclk[1] = ~tclk[1];   // 400 MHz
  always #1.6667 tclk[2] = ~tclk[2];   // 300 MHz
  always #2.0    tclk[3] = ~tclk[3];   // 250 MHz

  reg        cfg_wr = 0; reg [7:0] cfg_flags = 0; reg [15:0] cfg_seq_len = 0; reg [31:0] cfg_win_len = 0;
  reg        hw_run = 0;
  reg        seq_wr = 0; reg [7:0] seq_wr_tile = 0; reg [15:0] seq_wr_idx = 0; reg [7:0] seq_wr_val = 0;
  reg        rec_pop = 0;
  wire       seq_run, win_toggle, overflow, running;
  wire [NUM_TILES*8-1:0]  duty_off_flat;
  wire [NUM_TILES*32-1:0] nk_flat;
  wire [REC_W-1:0] rec_head;
  wire [15:0] rec_count;
  wire [31:0] win_count;

  win_ctrl #(
    .NUM_TILES(NUM_TILES), .SEQ_DEPTH(256), .DUTY_W(8), .DUTY_DEN(100),
    .WIN_LEN_DEFAULT(200000), .REC_DEPTH(REC_DEPTH), .NK_W(NK_W), .REC_W(REC_W)
  ) dut (
    .clk(clk_mem), .rst_n(rst_n),
    .cfg_wr(cfg_wr), .cfg_flags(cfg_flags), .cfg_seq_len(cfg_seq_len), .cfg_win_len(cfg_win_len),
    .hw_run(hw_run),
    .seq_wr(seq_wr), .seq_wr_tile(seq_wr_tile), .seq_wr_idx(seq_wr_idx), .seq_wr_val(seq_wr_val),
    .seq_run(seq_run), .win_toggle(win_toggle), .duty_off_flat(duty_off_flat),
    .nk_hold_flat(nk_flat),
    .rec_pop(rec_pop), .rec_head(rec_head), .rec_count(rec_count), .overflow(overflow),
    .running(running), .win_count(win_count)
  );

  // ---- four tiles' enable FSMs + independent observers -------------------
  integer obs_n   [0:NUM_TILES-1][0:255];   // en count of each closed window
  integer obs_cyc [0:NUM_TILES-1][0:255];   // cycles in each closed window
  integer obs_cnt [0:NUM_TILES-1];

  genvar gt;
  generate
    for (gt = 0; gt < NUM_TILES; gt = gt + 1) begin : T
      wire en, tick;
      wire [31:0] act;
      en_gen #(.DUTY_DEN(100), .DUTY_W(8)) u_en (
        .clk(tclk[gt]), .rst_n(rst_n),
        .tile_en_in(1'b1), .seq_run_in(seq_run), .win_toggle_in(win_toggle),
        .duty_off_in(duty_off_flat[gt*8 +: 8]),
        .en(en), .activity_count(act), .nk_hold(nk_flat[gt*32 +: 32]),
        .duty_cur(), .tick(tick)
      );
      integer c_en = 0, c_cyc = 0;
      always @(posedge tclk[gt]) if (rst_n) begin
        if (tick) begin
          if (obs_cnt[gt] < 256) begin
            obs_n[gt][obs_cnt[gt]]   = c_en + en;
            obs_cyc[gt][obs_cnt[gt]] = c_cyc + 1;
          end
          obs_cnt[gt] = obs_cnt[gt] + 1;
          c_en = 0; c_cyc = 0;
        end else begin
          c_en = c_en + en; c_cyc = c_cyc + 1;
        end
      end
    end
  endgenerate

  // power-on duty sequence (independent copy of the hash in win_ctrl.v)
  function automatic [31:0] xs32 (input [31:0] x_in);
    reg [31:0] x;
    begin
      x = x_in; x = x ^ (x << 13); x = x ^ (x >> 17); x = x ^ (x << 5); xs32 = x;
    end
  endfunction
  function automatic integer def_off (input integer t, input integer i);
    reg [31:0] x;
    begin
      x = 32'h5EC0D0FF ^ (t * 32'h9E3779B9) ^ ((i >> 1) * 32'h85EBCA6B) ^ 32'h1;
      if (x == 0) x = 1;
      x = xs32(xs32(xs32(x))) % 21;
      def_off = (i % 2) ? 20 - x : x;
    end
  endfunction
  integer errors = 0;
  integer t, k, n_rec, idx, off, expn, nk, sum_act [0:NUM_TILES-1];
  integer rec_base;   // obs index offset for the current run

  task send_cfg;
    input [7:0] flags; input [15:0] slen; input [31:0] wlen;
    begin
      @(posedge clk_mem); #0.1;
      cfg_flags = flags; cfg_seq_len = slen; cfg_win_len = wlen; cfg_wr = 1;
      @(posedge clk_mem); #0.1;
      cfg_wr = 0;
    end
  endtask

  task pop_and_check;
    input integer expect_idx;
    input integer tile0_custom;   // 1 = tile 0 uses the custom sequence
    reg [31:0] r_idx;
    integer tt;
    begin
      @(posedge clk_mem); #0.1;
      r_idx = rec_head[REC_W-1 -: 32];
      if (r_idx !== expect_idx) begin
        $display("ERROR: record index %0d, expected %0d", r_idx, expect_idx); errors = errors + 1;
      end
      for (tt = 0; tt < NUM_TILES; tt = tt + 1) begin
        nk  = rec_head[REC_W-33-NK_W*tt -: NK_W];
        if (tile0_custom && tt == 0) off = (expect_idx % 2) ? 0 : 50;
        else                         off = def_off(tt, expect_idx);
        if (nk !== obs_n[tt][rec_base + expect_idx + 1]) begin
          $display("ERROR: rec %0d tile %0d: N=%0d, observed %0d", expect_idx, tt, nk,
                   obs_n[tt][rec_base + expect_idx + 1]);
          errors = errors + 1;
        end
        expn = (obs_cyc[tt][rec_base + expect_idx + 1] * (100 - off)) / 100;
        if (nk + 2 < expn || nk > expn + 2) begin
          $display("ERROR: rec %0d tile %0d: N=%0d, expected ~%0d (OFF=%0d)", expect_idx, tt, nk, expn, off);
          errors = errors + 1;
        end
        sum_act[tt] = sum_act[tt] + 100 - off;
      end
      rec_pop = 1; @(posedge clk_mem); #0.1; rec_pop = 0;
      repeat (3) @(posedge clk_mem);
    end
  endtask

  initial begin
    for (t = 0; t < NUM_TILES; t = t + 1) begin obs_cnt[t] = 0; sum_act[t] = 0; end

    rst_n = 0;
    repeat (10) @(posedge clk_mem);
    rst_n = 1;
    repeat (10) @(posedge clk_mem);

    // ---- run with the default sequence, window = 2000 clk_mem cycles ----
    rec_base = 0;
    send_cfg(8'h01, 16'd0, WIN);          // run, keep seq_len (256), 10 us windows
    // drain records as they arrive: 20 windows = one full sequence period
    for (k = 0; k < 20; k = k + 1) begin
      wait (rec_count != 0);
      repeat (4) @(posedge clk_mem);
      pop_and_check(k, 0);
    end
    for (t = 0; t < NUM_TILES; t = t + 1)
      $display("INFO tile %0d: 20-window average activity = %0d.%0d%% (target 90%%), last window %0d cycles",
               t, sum_act[t] / 20, (sum_act[t] * 10 / 20) % 10, obs_cyc[t][20]);
    for (t = 0; t < NUM_TILES; t = t + 1)
      if (sum_act[t] != 1800) begin
        $display("ERROR: tile %0d sequence average is not 90%%", t); errors = errors + 1;
      end
    // same real time for every tile: cycles scale with frequency
    $display("INFO cycles per 10 us window: %0d / %0d / %0d / %0d  (466.67 / 400 / 300 / 250 MHz)",
             obs_cyc[0][5], obs_cyc[1][5], obs_cyc[2][5], obs_cyc[3][5]);
    if (obs_cyc[0][5] < 4664 || obs_cyc[0][5] > 4670 || obs_cyc[3][5] < 2497 || obs_cyc[3][5] > 2503) begin
      $display("ERROR: window length in tile cycles is off"); errors = errors + 1;
    end

    // ---- stop, rewrite tile 0's sequence (50%, 100%, ...), restart -------
    send_cfg(8'h02, 16'd0, 32'd0);       // stop + clear
    repeat (WIN) @(posedge clk_mem);
    for (k = 0; k < 20; k = k + 1) begin
      @(posedge clk_mem); #0.1;
      seq_wr = 1; seq_wr_tile = 0; seq_wr_idx = k; seq_wr_val = (k % 2) ? 8'd0 : 8'd50;
      @(posedge clk_mem); #0.1;
      seq_wr = 0;
    end
    for (t = 0; t < NUM_TILES; t = t + 1) begin rec_base = obs_cnt[t]; end
    // obs index of the first tick of the new run is obs_cnt at start
    rec_base = obs_cnt[0];
    for (t = 1; t < NUM_TILES; t = t + 1)
      if (obs_cnt[t] != rec_base) begin
        $display("ERROR: tiles saw different numbers of window ticks (%0d vs %0d)", obs_cnt[t], rec_base);
        errors = errors + 1;
      end
    send_cfg(8'h01, 16'd0, 32'd0);
    for (k = 0; k < 6; k = k + 1) begin
      wait (rec_count != 0);
      repeat (4) @(posedge clk_mem);
      pop_and_check(k, 1);
    end
    $display("OK   rewritten tile-0 sequence (50%%/100%% alternating) shows up in N_k");

    // ---- overflow + clear --------------------------------------------------
    repeat ((REC_DEPTH + 3) * WIN) @(posedge clk_mem);   // don't pop
    if (!overflow || rec_count != REC_DEPTH) begin
      $display("ERROR: expected overflow with %0d records, got overflow=%0d count=%0d",
               REC_DEPTH, overflow, rec_count);
      errors = errors + 1;
    end else $display("OK   FIFO full -> overflow flag set");
    send_cfg(8'h03, 16'd0, 32'd0);       // keep running + clear
    repeat (4) @(posedge clk_mem);
    if (overflow || rec_count != 0) begin
      $display("ERROR: clear didn't reset overflow/count"); errors = errors + 1;
    end else $display("OK   clear flag empties the FIFO and resets overflow");

    // ---- hw_run (DIP switch) alone starts the engine ----------------------
    send_cfg(8'h00, 16'd0, 32'd0);       // stop
    repeat (100) @(posedge clk_mem);
    if (running) begin $display("ERROR: still running after stop"); errors = errors + 1; end
    hw_run = 1;
    repeat (10) @(posedge clk_mem);
    if (!running || !seq_run) begin
      $display("ERROR: hw_run did not start the engine"); errors = errors + 1;
    end else $display("OK   DIP-switch run input starts the engine");

    if (errors == 0) $display("TB_WIN_CTRL: PASS");
    else             $display("TB_WIN_CTRL: FAIL (%0d errors)", errors);
    $finish;
  end

  initial begin #20_000_000; $display("TB_WIN_CTRL: FAIL (timeout)"); $finish; end
endmodule
