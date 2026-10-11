`timescale 1ns/1ps
// tb_sm.v (was tb_tile.v) -- one sm_top.v (en_gen + sm_core with 4 tensor
// cores + handshakes) against a behavioural GEMM reference computed in the
// testbench.
//
//   SM clock 400 MHz (2.5 ns), clk_mem 200 MHz.
//   1. power-on BRAM contents: RUN_ONE x2 (blocks 0 and 1, K = 256), all
//      64 results (8 x 8) of each must equal C = A x B exactly; results are also
//      dumped to tb_out/tile_poweron.txt for host/check_model.py (proves
//      host/tile_model.py computes the same numbers as the RTL)
//   2. host writes: new A / B rows through the clk_mem write ports
//      (random data incl. -32768 / +32767), TILE_CFG K = 64, RUN_ONE x3
//   3. K clamp: TILE_CFG K = 5 -> 40 (K_MIN), RUN_ONE
//   4. continuous run (tile_en) with a random on/off enable and the duty
//      FSM, K = 256 and K = 64: snapshot {activity, blk_done, chk} at random
//      times while running; chk must always equal the ReLU sum of exactly
//      blk_done reference blocks (consistent snapshots); after stopping,
//      blk_done must equal floor(enabled steps / K)
//   5. TILE_CFG while running clears everything
`default_nettype none
module tb_sm;

  localparam integer K0    = 256;
  localparam integer N_SUB = 4;
  localparam integer ROWS  = 2;                 // rows per sub-partition
  localparam integer COLS  = 8;
  localparam integer NROW  = N_SUB * ROWS;      // 8
  localparam integer NRES  = NROW * COLS;       // 64
  localparam integer WDW   = NROW * 16;         // 128

  reg clk_mem = 1'b0;
  always #2.5 clk_mem = ~clk_mem;
  reg tclk = 1'b0;
  always #1.25 tclk = ~tclk;

  reg rst_n_mem = 1'b0;
  reg tile_rst_req = 1'b1;
  reg tile_en = 1'b0;
  reg seq_run = 1'b0, win_toggle = 1'b0;
  reg [7:0] duty_off = 8'd0;

  reg wr_stb_a = 1'b0, wr_stb_b = 1'b0;
  reg [8:0] wr_addr = 9'd0;
  reg [WDW-1:0] wr_data = {WDW{1'b0}};
  reg [5:0] res_raddr = 6'd0;
  wire [47:0] res_rdata;
  reg stat_req = 1'b0, clr_req = 1'b0, os_req = 1'b0;
  wire stat_ack, clr_ack, os_ack;
  wire [127:0] stat_data;
  reg [9:0] cfg_k = 10'd256;
  wire [31:0] nk_hold;

  sm_top #(.SEED_A(32'hA5A50000 ^ 32'h9E3779B9), .SEED_B(32'h5A5A1234)) dut (
    .clk_mem (clk_mem), .rst_n_mem (rst_n_mem),
    .tile_clk (tclk), .tile_rst_req (tile_rst_req),
    .tile_en_in (tile_en), .seq_run_in (seq_run), .win_toggle_in (win_toggle), .duty_off_in (duty_off),
    .nk_hold (nk_hold),
    .wr_stb_a (wr_stb_a), .wr_stb_b (wr_stb_b), .wr_addr (wr_addr), .wr_data (wr_data),
    .res_raddr (res_raddr), .res_rdata (res_rdata),
    .stat_req_tog (stat_req), .stat_ack_tog (stat_ack), .stat_data (stat_data),
    .clr_req_tog (clr_req), .clr_ack_tog (clr_ack), .cfg_k (cfg_k),
    .os_req_tog (os_req), .os_ack_tog (os_ack),
    .led_blk ()
  );

  // ---------------------------------------------------------------- reference
  reg signed [15:0] Aref [0:NROW-1][0:511];
  reg signed [15:0] Bref [0:511][0:COLS-1];
  longint Cref [0:NRES-1];
  integer errors = 0;
  integer kcur = K0;
  integer blk_model = 0;        // blocks the tile has finished since the last clear
  longint chk_model = 0;

  task load_ref_from_dut;
    integer a, r;
    begin
      for (a = 0; a < 512; a = a + 1) begin
        for (r = 0; r < ROWS; r = r + 1) begin
          Aref[0*ROWS + r][a] = dut.u_core.SUBP[0].mem[a][r*16 +: 16];
          Aref[1*ROWS + r][a] = dut.u_core.SUBP[1].mem[a][r*16 +: 16];
          Aref[2*ROWS + r][a] = dut.u_core.SUBP[2].mem[a][r*16 +: 16];
          Aref[3*ROWS + r][a] = dut.u_core.SUBP[3].mem[a][r*16 +: 16];
        end
        for (r = 0; r < COLS; r = r + 1)
          Bref[a][r] = dut.u_core.bmem[a][r*16 +: 16];
      end
    end
  endtask

  function automatic void ref_block (input integer b, input integer K);
    integer i, j, k, a;
    longint acc;
    begin
      for (i = 0; i < NROW; i = i + 1)
        for (j = 0; j < COLS; j = j + 1) begin
          acc = 0;
          for (k = 0; k < K; k = k + 1) begin
            a = (b * K + k) % 512;
            acc = acc + Aref[i][a] * Bref[a][j];
          end
          Cref[i*COLS + j] = acc;
        end
    end
  endfunction

  // ReLU sum of one block, cached per start address (blocks repeat)
  longint relu_cache [0:511];
  reg     relu_valid [0:511];
  integer relu_k = -1;
  function automatic longint relu_sum (input integer b, input integer K);
    integer i, st;
    longint s;
    begin
      st = (b * K) % 512;
      if (relu_k != K) begin
        for (i = 0; i < 512; i = i + 1) relu_valid[i] = 1'b0;
        relu_k = K;
      end
      if (!relu_valid[st]) begin
        ref_block(b, K);
        s = 0;
        for (i = 0; i < NRES; i = i + 1) if (Cref[i] > 0) s = s + Cref[i];
        relu_cache[st] = s;
        relu_valid[st] = 1'b1;
      end
      relu_sum = relu_cache[st];
    end
  endfunction

  task invalidate_cache; integer i; begin for (i = 0; i < 512; i = i + 1) relu_valid[i] = 1'b0; end endtask

  // ---------------------------------------------------------------- host ops
  task write_a (input integer addr, input [WDW-1:0] w);
    integer r;
    begin
      @(posedge clk_mem); wr_addr <= addr; wr_data <= w; wr_stb_a <= 1'b1;
      @(posedge clk_mem); wr_stb_a <= 1'b0;
      repeat (4) @(posedge clk_mem);
      for (r = 0; r < NROW; r = r + 1) Aref[r][addr] = w[r*16 +: 16];
      invalidate_cache;
    end
  endtask

  task write_b (input integer addr, input [127:0] w);
    integer c;
    begin
      @(posedge clk_mem); wr_addr <= addr; wr_data <= w; wr_stb_b <= 1'b1;
      @(posedge clk_mem); wr_stb_b <= 1'b0;
      repeat (4) @(posedge clk_mem);
      for (c = 0; c < COLS; c = c + 1) Bref[addr][c] = w[c*16 +: 16];
      invalidate_cache;
    end
  endtask

  task tile_cfg (input integer K);
    integer n;
    begin
      @(posedge clk_mem); cfg_k <= K; clr_req <= ~clr_req;
      n = 0;
      @(posedge clk_mem);
      while (clr_ack !== clr_req && n < 100000) begin @(posedge clk_mem); n = n + 1; end
      if (clr_ack !== clr_req) begin $display("FAIL tile_cfg: no ack"); errors = errors + 1; end
      kcur = (K < 40) ? 40 : K;
      blk_model = 0;
      chk_model = 0;
    end
  endtask

  task snapshot (output [31:0] act, output [31:0] blk, output [63:0] chk);
    integer n;
    begin
      @(posedge clk_mem); stat_req <= ~stat_req;
      n = 0;
      @(posedge clk_mem);
      while (stat_ack !== stat_req && n < 100000) begin @(posedge clk_mem); n = n + 1; end
      if (stat_ack !== stat_req) begin $display("FAIL snapshot: no ack"); errors = errors + 1; end
      act = stat_data[127:96];
      blk = stat_data[95:64];
      chk = stat_data[63:0];
    end
  endtask

  task run_one_and_check (input [255:0] label);
    integer n, r, bad;
    longint got;
    reg [31:0] act, blk; reg [63:0] chk;
    begin
      @(posedge clk_mem); os_req <= ~os_req;
      n = 0;
      @(posedge clk_mem);
      while (os_ack !== os_req && n < 100000) begin @(posedge clk_mem); n = n + 1; end
      if (os_ack !== os_req) begin $display("FAIL run_one %0s: no ack", label); errors = errors + 1; end
      ref_block(blk_model, kcur);
      chk_model = chk_model + relu_sum(blk_model, kcur);
      blk_model = blk_model + 1;
      bad = 0;
      for (r = 0; r < NRES; r = r + 1) begin
        @(posedge clk_mem); res_raddr <= r;
        repeat (3) @(posedge clk_mem);
        got = {{16{res_rdata[47]}}, res_rdata};
        if (got != Cref[r]) begin
          if (bad < 5) $display("FAIL %0s: C[%0d][%0d] = %0d, expected %0d", label, r/COLS, r%COLS, got, Cref[r]);
          bad = bad + 1;
        end
      end
      snapshot(act, blk, chk);
      if (blk != blk_model || chk != chk_model) begin
        $display("FAIL %0s: blk_done %0d (exp %0d) chk %h (exp %h)", label, blk, blk_model, chk, chk_model);
        bad = bad + 1;
      end
      if (bad) errors = errors + 1;
      else $display("OK   %0s: block %0d, K = %0d, %0d / %0d results exact, chk = %h", label, blk_model - 1, kcur, NRES, NRES, chk);
    end
  endtask

  integer fd;
  task dump_results (input integer b);
    integer r;
    begin
      for (r = 0; r < NRES; r = r + 1) begin
        @(posedge clk_mem); res_raddr <= r;
        repeat (3) @(posedge clk_mem);
        $fdisplay(fd, "res %0d %0d %0d", b, r, $signed(res_rdata));
      end
    end
  endtask

  // continuous run with a random enable; snapshots while running
  integer steps_seen;
  always @(posedge tclk) if (dut.u_core.step) steps_seen <= steps_seen + 1;

  task cont_run (input integer K, input integer cycles, input integer label_n);
    integer c, nsnap, bad;
    reg [31:0] act, blk, act0; reg [63:0] chk;
    longint sum;
    integer b;
    begin
      tile_cfg(K);
      snapshot(act0, blk, chk);
      steps_seen = 0;
      nsnap = 0; bad = 0;
      for (c = 0; c < cycles; c = c + 1) begin
        @(posedge clk_mem);
        if (($random & 32'hFF) == 0) tile_en <= ~tile_en;
        if (c > 2000 && ($random & 32'h7FF) == 0) begin
          snapshot(act, blk, chk);
          sum = 0;
          for (b = 0; b < blk; b = b + 1) sum = sum + relu_sum(b, K);
          if (chk != sum) begin
            $display("FAIL cont K=%0d: snapshot blk_done=%0d chk=%h, reference for %0d blocks = %h", K, blk, chk, blk, sum);
            bad = bad + 1;
          end
          nsnap = nsnap + 1;
        end
      end
      tile_en <= 1'b0;
      repeat (300) @(posedge clk_mem);
      snapshot(act, blk, chk);
      sum = 0;
      for (b = 0; b < blk; b = b + 1) sum = sum + relu_sum(b, K);
      if (chk != sum) begin $display("FAIL cont K=%0d: final chk %h, expected %h", K, chk, sum); bad = bad + 1; end
      if (blk != steps_seen / K) begin
        $display("FAIL cont K=%0d: blk_done %0d, steps %0d -> expected %0d", K, blk, steps_seen, steps_seen / K); bad = bad + 1;
      end
      if (act - act0 < steps_seen) begin
        $display("FAIL cont K=%0d: activity grew by %0d < %0d steps", K, act - act0, steps_seen); bad = bad + 1;
      end
      if (bad) errors = errors + 1;
      else $display("OK   continuous K=%0d: %0d steps (random enable), %0d blocks, %0d consistent snapshots while running, chk = %h",
                    K, steps_seen, blk, nsnap, chk);
      blk_model = blk; chk_model = chk;
    end
  endtask

  // ---------------------------------------------------------------- test
  integer i, a;
  reg [WDW-1:0] wa;
  reg [127:0] w128;
  reg [31:0] act, blk; reg [63:0] chk;

  initial begin
    fd = $fopen("tb_out/tile_poweron.txt", "w");
    $fdisplay(fd, "seed_a %0d seed_b %0d", 32'hA5A50000 ^ 32'h9E3779B9, 32'h5A5A1234);
    load_ref_from_dut;
    repeat (5) @(posedge clk_mem);
    rst_n_mem <= 1'b1;
    tile_rst_req <= 1'b0;
    repeat (100) @(posedge clk_mem);

    // ---- 1. power-on data -------------------------------------------------
    run_one_and_check("power-on block 0");
    dump_results(0);
    run_one_and_check("power-on block 1");
    dump_results(1);
    $fclose(fd);

    // ---- 2. host writes ------------------------------------------------------
    for (i = 0; i < 40; i = i + 1) begin
      for (a = 0; a < WDW / 32; a = a + 1) wa[a*32 +: 32] = $random;
      w128 = {$random, $random, $random, $random};
      if (i == 0) begin wa[15:0] = 16'h8000; w128[15:0] = 16'h8000; end     // (-32768)^2
      if (i == 1) begin wa[15:0] = 16'h7FFF; w128[15:0] = 16'h8000; end
      write_a(i, wa);
      write_b(i, w128);
    end
    tile_cfg(64);
    run_one_and_check("written data, K=64, block 0");
    run_one_and_check("written data, K=64, block 1");
    run_one_and_check("written data, K=64, block 2");

    // ---- 3. K clamp ----------------------------------------------------------
    tile_cfg(5);
    run_one_and_check("K=5 clamped to 40");

    // ---- 4. continuous ---------------------------------------------------------
    cont_run(256, 60000, 0);
    cont_run(64, 40000, 1);

    // ---- 4b. duty FSM path (en_gen with a running sequence) --------------------
    tile_cfg(48);
    duty_off <= 8'd20;  seq_run <= 1'b1;
    repeat (10) @(posedge clk_mem);
    win_toggle <= ~win_toggle;              // en_gen latches OFF = 20 (80%)
    tile_en <= 1'b1;
    steps_seen = 0;
    repeat (20000) @(posedge clk_mem);
    tile_en <= 1'b0; seq_run <= 1'b0;
    repeat (300) @(posedge clk_mem);
    snapshot(act, blk, chk);
    begin : duty_chk
      longint sum; integer b;
      sum = 0;
      for (b = 0; b < blk; b = b + 1) sum = sum + relu_sum(b, 48);
      if (chk != sum || blk != steps_seen / 48 || steps_seen < 30000 || steps_seen > 33000) begin
        $display("FAIL duty 80%%: steps %0d (expected ~32000 of 40000), blk %0d, chk %h vs %h", steps_seen, blk, chk, sum);
        errors = errors + 1;
      end else
        $display("OK   duty FSM at 80%%: %0d steps in 40000 tile cycles, %0d blocks, chk matches", steps_seen, blk);
    end

    // ---- 5. clear while running ---------------------------------------------------
    tile_cfg(256);
    tile_en <= 1'b1;
    repeat (3000) @(posedge clk_mem);
    tile_cfg(256);                          // clear in the middle of a block
    tile_en <= 1'b0;
    repeat (50) @(posedge clk_mem);
    snapshot(act, blk, chk);
    if (blk != 0 || chk != 0) begin $display("FAIL clear while running: blk %0d chk %h", blk, chk); errors = errors + 1; end
    run_one_and_check("after clear, block 0");

    if (errors == 0) $display("PASS: tb_sm");
    else             $display("FAIL: tb_sm, %0d error(s)", errors);
    $finish;
  end

  initial begin
    #20_000_000;
    $display("FAIL: tb_sm timed out");
    $finish;
  end

endmodule
`default_nettype wire
