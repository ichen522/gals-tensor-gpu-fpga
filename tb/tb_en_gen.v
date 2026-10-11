// tb_en_gen.v
//
// Checks the per-tile enable FSM (en_gen.v) on its own, with the window
// control signals driven from a separate 200 MHz "clk_mem" exactly the
// way win_ctrl.v drives them (duty value set >= 64 cycles before the
// toggle, held >= 64 cycles after):
//   1. seq_run = 0: en simply follows tile_en (100% while on, 0 while off)
//   2. seq_run = 1: for each OFF value, the window's N_k (nk_hold) equals
//      an independent count of en=1 cycles between the module's own
//      window ticks, AND is within +-2 of cycles_in_window*(100-OFF)/100
//   3. the off-cycles are spread evenly: spacing between consecutive off
//      cycles is always floor(100/OFF) or ceil(100/OFF) (e.g. OFF=10 ->
//      exactly every 10th cycle, OFF=3 -> every 33rd or 34th)
//   4. OFF > 100 is clamped to 100 (tile fully off)
//   5. activity_count == total number of en=1 cycles
`timescale 1ns/1ps

module tb_en_gen;
  reg clk = 0, clk_mem = 0, rst_n = 0;
  always #1.0715 clk = ~clk;        // 466.67 MHz tile clock
  always #2.5    clk_mem = ~clk_mem; // 200 MHz

  reg       tile_en = 0, seq_run = 0, win_toggle = 0;
  reg [7:0] duty_off = 0;
  wire      en, tick;
  wire [31:0] activity_count, nk_hold;
  wire [7:0]  duty_cur;

  en_gen #(.DUTY_DEN(100), .DUTY_W(8)) dut (
    .clk(clk), .rst_n(rst_n),
    .tile_en_in(tile_en), .seq_run_in(seq_run),
    .win_toggle_in(win_toggle), .duty_off_in(duty_off),
    .en(en), .activity_count(activity_count), .nk_hold(nk_hold),
    .duty_cur(duty_cur), .tick(tick)
  );

  // ---- independent observers (tile clock) --------------------------------
  integer total_en = 0, win_en = 0, win_cycles = 0;
  integer last_win_en = 0, last_win_cycles = 0, windows_seen = 0;
  integer since_off = 0, spacing_min = 1000000, spacing_max = 0, off_seen = 0;
  always @(posedge clk) if (rst_n) begin
    total_en = total_en + en;
    if (tick) begin
      last_win_en     = win_en + en;
      last_win_cycles = win_cycles + 1;
      win_en = 0; win_cycles = 0;
      windows_seen = windows_seen + 1;
      since_off = 0; off_seen = 0; spacing_min = 1000000; spacing_max = 0;
    end else begin
      win_en = win_en + en;
      win_cycles = win_cycles + 1;
      // spacing between off cycles, ignoring the first few cycles of a window
      if (win_cycles > 8) begin
        since_off = since_off + 1;
        if (!en && tile_en) begin
          if (off_seen) begin
            if (since_off < spacing_min) spacing_min = since_off;
            if (since_off > spacing_max) spacing_max = since_off;
          end
          off_seen = 1; since_off = 0;
        end
      end
    end
  end

  integer errors = 0;
  integer k, i;

  // one window of WIN clk_mem cycles with the given OFF value
  localparam integer WIN = 4000;     // 20 us -> ~9,333 tile cycles
  task run_window;
    input [7:0] off;
    begin
      @(posedge clk_mem); duty_off = off;          // set >= 64 cycles before toggle
      repeat (64) @(posedge clk_mem);
      win_toggle = ~win_toggle;                    // this window starts
      repeat (WIN - 64) @(posedge clk_mem);
    end
  endtask

  task close_and_check;
    input [7:0] off;
    integer eff, expect_n, lo_sp, hi_sp;
    begin
      repeat (64) @(posedge clk_mem);
      win_toggle = ~win_toggle;                    // previous window ends
      repeat (20) @(posedge clk_mem);              // let tick + nk_hold settle
      eff = (off > 100) ? 100 : off;
      expect_n = (last_win_cycles * (100 - eff)) / 100;
      if (nk_hold !== last_win_en) begin
        $display("ERROR OFF=%0d: nk_hold=%0d but observed en count=%0d", off, nk_hold, last_win_en);
        errors = errors + 1;
      end
      if (nk_hold + 2 < expect_n || nk_hold > expect_n + 2) begin
        $display("ERROR OFF=%0d: N=%0d, expected ~%0d of %0d cycles", off, nk_hold, expect_n, last_win_cycles);
        errors = errors + 1;
      end
      if (duty_cur !== eff[7:0]) begin
        $display("ERROR OFF=%0d: duty_cur=%0d", off, duty_cur); errors = errors + 1;
      end
      $display("  OFF=%3d  N=%5d of %5d cycles  (%0d.%0d%% active, target %0d%%)",
               off, nk_hold, last_win_cycles, (nk_hold*100)/last_win_cycles,
               ((nk_hold*1000)/last_win_cycles) % 10, 100 - eff);
    end
  endtask

  integer offs [0:9];
  integer prev_act;

  initial begin
    offs[0]=0; offs[1]=1; offs[2]=3; offs[3]=10; offs[4]=20; offs[5]=7;
    offs[6]=50; offs[7]=97; offs[8]=100; offs[9]=150;

    rst_n = 0;
    repeat (5) @(posedge clk_mem);
    rst_n = 1;
    repeat (5) @(posedge clk_mem);

    // ---- 1. seq_run = 0: en follows tile_en ---------------------------
    tile_en = 1;
    repeat (10) @(posedge clk_mem);
    prev_act = activity_count;
    repeat (100) @(posedge clk);
    #0.05;
    if (activity_count - prev_act != 100) begin
      $display("ERROR: seq_run=0, tile_en=1: %0d en cycles in 100", activity_count - prev_act);
      errors = errors + 1;
    end else $display("OK   seq_run=0: en=1 every cycle while tile_en=1");
    tile_en = 0;
    repeat (10) @(posedge clk_mem);
    prev_act = activity_count;
    repeat (100) @(posedge clk);
    #0.05;
    if (activity_count != prev_act) begin
      $display("ERROR: tile_en=0 but en still high"); errors = errors + 1;
    end else $display("OK   seq_run=0: en=0 while tile_en=0");

    // ---- 2-4. duty windows ----------------------------------------------
    tile_en = 1; seq_run = 1;
    run_window(offs[0]);                // first window (no previous to close)
    for (k = 0; k < 10; k = k + 1) begin
      if (k > 0) run_window(offs[k]);
      // the window just started with offs[k]; close it and check
      close_and_check(offs[k]);
      // (close_and_check's toggle also started a new window with offs[k];
      //  run_window for the next value re-times it)
    end

    // spacing check for a few specific values, run separately
    for (k = 2; k <= 5; k = k + 1) begin
      run_window(offs[k]);
      repeat (64) @(posedge clk_mem);
      begin : SPC
        integer lo, hi;
        lo = 100 / offs[k];
        hi = (100 + offs[k] - 1) / offs[k];
        if (spacing_min < lo || spacing_max > hi) begin
          $display("ERROR OFF=%0d: off-cycle spacing %0d..%0d, expected %0d..%0d",
                   offs[k], spacing_min, spacing_max, lo, hi);
          errors = errors + 1;
        end else
          $display("OK   OFF=%0d: off cycles evenly spread (every %0d..%0d cycles)", offs[k], spacing_min, spacing_max);
      end
    end

    // ---- 5. activity_count total -----------------------------------------
    repeat (10) @(posedge clk);
    #0.05;
    if (activity_count !== total_en) begin
      $display("ERROR: activity_count=%0d, observed %0d", activity_count, total_en);
      errors = errors + 1;
    end else $display("OK   activity_count = total enabled cycles (%0d)", total_en);

    if (errors == 0) $display("TB_EN_GEN: PASS");
    else             $display("TB_EN_GEN: FAIL (%0d errors)", errors);
    $finish;
  end
endmodule
