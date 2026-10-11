`timescale 1ns/1ps
// tb_clk.v -- clk_ctrl.v + 3 x sm_clk.v + BUFH / BUFHCE, against the
// MMCME2_ADV / PLLE2_ADV models in sim/sim_prims.v (which emit garbage
// before lock and check every DRP access). SM 0 and SM 2 use an MMCM
// (slot 0), SM 1 a PLL (slot 1) -- like one TPC + the next one.
//
//   1. boot: SM 0 -> D 6 (200 MHz), SM 1 stays at D 3 (400 MHz, the
//      compiled value, no DRP), SM 2 -> D 8 (150 MHz); periods measured
//   2. live changes: SM 1 (PLL) -> 4 (300), SM 0 -> 12 (100), SM 2 -> 5 (240);
//      during each change the OTHER SMs' clocks must not pause -- in
//      particular SM 0 (MMCM) keeps running while SM 1 (the PLL of the same
//      TPC) is reprogrammed and vice versa -- and the changed SM's clock
//      must never show a short pulse (the BUFHCE gate must hide the
//      pre-lock garbage)
//   3. invalid requests (SM 3, D 2, D 127) -> ack_ok = 0, nothing changes
//   4. no MMCM-ERROR / PLL-ERROR from the models (DRP protocol, reserved
//      bits, power reg)
`default_nettype none
module tb_clk;

  localparam integer NT = 3;

  reg clk = 1'b0;
  always #2.5 clk = ~clk;
  reg rst_n = 1'b0;

  reg        req_tog = 1'b0;
  reg  [7:0] req_tile = 8'd0, req_div = 8'd0;
  wire       ack_tog, ack_ok, busy, boot_done;
  wire [8*NT-1:0]  div_cur;
  wire [NT-1:0]    locked_s, ceon_s, den, drdy, mrst, gate;
  wire             dwe;
  wire [6:0]       daddr;
  wire [15:0]      di;
  wire [16*NT-1:0] do_flat;

  clk_ctrl #(.NUM_TILES(NT), .DIV_MIN(3), .BOOT_DIV({8'd8, 8'd3, 8'd6})) u_ctrl (
    .clk (clk), .rst_n (rst_n),
    .req_tog (req_tog), .req_tile (req_tile), .req_div (req_div),
    .ack_tog (ack_tog), .ack_ok (ack_ok), .busy (busy), .boot_done (boot_done), .div_cur (div_cur),
    .locked_s (locked_s), .ceon_s (ceon_s),
    .drp_den (den), .drp_dwe (dwe), .drp_daddr (daddr), .drp_di (di),
    .drp_do_flat (do_flat), .drp_drdy (drdy), .mmcm_rst (mrst), .gate_off (gate)
  );

  wire [NT-1:0] tclk;
  genvar g;
  generate
    for (g = 0; g < NT; g = g + 1) begin : T
      wire raw, free, ce;
      sm_clk #(.USE_PLL(g % 2), .DIV_MIN(3)) u_m (
        .clk_mem (clk),
        .drp_den (den[g]), .drp_dwe (dwe), .drp_daddr (daddr), .drp_di (di),
        .drp_do (do_flat[g*16 +: 16]), .drp_drdy (drdy[g]),
        .mmcm_rst (mrst[g]), .gate_off (gate[g]),
        .locked_s (locked_s[g]), .ceon_s (ceon_s[g]),
        .clk_raw (raw), .clk_free (free), .ce (ce)
      );
      BUFH   u_bf (.I(raw), .O(free));
      BUFHCE u_bg (.I(raw), .CE(ce), .O(tclk[g]));

      // pulse-width / gap monitor on the gated tile clock
      realtime t_rise = 0, t_fall = 0;
      real     min_pulse = 1.0e9;
      real     max_gap   = 0.0;
      integer  edges = 0;
      reg      watch_gap = 1'b0;
      always @(posedge tclk[g]) begin
        if (t_fall > 0 && $realtime - t_fall < min_pulse) min_pulse = $realtime - t_fall;
        if (watch_gap && t_rise > 0 && $realtime - t_rise > max_gap) max_gap = $realtime - t_rise;
        t_rise = $realtime;
        edges  = edges + 1;
      end
      always @(negedge tclk[g]) begin
        if (t_rise > 0 && $realtime - t_rise < min_pulse) min_pulse = $realtime - t_rise;
        t_fall = $realtime;
      end
    end
  endgenerate

  integer errors = 0;

  task measure (input integer t, output real per);
    integer e0;
    begin
      case (t)
        0: e0 = T[0].edges;  1: e0 = T[1].edges;  default: e0 = T[2].edges;
      endcase
      #2000;
      case (t)
        0: per = 2000.0 / (T[0].edges - e0);
        1: per = 2000.0 / (T[1].edges - e0);
        default: per = 2000.0 / (T[2].edges - e0);
      endcase
    end
  endtask

  task expect_period (input integer t, input integer d);
    real per, exp;
    begin
      measure(t, per);
      exp = 5.0 * d / 6.0;
      if (per < exp * 0.98 || per > exp * 1.02) begin
        $display("FAIL SM %0d: period %.3f ns, expected %.3f (D = %0d)", t, per, exp, d);
        errors = errors + 1;
      end else
        $display("OK   SM %0d (%s): period %.3f ns = %.1f MHz (D = %0d)", t, (t % 2) ? "PLL " : "MMCM", per, 1000.0 / per, d);
      if (div_cur[t*8 +: 8] != d) begin
        $display("FAIL SM %0d: div_cur = %0d, expected %0d", t, div_cur[t*8 +: 8], d);
        errors = errors + 1;
      end
    end
  endtask

  task set_freq (input integer t, input integer d, input integer want_ok);
    integer n;
    begin
      @(posedge clk); req_tile <= t; req_div <= d; req_tog <= ~req_tog;
      n = 0;
      @(posedge clk);
      while (ack_tog !== req_tog && n < 2_000_000) begin @(posedge clk); n = n + 1; end
      if (ack_tog !== req_tog || ack_ok !== want_ok[0]) begin
        $display("FAIL set_freq(SM %0d, D %0d): ack %b ok %b, expected ok %0d", t, d, ack_tog === req_tog, ack_ok, want_ok);
        errors = errors + 1;
      end else
        $display("OK   set_freq(SM %0d, D %0d) -> ok = %0d after %0d cycles", t, d, ack_ok, n);
    end
  endtask

  task watch (input reg on);
    begin
      T[0].watch_gap = on; T[1].watch_gap = on; T[2].watch_gap = on;
      if (on) begin T[0].max_gap = 0; T[1].max_gap = 0; T[2].max_gap = 0; end
    end
  endtask

  task check_gap (input integer t, input real limit);
    real g;
    begin
      case (t) 0: g = T[0].max_gap; 1: g = T[1].max_gap; default: g = T[2].max_gap; endcase
      if (g > limit) begin
        $display("FAIL SM %0d clock paused for %.2f ns while another SM was reprogrammed", t, g);
        errors = errors + 1;
      end
    end
  endtask

  integer nerr;
  initial begin
    repeat (10) @(posedge clk);
    rst_n <= 1'b1;
    wait (boot_done);
    $display("     boot done at %.0f ns", $realtime);
    repeat (200) @(posedge clk);
    expect_period(0, 6);
    expect_period(1, 3);
    expect_period(2, 8);

    watch(1'b1);
    set_freq(1, 4, 1);
    watch(1'b0);
    check_gap(0, 5.5);  check_gap(2, 7.0);
    repeat (100) @(posedge clk);
    expect_period(1, 4);

    watch(1'b1);
    set_freq(0, 12, 1);
    watch(1'b0);
    check_gap(1, 3.5);  check_gap(2, 7.0);
    set_freq(2, 5, 1);
    repeat (100) @(posedge clk);
    expect_period(0, 12);
    expect_period(2, 5);

    set_freq(3, 6, 0);       // no such SM
    set_freq(0, 2, 0);       // faster than DIV_MIN
    set_freq(0, 127, 0);     // > DIV_MAX
    expect_period(0, 12);

    if (T[0].min_pulse < 1.2 || T[1].min_pulse < 1.2 || T[2].min_pulse < 1.2) begin
      $display("FAIL short pulse on an SM clock: %.3f / %.3f / %.3f ns",
               T[0].min_pulse, T[1].min_pulse, T[2].min_pulse);
      errors = errors + 1;
    end else
      $display("OK   shortest SM-clock pulse %.3f / %.3f / %.3f ns (pre-lock garbage never reached an SM)",
               T[0].min_pulse, T[1].min_pulse, T[2].min_pulse);

    nerr = T[0].u_m.G_MMCM.u_mmcm.nerr + T[1].u_m.G_PLL.u_pll.nerr + T[2].u_m.G_MMCM.u_mmcm.nerr;
    if (nerr != 0) begin $display("FAIL %0d MMCM-/PLL-ERROR(s)", nerr); errors = errors + 1; end
    else           $display("OK   MMCM + PLL models: every DRP access legal, reserved bits kept, POWER written");

    if (errors == 0) $display("PASS: tb_clk");
    else             $display("FAIL: tb_clk, %0d error(s)", errors);
    $finish;
  end

  initial begin
    #2_000_000;
    $display("FAIL: tb_clk timed out");
    $finish;
  end

endmodule
`default_nettype wire
