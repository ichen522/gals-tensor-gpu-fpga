`timescale 1ns/1ps
// tb_system.v -- the whole vc707_top over its UART pins: 3 SMs (SM 0 and
// SM 2 on an MMCM, SM 1 on a PLL -- sim/sim_prims.v models), fast baud for
// simulation. 3 SMs -> 1 mask byte.
//
//   1. boot (every SM clock reprogrammed to BOOT_DIV), PING, INFO, GET_FREQ
//   2. WRITE_A to SM 1, WRITE_B broadcast -> BRAM contents
//   3. TILE_CFG (K = 64) -> RUN_ONE SM 1 -> READ_RES: 64 x 48-bit exact,
//      READ_STAT: blk_done = 1, chk = ReLU sum
//   4. SET_EN SM 2, continuous run, SET_EN 0 -> READ_STAT chk = reference
//   5. SET_FREQ SM 0 (MMCM) -> D 24, SM 1 (PLL) -> D 6 (GALS: every SM its
//      own frequency); measured SM clocks; out-of-range requests -> E1
//   6. window engine over UART with all 3 SMs running on their POWER-ON
//      sequences: every 3-byte N_k matches that SM's frequency x its duty
//      (the hash in win_ctrl.v, recomputed here); then every SM's checksum
//      matches the reference for its blk_done
//   7. invalid SM index -> E1, unknown opcode ignored, no MMCM/PLL-ERROR
`default_nettype none
module tb_system;

  localparam integer NT     = 3;
  localparam integer BAUD   = 25_000_000;      // 8 clk_mem cycles per bit
  localparam integer KT     = 64;
  localparam integer WIN    = 2000;            // clk_mem cycles = 10 us

  reg sys = 1'b0;
  always #2.5 sys = ~sys;
  reg cpu_reset = 1'b1;
  reg [7:0] dip = 8'h00;
  wire [7:0] led;
  wire dut_rxd, dut_txd;

  localparam integer ROWS = 2, NROW = 8, COLS = 8, NRES = 64;
  localparam integer RECB = 4 + 3 * NT;              // READ_WIN record bytes

  vc707_top #(.NUM_SM(NT), .HOST_BAUD(BAUD), .BOOT_DIV_ALL(12), .K_DEFAULT(256)) u_top (
    .sysclk_200_p (sys), .sysclk_200_n (~sys), .cpu_reset (cpu_reset),
    .gpio_dip_sw (dip), .gpio_led (led), .usb_uart_rxd (dut_rxd), .usb_uart_txd (dut_txd)
  );

  // ---- host side UART -------------------------------------------------------
  reg  rst_n_h = 1'b0;
  reg        h_start = 1'b0;
  reg  [7:0] h_data = 8'd0;
  wire       h_busy, h_rv;
  wire [7:0] h_rd;
  uart #(.CLK_FREQ_HZ(200_000_000), .BAUD_RATE(BAUD)) u_h (
    .clk (sys), .rst_n (rst_n_h),
    .tx_start (h_start), .tx_data (h_data), .tx_busy (h_busy), .txd (dut_rxd),
    .rxd (dut_txd), .rx_valid (h_rv), .rx_data (h_rd)
  );

  reg [7:0] rx [0:1023];
  integer   nrx = 0;
  always @(posedge sys) if (h_rv) begin rx[nrx] = h_rd; nrx = nrx + 1; end

  task send (input [7:0] b);
    begin
      wait (!h_busy);
      @(posedge sys); h_data <= b; h_start <= 1'b1;
      @(posedge sys); h_start <= 1'b0;
      @(posedge sys);
    end
  endtask

  integer errors = 0;
  task expect_n (input integer n, input integer tmo_us);
    integer t;
    begin
      t = 0;
      while (nrx < n && t < tmo_us * 200) begin @(posedge sys); t = t + 1; end
      if (nrx < n) begin $display("FAIL: expected %0d reply bytes, got %0d", n, nrx); errors = errors + 1; end
      repeat (400) @(posedge sys);              // nothing extra may arrive
      if (nrx != n) begin $display("FAIL: expected %0d reply bytes, got %0d", n, nrx); errors = errors + 1; end
    end
  endtask

  task cmd_ack (input [8*48-1:0] label, input [7:0] want);
    begin
      expect_n(1, 3000);
      if (rx[0] !== want) begin
        $display("FAIL %0s: reply %02h, expected %02h", label, rx[0], want); errors = errors + 1;
      end else $display("OK   %0s -> %02h", label, rx[0]);
    end
  endtask

  // ---- reference (from the DUT's BRAM contents) -------------------------------
  reg signed [15:0] Aref [0:NT-1][0:NROW-1][0:511];
  reg signed [15:0] Bref [0:NT-1][0:511][0:COLS-1];
  longint Cref [0:NRES-1];

  task load_ref;
    integer a, r;
    begin
      for (a = 0; a < 512; a = a + 1) begin
        for (r = 0; r < ROWS; r = r + 1) begin
          Aref[0][0*ROWS+r][a] = u_top.SM[0].u_sm.u_core.SUBP[0].mem[a][r*16 +: 16];
          Aref[0][1*ROWS+r][a] = u_top.SM[0].u_sm.u_core.SUBP[1].mem[a][r*16 +: 16];
          Aref[0][2*ROWS+r][a] = u_top.SM[0].u_sm.u_core.SUBP[2].mem[a][r*16 +: 16];
          Aref[0][3*ROWS+r][a] = u_top.SM[0].u_sm.u_core.SUBP[3].mem[a][r*16 +: 16];
          Aref[1][0*ROWS+r][a] = u_top.SM[1].u_sm.u_core.SUBP[0].mem[a][r*16 +: 16];
          Aref[1][1*ROWS+r][a] = u_top.SM[1].u_sm.u_core.SUBP[1].mem[a][r*16 +: 16];
          Aref[1][2*ROWS+r][a] = u_top.SM[1].u_sm.u_core.SUBP[2].mem[a][r*16 +: 16];
          Aref[1][3*ROWS+r][a] = u_top.SM[1].u_sm.u_core.SUBP[3].mem[a][r*16 +: 16];
          Aref[2][0*ROWS+r][a] = u_top.SM[2].u_sm.u_core.SUBP[0].mem[a][r*16 +: 16];
          Aref[2][1*ROWS+r][a] = u_top.SM[2].u_sm.u_core.SUBP[1].mem[a][r*16 +: 16];
          Aref[2][2*ROWS+r][a] = u_top.SM[2].u_sm.u_core.SUBP[2].mem[a][r*16 +: 16];
          Aref[2][3*ROWS+r][a] = u_top.SM[2].u_sm.u_core.SUBP[3].mem[a][r*16 +: 16];
        end
        for (r = 0; r < COLS; r = r + 1) begin
          Bref[0][a][r] = u_top.SM[0].u_sm.u_core.bmem[a][r*16 +: 16];
          Bref[1][a][r] = u_top.SM[1].u_sm.u_core.bmem[a][r*16 +: 16];
          Bref[2][a][r] = u_top.SM[2].u_sm.u_core.bmem[a][r*16 +: 16];
        end
      end
    end
  endtask

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

  function automatic void ref_block (input integer t, input integer b, input integer K);
    integer i, j, k, a;
    longint acc;
    begin
      for (i = 0; i < NROW; i = i + 1)
        for (j = 0; j < COLS; j = j + 1) begin
          acc = 0;
          for (k = 0; k < K; k = k + 1) begin
            a = (b * K + k) % 512;
            acc = acc + Aref[t][i][a] * Bref[t][a][j];
          end
          Cref[i*COLS + j] = acc;
        end
    end
  endfunction

  function automatic longint relu_total (input integer t, input integer nb, input integer K);
    integer b, i, period;
    longint s, per;
    begin
      period = 512 / ((K % 512 == 0) ? 512 : (K & -K));   // K = 64 -> 8 blocks
      per = 0;
      for (b = 0; b < period; b = b + 1) begin
        ref_block(t, b, K);
        for (i = 0; i < NRES; i = i + 1) if (Cref[i] > 0) per = per + Cref[i];
      end
      s = (nb / period) * per;
      for (b = 0; b < nb % period; b = b + 1) begin
        ref_block(t, b, K);
        for (i = 0; i < NRES; i = i + 1) if (Cref[i] > 0) s = s + Cref[i];
      end
      relu_total = s;
    end
  endfunction

  // ---- helpers -----------------------------------------------------------------
  task read_stat (input integer t, output [31:0] act, output [31:0] blk, output [63:0] chk);
    integer i;
    begin
      nrx = 0; send(8'h0C); send(t);
      expect_n(16, 3000);
      act = {rx[0], rx[1], rx[2], rx[3]};
      blk = {rx[4], rx[5], rx[6], rx[7]};
      chk = 0;
      for (i = 0; i < 8; i = i + 1) chk = {chk[55:0], rx[8+i]};
    end
  endtask

  task measure (input integer t, output real per);
    integer e0, e1;
    begin
      case (t) 0: e0 = ed0; 1: e0 = ed1; default: e0 = ed2; endcase
      #4000;
      case (t) 0: e1 = ed0; 1: e1 = ed1; default: e1 = ed2; endcase
      per = 4000.0 / (e1 - e0);
    end
  endtask
  integer ed0 = 0, ed1 = 0, ed2 = 0;
  always @(posedge u_top.SM[0].tclk) ed0 = ed0 + 1;
  always @(posedge u_top.SM[1].tclk) ed1 = ed1 + 1;
  always @(posedge u_top.SM[2].tclk) ed2 = ed2 + 1;

  // ---- the test -------------------------------------------------------------------
  integer i, t, r, bad;
  reg [127:0] wa;
  reg [127:0] wb;
  reg [31:0]  act, blk;
  reg [63:0]  chk;
  longint     got, exp_chk;
  real        per;
  integer     divs [0:NT-1];

  initial begin
    repeat (20) @(posedge sys);
    cpu_reset = 1'b0;
    rst_n_h   = 1'b1;
    load_ref;
    wait (u_top.boot_done);
    $display("     boot done at %.0f ns", $realtime);
    repeat (500) @(posedge sys);

    // ---- 1. link, info, clocks ----------------------------------------------
    nrx = 0; send(8'h09); expect_n(5, 100);
    if ({rx[0], rx[1], rx[2], rx[3], rx[4]} !== 40'h47414C5390) begin
      $display("FAIL PING: %02h %02h %02h %02h %02h", rx[0], rx[1], rx[2], rx[3], rx[4]); errors = errors + 1;
    end else $display("OK   PING -> GALS 0x90");

    nrx = 0; send(8'h0F); expect_n(12, 100);
    if ({rx[0], rx[1], rx[2], rx[3], rx[4], rx[5], rx[6], rx[7], rx[8], rx[9], rx[10], rx[11]} !==
        {8'd3, 8'd4, 8'd2, 8'd8, 8'd9, 16'd1200, 8'd3, 8'd126, 8'd40, 8'd6, 8'd3}) begin
      $display("FAIL INFO"); errors = errors + 1;
    end else $display("OK   INFO -> 3 SMs, 4 sub-partitions x 2 x 8 MACs, depth 2^9, VCO 1200 MHz, D 3..126, K_MIN 40, 6 SM/GPC, 3-byte N_k");

    for (t = 0; t < NT; t = t + 1) begin
      nrx = 0; send(8'h10); send(t); expect_n(2, 100);
      if (rx[0] !== 8'd12 || rx[1][3:0] !== 4'b1011) begin
        $display("FAIL GET_FREQ SM %0d: D %0d status %b", t, rx[0], rx[1]); errors = errors + 1;
      end
    end
    measure(0, per);
    if (per < 9.8 || per > 10.2) begin $display("FAIL SM 0 boot clock %.3f ns", per); errors = errors + 1; end
    else begin
      measure(1, per);
      if (per < 9.8 || per > 10.2) begin $display("FAIL SM 1 (PLL) boot clock %.3f ns", per); errors = errors + 1; end
      else $display("OK   boot: every SM reprogrammed to D = 12 (100 MHz), SM 0 (MMCM) and SM 1 (PLL) measured");
    end

    // ---- 2. writes ---------------------------------------------------------------
    for (i = 0; i < 4; i = i + 1) wa[i*32 +: 32] = $random;
    wb = {$random, $random, $random, $random};
    nrx = 0; send(8'h01); send(8'd1); send(8'h00); send(8'h05);
    for (i = 0; i < 16; i = i + 1) send(wa[(15-i)*8 +: 8]);
    cmd_ack("WRITE_A SM 1 addr 5", 8'hA0);
    nrx = 0; send(8'h0B); send(8'hFF); send(8'h00); send(8'h05);
    for (i = 0; i < 16; i = i + 1) send(wb[(15-i)*8 +: 8]);
    cmd_ack("WRITE_B all SMs addr 5", 8'hA0);
    repeat (20) @(posedge sys);
    if (u_top.SM[1].u_sm.u_core.SUBP[0].mem[5] !== wa[31:0]   || u_top.SM[1].u_sm.u_core.SUBP[3].mem[5] !== wa[127:96] ||
        u_top.SM[0].u_sm.u_core.bmem[5]        !== wb         || u_top.SM[2].u_sm.u_core.bmem[5]        !== wb ||
        u_top.SM[0].u_sm.u_core.SUBP[0].mem[5] === wa[31:0]) begin
      $display("FAIL writes did not land in the right BRAMs"); errors = errors + 1;
    end else $display("OK   BRAM contents: A column in SM 1 only (4 sub-partitions), B row in every SM");
    load_ref;

    // ---- 3. one block ---------------------------------------------------------------
    nrx = 0; send(8'h0A); send(8'h07); send(8'h00); send(KT);
    cmd_ack("TILE_CFG SMs 0..2, K = 64", 8'hA0);
    nrx = 0; send(8'h0D); send(8'd1);
    cmd_ack("RUN_ONE SM 1", 8'hA0);
    nrx = 0; send(8'h0E); send(8'd1);
    expect_n(6 * NRES, 3000);
    ref_block(1, 0, KT);
    bad = 0;
    for (r = 0; r < NRES; r = r + 1) begin
      got = 0;
      for (i = 0; i < 6; i = i + 1) got = (got << 8) | rx[r*6 + i];
      if (got[47]) got = got - (64'sd1 <<< 48);
      if (got != Cref[r]) begin
        if (bad < 4) $display("FAIL READ_RES C[%0d][%0d] = %0d, expected %0d", r/COLS, r%COLS, got, Cref[r]);
        bad = bad + 1;
      end
    end
    if (bad) errors = errors + 1;
    else $display("OK   READ_RES SM 1: 64 / 64 values of C = A x B exact (block 0, K = 64, includes the written column)");
    read_stat(1, act, blk, chk);
    exp_chk = relu_total(1, 1, KT);
    if (blk !== 1 || chk !== exp_chk) begin
      $display("FAIL READ_STAT SM 1: blk %0d chk %h, expected 1 / %h", blk, chk, exp_chk); errors = errors + 1;
    end else $display("OK   READ_STAT SM 1: blk_done 1, chk %h", chk);

    // ---- 4. continuous on SM 2 -----------------------------------------------------------
    nrx = 0; send(8'h04); send(8'h04);
    cmd_ack("SET_EN SM 2", 8'hA0);
    #60000;
    nrx = 0; send(8'h04); send(8'h00);
    cmd_ack("SET_EN off", 8'hA0);
    #2000;
    read_stat(2, act, blk, chk);
    exp_chk = relu_total(2, blk, KT);
    if (blk < 50 || chk !== exp_chk || act < blk * KT) begin
      $display("FAIL continuous SM 2: act %0d blk %0d chk %h, expected chk %h", act, blk, chk, exp_chk); errors = errors + 1;
    end else $display("OK   continuous SM 2: %0d active cycles, %0d blocks, chk = reference", act, blk);

    // ---- 5. GALS: different frequencies ------------------------------------------------------
    nrx = 0; send(8'h05); send(8'd0); send(8'd24);
    cmd_ack("SET_FREQ SM 0 (MMCM) D 24 (50 MHz)", 8'hA0);
    nrx = 0; send(8'h05); send(8'd1); send(8'd6);
    cmd_ack("SET_FREQ SM 1 (PLL) D 6 (200 MHz)", 8'hA0);
    nrx = 0; send(8'h05); send(8'd2); send(8'd2);
    cmd_ack("SET_FREQ SM 2 D 2 (too fast)", 8'hE1);
    nrx = 0; send(8'h05); send(8'd7); send(8'd6);
    cmd_ack("SET_FREQ SM 7 (no such SM)", 8'hE1);
    divs[0] = 24; divs[1] = 6; divs[2] = 12;
    for (t = 0; t < NT; t = t + 1) begin
      nrx = 0; send(8'h10); send(t); expect_n(2, 100);
      measure(t, per);
      if (rx[0] !== divs[t] || per < 5.0 * divs[t] / 6.0 * 0.97 || per > 5.0 * divs[t] / 6.0 * 1.03) begin
        $display("FAIL SM %0d: GET_FREQ D %0d, period %.3f ns, expected D %0d", t, rx[0], per, divs[t]); errors = errors + 1;
      end else $display("OK   SM %0d (%s) runs at %.1f MHz (D = %0d)", t, (t % 2) ? "PLL " : "MMCM", 1000.0 / per, rx[0]);
    end

    // ---- 6. window engine, all SMs running ---------------------------------------------------
    nrx = 0; send(8'h0A); send(8'h07); send(8'h00); send(KT);
    cmd_ack("TILE_CFG (clear) before the window run", 8'hA0);
    nrx = 0; send(8'h04); send(8'h07);
    cmd_ack("SET_EN all", 8'hA0);
    nrx = 0; send(8'h07); send(8'h03); send(8'h00); send(8'h00);
    send(8'h00); send(8'h00); send(WIN >> 8); send(WIN & 8'hFF);
    cmd_ack("SEQ_CFG run + clear, 10 us windows", 8'hA0);
    wait (u_top.rec_count >= 5);
    nrx = 0; send(8'h08); send(8'd4);
    expect_n(2 + 4 * RECB, 3000);
    bad = 0;
    if (rx[0] !== 8'h01 || rx[1] !== 8'd4) begin $display("FAIL READ_WIN header %02h %02h", rx[0], rx[1]); bad = bad + 1; end
    for (r = 0; r < 4; r = r + 1) begin : recs
      integer idx, nk, expn, off, cyc;
      idx = {rx[2 + r*RECB], rx[3 + r*RECB], rx[4 + r*RECB], rx[5 + r*RECB]};
      if (idx != r) begin $display("FAIL READ_WIN record %0d index %0d", r, idx); bad = bad + 1; end
      for (t = 0; t < NT; t = t + 1) begin
        nk   = {rx[6 + r*RECB + 3*t], rx[7 + r*RECB + 3*t], rx[8 + r*RECB + 3*t]};
        off  = def_off(t, r);
        cyc  = WIN * 6 / divs[t];                      // SM cycles per window
        expn = cyc * (100 - off) / 100;
        if (nk + 2 < expn || nk > expn + 2) begin
          $display("FAIL window %0d SM %0d: N = %0d, expected %0d (%0d cycles x %0d%%)", r, t, nk, expn, cyc, 100 - off);
          bad = bad + 1;
        end
      end
    end
    if (bad) errors = errors + 1;
    else $display("OK   READ_WIN: 4 windows x 3 SMs at 50 / 200 / 100 MHz, every 3-byte N_k = f x power-on duty (+-2): OFF %0d,%0d,%0d / %0d,%0d,%0d ...",
                  def_off(0,0), def_off(1,0), def_off(2,0), def_off(0,1), def_off(1,1), def_off(2,1));
    nrx = 0; send(8'h07); send(8'h00); send(8'h00); send(8'h00);
    send(8'h00); send(8'h00); send(8'h00); send(8'h00);
    cmd_ack("SEQ_CFG stop", 8'hA0);
    nrx = 0; send(8'h04); send(8'h00);
    cmd_ack("SET_EN off", 8'hA0);
    #3000;
    for (t = 0; t < NT; t = t + 1) begin
      read_stat(t, act, blk, chk);
      exp_chk = relu_total(t, blk, KT);
      if (blk < 5 || chk !== exp_chk) begin
        $display("FAIL SM %0d after the window run: blk %0d chk %h, expected %h", t, blk, chk, exp_chk); errors = errors + 1;
      end else $display("OK   SM %0d after the window run: %0d active cycles, %0d blocks, chk = reference", t, act, blk);
    end

    // ---- 7. errors -------------------------------------------------------------------------------------
    nrx = 0; send(8'h02); send(8'd5);
    cmd_ack("READ_ACT SM 5 (no such SM)", 8'hE1);
    nrx = 0; send(8'h55); send(8'h09); expect_n(5, 100);
    if (rx[0] !== 8'h47) begin $display("FAIL unknown opcode not ignored"); errors = errors + 1; end
    else $display("OK   unknown opcode 0x55 ignored, PING still answers");

    i = u_top.SM[0].u_clk.G_MMCM.u_mmcm.nerr + u_top.SM[1].u_clk.G_PLL.u_pll.nerr + u_top.SM[2].u_clk.G_MMCM.u_mmcm.nerr;
    if (i != 0) begin $display("FAIL %0d MMCM-/PLL-ERROR(s)", i); errors = errors + 1; end
    if (led[4] !== 1'b1) begin $display("FAIL LED 4 (boot + locked) off"); errors = errors + 1; end

    if (errors == 0) $display("PASS: tb_system");
    else             $display("FAIL: tb_system, %0d error(s)", errors);
    $finish;
  end

  initial begin
    #5_000_000;
    $display("FAIL: tb_system timed out");
    $finish;
  end

endmodule
`default_nettype wire
