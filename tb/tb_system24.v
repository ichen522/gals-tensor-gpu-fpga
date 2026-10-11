`timescale 1ns/1ps
// tb_system24.v -- the default build: vc707_top with all 24 SMs
// (4 GPC x 3 TPC x 2 SM; 12 MMCMs + 12 PLLs, 24 DRP boots). Short run,
// checks what changes with the SM count and the GPU hierarchy:
//   * boot reprograms all 24 SM clocks (MMCMs and PLLs), LED 4
//   * INFO reports 24 SMs, 6 per GPC; GET_FREQ for SM 22 (MMCM) / 23 (PLL)
//   * TILE_CFG with a 3-byte mask is acknowledged by every SM
//   * DIP switch 2 = GPC 2: exactly SMs 12..17 run
//   * SET_EN FF FF FF + window engine: READ_WIN records are 4 + 3*24 = 76
//     bytes and every SM's N_k follows its own power-on sequence
//   * RUN_ONE + READ_RES on SM 23 (the last SM, a PLL SM) matches the reference
`default_nettype none
module tb_system24;

  localparam integer NT   = 24;
  localparam integer BAUD = 25_000_000;
  localparam integer WIN  = 2000;
  localparam integer RECB = 4 + 3 * NT;            // 76
  localparam integer ROWS = 2, NROW = 8, COLS = 8, NRES = 64;

  reg sys = 1'b0;
  always #2.5 sys = ~sys;
  reg cpu_reset = 1'b1;
  reg [7:0] dip = 8'h00;
  wire [7:0] led;
  wire dut_rxd, dut_txd;

  vc707_top #(.NUM_SM(NT), .HOST_BAUD(BAUD), .BOOT_DIV_ALL(12)) u_top (
    .sysclk_200_p (sys), .sysclk_200_n (~sys), .cpu_reset (cpu_reset),
    .gpio_dip_sw (dip), .gpio_led (led), .usb_uart_rxd (dut_rxd), .usb_uart_txd (dut_txd)
  );

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
  task expect_n (input integer n);
    integer t;
    begin
      t = 0;
      while (nrx < n && t < 600000) begin @(posedge sys); t = t + 1; end
      repeat (400) @(posedge sys);
      if (nrx != n) begin $display("FAIL: expected %0d reply bytes, got %0d", n, nrx); errors = errors + 1; end
    end
  endtask

  task read_act (input integer t, output [31:0] v);
    begin
      nrx = 0; send(8'h02); send(t); expect_n(4);
      v = {rx[0], rx[1], rx[2], rx[3]};
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

  reg signed [15:0] A23 [0:NROW-1][0:511];
  reg signed [15:0] B23 [0:511][0:COLS-1];

  integer i, r, t, k, bad, off, nk, expn, a, nrun;
  reg [31:0] act0 [0:NT-1];
  reg [31:0] v;
  longint acc, got;

  initial begin
    for (a = 0; a < 512; a = a + 1) begin
      for (r = 0; r < ROWS; r = r + 1) begin
        A23[0*ROWS+r][a] = u_top.SM[23].u_sm.u_core.SUBP[0].mem[a][r*16 +: 16];
        A23[1*ROWS+r][a] = u_top.SM[23].u_sm.u_core.SUBP[1].mem[a][r*16 +: 16];
        A23[2*ROWS+r][a] = u_top.SM[23].u_sm.u_core.SUBP[2].mem[a][r*16 +: 16];
        A23[3*ROWS+r][a] = u_top.SM[23].u_sm.u_core.SUBP[3].mem[a][r*16 +: 16];
      end
      for (r = 0; r < COLS; r = r + 1) B23[a][r] = u_top.SM[23].u_sm.u_core.bmem[a][r*16 +: 16];
    end

    repeat (20) @(posedge sys);
    cpu_reset = 1'b0;
    rst_n_h   = 1'b1;
    wait (u_top.boot_done);
    repeat (500) @(posedge sys);
    if (led[4] !== 1'b1) begin $display("FAIL LED 4 after boot"); errors = errors + 1; end
    else $display("OK   boot: 24 SM clocks (12 MMCMs + 12 PLLs) reprogrammed in %.1f us, LED 4 on", $realtime / 1000.0);

    nrx = 0; send(8'h0F); expect_n(12);
    if (rx[0] !== 8'd24 || rx[10] !== 8'd6 || rx[11] !== 8'd3) begin
      $display("FAIL INFO SMs = %0d, SM/GPC = %0d, N_k bytes = %0d", rx[0], rx[10], rx[11]); errors = errors + 1;
    end
    for (t = 22; t < 24; t = t + 1) begin
      nrx = 0; send(8'h10); send(t); expect_n(2);
      if (rx[0] !== 8'd12 || rx[1][1:0] !== 2'b11) begin $display("FAIL GET_FREQ SM %0d", t); errors = errors + 1; end
    end
    $display("OK   INFO = 24 SMs (6 per GPC, 3-byte N_k), SM 22 (MMCM) / SM 23 (PLL) at D 12, locked + running");

    nrx = 0; send(8'h0A); send(8'hFF); send(8'hFF); send(8'hFF); send(8'h00); send(8'd64); expect_n(1);
    if (rx[0] !== 8'hA0) begin $display("FAIL TILE_CFG FF FF FF: %02h", rx[0]); errors = errors + 1; end
    else $display("OK   TILE_CFG mask FF FF FF: all 24 SMs acknowledged");

    // ---- DIP 2 = GPC 2 (SMs 12..17) ----
    for (t = 0; t < NT; t = t + 1) begin read_act(t, v); act0[t] = v; end
    dip = 8'b0000_0100;
    #20000;
    dip = 8'h00;
    #2000;
    bad = 0; nrun = 0;
    for (t = 0; t < NT; t = t + 1) begin
      read_act(t, v);
      if ((v != act0[t]) != (t >= 12 && t < 18)) begin
        $display("FAIL DIP GPC 2: SM %0d activity %0d -> %0d", t, act0[t], v); bad = bad + 1;
      end
      if (v != act0[t]) nrun = nrun + 1;
    end
    if (bad) errors = errors + 1;
    else $display("OK   DIP switch 2: exactly the %0d SMs of GPC 2 (SM 12..17) ran", nrun);

    // ---- every SM on, window engine with the power-on sequences ----
    nrx = 0; send(8'h04); send(8'hFF); send(8'hFF); send(8'hFF); expect_n(1);
    nrx = 0; send(8'h07); send(8'h03); send(8'h00); send(8'h00);
    send(8'h00); send(8'h00); send(WIN >> 8); send(WIN & 8'hFF); expect_n(1);
    wait (u_top.rec_count >= 3);
    nrx = 0; send(8'h08); send(8'd2); expect_n(2 + 2 * RECB);
    bad = 0;
    for (r = 0; r < 2; r = r + 1)
      for (t = 0; t < NT; t = t + 1) begin
        nk   = {rx[6 + r*RECB + 3*t], rx[7 + r*RECB + 3*t], rx[8 + r*RECB + 3*t]};
        off  = def_off(t, r);
        expn = (WIN / 2) * (100 - off) / 100;     // 100 MHz SMs: 1000 cycles per window
        if (nk + 2 < expn || nk > expn + 2) begin
          $display("FAIL window %0d SM %0d: N = %0d expected %0d", r, t, nk, expn); bad = bad + 1;
        end
      end
    if (bad) errors = errors + 1;
    else $display("OK   READ_WIN: 76-byte records, all 24 SMs follow their own power-on duty sequence");
    nrx = 0; send(8'h07); send(8'h00); send(8'h00); send(8'h00);
    send(8'h00); send(8'h00); send(8'h00); send(8'h00); expect_n(1);
    nrx = 0; send(8'h04); send(8'h00); send(8'h00); send(8'h00); expect_n(1);
    #3000;

    // ---- the last SM (a PLL SM) ----
    nrx = 0; send(8'h0A); send(8'h80); send(8'h00); send(8'h00); send(8'h00); send(8'd64); expect_n(1);   // clear SM 23
    nrx = 0; send(8'h0D); send(8'd23); expect_n(1);
    nrx = 0; send(8'h0E); send(8'd23); expect_n(6 * NRES);
    bad = 0;
    for (r = 0; r < NRES; r = r + 1) begin
      acc = 0;
      for (k = 0; k < 64; k = k + 1) acc = acc + A23[r/COLS][k] * B23[k][r%COLS];
      got = 0;
      for (i = 0; i < 6; i = i + 1) got = (got << 8) | rx[r*6 + i];
      if (got[47]) got = got - (64'sd1 <<< 48);
      if (got != acc) bad = bad + 1;
    end
    if (bad) begin $display("FAIL READ_RES SM 23: %0d mismatches", bad); errors = errors + 1; end
    else $display("OK   RUN_ONE + READ_RES on SM 23 (GPC 3, PLL): 64 / 64 exact");

    if (errors == 0) $display("PASS: tb_system24");
    else             $display("FAIL: tb_system24, %0d error(s)", errors);
    $finish;
  end

  initial begin
    #4_000_000;
    $display("FAIL: tb_system24 timed out");
    $finish;
  end

endmodule
`default_nettype wire
