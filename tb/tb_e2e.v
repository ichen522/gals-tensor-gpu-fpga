`timescale 1ns/1ps
// tb_e2e.v -- plays a command script written by host/e2e_rtl.py (the
// SAME byte encoders host/gals_host.py uses on the board) into vc707_top's
// UART pin and writes every reply back for the Python side to decode and
// check against host/tile_model.py.
//
//   tb_out/e2e_cmds.txt     T <ntx> <byte> .. <byte> <nrx>     (hex bytes)
//                           W <ns>                              (wait)
//   tb_out/e2e_replies.txt  R <n> <byte> .. <byte>
`default_nettype none
module tb_e2e;

  localparam integer NT   = 3;
  localparam integer BAUD = 25_000_000;

  reg sys = 1'b0;
  always #2.5 sys = ~sys;
  reg cpu_reset = 1'b1;
  wire dut_rxd, dut_txd;

  vc707_top #(.NUM_SM(NT), .HOST_BAUD(BAUD), .BOOT_DIV_ALL(12)) u_top (
    .sysclk_200_p (sys), .sysclk_200_n (~sys), .cpu_reset (cpu_reset),
    .gpio_dip_sw (8'h00), .gpio_led (), .usb_uart_rxd (dut_rxd), .usb_uart_txd (dut_txd)
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
  reg [7:0] rx [0:2047];
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

  integer fi, fo, rc, ntx, nr, i, t, ns, ntr;
  reg [7:0]  b;
  reg [8*4-1:0] tag;

  initial begin
    fi = $fopen("tb_out/e2e_cmds.txt", "r");
    fo = $fopen("tb_out/e2e_replies.txt", "w");
    if (fi == 0) begin $display("FAIL: no tb_out/e2e_cmds.txt"); $finish; end
    repeat (20) @(posedge sys);
    cpu_reset = 1'b0;
    rst_n_h   = 1'b1;
    wait (u_top.boot_done);
    repeat (500) @(posedge sys);
    ntr = 0;
    while (!$feof(fi)) begin
      rc = $fscanf(fi, "%s", tag);
      if (rc != 1) begin
        rc = $fgetc(fi);                 // skip
      end else if (tag == "W") begin
        rc = $fscanf(fi, "%d", ns);
        #(ns);
      end else if (tag == "T") begin
        rc = $fscanf(fi, "%d", ntx);
        nrx = 0;
        for (i = 0; i < ntx; i = i + 1) begin
          rc = $fscanf(fi, "%h", b);
          send(b);
        end
        rc = $fscanf(fi, "%d", nr);
        t = 0;
        while (nrx < nr && t < 4_000_000) begin @(posedge sys); t = t + 1; end
        repeat (300) @(posedge sys);
        $fwrite(fo, "R %0d", nrx);
        for (i = 0; i < nrx; i = i + 1) $fwrite(fo, " %02h", rx[i]);
        $fwrite(fo, "\n");
        ntr = ntr + 1;
      end
    end
    $fclose(fo);
    $display("tb_e2e: %0d transactions played", ntr);
    $finish;
  end

  initial begin
    #20_000_000;
    $display("FAIL: tb_e2e timed out");
    $finish;
  end

endmodule
`default_nettype wire
