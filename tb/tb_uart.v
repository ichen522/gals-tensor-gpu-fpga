`timescale 1ns/1ps
// tb_uart.v
//
// Loopback test: two independent uart.v instances, A's txd wired to B's
// rxd and vice versa. Sends a handful of bytes each way (including
// 0x00 and 0xFF corner cases) and checks the receiving side reproduces
// them exactly. Pure behavioral Verilog, no Xilinx primitives -- fully
// covered by iverilog.
// v2: also checks that a line held low (framing error) produces no bytes,
// and that reception recovers right after.

module tb_uart;

  localparam CLK_FREQ_HZ = 200_000_000;
  localparam BAUD_RATE   = 3_000_000;
  localparam CLK_PERIOD_NS = 5;   // 200MHz

  reg clk = 0;
  reg rst_n = 0;
  always #(CLK_PERIOD_NS/2.0) clk = ~clk;   // v2: was CLK_PERIOD_NS/2 = integer 2 -> a 4 ns (250 MHz) clock

  // ---- A ----
  reg        a_tx_start; reg [7:0] a_tx_data; wire a_tx_busy; wire a_txd;
  wire       a_rx_valid; wire [7:0] a_rx_data;
  // ---- B ----
  reg        b_tx_start; reg [7:0] b_tx_data; wire b_tx_busy; wire b_txd;
  wire       b_rx_valid; wire [7:0] b_rx_data;

  uart #(.CLK_FREQ_HZ(CLK_FREQ_HZ), .BAUD_RATE(BAUD_RATE)) uA (
    .clk(clk), .rst_n(rst_n),
    .tx_start(a_tx_start), .tx_data(a_tx_data), .tx_busy(a_tx_busy), .txd(a_txd),
    .rxd(b_txd), .rx_valid(a_rx_valid), .rx_data(a_rx_data)
  );

  uart #(.CLK_FREQ_HZ(CLK_FREQ_HZ), .BAUD_RATE(BAUD_RATE)) uB (
    .clk(clk), .rst_n(rst_n),
    .tx_start(b_tx_start), .tx_data(b_tx_data), .tx_busy(b_tx_busy), .txd(b_txd),
    .rxd(a_txd), .rx_valid(b_rx_valid), .rx_data(b_rx_data)
  );

  // ---- C: receiver only, rxd driven directly by the testbench (v2) ----
  reg  c_rxd = 1'b1;
  wire c_rx_valid; wire [7:0] c_rx_data;
  uart #(.CLK_FREQ_HZ(CLK_FREQ_HZ), .BAUD_RATE(BAUD_RATE)) uC (
    .clk(clk), .rst_n(rst_n),
    .tx_start(1'b0), .tx_data(8'h00), .tx_busy(), .txd(),
    .rxd(c_rxd), .rx_valid(c_rx_valid), .rx_data(c_rx_data)
  );
  integer c_count = 0;
  always @(posedge clk) if (c_rx_valid) c_count = c_count + 1;
  localparam integer BIT_NS = 1_000_000_000 / BAUD_RATE;
  task c_bitbang;   // send one well-formed byte on c_rxd
    input [7:0] d;
    integer b;
    begin
      c_rxd = 1'b0; #(BIT_NS);
      for (b = 0; b < 8; b = b + 1) begin c_rxd = d[b]; #(BIT_NS); end
      c_rxd = 1'b1; #(BIT_NS);
    end
  endtask

  integer errors = 0;

  task send_a;
    input [7:0] d;
    begin
      @(posedge clk);
      wait (!a_tx_busy);
      @(posedge clk);
      a_tx_data  = d;
      a_tx_start = 1'b1;
      @(posedge clk);
      a_tx_start = 1'b0;
    end
  endtask

  task send_b;
    input [7:0] d;
    begin
      @(posedge clk);
      wait (!b_tx_busy);
      @(posedge clk);
      b_tx_data  = d;
      b_tx_start = 1'b1;
      @(posedge clk);
      b_tx_start = 1'b0;
    end
  endtask

  reg [7:0] test_bytes [0:5];
  integer i;

  initial begin
    a_tx_start = 0; a_tx_data = 0;
    b_tx_start = 0; b_tx_data = 0;
    test_bytes[0] = 8'h00;
    test_bytes[1] = 8'hFF;
    test_bytes[2] = 8'hA5;
    test_bytes[3] = 8'h01;
    test_bytes[4] = 8'h5A;
    test_bytes[5] = 8'h81;

    rst_n = 0;
    repeat (5) @(posedge clk);
    rst_n = 1;
    repeat (5) @(posedge clk);

    // A -> B
    for (i = 0; i < 6; i = i + 1) begin
      send_a(test_bytes[i]);
      wait (b_rx_valid);
      if (b_rx_data !== test_bytes[i]) begin
        $display("FAIL A->B: sent %02h, got %02h", test_bytes[i], b_rx_data);
        errors = errors + 1;
      end else begin
        $display("OK   A->B: %02h", test_bytes[i]);
      end
      @(posedge clk);
    end

    // B -> A
    for (i = 0; i < 6; i = i + 1) begin
      send_b(test_bytes[i]);
      wait (a_rx_valid);
      if (a_rx_data !== test_bytes[i]) begin
        $display("FAIL B->A: sent %02h, got %02h", test_bytes[i], a_rx_data);
        errors = errors + 1;
      end else begin
        $display("OK   B->A: %02h", test_bytes[i]);
      end
      @(posedge clk);
    end

    // v2: framing errors are dropped. Line held low for 30 bit times
    // (unpowered bridge / cable glitch) must NOT produce any byte; a good
    // byte right after the line recovers must still be received.
    c_count = 0;
    c_rxd = 1'b0; #(30*BIT_NS);
    c_rxd = 1'b1; #(3*BIT_NS);
    if (c_count != 0) begin
      $display("FAIL break: %0d byte(s) accepted from a line held low", c_count);
      errors = errors + 1;
    end else $display("OK   line held low for 30 bit times -> no bytes accepted");
    c_bitbang(8'h02);
    #(2*BIT_NS);
    if (c_count != 1 || c_rx_data !== 8'h02) begin
      $display("FAIL after break: count=%0d data=%02h", c_count, c_rx_data);
      errors = errors + 1;
    end else $display("OK   next good byte (02) received after the break");

    if (errors == 0) $display("PASS: tb_uart, all bytes round-tripped correctly");
    else              $display("FAIL: tb_uart, %0d error(s)", errors);

    $finish;
  end

endmodule
