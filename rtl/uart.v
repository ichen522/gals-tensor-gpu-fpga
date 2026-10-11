// uart.v
//
// Plain 8-N-1 UART transmitter + receiver, parameterized by the clock
// this instance runs on and the desired baud rate. No RTS/CTS flow
// control -- the VC707's USB-UART bridge exposes those two pins (see
// vc707_pins.xdc), but this design doesn't drive them, because host_if.v
// is a strict request/response protocol (host sends one command, waits
// for one reply, sends the next), never a continuous firehose. Even the
// biggest command this project defines -- one full 256-bit memory-word
// write -- is 36 bytes; at 3 Mbaud that is ~120us, utterly negligible
// next to how often a human (or even a host-side automation script)
// would actually issue commands. NOT verified against real hardware in
// this sandbox (no Xilinx sim libraries here, see tb/tb_uart.v for what
// WAS verified: the TX/RX bit-level protocol itself, in plain Verilog,
// with iverilog).
//
// This module intentionally has ZERO Xilinx-specific primitives -- it's
// plain synthesizable RTL, the same way async_fifo.v's Gray-code logic
// is. The only thing that needs care at the top level (vc707_top.v) is
// the usual 2-flop synchronizer on the async `rxd` input pin, which is
// done inside this module already (see `rxd_sync`).

module uart #(
  parameter CLK_FREQ_HZ = 200_000_000,
  parameter BAUD_RATE   = 3_000_000
)(
  input  wire       clk,
  input  wire       rst_n,

  // ---- TX: pulse tx_start for 1 cycle with tx_data stable; tx_busy
  //      stays high until the stop bit has gone out. Do not pulse
  //      tx_start again while tx_busy is high -- host_if.v respects this. ----
  input  wire       tx_start,
  input  wire [7:0] tx_data,
  output reg        tx_busy,
  output reg        txd,

  // ---- RX: rx_valid pulses for exactly 1 clk cycle the moment a full
  //      byte has been received (stop bit sampled), with rx_data holding
  //      that byte for the same cycle (and beyond, until the next byte
  //      completes -- host_if.v should register it on the rx_valid
  //      pulse rather than relying on it staying put indefinitely). ----
  input  wire       rxd,
  output reg        rx_valid,
  output reg [7:0]  rx_data
);

  localparam integer CLKS_PER_BIT      = CLK_FREQ_HZ / BAUD_RATE;
  localparam integer CLKS_PER_HALF_BIT = CLKS_PER_BIT / 2;
  localparam integer CNT_W             = $clog2(CLKS_PER_BIT + 1);

  // ============================================================ TX ====
  localparam TX_IDLE  = 3'd0,
             TX_START = 3'd1,
             TX_DATA  = 3'd2,
             TX_STOP  = 3'd3;

  reg [2:0]       tx_state;
  reg [CNT_W-1:0] tx_cnt;
  reg [2:0]       tx_bit_idx;
  reg [7:0]       tx_shift;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tx_state   <= TX_IDLE;
      tx_cnt     <= 0;
      tx_bit_idx <= 0;
      tx_shift   <= 8'h00;
      tx_busy    <= 1'b0;
      txd        <= 1'b1;   // idle line is high
    end else begin
      case (tx_state)
        TX_IDLE: begin
          txd <= 1'b1;
          if (tx_start) begin
            tx_shift <= tx_data;
            tx_busy  <= 1'b1;
            tx_cnt   <= 0;
            tx_state <= TX_START;
          end else begin
            tx_busy <= 1'b0;
          end
        end

        TX_START: begin
          txd <= 1'b0;   // start bit
          if (tx_cnt == CLKS_PER_BIT - 1) begin
            tx_cnt     <= 0;
            tx_bit_idx <= 0;
            tx_state   <= TX_DATA;
          end else begin
            tx_cnt <= tx_cnt + 1'b1;
          end
        end

        TX_DATA: begin
          txd <= tx_shift[tx_bit_idx];
          if (tx_cnt == CLKS_PER_BIT - 1) begin
            tx_cnt <= 0;
            if (tx_bit_idx == 3'd7) begin
              tx_state <= TX_STOP;
            end else begin
              tx_bit_idx <= tx_bit_idx + 1'b1;
            end
          end else begin
            tx_cnt <= tx_cnt + 1'b1;
          end
        end

        TX_STOP: begin
          txd <= 1'b1;   // stop bit
          if (tx_cnt == CLKS_PER_BIT - 1) begin
            tx_cnt   <= 0;
            tx_busy  <= 1'b0;
            tx_state <= TX_IDLE;
          end else begin
            tx_cnt <= tx_cnt + 1'b1;
          end
        end

        default: tx_state <= TX_IDLE;
      endcase
    end
  end

  // ============================================================ RX ====
  // 2-flop synchronizer on the async external rxd pin -- same idea as
  // every other async-input crossing in this project (vc707_top.v's
  // reset sync, async_fifo.v's Gray pointers), just for a single bit.
  (* ASYNC_REG = "TRUE" *) reg [1:0] rxd_sync;
  always @(posedge clk or negedge rst_n)
    if (!rst_n) rxd_sync <= 2'b11;
    else        rxd_sync <= {rxd_sync[0], rxd};
  wire rxd_s = rxd_sync[1];

  localparam RX_IDLE   = 3'd0,
             RX_START  = 3'd1,
             RX_DATA   = 3'd2,
             RX_STOP   = 3'd3,
             RX_BREAK  = 3'd4;   // v2: framing error -> wait for the line to go idle

  reg [2:0]       rx_state;
  reg [CNT_W-1:0] rx_cnt;
  reg [2:0]       rx_bit_idx;
  reg [7:0]       rx_shift;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rx_state   <= RX_IDLE;
      rx_cnt     <= 0;
      rx_bit_idx <= 0;
      rx_shift   <= 8'h00;
      rx_valid   <= 1'b0;
      rx_data    <= 8'h00;
    end else begin
      rx_valid <= 1'b0;   // default: 1-cycle pulse only

      case (rx_state)
        RX_IDLE: begin
          if (rxd_s == 1'b0) begin   // possible start bit (falling edge)
            rx_cnt   <= 0;
            rx_state <= RX_START;
          end
        end

        RX_START: begin
          // sample at the middle of the start bit to confirm it's real
          // (reject glitches shorter than half a bit period)
          if (rx_cnt == CLKS_PER_HALF_BIT - 1) begin
            if (rxd_s == 1'b0) begin
              rx_cnt     <= 0;
              rx_bit_idx <= 0;
              rx_state   <= RX_DATA;
            end else begin
              rx_state <= RX_IDLE;   // was a glitch, not a real start bit
            end
          end else begin
            rx_cnt <= rx_cnt + 1'b1;
          end
        end

        RX_DATA: begin
          // from here on, sample once per full bit period, landing in
          // the middle of each subsequent bit (started the count at the
          // start bit's middle, so +CLKS_PER_BIT lands at the middle of
          // bit N)
          if (rx_cnt == CLKS_PER_BIT - 1) begin
            rx_cnt              <= 0;
            rx_shift[rx_bit_idx] <= rxd_s;
            if (rx_bit_idx == 3'd7) begin
              rx_state <= RX_STOP;
            end else begin
              rx_bit_idx <= rx_bit_idx + 1'b1;
            end
          end else begin
            rx_cnt <= rx_cnt + 1'b1;
          end
        end

        // v2: the stop bit is now actually checked (sampled mid-bit, like
        // the data bits). A low stop bit = framing error: the byte is
        // DROPPED and the receiver waits for the line to return high
        // before looking for the next start bit. v1 accepted the byte
        // either way, so a line held low (USB-UART bridge unpowered or
        // still enumerating, a cable being plugged in) turned into a
        // stream of 0x00 bytes, and any glitch byte went straight into
        // host_if.v's command parser.
        RX_STOP: begin
          if (rx_cnt == CLKS_PER_BIT - 1) begin
            rx_cnt <= 0;
            if (rxd_s == 1'b1) begin
              rx_data  <= rx_shift;
              rx_valid <= 1'b1;
              rx_state <= RX_IDLE;
            end else begin
              rx_state <= RX_BREAK;
            end
          end else begin
            rx_cnt <= rx_cnt + 1'b1;
          end
        end

        RX_BREAK: begin
          if (rxd_s == 1'b1) rx_state <= RX_IDLE;
        end

        default: rx_state <= RX_IDLE;
      endcase
    end
  end

endmodule
