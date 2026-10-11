// vc707_top.v  (10_6, build 0x90)
//
// NVIDIA-style GPU on the VC707 (XC7VX485T-2): the same hierarchy a
// Hopper / Blackwell GPU has, scaled down --
//
//   GPU   = 4 GPCs                       one per corner of the die
//   GPC   = 3 TPCs                       3 clock regions stacked from the corner
//   TPC   = 2 SMs                        one clock region (= one CMT: 1 MMCM + 1 PLL)
//   SM    = 4 sub-partitions             half a clock region, its own clock
//   sub-partition = 1 tensor core        2 x 8 = 16 DSP48E1 MACs
//   => 24 SMs, 96 tensor cores, 1,536 MACs (16-bit in, 48-bit accumulate)
//
// SM numbering:  sm = 6*gpc + 2*tpc + slot   (slot 0: MMCM, slot 1: PLL)
//   GPC 0 = bottom-left  (X0 Y0..Y2)   GPC 1 = bottom-right (X1 Y0..Y2)
//   GPC 2 = top-left     (X0 Y6..Y4)   GPC 3 = top-right    (X1 Y6..Y4)
//   TPC 0 is the corner region of its GPC; slot 0 = lower half, 1 = upper
//   (constraints/pblocks.tcl). The middle row (Y3) holds the front end:
//   host_if (the "host interface"), win_ctrl, clk_ctrl.
//
// What is industry and what is ours (10/3 meeting: "keep the industry
// architecture, only add the power mechanism"):
//   industry : GPC/TPC/SM hierarchy, 4 sub-partitions per SM with one
//              tensor core each, output-stationary MMA over K, shared B
//              (weights) / per-SM A, epilogue (ReLU) after the MMA
//   ours     : every SM has its OWN clock (own MMCM or PLL, own regional
//              clock tree), changeable live per SM (SET_FREQ), per-SM clock
//              gating, per-SM activity counters and the 1 ms window log.
//              (Public NVIDIA GPUs run all SMs on one SM clock domain.)
//
//   board 200 MHz --IBUFDS--BUFG--> clk_mem (host_if, win_ctrl, clk_ctrl,
//                                   BRAM write ports, every MMCM / PLL input)
//   per SM i:
//     sm_clk.v     its own MMCM (slot 0) or PLL (slot 1), 1200 MHz VCO / D,
//                  DRP-programmable
//     BUFH         clk_free  (only for the clock-enable shift register)
//     BUFHCE       SM clock (glitch-free stop / start around a DRP change)
//                  Both are REGIONAL buffers: an SM never leaves its clock
//                  region, so its clock needs no global BUFG.
//     sm_top.v     en_gen + sm_core (4 tensor cores, BRAMs, readout,
//                  checksum) + handshakes
//   host_if.v      UART commands (921,600 baud)
//   win_ctrl.v     1 ms windows, per-SM duty sequences, N_k log
//   clk_ctrl.v     boots every SM to BOOT_DIV, then live per-SM frequency
//                  changes (SET_FREQ)
//
// The "spine" registers below are deliberately NOT in any Pblock, so the
// placer can park them between the front end and an SM: no clk_mem path
// has to cross the die in one cycle.
//
// Board I/O (pins as v7.3 / 110_3, constraints/vc707_pins.xdc):
//   DIP 0..3  GPC 0..3 runs (its 6 SMs)     DIP 5  every SM runs
//   DIP 4     start the window engine with its power-on sequences
//   (DIP switches are ORed with the host's SET_EN mask)
//   LED 0..3  GPC 0..3 alive (SM 6*g, blk_done bit 16: blinks while computing)
//   LED 4     boot finished and every SM clock locked
//   LED 5     clk_mem heartbeat, exactly 1 blink per second
//   LED 6     window engine running   LED 7  blinks every 512 windows
//
// POWER: the VC707 cuts VCCINT at 20 A (UCD9248, UG885). The bitstream is
// timed for 400 MHz (DIV_MIN = 3) but every SM boots at 200 MHz
// (BOOT_DIV_ALL = 6). Same 1,536 MACs as the 12-tile 110_3 build, so the
// same power budget. Run scripts/power_realistic.tcl before enabling many
// SMs at once, and raise the frequency per SM with SET_FREQ.

module vc707_top #(
  parameter integer NUM_SM          = 24,          // 1..24 (2 per TPC, 6 per GPC)
  parameter integer DIV_MIN         = 3,           // compiled divider: 1200/3 = 400 MHz (timing target)
  parameter integer BOOT_DIV_ALL    = 6,           // every SM starts at 1200/6 = 200 MHz
  parameter integer HOST_BAUD       = 921_600,
  parameter integer WIN_LEN_DEFAULT = 200_000,     // clk_mem cycles = 1 ms
  parameter integer K_DEFAULT       = 256
)(
  input  wire       sysclk_200_p,
  input  wire       sysclk_200_n,
  input  wire       cpu_reset,        // SW8, active high
  input  wire [7:0] gpio_dip_sw,
  output wire [7:0] gpio_led,
  input  wire       usb_uart_rxd,     // AU33 (data from the PC)
  output wire       usb_uart_txd      // AU36 (data to the PC)
);

  // ---- fixed architecture ------------------------------------------------
  localparam integer NUM_TILES  = NUM_SM;     // (name used by host_if / win_ctrl / clk_ctrl)
  localparam integer SM_PER_TPC = 2;
  localparam integer TPC_PER_GPC= 3;
  localparam integer SM_PER_GPC = SM_PER_TPC * TPC_PER_GPC;   // 6
  localparam integer N_SUB      = 4;          // sub-partitions per SM
  localparam integer ROWS       = 2;          // C rows per sub-partition (8 per SM)
  localparam integer COLS       = 8;          // C columns
  localparam integer AW         = 9;          // BRAM depth 512
  localparam integer KW         = 10;
  localparam integer K_MIN      = 40;
  localparam integer RAW        = 6;          // log2(64 results)
  localparam integer WDW        = N_SUB * ROWS * 16;   // 128
  localparam integer CLK_MEM_HZ = 200_000_000;
  localparam integer VCO_MHZ    = 1200;
  localparam integer MULT       = 6;
  localparam integer DIV_MAX    = 126;
  localparam integer SEQ_DEPTH  = 256;
  localparam integer REC_DEPTH  = 1024;
  localparam integer NK_W       = 24;         // N_k per SM in a window record (exact up to 16.7 M cycles)
  localparam integer REC_W      = 32 + NK_W * NUM_TILES;   // 76 bytes per 1 ms window: fits 921,600 baud
  localparam [7:0]   BOOT8      = BOOT_DIV_ALL;

  // ---- clk_mem: the board clock itself ----------------------------------
  wire sys_ibuf, clk_mem;
  IBUFDS u_ibufds (.I(sysclk_200_p), .IB(sysclk_200_n), .O(sys_ibuf));
  BUFG   u_bufg_mem (.I(sys_ibuf), .O(clk_mem));

  (* ASYNC_REG = "TRUE" *) reg [1:0] rst_sync = 2'b00;
  always @(posedge clk_mem or posedge cpu_reset)
    if (cpu_reset) rst_sync <= 2'b00;
    else           rst_sync <= {rst_sync[0], 1'b1};
  wire rst_n_mem = rst_sync[1];

  // ---- host interface ----------------------------------------------------------
  wire [NUM_TILES-1:0]      h_stb_a, h_stb_b;
  wire [AW-1:0]             h_addr;
  wire [WDW-1:0]            h_data;
  wire [RAW-1:0]            h_raddr;
  wire [48*NUM_TILES-1:0]   res_flat;
  wire [NUM_TILES-1:0]      stat_req, stat_ack, clr_req, clr_ack, os_req, os_ack;
  wire [128*NUM_TILES-1:0]  stat_flat;
  wire [KW-1:0]             h_k;
  wire [NUM_TILES-1:0]      en_override;
  wire                      freq_req, freq_ack, freq_ok, freq_busy, boot_done;
  wire [7:0]                freq_tile, freq_div;
  wire [8*NUM_TILES-1:0]    div_cur;
  wire [NUM_TILES-1:0]      locked_v, ceon_v;

  wire                      cfg_wr, seq_wr, rec_pop;
  wire [7:0]                cfg_flags, seq_wr_tile, seq_wr_val;
  wire [15:0]               cfg_seq_len, seq_wr_idx, rec_count;
  wire [31:0]               cfg_win_len, win_count;
  wire [REC_W-1:0]          rec_head;
  wire                      rec_overflow, win_running;

  host_if #(
    .NUM_TILES   (NUM_TILES),
    .N_SUB       (N_SUB),
    .ROWS        (ROWS),
    .COLS        (COLS),
    .AW          (AW),
    .KW          (KW),
    .K_MIN       (K_MIN),
    .K_DEFAULT   (K_DEFAULT),
    .CLK_FREQ_HZ (CLK_MEM_HZ),
    .BAUD_RATE   (HOST_BAUD),
    .REC_W       (REC_W),
    .VCO_MHZ     (VCO_MHZ),
    .DIV_MIN     (DIV_MIN),
    .DIV_MAX     (DIV_MAX),
    .SM_PER_GPC  (SM_PER_GPC),
    .NK_W        (NK_W)
  ) u_host_if (
    .clk (clk_mem), .rst_n (rst_n_mem),
    .rxd (usb_uart_rxd), .txd (usb_uart_txd),
    .wr_stb_a (h_stb_a), .wr_stb_b (h_stb_b), .wr_addr (h_addr), .wr_data (h_data),
    .res_raddr (h_raddr), .res_rdata_flat (res_flat),
    .stat_req_tog (stat_req), .stat_ack_tog (stat_ack), .stat_data_flat (stat_flat),
    .clr_req_tog (clr_req), .clr_ack_tog (clr_ack), .cfg_k (h_k),
    .os_req_tog (os_req), .os_ack_tog (os_ack),
    .tile_en_override (en_override),
    .freq_req_tog (freq_req), .freq_tile (freq_tile), .freq_div (freq_div),
    .freq_ack_tog (freq_ack), .freq_ok (freq_ok), .freq_busy (freq_busy),
    .boot_done (boot_done), .div_cur_flat (div_cur), .locked_s (locked_v), .ceon_s (ceon_v),
    .cfg_wr (cfg_wr), .cfg_flags (cfg_flags), .cfg_seq_len (cfg_seq_len), .cfg_win_len (cfg_win_len),
    .seq_wr (seq_wr), .seq_wr_tile (seq_wr_tile), .seq_wr_idx (seq_wr_idx), .seq_wr_val (seq_wr_val),
    .rec_pop (rec_pop), .rec_head (rec_head), .rec_count (rec_count),
    .rec_overflow (rec_overflow), .win_running (win_running)
  );

  // ---- window engine -------------------------------------------------------------
  wire                    seq_run, win_toggle;
  wire [8*NUM_TILES-1:0]  duty_off;
  wire [32*NUM_TILES-1:0] nk_flat;

  win_ctrl #(
    .NUM_TILES       (NUM_TILES),
    .SEQ_DEPTH       (SEQ_DEPTH),
    .DUTY_W          (8),
    .DUTY_DEN        (100),
    .WIN_LEN_DEFAULT (WIN_LEN_DEFAULT),
    .REC_DEPTH       (REC_DEPTH),
    .NK_W            (NK_W),
    .REC_W           (REC_W)
  ) u_win_ctrl (
    .clk (clk_mem), .rst_n (rst_n_mem),
    .cfg_wr (cfg_wr), .cfg_flags (cfg_flags), .cfg_seq_len (cfg_seq_len), .cfg_win_len (cfg_win_len),
    .hw_run (gpio_dip_sw[4]),
    .seq_wr (seq_wr), .seq_wr_tile (seq_wr_tile), .seq_wr_idx (seq_wr_idx), .seq_wr_val (seq_wr_val),
    .seq_run (seq_run), .win_toggle (win_toggle), .duty_off_flat (duty_off),
    .nk_hold_flat (nk_flat),
    .rec_pop (rec_pop), .rec_head (rec_head), .rec_count (rec_count),
    .overflow (rec_overflow), .running (win_running), .win_count (win_count)
  );

  // ---- per-SM clock control -------------------------------------------------------
  wire [NUM_TILES-1:0]    c_den, c_rst, c_gate, c_drdy;
  wire                    c_dwe;
  wire [6:0]              c_daddr;
  wire [15:0]             c_di;
  wire [16*NUM_TILES-1:0] c_do;

  clk_ctrl #(
    .NUM_TILES (NUM_TILES),
    .DIV_MIN   (DIV_MIN),
    .DIV_MAX   (DIV_MAX),
    .BOOT_DIV  ({NUM_TILES{BOOT8}})
  ) u_clk_ctrl (
    .clk (clk_mem), .rst_n (rst_n_mem),
    .req_tog (freq_req), .req_tile (freq_tile), .req_div (freq_div),
    .ack_tog (freq_ack), .ack_ok (freq_ok), .busy (freq_busy), .boot_done (boot_done),
    .div_cur (div_cur),
    .locked_s (locked_v), .ceon_s (ceon_v),
    .drp_den (c_den), .drp_dwe (c_dwe), .drp_daddr (c_daddr), .drp_di (c_di),
    .drp_do_flat (c_do), .drp_drdy (c_drdy),
    .mmcm_rst (c_rst), .gate_off (c_gate)
  );

  // ---- SMs -----------------------------------------------------------------------------
  wire [NUM_TILES-1:0] tile_en;
  wire [NUM_TILES-1:0] led_blk;
  wire                 tile_rst_req = ~rst_n_mem | ~boot_done;

  genvar i;
  generate
    for (i = 0; i < NUM_TILES; i = i + 1) begin : SM
      localparam integer GPC  = i / SM_PER_GPC;
      localparam integer SLOT = i % SM_PER_TPC;     // 0: MMCM, 1: PLL
      assign tile_en[i] = en_override[i] | gpio_dip_sw[5] | gpio_dip_sw[GPC % 4];

      // ---- spine registers (clk_mem, no Pblock) ----
      (* keep = "true" *) reg            sp_sa = 1'b0, sp_sb = 1'b0;
      (* keep = "true" *) reg [AW-1:0]   sp_addr  = {AW{1'b0}};
      (* keep = "true" *) reg [WDW-1:0]  sp_data  = {WDW{1'b0}};
      (* keep = "true" *) reg [RAW-1:0]  sp_raddr = {RAW{1'b0}};
      (* keep = "true" *) reg [KW-1:0]   sp_k     = {KW{1'b0}};
      reg sp_stat_req = 1'b0, sp_clr_req = 1'b0, sp_os_req = 1'b0;
      (* keep = "true" *) reg [47:0]     sp_res   = 48'd0;
      (* keep = "true" *) reg [127:0]    sp_stat  = 128'd0;
      (* keep = "true" *) reg [31:0]     sp_nk    = 32'd0;
      reg sp_stat_ack = 1'b0, sp_clr_ack = 1'b0, sp_os_ack = 1'b0;
      reg sp_den = 1'b0, sp_rst = 1'b0, sp_gate = 1'b0;
      // keep: the DRP bus is the same for every SM, so without keep synthesis
      // merges these copies into one register that drives all 24 MMCMs /
      // PLLs across the whole chip (one long hop at 200 MHz instead of two).
      (* keep = "true" *) reg        sp_dwe   = 1'b0;
      (* keep = "true" *) reg [6:0]  sp_daddr = 7'd0;
      (* keep = "true" *) reg [15:0] sp_di    = 16'd0;
      reg [15:0] sp_do    = 16'd0;
      reg        sp_drdy  = 1'b0, sp_lk = 1'b0, sp_ceon = 1'b0;

      wire [47:0]  t_res;
      wire [127:0] t_stat;
      wire [31:0]  t_nk;
      wire         t_stat_ack, t_clr_ack, t_os_ack;
      wire [15:0]  m_do;
      wire         m_drdy, m_lk, m_ceon;

      always @(posedge clk_mem) begin
        sp_sa       <= h_stb_a[i];
        sp_sb       <= h_stb_b[i];
        sp_addr     <= h_addr;
        sp_data     <= h_data;
        sp_raddr    <= h_raddr;
        sp_k        <= h_k;
        sp_stat_req <= stat_req[i];
        sp_clr_req  <= clr_req[i];
        sp_os_req   <= os_req[i];
        sp_res      <= t_res;
        sp_stat     <= t_stat;
        sp_nk       <= t_nk;
        sp_stat_ack <= t_stat_ack;
        sp_clr_ack  <= t_clr_ack;
        sp_os_ack   <= t_os_ack;
        sp_den      <= c_den[i];
        sp_dwe      <= c_dwe;
        sp_daddr    <= c_daddr;
        sp_di       <= c_di;
        sp_rst      <= c_rst[i];
        sp_gate     <= c_gate[i];
        sp_do       <= m_do;
        sp_drdy     <= m_drdy;
        sp_lk       <= m_lk;
        sp_ceon     <= m_ceon;
      end

      assign res_flat[i*48 +: 48]    = sp_res;
      assign stat_flat[i*128 +: 128] = sp_stat;
      assign nk_flat[i*32 +: 32]     = sp_nk;
      assign stat_ack[i]             = sp_stat_ack;
      assign clr_ack[i]              = sp_clr_ack;
      assign os_ack[i]               = sp_os_ack;
      assign c_do[i*16 +: 16]        = sp_do;
      assign c_drdy[i]               = sp_drdy;
      assign locked_v[i]             = sp_lk;
      assign ceon_v[i]               = sp_ceon;

      // ---- this SM's clock: the TPC's MMCM (slot 0) or PLL (slot 1) ----
      wire clk_raw, clk_free, ce, tclk;
      sm_clk #(
        .USE_PLL      (SLOT),
        .DIV_MIN      (DIV_MIN),
        .MULT         (MULT),
        .CLKIN_PERIOD (5.0)
      ) u_clk (
        .clk_mem   (clk_mem),
        .drp_den   (sp_den), .drp_dwe (sp_dwe), .drp_daddr (sp_daddr), .drp_di (sp_di),
        .drp_do    (m_do),   .drp_drdy (m_drdy),
        .mmcm_rst  (sp_rst), .gate_off (sp_gate),
        .locked_s  (m_lk),   .ceon_s (m_ceon),
        .clk_raw   (clk_raw), .clk_free (clk_free), .ce (ce)
      );
      // regional clock buffers: the placer puts them in this SM's clock
      // region (its MMCM / PLL is LOCed there and all their loads are in
      // its Pblock)
      BUFH   u_bufh_free (.I(clk_raw), .O(clk_free));
      BUFHCE #(
        .CE_TYPE  ("SYNC"),     // glitch-free: CE is sampled on I
        .INIT_OUT (0)
      ) u_bufhce (.I(clk_raw), .CE(ce), .O(tclk));

      // ---- the SM ----
      sm_top #(
        .N_SUB     (N_SUB),
        .ROWS      (ROWS),
        .COLS      (COLS),
        .AW        (AW),
        .KW        (KW),
        .K_MIN     (K_MIN),
        .K_DEFAULT (K_DEFAULT),
        .DUTY_DEN  (100),
        .SEED_A    (32'hA5A50000 ^ (i * 32'h9E3779B9)),   // different A (activations) in every SM
        .SEED_B    (32'h5A5A1234)                          // the same B (weights) everywhere
      ) u_sm (
        .clk_mem       (clk_mem),
        .rst_n_mem     (rst_n_mem),
        .tile_clk      (tclk),
        .tile_rst_req  (tile_rst_req),
        .tile_en_in    (tile_en[i]),
        .seq_run_in    (seq_run),
        .win_toggle_in (win_toggle),
        .duty_off_in   (duty_off[i*8 +: 8]),
        .nk_hold       (t_nk),
        .wr_stb_a      (sp_sa),
        .wr_stb_b      (sp_sb),
        .wr_addr       (sp_addr),
        .wr_data       (sp_data),
        .res_raddr     (sp_raddr),
        .res_rdata     (t_res),
        .stat_req_tog  (sp_stat_req),
        .stat_ack_tog  (t_stat_ack),
        .stat_data     (t_stat),
        .clr_req_tog   (sp_clr_req),
        .clr_ack_tog   (t_clr_ack),
        .cfg_k         (sp_k),
        .os_req_tog    (sp_os_req),
        .os_ack_tog    (t_os_ack),
        .led_blk       (led_blk[i])
      );
    end
  endgenerate

  // ---- LEDs ------------------------------------------------------------------------------
  genvar gl;
  generate
    for (gl = 0; gl < 4; gl = gl + 1) begin : LED_G
      if (gl * SM_PER_GPC < NUM_TILES) begin : ON
        assign gpio_led[gl] = led_blk[gl * SM_PER_GPC];   // SM 0 of GPC gl
      end else begin : OFF
        assign gpio_led[gl] = 1'b0;
      end
    end
  endgenerate
  assign gpio_led[4] = boot_done & (&locked_v);

  reg [31:0] hb_cnt = 32'd0;
  reg        hb_led = 1'b0;
  always @(posedge clk_mem or negedge rst_n_mem)
    if (!rst_n_mem) begin
      hb_cnt <= 32'd0; hb_led <= 1'b0;
    end else if (hb_cnt == CLK_MEM_HZ/2 - 1) begin
      hb_cnt <= 32'd0; hb_led <= ~hb_led;
    end else
      hb_cnt <= hb_cnt + 32'd1;
  assign gpio_led[5] = hb_led;
  assign gpio_led[6] = win_running;
  assign gpio_led[7] = win_running & win_count[9];

endmodule
