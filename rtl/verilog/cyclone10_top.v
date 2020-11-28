/**
 * C10 RISCV SoC top level design
 *
 * Matt Thompson <matt@extent3d.com>
 */
module cyclone10_top
(
  input 	      c10_clk50m,
  input 	      c10_resetn,
  inout  [35:0] gpio,
  output [3:0]  user_led,
  input  [3:0]  user_pb,
  input         enet_clk_125m,
  input         enet_rx_clk,
  input  [3:0]  enet_rx_d,
  input         enet_rx_dv,
  output        enet_tx_clk,
  output        enet_tx_en,
  output [3:0]  enet_tx_d,
  output        enet_resetn,
  output        enet_mdc,
  inout         enet_mdio,
  input         enet_int
);

assign gpio[0] = 1'b0;
assign gpio[9] = 1'b0;


////////////////////////////////////////////////////////////////////////
//
// Clock and reset generation module
//
////////////////////////////////////////////////////////////////////////

wire 	 wb_clk;
wire 	 wb_rst;

clkgen clkgen0
(
  .sys_clk_pad_i (c10_clk50m),
  .rst_n_pad_i   (c10_resetn),
  .wb_clk_o      (wb_clk),
  .wb_rst_o      (wb_rst),
);

////////////////////////////////////////////////////////////////////////
//
// JTAG
//
////////////////////////////////////////////////////////////////////////

wire 	 dbg_if_select;
wire 	 dbg_if_tdo;
wire 	 dbg_tck;
wire 	 jtag_tap_tdo;
wire 	 jtag_tap_shift_dr;
wire 	 jtag_tap_pause_dr;
wire 	 jtag_tap_update_dr;
wire 	 jtag_tap_capture_dr;

altera_virtual_jtag jtag_tap0
(
  .tck_o			(dbg_tck),
  .debug_tdo_i		(dbg_if_tdo),
  .tdi_o			(jtag_tap_tdo),
  .test_logic_reset_o	(),
  .run_test_idle_o	(),
  .shift_dr_o		(jtag_tap_shift_dr),
  .capture_dr_o		(jtag_tap_capture_dr),
  .pause_dr_o		(jtag_tap_pause_dr),
  .update_dr_o		(jtag_tap_update_dr),
  .debug_select_o		(dbg_if_select));

////////////////////////////////////////////////////////////////////////
//
// Core
//
////////////////////////////////////////////////////////////////////////

// UART TX is gpio[3]
// UART RX is gpio[5]

soc_vex core
(
  //Clock and reset
  .wb_clk (wb_clk),
  .wb_rst (wb_rst),

  //Debug interface
  .dbg_if_select_i       (dbg_if_select),
  .dbg_if_tdo_o          (dbg_if_tdo),
  .dbg_tck_i             (dbg_tck),
  .jtag_tap_tdo_i        (jtag_tap_tdo),
  .jtag_tap_shift_dr_i   (jtag_tap_shift_dr),
  .jtag_tap_pause_dr_i   (jtag_tap_pause_dr),
  .jtag_tap_update_dr_i  (jtag_tap_update_dr),
  .jtag_tap_capture_dr_i (jtag_tap_capture_dr),

  .uart0_srx_pad_i	(gpio[5]),
  .uart0_stx_pad_o	(gpio[3]),

  .gpio0_io (user_led[3:0])
);

endmodule
