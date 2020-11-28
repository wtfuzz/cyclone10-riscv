/**
 * C10 RISCV SoC core
 * 
 * Matt Thompson <matt@extent3d.com>
 *
 *
 */

module soc_vex
#(
  parameter bootrom_file = "../src/cyclone10_1.0/sw/boot.vh"
)
(
  input 	      wb_clk,
  input 	      wb_rst,

  input 	      dbg_if_select_i,
  output 	      dbg_if_tdo_o,
  input 	      dbg_tck_i,
  input 	      jtag_tap_tdo_i,
  input 	      jtag_tap_shift_dr_i,
  input 	      jtag_tap_pause_dr_i,
  input 	      jtag_tap_update_dr_i,
  input 	      jtag_tap_capture_dr_i,

  input         uart0_srx_pad_i,
  output 	      uart0_stx_pad_o,

  inout [7:0] 	gpio0_io,
  input [3:0] 	gpio1_i
);

  `include "wb_intercon.vh"

  // Endian swap the wishbone sel bits
  wire [3:0] ibus_sel;
  assign wb_m2s_ibus_sel = { ibus_sel[0], ibus_sel[1], ibus_sel[2], ibus_sel[3] };
  
  wire [3:0] dbus_sel;
  assign wb_m2s_dbus_sel = { dbus_sel[0], dbus_sel[1], dbus_sel[2], dbus_sel[3] };
  
  
  assign scope[7:0] = wb_m2s_ibus_adr[7:0];
  
  wire debug_resetOut;
  
  VexRiscvWishbone cpu0
  (
    .clk		                  (wb_clk),
    .reset		                (wb_rst | debug_resetOut),
    .debugReset               (wb_rst),
    .debug_resetOut           (debug_resetOut),
  
    .jtag_tms                 (jtag_tms),
    .jtag_tdi                 (dbg_if_tdo_o),
    .jtag_tdo                 (jtag_tap_tdo_i),
    .jtag_tck                 (dbg_tck_i),
  
    .iBusWishbone_CYC         (wb_m2s_ibus_cyc),
    .iBusWishbone_STB         (wb_m2s_ibus_stb),
    .iBusWishbone_ACK         (wb_s2m_ibus_ack),
    .iBusWishbone_WE          (wb_m2s_ibus_we),
    .iBusWishbone_ADR         (wb_m2s_ibus_adr[31:2]),
    .iBusWishbone_DAT_MISO    (wb_s2m_ibus_dat),
    .iBusWishbone_DAT_MOSI    (wb_m2s_ibus_dat),
    .iBusWishbone_SEL         (ibus_sel),
    .iBusWishbone_ERR         (wb_s2m_ibus_err),
  
    .dBusWishbone_CYC         (wb_m2s_dbus_cyc),
    .dBusWishbone_STB         (wb_m2s_dbus_stb),
    .dBusWishbone_ACK         (wb_s2m_dbus_ack),
    .dBusWishbone_WE          (wb_m2s_dbus_we),
    .dBusWishbone_ADR         (wb_m2s_dbus_adr[31:2]),
    .dBusWishbone_DAT_MISO    (wb_s2m_dbus_dat),
    .dBusWishbone_DAT_MOSI    (wb_m2s_dbus_dat),
    .dBusWishbone_SEL         (dbus_sel),
    .dBusWishbone_ERR         (wb_s2m_dbus_err)
  );
 
  /** UART */
  uart_top uart0
  (
    // Wishbone slave interface  
    .wb_clk_i (wb_clk),
    .wb_rst_i (wb_rst),
    .wb_adr_i (wb_m2s_uart0_adr),
    .wb_dat_i (wb_m2s_uart0_dat),
    .wb_we_i  (wb_m2s_uart0_we), 
    .wb_stb_i (wb_m2s_uart0_stb),
    .wb_cyc_i (wb_m2s_uart0_cyc),
    .wb_sel_i (4'b0), // Not used in 8-bit mode
    .wb_dat_o (wb_s2m_uart0_dat),
    .wb_ack_o (wb_s2m_uart0_ack),
  
    // Outputs
    .int_o      (uart0_irq),
    .stx_pad_o  (uart0_stx_pad_o),
    .rts_pad_o  (),
    .dtr_pad_o  (),
  
    // Inputs
    .srx_pad_i  (uart0_srx_pad_i),
    .cts_pad_i  (1'b0),  
    .dsr_pad_i  (1'b0),  
    .ri_pad_i   (1'b0),
    .dcd_pad_i  (1'b0)
  );
  
  
  /** Coreport GPIO Peripheral */
  coreport #(
    .WIDTH(8)
  )
  gpio0
  ( 
    .wb_clk   (wb_clk),
    .wb_rst   (wb_rst),
    .wb_adr_i (wb_m2s_gpio0_adr),
    .wb_dat_i (wb_m2s_gpio0_dat),
    .wb_we_i  (wb_m2s_gpio0_we), 
    .wb_cyc_i (wb_m2s_gpio0_cyc),
    .wb_stb_i (wb_m2s_gpio0_stb), 
    .wb_cti_i (wb_m2s_gpio0_cti), 
    .wb_bte_i (wb_m2s_gpio0_bte),
    .wb_dat_o (wb_s2m_gpio0_dat),
    .wb_ack_o (wb_s2m_gpio0_ack),
    .wb_err_o (wb_s2m_gpio0_err),
    .wb_rty_o (wb_s2m_gpio0_rty),
  
    .gpio_io  (gpio0_io[7:0]),
    .irq      (gpio0_irq)
  );
  
  
  /** RAM */
  wb_ram #(
    .depth(16384)
  )
  ram0
  (
    .wb_clk_i (wb_clk),
    .wb_rst_i (wb_rst),
    .wb_adr_i (wb_m2s_ram_adr),
    .wb_cyc_i (wb_m2s_ram_cyc),
    .wb_stb_i (wb_m2s_ram_stb),
    .wb_dat_o (wb_s2m_ram_dat),
    .wb_ack_o (wb_s2m_ram_ack)
  );
  
  assign wb_s2m_ram_err = 1'b0;
  assign wb_s2m_ram_rty = 1'b0;
  
  /** ROM */
  localparam WB_BOOTROM_MEM_DEPTH = 1024;
  
  wb_bootrom
  #(
    .DEPTH (WB_BOOTROM_MEM_DEPTH),
    .MEMFILE (bootrom_file)
  )
  rom0
  (
    //Wishbone Master interface
    .wb_clk_i (wb_clk),
    .wb_rst_i (wb_rst),
    .wb_adr_i (wb_m2s_rom0_adr),
    .wb_cyc_i (wb_m2s_rom0_cyc),
    .wb_stb_i (wb_m2s_rom0_stb),
    .wb_dat_o (wb_s2m_rom0_dat),
    .wb_ack_o (wb_s2m_rom0_ack)
  );
  
  assign wb_s2m_rom0_err = 1'b0;
  assign wb_s2m_rom0_rty = 1'b0;

endmodule
