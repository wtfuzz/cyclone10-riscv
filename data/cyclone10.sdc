# Main system clock (50 Mhz)
create_clock -name "c10_clk50m" -period 20.000ns [get_ports {c10_clk50m}]
create_clock -name "enet_clk_125m" -period 8.000ns [get_ports {enet_clk_125m}]
create_clock -name "enet_rx_clk" -period 8.000ns [get_ports {enet_rx_clk}]

# Automatically constrain PLL and other generated clocks
derive_pll_clocks -create_base_clocks

# Automatically calculate clock uncertainty to jitter and other effects.
derive_clock_uncertainty

# Ignore timing on the reset input
# set_false_path -through [get_nets {rst_n_pad_i}]

