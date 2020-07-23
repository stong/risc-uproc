# Constrain clock port CLOCK_50 with a 20-ns requirement

create_clock -period "50.0 MHz" [get_ports CLOCK_50]

# Automatically apply a generate clock on the output of phase-locked loops (PLLs)
# This command can be safely left in the SDC even if no PLLs exist in the design

derive_pll_clocks
derive_clock_uncertainty

# Constrain the input I/O path

set_input_delay -clock [get_clocks *clk] -max 3 [all_inputs]

set_input_delay -clock [get_clocks *clk] -min 2 [all_inputs]

# Constrain the output I/O path

set_output_delay -clock [get_clocks *clk] 0 [all_outputs]