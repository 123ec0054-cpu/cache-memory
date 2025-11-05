# =====================================================================
# constraint_parallel_prefix_90nm.sdc
# Timing and Physical Constraints for 32-bit Parallel Prefix Comparator + Adder
# Target: 90 nm CMOS (High-Speed)
# =====================================================================

# ------------------------------------------------------------
# Clock Definition
# ------------------------------------------------------------
# Define a 10 ns clock (100 MHz typical for 90 nm)
create_clock -name clk -period 10 [get_ports clk]

# ------------------------------------------------------------
# Input and Output Delays
# ------------------------------------------------------------
# Moderate interface delay assumptions for high-speed 90 nm logic
set_input_delay 1.5 -clock clk [get_ports {A[*] B[*] enable reset_n}]
set_output_delay 1.5 -clock clk [get_ports {SUM[*] A_greater A_equal A_smaller}]

# ------------------------------------------------------------
# Drive and Load Assumptions
# ------------------------------------------------------------
# Typical driving strength for all inputs
set_drive 1 [all_inputs]

# Output load (in pF)
set_load 0.05 [all_outputs]

# ------------------------------------------------------------
# Operating Conditions
# ------------------------------------------------------------
# Specify slow corner (matches slow.lib)
set_operating_conditions -library slow.lib -analysis_type on_chip_variation

# ------------------------------------------------------------
# Design Exceptions / False Paths
# ------------------------------------------------------------
# Ignore paths from enable or reset to outputs (non-timing-critical)
set_false_path -from [get_ports {enable reset_n}] -to [get_ports {SUM[*] A_greater A_equal A_smaller}]

# ------------------------------------------------------------
# End of File
# =====================================================================
