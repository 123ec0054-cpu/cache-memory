# =====================================================================
# Cadence Genus Synthesis Script for Cache Memory Design
# Target Technology : 90 nm Digital CMOS
# Author            : Venkata Dharun
# Date              : 2025-10-31
# =====================================================================

# ------------------------------------------------------------
# Library Setup
# ------------------------------------------------------------
# Update this path and file name according to your 90nm PDK installation
set_db init_lib_search_path {/home/install/FOUNDRY/digital/90nm/dig/lib/}

# Specify the timing library (example: slow.lib, typical.lib, fast.lib)
set_db library slow.lib

# ------------------------------------------------------------
# Read RTL Sources
# ------------------------------------------------------------
read_hdl {./cache_design.v}

# ------------------------------------------------------------
# Elaborate and Set Current Design
# ------------------------------------------------------------
elaborate cache_memory
current_design cache_memory

# ------------------------------------------------------------
# Apply Timing Constraints
# ------------------------------------------------------------
# If you have an SDC file, read it here, else set default clock constraints
if {[file exists "./constraint_cache_90nm.sdc"]} {
    read_sdc ./constraint_cache_90nm.sdc
} else {
    create_clock -name CLK -period 10 [get_ports clk]
    set_input_delay 1 -clock CLK [all_inputs]
    set_output_delay 1 -clock CLK [all_outputs]
}

# ------------------------------------------------------------
# Set Synthesis Effort Levels
# ------------------------------------------------------------
set_db syn_generic_effort medium
set_db syn_map_effort medium
set_db syn_opt_effort medium

# ------------------------------------------------------------
# Run Full Synthesis Flow
# ------------------------------------------------------------
syn_generic
syn_map
syn_opt

# ------------------------------------------------------------
# Write Synthesized Netlist and Constraint Outputs
# ------------------------------------------------------------
write_hdl > cache_memory_90nm_netlist.v
write_sdc > cache_memory_90nm_output.sdc

# ------------------------------------------------------------
# Generate Reports
# ------------------------------------------------------------
report timing > cache_memory_90nm_timing.rpt
report power  > cache_memory_90nm_power.rpt
report area   > cache_memory_90nm_area.rpt
report gates  > cache_memory_90nm_gates.rpt

# ------------------------------------------------------------
# Optional GUI
# ------------------------------------------------------------
# gui_show  ;# Uncomment if you want to open GUI after synthesis

# =====================================================================
# End of Script
# =====================================================================
