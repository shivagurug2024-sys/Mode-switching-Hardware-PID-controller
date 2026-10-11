# ============================================================
# Synthesis Script for PS-02 Mode-Switching Hardware PID Controller
# Tool: Cadence Genus
# ============================================================

# --- 0. Point to RTL files ---
set_db init_hdl_search_path {.}

# --- 1. Load Technology Library (tutorial.lib bundled with Genus) ---
set_db library /home/install/GENUS211/share/synth/tutorials/tech/tutorial.lib

# --- 2. Read RTL ---
read_hdl {pid_core.v watchdog.v top_controller.v}

# --- 3. Elaborate ---
elaborate top_controller

# --- 4. Create Clock (100 MHz = 10 ns) ---
create_clock -name clk -period 10.0 [get_ports clk]

# --- 5. Synthesize ---
syn_generic
syn_map
syn_opt

# --- 6. Report ---
report_area    > area_report.rpt
report_timing  > timing_report.rpt
report_power   > power_report.rpt
report_gates   > gate_count.rpt

# --- 7. Write Netlist ---
write_hdl > top_controller_netlist.v

puts "==========================================="
puts "  SYNTHESIS COMPLETE"
puts "==========================================="
