# Logic library setup:
set search_path "* . ./rtl ./libs"
set target_library "nldm_op_cond_typ.db"
set link_library "* $target_library"

# RTL reading
analyze -format verilog {regbank.v muxed_regbank.v control.v alu.v top.v}
elaborate top -parameters "WIDTH=8"
link

# Constrains setup
set clk_val 30

create_clock -period $clk_val [get_ports clk*] -name clk
set_clock_uncertainty -setup [expr $clk_val*0.1] [get_clocks clk]
set_clock_transition -max [expr $clk_val*0.2] [get_clocks clk]
set_clock_latency -source -max [expr $clk_val*0.05] [get_clocks clk]

set_input_delay -max [expr $clk_val*0.4] -clock clk [get_ports [remove_from_collection [all_inputs] clk]]
set_output_delay -max [expr $clk_val*0.5] -clock clk [get_ports [all_outputs]]

set_load -max 0.04 [get_ports [all_outputs]]
set_input_transition -min [expr $clk_val*0.01] [get_ports [all_inputs]]
set_input_transition -max [expr $clk_val*0.1] [get_ports [all_inputs]]


# Pre-compile reports
report_clock > ./reports/report_clock.rpt
analyze_datapath_extraction > ./reports/analyze_datapath_extraction.rpt                                                       

# Compile
compile_ultra -no_autoungroup

# Post-compile reports
analyze_datapath > ./reports/analyze_datapath.rpt
report_resources -hier > ./reports/report_resources.rpt
report_qor > ./reports/report_qor.rpt
report_timing > ./reports/report_timing.rpt
report_area > ./reports/report_area.rpt
report_power > ./reports/report_power.rpt

# Save design
write_file -format ddc -hierarchy -out outputs/lab1.ddc
write_file -format verilog -hier -out outputs/lab1.v
write_sdc outputs/lab1.sdc

# Exit
exit
