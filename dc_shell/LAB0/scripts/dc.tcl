# Set tool variables
set search_path "* . ./rtl ./libs"
set target_library "c5n_utah_std_v5_t27.db"
set link_library "* $target_library io.db"

# Generate netlist from RTL
read_verilog fulladder.v
link
