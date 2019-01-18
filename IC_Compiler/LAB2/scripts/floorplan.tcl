###############################################################################
# Design Planning
# Floorplan:
# - When short on routing layers adjust row_core_ratio to provide space 
#   between site rows.
################################################################################

# Append IO Cells 
source scripts/create_pads.tcl 

create_floorplan \
    -control_type "aspect_ratio" \
    -core_aspect_ratio "1" \
    -core_utilization "0.7" \
    -row_core_ratio "0.8" \
    -no_double_back \
    -start_first_row \
    -left_io2core  -30 \
    -right_io2core -30 \
    -bottom_io2core -30 \
    -top_io2core -30

# row core ratio defines spacing between rows
     
insert_pad_filler -cell "pad_space43_2 " 
derive_pg_connection -power_net {vdd!} -ground_net {gnd!}  -create_ports top 
create_pad_rings -nets {vdd! gnd!}  

# PNS
# Strap numbers is a trade off between congestion and IR Drop 
create_power_straps  \
    -direction vertical \
    -num_placement_strap 7 \
    -start_at 450  \
    -increment_x_or_y 420 \
    -nets  {vdd! gnd!}  \
    -width 2.700 \
    -layer metal3
#
