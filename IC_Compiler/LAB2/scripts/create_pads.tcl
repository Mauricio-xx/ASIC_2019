## Create and Hook up PADS
create_cell {u_cornerul u_cornerur u_cornerlr u_cornerll } Pad_Corner_New -view FRAM 

create_cell {u_vdd} pad_vdd -view FRAM
create_cell {u_gnd} pad_gnd -view FRAM 
create_net -power vdd!
create_net -ground gnd!
connect_net vdd! u_vdd/vdd!
connect_net gnd! u_gnd/gnd!

set inputs [get_ports -filter "direction==in"]
set outputs [get_ports -filter "direction==out"]
foreach_in_collection port $inputs {
    set name [ get_attribute $port name]
    create_cell u_${name}_pad pad_bidirhe -view FRAM 
    connect_net [get_nets -of_object [get_ports $name]] u_${name}_pad/DataIn
    create_cell u_tielo_$name TIELO
    connect_pin -from u_tielo_$name/Y -to u_${name}_pad/EN

    disconnect_net [get_nets -of_objects [get_ports $name]] [get_ports $name]
    create_net "${name}_pad"
    connect_net ${name}_pad u_${name}_pad/pad
    connect_net ${name}_pad [get_ports $name]
}
foreach_in_collection port $outputs {
    set name [ get_attribute $port name]
    create_cell u_${name}_pad pad_bidirhe -view FRAM
    connect_net [get_nets -of_objects [get_ports $name]]  u_${name}_pad/DataOut
    create_cell  u_tiehi_$name TIEHI
    connect_pin -from u_tiehi_$name/Y -to u_${name}_pad/EN

    disconnect_net [get_nets -of_objects [get_ports $name]] [get_ports $name]
    create_net "${name}_pad"
    connect_net ${name}_pad u_${name}_pad/pad
    connect_net ${name}_pad [get_ports $name]
}

# Pad constraints 
set_pad_physical_constraints -pad_name "u_cornerul" -side 1  
set_pad_physical_constraints -pad_name "u_cornerur" -side 2 
set_pad_physical_constraints -pad_name "u_cornerlr" -side 3 
set_pad_physical_constraints -pad_name "u_cornerll" -side 4

#foreach_in_collection port $outputs {
#  set portname [get_object_name $port]
#  set_pad_physical_constraints -pad_name u_${portname}_pad -side 4
#}

foreach_in_collection pad [get_cells *_pad] {
  set_attribute -type boolean [get_pins [get_object_name $pad]/pad] is_pad true
}
