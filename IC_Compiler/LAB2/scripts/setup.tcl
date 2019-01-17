

set target_library "c5n_utah_std_v5_t27.db"
set link_library "* $target_library c5n_utah_std_v5_t27.db io.db"
set search_path "./MW ./logic ./scripts ./inputs/"
set mw_reference_libraries "./MW/UTAH_V1P1 ./MW/ICG ./MW/IO"
set TECH_FILE "./MW/UTAH.tf"
set TLUPLUS_MAX_FILE "./MW/ami500.tluplus"
set TLUPLUS_MIN_FILE "./MW/ami500.tluplus"
set MAP_FILE "./MW/ami500hxkx_3m.map"; #map file


set mw_design_library MW_TOP_LIB 
set mw_reference_library "./MW/UTAH_V1P1 ./MW/ICG ./MW/IO"
#
sh rm -rf $mw_design_library

#
set mw_use_layer_enhancement true
create_mw_lib  -technology $TECH_FILE\
	       -mw_reference_library $mw_reference_library\
	        $mw_design_library


open_mw_lib $mw_design_library

check_library
#
set_tlu_plus_files -max_tluplus $TLUPLUS_MAX_FILE\
		    -min_tluplus $TLUPLUS_MIN_FILE\
		    -tech2itf_map $MAP_FILE
 
check_tlu_plus_files
