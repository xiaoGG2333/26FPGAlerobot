set root [file normalize [file join [file dirname [info script]] ..]]
open_project $root/build/rk7100_so101.xpr
add_files $root/rtl/chip_thermal.v
update_compile_order -fileset sources_1
open_bd_design [get_files */design_1.bd]
source $root/hardware/thermal_bd.tcl
validate_bd_design
save_bd_design
add_files -fileset constrs_1 $root/hardware/fan.xdc
generate_target all [get_files */design_1.bd]
add_files -norecurse [make_wrapper -files [get_files */design_1.bd] -top]
update_compile_order -fileset sources_1
close_project
source $root/scripts/rebuild_hardware.tcl
