set root [file normalize [file join [file dirname [info script]] ..]]
if {![string match "2025.2*" [version -short]]} {error "Vivado 2025.2 required"}
set artifact_dir $root/build/hardware_export
file mkdir $artifact_dir
if {[llength [get_parts -quiet xc7z100ffg900-2]] != 1} {error "Zynq-7000 device support missing"}
create_project ik $root/build/v -part xc7z100ffg900-2 -force
set_property target_language Verilog [current_project]
set_property ip_repo_paths [list $root/build/hls/hls/impl/ip] [current_project]
update_ip_catalog
add_files $root/rtl/chip_thermal.v
source $root/hardware/design_1.tcl
add_files -fileset constrs_1 [list $root/hardware/servo_uart.xdc $root/hardware/fan.xdc]
set bd [get_files */design_1.bd]
if {abs([get_property CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ [get_bd_cells processing_system7_0]]-125)>0.001} {error "Clock mismatch"}
write_bd_tcl -force $artifact_dir/recreate_bd.tcl
generate_target all $bd
add_files -norecurse [make_wrapper -files $bd -top]
set_property top design_1_wrapper [current_fileset]
update_compile_order -fileset sources_1
launch_runs synth_1 -jobs 4
wait_on_run synth_1
if {[get_property PROGRESS [get_runs synth_1]] ne "100%"} {error "Synthesis failed"}
launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] ne "100%"} {error "Implementation failed"}
open_run impl_1
report_timing_summary -file $artifact_dir/timing.rpt
report_drc -file $artifact_dir/drc.rpt
report_utilization -file $artifact_dir/utilization.rpt
foreach type {max min} {
 set paths [get_timing_paths -delay_type $type -max_paths 1]
 if {[llength $paths] && [get_property SLACK [lindex $paths 0]] < 0} {error "Timing failed"}
}
file copy -force $root/build/v/ik.runs/impl_1/design_1_wrapper.bit $artifact_dir/rk7100_so101.bit
file copy -force $root/build/v/ik.gen/sources_1/bd/design_1/hw_handoff/design_1.hwh $artifact_dir/rk7100_so101.hwh
write_hw_platform -fixed -include_bit -force -file $artifact_dir/rk7100_so101.xsa
close_project
puts HARDWARE_2025_2_COMPLETE
