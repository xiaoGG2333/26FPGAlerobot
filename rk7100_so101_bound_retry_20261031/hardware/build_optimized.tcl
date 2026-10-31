if {[file exists D:/VivadoProjects/RK7020_So_101/build_opt125/rk7100_so101.xpr]} {
    source [file join [file dirname [info script]] ../scripts/rebuild_hardware.tcl]
    return
}
set here [file dirname [info script]]
set artifact_dir D:/VivadoProjects/RK7020_So_101/build/hardware_export
file mkdir $artifact_dir
set project_dir D:/VivadoProjects/RK7020_So_101/build_opt125
set ip_repo D:/VivadoProjects/RK7020_So_101/hls_build/hls/impl/ip
create_project rk7100_so101 $project_dir -part xc7z100ffg900-2 -force
set_property target_language Verilog [current_project]
set_property ip_repo_paths [list $ip_repo] [current_project]
update_ip_catalog
add_files $here/../rtl/chip_thermal.v
source $here/design_1.tcl
add_files -fileset constrs_1 $here/servo_uart.xdc
add_files -fileset constrs_1 $here/fan.xdc
set bd [get_files */design_1.bd]
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
file copy -force $project_dir/rk7100_so101.runs/impl_1/design_1_wrapper.bit $artifact_dir/rk7100_so101.bit
file copy -force $project_dir/rk7100_so101.gen/sources_1/bd/design_1/hw_handoff/design_1.hwh $artifact_dir/rk7100_so101.hwh
write_hw_platform -fixed -include_bit -force -file $artifact_dir/rk7100_so101.xsa
puts UART1_BUILD_COMPLETE

