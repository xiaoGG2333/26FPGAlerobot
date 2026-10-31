# Rebuild the current editable Vivado project without replacing its Block Design.
set root [file normalize [file join [file dirname [info script]] ..]]
set project_dir $root/build_opt125
set artifact_dir $root/build/hardware_export
open_project $project_dir/rk7100_so101.xpr
update_ip_catalog
open_bd_design [get_files */design_1.bd]
set_property CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ 125 [get_bd_cells processing_system7_0]
set cores [get_bd_cells -filter {VLNV =~ *:xf_solver_lm_so101:*}]
if {[llength $cores]} {upgrade_bd_cells $cores}
validate_bd_design
save_bd_design
generate_target all [get_files */design_1.bd]
update_compile_order -fileset sources_1
reset_run synth_1
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
close_project
