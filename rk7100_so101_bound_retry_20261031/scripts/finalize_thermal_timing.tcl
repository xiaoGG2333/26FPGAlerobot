set root [file normalize [file join [file dirname [info script]] ..]]
open_project $root/build/rk7100_so101.xpr
open_run impl_1
source $root/hardware/fan.xdc
set artifact_dir $root/build/hardware_export
report_timing_summary -file $artifact_dir/timing.rpt
report_drc -file $artifact_dir/drc.rpt
report_utilization -file $artifact_dir/utilization.rpt
foreach type {max min} {
 set p [get_timing_paths -delay_type $type -max_paths 1]
 if {[llength $p] && [get_property SLACK [lindex $p 0]]<0} {error "Timing failed"}
}
# Timing-only exceptions; no placement/routing or functional netlist changes.
write_checkpoint -force $root/build/rk7100_so101.runs/impl_1/design_1_wrapper_routed.dcp
write_bitstream -force $root/build/rk7100_so101.runs/impl_1/design_1_wrapper.bit
file copy -force $root/build/rk7100_so101.runs/impl_1/design_1_wrapper.bit $artifact_dir/rk7100_so101.bit
write_hw_platform -fixed -include_bit -force -file $artifact_dir/rk7100_so101.xsa
close_project
