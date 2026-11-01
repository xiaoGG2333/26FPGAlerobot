set root [file normalize [file join [file dirname [info script]] ..]]
if {![string match "2025.2*" [version -short]]} {error "Wrong Vivado version"}
if {[llength [get_parts -quiet xc7z100ffg900-2]] != 1} {error "Missing device support"}
create_project preflight $root/build/preflight -part xc7z100ffg900-2 -force
create_bd_design ps_check
create_bd_cell -type ip -vlnv xilinx.com:ip:processing_system7:5.5 processing_system7_0
source $root/hardware/rk7100_ps.tcl
set f [open $root/reports/ps_config_2025_2.txt w]
report_property [get_bd_cells processing_system7_0] -file $root/reports/ps_properties.txt
puts $f "Vivado [version -short]"
puts $f "part=[get_property PART [current_project]]"
puts $f "fclk0=[get_property CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ [get_bd_cells processing_system7_0]]"
close $f
close_project
puts DEVICE_PS_PREFLIGHT_PASS
