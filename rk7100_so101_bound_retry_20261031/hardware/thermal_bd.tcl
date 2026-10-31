# Independent cooling + read-only telemetry, keeping both IK addresses intact.
set_property CONFIG.NUM_MI 3 [get_bd_cells ps7_0_axi_periph]
create_bd_cell -type module -reference chip_thermal_top chip_thermal_0
create_bd_cell -type ip -vlnv xilinx.com:ip:axi_gpio:2.0 thermal_gpio
set_property -dict [list CONFIG.C_GPIO_WIDTH 32 CONFIG.C_ALL_INPUTS 1 CONFIG.C_IS_DUAL 1 CONFIG.C_GPIO2_WIDTH 32 CONFIG.C_ALL_INPUTS_2 1] [get_bd_cells thermal_gpio]
connect_bd_intf_net [get_bd_intf_pins ps7_0_axi_periph/M02_AXI] [get_bd_intf_pins thermal_gpio/S_AXI]
foreach pin {ps7_0_axi_periph/M02_ACLK thermal_gpio/s_axi_aclk chip_thermal_0/clk} {
 connect_bd_net [get_bd_pins processing_system7_0/FCLK_CLK0] [get_bd_pins $pin]
}
foreach pin {ps7_0_axi_periph/M02_ARESETN thermal_gpio/s_axi_aresetn chip_thermal_0/resetn} {
 connect_bd_net [get_bd_pins rst_ps7_0_100M/peripheral_aresetn] [get_bd_pins $pin]
}
connect_bd_net [get_bd_pins chip_thermal_0/thermal_status] [get_bd_pins thermal_gpio/gpio_io_i]
connect_bd_net [get_bd_pins chip_thermal_0/fan_rpm] [get_bd_pins thermal_gpio/gpio2_io_i]
create_bd_port -dir O fan_pwm
create_bd_port -dir I fan_tach
connect_bd_net [get_bd_ports fan_pwm] [get_bd_pins chip_thermal_0/fan_pwm]
connect_bd_net [get_bd_ports fan_tach] [get_bd_pins chip_thermal_0/fan_tach]
assign_bd_address -offset 0x41200000 -range 0x00010000 -target_address_space [get_bd_addr_spaces processing_system7_0/Data] [get_bd_addr_segs thermal_gpio/S_AXI/Reg]
