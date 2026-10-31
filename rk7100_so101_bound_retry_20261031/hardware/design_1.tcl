# RK-ZYNQ7100-F dual SO101 accelerator block design (Vivado 2024.2).
# Called by ik.tcl after installing the matching HLS IP repository.
set overlay_dir [file dirname [info script]]
create_bd_design design_1
create_bd_cell -type ip -vlnv xilinx.com:ip:processing_system7:5.5 processing_system7_0
source $overlay_dir/rk7100_ps.tcl
make_bd_intf_pins_external [get_bd_intf_pins processing_system7_0/DDR]
make_bd_intf_pins_external [get_bd_intf_pins processing_system7_0/FIXED_IO]
set_property name DDR [get_bd_intf_ports DDR_0]
set_property name FIXED_IO [get_bd_intf_ports FIXED_IO_0]
create_bd_cell -type ip -vlnv xilinx.com:ip:axi_interconnect:2.1 ps7_0_axi_periph
set_property CONFIG.NUM_MI 2 [get_bd_cells ps7_0_axi_periph]
create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 rst_ps7_0_100M
set_property CONFIG.C_EXT_RESET_HIGH 0 [get_bd_cells rst_ps7_0_100M]
create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant:1.1 reset_lock
set_property CONFIG.CONST_VAL 1 [get_bd_cells reset_lock]
connect_bd_net [get_bd_pins reset_lock/dout] [get_bd_pins rst_ps7_0_100M/dcm_locked]
connect_bd_intf_net [get_bd_intf_pins processing_system7_0/M_AXI_GP0] [get_bd_intf_pins ps7_0_axi_periph/S00_AXI]
foreach i {0 1} {
    set core xf_solver_lm_so101_$i
    create_bd_cell -type ip -vlnv xilinx.com:hls:xf_solver_lm_so101:1.0 $core
    connect_bd_intf_net [get_bd_intf_pins ps7_0_axi_periph/M0${i}_AXI] [get_bd_intf_pins $core/s_axi_control]
    connect_bd_net [get_bd_pins processing_system7_0/FCLK_CLK0] [get_bd_pins $core/ap_clk]
    connect_bd_net [get_bd_pins rst_ps7_0_100M/peripheral_aresetn] [get_bd_pins $core/ap_rst_n]
    assign_bd_address -offset [format 0x%08X [expr {0x40000000+$i*0x10000}]] -range 0x00010000 \
        -target_address_space [get_bd_addr_spaces processing_system7_0/Data] [get_bd_addr_segs $core/s_axi_control/Reg]
}
foreach pin {processing_system7_0/M_AXI_GP0_ACLK ps7_0_axi_periph/ACLK ps7_0_axi_periph/S00_ACLK ps7_0_axi_periph/M00_ACLK ps7_0_axi_periph/M01_ACLK rst_ps7_0_100M/slowest_sync_clk} {
    connect_bd_net [get_bd_pins processing_system7_0/FCLK_CLK0] [get_bd_pins $pin]
}
connect_bd_net [get_bd_pins processing_system7_0/FCLK_RESET0_N] [get_bd_pins rst_ps7_0_100M/ext_reset_in]
connect_bd_net [get_bd_pins rst_ps7_0_100M/interconnect_aresetn] [get_bd_pins ps7_0_axi_periph/ARESETN]
foreach pin {S00_ARESETN M00_ARESETN M01_ARESETN} {
    connect_bd_net [get_bd_pins rst_ps7_0_100M/peripheral_aresetn] [get_bd_pins ps7_0_axi_periph/$pin]
}
# UART1 EMIO: J8 physical pin 3 TX / pin 4 RX.
create_bd_port -dir O servo_uart_tx
create_bd_port -dir I servo_uart_rx
connect_bd_net [get_bd_pins processing_system7_0/UART1_TX] [get_bd_ports servo_uart_tx]
connect_bd_net [get_bd_pins processing_system7_0/UART1_RX] [get_bd_ports servo_uart_rx]
source $overlay_dir/thermal_bd.tcl
validate_bd_design
save_bd_design
