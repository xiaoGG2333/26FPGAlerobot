# RK-ZYNQ7100-F J8 3/4, Bank10 VCC_ADJ2 MUST be 3.3 V.
set_property PACKAGE_PIN AK16 [get_ports servo_uart_tx]
set_property PACKAGE_PIN AJ16 [get_ports servo_uart_rx]
set_property IOSTANDARD LVCMOS33 [get_ports {servo_uart_tx servo_uart_rx}]
set_property SLEW SLOW [get_ports servo_uart_tx]
set_property DRIVE 8 [get_ports servo_uart_tx]
set_property PULLUP true [get_ports servo_uart_rx]
