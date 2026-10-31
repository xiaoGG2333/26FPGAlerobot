# Manufacturer RK7100-F fan pins and Q7 inversion, Bank11 3.3V.
set_property PACKAGE_PIN W21 [get_ports fan_pwm]
set_property PACKAGE_PIN Y21 [get_ports fan_tach]
set_property IOSTANDARD LVCMOS33 [get_ports {fan_pwm fan_tach}]
set_property SLEW SLOW [get_ports fan_pwm]
set_property DRIVE 8 [get_ports fan_pwm]
# Tach is asynchronous and enters the three-stage ASYNC_REG synchronizer.
# The fan PWM has no external sampling clock or synchronous timing contract.
set_false_path -from [get_ports fan_tach]
set_false_path -to [get_ports fan_pwm]
