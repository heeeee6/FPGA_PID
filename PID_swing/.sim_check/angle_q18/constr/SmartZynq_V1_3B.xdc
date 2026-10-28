# SmartZynq V1.3B onboard 50 MHz PL clock.
set_property PACKAGE_PIN M19 [get_ports clk_50m]
set_property IOSTANDARD LVCMOS33 [get_ports clk_50m]
create_clock -name clk_50m -period 20.000 [get_ports clk_50m]

# Proposed J6 harness allocation. J6 VADJ must be set to 3.3 V.
# J6 physical pins 1..10 respectively connect these PL signals:
#   1 motor_pwm, 2 motor_in1, 3 motor_in2, 4 motor_stby,
#   5 motor_enc_a, 6 motor_enc_b, 7 oled_scl, 8 oled_sda,
#   9 rst_n, 10 run_enable.
set_property PACKAGE_PIN U22  [get_ports motor_pwm]
set_property PACKAGE_PIN T22  [get_ports motor_in1]
set_property PACKAGE_PIN W22  [get_ports motor_in2]
set_property PACKAGE_PIN V22  [get_ports motor_stby]
set_property PACKAGE_PIN Y21  [get_ports motor_enc_a]
set_property PACKAGE_PIN Y20  [get_ports motor_enc_b]
set_property PACKAGE_PIN AB22 [get_ports oled_scl]
set_property PACKAGE_PIN AA22 [get_ports oled_sda]
set_property PACKAGE_PIN AB21 [get_ports rst_n]
set_property PACKAGE_PIN AA21 [get_ports run_enable]
set_property IOSTANDARD LVCMOS33 [get_ports {motor_pwm motor_in1 motor_in2 motor_stby motor_enc_a motor_enc_b oled_scl oled_sda rst_n run_enable}]
# Floating control inputs hold the FPGA in reset with the motor disabled.
# Drive each input high at 3.3 V through an external switch/jumper to enable.
set_property PULLDOWN true [get_ports rst_n]
set_property PULLDOWN true [get_ports run_enable]

# Dedicated XADC VAUX inputs on J5, in vauxp/n[0..3] order. Condition each
# sensor signal to the XADC's 0..1 V unipolar input range before connecting it.
#   VAUX0  P F16 / N E16  (J5 physical pins 38 / 37)
#   VAUX2  P C15 / N B15  (J5 physical pins 30 / 29)
#   VAUX3  P A16 / N A17  (J5 physical pins 26 / 25)
#   VAUX4  P D20 / N C20  (J5 physical pins 27 / 28)
set_property PACKAGE_PIN F16 [get_ports {vauxp[0]}]
set_property PACKAGE_PIN E16 [get_ports {vauxn[0]}]
set_property PACKAGE_PIN C15 [get_ports {vauxp[1]}]
set_property PACKAGE_PIN B15 [get_ports {vauxn[1]}]
set_property PACKAGE_PIN A16 [get_ports {vauxp[2]}]
set_property PACKAGE_PIN A17 [get_ports {vauxn[2]}]
set_property PACKAGE_PIN D20 [get_ports {vauxp[3]}]
set_property PACKAGE_PIN C20 [get_ports {vauxn[3]}]
