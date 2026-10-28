# SmartZynq V1.3B onboard 50 MHz PL clock.
set_property PACKAGE_PIN M19 [get_ports clk_50m]
set_property IOSTANDARD LVCMOS33 [get_ports clk_50m]
create_clock -period 20.000 -name clk_50m [get_ports clk_50m]

# Proposed J6 harness allocation. J6 VADJ must be set to 3.3 V.
# J6 physical pins 1..10 respectively connect these PL signals:
#   1 motor_pwm, 2 motor_in1, 3 motor_in2, 4 motor_stby,
#   5 motor_enc_a, 6 motor_enc_b, 7 oled_scl, 8 oled_sda,
#   9 rst_n, 10 run_enable.
set_property PACKAGE_PIN U22 [get_ports motor_pwm]
set_property PACKAGE_PIN T22 [get_ports motor_in1]
set_property PACKAGE_PIN W22 [get_ports motor_in2]
set_property PACKAGE_PIN V22 [get_ports motor_stby]
set_property PACKAGE_PIN Y21 [get_ports motor_enc_a]
set_property PACKAGE_PIN Y20 [get_ports motor_enc_b]
set_property PACKAGE_PIN AB22 [get_ports oled_scl]
set_property PACKAGE_PIN AA22 [get_ports oled_sda]
set_property PACKAGE_PIN AB21 [get_ports rst_n]
set_property PACKAGE_PIN AA21 [get_ports run_enable]
set_property IOSTANDARD LVCMOS33 [get_ports motor_pwm]
set_property IOSTANDARD LVCMOS33 [get_ports motor_in1]
set_property IOSTANDARD LVCMOS33 [get_ports motor_in2]
set_property IOSTANDARD LVCMOS33 [get_ports motor_stby]
set_property IOSTANDARD LVCMOS33 [get_ports motor_enc_a]
set_property IOSTANDARD LVCMOS33 [get_ports motor_enc_b]
set_property IOSTANDARD LVCMOS33 [get_ports oled_scl]
set_property IOSTANDARD LVCMOS33 [get_ports oled_sda]
set_property IOSTANDARD LVCMOS33 [get_ports rst_n]
set_property IOSTANDARD LVCMOS33 [get_ports run_enable]
# AUTO_START is enabled in pid_swing_hw_top, matching the STM32 application.
# Pull these optional manual-control inputs high so a disconnected J6 harness
# cannot leave a manually configured build permanently in reset/idle.
set_property PULLUP true [get_ports rst_n]
set_property PULLUP true [get_ports run_enable]
# Keep the TB6612 controls low while the FPGA is configuring or reconfiguring.
# Add an external 10 kOhm pull-down on STBY as the final hardware safeguard.
set_property PULLDOWN true [get_ports motor_pwm]
set_property PULLDOWN true [get_ports motor_in1]
set_property PULLDOWN true [get_ports motor_in2]
set_property PULLDOWN true [get_ports motor_stby]

# Four active-low push-buttons on the STM32-compatible baseboard. These are
# optional during motor commissioning; the RTL debounces them at 1 kHz.
# J6 pins 13..16: K1, K2, K3, K4.
set_property PACKAGE_PIN AA19 [get_ports {key_n[0]}]
set_property PACKAGE_PIN Y19  [get_ports {key_n[1]}]
set_property PACKAGE_PIN AB16 [get_ports {key_n[2]}]
set_property PACKAGE_PIN AA16 [get_ports {key_n[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {key_n[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {key_n[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {key_n[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {key_n[3]}]
set_property PULLUP true [get_ports {key_n[0]}]
set_property PULLUP true [get_ports {key_n[1]}]
set_property PULLUP true [get_ports {key_n[2]}]
set_property PULLUP true [get_ports {key_n[3]}]

# Two ADS1115 modules on a dedicated I2C bus (separate from the OLED bus).
#   ADS1115 #1 (ADDR -> GND, 0x48): AIN0 = SENSOR1 pendulum angle
#   ADS1115 #2 (ADDR -> VDD, 0x49): AIN0..AIN3 = RP1..RP4
# J6 pin 11 = ads_scl, J6 pin 12 = ads_sda.  Modules must be powered from
# 3.3 V; J6 VADJ stays at 3.3 V.  Most modules already have pull-ups; the
# FPGA pull-up is only a weak extra.
set_property PACKAGE_PIN AB19 [get_ports ads_scl]
set_property PACKAGE_PIN AB20 [get_ports ads_sda]
set_property IOSTANDARD LVCMOS33 [get_ports ads_scl]
set_property IOSTANDARD LVCMOS33 [get_ports ads_sda]
set_property PULLUP true [get_ports ads_scl]
set_property PULLUP true [get_ports ads_sda]
