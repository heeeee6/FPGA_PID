open_checkpoint D:/FPGA/PID_swing/.sim_check/route_check.dcp
puts "CELLS=[get_cells -hier *angle_pipe*]"
puts "PINS=[get_pins -hier -filter {NAME =~ "*angle_pipe*"}]"
close_design
