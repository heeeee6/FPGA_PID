open_checkpoint D:/FPGA/PID_swing/.sim_check/route_check.dcp
set src [get_pins -hier -filter {NAME =~ "*g_xadc*u_xadc4/ch0_reg*/C"}]
set dst [get_pins -hier -filter {NAME =~ "*u_angle_pipe*product_reg*/D"}]
puts "ANGLE_SRC=$src"
puts "ANGLE_DST=$dst"
if {[llength $src] && [llength $dst]} {
    report_timing -from $src -to $dst -max_paths 10 -file D:/FPGA/PID_swing/.sim_check/angle_pipe_path.rpt
}
close_design
