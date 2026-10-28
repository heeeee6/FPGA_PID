set project_root D:/FPGA/PID_swing
set rtl_files [glob -directory $project_root/rtl *.v]
read_verilog -sv $rtl_files
read_xdc $project_root/constr/SmartZynq_V1_3B.xdc
synth_design -top pid_swing_hw_top -part xc7z020clg484-1
opt_design
place_design
route_design
report_drc -file route_drc.rpt
report_timing_summary -file route_timing.rpt
report_utilization -file route_util.rpt
write_checkpoint -force route_check.dcp
puts "ROUTE_CHECK_PASS"
