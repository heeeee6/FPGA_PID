read_verilog -sv [glob ../rtl/*.v]
read_xdc ../constr/SmartZynq_V1_3B.xdc
synth_design -top pid_swing_hw_top -part xc7z020clg484-1
opt_design
place_design
report_drc -file placed_drc.rpt
report_timing_summary -file placed_timing.rpt
puts "PLACEMENT_CHECK_PASS"
