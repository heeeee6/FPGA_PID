set root [file normalize [file dirname [info script]]]
set rtl_files [glob -nocomplain -directory [file join $root rtl] *.v]
set xdc_file [file join $root constr SmartZynq_V1_3B.xdc]
set out [file join $root results]
read_verilog -sv $rtl_files
read_xdc $xdc_file
synth_design -top pid_swing_hw_top -part xc7z020clg484-1
report_utilization -file [file join $out synth_util.rpt]
opt_design
place_design
report_timing_summary -file [file join $out placed_timing.rpt]
route_design
report_timing_summary -file [file join $out route_timing.rpt]
report_drc -file [file join $out route_drc.rpt]
puts "ANGLE_PIPE_FIX_ROUTE_PASS"
