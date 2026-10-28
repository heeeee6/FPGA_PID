# Vivado 2018.3 batch implementation check for the hardware top level.
set script_dir [file dirname [file normalize [info script]]]
set project_root [file normalize [file join $script_dir ..]]
set rtl_files [glob -nocomplain -directory [file join $project_root rtl] *.v]
set xdc_file [file join $project_root constr SmartZynq_V1_3B.xdc]

if {[llength $rtl_files] == 0} {error "No RTL files found"}
if {![file exists $xdc_file]} {error "Missing XDC file: $xdc_file"}

read_verilog -sv $rtl_files
read_xdc $xdc_file
synth_design -top pid_swing_hw_top -part xc7z020clg484-1
opt_design
place_design
route_design
report_drc -file route_drc.rpt
report_timing_summary -file route_timing.rpt
write_checkpoint -force route_check.dcp
puts "ROUTE_CHECK_PASS"
