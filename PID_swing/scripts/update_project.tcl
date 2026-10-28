# Add the implementation and simulation sources to the Vivado 2018.3 project.
# Run with: vivado -mode batch -source scripts/update_project.tcl
set script_dir [file dirname [file normalize [info script]]]
set project_root [file normalize [file join $script_dir ..]]
set project_file [file join $project_root PID_swing.xpr]
set rtl_files [glob -nocomplain -directory [file join $project_root rtl] *.v]
set sim_files [glob -nocomplain -directory [file join $project_root sim] *.sv]
set xdc_file [file join $project_root constr SmartZynq_V1_3B.xdc]

if {[llength $rtl_files] == 0} {error "No RTL files found in $project_root/rtl"}
if {[llength $sim_files] == 0} {error "No simulation files found in $project_root/sim"}
foreach required_file [list $project_file $xdc_file] {
    if {![file exists $required_file]} {error "Missing project file: $required_file"}
}

open_project $project_file
foreach rtl_file $rtl_files {
    if {[llength [get_files -quiet $rtl_file]] == 0} {
        add_files -fileset sources_1 $rtl_file
    }
}
foreach sim_file $sim_files {
    if {[llength [get_files -quiet $sim_file]] == 0} {
        add_files -fileset sim_1 $sim_file
    }
}
if {[llength [get_files -quiet $xdc_file]] == 0} {
    add_files -fileset constrs_1 $xdc_file
}
set_property top pid_swing_hw_top [get_filesets sources_1]
set_property top tb_pid_swing_top [get_filesets sim_1]
update_compile_order -fileset sources_1
update_compile_order -fileset sim_1
puts "PROJECT_UPDATE_PASS: [llength $rtl_files] RTL files, [llength $sim_files] simulation files, and XDC registered"
close_project
