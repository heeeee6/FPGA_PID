# Generate the deployable bitstream from the fully routed checkpoint.
# Run from the project directory with Vivado 2018.3:
#   vivado -mode batch -source scripts/write_bitstream.tcl
set script_dir [file dirname [file normalize [info script]]]
set project_root [file normalize [file join $script_dir ..]]
set checkpoint [file join $project_root route_check.dcp]
set bitstream [file join $project_root pid_swing_hw_top.bit]

if {![file exists $checkpoint]} {
    error "Missing routed checkpoint: $checkpoint"
}

open_checkpoint $checkpoint
write_bitstream -force $bitstream
puts "BITSTREAM_PASS: $bitstream"
close_design
