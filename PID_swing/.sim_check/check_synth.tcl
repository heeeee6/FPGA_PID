read_verilog -sv [glob ../rtl/*.v]
read_xdc ../constr/SmartZynq_V1_3B.xdc
synth_design -top pid_swing_hw_top -part xc7z020clg484-1 -no_iobuf
puts "SYNTH_CHECK_PASS"
