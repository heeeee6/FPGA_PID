`timescale 1ns/1ps

// Verify that PWM magnitude and direction only change at a PWM period
// boundary, matching the reference FPGA controller's pwm_gen behavior.
module tb_tb6612_pwm_latched;
    reg clk = 1'b0;
    always #50 clk = ~clk; // 10 MHz, 100 kHz PWM -> 100 clock periods

    reg rst_n = 1'b0;
    reg enable = 1'b1;
    reg fault = 1'b0;
    reg signed [15:0] command = 16'sd0;
    wire pwm, in1, in2, stby;

    tb6612_pwm #(.CLK_HZ(10_000_000), .PWM_HZ(100_000)) dut (
        .clk(clk), .rst_n(rst_n), .enable(enable), .fault(fault),
        .command(command), .pwm(pwm), .in1(in1), .in2(in2), .stby(stby)
    );

    initial begin
        repeat (3) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;
        command = 16'sd10;

        repeat (105) @(posedge clk);
        #1;
        if (dut.duty_latched !== 32'd10 || dut.neg_latched !== 1'b0 ||
            !stby)
            $fatal(1, "first PWM period was not latched correctly");

        // Change direction in the middle of a period.  The current period
        // must retain the prior direction and duty.
        @(negedge clk);
        command = -16'sd20;
        while (dut.pwm_count < 32'd50) @(posedge clk);
        #1;
        if (dut.duty_latched !== 32'd10 || dut.neg_latched !== 1'b0 ||
            in1 !== 1'b0 || in2 !== 1'b1)
            $fatal(1, "command change was applied inside a PWM period");

        repeat (55) @(posedge clk);
        #1;
        if (dut.duty_latched !== 32'd20 || dut.neg_latched !== 1'b1 ||
            in1 !== 1'b1 || in2 !== 1'b0)
            $fatal(1, "new command was not latched at the next boundary");

        // Equal magnitudes in both directions must produce identical duty.
        @(negedge clk);
        command = 16'sd25;
        repeat (105) @(posedge clk);
        #1;
        if (dut.duty_latched !== 32'd25 || dut.neg_latched !== 1'b0)
            $fatal(1, "positive direction duty is not symmetric");
        @(negedge clk);
        command = -16'sd25;
        repeat (105) @(posedge clk);
        #1;
        if (dut.duty_latched !== 32'd25 || dut.neg_latched !== 1'b1)
            $fatal(1, "negative direction duty is not symmetric");

        @(negedge clk);
        enable = 1'b0;
        repeat (2) @(posedge clk);
        #1;
        if (pwm || in1 || in2 || stby)
            $fatal(1, "disabled bridge did not enter the safe output state");

        $display("tb_tb6612_pwm_latched: PASS");
        $finish;
    end

    initial begin
        #1_000_000;
        $fatal(1, "TB6612 PWM latch test timed out");
    end
endmodule
