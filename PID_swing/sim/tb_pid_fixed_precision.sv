`timescale 1ns/1ps

// The Q8 PID must retain fractional P/D terms until the final output
// truncation, matching the integer-domain reference-controller structure.
module tb_pid_fixed_precision;
    reg clk = 1'b0;
    always #5 clk = ~clk; // 100 MHz

    reg rst_n = 1'b0;
    reg clear = 1'b1;
    reg update = 1'b0;
    reg enable = 1'b1;
    reg signed [15:0] setpoint = 16'sd0;
    reg signed [15:0] measurement = 16'sd0;
    reg [15:0] kp_q8 = 16'd0;
    reg [15:0] ki_q8 = 16'd0;
    reg [15:0] kd_q8 = 16'd0;
    wire signed [15:0] output_value;

    pid_fixed #(.OUT_LIMIT(100), .INTEGRAL_LIMIT(20000),
                .PIPELINE_ERROR(0)) dut (
        .clk(clk), .rst_n(rst_n), .clear(clear), .update(update),
        .enable(enable), .setpoint(setpoint), .measurement(measurement),
        .kp_q8(kp_q8), .ki_q8(ki_q8), .kd_q8(kd_q8),
        .output_value(output_value), .error_value(), .integral_value()
    );

    task automatic reset_dut;
        begin
            @(negedge clk);
            rst_n = 1'b0;
            clear = 1'b1;
            update = 1'b0;
            repeat (2) @(negedge clk);
            rst_n = 1'b1;
            @(negedge clk);
            clear = 1'b0;
            @(negedge clk);
        end
    endtask

    task automatic step_pid;
        input signed [15:0] error_counts;
        begin
            @(negedge clk);
            setpoint = 16'sd0;
            measurement = -error_counts;
            update = 1'b1;
            @(negedge clk);
            update = 1'b0;
            @(negedge clk);
        end
    endtask

    initial begin
        repeat (3) @(posedge clk);
        reset_dut();

        // Kp=0.25: a one-count positive/negative error truncates to zero.
        kp_q8 = 16'd64;
        step_pid(16'sd1);
        if (output_value !== 16'sd0)
            $fatal(1, "positive Kp fraction was not preserved/truncated to zero");
        step_pid(-16'sd1);
        if (output_value !== 16'sd0)
            $fatal(1, "negative Kp result must truncate toward zero symmetrically");
        step_pid(16'sd4);
        if (output_value !== 16'sd1)
            $fatal(1, "Kp 0.25*4 must produce +1");

        // Kd=0.42: retain the numerator until final division.
        reset_dut();
        kp_q8 = 16'd0;
        ki_q8 = 16'd0;
        kd_q8 = 16'd108;
        step_pid(16'sd1);  // delta +1 -> 0
        if (output_value !== 16'sd0) $fatal(1, "Kd delta+1");
        step_pid(16'sd3);  // delta +2 -> 0
        if (output_value !== 16'sd0) $fatal(1, "Kd delta+2");
        step_pid(16'sd6);  // delta +3 -> +1
        if (output_value !== 16'sd1) $fatal(1, "Kd delta+3");
        step_pid(16'sd4);  // delta -2 -> 0
        if (output_value !== 16'sd0) $fatal(1, "Kd negative truncation");
        step_pid(16'sd1);  // delta -3 -> -1
        if (output_value !== -16'sd1) $fatal(1, "Kd negative delta-3");

        $display("tb_pid_fixed_precision: PASS");
        $finish;
    end

    initial begin
        #1_000_000;
        $fatal(1, "PID precision test timed out");
    end
endmodule
