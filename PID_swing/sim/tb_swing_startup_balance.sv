`timescale 1ns/1ps

// An initially upright pendulum must enter balance without issuing a
// swing-up kick.  A pendant start must still fall back to the normal kick.
module tb_swing_startup_balance;
    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg rst_n = 1'b0;
    reg tick = 1'b0;
    reg run_enable = 1'b0;
    reg [11:0] angle = 12'd2050;
    reg [11:0] center = 12'd2067;
    reg signed [15:0] pid_output = 16'sd12;
    wire signed [15:0] command;
    wire [1:0] mode;
    wire balancing, armed, capture_clear;
    reg capture_seen = 1'b0;

    swing_balance_ctrl #(
        .START_PWM(45),
        .START_TIME_MS(100),
        .STARTUP_HOLD_MS(2),
        .COMMAND_SLEW_STEP(1),
        .DIRECT_BALANCE_COMMAND(1),
        .RUNTIME_PARAMS(1)
    ) dut (
        .clk(clk), .rst_n(rst_n), .tick_1khz(tick),
        .run_enable(run_enable), .motor_fault(1'b0),
        .angle_adc_raw(angle), .angle2_adc_raw(12'd0),
        .center_angle(center),
        .pid_output(pid_output),
        .start_pwm_value(8'd45), .start_time_value(16'd100),
        .pwm_offset_value(8'd0),
        .command(command), .mode(mode), .balancing(balancing),
        .armed(armed), .capture_clear(capture_clear)
    );

    always @(posedge clk)
        if (capture_clear)
            capture_seen <= 1'b1;

    task automatic do_reset;
        begin
            @(negedge clk);
            rst_n = 1'b0;
            run_enable = 1'b0;
            repeat (3) @(negedge clk);
            rst_n = 1'b1;
            repeat (2) @(negedge clk);
        end
    endtask

    task automatic control_tick;
        begin
            @(negedge clk);
            tick = 1'b1;
            @(negedge clk);
            tick = 1'b0;
            @(negedge clk);
        end
    endtask

    initial begin
        do_reset();
        run_enable = 1'b1;
        repeat (4 + 40 + 2) control_tick();

        if (mode !== 2'd2 || !balancing || !armed || !capture_seen)
            $fatal(1, "upright startup did not enter balance directly");
        if (command !== 16'sd12)
            $fatal(1, "balance command did not bypass the swing slew");

        // The capture window must follow CENTER rather than the former fixed
        // 1700..2300 absolute limits.
        do_reset();
        center = 12'd2300;
        angle = 12'd2450;
        capture_seen = 1'b0;
        run_enable = 1'b1;
        repeat (4 + 40 + 2) control_tick();
        if (mode !== 2'd2 || !balancing || !capture_seen)
            $fatal(1, "centered capture window did not follow CENTER");

        // A hanging start must not be mistaken for an upright start.
        do_reset();
        center = 12'd2067;
        angle = 12'd200;
        capture_seen = 1'b0;
        run_enable = 1'b1;
        repeat (4 + 40 + 1) control_tick();
        if (mode !== 2'd1 || balancing || capture_seen)
            $fatal(1, "hanging startup did not return to swing mode");

        $display("tb_swing_startup_balance: PASS");
        $finish;
    end

    initial begin
        #1_000_000;
        $fatal(1, "startup balance test timed out");
    end
endmodule
