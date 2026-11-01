`timescale 1ns/1ps

// Regression for the restored STM32 Menu.c key map and runtime parameters.
module tb_stm32_menu_restore;
    reg clk = 1'b0;
    always #500 clk = ~clk;

    reg rst_n = 1'b0;
    reg sample_tick_1ms = 1'b0;
    reg capture_reset = 1'b0;
    reg [3:0] key_click = 4'b0000;
    reg [3:0] key_long = 4'b0000;
    reg [3:0] key_pressed = 4'b0000;
    reg [11:0] rp1_adc = 12'd0;
    reg [11:0] rp2_adc = 12'd0;
    reg [11:0] rp3_adc = 12'd0;
    reg [11:0] rp4_adc = 12'd0;

    wire run_request;
    wire [1:0] page;
    wire signed [31:0] location_target;
    wire [15:0] angle_kp_q8, angle_ki_q8, angle_kd_q8;
    wire [15:0] location_kp_q8, location_ki_q8, location_kd_q8;
    wire [11:0] center_angle;
    wire [7:0] start_pwm;
    wire [15:0] start_time_ms;
    wire [7:0] pwm_offset;
    wire [1:0] menu_index;
    wire menu_edit;

    pendulum_ui #(
        .ADC_SPAN_COUNTS(4095),
        .USE_RP4_TARGET(0)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .sample_tick_1ms(sample_tick_1ms),
        .capture_reset(capture_reset),
        .key_click(key_click), .key_long(key_long),
        .key_pressed(key_pressed),
        .rp1_adc(rp1_adc), .rp2_adc(rp2_adc),
        .rp3_adc(rp3_adc), .rp4_adc(rp4_adc),
        .run_request(run_request), .page(page),
        .location_target(location_target),
        .angle_kp_q8(angle_kp_q8),
        .angle_ki_q8(angle_ki_q8),
        .angle_kd_q8(angle_kd_q8),
        .location_kp_q8(location_kp_q8),
        .location_ki_q8(location_ki_q8),
        .location_kd_q8(location_kd_q8),
        .center_angle(center_angle),
        .start_pwm(start_pwm),
        .start_time_ms(start_time_ms),
        .pwm_offset(pwm_offset),
        .menu_index(menu_index), .menu_edit(menu_edit),
        .save_pulse(),
        .angle_kp_armed(), .angle_ki_armed(), .angle_kd_armed(),
        .location_kp_armed(), .location_ki_armed(), .location_kd_armed(),
        .location_target_armed()
    );

    task automatic click;
        input integer key;
        begin
            @(negedge clk);
            key_click[key] = 1'b1;
            @(negedge clk);
            key_click[key] = 1'b0;
            @(negedge clk);
        end
    endtask

    task automatic sample;
        begin
            @(negedge clk);
            sample_tick_1ms = 1'b1;
            @(negedge clk);
            sample_tick_1ms = 1'b0;
            @(negedge clk);
        end
    endtask

    task automatic long_press;
        input integer key;
        begin
            key_pressed[key] = 1'b1;
            @(negedge clk);
            key_long[key] = 1'b1;
            @(negedge clk);
            key_long[key] = 1'b0;
            repeat (60) sample();
            key_pressed[key] = 1'b0;
            sample();
        end
    endtask

    initial begin
        repeat (5) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;
        @(negedge clk);

        if (angle_kp_q8 !== 16'd82 || angle_ki_q8 !== 16'd3 ||
            angle_kd_q8 !== 16'd108 || location_kp_q8 !== 16'd102 ||
            location_ki_q8 !== 16'd0 || location_kd_q8 !== 16'd1024 ||
            center_angle !== 12'd2094 || start_pwm !== 8'd45 ||
            start_time_ms !== 16'd186)
            $fatal(1, "STM32-compatible defaults were not restored");

        // K4 cycles PAGE0 -> PAGE1 -> PAGE2 -> PAGE3.
        click(3);
        click(3);
        click(3);
        if (page !== 2'd3 || menu_index !== 2'd0)
            $fatal(1, "could not enter PAGE3");

        // PAGE3 browse mode: K1 directly starts and stops the pendulum.
        click(0);
        if (!run_request)
            $fatal(1, "K1 did not start the controller from PAGE3");
        click(0);
        if (run_request)
            $fatal(1, "K1 did not stop the controller from PAGE3");

        // K3 enters numeric editing; editing also enforces a motor stop.
        click(2);
        if (!menu_edit || run_request)
            $fatal(1, "PAGE3 edit mode or run interlock failed");

        click(0);
        if (center_angle !== 12'd2084)
            $fatal(1, "K1 short press did not decrease CENTER by 10");
        click(1);
        if (center_angle !== 12'd2094)
            $fatal(1, "K2 short press did not increase CENTER by 10");

        long_press(0);
        if (center_angle !== 12'd2093)
            $fatal(1, "long K1 did not repeat a one-count CENTER decrease");
        long_press(1);
        if (center_angle !== 12'd2094)
            $fatal(1, "long K2 did not repeat a one-count CENTER increase");

        click(2);
        if (menu_edit)
            $fatal(1, "K3 did not leave CENTER edit");

        // Select TIME and verify the same continuous adjustment path.
        click(1);
        click(1);
        if (menu_index !== 2'd2)
            $fatal(1, "did not select TIME");
        click(2);
        long_press(0);
        if (start_time_ms !== 16'd185)
            $fatal(1, "long K1 did not repeat a TIME decrease");
        long_press(1);
        if (start_time_ms !== 16'd186)
            $fatal(1, "long K2 did not repeat a TIME increase");
        click(2);

        // The edited values remain the runtime values, and K1 starts the
        // controller without leaving PAGE3.
        if (center_angle !== 12'd2094 || start_pwm !== 8'd45 ||
            start_time_ms !== 16'd186 || pwm_offset !== 8'd0)
            $fatal(1, "PAGE3 runtime parameters were not retained");
        click(0);
        if (!run_request)
            $fatal(1, "K1 did not start after PAGE3 editing");

        // Back to PAGE0, then K1 controls run as in Mode31.
        click(3);
        if (page !== 2'd0 || menu_edit)
            $fatal(1, "K4 did not return to PAGE0");
        click(0);
        if (!run_request)
            $fatal(1, "K1 did not request controller run from PAGE0");

        $display("tb_stm32_menu_restore: PASS");
        $finish;
    end

    initial begin
        #10_000_000;
        $fatal(1, "STM32 menu restore test timed out");
    end
endmodule
