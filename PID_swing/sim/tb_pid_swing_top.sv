`timescale 1ns/1ps

// Self-checking behavioral regression for the control path.  The DUT is run
// at 1 MHz here so its 1 kHz scheduler and the automatic swing-up state
// machine complete quickly; hardware uses the default 50 MHz parameter.
module tb_pid_swing_top;
    reg clk;
    reg rst_n;
    reg run_enable;
    reg [11:0] angle_adc_raw;
    reg [11:0] kp_adc_raw;
    reg [11:0] ki_adc_raw;
    reg [11:0] kd_adc_raw;
    reg motor_enc_a;
    reg motor_enc_b;
    reg motor_fault;
    wire [6:0] vauxp;
    wire [6:0] vauxn;
    wire oled_scl_low;
    wire oled_sda_low;

    wire motor_pwm;
    wire motor_in1;
    wire motor_in2;
    wire motor_stby;
    wire [1:0] mode;
    wire signed [15:0] pid_output;
    wire signed [31:0] motor_position;
    wire signed [31:0] motor_velocity;
    wire signed [15:0] angle_deg_x10;
    wire [15:0] kp_q8;
    wire [15:0] ki_q8;
    wire [15:0] kd_q8;

    assign vauxp = 7'b0;
    assign vauxn = 7'b0;

    always #500 clk = ~clk;

    pid_swing_top #(
        .CLK_HZ(1000000),
        .USE_XADC(0),
        .USE_POSITION_LOOP(1),
        .MOTOR_SIGN(1),
        .STARTUP_HOLD_MS(5),
        .START_TIME_MS(5),
        .COMMAND_SLEW_STEP(1)
    ) dut (
        .clk_50m(clk),
        .rst_n(rst_n),
        .run_enable(run_enable),
        .angle_adc_raw(angle_adc_raw),
        .angle2_adc_raw(12'd0),
        .kp_adc_raw(kp_adc_raw),
        .ki_adc_raw(ki_adc_raw),
        .kd_adc_raw(kd_adc_raw),
        .motor_enc_a(motor_enc_a),
        .motor_enc_b(motor_enc_b),
        .motor_fault(motor_fault),
        .key_n(4'hf),
        .vauxp(vauxp),
        .vauxn(vauxn),
        .oled_rst_n(1'b1),
        .motor_pwm(motor_pwm),
        .motor_in1(motor_in1),
        .motor_in2(motor_in2),
        .motor_stby(motor_stby),
        .oled_scl_low(oled_scl_low),
        .oled_sda_low(oled_sda_low),
        .oled_sda_in(1'b1),
        .ads_scl_low(), .ads_sda_low(), .ads_sda_in(1'b1),
        .mode(mode),
        .pid_output(pid_output),
        .motor_position(motor_position),
        .motor_velocity(motor_velocity),
        .angle_deg_x10(angle_deg_x10),
        .kp_q8(kp_q8),
        .ki_q8(ki_q8),
        .kd_q8(kd_q8)
    );

    // Direct map instance makes the angle endpoint checks independent of the
    // low-pass filter settling time in the complete top level.
    reg [11:0] map_adc;
    wire signed [15:0] map_angle_x10;
    angle_from_adc u_map_check (
        .adc(map_adc), .angle_deg_x10(map_angle_x10)
    );

    // Exhaustively compare the optimized gain mapper with the original
    // integer definitions over the complete 12-bit ADC range.
    reg [11:0] gain_adc;
    wire [15:0] gain_kp_q8;
    wire [15:0] gain_kp_x100;
    wire [15:0] gain_ki_q8;
    wire [15:0] gain_ki_x100;
    gain_from_adc #(.MAX_Q8(2048)) u_kp_map_check (
        .adc(gain_adc), .gain_q8(gain_kp_q8), .gain_x100(gain_kp_x100)
    );
    gain_from_adc #(.MAX_Q8(512)) u_ki_map_check (
        .adc(gain_adc), .gain_q8(gain_ki_q8), .gain_x100(gain_ki_x100)
    );

    // A short filter instance explicitly exercises a downward ADC step.
    reg filter_sample;
    reg [11:0] filter_raw;
    wire [11:0] filter_value;
    adc_filter_12 #(.FILTER_SHIFT(1)) u_filter_check (
        .clk(clk), .rst_n(rst_n), .sample_en(filter_sample),
        .raw(filter_raw), .filtered(filter_value)
    );

    reg enc_sample;
    wire signed [31:0] enc_position;
    wire signed [31:0] enc_velocity;
    quad_encoder u_encoder_check (
        .clk(clk), .rst_n(rst_n), .enc_a(motor_enc_a), .enc_b(motor_enc_b),
        .sample_en(enc_sample), .clear(1'b0),
        .position(enc_position), .velocity(enc_velocity)
    );

    reg pid_update;
    reg pid_enable;
    reg signed [15:0] pid_setpoint;
    reg signed [15:0] pid_measurement;
    reg [15:0] pid_kp;
    reg [15:0] pid_ki;
    reg [15:0] pid_kd;
    wire signed [15:0] check_pid_output;
    wire signed [31:0] check_pid_error;
    wire signed [31:0] check_pid_integral;
    pid_fixed #(.OUT_LIMIT(100), .INTEGRAL_LIMIT(1000)) u_pid_check (
        .clk(clk), .rst_n(rst_n), .clear(1'b0),
        .update(pid_update), .enable(pid_enable),
        .setpoint(pid_setpoint), .measurement(pid_measurement),
        .kp_q8(pid_kp), .ki_q8(pid_ki), .kd_q8(pid_kd),
        .output_value(check_pid_output), .error_value(check_pid_error),
        .integral_value(check_pid_integral)
    );

    reg swing_check_tick;
    reg swing_check_run;
    reg [11:0] swing_check_angle_adc;
    wire signed [15:0] swing_check_command;
    wire [1:0] swing_check_mode;
    wire swing_check_armed;
    wire swing_check_capture;
    swing_balance_ctrl #(.START_PWM(20), .START_TIME_MS(0),
                         .STARTUP_HOLD_MS(0), .COMMAND_SLEW_STEP(100)) u_swing_check (
        .clk(clk), .rst_n(rst_n), .tick_1khz(swing_check_tick),
        .run_enable(swing_check_run), .motor_fault(1'b0),
        .angle_adc_raw(swing_check_angle_adc),
        .angle2_adc_raw(12'd2050),
        .pid_output(16'sd0), .command(swing_check_command),
        .start_pwm_value(8'd20), .start_time_value(16'd0),
        .pwm_offset_value(8'd0),
        .mode(swing_check_mode), .balancing(), .armed(swing_check_armed),
        .capture_clear(swing_check_capture)
    );

    task automatic wait_cycles;
        input integer count;
        integer n;
        begin
            for (n = 0; n < count; n = n + 1)
                @(posedge clk);
        end
    endtask

    task automatic pid_step;
        begin
            pid_update = 1'b1;
            @(posedge clk);
            #1;
            pid_update = 1'b0;
            @(posedge clk);
            #1;
        end
    endtask

    task automatic filter_step;
        begin
            filter_sample = 1'b1;
            @(posedge clk);
            #1;
            filter_sample = 1'b0;
            @(posedge clk);
            #1;
        end
    endtask

    task automatic swing_check_step;
        begin
            @(negedge clk);
            swing_check_tick = 1'b1;
            @(posedge clk);
            #1;
            swing_check_tick = 1'b0;
            @(posedge clk);
            #1;
        end
    endtask

    // Advance the direct swing-controller instance through one 40 ms judge
    // sample while holding the requested absolute angle value.
    task automatic swing_judge_sample;
        input [11:0] angle_value;
        integer k;
        begin
            swing_check_angle_adc = angle_value;
            for (k = 0; k < 40; k = k + 1)
                swing_check_step;
        end
    endtask

    task automatic quadrature_state;
        input state_a;
        input state_b;
        begin
            motor_enc_a = state_a;
            motor_enc_b = state_b;
            wait_cycles(4);
        end
    endtask

    integer old_filter;
    integer gain_i;
    integer expected_kp_q8;
    integer expected_ki_q8;
    integer expected_kp_x100;
    integer expected_ki_x100;
    initial begin
        clk            = 1'b0;
        rst_n          = 1'b0;
        run_enable     = 1'b0;
        angle_adc_raw  = 12'd0;
        kp_adc_raw     = 12'd0;
        ki_adc_raw     = 12'd0;
        kd_adc_raw     = 12'd0;
        motor_enc_a    = 1'b0;
        motor_enc_b    = 1'b0;
        motor_fault    = 1'b0;
        filter_sample  = 1'b0;
        filter_raw     = 12'd0;
        enc_sample     = 1'b0;
        pid_update     = 1'b0;
        pid_enable     = 1'b0;
        pid_setpoint   = 16'sd0;
        pid_measurement= 16'sd0;
        pid_kp         = 16'd0;
        pid_ki         = 16'd0;
        pid_kd         = 16'd0;
        map_adc        = 12'd0;
        gain_adc       = 12'd0;
        swing_check_tick = 1'b0;
        swing_check_run = 1'b0;
        swing_check_angle_adc = 12'd0;

        wait_cycles(5);
        rst_n = 1'b1;
        #1;

        map_adc = 12'd0;
        #1;
        if (map_angle_x10 !== 16'sd0)
            $fatal(1, "angle 0 ADC did not map to 0 degrees: %0d", map_angle_x10);
        map_adc = 12'd1987;
        #1;
        if ((map_angle_x10 < 16'sd1799) || (map_angle_x10 > 16'sd1801))
            $fatal(1, "angle midpoint expected about 1800 tenths: %0d", map_angle_x10);
        map_adc = 12'd3974;
        #1;
        if (map_angle_x10 !== 16'sd0)
            $fatal(1, "angle divider full scale did not wrap to 0 degrees: %0d", map_angle_x10);
        map_adc = 12'd4095;
        #1;
        if (map_angle_x10 !== 16'sd0)
            $fatal(1, "angle ADC overrange did not clip to 0 degrees: %0d", map_angle_x10);

        for (gain_i = 0; gain_i <= 4095; gain_i = gain_i + 1) begin
            gain_adc = gain_i[11:0];
            #1;
            expected_kp_q8 = ((gain_i > 3974) ? 3974 : gain_i) * 2048 / 3974;
            expected_ki_q8 = ((gain_i > 3974) ? 3974 : gain_i) * 512  / 3974;
            expected_kp_x100 = expected_kp_q8 * 100 / 256;
            expected_ki_x100 = expected_ki_q8 * 100 / 256;
            if ((gain_kp_q8 !== expected_kp_q8[15:0]) ||
                (gain_kp_x100 !== expected_kp_x100[15:0]) ||
                (gain_ki_q8 !== expected_ki_q8[15:0]) ||
                (gain_ki_x100 !== expected_ki_x100[15:0]))
                $fatal(1, "gain map mismatch ADC=%0d: kp=%h/%h kp100=%h/%h ki=%h/%h ki100=%h/%h",
                       gain_i, gain_kp_q8, expected_kp_q8[15:0],
                       gain_kp_x100, expected_kp_x100[15:0],
                       gain_ki_q8, expected_ki_q8[15:0],
                       gain_ki_x100, expected_ki_x100[15:0]);
        end

        // The complete top must see either side of the ADC seam as zero,
        // without passing through an apparent 180-degree angle.
        angle_adc_raw = 12'd4095;
        #1;
        if (angle_deg_x10 !== 16'sd0)
            $fatal(1, "top angle at full scale was not zero: %0d", angle_deg_x10);
        angle_adc_raw = 12'd0;
        #1;
        if (angle_deg_x10 !== 16'sd0)
            $fatal(1, "top angle crossed false midpoint at ADC wrap: %0d", angle_deg_x10);

        // The complete top must traverse the 360/0-degree seam without a
        // linear ADC filter reporting an angle near 180 degrees.
        angle_adc_raw = 12'd3969;
        #1;
        if (angle_deg_x10 !== -16'sd5)
            $fatal(1, "top angle before wrap mismatch: %0d", angle_deg_x10);
        angle_adc_raw = 12'd5;
        #1;
        if (angle_deg_x10 !== 16'sd4)
            $fatal(1, "top angle after wrap mismatch: %0d", angle_deg_x10);
        angle_adc_raw = 12'd0;

        // A negative filter step must move down instead of wrapping upward.
        filter_raw = 12'd4095;
        filter_step;
        old_filter = filter_value;
        filter_raw = 12'd0;
        filter_step;
        if (filter_value >= old_filter)
            $fatal(1, "ADC filter wrapped on a negative step: old=%0d new=%0d",
                   old_filter, filter_value);

        // Proportional response, integral accumulation, and Ki=0 reset.
        pid_enable = 1'b1;
        pid_kp = 16'd256;
        pid_ki = 16'd0;
        pid_kd = 16'd0;
        pid_setpoint = 16'sd100;
        pid_measurement = 16'sd0;
        pid_step;
        if (check_pid_output !== 16'sd100)
            $fatal(1, "P response mismatch: %0d", check_pid_output);

        pid_setpoint = 16'sd250;
        pid_step;
        if (check_pid_output !== 16'sd100)
            $fatal(1, "positive PID saturation mismatch: %0d", check_pid_output);
        pid_setpoint = -16'sd250;
        pid_step;
        if (check_pid_output !== -16'sd100 || check_pid_error !== -32'sd250)
            $fatal(1, "negative PID saturation mismatch: output=%0d error=%0d",
                   check_pid_output, check_pid_error);
        pid_setpoint = -16'sd37;
        pid_step;
        if (check_pid_output !== -16'sd37)
            $fatal(1, "negative PID response mismatch: %0d", check_pid_output);

        pid_kp = 16'd0;
        pid_ki = 16'd256;
        pid_setpoint = 16'sd10;
        pid_measurement = 16'sd0;
        pid_step;
        if (check_pid_output !== 16'sd10)
            $fatal(1, "first I response mismatch: %0d", check_pid_output);
        pid_step;
        if (check_pid_output !== 16'sd20)
            $fatal(1, "second I response mismatch: %0d", check_pid_output);
        pid_ki = 16'd0;
        pid_step;
        if (check_pid_integral !== 32'sd0)
            $fatal(1, "integral did not clear when Ki=0: %0d", check_pid_integral);
        pid_enable = 1'b0;

        // 00 -> 01 -> 11 -> 10 -> 00 is one positive quadrature cycle.
        quadrature_state(1'b0, 1'b0);
        quadrature_state(1'b0, 1'b1);
        quadrature_state(1'b1, 1'b1);
        quadrature_state(1'b1, 1'b0);
        quadrature_state(1'b0, 1'b0);
        #1;
        if (enc_position !== 32'sd4)
            $fatal(1, "quadrature positive count mismatch: %0d", enc_position);
        enc_sample = 1'b1;
        @(posedge clk);
        #1;
        enc_sample = 1'b0;
        if (enc_velocity !== 32'sd4)
            $fatal(1, "quadrature velocity mismatch: %0d", enc_velocity);
        quadrature_state(1'b1, 1'b0);
        quadrature_state(1'b1, 1'b1);
        quadrature_state(1'b0, 1'b1);
        quadrature_state(1'b0, 1'b0);
        #1;
        if (enc_position !== 32'sd0)
            $fatal(1, "quadrature reverse count mismatch: %0d", enc_position);

        // --- STM32 Mode31 swing controller (absolute ADC coordinate) ---
        // A hanging pendulum near the ADC rails must still be allowed to
        // swing up. Only the balance state treats an out-of-range angle as
        // fallen, exactly like Mode31.c.
        swing_check_run = 1'b0;
        swing_check_angle_adc = 12'd200;
        wait_cycles(2);
        swing_check_run = 1'b1;
        swing_check_step;   // idle -> state 13
        if (swing_check_mode !== 2'd1)
            $fatal(1, "swing controller did not leave idle: %0d", swing_check_mode);
        // START_TIME_MS is zero, so states 13 and 14 complete over the next
        // two ticks and the state machine then holds a zero command in the
        // judge state.  One extra tick lets the command slew finish.
        swing_check_step;
        swing_check_step;
        swing_check_step;
        if (swing_check_mode !== 2'd1 || swing_check_command !== 16'sd0)
            $fatal(1, "judge state did not hold zero command: mode=%0d cmd=%0d",
                   swing_check_mode, swing_check_command);

        // Restart so the judge history and judge counter start from a known
        // state before testing capture.
        swing_check_run = 1'b0;
        wait_cycles(2);
        swing_check_angle_adc = 12'd2050;
        swing_check_run = 1'b1;
        swing_check_step;
        swing_check_step;
        swing_check_step;

        // The first in-window sample must not capture on its own.
        swing_judge_sample(12'd2050);
        if (swing_check_mode !== 2'd1)
            $fatal(1, "single in-window sample captured too early");

        // The second in-window sample captures the upright pendulum.  Drive
        // the last judge tick manually so the one-clock capture_clear pulse
        // can be observed before it clears.
        swing_check_angle_adc = 12'd2050;
        begin : capture_observe
            integer k;
            for (k = 0; k < 39; k = k + 1)
                swing_check_step;
            @(negedge clk);
            swing_check_tick = 1'b1;
            @(posedge clk);
            #1;
            swing_check_tick = 1'b0;
            if (swing_check_mode !== 2'd2 || !swing_check_capture)
                $fatal(1, "upright pair did not capture: mode=%0d capture=%0d",
                       swing_check_mode, swing_check_capture);
            @(posedge clk);
            #1;
        end

        // A fallen reading while balancing returns to idle, not to swing.
        swing_check_angle_adc = 12'd500;
        swing_check_step;
        if (swing_check_mode !== 2'd0)
            $fatal(1, "fallen pendulum did not stop the balance loop: %0d",
                   swing_check_mode);
        swing_check_run = 1'b0;
        wait_cycles(2);

        // Right peak: three samples beyond ANGLE_B with the middle one the
        // smallest selects the left impulse sequence (+START_PWM first).
        swing_check_angle_adc = 12'd2500;
        swing_check_run = 1'b1;
        swing_check_step;
        swing_check_step;
        swing_check_step;
        swing_judge_sample(12'd2500);
        swing_judge_sample(12'd2300);
        swing_judge_sample(12'd2500);
        swing_check_step;   // S_L1 sets the +START_PWM target
        swing_check_step;   // slew now reaches +START_PWM
        if (swing_check_mode !== 2'd1 || swing_check_command !== 16'sd20)
            $fatal(1, "right peak did not request left impulse: mode=%0d cmd=%0d",
                   swing_check_mode, swing_check_command);
        swing_check_run = 1'b0;
        wait_cycles(2);

        // Left peak: three samples below ANGLE_A with the middle one the
        // largest selects the right impulse sequence (-START_PWM first).
        swing_check_angle_adc = 12'd1000;
        swing_check_run = 1'b1;
        swing_check_step;
        swing_check_step;
        swing_check_step;
        swing_judge_sample(12'd1000);
        swing_judge_sample(12'd1300);
        swing_judge_sample(12'd1000);
        swing_check_step;
        swing_check_step;
        if (swing_check_mode !== 2'd1 || swing_check_command !== -16'sd20)
            $fatal(1, "left peak did not request right impulse: mode=%0d cmd=%0d",
                   swing_check_mode, swing_check_command);
        swing_check_run = 1'b0;
        wait_cycles(2);

        // --- Complete top-level startup/balance regression ---
        angle_adc_raw = 12'd200;
        run_enable = 1'b1;
        // The complete top must hold the bridge in standby during its
        // startup delay, even though run_enable is already asserted.
        wait_cycles(3000);
        #1;
        if (mode !== 2'd0 || motor_stby !== 1'b0 || motor_pwm !== 1'b0 ||
            motor_in1 !== 1'b0 || motor_in2 !== 1'b0)
            $fatal(1, "startup hold did not keep the motor safely off");
        wait_cycles(5000);
        #1;
        if (mode !== 2'd1)
            $fatal(1, "controller did not enter swing mode: %0d", mode);
        if ((dut.motor_command > 16'sd6) || (dut.motor_command < -16'sd6))
            $fatal(1, "startup command was not ramped: %0d", dut.motor_command);
        if (!motor_stby || ((motor_in1 !== 1'b1 || motor_in2 !== 1'b0) &&
                            (motor_in1 !== 1'b0 || motor_in2 !== 1'b1)))
            $fatal(1, "TB6612 swing direction/standby mismatch");
        angle_adc_raw = 12'd2050;
        wait_cycles(120000);
        #1;
        if (mode !== 2'd2)
            $fatal(1, "controller did not enter balance mode: %0d", mode);

        // Fallen during balance must return the controller to idle.
        angle_adc_raw = 12'd500;
        wait_cycles(1500);
        #1;
        if (mode !== 2'd0 || motor_stby !== 1'b0)
            $fatal(1, "fallen pendulum did not stop the top-level loop: %0d",
                   mode);

        motor_fault = 1'b1;
        wait_cycles(1200);
        #1;
        if (mode !== 2'd3 || motor_stby !== 1'b0)
            $fatal(1, "fault did not force safe motor state");

        $display("tb_pid_swing_top: PASS");
        $finish;
    end
endmodule
