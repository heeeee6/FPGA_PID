`timescale 1ns/1ps

// Zynq-7020 PL top level for the rotary inverted pendulum.
// Default USE_XADC=1 selects the FPGA's internal XADC and four VAUX pairs.
// USE_XADC=0 accepts four digital 12-bit samples for control-path simulation.
module pid_swing_top #(
    parameter integer CLK_HZ = 50000000,
    parameter integer USE_XADC = 1,
    parameter integer ADC_SPAN_COUNTS = 3974,
    parameter integer USE_POSITION_LOOP = 1,
    parameter integer MOTOR_SIGN = 1,
    parameter integer ANGLE_SIGN = 1,
    parameter integer ANGLE_TARGET_LIMIT_X10 = 600,
    parameter integer POSITION_ANGLE_SCALE_X10 = 2
) (
    input  wire               clk_50m,
    input  wire               rst_n,
    input  wire               run_enable,
    input  wire [11:0]        angle_adc_raw,
    input  wire [11:0]        kp_adc_raw,
    input  wire [11:0]        ki_adc_raw,
    input  wire [11:0]        kd_adc_raw,
    input  wire               motor_enc_a,
    input  wire               motor_enc_b,
    input  wire               motor_fault,
    input  wire [3:0]         vauxp,
    input  wire [3:0]         vauxn,
    output wire               motor_pwm,
    output wire               motor_in1,
    output wire               motor_in2,
    output wire               motor_stby,
    output wire               oled_scl_low,
    output wire               oled_sda_low,
    input  wire               oled_sda_in,
    output wire [1:0]         mode,
    output wire signed [15:0] pid_output,
    output wire signed [31:0] motor_position,
    output wire signed [31:0] motor_velocity,
    output wire signed [15:0] angle_deg_x10,
    output wire [15:0]        kp_q8,
    output wire [15:0]        ki_q8,
    output wire [15:0]        kd_q8
);
    localparam integer CONTROL_DIV = CLK_HZ / 1000;

    reg [31:0] control_count;
    reg [5:0]  outer_count;
    reg        control_tick;
    reg        outer_tick;

    wire [11:0] angle_adc_source;
    wire [11:0] kp_adc_source;
    wire [11:0] ki_adc_source;
    wire [11:0] kd_adc_source;
    wire        xadc_sample_valid;
    wire        adc_ready;
    wire        control_run_enable = run_enable && adc_ready;

    generate
        if (USE_XADC != 0) begin : g_xadc
            reg scan_seen;
            always @(posedge clk_50m) begin
                if (!rst_n)
                    scan_seen <= 1'b0;
                else if (xadc_sample_valid)
                    scan_seen <= 1'b1;
            end
            assign adc_ready = scan_seen;
            xadc4_reader u_xadc4 (
                .clk(clk_50m), .rst_n(rst_n), .vauxp(vauxp), .vauxn(vauxn),
                .ch0(angle_adc_source), .ch1(kp_adc_source),
                .ch2(ki_adc_source), .ch3(kd_adc_source),
                .sample_valid(xadc_sample_valid)
            );
        end else begin : g_digital_adc
            assign angle_adc_source = angle_adc_raw;
            assign kp_adc_source    = kp_adc_raw;
            assign ki_adc_source    = ki_adc_raw;
            assign kd_adc_source    = kd_adc_raw;
            assign xadc_sample_valid= 1'b0;
            assign adc_ready       = 1'b1;
        end
    endgenerate

    // 1 kHz control scheduler and 20 Hz outer position loop tick.
    always @(posedge clk_50m) begin
        if (!rst_n) begin
            control_count <= 32'd0;
            outer_count   <= 6'd0;
            control_tick  <= 1'b0;
            outer_tick    <= 1'b0;
        end else begin
            control_tick <= 1'b0;
            outer_tick   <= 1'b0;
            if (control_count >= CONTROL_DIV - 1) begin
                control_count <= 32'd0;
                control_tick  <= 1'b1;
                if (outer_count >= 6'd49) begin
                    outer_count <= 6'd0;
                    outer_tick  <= 1'b1;
                end else begin
                    outer_count <= outer_count + 1'b1;
                end
            end else begin
                control_count <= control_count + 1'b1;
            end
        end
    end

    wire [11:0] kp_adc_f;
    wire [11:0] ki_adc_f;
    wire [11:0] kd_adc_f;

    // The mechanical angle wraps at 0/360 degrees. Filtering raw ADC counts
    // linearly across 4095->0 would create a false ~180-degree measurement.
    // Map the latest complete XADC conversion directly; gain knobs are still
    // low-pass filtered below.
    adc_filter_12 u_kp_filter (
        .clk(clk_50m), .rst_n(rst_n), .sample_en(control_tick),
        .raw(kp_adc_source), .filtered(kp_adc_f)
    );
    adc_filter_12 u_ki_filter (
        .clk(clk_50m), .rst_n(rst_n), .sample_en(control_tick),
        .raw(ki_adc_source), .filtered(ki_adc_f)
    );
    adc_filter_12 u_kd_filter (
        .clk(clk_50m), .rst_n(rst_n), .sample_en(control_tick),
        .raw(kd_adc_source), .filtered(kd_adc_f)
    );

    wire signed [15:0] mapped_angle_deg_x10;
    wire signed [15:0] piped_angle_deg_x10;
    angle_from_adc #(.ADC_SPAN_COUNTS(ADC_SPAN_COUNTS)) u_angle_map (
        .adc(angle_adc_source), .angle_deg_x10(mapped_angle_deg_x10)
    );
    angle_from_adc_pipe #(.ADC_SPAN_COUNTS(ADC_SPAN_COUNTS)) u_angle_pipe (
        .clk(clk_50m), .rst_n(rst_n), .update(control_tick),
        .adc(angle_adc_source), .angle_deg_x10(piped_angle_deg_x10)
    );
    generate
        if (ANGLE_SIGN >= 0) begin : g_angle_sign_positive
            assign angle_deg_x10 = mapped_angle_deg_x10;
        end else begin : g_angle_sign_negative
            assign angle_deg_x10 = -mapped_angle_deg_x10;
        end
    endgenerate
    reg signed [15:0] control_angle_deg_x10;
    reg [15:0] control_kp_q8;
    reg [15:0] control_ki_q8;
    reg [15:0] control_kd_q8;
    wire [15:0] mapped_kp_q8;
    wire [15:0] mapped_ki_q8;
    wire [15:0] mapped_kd_q8;
    gain_from_adc #(.MAX_Q8(2048), .ADC_SPAN_COUNTS(ADC_SPAN_COUNTS)) u_kp_map (
        .adc(kp_adc_f), .gain_q8(mapped_kp_q8), .gain_x100()
    );
    gain_from_adc #(.MAX_Q8(512), .ADC_SPAN_COUNTS(ADC_SPAN_COUNTS)) u_ki_map (
        .adc(ki_adc_f), .gain_q8(mapped_ki_q8), .gain_x100()
    );
    gain_from_adc #(.MAX_Q8(2048), .ADC_SPAN_COUNTS(ADC_SPAN_COUNTS)) u_kd_map (
        .adc(kd_adc_f), .gain_q8(mapped_kd_q8), .gain_x100()
    );
    assign kp_q8 = mapped_kp_q8;
    assign ki_q8 = mapped_ki_q8;
    assign kd_q8 = mapped_kd_q8;

    // Sample the nonlinear ADC mappings at the 1 kHz control boundary.  The
    // live mapped outputs remain available for observation, while the PID
    // datapath starts from registers and does not include the ADC arithmetic.
    always @(posedge clk_50m) begin
        if (!rst_n) begin
            control_angle_deg_x10 <= 16'sd0;
            control_kp_q8 <= 16'd0;
            control_ki_q8 <= 16'd0;
            control_kd_q8 <= 16'd0;
        end else if (control_tick) begin
            control_angle_deg_x10 <= (ANGLE_SIGN >= 0) ? piped_angle_deg_x10 : -piped_angle_deg_x10;
            control_kp_q8 <= mapped_kp_q8;
            control_ki_q8 <= mapped_ki_q8;
            control_kd_q8 <= mapped_kd_q8;
        end
    end

    quad_encoder u_encoder (
        .clk(clk_50m), .rst_n(rst_n), .enc_a(motor_enc_a), .enc_b(motor_enc_b),
        .sample_en(control_tick), .position(motor_position),
        .velocity(motor_velocity)
    );

    // The optional outer loop keeps the motor around its zero position. Its
    // output is converted to a small angle setpoint correction for the inner
    // angle loop, matching the cascade in PID_library.
    wire signed [15:0] outer_output;
    wire signed [15:0] position_measurement = motor_position[15:0];
    pid_fixed #(.OUT_LIMIT(100), .INTEGRAL_LIMIT(20000)) u_position_pid (
        .clk(clk_50m), .rst_n(rst_n), .update(outer_tick),
        .enable(control_run_enable && (mode == 2'd2) && (USE_POSITION_LOOP != 0)),
        .setpoint(16'sd0), .measurement(position_measurement),
        .kp_q8(16'd102), .ki_q8(16'd0), .kd_q8(16'd1024),
        .output_value(outer_output), .error_value(), .integral_value()
    );

    reg signed [15:0] angle_setpoint;
    integer target_calc;
    always @* begin
        if (USE_POSITION_LOOP != 0)
            target_calc = -outer_output * POSITION_ANGLE_SCALE_X10;
        else
            target_calc = 0;
        if (target_calc > ANGLE_TARGET_LIMIT_X10)
            target_calc = ANGLE_TARGET_LIMIT_X10;
        else if (target_calc < -ANGLE_TARGET_LIMIT_X10)
            target_calc = -ANGLE_TARGET_LIMIT_X10;
        angle_setpoint = target_calc;
    end

    pid_fixed #(.OUT_LIMIT(100), .INTEGRAL_LIMIT(20000)) u_angle_pid (
        .clk(clk_50m), .rst_n(rst_n), .update(control_tick),
        .enable(control_run_enable && (mode == 2'd2)), .setpoint(angle_setpoint),
        .measurement(control_angle_deg_x10), .kp_q8(control_kp_q8),
        .ki_q8(control_ki_q8), .kd_q8(control_kd_q8),
        .output_value(pid_output), .error_value(), .integral_value()
    );

    wire signed [15:0] controller_command;
    wire signed [15:0] motor_command;
    swing_balance_ctrl u_swing_ctrl (
        .clk(clk_50m), .rst_n(rst_n), .tick_1khz(control_tick),
        .run_enable(control_run_enable), .motor_fault(motor_fault),
        .angle_deg_x10(control_angle_deg_x10), .motor_velocity(motor_velocity),
        .pid_output(pid_output), .command(controller_command),
        .mode(mode), .balancing()
    );

    generate
        if (MOTOR_SIGN >= 0) begin : g_motor_sign_positive
            assign motor_command = controller_command;
        end else begin : g_motor_sign_negative
            assign motor_command = -controller_command;
        end
    endgenerate

    tb6612_pwm #(.CLK_HZ(CLK_HZ), .PWM_HZ(20000)) u_motor_pwm (
        .clk(clk_50m), .rst_n(rst_n), .enable(control_run_enable), .fault(motor_fault),
        .command(motor_command), .pwm(motor_pwm), .in1(motor_in1),
        .in2(motor_in2), .stby(motor_stby)
    );

    wire [15:0] kp_x100_raw;
    wire [15:0] ki_x100_raw;
    wire [15:0] kd_x100_raw;
    reg  [15:0] kp_x100;
    reg  [15:0] ki_x100;
    reg  [15:0] kd_x100;
    gain_from_adc #(.MAX_Q8(2048), .ADC_SPAN_COUNTS(ADC_SPAN_COUNTS)) u_kp_display (
        .adc(kp_adc_f), .gain_q8(), .gain_x100(kp_x100_raw)
    );
    gain_from_adc #(.MAX_Q8(512), .ADC_SPAN_COUNTS(ADC_SPAN_COUNTS)) u_ki_display (
        .adc(ki_adc_f), .gain_q8(), .gain_x100(ki_x100_raw)
    );
    gain_from_adc #(.MAX_Q8(2048), .ADC_SPAN_COUNTS(ADC_SPAN_COUNTS)) u_kd_display (
        .adc(kd_adc_f), .gain_q8(), .gain_x100(kd_x100_raw)
    );

    // The OLED formatter contains integer decimal divisions.  Sample its
    // inputs at the control rate so that this slow display path cannot sit
    // in the same 50 MHz timing path as the live gain calculations.
    always @(posedge clk_50m) begin
        if (!rst_n) begin
            kp_x100 <= 16'd0;
            ki_x100 <= 16'd0;
            kd_x100 <= 16'd0;
        end else if (control_tick) begin
            kp_x100 <= kp_x100_raw;
            ki_x100 <= ki_x100_raw;
            kd_x100 <= kd_x100_raw;
        end
    end

    oled_ssd1306 u_oled (
        .clk(clk_50m), .rst_n(rst_n), .kp_x100(kp_x100), .ki_x100(ki_x100),
        .kd_x100(kd_x100), .mode(mode),
        .oled_scl_low(oled_scl_low), .oled_sda_low(oled_sda_low),
        .oled_sda_in(oled_sda_in)
    );
endmodule

