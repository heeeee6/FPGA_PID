`timescale 1ns/1ps

// Zynq-7020 PL top level for the rotary inverted pendulum.
// Default USE_XADC=1 selects the FPGA's internal XADC.  The physical V1.3B
// mapping is angle1=VAUX0, optional angle2=VAUX1, RP1..RP3=VAUX2..VAUX4 and
// RP4=VAUX6 (VAUX5 is not routed on the J5 header).  The second angle input is
// kept optional so an unpopulated connector cannot inhibit the motor.
// USE_XADC=0 accepts digital 12-bit samples for control-path simulation.
module pid_swing_top #(
    parameter integer CLK_HZ = 50000000,
    parameter integer USE_XADC = 1,
    parameter integer ADC_SPAN_COUNTS = 3974,
    parameter integer USE_POSITION_LOOP = 1,
    parameter integer MOTOR_SIGN = 1,
    parameter integer ANGLE_SIGN = 1,
    parameter integer ANGLE_TARGET_LIMIT_X10 = 600,
    parameter integer POSITION_ANGLE_SCALE_X10 = 2,
    parameter integer STARTUP_HOLD_MS = 3000,
    parameter integer START_TIME_MS = 90,
    parameter integer COMMAND_SLEW_STEP = 1,
    parameter integer START_PWM = 20,
    parameter integer MOTOR_COMMAND_LIMIT = 35,
    // Set to a positive value only for accelerated simulation.  The physical
    // build keeps the default and therefore waits 100 ms after configuration.
    parameter integer OLED_STARTUP_CYCLES = 0,
    // By default the display follows the controller reset for compatibility
    // with the simulation top level.  The physical top can select a separate
    // power-on reset so an optional external motor inhibit cannot hold the
    // SSD1306 in reset.
    parameter integer OLED_RESET_INDEPENDENT = 0
    ,parameter integer USE_STM32_UI = 0
    ,parameter integer USE_RP4_TARGET = 1
    // Set to one when both PCB angle sensors are fitted and should be
    // included in the STM32-style capture/fallen checks.
    ,parameter integer USE_SECOND_ANGLE = 0
    // USE_ADS1115=1 reads the angle sensor and RP1..RP4 from two ADS1115
    // converters (0x48 angle, 0x49 pots) instead of the XADC.  Their I2C bus
    // is separate from the OLED bus.  The 12-bit codes keep the STM32 scale.
    ,parameter integer USE_ADS1115 = 0
) (
    input  wire               clk_50m,
    input  wire               rst_n,
    input  wire               run_enable,
    input  wire [11:0]        angle_adc_raw,
    input  wire [11:0]        kp_adc_raw,
    input  wire [11:0]        ki_adc_raw,
    input  wire [11:0]        kd_adc_raw,
    input  wire [11:0]        angle2_adc_raw,
    input  wire [11:0]        rp4_adc_raw,
    input  wire               motor_enc_a,
    input  wire               motor_enc_b,
    input  wire               motor_fault,
    // Indexed by VAUX number: bits 0..4 and bit 6 are used; bit 5 is spare.
    input  wire [6:0]         vauxp,
    input  wire [6:0]         vauxn,
    input  wire [3:0]         key_n,
    // Used only when OLED_RESET_INDEPENDENT is non-zero.  Leaving this port
    // unconnected is safe for existing instantiations because the default
    // parameter selects rst_n instead.
    input  wire               oled_rst_n,
    output wire               motor_pwm,
    output wire               motor_in1,
    output wire               motor_in2,
    output wire               motor_stby,
    output wire               oled_scl_low,
    output wire               oled_sda_low,
    input  wire               oled_sda_in,
    output wire               ads_scl_low,
    output wire               ads_sda_low,
    input  wire               ads_sda_in,
    output wire [1:0]         mode,
    output wire signed [15:0] pid_output,
    output wire signed [31:0] motor_position,
    output wire signed [31:0] motor_velocity,
    output wire signed [15:0] angle_deg_x10,
    output wire [15:0]        kp_q8,
    output wire [15:0]        ki_q8,
    output wire [15:0]        kd_q8
    ,output wire [11:0]       angle_adc_counts
    ,output wire signed [15:0] angle_error_counts
    ,output wire signed [31:0] location_target
    ,output wire [1:0]        ui_page
    ,output wire              run_request
    ,output wire signed [15:0] angle_setpoint_out
    ,output wire signed [15:0] location_pid_output
);
    localparam integer CONTROL_DIV = CLK_HZ / 1000;

    reg [31:0] control_count;
    reg [5:0]  outer_count;
    reg [2:0]  inner_count;
    reg        control_tick;
    reg        outer_tick;
    reg        inner_tick;

    wire [11:0] angle_adc_source;
    wire [11:0] angle2_adc_source;
    wire [11:0] kp_adc_source;
    wire [11:0] ki_adc_source;
    wire [11:0] kd_adc_source;
    wire [11:0] rp4_adc_source;
    wire        xadc_sample_valid;
    wire        adc_ready;
    wire        ui_run_request;
    wire        control_run_enable = run_enable && adc_ready &&
                                     ((USE_STM32_UI != 0) ? ui_run_request : 1'b1);

    generate
        if (USE_ADS1115 != 0) begin : g_ads
            wire ads_online;
            wire ads_sample_valid;
            ads1115_reader #(.CLK_HZ(CLK_HZ)) u_ads (
                .clk(clk_50m), .rst_n(rst_n),
                .angle1(angle_adc_source),
                .rp1(kp_adc_source), .rp2(ki_adc_source),
                .rp3(kd_adc_source), .rp4(rp4_adc_source),
                .sample_valid(ads_sample_valid), .online(ads_online),
                .scl_low(ads_scl_low), .sda_low(ads_sda_low),
                .sda_in(ads_sda_in)
            );
            assign angle2_adc_source = 12'd0;
            assign xadc_sample_valid = ads_sample_valid;
            // The motor stays inhibited while either ADS1115 is missing or
            // the bus reports a NACK.
            assign adc_ready = ads_online;
        end else if (USE_XADC != 0) begin : g_xadc
            assign ads_scl_low = 1'b0;
            assign ads_sda_low = 1'b0;
            reg scan_seen;
            always @(posedge clk_50m) begin
                if (!rst_n)
                    scan_seen <= 1'b0;
                else if (xadc_sample_valid)
                    scan_seen <= 1'b1;
            end
            assign adc_ready = scan_seen;
            // V1.3B maps the two angle sensors to VAUX0/VAUX1, RP1..RP3 to
            // VAUX2..VAUX4, and RP4 to VAUX6.  xadc6_reader keeps this
            // channel numbering identical to the XADC sequencer.
            xadc6_reader u_xadc6 (
                .clk(clk_50m), .rst_n(rst_n), .vauxp(vauxp), .vauxn(vauxn),
                .angle1(angle_adc_source), .angle2(angle2_adc_source),
                .rp1(kp_adc_source), .rp2(ki_adc_source),
                .rp3(kd_adc_source), .rp4(rp4_adc_source),
                .sample_valid(xadc_sample_valid)
            );
        end else begin : g_digital_adc
            assign ads_scl_low = 1'b0;
            assign ads_sda_low = 1'b0;
            assign angle_adc_source = angle_adc_raw;
            assign angle2_adc_source= angle2_adc_raw;
            assign kp_adc_source    = kp_adc_raw;
            assign ki_adc_source    = ki_adc_raw;
            assign kd_adc_source    = kd_adc_raw;
            assign rp4_adc_source   = rp4_adc_raw;
            assign xadc_sample_valid= 1'b0;
            assign adc_ready       = 1'b1;
        end
    endgenerate

    // 1 kHz control scheduler and 20 Hz outer position loop tick.
    always @(posedge clk_50m) begin
        if (!rst_n) begin
            control_count <= 32'd0;
            outer_count   <= 6'd0;
            inner_count   <= 3'd0;
            control_tick  <= 1'b0;
            outer_tick    <= 1'b0;
        end else begin
            control_tick <= 1'b0;
            outer_tick   <= 1'b0;
            inner_tick   <= 1'b0;
            inner_tick   <= 1'b0;
            if (control_count >= CONTROL_DIV - 1) begin
                control_count <= 32'd0;
                control_tick  <= 1'b1;
                if (inner_count >= 3'd4) begin
                    inner_count <= 3'd0;
                    inner_tick <= 1'b1;
                end else begin
                    inner_count <= inner_count + 1'b1;
                end
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
    wire [11:0] rp4_adc_f;

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
    adc_filter_12 u_rp4_filter (
        .clk(clk_50m), .rst_n(rst_n), .sample_en(control_tick),
        .raw(rp4_adc_source), .filtered(rp4_adc_f)
    );

    // Outputs of pendulum_ui are nets at this hierarchy.  Vivado 2018.3
    // rejects a variable driven by a submodule output when the same signal is
    // subsequently used as an input to angle_center_map.
    wire [11:0] ui_center_angle;
    wire signed [15:0] mapped_angle_error_counts;
    wire signed [15:0] mapped_angle_error_deg_x10;
    wire signed [15:0] mapped_angle2_error_counts;
    wire signed [15:0] legacy_angle_deg_x10;
    // Preserve the original 0..360-degree behavior for the legacy
    // simulation/control build.  The STM32 UI build instead uses the
    // signed ADC-count error around CENTER_ANGLE.
    angle_from_adc #(.ADC_SPAN_COUNTS(ADC_SPAN_COUNTS)) u_legacy_angle_map (
        .adc(angle_adc_source), .angle_deg_x10(legacy_angle_deg_x10)
    );
    angle_center_map #(.ADC_SPAN_COUNTS(ADC_SPAN_COUNTS)) u_angle_center_map (
        .adc(angle_adc_source), .center_angle(ui_center_angle),
        .error_counts(mapped_angle_error_counts),
        .angle_deg_x10(mapped_angle_error_deg_x10)
    );
    angle_center_map #(.ADC_SPAN_COUNTS(ADC_SPAN_COUNTS)) u_angle2_center_map (
        .adc(angle2_adc_source), .center_angle(ui_center_angle),
        .error_counts(mapped_angle2_error_counts), .angle_deg_x10()
    );
    // STM32 Mode31 uses the absolute ADC count minus CENTER_ANGLE as the
    // control error. Keep this path separate from the circular angle display
    // mapping: ADC values below 1000 or above 3000 are handled as a fallen
    // pendulum by swing_balance_ctrl and must not wrap at 0/360 degrees.
    integer stm_angle_diff_i;
    integer stm_angle2_diff_i;
    reg signed [15:0] stm_angle_error_counts;
    reg signed [15:0] stm_angle2_error_counts;
    always @* begin
        stm_angle_diff_i = $signed({1'b0, angle_adc_source}) -
                           $signed({1'b0, ui_center_angle});
        stm_angle2_diff_i = $signed({1'b0, angle2_adc_source}) -
                            $signed({1'b0, ui_center_angle});
        if (stm_angle_diff_i > 32767) stm_angle_error_counts = 16'sh7fff;
        else if (stm_angle_diff_i < -32768) stm_angle_error_counts = 16'sh8000;
        else stm_angle_error_counts = stm_angle_diff_i;
        if (stm_angle2_diff_i > 32767) stm_angle2_error_counts = 16'sh7fff;
        else if (stm_angle2_diff_i < -32768) stm_angle2_error_counts = 16'sh8000;
        else stm_angle2_error_counts = stm_angle2_diff_i;
    end
    assign angle_adc_counts = angle_adc_source;
    wire signed [15:0] angle_error_selected = (USE_STM32_UI != 0) ?
        stm_angle_error_counts : legacy_angle_deg_x10;
    wire signed [15:0] angle_deg_selected = (USE_STM32_UI != 0) ?
        mapped_angle_error_deg_x10 : legacy_angle_deg_x10;
    assign angle_error_counts = (ANGLE_SIGN >= 0) ? angle_error_selected :
                                -angle_error_selected;
    assign angle_deg_x10 = (ANGLE_SIGN >= 0) ? angle_deg_selected :
                           -angle_deg_selected;
    reg signed [15:0] control_angle_error_counts;
    reg [15:0] control_kp_q8;
    reg [15:0] control_ki_q8;
    reg [15:0] control_kd_q8;
    wire [15:0] mapped_kp_q8;
    wire [15:0] mapped_ki_q8;
    wire [15:0] mapped_kd_q8;
    gain_from_adc #(.MAX_Q8(256), .ADC_SPAN_COUNTS(ADC_SPAN_COUNTS)) u_kp_map (
        .adc(kp_adc_f), .gain_q8(mapped_kp_q8), .gain_x100()
    );
    gain_from_adc #(.MAX_Q8(128), .ADC_SPAN_COUNTS(ADC_SPAN_COUNTS)) u_ki_map (
        .adc(ki_adc_f), .gain_q8(mapped_ki_q8), .gain_x100()
    );
    gain_from_adc #(.MAX_Q8(256), .ADC_SPAN_COUNTS(ADC_SPAN_COUNTS)) u_kd_map (
        .adc(kd_adc_f), .gain_q8(mapped_kd_q8), .gain_x100()
    );
    wire [15:0] ui_angle_kp_q8;
    wire [15:0] ui_angle_ki_q8;
    wire [15:0] ui_angle_kd_q8;
    wire [15:0] ui_location_kp_q8;
    wire [15:0] ui_location_ki_q8;
    wire [15:0] ui_location_kd_q8;
    wire [31:0] ui_location_target;
    wire [7:0]  ui_start_pwm;
    wire [15:0] ui_start_time_ms;
    wire [7:0]  ui_pwm_offset;
    wire [1:0]  ui_page_int;
    wire [1:0]  ui_menu_index;
    wire        ui_menu_edit;
    wire [3:0]  key_click;
    wire [3:0]  key_long;
    wire [3:0]  key_pressed;
    wire key_tick_1ms;
    wire capture_clear;

    key_events #(.CLK_HZ(CLK_HZ)) u_keys (
        .clk(clk_50m), .rst_n(rst_n), .key_n(key_n),
        .click(key_click), .long_press(key_long), .pressed(key_pressed),
        .tick_1ms(key_tick_1ms)
    );
    pendulum_ui #(.ADC_SPAN_COUNTS(ADC_SPAN_COUNTS), .USE_RP4_TARGET(USE_RP4_TARGET)) u_ui (
        .clk(clk_50m), .rst_n(rst_n), .sample_tick_1ms(control_tick),
        .capture_reset(capture_clear),
        .key_click(key_click), .key_long(key_long),
        .key_pressed(key_pressed),
        .rp1_adc(kp_adc_f), .rp2_adc(ki_adc_f), .rp3_adc(kd_adc_f),
        .rp4_adc(rp4_adc_f), .run_request(ui_run_request), .page(ui_page_int),
        .location_target(ui_location_target), .angle_kp_q8(ui_angle_kp_q8),
        .angle_ki_q8(ui_angle_ki_q8), .angle_kd_q8(ui_angle_kd_q8),
        .location_kp_q8(ui_location_kp_q8), .location_ki_q8(ui_location_ki_q8),
        .location_kd_q8(ui_location_kd_q8), .center_angle(ui_center_angle),
        .start_pwm(ui_start_pwm), .start_time_ms(ui_start_time_ms),
        .pwm_offset(ui_pwm_offset), .menu_index(ui_menu_index),
        .menu_edit(ui_menu_edit), .save_pulse(),
        .angle_kp_armed(), .angle_ki_armed(),
        .angle_kd_armed(), .location_kp_armed(), .location_ki_armed(),
        .location_kd_armed(), .location_target_armed()
    );
    assign run_request = ui_run_request;
    assign ui_page = ui_page_int;
    assign location_target = ui_location_target;
    assign kp_q8 = (USE_STM32_UI != 0) ? ui_angle_kp_q8 : mapped_kp_q8;
    assign ki_q8 = (USE_STM32_UI != 0) ? ui_angle_ki_q8 : mapped_ki_q8;
    assign kd_q8 = (USE_STM32_UI != 0) ? ui_angle_kd_q8 : mapped_kd_q8;


    // Sample the nonlinear ADC mappings at the 1 kHz control boundary.  The
    // live mapped outputs remain available for observation, while the PID
    // datapath starts from registers and does not include the ADC arithmetic.
    always @(posedge clk_50m) begin
        if (!rst_n) begin
            control_angle_error_counts <= 16'sd0;
            control_kp_q8 <= 16'd64;
            control_ki_q8 <= 16'd2;
            control_kd_q8 <= 16'd105;
        end else if (control_tick) begin
            control_angle_error_counts <= angle_error_counts;
            control_kp_q8 <= (USE_STM32_UI != 0) ? ui_angle_kp_q8 : mapped_kp_q8;
            control_ki_q8 <= (USE_STM32_UI != 0) ? ui_angle_ki_q8 : mapped_ki_q8;
            control_kd_q8 <= (USE_STM32_UI != 0) ? ui_angle_kd_q8 : mapped_kd_q8;
        end
    end

    quad_encoder u_encoder (
        .clk(clk_50m), .rst_n(rst_n), .enc_a(motor_enc_a), .enc_b(motor_enc_b),
        .sample_en(control_tick), .clear(capture_clear), .position(motor_position),
        .velocity(motor_velocity)
    );

    // The optional outer loop keeps the motor around its zero position. Its
    // output is converted to a small angle setpoint correction for the inner
    // angle loop, matching the cascade in PID_library.
    wire signed [15:0] outer_output;
    reg signed [15:0] position_measurement;
    always @(posedge clk_50m) begin
        if (!rst_n)
            position_measurement <= 16'sd0;
        else if (control_tick)
            position_measurement <= motor_position[15:0];
    end
    pid_fixed #(.OUT_LIMIT(100), .INTEGRAL_LIMIT(20000),
                .INTEGRAL_THRESHOLD(100), .PIPELINE_ERROR(1)) u_position_pid (
        .clk(clk_50m), .rst_n(rst_n), .clear(capture_clear), .update(outer_tick),
        .enable(control_run_enable && (mode == 2'd2) && (USE_POSITION_LOOP != 0)),
        .setpoint((USE_STM32_UI != 0) ? $signed(ui_location_target[15:0]) : 16'sd0),
        .measurement(position_measurement),
        .kp_q8((USE_STM32_UI != 0) ? ui_location_kp_q8 : 16'd133),
        .ki_q8((USE_STM32_UI != 0) ? ui_location_ki_q8 : 16'd3),
        .kd_q8((USE_STM32_UI != 0) ? ui_location_kd_q8 : 16'd1167),
        .output_value(outer_output), .error_value(), .integral_value()
    );

    reg signed [15:0] angle_setpoint;
    reg signed [15:0] angle_setpoint_next;
    integer target_calc;
    always @* begin
        if (USE_POSITION_LOOP != 0)
            target_calc = -outer_output;
        else
            target_calc = 0;
        if (target_calc > ANGLE_TARGET_LIMIT_X10)
            target_calc = ANGLE_TARGET_LIMIT_X10;
        else if (target_calc < -ANGLE_TARGET_LIMIT_X10)
            target_calc = -ANGLE_TARGET_LIMIT_X10;
        angle_setpoint_next = target_calc;
    end

    // The outer loop updates at 20 Hz.  Register its setpoint before feeding
    // the 1 kHz angle PID so the outer-loop multiply/limit logic is not in the
    // same 50 MHz timing path as the inner PID arithmetic.
    always @(posedge clk_50m) begin
        if (!rst_n)
            angle_setpoint <= 16'sd0;
        else if (outer_tick)
            angle_setpoint <= angle_setpoint_next;
    end

    // STM32 Mode31_Tick updates the inner loop every 5 ms.  The scheduler
    // keeps the motor state machine at 1 kHz but feeds this PID at inner_tick.
    pid_fixed #(.OUT_LIMIT(100), .INTEGRAL_LIMIT(20000), .PIPELINE_ERROR(1)) u_angle_pid (
        .clk(clk_50m), .rst_n(rst_n), .clear(capture_clear), .update(inner_tick),
        .enable(control_run_enable && (mode == 2'd2)), .setpoint(angle_setpoint),
        .measurement(control_angle_error_counts), .kp_q8(control_kp_q8),
        .ki_q8(control_ki_q8), .kd_q8(control_kd_q8),
        .output_value(pid_output), .error_value(), .integral_value()
    );
    assign angle_setpoint_out = angle_setpoint;
    assign location_pid_output = outer_output;

    wire signed [15:0] controller_command;
    wire               controller_armed;
    reg  signed [15:0] command_limited;
    wire signed [15:0] motor_command;
    // Explicitly size integer parameters before passing them to the runtime
    // parameter ports.  Vivado 2018.3 otherwise warns about 32-bit actuals
    // connected to 8/16-bit formals.
    wire [7:0]  legacy_start_pwm = START_PWM;
    wire [15:0] legacy_start_time = START_TIME_MS;
    swing_balance_ctrl #(
        .START_PWM(START_PWM), .START_TIME_MS(START_TIME_MS), .STARTUP_HOLD_MS(STARTUP_HOLD_MS),
        .COMMAND_SLEW_STEP(COMMAND_SLEW_STEP),
        .USE_SECOND_ANGLE(USE_SECOND_ANGLE),
        .RUNTIME_PARAMS(USE_STM32_UI)
    ) u_swing_ctrl (
        .clk(clk_50m), .rst_n(rst_n), .tick_1khz(control_tick),
        .run_enable(control_run_enable), .motor_fault(motor_fault),
        .angle_adc_raw(angle_adc_source),
        .angle2_adc_raw(angle2_adc_source),
        .pid_output(pid_output), .command(controller_command),
        .start_pwm_value((USE_STM32_UI != 0) ? ui_start_pwm : legacy_start_pwm),
        .start_time_value((USE_STM32_UI != 0) ? ui_start_time_ms : legacy_start_time),
        .pwm_offset_value((USE_STM32_UI != 0) ? ui_pwm_offset : 8'd0),
        .mode(mode), .balancing(), .armed(controller_armed),
        .capture_clear(capture_clear)
    );

    generate
        if (MOTOR_SIGN >= 0) begin : g_motor_sign_positive
            assign motor_command = command_limited;
        end else begin : g_motor_sign_negative
            assign motor_command = -command_limited;
        end
    endgenerate

    // Keep the commissioning torque bounded even if a freshly adjusted PID
    // coefficient produces a saturated correction.  The limit is applied
    // before the TB6612 interface and can be raised after the mechanism is
    // proven safe.
    always @* begin
        command_limited = controller_command;
        if (command_limited > MOTOR_COMMAND_LIMIT)
            command_limited = MOTOR_COMMAND_LIMIT;
        else if (command_limited < -MOTOR_COMMAND_LIMIT)
            command_limited = -MOTOR_COMMAND_LIMIT;
    end

    tb6612_pwm #(.CLK_HZ(CLK_HZ), .PWM_HZ(20000)) u_motor_pwm (
        .clk(clk_50m), .rst_n(rst_n),
        .enable(control_run_enable && controller_armed), .fault(motor_fault),
        .command(motor_command), .pwm(motor_pwm), .in1(motor_in1),
        .in2(motor_in2), .stby(motor_stby)
    );

    wire [15:0] kp_x100_raw;
    wire [15:0] ki_x100_raw;
    wire [15:0] kd_x100_raw;
    reg  [15:0] kp_x100;
    reg  [15:0] ki_x100;
    reg  [15:0] kd_x100;
    gain_from_adc #(.MAX_Q8(256), .ADC_SPAN_COUNTS(ADC_SPAN_COUNTS)) u_kp_display (
        .adc(kp_adc_f), .gain_q8(), .gain_x100(kp_x100_raw)
    );
    gain_from_adc #(.MAX_Q8(128), .ADC_SPAN_COUNTS(ADC_SPAN_COUNTS)) u_ki_display (
        .adc(ki_adc_f), .gain_q8(), .gain_x100(ki_x100_raw)
    );
    gain_from_adc #(.MAX_Q8(256), .ADC_SPAN_COUNTS(ADC_SPAN_COUNTS)) u_kd_display (
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

    localparam integer OLED_STARTUP_DELAY =
        (OLED_STARTUP_CYCLES > 0) ? OLED_STARTUP_CYCLES : (CLK_HZ / 10);

    generate
        if (USE_STM32_UI != 0) begin : g_oled_ui
            // The STM32-compatible screen has eight pages so that all live
            // commissioning values fit: raw angle/error, encoder position
            // and speed, both PID gain sets, and mode/page state.
            oled_ssd1306_ui #(
                .STARTUP_CYCLES(OLED_STARTUP_DELAY)
            ) u_oled_ui (
                .clk(clk_50m),
                .rst_n((OLED_RESET_INDEPENDENT != 0) ? oled_rst_n : rst_n),
                .angle_adc(angle_adc_source),
                .angle_error_counts(angle_error_counts),
                .encoder_position(motor_position[15:0]),
                .encoder_velocity(motor_velocity[15:0]),
                .angle_kp_q8(ui_angle_kp_q8),
                .angle_ki_q8(ui_angle_ki_q8),
                .angle_kd_q8(ui_angle_kd_q8),
                .location_kp_q8(ui_location_kp_q8),
                .location_ki_q8(ui_location_ki_q8),
                .location_kd_q8(ui_location_kd_q8),
                .location_target(ui_location_target),
                .mode(mode), .page(ui_page_int),
                .center_angle(ui_center_angle),
                .start_pwm(ui_start_pwm),
                .start_time_ms(ui_start_time_ms),
                .pwm_offset(ui_pwm_offset),
                .menu_index(ui_menu_index),
                .menu_edit(ui_menu_edit),
                .run_request(ui_run_request),
                .k1_pressed(key_pressed[0]),
                .oled_scl_low(oled_scl_low), .oled_sda_low(oled_sda_low),
                .oled_sda_in(oled_sda_in)
            );
        end else begin : g_oled_legacy
            // The compact driver is kept for the original simulation top.
            oled_ssd1306 #(
                .STARTUP_CYCLES(OLED_STARTUP_DELAY)
            ) u_oled (
                .clk(clk_50m),
                .rst_n((OLED_RESET_INDEPENDENT != 0) ? oled_rst_n : rst_n),
                .kp_x100(kp_x100), .ki_x100(ki_x100),
                .kd_x100(kd_x100), .mode(mode),
                .oled_scl_low(oled_scl_low), .oled_sda_low(oled_sda_low),
                .oled_sda_in(oled_sda_in)
            );
        end
    endgenerate
endmodule


