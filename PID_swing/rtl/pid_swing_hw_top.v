`timescale 1ns/1ps

// Physical V1.3B top level.  The pendulum angle sensor (SENSOR1) and the
// four potentiometers RP1..RP4 are read by two ADS1115 modules on a dedicated
// I2C bus (ads_scl/ads_sda), separate from the OLED bus:
//   ADS1115 #1, ADDR -> GND (0x48): AIN0 = SENSOR1 (angle)
//   ADS1115 #2, ADDR -> VDD (0x49): AIN0..AIN3 = RP1..RP4
// Both modules run from 3.3 V and take the PID baseboard's 0..3.3 V signals
// directly (+/-4.096 V range); no resistor divider is needed.  The raw result
// is rescaled to the STM32 12-bit 0..4095 range, so all STM32 thresholds and
// gain mappings are unchanged.  The XADC is not used in this build.
module pid_swing_hw_top #(
    parameter integer ADC_SPAN_COUNTS = 4095,
    parameter integer MOTOR_SIGN = 1,
    parameter integer ANGLE_SIGN = 1,
    parameter integer USE_POSITION_LOOP = 1,
    // Match the STM32 reference application, which starts its control loop
    // from main() without requiring an external run switch.  Set AUTO_START
    // to zero to restore explicit rst_n/run_enable control.
    parameter integer AUTO_START = 1,
    parameter integer POR_CYCLES = 5000000,
    parameter integer STARTUP_HOLD_MS = 3000,
    parameter integer COMMAND_SLEW_STEP = 1,
    parameter integer START_PWM = 20,
    parameter integer MOTOR_COMMAND_LIMIT = 35
) (
    input  wire        clk_50m,
    input  wire        rst_n,
    input  wire        run_enable,
    input  wire        motor_enc_a,
    input  wire        motor_enc_b,
    // Active-low K1..K4 inputs from the STM32-compatible baseboard.
    input  wire [3:0]  key_n,
    output wire        motor_pwm,
    output wire        motor_in1,
    output wire        motor_in2,
    output wire        motor_stby,
    inout  wire        oled_scl,
    inout  wire        oled_sda,
    inout  wire        ads_scl,
    inout  wire        ads_sda
);
    wire oled_scl_low;
    wire oled_sda_low;
    wire oled_sda_in;
    wire ads_scl_low;
    wire ads_sda_low;
    wire ads_sda_in;

    // Xilinx configuration initializes these registers to zero.  The counter
    // then provides a deterministic 100 ms power-on delay at 50 MHz, allowing
    // the OLED and TB6612 supplies to settle before the control logic is
    // released.  This is a power-on delay, so it intentionally does not
    // depend on the optional external motor-inhibit input: a low J6-9 signal
    // must not keep the OLED permanently in reset.  The swing controller adds
    // a separate 3 s motor standby hold.
    reg [31:0] por_count = 32'd0;
    reg        por_rst_n = 1'b0;
    always @(posedge clk_50m) begin
        if (por_count < POR_CYCLES) begin
            por_count <= por_count + 1'b1;
            por_rst_n <= 1'b0;
        end else begin
            por_rst_n <= 1'b1;
        end
    end
    wire oled_rst_n = por_rst_n;
    wire ctrl_rst_n = rst_n && ((AUTO_START != 0) ? por_rst_n : 1'b1);
    // run_enable is always honored as a physical motor inhibit.  In the
    // default automatic mode the XDC pull-up keeps it high, while pulling J6
    // pin 10 low immediately returns the controller to its safe idle state.
    wire ctrl_run_enable = rst_n && ((AUTO_START != 0) ? por_rst_n : 1'b1) && run_enable;
    // Physical open-drain I2C buffers must sit at the package-pin boundary.
    IOBUF u_oled_scl_iobuf (
        .I(1'b0), .T(~oled_scl_low), .O(), .IO(oled_scl)
    );
    IOBUF u_oled_sda_iobuf (
        .I(1'b0), .T(~oled_sda_low), .O(oled_sda_in), .IO(oled_sda)
    );

    IOBUF u_ads_scl_iobuf (
        .I(1'b0), .T(~ads_scl_low), .O(), .IO(ads_scl)
    );
    IOBUF u_ads_sda_iobuf (
        .I(1'b0), .T(~ads_sda_low), .O(ads_sda_in), .IO(ads_sda)
    );

    pid_swing_top #(
        .USE_XADC(0),
        .USE_ADS1115(1),
        .ADC_SPAN_COUNTS(ADC_SPAN_COUNTS),
        .MOTOR_SIGN(MOTOR_SIGN),
        .ANGLE_SIGN(ANGLE_SIGN),
        .USE_POSITION_LOOP(USE_POSITION_LOOP),
        .USE_STM32_UI(1),
        // The present mechanical assembly has one fitted angle sensor.  Keep
        // the second VAUX channel available for the optional second sensor,
        // but do not let an unpopulated input hold the controller in the
        // safety-idle state.
        .USE_SECOND_ANGLE(0),
        // RP4 is reserved for future commissioning.  The STM32 reference
        // application changes the position target with K2/K3, so do not let
        // an unintentional knob movement change the target at power-up.
        .USE_RP4_TARGET(0),
        .STARTUP_HOLD_MS(STARTUP_HOLD_MS),
        .COMMAND_SLEW_STEP(COMMAND_SLEW_STEP),
        .START_PWM(START_PWM),
        .MOTOR_COMMAND_LIMIT(MOTOR_COMMAND_LIMIT),
        .OLED_RESET_INDEPENDENT(1)
    ) u_control (
        .clk_50m(clk_50m),
        .rst_n(ctrl_rst_n),
        .run_enable(ctrl_run_enable),
        .angle_adc_raw(12'd0),
        .kp_adc_raw(12'd0),
        .ki_adc_raw(12'd0),
        .kd_adc_raw(12'd0),
        .angle2_adc_raw(12'd0),
        .rp4_adc_raw(12'd0),
        .motor_enc_a(motor_enc_a),
        .motor_enc_b(motor_enc_b),
        .motor_fault(1'b0),
        .vauxp(7'd0),
        .vauxn(7'd0),
        .key_n(key_n),
        .oled_rst_n(oled_rst_n),
        .motor_pwm(motor_pwm),
        .motor_in1(motor_in1),
        .motor_in2(motor_in2),
        .motor_stby(motor_stby),
        .oled_scl_low(oled_scl_low),
        .oled_sda_low(oled_sda_low),
        .oled_sda_in(oled_sda_in),
        .ads_scl_low(ads_scl_low),
        .ads_sda_low(ads_sda_low),
        .ads_sda_in(ads_sda_in),
        .mode(),
        .pid_output(),
        .motor_position(),
        .motor_velocity(),
        .angle_deg_x10(),
        .kp_q8(),
        .ki_q8(),
        .kd_q8(),
        .angle_adc_counts(), .angle_error_counts(), .location_target(),
        .ui_page(), .run_request(), .angle_setpoint_out(), .location_pid_output()
    );
endmodule

