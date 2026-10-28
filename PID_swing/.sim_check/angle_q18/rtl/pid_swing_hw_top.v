`timescale 1ns/1ps

// Physical V1.3B top level. Sensor voltages must be conditioned to the
// XADC's 0..1 V unipolar range before reaching the selected VAUX pins.
// Each sensor wiper goes through 24 kOhm series, with 10 kOhm to ground at
// VAUXP. Connect the corresponding VAUXN to analog ground. Sensor and FPGA
// grounds must be common. This maps 3.3 V to ~0.971 V (~3974 counts).
// Calibrate ADC_SPAN_COUNTS on real hardware before motor operation.
module pid_swing_hw_top #(
    parameter integer ADC_SPAN_COUNTS = 3974,
    parameter integer MOTOR_SIGN = 1,
    parameter integer ANGLE_SIGN = 1,
    parameter integer USE_POSITION_LOOP = 1
) (
    input  wire        clk_50m,
    input  wire        rst_n,
    input  wire        run_enable,
    input  wire        motor_enc_a,
    input  wire        motor_enc_b,
    input  wire [3:0]  vauxp,
    input  wire [3:0]  vauxn,
    output wire        motor_pwm,
    output wire        motor_in1,
    output wire        motor_in2,
    output wire        motor_stby,
    inout  wire        oled_scl,
    inout  wire        oled_sda
);
    wire oled_scl_low;
    wire oled_sda_low;
    wire oled_sda_in;

    // Physical open-drain I2C buffers must sit at the package-pin boundary.
    IOBUF u_oled_scl_iobuf (
        .I(1'b0), .T(~oled_scl_low), .O(), .IO(oled_scl)
    );
    IOBUF u_oled_sda_iobuf (
        .I(1'b0), .T(~oled_sda_low), .O(oled_sda_in), .IO(oled_sda)
    );

    pid_swing_top #(
        .USE_XADC(1),
        .ADC_SPAN_COUNTS(ADC_SPAN_COUNTS),
        .MOTOR_SIGN(MOTOR_SIGN),
        .ANGLE_SIGN(ANGLE_SIGN),
        .USE_POSITION_LOOP(USE_POSITION_LOOP)
    ) u_control (
        .clk_50m(clk_50m),
        .rst_n(rst_n),
        .run_enable(run_enable),
        .angle_adc_raw(12'd0),
        .kp_adc_raw(12'd0),
        .ki_adc_raw(12'd0),
        .kd_adc_raw(12'd0),
        .motor_enc_a(motor_enc_a),
        .motor_enc_b(motor_enc_b),
        .motor_fault(1'b0),
        .vauxp(vauxp),
        .vauxn(vauxn),
        .motor_pwm(motor_pwm),
        .motor_in1(motor_in1),
        .motor_in2(motor_in2),
        .motor_stby(motor_stby),
        .oled_scl_low(oled_scl_low),
        .oled_sda_low(oled_sda_low),
        .oled_sda_in(oled_sda_in),
        .mode(),
        .pid_output(),
        .motor_position(),
        .motor_velocity(),
        .angle_deg_x10(),
        .kp_q8(),
        .ki_q8(),
        .kd_q8()
    );
endmodule
