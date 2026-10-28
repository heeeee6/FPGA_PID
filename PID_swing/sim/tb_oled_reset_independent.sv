`timescale 1ns/1ps

// The external J6-9 reset is a motor inhibit in the physical top level.  It
// must not prevent the OLED's independent power-on reset from starting I2C.
module tb_oled_reset_independent;
    reg clk = 1'b0;
    always #500 clk = ~clk;

    reg rst_n = 1'b0;
    reg oled_rst_n = 1'b0;
    reg run_enable = 1'b0;
    reg [11:0] angle_adc_raw = 12'd0;
    reg [11:0] kp_adc_raw = 12'd0;
    reg [11:0] ki_adc_raw = 12'd0;
    reg [11:0] kd_adc_raw = 12'd0;
    reg motor_enc_a = 1'b0;
    reg motor_enc_b = 1'b0;
    wire [6:0] vauxp = 7'b0;
    wire [6:0] vauxn = 7'b0;
    wire motor_pwm;
    wire motor_in1;
    wire motor_in2;
    wire motor_stby;
    wire oled_scl_low;
    wire oled_sda_low;
    wire [1:0] mode;

    pid_swing_top #(
        .CLK_HZ(1_000_000),
        .USE_XADC(0),
        .OLED_RESET_INDEPENDENT(1),
        .OLED_STARTUP_CYCLES(2)
    ) dut (
        .clk_50m(clk),
        .rst_n(rst_n),
        .run_enable(run_enable),
        .angle_adc_raw(angle_adc_raw),
        .kp_adc_raw(kp_adc_raw),
        .ki_adc_raw(ki_adc_raw),
        .kd_adc_raw(kd_adc_raw),
        .angle2_adc_raw(12'd0),
        .rp4_adc_raw(12'd0),
        .motor_enc_a(motor_enc_a),
        .motor_enc_b(motor_enc_b),
        .motor_fault(1'b0),
        .key_n(4'hf),
        .vauxp(vauxp),
        .vauxn(vauxn),
        .oled_rst_n(oled_rst_n),
        .motor_pwm(motor_pwm),
        .motor_in1(motor_in1),
        .motor_in2(motor_in2),
        .motor_stby(motor_stby),
        .oled_scl_low(oled_scl_low),
        .oled_sda_low(oled_sda_low),
        // A pulled-up idle SDA is enough to let the I2C master run through
        // initialization; ACK handling is covered by tb_oled_ssd1306.
        .oled_sda_in(1'b1),
        .ads_scl_low(), .ads_sda_low(), .ads_sda_in(1'b1),
        .mode(mode),
        .pid_output(),
        .motor_position(),
        .motor_velocity(),
        .angle_deg_x10(),
        .kp_q8(),
        .ki_q8(),
        .kd_q8()
    );

    integer n;
    reg saw_i2c_activity;
    initial begin
        // Keep both external controls low.  This is the condition that used
        // to hold the OLED in reset after the motor safety change.
        repeat (5) @(posedge clk);
        if (motor_pwm !== 1'b0 || motor_in1 !== 1'b0 ||
            motor_in2 !== 1'b0 || motor_stby !== 1'b0)
            $fatal(1, "motor outputs were not safe during external reset");

        oled_rst_n = 1'b1;
        saw_i2c_activity = 1'b0;
        for (n = 0; n < 200; n = n + 1) begin
            @(posedge clk);
            if (oled_scl_low === 1'b1)
                saw_i2c_activity = 1'b1;
        end
        if (!saw_i2c_activity)
            $fatal(1, "OLED did not start I2C while external reset was low");
        if (motor_pwm !== 1'b0 || motor_in1 !== 1'b0 ||
            motor_in2 !== 1'b0 || motor_stby !== 1'b0 || mode !== 2'd0)
            $fatal(1, "external reset did not keep motor controller idle");

        $display("tb_oled_reset_independent: PASS");
        $finish;
    end
endmodule
