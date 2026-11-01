`timescale 1ns/1ps

// Regression for the extended eight-line OLED display.  It verifies that a
// refresh reaches all eight SSD1306 pages, writes the full 128 columns, and
// takes a new input snapshot after the first frame completes.
module tb_oled_ssd1306_ui;
    reg clk = 1'b0;
    always #500 clk = ~clk;

    reg rst_n = 1'b0;
    reg [11:0] angle_adc = 12'd1234;
    reg signed [15:0] angle_error_counts = -16'sd321;
    reg signed [15:0] encoder_position = 16'sd4321;
    reg signed [15:0] encoder_velocity = -16'sd87;
    reg [15:0] angle_kp_q8 = 16'd64;
    reg [15:0] angle_ki_q8 = 16'd2;
    reg [15:0] angle_kd_q8 = 16'd105;
    reg [15:0] location_kp_q8 = 16'd133;
    reg [15:0] location_ki_q8 = 16'd3;
    reg [15:0] location_kd_q8 = 16'd1167;
    reg signed [31:0] location_target = 32'sd408;
    reg [1:0] mode = 2'd2;
    reg [1:0] page = 2'd0;
    reg [11:0] center_angle = 12'd2094;
    reg [7:0] start_pwm = 8'd45;
    reg [15:0] start_time_ms = 16'd186;
    reg [7:0] pwm_offset = 8'd0;
    reg [1:0] menu_index = 2'd2;
    reg menu_edit = 1'b1;
    reg run_request = 1'b1;
    reg k1_pressed = 1'b0;

    wire oled_scl_low;
    wire oled_sda_low;
    integer pages_seen = 0;
    integer capture_count = 0;
    reg page_start_prev = 1'b0;
    reg saw_last_column = 1'b0;

    oled_ssd1306_ui #(
        .CLK_HZ(1_000_000),
        .I2C_HZ(250_000),
        .STARTUP_CYCLES(2),
        .REFRESH_CYCLES(5)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .angle_adc(angle_adc),
        .angle_error_counts(angle_error_counts),
        .encoder_position(encoder_position),
        .encoder_velocity(encoder_velocity),
        .angle_kp_q8(angle_kp_q8),
        .angle_ki_q8(angle_ki_q8),
        .angle_kd_q8(angle_kd_q8),
        .location_kp_q8(location_kp_q8),
        .location_ki_q8(location_ki_q8),
        .location_kd_q8(location_kd_q8),
        .location_target(location_target),
        .mode(mode), .page(page),
        .center_angle(center_angle),
        .start_pwm(start_pwm),
        .start_time_ms(start_time_ms),
        .pwm_offset(pwm_offset),
        .menu_index(menu_index), .menu_edit(menu_edit),
        .run_request(run_request), .k1_pressed(k1_pressed),
        .oled_scl_low(oled_scl_low),
        .oled_sda_low(oled_sda_low),
        .oled_sda_in(1'b1)
    );

    always @(posedge clk) begin
        if (dut.state == 5'd5 && !page_start_prev)
            pages_seen = pages_seen | (1 << dut.line_index);
        page_start_prev = (dut.state == 5'd5);

        if (dut.state == 5'd4)
            capture_count = capture_count + 1;

        // The last data byte on each page is char 21 / glyph column 1.
        if (dut.state == 5'd11 &&
            dut.char_index == 5'd21 && dut.glyph_index == 3'd1)
            saw_last_column = 1'b1;
    end

    initial begin
        repeat (5) @(posedge clk);
        rst_n = 1'b1;

        wait (capture_count == 1);
        angle_adc = 12'd2345;
        angle_error_counts = 16'sd295;

        wait (capture_count == 2);
        #1;
        if (dut.angle_adc_s !== 12'd2345)
            $fatal(1, "second refresh did not capture the changed angle");
        if (dut.angle_error_s !== 16'sd295)
            $fatal(1, "second refresh did not capture the changed error");

        page = 2'd3;
        center_angle = 12'd2094;
        start_pwm = 8'd45;
        start_time_ms = 16'd186;
        menu_index = 2'd2;
        menu_edit = 1'b1;
        run_request = 1'b0;
        k1_pressed = 1'b1;
        location_target = 32'sd408;

        wait (capture_count == 3);
        #1;
        if (dut.page_s !== 2'd3 ||
            dut.center_angle_s !== 12'd2094 ||
            dut.start_pwm_s !== 8'd45 ||
            dut.start_time_ms_s !== 16'd186 ||
            dut.menu_index_s !== 2'd2 ||
            dut.menu_edit_s !== 1'b1 ||
            dut.k1_pressed_s !== 1'b1 ||
            dut.run_request_s !== 1'b0 ||
            dut.location_target_s !== 16'sd408)
            $fatal(1, "PAGE3/parameter/K1 snapshot was not captured");

        wait (pages_seen == 8'hff);
        if (!saw_last_column)
            $fatal(1, "display rows did not clear through column 127");

        $display("tb_oled_ssd1306_ui: PASS");
        $finish;
    end

    initial begin
        #500_000_000;
        $fatal(1, "OLED UI refresh timed out: pages_seen=%02x", pages_seen);
    end
endmodule
