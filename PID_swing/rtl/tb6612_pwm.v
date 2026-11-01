`timescale 1ns/1ps

// TB6612 single-channel interface.  command is signed percent [-100,100].
// PWM is 20 kHz for a 50 MHz clock by default.
module tb6612_pwm #(
    parameter integer CLK_HZ = 50000000,
    parameter integer PWM_HZ = 20000
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               enable,
    input  wire               fault,
    input  wire signed [15:0] command,
    output reg                pwm,
    output reg                in1,
    output reg                in2,
    output reg                stby
);
    localparam integer PWM_PERIOD = CLK_HZ / PWM_HZ;
    // The hardware configuration uses 2500 counts per PWM period, so a
    // percentage command maps to 25 counts per percent.  Keep a generic
    // fallback for other parameter combinations.
    localparam integer PWM_COUNTS_PER_PERCENT = PWM_PERIOD / 100;
    reg [31:0] pwm_count;
    reg [31:0] duty_latched;
    reg        neg_latched;
    reg [31:0] duty_next;
    integer abs_command;

    always @* begin
        if (command < 0)
            abs_command = -command;
        else
            abs_command = command;
        if (abs_command > 100)
            abs_command = 100;
        if (PWM_PERIOD == 2500)
            duty_next = abs_command * 25;
        else
            duty_next = abs_command * PWM_COUNTS_PER_PERCENT;
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            pwm_count <= 32'd0;
            duty_latched <= 32'd0;
            neg_latched  <= 1'b0;
            pwm       <= 1'b0;
            in1       <= 1'b0;
            in2       <= 1'b0;
            stby      <= 1'b0;
        end else begin
            if (pwm_count >= PWM_PERIOD - 1) begin
                pwm_count <= 32'd0;
                // Latch magnitude and direction only at the PWM period
                // boundary.  A command change can therefore never create a
                // shortened pulse or a mid-period direction reversal.
                duty_latched <= (!enable || fault || (command == 0)) ?
                                32'd0 : duty_next;
                neg_latched  <= (command < 0);
            end else begin
                pwm_count <= pwm_count + 1'b1;
            end

            // Keep the TB6612 in standby while the command is zero.  The
            // control state machine uses this during reset and its startup
            // hold, so a powered board cannot energize the motor merely
            // because the FPGA has finished configuration.
            // Keep the bridge enabled for the whole control run.  Magnitude
            // and direction are latched below, so a zero command becomes a
            // zero-duty period instead of chopping the current PWM period.
            stby <= enable && !fault;
            if (!enable || fault) begin
                pwm <= 1'b0;
                in1 <= 1'b0;
                in2 <= 1'b0;
            end else begin
                pwm <= (pwm_count < duty_latched);
                // Match Motor_SetPWM() in the STM32 reference.  Sign is also
                // latched at the PWM boundary to keep each period coherent.
                if (neg_latched) begin
                    in1 <= 1'b1;
                    in2 <= 1'b0;
                end else begin
                    in1 <= 1'b0;
                    in2 <= 1'b1;
                end
            end
        end
    end
endmodule

