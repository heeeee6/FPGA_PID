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
    reg [31:0] duty_count_reg;
    reg [31:0] duty_count_next;
    integer abs_command;

    always @* begin
        if (command < 0)
            abs_command = -command;
        else
            abs_command = command;
        if (abs_command > 100)
            abs_command = 100;
        if (PWM_PERIOD == 2500)
            duty_count_next = abs_command * 25;
        else
            duty_count_next = abs_command * PWM_COUNTS_PER_PERCENT;
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            pwm_count <= 32'd0;
            duty_count_reg <= 32'd0;
            pwm       <= 1'b0;
            in1       <= 1'b0;
            in2       <= 1'b0;
            stby      <= 1'b0;
        end else begin
            if (pwm_count >= PWM_PERIOD - 1)
                pwm_count <= 32'd0;
            else
                pwm_count <= pwm_count + 1'b1;

            // Keep the TB6612 in standby while the command is zero.  The
            // control state machine uses this during reset and its startup
            // hold, so a powered board cannot energize the motor merely
            // because the FPGA has finished configuration.
            stby <= enable && !fault && (command != 0);
            // Register the slow command-derived values.  This isolates the
            // control command from the high-rate PWM compare path and adds
            // at most one 50 MHz clock of command-to-output latency.
            duty_count_reg  <= (!enable || fault) ? 32'd0 : duty_count_next;
            if (!enable || fault || (command == 0)) begin
                pwm <= 1'b0;
                in1 <= 1'b0;
                in2 <= 1'b0;
            end else begin
                pwm <= (pwm_count < duty_count_reg);
                // Match Motor_SetPWM() in the STM32 reference:
                // positive command: PB12/AIN1=0, PB13/AIN2=1.
                if (command < 0) begin
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

