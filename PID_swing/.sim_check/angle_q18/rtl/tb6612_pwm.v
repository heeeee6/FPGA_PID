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
    reg [31:0] pwm_count;
    reg [31:0] duty_count;
    integer abs_command;

    always @* begin
        if (command < 0)
            abs_command = -command;
        else
            abs_command = command;
        if (abs_command > 100)
            abs_command = 100;
        duty_count = (abs_command * PWM_PERIOD) / 100;
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            pwm_count <= 32'd0;
            pwm       <= 1'b0;
            in1       <= 1'b0;
            in2       <= 1'b0;
            stby      <= 1'b0;
        end else begin
            if (pwm_count >= PWM_PERIOD - 1)
                pwm_count <= 32'd0;
            else
                pwm_count <= pwm_count + 1'b1;

            stby <= enable && !fault;
            if (!enable || fault || (command == 0)) begin
                pwm <= 1'b0;
                in1 <= 1'b0;
                in2 <= 1'b0;
            end else begin
                pwm <= (pwm_count < duty_count);
                if (command > 0) begin
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

