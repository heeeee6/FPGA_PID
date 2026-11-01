`timescale 1ns/1ps

// PID in Q8.8 coefficient format.
//
// The P/I/D numerator terms are added in full precision before one final
// divide and truncation-toward-zero conversion.  This follows the integer
// control structure used by the reference FPGA inverted-pendulum project and
// avoids discarding small P/D fractions in each term independently.
module pid_fixed #(
    parameter integer OUT_LIMIT = 100,
    parameter integer INTEGRAL_LIMIT = 20000,
    parameter integer INTEGRAL_THRESHOLD = 0,
    parameter integer PIPELINE_ERROR = 0
) (
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  clear,
    input  wire                  update,
    input  wire                  enable,
    input  wire signed [15:0]    setpoint,
    input  wire signed [15:0]    measurement,
    input  wire [15:0]           kp_q8,
    input  wire [15:0]           ki_q8,
    input  wire [15:0]           kd_q8,
    output reg  signed [15:0]    output_value,
    output reg  signed [31:0]    error_value,
    output reg  signed [31:0]    integral_value
);
    reg signed [31:0] error_prev;
    reg signed [31:0] error_input_reg;
    reg signed [31:0] error_next;
    reg signed [31:0] integral_next;
    reg signed [63:0] p_term;
    reg signed [63:0] i_term;
    reg signed [63:0] d_term;
    reg signed [63:0] sum_term;
    reg        [63:0] sum_abs;
    reg        [63:0] out_limit_scaled;
    reg        [63:0] out_mag;

    always @* begin
        if (PIPELINE_ERROR != 0)
            error_next = error_input_reg;
        else
            error_next = $signed(setpoint) - $signed(measurement);
        integral_next = integral_value + error_next;
        if (integral_next > INTEGRAL_LIMIT)
            integral_next = INTEGRAL_LIMIT;
        else if (integral_next < -INTEGRAL_LIMIT)
            integral_next = -INTEGRAL_LIMIT;

        p_term = $signed(kp_q8) * error_next;
        i_term = $signed(ki_q8) * integral_next;
        d_term = $signed(kd_q8) * (error_next - error_prev);
        sum_term = p_term + i_term + d_term;
        sum_abs = sum_term[63] ? -sum_term : sum_term;
        out_limit_scaled = OUT_LIMIT << 8;
        out_mag = sum_abs >> 8;
    end

    always @(posedge clk) begin
        if (!rst_n || clear) begin
            error_input_reg <= 32'sd0;
            error_prev    <= 32'sd0;
            error_value   <= 32'sd0;
            integral_value<= 32'sd0;
            output_value  <= 16'sd0;
        end else if (!enable) begin
            error_input_reg <= 32'sd0;
            error_prev    <= 32'sd0;
            error_value   <= 32'sd0;
            integral_value<= 32'sd0;
            output_value  <= 16'sd0;
        end else if (update) begin
            error_prev     <= error_next;
            error_value    <= error_next;
            if (ki_q8 == 16'd0)
                integral_value <= 32'sd0;
            else if ((INTEGRAL_THRESHOLD != 0) &&
                     ((error_next >= INTEGRAL_THRESHOLD) ||
                      (error_next <= -INTEGRAL_THRESHOLD)))
                integral_value <= 32'sd0;
            else if (((sum_term > $signed(out_limit_scaled)) &&
                      (error_next > 0)) ||
                     ((sum_term < -$signed(out_limit_scaled)) &&
                      (error_next < 0)))
                // Conditional integration: do not wind farther after the
                // output has saturated in the same direction as the error.
                integral_value <= integral_value;
            else
                integral_value <= integral_next;
            if (sum_term > $signed(out_limit_scaled))
                output_value <= OUT_LIMIT;
            else if (sum_term < -$signed(out_limit_scaled))
                output_value <= -OUT_LIMIT;
            else if (sum_term[63])
                output_value <= -$signed(out_mag[15:0]);
            else
                output_value <= $signed(out_mag[15:0]);
        end
        if (rst_n && enable && (PIPELINE_ERROR != 0))
            error_input_reg <= $signed(setpoint) - $signed(measurement);
    end
endmodule

