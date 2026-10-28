`timescale 1ns/1ps

// Position PID in Q8.8 coefficient format.
// The error and output use engineering units of 0.1 degree and percent,
// respectively.  This is the same position form used by PID_library:
// u = Kp*e + Ki*sum(e) + Kd*(e-e_prev), with integral and output limits.
module pid_fixed #(
    parameter integer OUT_LIMIT = 100,
    parameter integer INTEGRAL_LIMIT = 20000
) (
    input  wire                  clk,
    input  wire                  rst_n,
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
    reg signed [31:0] error_next;
    reg signed [31:0] integral_next;
    reg signed [63:0] p_term;
    reg signed [63:0] i_term;
    reg signed [63:0] d_term;
    reg signed [63:0] sum_term;
    reg signed [63:0] product_term;

    always @* begin
        error_next = $signed(setpoint) - $signed(measurement);
        integral_next = integral_value + error_next;
        if (integral_next > INTEGRAL_LIMIT)
            integral_next = INTEGRAL_LIMIT;
        else if (integral_next < -INTEGRAL_LIMIT)
            integral_next = -INTEGRAL_LIMIT;

        product_term = $signed(kp_q8) * error_next;
        p_term = product_term >>> 8;
        product_term = $signed(ki_q8) * integral_next;
        i_term = product_term >>> 8;
        product_term = $signed(kd_q8) * (error_next - error_prev);
        d_term = product_term >>> 8;
        sum_term = p_term + i_term + d_term;
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            error_prev    <= 32'sd0;
            error_value   <= 32'sd0;
            integral_value<= 32'sd0;
            output_value  <= 16'sd0;
        end else if (!enable) begin
            error_prev    <= 32'sd0;
            error_value   <= 32'sd0;
            integral_value<= 32'sd0;
            output_value  <= 16'sd0;
        end else if (update) begin
            error_prev     <= error_next;
            error_value    <= error_next;
            integral_value <= (ki_q8 == 16'd0) ? 32'sd0 : integral_next;
            if (sum_term > OUT_LIMIT)
                output_value <= OUT_LIMIT;
            else if (sum_term < -OUT_LIMIT)
                output_value <= -OUT_LIMIT;
            else
                output_value <= sum_term[15:0];
        end
    end
endmodule

