`timescale 1ns/1ps

// Startup and operating-state controller derived from the 16-倒立摆-自动启摆
// reference program.  A pair of fixed amplitude impulses builds energy until
// the angle enters the capture window; the PID loop then takes over.
module swing_balance_ctrl #(
    parameter integer START_PWM = 35,
    parameter integer START_TIME_MS = 100,
    parameter integer SWING_TRIGGER_X10 = 250,
    parameter integer CAPTURE_ANGLE_X10 = 180,
    parameter integer EXIT_ANGLE_X10 = 450,
    parameter integer CAPTURE_SPEED = 40,
    parameter integer CAPTURE_COUNT = 30
) (
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  tick_1khz,
    input  wire                  run_enable,
    input  wire                  motor_fault,
    input  wire signed [15:0]    angle_deg_x10,
    input  wire signed [31:0]    motor_velocity,
    input  wire signed [15:0]    pid_output,
    output reg  signed [15:0]    command,
    output reg  [1:0]            mode,
    output reg                    balancing
);
    localparam [1:0] MODE_IDLE    = 2'd0;
    localparam [1:0] MODE_SWING   = 2'd1;
    localparam [1:0] MODE_BALANCE = 2'd2;
    localparam [1:0] MODE_FAULT   = 2'd3;

    reg [1:0] state;
    reg [31:0] phase_count;
    reg [15:0] capture_count;
    reg swing_direction;
    integer angle_abs;

    always @* begin
        if (angle_deg_x10 < 0)
            angle_abs = -angle_deg_x10;
        else
            angle_abs = angle_deg_x10;
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            state          <= MODE_IDLE;
            phase_count    <= 32'd0;
            capture_count  <= 16'd0;
            swing_direction<= 1'b0;
            command        <= 16'sd0;
            mode           <= MODE_IDLE;
            balancing      <= 1'b0;
        end else if (!run_enable) begin
            state          <= MODE_IDLE;
            phase_count    <= 32'd0;
            capture_count  <= 16'd0;
            command        <= 16'sd0;
            mode           <= MODE_IDLE;
            balancing      <= 1'b0;
        end else if (motor_fault) begin
            state          <= MODE_FAULT;
            command        <= 16'sd0;
            mode           <= MODE_FAULT;
            balancing      <= 1'b0;
        end else if (tick_1khz) begin
            case (state)
                MODE_IDLE: begin
                    phase_count   <= 32'd0;
                    capture_count <= 16'd0;
                    if ((angle_abs <= CAPTURE_ANGLE_X10) &&
                        (motor_velocity >= -CAPTURE_SPEED) &&
                        (motor_velocity <= CAPTURE_SPEED)) begin
                        balancing <= 1'b1;
                        mode      <= MODE_BALANCE;
                        state     <= MODE_BALANCE;
                        command   <= pid_output;
                    end else begin
                        balancing <= 1'b0;
                        mode      <= MODE_SWING;
                        state     <= MODE_SWING;
                        if (angle_deg_x10 > SWING_TRIGGER_X10) begin
                            swing_direction <= 1'b0;
                            command <= START_PWM;
                        end else if (angle_deg_x10 < -SWING_TRIGGER_X10) begin
                            swing_direction <= 1'b1;
                            command <= -START_PWM;
                        end else begin
                            swing_direction <= ~swing_direction;
                            command <= swing_direction ? START_PWM : -START_PWM;
                        end
                    end
                end

                MODE_SWING: begin
                    mode       <= MODE_SWING;
                    balancing  <= 1'b0;
                    command    <= swing_direction ? -START_PWM : START_PWM;
                    if ((angle_abs <= CAPTURE_ANGLE_X10) &&
                        (motor_velocity >= -CAPTURE_SPEED) &&
                        (motor_velocity <= CAPTURE_SPEED)) begin
                        if (capture_count < CAPTURE_COUNT)
                            capture_count <= capture_count + 1'b1;
                        else begin
                            state       <= MODE_BALANCE;
                            mode        <= MODE_BALANCE;
                            balancing   <= 1'b1;
                            phase_count <= 32'd0;
                            command     <= pid_output;
                        end
                    end else begin
                        capture_count <= 16'd0;
                    end

                    if (phase_count >= (START_TIME_MS - 1)) begin
                        phase_count     <= 32'd0;
                        swing_direction <= ~swing_direction;
                    end else begin
                        phase_count <= phase_count + 1'b1;
                    end
                end

                MODE_BALANCE: begin
                    mode      <= MODE_BALANCE;
                    balancing <= 1'b1;
                    command   <= pid_output;
                    if (angle_abs > EXIT_ANGLE_X10) begin
                        state         <= MODE_SWING;
                        mode          <= MODE_SWING;
                        balancing     <= 1'b0;
                        capture_count <= 16'd0;
                        phase_count   <= 32'd0;
                        swing_direction <= (angle_deg_x10 > 0);
                        command       <= (angle_deg_x10 > 0) ? -START_PWM : START_PWM;
                    end
                end

                default: begin
                    state       <= MODE_IDLE;
                    mode        <= MODE_FAULT;
                    balancing   <= 1'b0;
                    command     <= 16'sd0;
                end
            endcase
        end
    end
endmodule

