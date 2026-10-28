`timescale 1ns/1ps

// Faithful port of the STM32 V1.1 Mode31_Tick run state machine.
// RunState semantics match Mode31.c exactly:
//   0        stop
//   1        judge (sample the angle every 40 ms, detect swing extrema or
//            the upright capture window)
//   11..14   left-impulse sequence (+START, hold, -START, hold)
//   21..24   right-impulse sequence (-START, hold, +START, hold)
//   2        balance (the cascade PID loop takes over)
// Balance exits to STOP only when the pendulum has actually fallen (ADC
// outside FALL_LOW..FALL_HIGH), as in Mode31.c.  Swing-up is not inhibited
// by the hanging angle, which normally sits near one of the ADC rails.
module swing_balance_ctrl #(
    parameter integer START_PWM = 35,
    parameter integer START_TIME_MS = 90,
    parameter integer STARTUP_HOLD_MS = 3000,
    parameter integer COMMAND_SLEW_STEP = 1,
    // STM32 Mode31 absolute ADC thresholds.
    parameter integer ANGLE_A = 1700,
    parameter integer ANGLE_B = 2300,
    parameter integer FALL_LOW = 1000,
    parameter integer FALL_HIGH = 3000,
    parameter integer PEAK_MARGIN = 2,
    parameter integer USE_SECOND_ANGLE = 0,
    parameter integer RUNTIME_PARAMS = 0
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               tick_1khz,
    input  wire               run_enable,
    input  wire               motor_fault,
    input  wire [11:0]        angle_adc_raw,
    input  wire [11:0]        angle2_adc_raw,
    input  wire signed [15:0] pid_output,
    input  wire [7:0]         start_pwm_value,
    input  wire [15:0]        start_time_value,
    input  wire [7:0]         pwm_offset_value,
    output reg  signed [15:0] command,
    output reg  [1:0]         mode,
    output reg                balancing,
    output reg                armed,
    output reg                capture_clear
);
    localparam [3:0] S_IDLE    = 4'd0;
    localparam [3:0] S_JUDGE   = 4'd1;
    localparam [3:0] S_L1      = 4'd11;
    localparam [3:0] S_L2      = 4'd12;
    localparam [3:0] S_L3      = 4'd13;
    localparam [3:0] S_L4      = 4'd14;
    localparam [3:0] S_R1      = 4'd21;
    localparam [3:0] S_R2      = 4'd22;
    localparam [3:0] S_R3      = 4'd23;
    localparam [3:0] S_R4      = 4'd24;
    localparam [3:0] S_BALANCE = 4'd2;

    localparam [1:0] MODE_IDLE    = 2'd0;
    localparam [1:0] MODE_SWING   = 2'd1;
    localparam [1:0] MODE_BALANCE = 2'd2;
    localparam [1:0] MODE_FAULT   = 2'd3;

    reg [3:0]   state;
    reg [31:0]  startup_count;
    reg [31:0]  phase_count;
    reg [15:0]  judge_count;
    reg [11:0]  ad0, ad1, ad2;
    reg signed [15:0] target_command;
    reg         run_prev;
    reg         run_start;

    reg [11:0]  n_ad0, n_ad1, n_ad2;
    reg         n_capture, n_right, n_left, n_fallen;
    integer     i_n0, i_n1, i_n2;
    integer     start_pwm_eff;
    integer     start_time_eff;
    integer     pwm_offset_eff;
    integer     pid_command_eff;

    always @* begin
        start_pwm_eff = (RUNTIME_PARAMS != 0) ? start_pwm_value : START_PWM;
        start_time_eff = (RUNTIME_PARAMS != 0) ? start_time_value : START_TIME_MS;
        pwm_offset_eff = (RUNTIME_PARAMS != 0) ? pwm_offset_value : 0;
        if (start_pwm_eff < 0) start_pwm_eff = 0;
        if (start_pwm_eff > 100) start_pwm_eff = 100;
        if (start_time_eff < 0) start_time_eff = 0;
        if (pwm_offset_eff < 0) pwm_offset_eff = 0;
        if (pwm_offset_eff > 50) pwm_offset_eff = 50;

        pid_command_eff = pid_output;
        if (pid_command_eff > 0)
            pid_command_eff = pid_command_eff + pwm_offset_eff;
        else if (pid_command_eff < 0)
            pid_command_eff = pid_command_eff - pwm_offset_eff;
        if (pid_command_eff > 100) pid_command_eff = 100;
        if (pid_command_eff < -100) pid_command_eff = -100;
    end

    // Compute the values that will be sampled on a judge tick so that the
    // sequential state machine can shift the history and classify the new
    // sample in the same 1 kHz boundary without a pipeline delay.
    always @* begin
        n_ad0 = ad0;
        n_ad1 = ad1;
        n_ad2 = ad2;
        if ((state == S_JUDGE) && (judge_count >= 39)) begin
            n_ad0 = angle_adc_raw;
            n_ad1 = ad0;
            n_ad2 = ad1;
        end

        i_n0 = {1'b0, n_ad0};
        i_n1 = {1'b0, n_ad1};
        i_n2 = {1'b0, n_ad2};

        // Capture: the latest two samples are inside the upright window.
        n_capture = (i_n0 > ANGLE_A) && (i_n0 < ANGLE_B) &&
                    (i_n1 > ANGLE_A) && (i_n1 < ANGLE_B) &&
                    ((USE_SECOND_ANGLE == 0) ||
                     ((angle2_adc_raw > ANGLE_A) && (angle2_adc_raw < ANGLE_B)));

        // Right peak: three samples beyond ANGLE_B, the middle one smallest.
        n_right = (i_n0 >= ANGLE_B) && (i_n1 >= ANGLE_B) && (i_n2 >= ANGLE_B) &&
                  (i_n1 < i_n0 - PEAK_MARGIN) && (i_n1 < i_n2 - PEAK_MARGIN);

        // Left peak: three samples beyond ANGLE_A, the middle one largest.
        n_left = (i_n0 <= ANGLE_A) && (i_n1 <= ANGLE_A) && (i_n2 <= ANGLE_A) &&
                 (i_n1 > i_n0 + PEAK_MARGIN) && (i_n1 > i_n2 + PEAK_MARGIN);

        n_fallen = (angle_adc_raw < FALL_LOW) || (angle_adc_raw > FALL_HIGH) ||
                   ((USE_SECOND_ANGLE != 0) &&
                    ((angle2_adc_raw < FALL_LOW) || (angle2_adc_raw > FALL_HIGH)));
    end

    always @(posedge clk) begin
        run_prev <= run_enable;
        if (!rst_n) begin
            state         <= S_IDLE;
            run_start     <= 1'b0;
            startup_count <= 32'd0;
            phase_count   <= 32'd0;
            judge_count   <= 16'd0;
            ad0           <= 12'd0;
            ad1           <= 12'd0;
            ad2           <= 12'd0;
            target_command<= 16'sd0;
            command       <= 16'sd0;
            mode          <= MODE_IDLE;
            balancing     <= 1'b0;
            armed         <= 1'b0;
            capture_clear <= 1'b0;
        end else if (!run_enable) begin
            state         <= S_IDLE;
            run_start     <= 1'b0;
            startup_count <= 32'd0;
            phase_count   <= 32'd0;
            judge_count   <= 16'd0;
            target_command<= 16'sd0;
            command       <= 16'sd0;
            mode          <= MODE_IDLE;
            balancing     <= 1'b0;
            armed         <= 1'b0;
            capture_clear <= 1'b0;
        end else if (motor_fault) begin
            state         <= S_IDLE;
            run_start     <= 1'b0;
            target_command<= 16'sd0;
            command       <= 16'sd0;
            mode          <= MODE_FAULT;
            balancing     <= 1'b0;
            armed         <= 1'b0;
            capture_clear <= 1'b0;
        end else begin
            if (run_enable && !run_prev)
                run_start <= 1'b1;
            capture_clear <= 1'b0;
            if (tick_1khz) begin
                if (run_start) begin
                    if (startup_count < STARTUP_HOLD_MS) begin
                        startup_count <= startup_count + 1'b1;
                        state         <= S_IDLE;
                        mode          <= MODE_IDLE;
                        balancing     <= 1'b0;
                        armed         <= 1'b0;
                        phase_count   <= 32'd0;
                        judge_count   <= 16'd0;
                        target_command<= 16'sd0;
                    end else begin
                        run_start     <= 1'b0;
                        armed         <= 1'b1;
                        state         <= S_L3;
                        mode          <= MODE_SWING;
                        balancing     <= 1'b0;
                        phase_count   <= 32'd0;
                        judge_count   <= 16'd0;
                    end
                end else begin
                    case (state)
                        S_IDLE: begin
                            // Mode31 stop state: hold a zero command and wait
                            // for a new K1 run_request edge.
                            state         <= S_IDLE;
                            mode          <= MODE_IDLE;
                            balancing     <= 1'b0;
                            target_command<= 16'sd0;
                        end

                        S_JUDGE: begin
                            if (judge_count >= 39) begin
                                judge_count <= 16'd0;
                                ad0 <= n_ad0;
                                ad1 <= n_ad1;
                                ad2 <= n_ad2;
                                if (n_capture) begin
                                    capture_clear <= 1'b1;
                                    state         <= S_BALANCE;
                                    mode          <= MODE_BALANCE;
                                    balancing     <= 1'b1;
                                    phase_count   <= 32'd0;
                                    target_command<= pid_command_eff;
                                end else if (n_right) begin
                                    state         <= S_L1;
                                    mode          <= MODE_SWING;
                                    balancing     <= 1'b0;
                                end else if (n_left) begin
                                    state         <= S_R1;
                                    mode          <= MODE_SWING;
                                    balancing     <= 1'b0;
                                end
                            end else begin
                                judge_count <= judge_count + 1'b1;
                            end
                        end

                        S_L1: begin
                            target_command<= start_pwm_eff;
                            phase_count   <= start_time_eff;
                            state         <= S_L2;
                            mode          <= MODE_SWING;
                            balancing     <= 1'b0;
                        end
                        S_L2: begin
                            if (phase_count == 0)
                                state <= S_L3;
                            else
                                phase_count <= phase_count - 1'b1;
                        end
                        S_L3: begin
                            target_command<= -start_pwm_eff;
                            phase_count   <= start_time_eff;
                            state         <= S_L4;
                            mode          <= MODE_SWING;
                            balancing     <= 1'b0;
                        end
                        S_L4: begin
                            if (phase_count == 0) begin
                                target_command<= 16'sd0;
                                state         <= S_JUDGE;
                                judge_count   <= 16'd0;
                            end else begin
                                phase_count <= phase_count - 1'b1;
                            end
                        end

                        S_R1: begin
                            target_command<= -start_pwm_eff;
                            phase_count   <= start_time_eff;
                            state         <= S_R2;
                            mode          <= MODE_SWING;
                            balancing     <= 1'b0;
                        end
                        S_R2: begin
                            if (phase_count == 0)
                                state <= S_R3;
                            else
                                phase_count <= phase_count - 1'b1;
                        end
                        S_R3: begin
                            target_command<= start_pwm_eff;
                            phase_count   <= start_time_eff;
                            state         <= S_R4;
                            mode          <= MODE_SWING;
                            balancing     <= 1'b0;
                        end
                        S_R4: begin
                            if (phase_count == 0) begin
                                target_command<= 16'sd0;
                                state         <= S_JUDGE;
                                judge_count   <= 16'd0;
                            end else begin
                                phase_count <= phase_count - 1'b1;
                            end
                        end

                        S_BALANCE: begin
                            mode      <= MODE_BALANCE;
                            balancing <= 1'b1;
                            target_command <= pid_command_eff;
                            if (n_fallen) begin
                                state         <= S_IDLE;
                                mode          <= MODE_IDLE;
                                balancing     <= 1'b0;
                                target_command<= 16'sd0;
                                phase_count   <= 32'd0;
                                judge_count   <= 16'd0;
                            end
                        end

                        default: begin
                            state         <= S_IDLE;
                            mode          <= MODE_FAULT;
                            balancing     <= 1'b0;
                            armed         <= 1'b0;
                            target_command<= 16'sd0;
                        end
                    endcase
                end

                // Ramp the actual bridge command toward the state machine
                // target by at most COMMAND_SLEW_STEP percentage points per
                // control tick.  Reset, run-disable and fault branches above
                // still force an immediate zero.
                if (target_command > command) begin
                    if ((target_command - command) > COMMAND_SLEW_STEP)
                        command <= command + COMMAND_SLEW_STEP;
                    else
                        command <= target_command;
                end else if (target_command < command) begin
                    if ((command - target_command) > COMMAND_SLEW_STEP)
                        command <= command - COMMAND_SLEW_STEP;
                    else
                        command <= target_command;
                end else begin
                    command <= target_command;
                end
            end
        end
    end
endmodule
