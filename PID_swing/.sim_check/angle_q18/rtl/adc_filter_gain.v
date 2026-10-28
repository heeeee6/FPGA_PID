`timescale 1ns/1ps

// Small integer low-pass filter for a 12-bit XADC result.  The input is
// sampled at sample_en (1 kHz by default); no clock is assumed on the ADC
// side, so this module is also suitable for a bridged external ADC.
module adc_filter_12 #(
    parameter integer FILTER_SHIFT = 3
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       sample_en,
    input  wire [11:0] raw,
    output reg  [11:0] filtered
);
    integer diff;
    integer step;

    always @(posedge clk) begin
        if (!rst_n) begin
            filtered <= 12'd0;
        end else if (sample_en) begin
            // Both operands are unsigned vectors.  Split the subtraction so
            // a downward step cannot wrap around before it is assigned to the
            // signed integer temporary.
            if (raw >= filtered)
                diff = raw - filtered;
            else
                diff = -(filtered - raw);
            step = diff >>> FILTER_SHIFT;
            // Do not leave the filter one count away forever.
            if ((step == 0) && (diff > 0))  step = 1;
            if ((step == 0) && (diff < 0))  step = -1;
            if (step > 2047)  step = 2047;
            if (step < -2048) step = -2048;
            filtered <= filtered + step;
        end
    end
endmodule

// Convert a 0..4095 angle sample to a signed angle in tenths of a degree.
// 0 V is the user's mechanical zero.  Values above 180 degrees are wrapped
// negative so that the balance controller sees the shortest angular error.
module angle_from_adc #(
    parameter integer ADC_SPAN_COUNTS = 3974
) (
    input  wire [11:0] adc,
    output reg signed [15:0] angle_deg_x10
);
    // For the default calibrated span, ceil(3600 * 2^22 / 3974) is an
    // exact reciprocal over every integer input from 0 through 3974 after
    // the right shift. This replaces a large variable integer divider.
    localparam [17:0] RECIP_Q18 = 18'd237473;
    localparam integer ANGLE_WRAP_ADC = 1989;
    reg [11:0] clipped;
    reg [29:0] product;
    reg [11:0] scaled;
    always @* begin
        clipped = (adc > ADC_SPAN_COUNTS) ? ADC_SPAN_COUNTS[11:0] : adc;
        if (ADC_SPAN_COUNTS == 3974) begin
            product = {18'd0, clipped} * RECIP_Q18;
            scaled = product[29:18];
        end else begin
            product = 34'd0;
            scaled = (clipped * 3600) / ADC_SPAN_COUNTS;
        end
        if (clipped >= ANGLE_WRAP_ADC)
            angle_deg_x10 = $signed({1'b0, scaled}) - 16'sd3600;
        else
            angle_deg_x10 = $signed({1'b0, scaled});
    end
endmodule

// Pipelined angle conversion for the live control path.  The first stage is
// a constant reciprocal multiply; the second stage performs the 0/360-degree
// wrap.  The output is updated every clock so pid_swing_top can sample it at
// the 1 kHz scheduler boundary without adding a full control-period delay.
module angle_from_adc_pipe #(
    parameter integer ADC_SPAN_COUNTS = 3974
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               update,
    input  wire [11:0]        adc,
    output reg signed [15:0]  angle_deg_x10
);
    localparam [17:0] RECIP_Q18 = 18'd237473;
    localparam [29:0] ANGLE_WRAP_PRODUCT = 30'd1989 * RECIP_Q18;
    wire [11:0] clipped = (adc > ADC_SPAN_COUNTS) ? ADC_SPAN_COUNTS[11:0] : adc;
    reg [29:0] product_reg;

    always @(posedge clk) begin
        if (!rst_n) begin
            product_reg    <= 30'd0;
            angle_deg_x10  <= 16'sd0;
        end else begin
            if (ADC_SPAN_COUNTS == 3974)
                product_reg <= {18'd0, clipped} * RECIP_Q18;
            else
                product_reg <= (((clipped * 3600) / ADC_SPAN_COUNTS) << 18);
            // Keep this register free-running.  If it were gated by update,
            // the outer control register would sample the previous 1 kHz
            // value because both always blocks use nonblocking assignments.
                if (product_reg >= ANGLE_WRAP_PRODUCT)
                    angle_deg_x10 <= $signed({1'b0, product_reg[29:18]}) - 16'sd3600;
            else
                    angle_deg_x10 <= $signed({1'b0, product_reg[29:18]});
        end
    end
endmodule

// Q8.8 coefficient map used by pid_fixed.  The maxima match the scale of
// the reference STM32 project while leaving enough range for commissioning.
module gain_from_adc #(
    parameter integer MAX_Q8 = 2048,
    parameter integer ADC_SPAN_COUNTS = 3974
) (
    input  wire [11:0] adc,
    output reg  [15:0] gain_q8,
    output reg  [15:0] gain_x100
);
    integer scaled_q8;
    integer scaled_x100;
    integer clipped;
    always @* begin
        clipped     = (adc > ADC_SPAN_COUNTS) ? ADC_SPAN_COUNTS : adc;
        scaled_q8   = (clipped * MAX_Q8) / ADC_SPAN_COUNTS;
        scaled_x100 = (scaled_q8 * 100) / 256;
        if (scaled_q8 < 0) scaled_q8 = 0;
        if (scaled_q8 > 65535) scaled_q8 = 65535;
        if (scaled_x100 < 0) scaled_x100 = 0;
        if (scaled_x100 > 65535) scaled_x100 = 65535;
        gain_q8   = scaled_q8;
        gain_x100 = scaled_x100;
    end
endmodule
