`timescale 1ns/1ps

// Convert the PCB angle sensor's absolute 0..360-degree ADC value into a
// signed error relative to the STM32 CENTER_ANGLE parameter.  The result is
// the shortest circular difference, so the 0/360 seam cannot look like a
// 180-degree jump.  angle_deg_x10 is in tenths of a degree; error_counts is
// retained for a direct STM32-compatible PID implementation.
module angle_center_map #(
    parameter integer ADC_SPAN_COUNTS = 3974
) (
    input  wire [11:0] adc,
    input  wire [11:0] center_angle,
    output reg signed [15:0] error_counts,
    output reg signed [15:0] angle_deg_x10
);
    integer diff;
    integer clipped;
    integer half_span;
    integer scaled;
    always @* begin
        clipped = adc;
        if (clipped > ADC_SPAN_COUNTS) clipped = ADC_SPAN_COUNTS;
        diff = clipped - center_angle;
        half_span = ADC_SPAN_COUNTS / 2;
        if (diff > half_span)
            diff = diff - ADC_SPAN_COUNTS;
        else if (diff < -half_span)
            diff = diff + ADC_SPAN_COUNTS;
        if (diff > 32767) diff = 32767;
        if (diff < -32768) diff = -32768;
        error_counts = diff;
        scaled = (diff * 3600) / ADC_SPAN_COUNTS;
        if (scaled > 32767) scaled = 32767;
        if (scaled < -32768) scaled = -32768;
        angle_deg_x10 = scaled;
    end
endmodule
