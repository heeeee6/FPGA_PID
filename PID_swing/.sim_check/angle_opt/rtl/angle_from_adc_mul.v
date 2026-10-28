`timescale 1ns/1ps

// Constant-reciprocal angle map used for timing experiments.  The original
// map inferred a variable divider (a long CARRY chain).  This implementation
// replaces it with one 12x12 constant multiply followed by a binary shift.
// For ADC_SPAN_COUNTS=3974, Q12 gain 3711 gives a maximum magnitude error of
// one output LSB (0.1 degree) over the complete 12-bit input range.
module angle_from_adc_mul #(
    parameter integer ADC_SPAN_COUNTS = 3974,
    parameter integer ANGLE_SHIFT = 12
) (
    input  wire [11:0] adc,
    output reg signed [15:0] angle_deg_x10
);
    // Rounded reciprocal coefficient: round(3600*2^ANGLE_SHIFT/span).
    // The bounded calibration span used by this project keeps this within
    // 12 bits, allowing Vivado to map the multiply directly into DSP48E1.
    localparam [11:0] ANGLE_GAIN_Q12 =
        (3600 * (1 << ANGLE_SHIFT) + ADC_SPAN_COUNTS / 2) /
        ADC_SPAN_COUNTS;
    // floor(adc*3600/span) first exceeds 1800 at this input count.
    localparam integer ANGLE_WRAP_ADC =
        (1801 * ADC_SPAN_COUNTS + 3599) / 3600;

    reg [11:0] clipped;
    wire [23:0] product = clipped * ANGLE_GAIN_Q12;
    wire [11:0] scaled_positive = product >> ANGLE_SHIFT;

    always @* begin
        if (adc > ADC_SPAN_COUNTS)
            clipped = ADC_SPAN_COUNTS;
        else
            clipped = adc;

        // Use the input-domain wrap threshold rather than comparing the
        // approximated quotient.  This preserves the exact +/-180 degree
        // transition even when the reciprocal multiply is one count high.
        if (clipped >= ANGLE_WRAP_ADC)
            angle_deg_x10 = {4'b0000, scaled_positive} - 16'sd3600;
        else
            angle_deg_x10 = {4'b0000, scaled_positive};
    end
endmodule
