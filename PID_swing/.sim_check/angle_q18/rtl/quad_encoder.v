`timescale 1ns/1ps

// 4x quadrature decoder for the JGA25-370 encoder.
// The output position is a signed count.  velocity is the count delta
// between two sample_en events, which is convenient for the outer loop.
module quad_encoder #(
    parameter integer SAMPLE_WIDTH = 32
) (
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         enc_a,
    input  wire                         enc_b,
    input  wire                         sample_en,
    output reg signed [SAMPLE_WIDTH-1:0] position,
    output reg signed [SAMPLE_WIDTH-1:0] velocity
);

    reg a_meta, a_sync, a_last;
    reg b_meta, b_sync, b_last;
    reg signed [SAMPLE_WIDTH-1:0] position_last;
    reg signed [SAMPLE_WIDTH-1:0] position_next;
    reg signed [SAMPLE_WIDTH-1:0] delta;

    always @* begin
        position_next = position;
        case ({a_last, b_last, a_sync, b_sync})
            4'b0001, 4'b0111, 4'b1110, 4'b1000: position_next = position + 1;
            4'b0010, 4'b1011, 4'b1101, 4'b0100: position_next = position - 1;
            default: position_next = position;
        endcase
        delta = position_next - position_last;
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            a_meta       <= 1'b0;
            a_sync       <= 1'b0;
            a_last       <= 1'b0;
            b_meta       <= 1'b0;
            b_sync       <= 1'b0;
            b_last       <= 1'b0;
            position     <= {SAMPLE_WIDTH{1'b0}};
            position_last<= {SAMPLE_WIDTH{1'b0}};
            velocity     <= {SAMPLE_WIDTH{1'b0}};
        end else begin
            a_meta <= enc_a;
            a_sync <= a_meta;
            b_meta <= enc_b;
            b_sync <= b_meta;

            // Decode one state transition each clock.  Illegal transitions
            // caused by noise are ignored by the default branch.
            a_last   <= a_sync;
            b_last   <= b_sync;
            position <= position_next;

            if (sample_en) begin
                velocity      <= delta;
                position_last <= position_next;
            end
        end
    end

endmodule
