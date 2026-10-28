`timescale 1ns/1ps

// Four active-low push-buttons used by the STM32-compatible PCB.
//
// The STM32 reference samples the switches every 20 ms and reports a click
// after release, or a long press after 1 s.  This module keeps the same user
// visible timing while synchronizing the asynchronous package pins to the PL
// clock.  click and long_press are one-clock pulses; long_press is emitted
// once while the key remains held and suppresses the corresponding click.
module key_events #(
    parameter integer CLK_HZ = 50000000,
    parameter integer SAMPLE_HZ = 1000,
    parameter integer DEBOUNCE_MS = 20,
    parameter integer LONG_MS = 1000
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [3:0] key_n,
    output reg  [3:0] click,
    output reg  [3:0] long_press,
    output reg  [3:0] pressed,
    output reg        tick_1ms
);
    localparam integer SAMPLE_DIV = (CLK_HZ / SAMPLE_HZ < 1) ? 1 : CLK_HZ / SAMPLE_HZ;
    localparam integer DB_SAMPLES = (DEBOUNCE_MS < 1) ? 1 : DEBOUNCE_MS;
    localparam integer LONG_SAMPLES = (LONG_MS < 1) ? 1 : LONG_MS;
    localparam integer DB_W = (DB_SAMPLES <= 1) ? 1 : $clog2(DB_SAMPLES + 1);
    localparam integer LONG_W = (LONG_SAMPLES <= 1) ? 1 : $clog2(LONG_SAMPLES + 1);

    reg [31:0] sample_count;
    reg [3:0] key_meta;
    reg [3:0] key_sync;
    reg [3:0] stable_pressed;
    reg [DB_W-1:0] db_count [0:3];
    reg [LONG_W-1:0] hold_count [0:3];
    reg [3:0] long_sent;
    integer i;

    always @(posedge clk) begin
        if (!rst_n) begin
            sample_count <= 32'd0;
            key_meta <= 4'b1111;
            key_sync <= 4'b1111;
            stable_pressed <= 4'b0000;
            pressed <= 4'b0000;
            click <= 4'b0000;
            long_press <= 4'b0000;
            long_sent <= 4'b0000;
            tick_1ms <= 1'b0;
            for (i = 0; i < 4; i = i + 1) begin
                db_count[i] <= {DB_W{1'b0}};
                hold_count[i] <= {LONG_W{1'b0}};
            end
        end else begin
            key_meta <= key_n;
            key_sync <= key_meta;
            click <= 4'b0000;
            long_press <= 4'b0000;
            tick_1ms <= 1'b0;

            if (sample_count >= SAMPLE_DIV - 1) begin
                sample_count <= 32'd0;
                tick_1ms <= 1'b1;

                for (i = 0; i < 4; i = i + 1) begin
                    // Debounce by requiring the synchronized level to remain
                    // unchanged for DB_SAMPLES successive 1 ms samples.
                    if (key_sync[i] == (stable_pressed[i] ? 1'b0 : 1'b1)) begin
                        db_count[i] <= {DB_W{1'b0}};
                    end else if (db_count[i] < DB_SAMPLES) begin
                        db_count[i] <= db_count[i] + 1'b1;
                    end

                    if ((key_sync[i] == 1'b0) && !stable_pressed[i] &&
                        (db_count[i] >= DB_SAMPLES - 1)) begin
                        stable_pressed[i] <= 1'b1;
                        pressed[i] <= 1'b1;
                        hold_count[i] <= {LONG_W{1'b0}};
                        long_sent[i] <= 1'b0;
                    end else if ((key_sync[i] == 1'b1) && stable_pressed[i] &&
                                 (db_count[i] >= DB_SAMPLES - 1)) begin
                        stable_pressed[i] <= 1'b0;
                        pressed[i] <= 1'b0;
                        if (!long_sent[i])
                            click[i] <= 1'b1;
                        hold_count[i] <= {LONG_W{1'b0}};
                        long_sent[i] <= 1'b0;
                    end else if (stable_pressed[i]) begin
                        if (!long_sent[i]) begin
                            if (hold_count[i] < LONG_SAMPLES)
                                hold_count[i] <= hold_count[i] + 1'b1;
                            if (hold_count[i] >= LONG_SAMPLES - 1) begin
                                long_press[i] <= 1'b1;
                                long_sent[i] <= 1'b1;
                            end
                        end
                    end
                end
            end else begin
                sample_count <= sample_count + 1'b1;
            end
        end
    end
endmodule
