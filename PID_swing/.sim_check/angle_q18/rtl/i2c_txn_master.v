`timescale 1ns/1ps

// Open-drain I2C master. Each transaction contains one to four bytes.
module i2c_txn_master #(
    parameter integer CLK_HZ = 50000000,
    parameter integer I2C_HZ = 400000
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       start,
    input  wire [2:0] length,
    input  wire [7:0] data0,
    input  wire [7:0] data1,
    input  wire [7:0] data2,
    input  wire [7:0] data3,
    output reg        busy,
    output reg        done,
    output reg        ack_error,
    output reg        scl_low,
    output reg        sda_low,
    input  wire       sda_in
);
    localparam integer HALF_DIV = (CLK_HZ / (I2C_HZ * 2));
    localparam [3:0] ST_IDLE      = 4'd0;
    localparam [3:0] ST_START     = 4'd1;
    localparam [3:0] ST_START_LOW = 4'd2;
    localparam [3:0] ST_BIT_LOW   = 4'd3;
    localparam [3:0] ST_BIT_HIGH  = 4'd4;
    localparam [3:0] ST_ACK_LOW   = 4'd5;
    localparam [3:0] ST_ACK_HIGH  = 4'd6;
    localparam [3:0] ST_STOP_LOW  = 4'd7;
    localparam [3:0] ST_STOP_HIGH = 4'd8;
    localparam [3:0] ST_DONE      = 4'd9;
    localparam [3:0] ST_NEXT_BYTE = 4'd10;
    localparam [3:0] ST_STOP_SCL_HIGH = 4'd11;

    reg [3:0] state;
    reg [31:0] div_count;
    reg [2:0] length_reg;
    reg [2:0] byte_index;
    reg [3:0] bit_index;
    reg [7:0] tx0, tx1, tx2, tx3;
    reg [7:0] current_byte;
    reg sda_meta;
    reg sda_sync;

    always @* begin
        case (byte_index)
            3'd0: current_byte = tx0;
            3'd1: current_byte = tx1;
            3'd2: current_byte = tx2;
            default: current_byte = tx3;
        endcase
    end

    always @(posedge clk) begin
        sda_meta <= sda_in;
        sda_sync <= sda_meta;
        if (!rst_n) begin
            state      <= ST_IDLE;
            div_count  <= 32'd0;
            scl_low    <= 1'b0;
            sda_low    <= 1'b0;
            length_reg <= 3'd0;
            byte_index <= 3'd0;
            bit_index  <= 4'd7;
            tx0        <= 8'd0;
            tx1        <= 8'd0;
            tx2        <= 8'd0;
            tx3        <= 8'd0;
            busy       <= 1'b0;
            done       <= 1'b0;
            ack_error  <= 1'b0;
        end else begin
            done <= 1'b0;
            // Accept a one-clock start pulse immediately; otherwise the
            // divider could miss it while waiting for its next half-period.
            if (state == ST_IDLE && start && (length != 0)) begin
                tx0        <= data0;
                tx1        <= data1;
                tx2        <= data2;
                tx3        <= data3;
                length_reg <= (length > 4) ? 3'd4 : length;
                byte_index <= 3'd0;
                bit_index  <= 4'd7;
                busy       <= 1'b1;
                ack_error  <= 1'b0;
                state      <= ST_START;
                div_count  <= 32'd0;
            end else if (div_count < HALF_DIV - 1)
                div_count <= div_count + 1'b1;
            else begin
                div_count <= 32'd0;
                case (state)
                    ST_IDLE: begin
                        scl_low <= 1'b0;
                        sda_low <= 1'b0;
                        busy    <= 1'b0;
                    end
                    ST_START: begin
                        scl_low <= 1'b0;
                        sda_low <= 1'b1;
                        state   <= ST_START_LOW;
                    end
                    ST_START_LOW: begin
                        scl_low <= 1'b1;
                        sda_low <= (current_byte[bit_index] == 1'b0);
                        state   <= ST_BIT_LOW;
                    end
                    ST_BIT_LOW: begin
                        scl_low <= 1'b0;
                        state   <= ST_BIT_HIGH;
                    end
                    ST_BIT_HIGH: begin
                        scl_low <= 1'b1;
                        if (bit_index == 0) begin
                            state <= ST_ACK_LOW;
                        end else begin
                            bit_index <= bit_index - 1'b1;
                            sda_low   <= (current_byte[bit_index - 1'b1] == 1'b0);
                            state     <= ST_BIT_LOW;
                        end
                    end
                    ST_ACK_LOW: begin
                        sda_low <= 1'b0;
                        scl_low <= 1'b0;
                        state   <= ST_ACK_HIGH;
                    end
                    ST_ACK_HIGH: begin
                        scl_low <= 1'b1;
                        if (sda_sync == 1'b1)
                            ack_error <= 1'b1;
                        if (byte_index + 1'b1 >= length_reg) begin
                            state <= ST_STOP_LOW;
                        end else begin
                            byte_index <= byte_index + 1'b1;
                            bit_index  <= 4'd7;
                            state      <= ST_NEXT_BYTE;
                        end
                    end
                    ST_NEXT_BYTE: begin
                        // byte_index has updated, so current_byte now
                        // selects the next byte while SCL stays low.
                        scl_low <= 1'b1;
                        sda_low <= (current_byte[7] == 1'b0);
                        state   <= ST_BIT_LOW;
                    end
                    ST_STOP_LOW: begin
                        scl_low <= 1'b1;
                        sda_low <= 1'b1;
                        state   <= ST_STOP_SCL_HIGH;
                    end
                    ST_STOP_SCL_HIGH: begin
                        scl_low <= 1'b0;
                        sda_low <= 1'b1;
                        state   <= ST_STOP_HIGH;
                    end
                    ST_STOP_HIGH: begin
                        scl_low <= 1'b0;
                        sda_low <= 1'b0;
                        state   <= ST_DONE;
                    end
                    ST_DONE: begin
                        busy  <= 1'b0;
                        done  <= 1'b1;
                        state <= ST_IDLE;
                    end
                    default: state <= ST_IDLE;
                endcase
            end
        end
    end
endmodule
