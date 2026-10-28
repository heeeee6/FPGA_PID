`timescale 1ns/1ps

// Minimal open-drain I2C master used for the ADS1115 converters.
//   write: START, tx0 (address+W), tx1.. (tx_count bytes in total), STOP
//   read : START, tx0 (address+R), read two bytes (ACK, then NACK), STOP
// tx_count counts the bytes sent by the master including the address byte
// (1..4); a read uses tx_count = 1.  scl_low/sda_low are open-drain
// "pull low" requests, mapped to IOBUFs by the top level.  A NACK from the
// slave aborts the transaction (STOP) and raises nack.  Clock stretching is
// not supported (ADS1115 does not stretch).
module i2c_ads_master #(
    parameter integer CLK_HZ = 50000000,
    parameter integer I2C_HZ = 400000
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire        is_read,
    input  wire [2:0]  tx_count,
    input  wire [7:0]  tx0,
    input  wire [7:0]  tx1,
    input  wire [7:0]  tx2,
    input  wire [7:0]  tx3,
    output reg         busy,
    output reg         done,
    output reg         nack,
    output reg  [15:0] rx_data,
    output reg         scl_low,
    output reg         sda_low,
    input  wire        sda_in
);
    localparam integer QDIV_RAW = CLK_HZ / (I2C_HZ * 4);
    localparam integer QDIV = (QDIV_RAW < 1) ? 1 : QDIV_RAW;

    localparam [2:0] ST_IDLE  = 3'd0;
    localparam [2:0] ST_START = 3'd1;
    localparam [2:0] ST_TX    = 3'd2;
    localparam [2:0] ST_TXACK = 3'd3;
    localparam [2:0] ST_RX    = 3'd4;
    localparam [2:0] ST_RXACK = 3'd5;
    localparam [2:0] ST_STOP  = 3'd6;
    localparam [2:0] ST_FIN   = 3'd7;

    reg [2:0]  state;
    reg [31:0] qcnt;
    reg [1:0]  q;
    reg [2:0]  bit_idx;
    reg [2:0]  byte_idx;
    reg        rx_second;
    reg [2:0]  count_reg;
    reg        read_reg;
    reg [7:0]  t0, t1, t2, t3;
    reg [15:0] shift;
    reg        sda_meta, sda_sync;
    reg [7:0]  cur_byte;

    always @* begin
        case (byte_idx)
            3'd0:    cur_byte = t0;
            3'd1:    cur_byte = t1;
            3'd2:    cur_byte = t2;
            default: cur_byte = t3;
        endcase
    end

    always @(posedge clk) begin
        sda_meta <= sda_in;
        sda_sync <= sda_meta;
        done     <= 1'b0;
        if (!rst_n) begin
            state     <= ST_IDLE;
            qcnt      <= 32'd0;
            q         <= 2'd0;
            bit_idx   <= 3'd7;
            byte_idx  <= 3'd0;
            rx_second <= 1'b0;
            count_reg <= 3'd1;
            read_reg  <= 1'b0;
            t0 <= 8'd0; t1 <= 8'd0; t2 <= 8'd0; t3 <= 8'd0;
            shift     <= 16'd0;
            rx_data   <= 16'd0;
            busy      <= 1'b0;
            nack      <= 1'b0;
            scl_low   <= 1'b0;
            sda_low   <= 1'b0;
        end else if (state == ST_IDLE) begin
            scl_low <= 1'b0;
            sda_low <= 1'b0;
            qcnt    <= 32'd0;
            q       <= 2'd0;
            if (start && (tx_count != 3'd0)) begin
                t0 <= tx0; t1 <= tx1; t2 <= tx2; t3 <= tx3;
                count_reg <= (tx_count > 3'd4) ? 3'd4 : tx_count;
                read_reg  <= is_read;
                byte_idx  <= 3'd0;
                bit_idx   <= 3'd7;
                rx_second <= 1'b0;
                busy      <= 1'b1;
                nack      <= 1'b0;
                state     <= ST_START;
            end
        end else if (qcnt < QDIV - 1) begin
            qcnt <= qcnt + 1'b1;
        end else begin
            qcnt <= 32'd0;
            q    <= q + 1'b1;
            case (state)
                ST_START: begin
                    case (q)
                        2'd0: begin scl_low <= 1'b0; sda_low <= 1'b0; end
                        2'd1: sda_low <= 1'b1;
                        2'd3: begin scl_low <= 1'b1; state <= ST_TX; bit_idx <= 3'd7; end
                        default: begin end
                    endcase
                end
                ST_TX: begin
                    case (q)
                        2'd0: sda_low <= ~cur_byte[bit_idx];
                        2'd1: scl_low <= 1'b0;
                        2'd3: begin
                            scl_low <= 1'b1;
                            if (bit_idx == 3'd0) state <= ST_TXACK;
                            else bit_idx <= bit_idx - 1'b1;
                        end
                        default: begin end
                    endcase
                end
                ST_TXACK: begin
                    case (q)
                        2'd0: sda_low <= 1'b0;
                        2'd1: scl_low <= 1'b0;
                        2'd2: if (sda_sync) nack <= 1'b1;
                        2'd3: begin
                            scl_low <= 1'b1;
                            if (nack) begin
                                state <= ST_STOP;
                            end else if ((byte_idx + 3'd1) < count_reg) begin
                                byte_idx <= byte_idx + 1'b1;
                                bit_idx  <= 3'd7;
                                state    <= ST_TX;
                            end else if (read_reg) begin
                                bit_idx   <= 3'd7;
                                rx_second <= 1'b0;
                                state     <= ST_RX;
                            end else begin
                                state <= ST_STOP;
                            end
                        end
                        default: begin end
                    endcase
                end
                ST_RX: begin
                    case (q)
                        2'd0: sda_low <= 1'b0;
                        2'd1: scl_low <= 1'b0;
                        2'd2: shift <= {shift[14:0], sda_sync};
                        2'd3: begin
                            scl_low <= 1'b1;
                            if (bit_idx == 3'd0) state <= ST_RXACK;
                            else bit_idx <= bit_idx - 1'b1;
                        end
                        default: begin end
                    endcase
                end
                ST_RXACK: begin
                    case (q)
                        2'd0: sda_low <= ~rx_second;   // ACK byte 1, NACK byte 2
                        2'd1: scl_low <= 1'b0;
                        2'd3: begin
                            scl_low <= 1'b1;
                            if (!rx_second) begin
                                rx_second <= 1'b1;
                                bit_idx   <= 3'd7;
                                state     <= ST_RX;
                            end else begin
                                state <= ST_STOP;
                            end
                        end
                        default: begin end
                    endcase
                end
                ST_STOP: begin
                    case (q)
                        2'd0: begin sda_low <= 1'b1; scl_low <= 1'b1; end
                        2'd1: scl_low <= 1'b0;
                        2'd2: sda_low <= 1'b0;
                        2'd3: state <= ST_FIN;
                        default: begin end
                    endcase
                end
                default: begin
                    rx_data <= shift;
                    busy    <= 1'b0;
                    done    <= 1'b1;
                    state   <= ST_IDLE;
                end
            endcase
        end
    end
endmodule
