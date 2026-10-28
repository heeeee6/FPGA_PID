`timescale 1ns/1ps

// Minimal SSD1306 128x64 text driver. It refreshes four 6x8 text rows:
// KP, KI, KD and MODE. The I2C master sends the display address (0x78),
// control byte and one command/data byte in each transaction.
module oled_ssd1306 #(
    parameter integer CLK_HZ = 50000000,
    parameter integer I2C_HZ = 400000,
    parameter integer STARTUP_CYCLES = 2500000,
    parameter integer REFRESH_CYCLES = 25000000
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [15:0] kp_x100,
    input  wire [15:0] ki_x100,
    input  wire [15:0] kd_x100,
    input  wire [1:0] mode,
    output wire       oled_scl_low,
    output wire       oled_sda_low,
    input  wire       oled_sda_in
);
    localparam [4:0] ST_RESET_WAIT  = 5'd0;
    localparam [4:0] ST_INIT_START  = 5'd1;
    localparam [4:0] ST_INIT_WAIT   = 5'd2;
    localparam [4:0] ST_REFRESH_WAIT= 5'd3;
    localparam [4:0] ST_PAGE_START  = 5'd4;
    localparam [4:0] ST_PAGE_WAIT   = 5'd5;
    localparam [4:0] ST_LOW_START   = 5'd6;
    localparam [4:0] ST_LOW_WAIT    = 5'd7;
    localparam [4:0] ST_HIGH_START  = 5'd8;
    localparam [4:0] ST_HIGH_WAIT   = 5'd9;
    localparam [4:0] ST_DATA_START  = 5'd10;
    localparam [4:0] ST_DATA_WAIT   = 5'd11;
    localparam [4:0] ST_BCD_LOAD    = 5'd12;
    localparam [4:0] ST_BCD_SHIFT   = 5'd13;

    reg [4:0] state;
    reg [31:0] wait_count;
    reg [4:0] init_index;
    reg [1:0] line_index;
    reg [3:0] char_index;
    reg [2:0] glyph_index;
    reg i2c_start;
    reg [2:0] i2c_length;
    reg [7:0] i2c_data0, i2c_data1, i2c_data2, i2c_data3;
    wire i2c_busy;
    wire i2c_done;
    wire i2c_ack_error;
    reg [7:0] current_char;
    reg [7:0] glyph_data;
    reg [15:0] kp_shift, ki_shift, kd_shift;
    reg [15:0] kp_bcd_work, ki_bcd_work, kd_bcd_work;
    reg [15:0] kp_bcd, ki_bcd, kd_bcd;
    reg [4:0]  bcd_count;
    reg [1:0]  mode_snapshot;
    wire [15:0] kp_bcd_next;
    wire [15:0] ki_bcd_next;
    wire [15:0] kd_bcd_next;

    i2c_txn_master #(
        .CLK_HZ(CLK_HZ), .I2C_HZ(I2C_HZ)
    ) u_i2c (
        .clk(clk), .rst_n(rst_n), .start(i2c_start), .length(i2c_length),
        .data0(i2c_data0), .data1(i2c_data1), .data2(i2c_data2), .data3(i2c_data3),
        .busy(i2c_busy), .done(i2c_done), .ack_error(i2c_ack_error),
        .scl_low(oled_scl_low), .sda_low(oled_sda_low),
        .sda_in(oled_sda_in)
    );

    function [7:0] init_command;
        input [4:0] index;
        begin
            case (index)
                4'd0: init_command = 8'hAE;
                4'd1: init_command = 8'hD5;
                4'd2: init_command = 8'h80;
                4'd3: init_command = 8'hA8;
                4'd4: init_command = 8'h3F;
                4'd5: init_command = 8'hD3;
                4'd6: init_command = 8'h00;
                4'd7: init_command = 8'h40;
                4'd8: init_command = 8'h8D;
                4'd9: init_command = 8'h14;
                4'd10: init_command = 8'h20;
                4'd11: init_command = 8'h02;
                4'd12: init_command = 8'hA1;
                4'd13: init_command = 8'hC8;
                4'd14: init_command = 8'hDA;
                4'd15: init_command = 8'h12;
                5'd16: init_command = 8'hAF;
                default: init_command = 8'hAF;
            endcase
        end
    endfunction

    function [7:0] decimal_char;
        input integer number;
        begin
            if (number < 0)
                decimal_char = "0";
            else if (number > 9)
                decimal_char = "9";
            else
                decimal_char = 8'h30 + number;
        end
    endfunction

    function [7:0] text_char;
        input [1:0] row;
        input [3:0] index;
        input [15:0] kp;
        input [15:0] ki;
        input [15:0] kd;
        input [1:0] current_mode;
        integer value;
        begin
            value = 0;
            case (row)
                2'd0: begin
                    value = kp;
                    case (index)
                        4'd0: text_char = "K";
                        4'd1: text_char = "P";
                        4'd2: text_char = ":";
                        4'd3: text_char = decimal_char(value / 1000);
                        4'd4: text_char = decimal_char((value / 100) % 10);
                        4'd5: text_char = ".";
                        4'd6: text_char = decimal_char((value / 10) % 10);
                        4'd7: text_char = decimal_char(value % 10);
                        default: text_char = " ";
                    endcase
                end
                2'd1: begin
                    value = ki;
                    case (index)
                        4'd0: text_char = "K";
                        4'd1: text_char = "I";
                        4'd2: text_char = ":";
                        4'd3: text_char = decimal_char(value / 1000);
                        4'd4: text_char = decimal_char((value / 100) % 10);
                        4'd5: text_char = ".";
                        4'd6: text_char = decimal_char((value / 10) % 10);
                        4'd7: text_char = decimal_char(value % 10);
                        default: text_char = " ";
                    endcase
                end
                2'd2: begin
                    value = kd;
                    case (index)
                        4'd0: text_char = "K";
                        4'd1: text_char = "D";
                        4'd2: text_char = ":";
                        4'd3: text_char = decimal_char(value / 1000);
                        4'd4: text_char = decimal_char((value / 100) % 10);
                        4'd5: text_char = ".";
                        4'd6: text_char = decimal_char((value / 10) % 10);
                        4'd7: text_char = decimal_char(value % 10);
                        default: text_char = " ";
                    endcase
                end
                default: begin
                    case (index)
                        4'd0: text_char = "M";
                        4'd1: text_char = "O";
                        4'd2: text_char = "D";
                        4'd3: text_char = "E";
                        4'd4: text_char = ":";
                        4'd5: text_char = (current_mode == 2'd0) ? "I" :
                                          (current_mode == 2'd1) ? "S" :
                                          (current_mode == 2'd2) ? "B" : "F";
                        4'd6: text_char = (current_mode == 2'd0) ? "D" :
                                          (current_mode == 2'd1) ? "W" :
                                          (current_mode == 2'd2) ? "L" : "T";
                        default: text_char = " ";
                    endcase
                end
            endcase
        end
    endfunction

    // 5x7 font, least significant bit is the top pixel. Column 5 is blank.
    function [7:0] glyph_column;
        input [7:0] code;
        input [2:0] column;
        begin
            if (column == 5) begin
                glyph_column = 8'h00;
            end else begin
                case (code)
                    "0": case (column) 0:glyph_column=8'h3E;1:glyph_column=8'h51;2:glyph_column=8'h49;3:glyph_column=8'h45;default:glyph_column=8'h3E; endcase
                    "1": case (column) 0:glyph_column=8'h00;1:glyph_column=8'h42;2:glyph_column=8'h7F;3:glyph_column=8'h40;default:glyph_column=8'h00; endcase
                    "2": case (column) 0:glyph_column=8'h62;1:glyph_column=8'h51;2:glyph_column=8'h49;3:glyph_column=8'h49;default:glyph_column=8'h46; endcase
                    "3": case (column) 0:glyph_column=8'h22;1:glyph_column=8'h49;2:glyph_column=8'h49;3:glyph_column=8'h49;default:glyph_column=8'h36; endcase
                    "4": case (column) 0:glyph_column=8'h18;1:glyph_column=8'h14;2:glyph_column=8'h12;3:glyph_column=8'h7F;default:glyph_column=8'h10; endcase
                    "5": case (column) 0:glyph_column=8'h2F;1:glyph_column=8'h49;2:glyph_column=8'h49;3:glyph_column=8'h49;default:glyph_column=8'h31; endcase
                    "6": case (column) 0:glyph_column=8'h3E;1:glyph_column=8'h49;2:glyph_column=8'h49;3:glyph_column=8'h49;default:glyph_column=8'h32; endcase
                    "7": case (column) 0:glyph_column=8'h01;1:glyph_column=8'h71;2:glyph_column=8'h09;3:glyph_column=8'h05;default:glyph_column=8'h03; endcase
                    "8": case (column) 0:glyph_column=8'h36;1:glyph_column=8'h49;2:glyph_column=8'h49;3:glyph_column=8'h49;default:glyph_column=8'h36; endcase
                    "9": case (column) 0:glyph_column=8'h06;1:glyph_column=8'h49;2:glyph_column=8'h49;3:glyph_column=8'h29;default:glyph_column=8'h1E; endcase
                    "K": case (column) 0:glyph_column=8'h7F;1:glyph_column=8'h08;2:glyph_column=8'h14;3:glyph_column=8'h22;default:glyph_column=8'h41; endcase
                    "P": case (column) 0:glyph_column=8'h7F;1:glyph_column=8'h09;2:glyph_column=8'h09;3:glyph_column=8'h09;default:glyph_column=8'h06; endcase
                    "I": case (column) 0:glyph_column=8'h00;1:glyph_column=8'h41;2:glyph_column=8'h7F;3:glyph_column=8'h41;default:glyph_column=8'h00; endcase
                    "D": case (column) 0:glyph_column=8'h7F;1:glyph_column=8'h41;2:glyph_column=8'h41;3:glyph_column=8'h22;default:glyph_column=8'h1C; endcase
                    "M": case (column) 0:glyph_column=8'h7F;1:glyph_column=8'h02;2:glyph_column=8'h0C;3:glyph_column=8'h02;default:glyph_column=8'h7F; endcase
                    "O": case (column) 0:glyph_column=8'h3E;1:glyph_column=8'h41;2:glyph_column=8'h41;3:glyph_column=8'h41;default:glyph_column=8'h3E; endcase
                    "E": case (column) 0:glyph_column=8'h7F;1:glyph_column=8'h49;2:glyph_column=8'h49;3:glyph_column=8'h49;default:glyph_column=8'h41; endcase
                    "S": case (column) 0:glyph_column=8'h46;1:glyph_column=8'h49;2:glyph_column=8'h49;3:glyph_column=8'h49;default:glyph_column=8'h31; endcase
                    "W": case (column) 0:glyph_column=8'h7F;1:glyph_column=8'h20;2:glyph_column=8'h18;3:glyph_column=8'h20;default:glyph_column=8'h7F; endcase
                    "B": case (column) 0:glyph_column=8'h7F;1:glyph_column=8'h49;2:glyph_column=8'h49;3:glyph_column=8'h49;default:glyph_column=8'h36; endcase
                    "L": case (column) 0:glyph_column=8'h7F;1:glyph_column=8'h40;2:glyph_column=8'h40;3:glyph_column=8'h40;default:glyph_column=8'h40; endcase
                    "F": case (column) 0:glyph_column=8'h7F;1:glyph_column=8'h09;2:glyph_column=8'h09;3:glyph_column=8'h09;default:glyph_column=8'h01; endcase
                    ":": case (column) 0:glyph_column=8'h00;1:glyph_column=8'h36;2:glyph_column=8'h36;3:glyph_column=8'h00;default:glyph_column=8'h00; endcase
                    ".": case (column) 0:glyph_column=8'h00;1:glyph_column=8'h60;2:glyph_column=8'h60;3:glyph_column=8'h00;default:glyph_column=8'h00; endcase
                    default: glyph_column = 8'h00;
                endcase
            end
        end
    endfunction

    always @* begin
        current_char = text_char(line_index, char_index, kp_x100, ki_x100, kd_x100, mode);
        glyph_data   = glyph_column(current_char, glyph_index);
    end

    task automatic prepare_command;
        input [7:0] command_byte;
        begin
            i2c_length = 3'd3;
            i2c_data0  = 8'h78;
            i2c_data1  = 8'h00;
            i2c_data2  = command_byte;
            i2c_data3  = 8'h00;
        end
    endtask

    task automatic prepare_data;
        input [7:0] data_byte;
        begin
            i2c_length = 3'd3;
            i2c_data0  = 8'h78;
            i2c_data1  = 8'h40;
            i2c_data2  = data_byte;
            i2c_data3  = 8'h00;
        end
    endtask

    always @(posedge clk) begin
        if (!rst_n) begin
            state       <= ST_RESET_WAIT;
            wait_count  <= 32'd0;
            init_index  <= 5'd0;
            line_index  <= 2'd0;
            char_index  <= 4'd0;
            glyph_index <= 3'd0;
            i2c_start   <= 1'b0;
            i2c_length  <= 3'd0;
            i2c_data0   <= 8'd0;
            i2c_data1   <= 8'd0;
            i2c_data2   <= 8'd0;
            i2c_data3   <= 8'd0;
        end else begin
            i2c_start <= 1'b0;
            case (state)
                ST_RESET_WAIT: begin
                    if (wait_count >= STARTUP_CYCLES - 1) begin
                        wait_count <= 32'd0;
                        init_index <= 5'd0;
                        state      <= ST_INIT_START;
                    end else wait_count <= wait_count + 1'b1;
                end
                ST_INIT_START: begin
                    prepare_command(init_command(init_index));
                    i2c_start <= 1'b1;
                    if (i2c_busy) state <= ST_INIT_WAIT;
                end
                ST_INIT_WAIT: begin
                    if (i2c_done) begin
                        if (init_index == 5'd16) begin
                            wait_count <= 32'd0;
                            state <= ST_REFRESH_WAIT;
                        end else begin
                            init_index <= init_index + 1'b1;
                            state <= ST_INIT_START;
                        end
                    end
                end
                ST_REFRESH_WAIT: begin
                    if (wait_count >= REFRESH_CYCLES - 1) begin
                        wait_count <= 32'd0;
                        line_index <= 2'd0;
                        state <= ST_PAGE_START;
                    end else wait_count <= wait_count + 1'b1;
                end
                ST_PAGE_START: begin
                    // One text row occupies one SSD1306 page.  Advance by
                    // one page so the four rows use pages 0 through 3.
                    prepare_command(8'hB0 + line_index);
                    i2c_start <= 1'b1;
                    if (i2c_busy) state <= ST_PAGE_WAIT;
                end
                ST_PAGE_WAIT: if (i2c_done) state <= ST_LOW_START;
                ST_LOW_START: begin
                    prepare_command(8'h00);
                    i2c_start <= 1'b1;
                    if (i2c_busy) state <= ST_LOW_WAIT;
                end
                ST_LOW_WAIT: if (i2c_done) state <= ST_HIGH_START;
                ST_HIGH_START: begin
                    prepare_command(8'h10);
                    i2c_start <= 1'b1;
                    if (i2c_busy) state <= ST_HIGH_WAIT;
                end
                ST_HIGH_WAIT: if (i2c_done) begin
                    char_index  <= 4'd0;
                    glyph_index <= 3'd0;
                    state       <= ST_DATA_START;
                end
                ST_DATA_START: begin
                    prepare_data(glyph_data);
                    i2c_start <= 1'b1;
                    if (i2c_busy) state <= ST_DATA_WAIT;
                end
                ST_DATA_WAIT: if (i2c_done) begin
                    if (glyph_index == 3'd5) begin
                        glyph_index <= 3'd0;
                        if (char_index == 4'd7) begin
                            char_index <= 4'd0;
                            if (line_index == 2'd3) begin
                                line_index <= 2'd0;
                                state <= ST_REFRESH_WAIT;
                            end else begin
                                line_index <= line_index + 1'b1;
                                state <= ST_PAGE_START;
                            end
                        end else begin
                            char_index <= char_index + 1'b1;
                            state <= ST_DATA_START;
                        end
                    end else begin
                        glyph_index <= glyph_index + 1'b1;
                        state <= ST_DATA_START;
                    end
                end
                default: state <= ST_RESET_WAIT;
            endcase
        end
    end
endmodule

