`timescale 1ns/1ps

// Extended SSD1306 text page used by the STM32-compatible inverted-pendulum
// top level.  The original oled_ssd1306 module is intentionally left
// unchanged because its compact regression test exercises its port list.
// This variant uses the same open-drain i2c_txn_master and shows eight 6x8
// text rows (20 characters per row): raw angle/error, encoder position and
// velocity, both PID gain sets, mode/page and commissioning notes.
module oled_ssd1306_ui #(
    parameter integer CLK_HZ = 50000000,
    parameter integer I2C_HZ = 400000,
    parameter integer STARTUP_CYCLES = 2500000,
    parameter integer REFRESH_CYCLES = 25000000
) (
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire [11:0]           angle_adc,
    input  wire signed [15:0]    angle_error_counts,
    input  wire signed [15:0]    encoder_position,
    input  wire signed [15:0]    encoder_velocity,
    input  wire [15:0]           angle_kp_q8,
    input  wire [15:0]           angle_ki_q8,
    input  wire [15:0]           angle_kd_q8,
    input  wire [15:0]           location_kp_q8,
    input  wire [15:0]           location_ki_q8,
    input  wire [15:0]           location_kd_q8,
    input  wire [1:0]            mode,
    input  wire [1:0]            page,
    output wire                  oled_scl_low,
    output wire                  oled_sda_low,
    input  wire                  oled_sda_in
);
    localparam integer SAFE_STARTUP = (STARTUP_CYCLES < 1) ? 1 : STARTUP_CYCLES;
    localparam integer SAFE_REFRESH = (REFRESH_CYCLES < 1) ? 1 : REFRESH_CYCLES;

    localparam [4:0] ST_RESET_WAIT = 5'd0;
    localparam [4:0] ST_INIT_START = 5'd1;
    localparam [4:0] ST_INIT_WAIT  = 5'd2;
    localparam [4:0] ST_REFRESH_WAIT = 5'd3;
    localparam [4:0] ST_CAPTURE     = 5'd4;
    localparam [4:0] ST_PAGE_START  = 5'd5;
    localparam [4:0] ST_PAGE_WAIT   = 5'd6;
    localparam [4:0] ST_LOW_START   = 5'd7;
    localparam [4:0] ST_LOW_WAIT    = 5'd8;
    localparam [4:0] ST_HIGH_START  = 5'd9;
    localparam [4:0] ST_HIGH_WAIT   = 5'd10;
    localparam [4:0] ST_DATA_START  = 5'd11;
    localparam [4:0] ST_DATA_WAIT   = 5'd12;
    localparam [4:0] ST_CHAR        = 5'd13;
    localparam [4:0] ST_GLYPH       = 5'd14;

    reg [4:0] state;
    reg [31:0] wait_count;
    reg [4:0] init_index;
    reg [2:0] line_index;
    reg [4:0] char_index;
    reg [2:0] glyph_index;
    reg       i2c_start;
    reg [2:0] i2c_length;
    reg [7:0] i2c_data0, i2c_data1, i2c_data2, i2c_data3;
    wire      i2c_busy;
    wire      i2c_done;
    wire      i2c_ack_error;

    reg [11:0] angle_adc_s;
    reg signed [15:0] angle_error_s;
    reg signed [15:0] encoder_position_s;
    reg signed [15:0] encoder_velocity_s;
    reg [15:0] angle_kp_s, angle_ki_s, angle_kd_s;
    reg [15:0] location_kp_s, location_ki_s, location_kd_s;
    reg [1:0] mode_s, page_s;
    reg [7:0] glyph_data_r;
    reg [7:0] char_r;
    reg       next_is_refresh;
    reg       next_is_page;
    reg [2:0] n_glyph;
    reg [4:0] n_char;
    reg [2:0] n_line;
    reg       wrap_refresh;

    i2c_txn_master #(.CLK_HZ(CLK_HZ), .I2C_HZ(I2C_HZ)) u_i2c (
        .clk(clk), .rst_n(rst_n), .start(i2c_start), .length(i2c_length),
        .data0(i2c_data0), .data1(i2c_data1), .data2(i2c_data2),
        .data3(i2c_data3), .busy(i2c_busy), .done(i2c_done),
        .ack_error(i2c_ack_error), .scl_low(oled_scl_low),
        .sda_low(oled_sda_low), .sda_in(oled_sda_in)
    );

    function [7:0] init_command;
        input [4:0] index;
        begin
            case (index)
                5'd0:  init_command = 8'hAE;
                5'd1:  init_command = 8'hD5;
                5'd2:  init_command = 8'h80;
                5'd3:  init_command = 8'hA8;
                5'd4:  init_command = 8'h3F;
                5'd5:  init_command = 8'hD3;
                5'd6:  init_command = 8'h00;
                5'd7:  init_command = 8'h40;
                5'd8:  init_command = 8'h8D;
                5'd9:  init_command = 8'h14;
                5'd10: init_command = 8'h20;
                5'd11: init_command = 8'h02;
                5'd12: init_command = 8'hA1;
                5'd13: init_command = 8'hC8;
                5'd14: init_command = 8'hDA;
                5'd15: init_command = 8'h12;
                5'd16: init_command = 8'hAF;
                default: init_command = 8'hAF;
            endcase
        end
    endfunction

    function [15:0] q8_to_x100;
        input [15:0] q8;
        reg [31:0] product;
        begin
            product = q8 * 100;
            q8_to_x100 = product >> 8;
        end
    endfunction

    function [7:0] digit_at;
        input [15:0] value;
        input integer pos;
        reg [15:0] av;
        reg [17:0] divisor;
        begin
            if (value[15])
                av = (~value) + 1'b1;
            else
                av = value;
            case (pos)
                1: divisor = 18'd100000;
                2: divisor = 18'd10000;
                3: divisor = 18'd1000;
                4: divisor = 18'd100;
                5: divisor = 18'd10;
                default: divisor = 18'd1;
            endcase
            if (pos == 0)
                digit_at = 8'h30;
            else
                digit_at = 8'h30 + ((av / divisor) % 10);
        end
    endfunction

    function [7:0] gain_digit;
        input [15:0] gain_x100;
        input integer pos;
        integer value;
        begin
            value = gain_x100;
            // x100 is a fixed-point value.  Display units, tenths and
            // hundredths; the fourth slot is kept blank.
            case (pos)
                0: gain_digit = 8'h30 + ((value / 100) % 10);
                1: gain_digit = 8'h30 + ((value / 10) % 10);
                2: gain_digit = 8'h30 + (value % 10);
                default: gain_digit = 8'h20;
            endcase
        end
    endfunction

    function [7:0] sign_char;
        input integer value;
        begin
            sign_char = (value < 0) ? 8'h2D : 8'h2B;
        end
    endfunction

    function [7:0] text_char;
        input [2:0] row;
        input [4:0] index;
        integer value;
        integer gx;
        begin
            text_char = 8'h20;
            value = 0;
            case (row)
                3'd0: begin // ANG:#### E:+####
                    case (index)
                        0:text_char="A"; 1:text_char="N"; 2:text_char="G"; 3:text_char=":";
                        4:text_char=digit_at(angle_adc_s,3); 5:text_char=digit_at(angle_adc_s,4);
                        6:text_char=digit_at(angle_adc_s,5); 7:text_char=digit_at(angle_adc_s,6);
                        9:text_char="E"; 10:text_char=":"; 11:text_char=sign_char(angle_error_s);
                        12:text_char=digit_at(angle_error_s,3); 13:text_char=digit_at(angle_error_s,4);
                        14:text_char=digit_at(angle_error_s,5); 15:text_char=digit_at(angle_error_s,6);
                        default: text_char=" ";
                    endcase
                end
                3'd1: begin // POS:+###### V:+####
                    case (index)
                        0:text_char="P"; 1:text_char="O"; 2:text_char="S"; 3:text_char=":";
                        4:text_char=sign_char(encoder_position_s);
                        5:text_char=digit_at(encoder_position_s,0); 6:text_char=digit_at(encoder_position_s,1);
                        7:text_char=digit_at(encoder_position_s,2); 8:text_char=digit_at(encoder_position_s,3);
                        9:text_char=digit_at(encoder_position_s,4); 10:text_char=digit_at(encoder_position_s,5);
                        12:text_char="V"; 13:text_char=":"; 14:text_char=sign_char(encoder_velocity_s);
                        15:text_char=digit_at(encoder_velocity_s,3); 16:text_char=digit_at(encoder_velocity_s,4);
                        17:text_char=digit_at(encoder_velocity_s,5); 18:text_char=digit_at(encoder_velocity_s,6);
                        default: text_char=" ";
                    endcase
                end
                3'd2: begin // A KP:0.00 KI:0.00
                    case (index)
                        0:text_char="A"; 1:text_char=" "; 2:text_char="K"; 3:text_char="P"; 4:text_char=":";
                        5:text_char=gain_digit(q8_to_x100(angle_kp_s),0); 6:text_char=".";
                        7:text_char=gain_digit(q8_to_x100(angle_kp_s),1); 8:text_char=gain_digit(q8_to_x100(angle_kp_s),2);
                        10:text_char="K"; 11:text_char="I"; 12:text_char=":";
                        13:text_char=gain_digit(q8_to_x100(angle_ki_s),0); 14:text_char=".";
                        15:text_char=gain_digit(q8_to_x100(angle_ki_s),1); 16:text_char=gain_digit(q8_to_x100(angle_ki_s),2);
                        default:text_char=" ";
                    endcase
                end
                3'd3: begin // A KD:0.00 MODE:X
                    case (index)
                        0:text_char="A"; 1:text_char=" "; 2:text_char="K"; 3:text_char="D"; 4:text_char=":";
                        5:text_char=gain_digit(q8_to_x100(angle_kd_s),0); 6:text_char=".";
                        7:text_char=gain_digit(q8_to_x100(angle_kd_s),1); 8:text_char=gain_digit(q8_to_x100(angle_kd_s),2);
                        10:text_char="M"; 11:text_char="O"; 12:text_char="D"; 13:text_char="E"; 14:text_char=":";
                        15:text_char=(mode_s==2'd0)?"I":(mode_s==2'd1)?"S":(mode_s==2'd2)?"B":"F";
                        default:text_char=" ";
                    endcase
                end
                3'd4: begin // L KP:0.00 KI:0.00
                    case (index)
                        0:text_char="L"; 1:text_char=" "; 2:text_char="K"; 3:text_char="P"; 4:text_char=":";
                        5:text_char=gain_digit(q8_to_x100(location_kp_s),0); 6:text_char=".";
                        7:text_char=gain_digit(q8_to_x100(location_kp_s),1); 8:text_char=gain_digit(q8_to_x100(location_kp_s),2);
                        10:text_char="K"; 11:text_char="I"; 12:text_char=":";
                        13:text_char=gain_digit(q8_to_x100(location_ki_s),0); 14:text_char=".";
                        15:text_char=gain_digit(q8_to_x100(location_ki_s),1); 16:text_char=gain_digit(q8_to_x100(location_ki_s),2);
                        default:text_char=" ";
                    endcase
                end
                3'd5: begin // L KD:00.00 PAGE:X
                    case (index)
                        0:text_char="L"; 1:text_char=" "; 2:text_char="K"; 3:text_char="D"; 4:text_char=":";
                        5:text_char=gain_digit(q8_to_x100(location_kd_s),0);
                        7:text_char="."; 8:text_char=gain_digit(q8_to_x100(location_kd_s),1); 9:text_char=gain_digit(q8_to_x100(location_kd_s),2);
                        11:text_char="P"; 12:text_char="A"; 13:text_char="G"; 14:text_char="E"; 15:text_char=":";
                        16:text_char=8'h30 + page_s;
                        default:text_char=" ";
                    endcase
                end
                3'd6: begin // commissioning protection reference
                    case (index)
                        0:text_char="A"; 1:text_char="D"; 2:text_char="C"; 3:text_char=" ";
                        4:text_char="R"; 5:text_char="A"; 6:text_char="N"; 7:text_char="G"; 8:text_char="E";
                        10:text_char="1"; 11:text_char="0"; 12:text_char="0"; 13:text_char="0";
                        14:text_char="-"; 15:text_char="3"; 16:text_char="0"; 17:text_char="0"; 18:text_char="0";
                        default:text_char=" ";
                    endcase
                end
                default: begin // I2C OLED 3C OK
                    case (index)
                        0:text_char="I"; 1:text_char="2"; 2:text_char="C"; 3:text_char=" ";
                        4:text_char="O"; 5:text_char="L"; 6:text_char="E"; 7:text_char="D";
                        9:text_char="3"; 10:text_char="C"; 12:text_char="O"; 13:text_char="K";
                        default:text_char=" ";
                    endcase
                end
            endcase
        end
    endfunction

    function [7:0] glyph_column;
        input [7:0] code;
        input [2:0] column;
        begin
            if (column == 5) glyph_column = 8'h00;
            else begin
                case (code)
                    "0": case(column)0:glyph_column=8'h3E;1:glyph_column=8'h51;2:glyph_column=8'h49;3:glyph_column=8'h45;default:glyph_column=8'h3E;endcase
                    "1": case(column)0:glyph_column=8'h00;1:glyph_column=8'h42;2:glyph_column=8'h7F;3:glyph_column=8'h40;default:glyph_column=8'h00;endcase
                    "2": case(column)0:glyph_column=8'h62;1:glyph_column=8'h51;2:glyph_column=8'h49;3:glyph_column=8'h49;default:glyph_column=8'h46;endcase
                    "3": case(column)0:glyph_column=8'h22;1:glyph_column=8'h49;2:glyph_column=8'h49;3:glyph_column=8'h49;default:glyph_column=8'h36;endcase
                    "4": case(column)0:glyph_column=8'h18;1:glyph_column=8'h14;2:glyph_column=8'h12;3:glyph_column=8'h7F;default:glyph_column=8'h10;endcase
                    "5": case(column)0:glyph_column=8'h2F;1:glyph_column=8'h49;2:glyph_column=8'h49;3:glyph_column=8'h49;default:glyph_column=8'h31;endcase
                    "6": case(column)0:glyph_column=8'h3E;1:glyph_column=8'h49;2:glyph_column=8'h49;3:glyph_column=8'h49;default:glyph_column=8'h32;endcase
                    "7": case(column)0:glyph_column=8'h01;1:glyph_column=8'h71;2:glyph_column=8'h09;3:glyph_column=8'h05;default:glyph_column=8'h03;endcase
                    "8": case(column)0:glyph_column=8'h36;1:glyph_column=8'h49;2:glyph_column=8'h49;3:glyph_column=8'h49;default:glyph_column=8'h36;endcase
                    "9": case(column)0:glyph_column=8'h06;1:glyph_column=8'h49;2:glyph_column=8'h49;3:glyph_column=8'h29;default:glyph_column=8'h1E;endcase
                    "A": case(column)0:glyph_column=8'h7E;1:glyph_column=8'h11;2:glyph_column=8'h11;3:glyph_column=8'h11;default:glyph_column=8'h7E;endcase
                    "C": case(column)0:glyph_column=8'h3E;1:glyph_column=8'h41;2:glyph_column=8'h41;3:glyph_column=8'h41;default:glyph_column=8'h22;endcase
                    "D": case(column)0:glyph_column=8'h7F;1:glyph_column=8'h41;2:glyph_column=8'h41;3:glyph_column=8'h22;default:glyph_column=8'h1C;endcase
                    "E": case(column)0:glyph_column=8'h7F;1:glyph_column=8'h49;2:glyph_column=8'h49;3:glyph_column=8'h49;default:glyph_column=8'h41;endcase
                    "G": case(column)0:glyph_column=8'h3E;1:glyph_column=8'h41;2:glyph_column=8'h49;3:glyph_column=8'h49;default:glyph_column=8'h7A;endcase
                    "I": case(column)0:glyph_column=8'h00;1:glyph_column=8'h41;2:glyph_column=8'h7F;3:glyph_column=8'h41;default:glyph_column=8'h00;endcase
                    "K": case(column)0:glyph_column=8'h7F;1:glyph_column=8'h08;2:glyph_column=8'h14;3:glyph_column=8'h22;default:glyph_column=8'h41;endcase
                    "L": case(column)0:glyph_column=8'h7F;1:glyph_column=8'h40;2:glyph_column=8'h40;3:glyph_column=8'h40;default:glyph_column=8'h40;endcase
                    "M": case(column)0:glyph_column=8'h7F;1:glyph_column=8'h02;2:glyph_column=8'h0C;3:glyph_column=8'h02;default:glyph_column=8'h7F;endcase
                    "N": case(column)0:glyph_column=8'h7F;1:glyph_column=8'h02;2:glyph_column=8'h0C;3:glyph_column=8'h20;default:glyph_column=8'h7F;endcase
                    "O": case(column)0:glyph_column=8'h3E;1:glyph_column=8'h41;2:glyph_column=8'h41;3:glyph_column=8'h41;default:glyph_column=8'h3E;endcase
                    "P": case(column)0:glyph_column=8'h7F;1:glyph_column=8'h09;2:glyph_column=8'h09;3:glyph_column=8'h09;default:glyph_column=8'h06;endcase
                    "R": case(column)0:glyph_column=8'h7F;1:glyph_column=8'h09;2:glyph_column=8'h19;3:glyph_column=8'h29;default:glyph_column=8'h46;endcase
                    "S": case(column)0:glyph_column=8'h46;1:glyph_column=8'h49;2:glyph_column=8'h49;3:glyph_column=8'h49;default:glyph_column=8'h31;endcase
                    "V": case(column)0:glyph_column=8'h1F;1:glyph_column=8'h20;2:glyph_column=8'h40;3:glyph_column=8'h20;default:glyph_column=8'h1F;endcase
                    "+": case(column)0:glyph_column=8'h08;1:glyph_column=8'h08;2:glyph_column=8'h3E;3:glyph_column=8'h08;default:glyph_column=8'h08;endcase
                    "-": case(column)0:glyph_column=8'h08;1:glyph_column=8'h08;2:glyph_column=8'h08;3:glyph_column=8'h08;default:glyph_column=8'h08;endcase
                    ":": case(column)0:glyph_column=8'h00;1:glyph_column=8'h36;2:glyph_column=8'h36;3:glyph_column=8'h00;default:glyph_column=8'h00;endcase
                    ".": case(column)0:glyph_column=8'h00;1:glyph_column=8'h60;2:glyph_column=8'h60;3:glyph_column=8'h00;default:glyph_column=8'h00;endcase
                    default: glyph_column = 8'h00;
                endcase
            end
        end
    endfunction

    // Next glyph position, used to register the glyph one I2C transaction
    // ahead so the long decimal-formatter path does not sit directly in the
    // 50 MHz data-launch path.
    always @* begin
        if (glyph_index == 3'd5) begin
            n_glyph = 3'd0;
            if (char_index == 5'd19) begin
                n_char = 5'd0;
                if (line_index == 3'd7) begin
                    n_line = 3'd0;
                    wrap_refresh = 1'b1;
                end else begin
                    n_line = line_index + 1'b1;
                    wrap_refresh = 1'b0;
                end
            end else begin
                n_char = char_index + 1'b1;
                n_line = line_index;
                wrap_refresh = 1'b0;
            end
        end else begin
            n_glyph = glyph_index + 1'b1;
            n_char = char_index;
            n_line = line_index;
            wrap_refresh = 1'b0;
        end
    end

    task automatic prepare_command;
        input [7:0] command_byte;
        begin
            i2c_length = 3'd3;
            i2c_data0 = 8'h78; i2c_data1 = 8'h00;
            i2c_data2 = command_byte; i2c_data3 = 8'h00;
        end
    endtask

    task automatic prepare_data;
        input [7:0] data_byte;
        begin
            i2c_length = 3'd3;
            i2c_data0 = 8'h78; i2c_data1 = 8'h40;
            i2c_data2 = data_byte; i2c_data3 = 8'h00;
        end
    endtask

    always @(posedge clk) begin
        if (!rst_n) begin
            state <= ST_RESET_WAIT; wait_count <= 0; init_index <= 0;
            line_index <= 0; char_index <= 0; glyph_index <= 0;
            i2c_start <= 0; i2c_length <= 0;
            i2c_data0 <= 0; i2c_data1 <= 0; i2c_data2 <= 0; i2c_data3 <= 0;
            angle_adc_s <= 0; angle_error_s <= 0; encoder_position_s <= 0;
            encoder_velocity_s <= 0; angle_kp_s <= 0; angle_ki_s <= 0;
            angle_kd_s <= 0; location_kp_s <= 0; location_ki_s <= 0;
            location_kd_s <= 0; mode_s <= 0; page_s <= 0;
            glyph_data_r <= 0; char_r <= 0; next_is_refresh <= 0; next_is_page <= 0;
        end else begin
            i2c_start <= 1'b0;
            case (state)
                ST_RESET_WAIT: begin
                    if (wait_count >= SAFE_STARTUP-1) begin
                        wait_count <= 0; init_index <= 0; state <= ST_INIT_START;
                    end else wait_count <= wait_count + 1'b1;
                end
                ST_INIT_START: begin
                    prepare_command(init_command(init_index));
                    i2c_start <= 1'b1;
                    if (i2c_busy) state <= ST_INIT_WAIT;
                end
                ST_INIT_WAIT: if (i2c_done) begin
                    if (init_index == 5'd16) begin wait_count <= 0; state <= ST_REFRESH_WAIT; end
                    else begin init_index <= init_index + 1'b1; state <= ST_INIT_START; end
                end
                ST_REFRESH_WAIT: begin
                    if (wait_count >= SAFE_REFRESH-1) begin wait_count <= 0; state <= ST_CAPTURE; end
                    else wait_count <= wait_count + 1'b1;
                end
                ST_CAPTURE: begin
                    angle_adc_s <= angle_adc; angle_error_s <= angle_error_counts;
                    encoder_position_s <= encoder_position; encoder_velocity_s <= encoder_velocity;
                    angle_kp_s <= angle_kp_q8; angle_ki_s <= angle_ki_q8; angle_kd_s <= angle_kd_q8;
                    location_kp_s <= location_kp_q8; location_ki_s <= location_ki_q8; location_kd_s <= location_kd_q8;
                    mode_s <= mode; page_s <= page; line_index <= 0; state <= ST_PAGE_START;
                end
                ST_PAGE_START: begin
                    prepare_command(8'hB0 + line_index); i2c_start <= 1'b1;
                    if (i2c_busy) state <= ST_PAGE_WAIT;
                end
                ST_PAGE_WAIT: if (i2c_done) state <= ST_LOW_START;
                ST_LOW_START: begin prepare_command(8'h00); i2c_start <= 1'b1; if (i2c_busy) state <= ST_LOW_WAIT; end
                ST_LOW_WAIT: if (i2c_done) state <= ST_HIGH_START;
                ST_HIGH_START: begin prepare_command(8'h10); i2c_start <= 1'b1; if (i2c_busy) state <= ST_HIGH_WAIT; end
                ST_HIGH_WAIT: if (i2c_done) begin char_index <= 0; glyph_index <= 0; state <= ST_CHAR; end
                ST_DATA_START: begin prepare_data(glyph_data_r); i2c_start <= 1'b1; if (i2c_busy) state <= ST_DATA_WAIT; end
                ST_DATA_WAIT: if (i2c_done) begin
                    glyph_index <= n_glyph;
                    char_index <= n_char;
                    line_index <= n_line;
                    next_is_refresh <= wrap_refresh;
                    next_is_page <= (glyph_index == 3'd5 && char_index == 5'd19);
                    state <= ST_CHAR;
                end
                ST_CHAR: begin
                    char_r <= text_char(line_index, char_index);
                    state <= ST_GLYPH;
                end
                ST_GLYPH: begin
                    glyph_data_r <= glyph_column(char_r, glyph_index);
                    if (next_is_refresh)
                        state <= ST_REFRESH_WAIT;
                    else if (next_is_page)
                        state <= ST_PAGE_START;
                    else
                        state <= ST_DATA_START;
                end
                default: state <= ST_RESET_WAIT;
            endcase
        end
    end
endmodule
