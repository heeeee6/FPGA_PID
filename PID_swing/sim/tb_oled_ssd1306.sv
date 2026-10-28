`timescale 1ns/1ps

// Capture the actual open-drain I2C bytes and ACK each one.  The checks below
// cover SSD1306 page addressing and the pixel columns of KP:12.34.
module tb_oled_ssd1306;
    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg rst_n = 1'b0;
    reg [15:0] kp_x100 = 16'd1234;
    reg [15:0] ki_x100 = 16'd56;
    reg [15:0] kd_x100 = 16'd789;
    reg [1:0] mode = 2'd2;

    wire scl_low;
    wire sda_low;
    tri1 scl;
    tri1 sda;
    reg slave_ack_low = 1'b0;
    assign scl = scl_low ? 1'b0 : 1'bz;
    assign sda = (sda_low || slave_ack_low) ? 1'b0 : 1'bz;

    oled_ssd1306 #(
        .CLK_HZ(1_000_000), .I2C_HZ(250_000),
        .STARTUP_CYCLES(2), .REFRESH_CYCLES(5)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .kp_x100(kp_x100), .ki_x100(ki_x100), .kd_x100(kd_x100),
        .mode(mode),
        .oled_scl_low(scl_low), .oled_sda_low(sda_low), .oled_sda_in(sda)
    );

    reg [7:0] observed [0:2];
    integer transaction_count = 0;
    integer init_command_count = 0;
    integer row_transaction = 0;
    integer page;
    integer offset;
    integer character;
    integer column;
    reg init_done = 1'b0;
    reg saw_address_mode_command = 1'b0;
    reg saw_page_address_mode = 1'b0;
    reg [7:0] previous_init_command = 8'h00;

    function [7:0] expected_digit_column;
        input integer digit;
        input integer col;
        begin
            expected_digit_column = 8'hxx;
            case (digit)
                0: case (col)
                    0: expected_digit_column = 8'h3E;
                    1: expected_digit_column = 8'h51;
                    2: expected_digit_column = 8'h49;
                    3: expected_digit_column = 8'h45;
                    4: expected_digit_column = 8'h3E;
                    5: expected_digit_column = 8'h00;
                endcase
                1: case (col)
                    0: expected_digit_column = 8'h00;
                    1: expected_digit_column = 8'h42;
                    2: expected_digit_column = 8'h7F;
                    3: expected_digit_column = 8'h40;
                    4: expected_digit_column = 8'h00;
                    5: expected_digit_column = 8'h00;
                endcase
                2: case (col)
                    0: expected_digit_column = 8'h62;
                    1: expected_digit_column = 8'h51;
                    2: expected_digit_column = 8'h49;
                    3: expected_digit_column = 8'h49;
                    4: expected_digit_column = 8'h46;
                    5: expected_digit_column = 8'h00;
                endcase
                3: case (col)
                    0: expected_digit_column = 8'h22;
                    1: expected_digit_column = 8'h49;
                    2: expected_digit_column = 8'h49;
                    3: expected_digit_column = 8'h49;
                    4: expected_digit_column = 8'h36;
                    5: expected_digit_column = 8'h00;
                endcase
                4: case (col)
                    0: expected_digit_column = 8'h18;
                    1: expected_digit_column = 8'h14;
                    2: expected_digit_column = 8'h12;
                    3: expected_digit_column = 8'h7F;
                    4: expected_digit_column = 8'h10;
                    5: expected_digit_column = 8'h00;
                endcase
            endcase
        end
    endfunction

    task automatic capture_transaction;
        integer byte_no;
        integer bit_no;
        begin
            @(negedge sda);
            if (scl !== 1'b1)
                $fatal(1, "I2C START occurred while SCL was low");

            for (byte_no = 0; byte_no < 3; byte_no = byte_no + 1) begin
                observed[byte_no] = 8'h00;
                for (bit_no = 7; bit_no >= 0; bit_no = bit_no - 1) begin
                    @(posedge scl);
                    observed[byte_no][bit_no] = sda;
                end
                @(negedge scl);
                #1;
                slave_ack_low = 1'b1;
                @(posedge scl);
                #1;
                if (sda !== 1'b0)
                    $fatal(1, "I2C slave ACK did not pull SDA low");
                @(negedge scl);
                #1;
                slave_ack_low = 1'b0;
            end

            // The slave releases SDA immediately after the ACK bit, which
            // can produce a harmless SDA rising edge while SCL is still low.
            // Ignore that edge and wait for the rising edge that forms STOP
            // while SCL is high.
            while (scl !== 1'b1)
                @(posedge sda);
        end
    endtask

    task automatic check_transaction;
        reg [7:0] expected;
        begin
            if (observed[0] !== 8'h78)
                $fatal(1, "Transaction %0d addressed %02x, expected 78",
                       transaction_count, observed[0]);

            if (!init_done) begin
                if (observed[1] !== 8'h00)
                    $fatal(1, "Initialization transaction %0d is not a command",
                           init_command_count);
                if (init_command_count == 0 && observed[2] !== 8'hAE)
                    $fatal(1, "The first initialization command is not AE");
                if (observed[2] == 8'h20)
                    saw_address_mode_command = 1'b1;
                if (previous_init_command == 8'h20 && observed[2] == 8'h02)
                    saw_page_address_mode = 1'b1;
                previous_init_command = observed[2];
                init_command_count = init_command_count + 1;
                if (observed[2] == 8'hAF) begin
                    if (!saw_address_mode_command || !saw_page_address_mode)
                        $fatal(1, "OLED page addressing mode was not initialized");
                    init_done = 1'b1;
                end
            end else begin
                page = row_transaction / 51;
                offset = row_transaction % 51;
                if (page >= 4)
                    $fatal(1, "Unexpected transaction after four display rows");

                if (offset < 3) begin
                    expected = (offset == 0) ? (8'hB0 + page) :
                               (offset == 1) ? 8'h00 : 8'h10;
                    if (observed[1] !== 8'h00 || observed[2] !== expected)
                        $fatal(1, "Page %0d command %0d: got %02x/%02x, expected 00/%02x",
                               page, offset, observed[1], observed[2], expected);
                end else begin
                    if (observed[1] !== 8'h40)
                        $fatal(1, "Page %0d pixel %0d is not display data",
                               page, offset - 3);
                    // KP:12.34 occupies eight 6-column characters on page 0.
                    // Check every column of all four decimal digits on the bus.
                    character = (offset - 3) / 6;
                    column = (offset - 3) % 6;
                    if (page == 0 && (character == 3 || character == 4 ||
                                      character == 6 || character == 7)) begin
                        case (character)
                            3: expected = expected_digit_column(1, column);
                            4: expected = expected_digit_column(2, column);
                            6: expected = expected_digit_column(3, column);
                            default: expected = expected_digit_column(4, column);
                        endcase
                        if (observed[2] !== expected)
                            $fatal(1, "KP:12.34 character %0d column %0d: got %02x, expected %02x",
                                   character, column, observed[2], expected);
                    end
                end

                row_transaction = row_transaction + 1;
                if (row_transaction == 204) begin
                    $display("tb_oled_ssd1306: PASS (%0d initialization commands, four pages, KP:12.34)",
                             init_command_count);
                    $finish;
                end
            end
            transaction_count = transaction_count + 1;
        end
    endtask

    initial begin
        #2_000_000;
        $fatal(1, "OLED test timed out after %0d transactions", transaction_count);
    end

    initial begin
        repeat (5) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;
    end

    initial begin
        wait (rst_n === 1'b1);
        forever begin
            capture_transaction();
            check_transaction();
        end
    end
endmodule
