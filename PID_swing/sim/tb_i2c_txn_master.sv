`timescale 1ns/1ps

module tb_i2c_txn_master;
    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg rst_n = 1'b0;
    reg start = 1'b0;
    reg [2:0] length = 3'd3;
    reg [7:0] data0 = 8'h78;
    reg [7:0] data1 = 8'h00;
    reg [7:0] data2 = 8'hAE;
    reg [7:0] data3 = 8'h00;
    wire busy, done, ack_error;
    wire scl_low, sda_low;
    tri1 scl, sda;
    reg slave_ack_low = 1'b0;
    assign scl = scl_low ? 1'b0 : 1'bz;
    assign sda = (sda_low || slave_ack_low) ? 1'b0 : 1'bz;

    i2c_txn_master #(.CLK_HZ(1_000_000), .I2C_HZ(100_000)) dut (
        .clk(clk), .rst_n(rst_n), .start(start), .length(length),
        .data0(data0), .data1(data1), .data2(data2), .data3(data3),
        .busy(busy), .done(done), .ack_error(ack_error),
        .scl_low(scl_low), .sda_low(sda_low), .sda_in(sda)
    );

    reg [7:0] observed [0:2];
    integer transaction_number;

    task automatic slave_capture;
        input integer nack_byte;
        integer byte_no, bit_no;
        begin
            @(negedge sda);
            if (scl !== 1'b1)
                $fatal(1, "START did not occur while SCL was high");
            for (byte_no = 0; byte_no < 3; byte_no = byte_no + 1) begin
                observed[byte_no] = 8'h00;
                for (bit_no = 7; bit_no >= 0; bit_no = bit_no - 1) begin
                    @(posedge scl);
                    observed[byte_no][bit_no] = sda;
                end
                @(negedge scl);
                #1;
                slave_ack_low = (byte_no != nack_byte);
                @(posedge scl);
                #1;
                if (sda !== ((byte_no == nack_byte) ? 1'b1 : 1'b0))
                    $fatal(1, "ACK level wrong on byte %0d", byte_no);
                @(negedge scl);
                #1;
                slave_ack_low = 1'b0;
            end
            if (observed[0] !== 8'h78 || observed[1] !== 8'h00 ||
                observed[2] !== 8'hAE)
                $fatal(1, "I2C bytes: %02x %02x %02x (expected 78 00 AE)",
                       observed[0], observed[1], observed[2]);
        end
    endtask

    task automatic send_transaction;
        begin
            @(negedge clk);
            start = 1'b1;
            wait (busy === 1'b1);
            @(negedge clk);
            start = 1'b0;
            wait (done === 1'b1);
            #1;
        end
    endtask

    initial begin
        #200_000;
        $fatal(1, "I2C transaction timed out");
    end

    initial begin
        repeat (5) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;

        transaction_number = 0;
        fork
            slave_capture(-1);
            send_transaction();
        join
        if (ack_error !== 1'b0)
            $fatal(1, "ACKed transaction reported an ACK error");

        repeat (5) @(posedge clk);
        transaction_number = 1;
        fork
            slave_capture(1);
            send_transaction();
        join
        if (ack_error !== 1'b1)
            $fatal(1, "NACK on control byte was not reported");

        $display("tb_i2c_txn_master: PASS");
        $finish;
    end
endmodule
