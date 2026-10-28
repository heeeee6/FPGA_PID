`timescale 1ns/1ps

module tb_xadc_scan;
    reg clk = 1'b0;
    reg rst_n = 1'b0;
    wire [11:0] ch0, ch1, ch2, ch3;
    wire sample_valid;
    integer scan_count = 0;
    integer drdy_count = 0;

    xadc4_reader dut (
        .clk(clk), .rst_n(rst_n),
        .vauxp(4'b0), .vauxn(4'b0),
        .ch0(ch0), .ch1(ch1), .ch2(ch2), .ch3(ch3),
        .sample_valid(sample_valid)
    );

    always #10 clk = ~clk;
    always @(posedge clk) begin
        if (sample_valid)
            scan_count = scan_count + 1;
        if (dut.xadc_drdy)
            drdy_count = drdy_count + 1;
    end

    initial begin
        repeat (10) @(posedge clk);
        rst_n = 1'b1;
        repeat (20000) @(posedge clk);
        if (scan_count < 2)
            $fatal(1, "XADC scans missing: scans=%0d DRDY=%0d", scan_count, drdy_count);
        $display("tb_xadc_scan: PASS scans=%0d DRDY=%0d", scan_count, drdy_count);
        $finish;
    end
endmodule
