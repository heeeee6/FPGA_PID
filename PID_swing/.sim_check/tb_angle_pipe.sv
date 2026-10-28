`timescale 1ns/1ps
module tb_angle_pipe;
    reg clk = 1'b0;
    reg rst_n = 1'b0;
    reg update = 1'b0;
    reg [11:0] adc = 12'd0;
    wire signed [15:0] angle;
    angle_from_adc_pipe dut (
        .clk(clk), .rst_n(rst_n), .update(update), .adc(adc),
        .angle_deg_x10(angle)
    );
    always #5 clk = ~clk;
    task sample;
        input [11:0] value;
        input signed [15:0] expected;
        begin
            adc = value;
            @(posedge clk);
            #1;
            @(posedge clk);
            #1;
            if (angle !== expected)
                $fatal(1, "adc=%0d expected=%0d got=%0d", value, expected, angle);
        end
    endtask
    initial begin
        repeat (2) @(posedge clk);
        rst_n = 1'b1;
        sample(12'd0,    16'sd0);
        sample(12'd3974, 16'sd0);
        sample(12'd1988, 16'sd1800);
        sample(12'd1989, -16'sd1799);
        sample(12'd4095, 16'sd0);
        $display("tb_angle_pipe: PASS");
        $finish;
    end
endmodule
