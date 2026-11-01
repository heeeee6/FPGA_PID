`timescale 1ns/1ps

// ADS1115 reader against two behavioral ADS1115 devices.
module tb_ads1115_reader;
    reg clk = 1'b0;
    always #50 clk = ~clk;          // 10 MHz
    reg rst_n = 1'b0;

    wire scl_low, sda_low;
    tri1 scl, sda;
    assign scl = scl_low ? 1'b0 : 1'bz;
    assign sda = sda_low ? 1'b0 : 1'bz;

    reg present_a = 1'b1;
    reg present_b = 1'b1;
    reg [15:0] angle_raw = 16'd13200;
    reg [15:0] rp1_raw = 16'd6600;
    reg [15:0] rp2_raw = 16'd13200;
    reg [15:0] rp3_raw = 16'd19800;
    reg [15:0] rp4_raw = 16'd26400;
    wire [15:0] cfg_a, cfg_b;
    wire [31:0] cfgw_a, cfgw_b, reads_a, reads_b;

    ads1115_model #(.ADDR(7'h48)) u_a (
        .scl(scl), .sda(sda), .present(present_a),
        .ain0(angle_raw), .ain1(16'd0), .ain2(16'd0), .ain3(16'd0),
        .cfg_reg(cfg_a), .cfg_writes(cfgw_a), .reads(reads_a));
    ads1115_model #(.ADDR(7'h49)) u_b (
        .scl(scl), .sda(sda), .present(present_b),
        .ain0(rp1_raw), .ain1(rp2_raw), .ain2(rp3_raw), .ain3(rp4_raw),
        .cfg_reg(cfg_b), .cfg_writes(cfgw_b), .reads(reads_b));

    wire [11:0] angle1, rp1, rp2, rp3, rp4;
    wire sample_valid, online;
    ads1115_reader #(.CLK_HZ(10_000_000), .I2C_HZ(400_000), .PERIOD_US(1000),
                     .STARTUP_MS(1), .RETRY_MS(2)) dut (
        .clk(clk), .rst_n(rst_n), .angle1(angle1), .rp1(rp1), .rp2(rp2),
        .rp3(rp3), .rp4(rp4), .sample_valid(sample_valid), .online(online),
        .scl_low(scl_low), .sda_low(sda_low), .sda_in(sda));

    integer errors = 0;
    task automatic check_near;
        input [255:0] name;
        input integer got;
        input integer expected;
        begin
            if ((got > expected + 1) || (got < expected - 1)) begin
                $display("FAIL %0s: got %0d expected %0d", name, got, expected);
                errors = errors + 1;
            end
        end
    endtask

    initial begin
        #400_000_000;
        $fatal(1, "tb_ads1115_reader timed out");
    end

    initial begin
        repeat (10) @(posedge clk);
        rst_n = 1'b1;

        wait (online === 1'b1);
        wait (sample_valid === 1'b1);
        repeat (10) @(posedge clk);
        // Expected 12-bit code = raw * 4095 / 26400.
        check_near("angle", angle1, 2047);
        check_near("rp1", rp1, 1023);
        check_near("rp2", rp2, 2047);
        check_near("rp3", rp3, 3071);
        check_near("rp4", rp4, 4095);
        if (cfg_a !== 16'h42E3) begin
            $display("FAIL angle config %04x (expected 42E3)", cfg_a); errors = errors + 1;
        end
        if (reads_a < 5 || reads_b < 4) begin
            $display("FAIL reads a=%0d b=%0d", reads_a, reads_b); errors = errors + 1;
        end

        // Angle data must advance on every 1 ms scheduling tick, independently
        // of the four-channel potentiometer scan.
        begin : angle_rate_check
            integer reads_before;
            reads_before = reads_a;
            repeat (200000) @(posedge clk); // 20 ms at 10 MHz
            if ((reads_a - reads_before) < 18) begin
                $display("FAIL angle reads advanced only %0d in 20 ms",
                         reads_a - reads_before);
                errors = errors + 1;
            end
        end

        // Values must track input changes; out-of-range inputs clamp.
        angle_raw = 16'd7000;          // 1085.8
        rp4_raw   = 16'd30000;         // above full scale -> clamp
        rp1_raw   = 16'h8000;          // negative -> 0
        repeat (100000) @(posedge clk);
        check_near("angle2", angle1, 1086);
        check_near("rp4 clamp", rp4, 4095);
        check_near("rp1 negative", rp1, 0);

        // A missing pot module must drop online; recovery must restore it.
        present_b = 1'b0;
        repeat (400000) @(posedge clk); // cover one 20 ms pot scan period
        if (online !== 1'b0) begin
            $display("FAIL online stayed high with ADS#2 absent"); errors = errors + 1;
        end
        present_b = 1'b1;
        wait (online === 1'b1);
        wait (sample_valid === 1'b1);
        repeat (10) @(posedge clk);
        check_near("angle after recovery", angle1, 1086);

        if (errors == 0) $display("tb_ads1115_reader: PASS");
        else $fatal(1, "tb_ads1115_reader: %0d errors", errors);
        $finish;
    end
endmodule
