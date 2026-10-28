`timescale 1ns/1ps

// Zynq-7000 XADC front end. The top-level analog pairs are indexed by
// VAUX channel number. VAUX0/1 are the two angle sensors, VAUX2/3/4 are
// RP1/RP2/RP3 and VAUX6 is RP4 (J5-15/J5-16, AD6P/AD6N).
module xadc4_reader (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [15:0] vauxp,
    input  wire [15:0] vauxn,
    output reg  [11:0] ch0,
    output reg  [11:0] ch1,
    output reg  [11:0] ch2,
    output reg  [11:0] ch3,
    output reg  [11:0] ch4,
    output reg  [11:0] ch5,
    output reg         sample_valid
);
`ifdef SIMULATION
    always @(posedge clk) begin
        if (!rst_n) begin
            ch0 <= 12'd0;
            ch1 <= 12'd0;
            ch2 <= 12'd0;
            ch3 <= 12'd0;
            ch4 <= 12'd0;
            ch5 <= 12'd0;
            sample_valid <= 1'b0;
        end else begin
            sample_valid <= 1'b0;
        end
    end
`else
    wire [15:0] xadc_do;
    wire [4:0]  xadc_channel;
    wire xadc_eoc;
    wire xadc_drdy;
    wire xadc_eos;
    wire xadc_busy;
    wire [7:0] xadc_alm;
    wire xadc_jtagbusy;
    wire xadc_jtaglocked;
    wire xadc_jtagmodified;
    wire [4:0] xadc_muxaddr;
    wire xadc_ot;
    reg [4:0] read_channel;
    reg [5:0] channel_seen;

    function [5:0] mask_for_channel;
        input [4:0] channel;
        begin
            case (channel)
                5'h10: mask_for_channel = 6'b000001; // VAUX0
                5'h11: mask_for_channel = 6'b000010; // VAUX1
                5'h12: mask_for_channel = 6'b000100; // VAUX2
                5'h13: mask_for_channel = 6'b001000; // VAUX3
                5'h14: mask_for_channel = 6'b010000; // VAUX4
                5'h16: mask_for_channel = 6'b100000; // VAUX6
                default: mask_for_channel = 6'b000000;
            endcase
        end
    endfunction
    wire [5:0] received_mask = mask_for_channel(read_channel);
    wire [5:0] next_seen = channel_seen | received_mask;

    XADC #(
        .INIT_40(16'h0000),
        .INIT_41(16'h21F0),
        .INIT_42(16'h0400),
        // Enable VAUX0,1,2,3,4 and 6. Returned CHANNEL codes are
        // 10h,11h,12h,13h,14h and 16h.
        // 48h selects internal channels 0..15; 49h selects VAUX 0..15.
        .INIT_48(16'h0000),
        .INIT_49(16'h005F),
        .SIM_DEVICE("ZYNQ")
    ) u_xadc (
        .ALM(xadc_alm),
        .BUSY(xadc_busy),
        .CHANNEL(xadc_channel),
        .DO(xadc_do),
        .DRDY(xadc_drdy),
        .EOC(xadc_eoc),
        .EOS(xadc_eos),
        .JTAGBUSY(xadc_jtagbusy),
        .JTAGLOCKED(xadc_jtaglocked),
        .JTAGMODIFIED(xadc_jtagmodified),
        .MUXADDR(xadc_muxaddr),
        .OT(xadc_ot),
        .CONVST(1'b0),
        .CONVSTCLK(1'b0),
        .DADDR({2'b00, xadc_channel}),
        .DCLK(clk),
        .DEN(xadc_eoc),
        .DI(16'd0),
        .DWE(1'b0),
        .RESET(!rst_n),
        .VAUXN(vauxn),
        .VAUXP(vauxp),
        .VN(1'b0),
        .VP(1'b0)
    );

    always @(posedge clk) begin
        if (!rst_n) begin
            ch0 <= 12'd0;
            ch1 <= 12'd0;
            ch2 <= 12'd0;
            ch3 <= 12'd0;
            ch4 <= 12'd0;
            ch5 <= 12'd0;
            sample_valid <= 1'b0;
            read_channel <= 5'd0;
            channel_seen <= 6'b000000;
        end else begin
            sample_valid <= 1'b0;
            if (xadc_eoc) begin
                read_channel <= xadc_channel;
            end
            if (xadc_drdy) begin
                case (read_channel)
                    5'h10: ch0 <= xadc_do[15:4]; // angle 1
                    5'h11: ch1 <= xadc_do[15:4]; // angle 2
                    5'h12: ch2 <= xadc_do[15:4]; // RP1
                    5'h13: ch3 <= xadc_do[15:4]; // RP2
                    5'h14: ch4 <= xadc_do[15:4]; // RP3
                    5'h16: ch5 <= xadc_do[15:4]; // RP4
                    default: begin end
                endcase
                if (received_mask != 6'b000000) begin
                    if (next_seen == 6'b111111) begin
                        channel_seen <= 6'b000000;
                        sample_valid <= 1'b1;
                    end else begin
                        channel_seen <= next_seen;
                    end
                end
            end
        end
    end
`endif
endmodule

