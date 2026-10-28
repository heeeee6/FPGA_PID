`timescale 1ns/1ps

// XADC sequencer for the complete STM32-compatible connector set:
// VAUX0 = angle sensor, VAUX1..VAUX4 = RP1..RP4.  Each channel must be
// conditioned to the Zynq XADC 0..1 V range before reaching the package pin.
module xadc5_reader (
    input wire clk,
    input wire rst_n,
    input wire [4:0] vauxp,
    input wire [4:0] vauxn,
    output reg [11:0] angle,
    output reg [11:0] rp1,
    output reg [11:0] rp2,
    output reg [11:0] rp3,
    output reg [11:0] rp4,
    output reg sample_valid
);
`ifdef SIMULATION
    always @(posedge clk) begin
        if (!rst_n) begin
            angle <= 12'd0; rp1 <= 12'd0; rp2 <= 12'd0;
            rp3 <= 12'd0; rp4 <= 12'd0; sample_valid <= 1'b0;
        end else sample_valid <= 1'b0;
    end
`else
    wire [15:0] xadc_do;
    wire [4:0] xadc_channel;
    wire xadc_eoc, xadc_drdy;
    wire [7:0] xadc_alm;
    wire xadc_busy, xadc_eos, xadc_jtagbusy, xadc_jtaglocked, xadc_jtagmodified;
    wire [4:0] xadc_muxaddr;
    wire xadc_ot;
    reg [4:0] read_channel;
    reg [4:0] channel_seen;
    wire [15:0] xadc_vauxp = {11'b0, vauxp};
    wire [15:0] xadc_vauxn = {11'b0, vauxn};

    function [4:0] mask_for_channel;
        input [4:0] channel;
        begin
            case (channel)
                5'h10: mask_for_channel = 5'b00001;
                5'h11: mask_for_channel = 5'b00010;
                5'h12: mask_for_channel = 5'b00100;
                5'h13: mask_for_channel = 5'b01000;
                5'h14: mask_for_channel = 5'b10000;
                default: mask_for_channel = 5'b00000;
            endcase
        end
    endfunction
    wire [4:0] received_mask = mask_for_channel(read_channel);
    wire [4:0] next_seen = channel_seen | received_mask;

    XADC #(
        .INIT_40(16'h0000), .INIT_41(16'h21F0), .INIT_42(16'h0400),
        // 0x001F selects VAUX0,1,2,3,4 in the XADC sequencer.
        .INIT_48(16'h0000), .INIT_49(16'h001F), .SIM_DEVICE("ZYNQ")
    ) u_xadc (
        .ALM(xadc_alm), .BUSY(xadc_busy), .CHANNEL(xadc_channel), .DO(xadc_do),
        .DRDY(xadc_drdy), .EOC(xadc_eoc), .EOS(xadc_eos), .JTAGBUSY(xadc_jtagbusy),
        .JTAGLOCKED(xadc_jtaglocked), .JTAGMODIFIED(xadc_jtagmodified),
        .MUXADDR(xadc_muxaddr), .OT(xadc_ot), .CONVST(1'b0), .CONVSTCLK(1'b0),
        .DADDR({2'b00,xadc_channel}), .DCLK(clk), .DEN(xadc_eoc), .DI(16'd0),
        .DWE(1'b0), .RESET(!rst_n), .VAUXN(xadc_vauxn), .VAUXP(xadc_vauxp),
        .VN(1'b0), .VP(1'b0)
    );

    always @(posedge clk) begin
        if (!rst_n) begin
            angle <= 12'd0; rp1 <= 12'd0; rp2 <= 12'd0;
            rp3 <= 12'd0; rp4 <= 12'd0; sample_valid <= 1'b0;
            read_channel <= 5'd0; channel_seen <= 5'd0;
        end else begin
            sample_valid <= 1'b0;
            if (xadc_eoc) read_channel <= xadc_channel;
            if (xadc_drdy) begin
                case (read_channel)
                    5'h10: angle <= xadc_do[15:4];
                    5'h11: rp1   <= xadc_do[15:4];
                    5'h12: rp2   <= xadc_do[15:4];
                    5'h13: rp3   <= xadc_do[15:4];
                    5'h14: rp4   <= xadc_do[15:4];
                    default: begin end
                endcase
                if (received_mask != 5'b0) begin
                    if (next_seen == 5'b11111) begin
                        channel_seen <= 5'b0;
                        sample_valid <= 1'b1;
                    end else channel_seen <= next_seen;
                end
            end
        end
    end
`endif
endmodule
