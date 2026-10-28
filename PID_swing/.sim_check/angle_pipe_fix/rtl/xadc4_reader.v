`timescale 1ns/1ps

// Zynq-7000 XADC front end. The four top-level analog pairs connect in this
// order to VAUX0, VAUX2, VAUX3 and VAUX4. Each 0..3.3 V sensor needs external
// conditioning to the XADC's approximately 0..1 V unipolar input range.
module xadc4_reader (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [3:0]  vauxp,
    input  wire [3:0]  vauxn,
    output reg  [11:0] ch0,
    output reg  [11:0] ch1,
    output reg  [11:0] ch2,
    output reg  [11:0] ch3,
    output reg         sample_valid
);
`ifdef SIMULATION
    always @(posedge clk) begin
        if (!rst_n) begin
            ch0 <= 12'd0;
            ch1 <= 12'd0;
            ch2 <= 12'd0;
            ch3 <= 12'd0;
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
    reg [3:0] channel_seen;
    wire [15:0] xadc_vauxp = {11'b0, vauxp[3:1], 1'b0, vauxp[0]};
    wire [15:0] xadc_vauxn = {11'b0, vauxn[3:1], 1'b0, vauxn[0]};

    function [3:0] mask_for_channel;
        input [4:0] channel;
        begin
            case (channel)
                5'h10: mask_for_channel = 4'b0001;
                5'h12: mask_for_channel = 4'b0010;
                5'h13: mask_for_channel = 4'b0100;
                5'h14: mask_for_channel = 4'b1000;
                default: mask_for_channel = 4'b0000;
            endcase
        end
    endfunction
    wire [3:0] received_mask = mask_for_channel(read_channel);
    wire [3:0] next_seen = channel_seen | received_mask;

    XADC #(
        .INIT_40(16'h0000),
        .INIT_41(16'h21F0),
        .INIT_42(16'h0400),
        // Sequence VAUX0, VAUX2, VAUX3 and VAUX4, in the vauxp/n[0..3]
        // order. Returned CHANNEL codes are 10h, 12h, 13h and 14h.
        // 48h selects internal channels 0..15; 49h selects VAUX 0..15.
        .INIT_48(16'h0000),
        .INIT_49(16'h001D),
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
        .VAUXN(xadc_vauxn),
        .VAUXP(xadc_vauxp),
        .VN(1'b0),
        .VP(1'b0)
    );

    always @(posedge clk) begin
        if (!rst_n) begin
            ch0 <= 12'd0;
            ch1 <= 12'd0;
            ch2 <= 12'd0;
            ch3 <= 12'd0;
            sample_valid <= 1'b0;
            read_channel <= 5'd0;
            channel_seen <= 4'b0000;
        end else begin
            sample_valid <= 1'b0;
            if (xadc_eoc) begin
                read_channel <= xadc_channel;
            end
            if (xadc_drdy) begin
                case (read_channel)
                    5'h10: ch0 <= xadc_do[15:4];
                    5'h12: ch1 <= xadc_do[15:4];
                    5'h13: ch2 <= xadc_do[15:4];
                    5'h14: ch3 <= xadc_do[15:4];
                    default: begin end
                endcase
                if (received_mask != 4'b0000) begin
                    if (next_seen == 4'b1111) begin
                        channel_seen <= 4'b0000;
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

