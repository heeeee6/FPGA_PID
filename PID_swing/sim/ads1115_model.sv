`timescale 1ns/1ps

// Behavioral ADS1115 slave for simulation.  Supports pointer/config writes
// and 16-bit conversion-register reads.  The conversion register always
// returns the AINx input selected by the MUX field of the config register
// (single-ended codes 100..111 -> AIN0..AIN3).  present=0 makes the device
// ignore its address (no ACK), which models a missing module.
module ads1115_model #(
    parameter [6:0] ADDR = 7'h48
) (
    input  wire        scl,
    inout  wire        sda,
    input  wire        present,
    input  wire [15:0] ain0,
    input  wire [15:0] ain1,
    input  wire [15:0] ain2,
    input  wire [15:0] ain3,
    output reg  [15:0] cfg_reg,
    output reg  [31:0] cfg_writes,
    output reg  [31:0] reads
);
    localparam [2:0] ST_IDLE = 3'd0, ST_ADDR = 3'd1, ST_WR = 3'd2,
                     ST_ACKD = 3'd3, ST_ACKH = 3'd4, ST_RD = 3'd5, ST_MACK = 3'd6;

    reg        sda_o;
    reg [2:0]  st;
    reg [3:0]  cnt;
    reg [7:0]  sh;
    reg        rw;
    reg        addr_ok;
    reg [1:0]  ptr;
    reg [2:0]  wcount;
    reg [7:0]  cfg_hi;
    reg        master_nack;
    reg        rd_second;
    reg [15:0] txword;
    reg [3:0]  txbit;
    reg        after_ack_rd;
    assign sda = sda_o ? 1'b0 : 1'bz;

    function [15:0] conv_value;
        input [2:0] mux;
        begin
            case (mux[1:0])
                2'd0: conv_value = ain0;
                2'd1: conv_value = ain1;
                2'd2: conv_value = ain2;
                default: conv_value = ain3;
            endcase
        end
    endfunction

    initial begin
        sda_o = 1'b0; st = ST_IDLE; cnt = 0; sh = 0; rw = 0; addr_ok = 0;
        ptr = 0; wcount = 0; cfg_hi = 0; master_nack = 0; rd_second = 0;
        txword = 0; txbit = 0; after_ack_rd = 0;
        cfg_reg = 16'h8583; cfg_writes = 0; reads = 0;
    end

    always @(negedge sda) if (scl === 1'b1) begin   // START
        st <= ST_ADDR; cnt <= 0; wcount <= 0;
    end
    always @(posedge sda) if (scl === 1'b1) begin   // STOP
        st <= ST_IDLE; sda_o <= 1'b0;
    end

    always @(posedge scl) begin
        case (st)
            ST_ADDR, ST_WR: begin
                sh  = {sh[6:0], sda};
                cnt = cnt + 1'b1;
                if (cnt == 4'd8) begin
                    cnt = 0;
                    if (st == ST_ADDR) begin
                        addr_ok = present && (sh[7:1] == ADDR);
                        rw      = sh[0];
                        after_ack_rd = sh[0];
                    end else begin
                        addr_ok = 1'b1;
                        if (wcount == 0) ptr = sh[1:0];
                        else if (wcount == 1) cfg_hi = sh;
                        else begin
                            cfg_reg = {cfg_hi, sh};
                            cfg_writes = cfg_writes + 1;
                        end
                        wcount = wcount + 1'b1;
                    end
                    st <= ST_ACKD;
                end
            end
            ST_MACK: master_nack = sda;
            default: begin end
        endcase
    end

    always @(negedge scl) begin
        case (st)
            ST_ACKD: begin
                if (addr_ok) begin
                    sda_o <= 1'b1;               // ACK
                    st    <= ST_ACKH;
                end else begin
                    st <= ST_IDLE;
                end
            end
            ST_ACKH: begin
                sda_o <= 1'b0;
                if (after_ack_rd && rw) begin
                    after_ack_rd = 1'b0;
                    rd_second = 1'b0;
                    txword = (ptr == 2'd1) ? cfg_reg : conv_value(cfg_reg[14:12]);
                    reads = reads + 1;
                    txbit = 0;
                    sda_o <= ~txword[15];
                    txbit = 4'd1;
                    st <= ST_RD;
                end else begin
                    st <= ST_WR;
                end
            end
            ST_RD: begin
                if (txbit < 4'd8) begin
                    sda_o <= ~txword[15 - (rd_second ? 8 : 0) - txbit];
                    txbit = txbit + 1'b1;
                end else begin
                    sda_o <= 1'b0;
                    st <= ST_MACK;
                end
            end
            ST_MACK: begin
                if (!master_nack && !rd_second) begin
                    rd_second = 1'b1;
                    txbit = 4'd1;
                    sda_o <= ~txword[7];
                    st <= ST_RD;
                end else begin
                    st <= ST_IDLE;
                end
            end
            default: begin end
        endcase
    end
endmodule
