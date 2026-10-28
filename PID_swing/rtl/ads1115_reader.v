`timescale 1ns/1ps

// Dual ADS1115 front end replacing the XADC reader.
//
//   ADS1115 #1, ADDR pin -> GND (0x48): SENSOR1 pendulum angle on AIN0,
//       continuous conversion, 860 SPS, +/-4.096 V range.  Read every
//       PERIOD_US so the newest sample is at most ~2 ms old.
//   ADS1115 #2, ADDR pin -> VDD (0x49): RP1..RP4 on AIN0..AIN3, single-shot
//       860 SPS, one channel at a time (config, pointer, read = 3 slots).
//
// Both converters share one I2C bus (separate from the OLED bus).  Raw
// 16-bit results (1 LSB = 125 uV, 3.3 V = FS_COUNTS = 26400) are rescaled
// to the 12-bit 0..4095 range used by the STM32 firmware, so every STM32
// threshold (1700/2300/1000/3000/2050) keeps its meaning.
//
// online is low until one complete pass over all channels has finished and
// drops again on any NACK, so the top level can inhibit the motor when an
// ADS1115 is missing or the bus is faulty.
module ads1115_reader #(
    parameter integer CLK_HZ     = 50000000,
    parameter integer I2C_HZ     = 200000,
    parameter integer PERIOD_US  = 1000,
    parameter integer STARTUP_MS = 20,
    parameter integer RETRY_MS   = 20,
    parameter integer FS_COUNTS  = 26400,
    parameter [6:0]   ADDR_ANGLE = 7'h48,
    parameter [6:0]   ADDR_POT   = 7'h49
) (
    input  wire        clk,
    input  wire        rst_n,
    output reg  [11:0] angle1,
    output reg  [11:0] rp1,
    output reg  [11:0] rp2,
    output reg  [11:0] rp3,
    output reg  [11:0] rp4,
    output reg         sample_valid,
    output reg         online,
    output wire        scl_low,
    output wire        sda_low,
    input  wire        sda_in
);
    localparam integer PERIOD_CYCLES = (CLK_HZ / 1000000) * PERIOD_US;
    localparam integer STARTUP_TICKS = (STARTUP_MS * 1000) / PERIOD_US;
    localparam integer RETRY_TICKS   = (RETRY_MS * 1000) / PERIOD_US;
    // 12-bit code = raw * 4095 / FS_COUNTS, as a Q16 reciprocal multiply.
    localparam integer RECIP_Q16 = (4095 * 65536 + FS_COUNTS / 2) / FS_COUNTS;

    localparam [2:0] S_WAIT   = 3'd0;
    localparam [2:0] S_LAUNCH = 3'd1;
    localparam [2:0] S_BUSY   = 3'd2;
    localparam [2:0] S_PERIOD = 3'd3;

    localparam [2:0] A_CFG  = 3'd0;
    localparam [2:0] A_PTR  = 3'd1;
    localparam [2:0] A_READ = 3'd2;
    localparam [2:0] B_CFG  = 3'd3;
    localparam [2:0] B_PTR  = 3'd4;
    localparam [2:0] B_READ = 3'd5;

    reg  [2:0]  state;
    reg  [2:0]  after;
    reg  [1:0]  b_ch;
    reg  [1:0]  b_phase;
    reg  [15:0] wait_ticks;
    reg  [31:0] pcnt;
    reg         tick_pending;

    reg         m_start;
    reg         m_is_read;
    reg  [2:0]  m_count;
    reg  [7:0]  m_t0, m_t1, m_t2, m_t3;
    wire        m_busy, m_done, m_nack;
    wire [15:0] m_rx;

    i2c_ads_master #(.CLK_HZ(CLK_HZ), .I2C_HZ(I2C_HZ)) u_i2c (
        .clk(clk), .rst_n(rst_n), .start(m_start), .is_read(m_is_read),
        .tx_count(m_count), .tx0(m_t0), .tx1(m_t1), .tx2(m_t2), .tx3(m_t3),
        .busy(m_busy), .done(m_done), .nack(m_nack), .rx_data(m_rx),
        .scl_low(scl_low), .sda_low(sda_low), .sda_in(sda_in)
    );

    // raw signed 16-bit conversion result -> clipped 12-bit code.
    reg [47:0] scale_product;
    reg [11:0] scaled_code;
    always @* begin
        scale_product = m_rx[14:0] * RECIP_Q16;
        scaled_code = (m_rx[15]) ? 12'd0 :
                      (scale_product[47:16] > 32'd4095) ? 12'd4095 :
                      scale_product[27:16];
    end

    task automatic load_txn;
        input        is_rd;
        input [2:0]  count;
        input [7:0]  b0;
        input [7:0]  b1;
        input [7:0]  b2;
        input [7:0]  b3;
        begin
            m_is_read <= is_rd;
            m_count   <= count;
            m_t0 <= b0; m_t1 <= b1; m_t2 <= b2; m_t3 <= b3;
            state     <= S_LAUNCH;
        end
    endtask

    // OS=1 | MUX=100+ch | PGA=001 (+/-4.096 V) | MODE=1 (single shot), then
    // DR=111 (860 SPS) | comparator disabled (COMP_QUE=11).
    wire [7:0] pot_cfg_hi = {2'b11, b_ch, 3'b001, 1'b1};

    always @(posedge clk) begin
        m_start      <= 1'b0;
        sample_valid <= 1'b0;
        if (!rst_n) begin
            state        <= S_WAIT;
            after        <= A_CFG;
            b_ch         <= 2'd0;
            b_phase      <= 2'd0;
            wait_ticks   <= STARTUP_TICKS;
            pcnt         <= 32'd0;
            tick_pending <= 1'b0;
            m_is_read    <= 1'b0;
            m_count      <= 3'd0;
            m_t0 <= 8'd0; m_t1 <= 8'd0; m_t2 <= 8'd0; m_t3 <= 8'd0;
            angle1 <= 12'd0; rp1 <= 12'd0; rp2 <= 12'd0;
            rp3 <= 12'd0; rp4 <= 12'd0;
            online <= 1'b0;
        end else begin
            if (pcnt >= PERIOD_CYCLES - 1) begin
                pcnt         <= 32'd0;
                tick_pending <= 1'b1;
            end else begin
                pcnt <= pcnt + 1'b1;
            end

            case (state)
                S_WAIT: if (tick_pending) begin
                    tick_pending <= 1'b0;
                    if (wait_ticks == 0) begin
                        b_ch    <= 2'd0;
                        b_phase <= 2'd0;
                        after   <= A_CFG;
                        load_txn(1'b0, 3'd4, {ADDR_ANGLE, 1'b0}, 8'h01, 8'h42, 8'hE3);
                    end else begin
                        wait_ticks <= wait_ticks - 1'b1;
                    end
                end

                S_LAUNCH: begin
                    m_start <= 1'b1;
                    state   <= S_BUSY;
                end

                S_BUSY: if (m_done) begin
                    if (m_nack) begin
                        online     <= 1'b0;
                        wait_ticks <= RETRY_TICKS;
                        state      <= S_WAIT;
                    end else begin
                        case (after)
                            A_CFG: begin
                                after <= A_PTR;
                                load_txn(1'b0, 3'd2, {ADDR_ANGLE, 1'b0}, 8'h00, 8'h00, 8'h00);
                            end
                            A_PTR: state <= S_PERIOD;
                            A_READ: begin
                                angle1 <= scaled_code;
                                case (b_phase)
                                    2'd0: begin
                                        after <= B_CFG;
                                        load_txn(1'b0, 3'd4, {ADDR_POT, 1'b0}, 8'h01,
                                                 pot_cfg_hi, 8'hE3);
                                    end
                                    2'd1: begin
                                        after <= B_PTR;
                                        load_txn(1'b0, 3'd2, {ADDR_POT, 1'b0}, 8'h00, 8'h00, 8'h00);
                                    end
                                    default: begin
                                        after <= B_READ;
                                        load_txn(1'b1, 3'd1, {ADDR_POT, 1'b1}, 8'h00, 8'h00, 8'h00);
                                    end
                                endcase
                            end
                            B_CFG: begin
                                b_phase <= 2'd1;
                                state   <= S_PERIOD;
                            end
                            B_PTR: begin
                                b_phase <= 2'd2;
                                state   <= S_PERIOD;
                            end
                            default: begin   // B_READ
                                case (b_ch)
                                    2'd0: rp1 <= scaled_code;
                                    2'd1: rp2 <= scaled_code;
                                    2'd2: rp3 <= scaled_code;
                                    default: rp4 <= scaled_code;
                                endcase
                                b_phase <= 2'd0;
                                b_ch    <= b_ch + 1'b1;
                                if (b_ch == 2'd3) begin
                                    online       <= 1'b1;
                                    sample_valid <= 1'b1;
                                end
                                state <= S_PERIOD;
                            end
                        endcase
                    end
                end

                default: if (tick_pending) begin   // S_PERIOD
                    tick_pending <= 1'b0;
                    after <= A_READ;
                    load_txn(1'b1, 3'd1, {ADDR_ANGLE, 1'b1}, 8'h00, 8'h00, 8'h00);
                end
            endcase
        end
    end
endmodule
