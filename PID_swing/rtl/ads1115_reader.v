`timescale 1ns/1ps

// Dual ADS1115 front end with independent angle and potentiometer scheduling.
//
// ADS1115 #1 (0x48) runs continuously at 860 SPS.  Its conversion register is
// read on every 1 ms period, so angle updates are no longer delayed by the
// four-channel potentiometer scan.  ADS1115 #2 (0x49) is polled in single-shot
// mode in the remaining I2C time; one channel completes every four periods,
// and a full four-channel scan completes in about 16 ms.
//
// The I2C bus runs at the ADS1115 fast-mode maximum of 400 kHz.  The ADS1115
// conversion rate remains the hard upper limit: 860 samples/second.
module ads1115_reader #(
    parameter integer CLK_HZ        = 50000000,
    parameter integer I2C_HZ        = 400000,
    parameter integer PERIOD_US     = 1000,
    parameter integer STARTUP_MS    = 20,
    parameter integer RETRY_MS      = 20,
    parameter integer POT_PERIOD_MS = 20,
    parameter integer FS_COUNTS     = 26400,
    parameter [6:0]   ADDR_ANGLE    = 7'h48,
    parameter [6:0]   ADDR_POT      = 7'h49
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
    localparam integer POT_PERIOD_TICKS =
        (POT_PERIOD_MS * 1000) / PERIOD_US;
    localparam integer POT_WAIT_TICKS = 2;
    // 12-bit code = raw * 4095 / FS_COUNTS, as a Q16 reciprocal multiply.
    localparam integer RECIP_Q16 = (4095 * 65536 + FS_COUNTS / 2) / FS_COUNTS;

    localparam [1:0] S_WAIT        = 2'd0;
    localparam [1:0] S_LAUNCH      = 2'd1;
    localparam [1:0] S_BUSY        = 2'd2;
    localparam [1:0] S_POT_SERVICE = 2'd3;

    localparam [2:0] J_ANGLE_CONFIG = 3'd0;
    localparam [2:0] J_ANGLE_PTR    = 3'd1;
    localparam [2:0] J_ANGLE_READ   = 3'd2;
    localparam [2:0] J_POT_CONFIG   = 3'd3;
    localparam [2:0] J_POT_PTR      = 3'd4;
    localparam [2:0] J_POT_READ     = 3'd5;

    localparam [2:0] P_NONE   = 3'd0;
    localparam [2:0] P_WAIT   = 3'd1;
    localparam [2:0] P_CONFIG = 3'd2;
    localparam [2:0] P_PTR    = 3'd3;
    localparam [2:0] P_READ   = 3'd4;

    reg [1:0]  state;
    reg [2:0]  job;
    reg [2:0]  pot_action;
    reg [1:0]  pot_ch;
    reg [15:0] pot_wait_ticks;
    reg [15:0] pot_period_count;
    reg        pot_scan_request;
    reg        angle_ready;
    reg        pot_healthy;
    reg [31:0] wait_ticks;
    reg [31:0] pcnt;
    reg        tick_pending;

    reg        m_start;
    reg        m_is_read;
    reg [2:0]  m_count;
    reg [7:0]  m_t0, m_t1, m_t2, m_t3;
    wire       m_busy, m_done, m_nack;
    wire [15:0] m_rx;

    i2c_ads_master #(.CLK_HZ(CLK_HZ), .I2C_HZ(I2C_HZ)) u_i2c (
        .clk(clk), .rst_n(rst_n), .start(m_start), .is_read(m_is_read),
        .tx_count(m_count), .tx0(m_t0), .tx1(m_t1), .tx2(m_t2), .tx3(m_t3),
        .busy(m_busy), .done(m_done), .nack(m_nack), .rx_data(m_rx),
        .scl_low(scl_low), .sda_low(sda_low), .sda_in(sda_in)
    );

    // Raw signed conversion result -> clipped 12-bit STM32 code.
    reg [47:0] scale_product;
    reg [11:0] scaled_code;
    always @* begin
        scale_product = m_rx[14:0] * RECIP_Q16;
        scaled_code = (m_rx[15]) ? 12'd0 :
                      (scale_product[47:16] > 32'd4095) ? 12'd4095 :
                      scale_product[27:16];
    end

    // OS=1 | MUX=100+ch | PGA=001 (+/-4.096 V) | MODE=1 (single shot),
    // then DR=111 (860 SPS) | comparator disabled.
    wire [7:0] pot_cfg_hi = {2'b11, pot_ch, 3'b001, 1'b1};

    task automatic load_job;
        input [2:0] next_job;
        input       is_rd;
        input [2:0] count;
        input [7:0] b0;
        input [7:0] b1;
        input [7:0] b2;
        input [7:0] b3;
        begin
            job       <= next_job;
            m_is_read <= is_rd;
            m_count   <= count;
            m_t0 <= b0; m_t1 <= b1; m_t2 <= b2; m_t3 <= b3;
            state <= S_LAUNCH;
        end
    endtask

    always @(posedge clk) begin
        m_start      <= 1'b0;
        sample_valid <= 1'b0;

        if (!rst_n) begin
            state             <= S_WAIT;
            job               <= J_ANGLE_CONFIG;
            pot_action        <= P_NONE;
            pot_ch            <= 2'd0;
            pot_wait_ticks    <= 16'd0;
            pot_period_count  <= 16'd0;
            pot_scan_request  <= 1'b0;
            angle_ready       <= 1'b0;
            pot_healthy       <= 1'b0;
            wait_ticks        <= STARTUP_TICKS;
            pcnt              <= 32'd0;
            tick_pending      <= 1'b0;
            m_is_read         <= 1'b0;
            m_count           <= 3'd0;
            m_t0 <= 8'd0; m_t1 <= 8'd0; m_t2 <= 8'd0; m_t3 <= 8'd0;
            angle1 <= 12'd0;
            rp1 <= 12'd0; rp2 <= 12'd0; rp3 <= 12'd0; rp4 <= 12'd0;
            online <= 1'b0;
        end else begin
            // The angle scheduler remains at 1 kHz, independent of the I2C FSM.
            if (pcnt >= PERIOD_CYCLES - 1) begin
                pcnt         <= 32'd0;
                tick_pending <= 1'b1;
            end else begin
                pcnt <= pcnt + 1'b1;
            end

            case (state)
                S_WAIT: if (tick_pending) begin
                    tick_pending <= 1'b0;
                    if (wait_ticks != 0) begin
                        wait_ticks <= wait_ticks - 1'b1;
                    end else if (!angle_ready) begin
                        // Continuous mode, AIN0, +/-4.096 V, 860 SPS.
                        load_job(J_ANGLE_CONFIG, 1'b0, 3'd4,
                                 {ADDR_ANGLE, 1'b0}, 8'h01, 8'h42, 8'hE3);
                    end else begin
                        // Preserve a 20 ms pot refresh period while allowing
                        // the angle read to run on every 1 ms tick.
                        if (pot_period_count == 0) begin
                            pot_period_count <= POT_PERIOD_TICKS[15:0] - 1'b1;
                            pot_scan_request <= 1'b1;
                        end else begin
                            pot_period_count <= pot_period_count - 1'b1;
                        end
                        if ((pot_action == P_WAIT) && (pot_wait_ticks != 0)) begin
                            if (pot_wait_ticks <= 1) begin
                                pot_wait_ticks <= 16'd0;
                                pot_action     <= P_PTR;
                            end else begin
                                pot_wait_ticks <= pot_wait_ticks - 1'b1;
                            end
                        end
                        if (pot_scan_request && (pot_action == P_NONE)) begin
                            pot_ch           <= 2'd0;
                            pot_action       <= P_CONFIG;
                            pot_scan_request <= 1'b0;
                        end
                        load_job(J_ANGLE_READ, 1'b1, 3'd1,
                                 {ADDR_ANGLE, 1'b1}, 8'h00, 8'h00, 8'h00);
                    end
                end

                S_LAUNCH: begin
                    m_start <= 1'b1;
                    state   <= S_BUSY;
                end

                S_BUSY: if (m_done) begin
                    if (m_nack) begin
                        online           <= 1'b0;
                        pot_action       <= P_NONE;
                        pot_scan_request <= 1'b0;
                        if (job <= J_ANGLE_READ) begin
                            // Angle converter failure is safety-critical.
                            angle_ready <= 1'b0;
                            pot_healthy <= 1'b0;
                            wait_ticks  <= RETRY_TICKS;
                        end else begin
                            // A missing pot converter does not stop angle data;
                            // retry it in the background.
                            pot_healthy      <= 1'b0;
                            pot_period_count <= RETRY_TICKS[15:0];
                        end
                        state <= S_WAIT;
                    end else begin
                        case (job)
                            J_ANGLE_CONFIG: begin
                                load_job(J_ANGLE_PTR, 1'b0, 3'd2,
                                         {ADDR_ANGLE, 1'b0}, 8'h00, 8'h00, 8'h00);
                            end
                            J_ANGLE_PTR: begin
                                angle_ready <= 1'b1;
                                state       <= S_WAIT;
                            end
                            J_ANGLE_READ: begin
                                angle1 <= scaled_code;
                                online <= pot_healthy;
                                state  <= S_POT_SERVICE;
                            end
                            J_POT_CONFIG: begin
                                pot_action     <= P_WAIT;
                                pot_wait_ticks <= POT_WAIT_TICKS;
                                state          <= S_WAIT;
                            end
                            J_POT_PTR: begin
                                pot_action <= P_READ;
                                state      <= S_WAIT;
                            end
                            J_POT_READ: begin
                                case (pot_ch)
                                    2'd0: rp1 <= scaled_code;
                                    2'd1: rp2 <= scaled_code;
                                    2'd2: rp3 <= scaled_code;
                                    default: rp4 <= scaled_code;
                                endcase
                                if (pot_ch == 2'd3) begin
                                    pot_ch          <= 2'd0;
                                    pot_action      <= P_NONE;
                                    pot_healthy     <= 1'b1;
                                    online          <= 1'b1;
                                    sample_valid    <= 1'b1;
                                end else begin
                                    pot_ch     <= pot_ch + 1'b1;
                                    pot_action <= P_CONFIG;
                                end
                                state <= S_WAIT;
                            end
                            default: state <= S_WAIT;
                        endcase
                    end
                end

                S_POT_SERVICE: begin
                    case (pot_action)
                        P_CONFIG: begin
                            load_job(J_POT_CONFIG, 1'b0, 3'd4,
                                     {ADDR_POT, 1'b0}, 8'h01,
                                     pot_cfg_hi, 8'hE3);
                        end
                        P_PTR: begin
                            load_job(J_POT_PTR, 1'b0, 3'd2,
                                     {ADDR_POT, 1'b0}, 8'h00, 8'h00, 8'h00);
                        end
                        P_READ: begin
                            load_job(J_POT_READ, 1'b1, 3'd1,
                                     {ADDR_POT, 1'b1}, 8'h00, 8'h00, 8'h00);
                        end
                        default: state <= S_WAIT;
                    endcase
                end
                default: state <= S_WAIT;
            endcase
        end
    end
endmodule
