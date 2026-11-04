// ============================================================
//  uart_rx.v —— 8N1 接收器（异步输入，2 级同步 + 中位采样）
//
//  ---- 三个必须做对的地方 ----
//    1) **2 级同步**：`rx` 是异步输入，直接进状态机会采到亚稳态。
//    2) **中位采样**：检测到 start 的下降沿后，先等半个位宽再确认，
//       之后每个位宽采一次 —— 采在位的正中间，容错最大（±半位）。
//    3) **停止位校验**：停止位不是 1 就是帧错误，宁可丢这一字节也不要错位。
//
//  ---- 用途 ----
//    P2 主要用 tx 打印；rx 是给"上位机下命令"留的（比如让 FPGA 切演示态）。
//    tb 里用 tx→rx 自环来验证两个模块。
// ============================================================
`default_nettype none

module uart_rx #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer BAUD   = 115_200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx,         // 异步输入，空闲为高
    output reg  [7:0] data,
    output reg        valid,      // 单周期脉冲：收到一个正确字节
    output reg        frm_err     // 单周期脉冲：停止位不对，本字节丢弃
);

    localparam integer DIV_RAW = CLK_HZ / BAUD;
    localparam integer DIV     = (DIV_RAW < 1) ? 1 : DIV_RAW;
    localparam integer HALF    = (DIV / 2 < 1) ? 1 : DIV / 2;

    // 2 级同步
    reg [1:0] rx_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) rx_sync <= 2'b11;
        else        rx_sync <= {rx_sync[0], rx};
    end
    wire rx_i = rx_sync[1];

    localparam [1:0]
        R_IDLE  = 2'd0,
        R_START = 2'd1,   // 等半个位宽，确认仍为低
        R_DATA  = 2'd2,   // 每位中间采一次，共 8 次
        R_STOP  = 2'd3;   // 采停止位

    reg [1:0]  state;
    reg [15:0] cnt;
    reg [2:0]  bit_i;
    reg [7:0]  sh;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= R_IDLE;
            cnt      <= 16'd0;
            bit_i    <= 3'd0;
            sh       <= 8'd0;
            data     <= 8'd0;
            valid    <= 1'b0;
            frm_err  <= 1'b0;
        end else begin
            valid   <= 1'b0;
            frm_err <= 1'b0;

            case (state)
            R_IDLE: begin
                cnt <= 16'd0;
                if (rx_i == 1'b0) state <= R_START;   // 起始位下降沿
            end

            R_START: begin
                if (cnt == (HALF - 1)) begin
                    cnt <= 16'd0;
                    if (rx_i == 1'b0) begin           // 确认是真的起始位
                        bit_i <= 3'd0;
                        state <= R_DATA;
                    end else begin
                        state <= R_IDLE;              // 毛刺，忽略
                    end
                end else cnt <= cnt + 16'd1;
            end

            R_DATA: begin
                if (cnt == (DIV - 1)) begin
                    cnt <= 16'd0;
                    sh  <= {rx_i, sh[7:1]};           // LSB 先到
                    if (bit_i == 3'd7) state <= R_STOP;
                    else               bit_i <= bit_i + 3'd1;
                end else cnt <= cnt + 16'd1;
            end

            R_STOP: begin
                if (cnt == (DIV - 1)) begin
                    cnt <= 16'd0;
                    state <= R_IDLE;
                    if (rx_i == 1'b1) begin
                        data  <= sh;
                        valid <= 1'b1;
                    end else begin
                        frm_err <= 1'b1;              // 停止位为低 = 帧错误
                    end
                end else cnt <= cnt + 16'd1;
            end

            default: state <= R_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire