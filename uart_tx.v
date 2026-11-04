// ============================================================
//  uart_tx.v —— 8N1 发送器（115200 起，可参数化到 1~3 Mbps）
//
//  ---- 为什么 P2 需要它 ----
//    上板时它是**唯一能看到"里面发生了什么"的通道**（板子上只有 1 颗 RGB）。
//    P2 的 bring-up 顺序是：I2C 能不能 ACK → 逐键 → 同时 8 键，
//    这三步全靠串口打印。没有它，就等于闭着眼睛接传感器。
//    另外它也是统筹同学做 FFT 自动判分的数据通道。
//
//  ---- 电气/拓扑 ----
//    板载 BL616 调试器把 FPGA 的 `FPGA_UART.TX/RX`（ball **U15 / V14**）
//    桥成 PC 的**虚拟串口**（Type-C 插上就出现一个 COM 口）。
//    所以 FPGA 这边只需要一根 tx、一根 rx，不需要 USB 协议栈。
//
//  ---- 参数 ----
//    `DIV = CLK_HZ / BAUD`（整数除法）。以 50 MHz / 115200 为例：
//      DIV = 434 → 实际波特率 50e6/434 = 115207（误差 +0.006%，可忽略）
//    tb 里用 4 MHz / 200 kHz（DIV=20）跑得快。
// ============================================================
`default_nettype none

module uart_tx #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer BAUD   = 115_200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       send,      // 单周期脉冲：发一个字节
    input  wire [7:0] data,
    output reg        tx,        // 空闲为高
    output reg        busy
);

    localparam integer DIV_RAW = CLK_HZ / BAUD;
    localparam integer DIV     = (DIV_RAW < 1) ? 1 : DIV_RAW;

    reg [15:0] cnt;      // 位内计数
    reg [3:0]  bit_i;    // 0=start, 1..8=data, 9=stop
    reg [9:0]  sr;       // {stop, data[7:0], start}

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx    <= 1'b1;
            busy  <= 1'b0;
            cnt   <= 16'd0;
            bit_i <= 4'd0;
            sr    <= 10'h3FF;
        end else if (!busy) begin
            tx <= 1'b1;
            if (send) begin
                sr    <= {1'b1, data, 1'b0};   // LSB 先发：sr[0] = start 位
                tx    <= 1'b0;                 // 立刻拉低 = start 位
                busy  <= 1'b1;
                cnt   <= 16'd0;
                bit_i <= 4'd0;
            end
        end else begin
            if (cnt == (DIV - 1)) begin
                cnt <= 16'd0;
                if (bit_i == 4'd9) begin
                    busy <= 1'b0;              // stop 位已发完
                    tx   <= 1'b1;
                end else begin
                    sr    <= {1'b1, sr[9:1]};  // 右移，高位补 1（无名位填 idle）
                    tx    <= sr[1];            // 下一个位
                    bit_i <= bit_i + 4'd1;
                end
            end else begin
                cnt <= cnt + 16'd1;
            end
        end
    end

endmodule

`default_nettype wire