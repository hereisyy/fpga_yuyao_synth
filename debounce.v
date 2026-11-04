// ============================================================
//  去抖（状态机版）
//
//  按键在按下/松开的瞬间会抖动几十微秒，必须滤掉，
//  否则一个音会响好几下。
//
//  工作原理：
//    只有当输入持续保持"与当前输出不同"达到 CNT_MAX 个时钟，
//    才承认这次变化；中途抖回去就退回 IDLE 重新计时。
//
//  参数：
//    CNT_WIDTH : 计数位宽（决定能表示多大的计数值）
//    CNT_MAX   : 需要连续保持多少个时钟才确认变化（默认 65535）
//
//  去抖时长 = (CNT_MAX + 1) / 时钟频率
//    100 MHz, CNT_MAX =  8191 ->  82 us   ← 推荐（手感好）
//    100 MHz, CNT_MAX = 65535 -> 655 us   ← 旧值，偏保守
//     50 MHz, CNT_MAX =  4095 ->  82 us
//
//  ⚠️ CNT_MAX 必须 < 2^CNT_WIDTH，否则计数器永远到不了，按键会失灵。
//     （CNT_WIDTH=16 -> CNT_MAX 最大 65535）
// ============================================================
module debounce #(
    parameter CNT_WIDTH = 16,
    parameter CNT_MAX   = 65535
)(
    input  wire clk,
    input  wire rst_n,
    input  wire key_in,               // 原始按键（会抖）
    output reg  key_out               // 去抖后（稳定）
);

    localparam S_IDLE = 2'd0;         // 空闲：输入和输出一致
    localparam S_WAIT = 2'd1;         // 发现变化，开始计时

    reg [1:0]           state;
    reg [CNT_WIDTH-1:0] cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state   <= S_IDLE;
            cnt     <= {CNT_WIDTH{1'b0}};
            key_out <= 1'b0;
        end else begin
            case (state)
                S_IDLE: begin
                    cnt <= {CNT_WIDTH{1'b0}};
                    if (key_in != key_out)        // 发现变化
                        state <= S_WAIT;
                end
                S_WAIT: begin
                    if (key_in != key_out) begin
                        if (cnt == CNT_MAX[CNT_WIDTH-1:0]) begin
                            key_out <= key_in;    // 一直是新值 -> 确认
                            state   <= S_IDLE;
                            cnt     <= {CNT_WIDTH{1'b0}};
                        end else
                            cnt <= cnt + 1'b1;
                    end else
                        state <= S_IDLE;          // 抖回去了 -> 不算
                end
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
