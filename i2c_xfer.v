// ============================================================
//  i2c_xfer.v —— 把"一次完整的 I2C 事务"封成一个模块
//
//  为什么要有这一层：
//    `i2c_master` 的接口是**单字节原子命令**（start/wr/rd/stop），
//    一次"读 MPR121 的 0x00/0x01 两个字节"需要 8 条命令、且顺序不能错。
//    如果 mpr121_cfg / touch_keys 各自手拼命令序列，会有三份几乎相同、
//    却各自可能写错一处（比如忘了重复起始）的状态机。
//    所以这里封两次操作：
//      · 写一个寄存器： start → wr(addr|W) → wr(reg) → wr(data) → stop
//      · 读 n 个字节 ： start → wr(addr|W) → wr(reg) → **start(Sr)** → wr(addr|R)
//                      → rd(ACK) … → rd(NACK) → stop
//
//  ⚠️ 读的时候第 4 步是**重复起始（Sr）**，不是 stop：
//     写成 stop 再 start 也能"通"，但中间会释放总线，某些器件会因此
//     认为事务结束、把指针复位 —— 这是 I2C 调试里最典型的"偶发读错一格"。
//
//  错误语义：
//    · 收到 NACK ⇒ err 置 1，但**序列继续走完**（含 stop），保证总线被正常释放
//    · 超时      ⇒ err 置 1，序列**立即中止**（master 已经把总线释放回空闲，
//                  再发 stop 反而会造出一个假起始条件）
// ============================================================
`default_nettype none

module i2c_xfer (
    input  wire       clk,
    input  wire       rst_n,

    // ---- 一次事务 ----
    input  wire       go,         // 单周期脉冲：开始
    input  wire       op_read,    // 0 = 写一个寄存器；1 = 读 rd_len 个字节
    input  wire [6:0] dev,        // 7 位从地址
    input  wire [7:0] reg_addr,   // 寄存器指针
    input  wire [7:0] wr_data,    // 写的数据
    input  wire [1:0] rd_len,     // 读几个字节（1~3；0 当 1 处理）

    output reg  [7:0] rd0,        // 读回的字节
    output reg  [7:0] rd1,
    output reg  [7:0] rd2,
    output reg        busy,
    output reg        done,       // 单周期脉冲：事务结束
    output reg        err,        // 事务期间出现过 NACK 或超时

    // ---- 接 i2c_master ----
    output reg        m_start,
    output reg        m_wr,
    output reg        m_rd,
    output reg        m_rd_ack,
    output reg        m_stop,
    output reg  [7:0] m_tx,
    input  wire [7:0] m_rx,
    input  wire       m_done,
    input  wire       m_nack,
    input  wire       m_timeout
);

    localparam [1:0]
        X_IDLE  = 2'd0,
        X_ISSUE = 2'd1,   // 发出一条命令
        X_WAIT  = 2'd2,   // 等 m_done
        X_FIN   = 2'd3;

    reg [1:0] state;
    reg [3:0] step;
    reg [3:0] last_step;
    reg       is_read;
    reg [1:0] nlen;
    reg [6:0] dev_r;
    reg [7:0] reg_r;
    reg [7:0] wdat_r;

    // 读事务的字节下标（step 5 起）
    wire [3:0] rd_idx    = (step >= 4'd5) ? (step - 4'd5) : 4'd0;
    // 最后一个字节回 NACK，其余回 ACK
    wire       rd_ack_lo = (rd_idx < (nlen - 2'd1));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= X_IDLE;
            step      <= 4'd0;
            last_step <= 4'd0;
            is_read   <= 1'b0;
            nlen      <= 2'd1;
            dev_r     <= 7'd0;
            reg_r     <= 8'd0;
            wdat_r    <= 8'd0;
            rd0       <= 8'd0;
            rd1       <= 8'd0;
            rd2       <= 8'd0;
            busy      <= 1'b0;
            done      <= 1'b0;
            err       <= 1'b0;
            m_start   <= 1'b0;
            m_wr      <= 1'b0;
            m_rd      <= 1'b0;
            m_rd_ack  <= 1'b0;
            m_stop    <= 1'b0;
            m_tx      <= 8'd0;
        end else begin
            // 命令脉冲默认清零
            m_start <= 1'b0;
            m_wr    <= 1'b0;
            m_rd    <= 1'b0;
            m_stop  <= 1'b0;
            done    <= 1'b0;

            case (state)
            X_IDLE: begin
                if (go) begin
                    is_read   <= op_read;
                    nlen      <= (rd_len == 2'd0) ? 2'd1 : rd_len;
                    dev_r     <= dev;
                    reg_r     <= reg_addr;
                    wdat_r    <= wr_data;
                    err       <= 1'b0;
                    step      <= 4'd0;
                    busy      <= 1'b1;
                    // 写：5 步(0..4)，最后一步是 stop
                    // 读：0..4 是 起始/地址/指针/Sr/地址，5..(4+n) 是数据，最后一步 (5+n) 是 stop
                    //     ⇒ last_step = 5 + nlen（第一版写成 4+nlen，少了一个字节，被 tb 抓到）
                    last_step <= op_read ? (4'd5 + ((rd_len == 2'd0) ? 2'd1 : rd_len))
                                         : 4'd4;
                    state     <= X_ISSUE;
                end
            end

            X_ISSUE: begin
                if (is_read == 1'b0) begin
                    // ---- 写一个寄存器 ----
                    case (step)
                    4'd0: begin m_start <= 1'b1;                            end
                    4'd1: begin m_wr <= 1'b1; m_tx <= {dev_r, 1'b0};        end
                    4'd2: begin m_wr <= 1'b1; m_tx <= reg_r;                end
                    4'd3: begin m_wr <= 1'b1; m_tx <= wdat_r;               end
                    default: begin m_stop <= 1'b1;                          end
                    endcase
                end else begin
                    // ---- 读 n 个字节 ----
                    //  0:start 1:wr(addr|W) 2:wr(reg) 3:start(Sr) 4:wr(addr|R)
                    //  5..5+n-1: rd          5+n: stop
                    case (step)
                    4'd0: begin m_start <= 1'b1;                            end
                    4'd1: begin m_wr <= 1'b1; m_tx <= {dev_r, 1'b0};        end
                    4'd2: begin m_wr <= 1'b1; m_tx <= reg_r;                end
                    4'd3: begin m_start <= 1'b1;                            end
                    4'd4: begin m_wr <= 1'b1; m_tx <= {dev_r, 1'b1};        end
                    default: begin
                        if (step == last_step) begin
                            m_stop <= 1'b1;
                        end else begin
                            m_rd     <= 1'b1;
                            m_rd_ack <= rd_ack_lo;
                        end
                    end
                    endcase
                end
                state <= X_WAIT;
            end

            X_WAIT: begin
                if (m_done) begin
                    if (m_timeout) begin
                        // master 已经把总线放回空闲；不再造 stop，直接收尾
                        err   <= 1'b1;
                        state <= X_FIN;
                    end else begin
                        if (m_nack) err <= 1'b1;
                        // 收数据：读命令的步号是 5..5+n-1
                        if (is_read && (step >= 4'd5) && (step < last_step)) begin
                            case (rd_idx)
                            4'd0: rd0 <= m_rx;
                            4'd1: rd1 <= m_rx;
                            default: rd2 <= m_rx;
                            endcase
                        end
                        if (step == last_step) state <= X_FIN;
                        else begin
                            step  <= step + 4'd1;
                            state <= X_ISSUE;
                        end
                    end
                end
            end

            X_FIN: begin
                done  <= 1'b1;
                busy  <= 1'b0;
                state <= X_IDLE;
            end

            default: state <= X_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire