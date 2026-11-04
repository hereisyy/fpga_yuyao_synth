// ============================================================
//  i2c_master.v —— 标准 I2C 主机（开漏、单字节原子操作、支持时钟延展）
//
//  用途：P2 的第一块砖。唯一客户是 MPR121（12 键触摸）。
//        将来若要补"表情"连续量，同一根总线挂 ADS1115 也能复用本模块。
//
//  ---- 接口语义（很重要，用错了会丢字节）----
//  这不是"给一个地址+长度，自动搬一串字节"的搬运器，而是**一次一条原子命令**：
//    start / wr / rd / stop 各是一个**单周期脉冲**，每执行完一条：
//       busy 拉低、done 打一个**单周期脉冲**、op_count 加一。
//    调用方必须**串行**发命令：等 done 再发下一条。典型时序见文件末的"用法"。
//
//  ---- 为什么这样切 ----
//    1) 没有内部 FIFO、没有长度寄存器 ⇒ 状态机浅，容易在仿真里穷举验证；
//    2) "每条命令一个 done" 让调用方能精确判断哪一步失败（写地址 NACK 还是写数据 NACK）；
//    3) 重复起始（repeated start）不需要新命令：在 `done` 之后**再发一次 start** 即可，
//       此时总线被"hold"住（SCL 保持低），波形就是标准的 Sr。
//
//  ---- 相位与时序 ----
//  每个数据位用 4 个四分之一相位（QTR）：
//      S_BIT_LOW(1Q) → S_BIT_HW(1Q，等 SCL 真正变高) → S_BIT_HIGH(1Q) → S_BIT_LB(1Q)
//  低电平共 2Q、高电平共 2Q ⇒ 占空比 50%，SCL 周期 = 4Q ⇒ SCL = CLK / (4*QTR)。
//  ⚠️ QTR 由参数算出来，**不是写死的常数**（换板/换频率只改参数）。
//
//  ---- 时钟延展（clock stretching）----
//  主机释放 SCL 后**不假设它立刻变高**，而是等 scl_i 读回 1 才继续；
//  等不到就按 TIMEOUT_CLK 放弃本次传输（timeout 脉冲 + 总线释放回空闲）。
//  这条不是"锦上添花"：MPR121 在做基线校准时会拉伸 SCL，不处理会把整个总线拖死。
//
//  ---- 电气 ----
//  scl/sda 是**开漏**输出（只拉低或高阻），高电平靠外部上拉（模块板载 4.7K）。
//  仿真里由 tb 用 `pullup` 提供；上板由 PMOD 侧模块的上拉提供。
// ============================================================
`default_nettype none

module i2c_master #(
    parameter integer CLK_HZ      = 50_000_000,  // 系统时钟
    parameter integer SCL_HZ      = 100_000,     // 目标 SCL（标准模式 100 kHz）
    parameter integer TIMEOUT_CLK = 50_000       // 等 SCL 变高的最长拍数（50MHz 下 = 1 ms）
)(
    input  wire        clk,
    input  wire        rst_n,

    // ---- 命令口（单周期脉冲；靠 busy/done 串行化）----
    input  wire        start,     // 发起 START（总线已 hold 时即重复 START）
    input  wire        wr,        // 发送 tx_byte（8 位 + 读 ACK）
    input  wire        rd,        // 接收一个字节
    input  wire        rd_ack,    // 与 rd 同拍采样：1=回 ACK（后面还要读），0=回 NACK（最后一个）
    input  wire        stop,      // 发起 STOP
    input  wire [7:0]  tx_byte,

    // ---- 状态 ----
    output reg  [7:0]  rx_byte,   // rd 命令收完后的字节（在 done 那一拍有效）
    output reg         busy,      // 1 = 正在执行一条命令
    output reg         done,      // 单周期脉冲：命令完成（无论成功与否）
    output reg         nack,      // 写字节收到 NACK（与 done 同拍有效；1 = 从机没应答）
    output reg         timeout,   // 单周期脉冲：SCL 被拉低超时，已放弃并回到空闲
    output reg  [31:0] op_count,  // 调试：累计完成的命令条数（上板可用 UART 打印）

    // ---- 开漏 pad ----
    output wire        scl,       // 1'b0 或 1'bz
    output wire        sda,       // 1'b0 或 1'bz
    output wire        scl_oe,    // 1 = 拉低
    output wire        sda_oe,    // 1 = 拉低
    input  wire        scl_i,     // 从 pad 读回（判断时钟延展）
    input  wire        sda_i      // 从 pad 读回（数据与 ACK）
);

    // ---------------- 相位节拍 ----------------
    // ⚠️ 一个位周期是 **5 个四分之一相位（QTR）**，不是 4：
    //      S_BIT_LOW(1Q，SCL低、摆数据) → S_BIT_HW(2Q，释放 SCL 并等它真的变高)
    //      → S_BIT_HIGH(1Q，保持高) → S_BIT_LB(1Q，拉低 SCL)
    //    "等它真的变高"必占 1Q（本拍释放、下一拍才读回高电平），所以高电平 3Q / 低电平 2Q。
    //    I2C 对占空比没有要求（只规定 tLOW/tHIGH 下限），60/40 完全合法；
    //    但**频率必须按 5Q 算**，否则 100 kHz 会跑成 80 kHz。
    //    （第一版按 4Q 算，实测 12500 ns = 50 拍 = 5Q，就是这么发现的。）
    localparam integer QTR_RAW = CLK_HZ / (5 * SCL_HZ);
    localparam integer QTR     = (QTR_RAW < 1) ? 1 : QTR_RAW;

    reg [15:0] qcnt;
    wire       q_tick = (qcnt == (QTR - 1));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) qcnt <= 16'd0;
        else        qcnt <= q_tick ? 16'd0 : (qcnt + 16'd1);
    end

    // ---------------- 开漏输出 ----------------
    reg scl_oe_r, sda_oe_r;
    assign scl_oe = scl_oe_r;
    assign sda_oe = sda_oe_r;
    assign scl    = scl_oe_r ? 1'b0 : 1'bz;
    assign sda    = sda_oe_r ? 1'b0 : 1'bz;

    // ---------------- 状态编码 ----------------
    localparam [4:0]
        S_IDLE      = 5'd0,   // 总线空闲（SCL/SDA 都释放）
        S_START_A   = 5'd1,   // 释放 SCL（重复起始时从这里把 SCL 拉高）
        S_START_B   = 5'd2,   // SCL 高时把 SDA 拉低 = START / Sr
        S_START_C   = 5'd3,   // 拉低 SCL
        S_BIT_LOW   = 5'd4,   // SCL 低：摆好本位
        S_BIT_HW    = 5'd5,   // 释放 SCL，等它真的变高（延展在这里等）
        S_BIT_HIGH  = 5'd6,   // SCL 高：保持
        S_BIT_LB    = 5'd7,   // 拉低 SCL，本位结束
        S_ACK_HW    = 5'd8,   // 释放 SDA，等 SCL 高，准备采 ACK
        S_ACK_HIGH  = 5'd9,   // 采 ACK
        S_ACK_LB    = 5'd10,  // 拉低 SCL
        S_RD_LOW    = 5'd11,  // SCL 低、释放 SDA
        S_RD_HW     = 5'd12,  // 等 SCL 高
        S_RD_HIGH   = 5'd13,  // 采样 SDA 并移位
        S_RD_LB     = 5'd14,  // 拉低 SCL
        S_MACK_LOW  = 5'd15,  // 主机回 ACK/NACK（SCL 低时摆好）
        S_MACK_HW   = 5'd16,
        S_MACK_HIGH = 5'd17,
        S_MACK_LB   = 5'd18,
        S_STOP_A    = 5'd19,  // SCL 低时拉低 SDA
        S_STOP_B    = 5'd20,  // 释放 SCL 等它变高
        S_STOP_C    = 5'd21,  // SCL 高时释放 SDA = STOP
        S_HOLD      = 5'd22;  // 一条字节命令结束后的"握住总线"状态（SCL 保持低）

    reg [4:0]  state;
    reg [7:0]  tx_shift;
    reg [2:0]  bit_cnt;
    reg        rd_ack_r;
    reg [15:0] wcnt;      // 等 SCL 变高的拍数（超时判定）

    // ---- 命令锁存 ----
    // ⚠️ 踩过的坑：FSM 只在 q_tick（每 QTR 拍一次）推进，而命令口是**单周期脉冲**。
    //    如果直接在 S_IDLE 里判 start/wr/rd/stop，有 (QTR-1)/QTR 的概率**整条命令被漏掉**，
    //    表现为"发了一条命令，done 永远不来"，而且很难往"采样率不匹配"上想。
    //    所以：命令**每一拍都看**，先锁进 cmd_*，再由 FSM 在下一个 q_tick 取走。
    localparam [1:0] CMD_START = 2'd1,
                     CMD_WR    = 2'd2,
                     CMD_RD    = 2'd3,
                     CMD_STOP  = 2'd0;
    reg        cmd_valid;
    reg [1:0]  cmd_kind;
    reg [7:0]  cmd_byte;
    reg        cmd_ackbit;

    // 正在等 SCL 变高的状态：这些状态里 wcnt 才累加
    wire waiting_scl = (state == S_BIT_HW) || (state == S_ACK_HW) ||
                       (state == S_RD_HW)  || (state == S_MACK_HW) ||
                       (state == S_STOP_B) || (state == S_START_B);

    // 总线可以接收新命令的状态
    wire ready = (state == S_IDLE) || (state == S_HOLD);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            scl_oe_r  <= 1'b0;
            sda_oe_r  <= 1'b0;
            busy      <= 1'b0;
            done      <= 1'b0;
            nack      <= 1'b0;
            timeout   <= 1'b0;
            rx_byte   <= 8'd0;
            tx_shift  <= 8'd0;
            bit_cnt   <= 3'd0;
            rd_ack_r  <= 1'b0;
            wcnt      <= 16'd0;
            op_count  <= 32'd0;
            cmd_valid <= 1'b0;
            cmd_kind  <= CMD_STOP;
            cmd_byte  <= 8'd0;
            cmd_ackbit<= 1'b0;
        end else begin
            // 脉冲输出默认清零（本拍内若被置位则以置位为准）
            done    <= 1'b0;
            timeout <= 1'b0;

            // 等 SCL 变高的看门狗（每拍累加；离开等待状态就清零）
            if (waiting_scl && (scl_i == 1'b0)) wcnt <= wcnt + 16'd1;
            else                                 wcnt <= 16'd0;

            // ---- 命令锁存：每拍都看，避免单周期脉冲被 q_tick 漏掉 ----
            if (!cmd_valid && !busy && ready) begin
                if (start) begin
                    cmd_valid  <= 1'b1;
                    cmd_kind   <= CMD_START;
                    cmd_byte   <= tx_byte;
                    cmd_ackbit <= rd_ack;
                end else if (wr) begin
                    cmd_valid  <= 1'b1;
                    cmd_kind   <= CMD_WR;
                    cmd_byte   <= tx_byte;
                    cmd_ackbit <= rd_ack;
                end else if (rd) begin
                    cmd_valid  <= 1'b1;
                    cmd_kind   <= CMD_RD;
                    cmd_byte   <= tx_byte;
                    cmd_ackbit <= rd_ack;
                end else if (stop) begin
                    cmd_valid  <= 1'b1;
                    cmd_kind   <= CMD_STOP;
                    cmd_byte   <= tx_byte;
                    cmd_ackbit <= rd_ack;
                end
            end

            if (q_tick) begin
                case (state)
                // ---------------- 空闲 / 握住总线：接收命令 ----------------
                S_IDLE, S_HOLD: begin
                    if (cmd_valid) begin
                        cmd_valid <= 1'b0;
                        busy      <= 1'b1;
                        case (cmd_kind)
                        CMD_START: state <= S_START_A;
                        CMD_WR: begin
                            tx_shift <= cmd_byte;
                            bit_cnt  <= 3'd0;
                            state    <= S_BIT_LOW;
                        end
                        CMD_RD: begin
                            rd_ack_r <= cmd_ackbit;
                            bit_cnt  <= 3'd0;
                            state    <= S_RD_LOW;
                        end
                        default: state <= S_STOP_A;
                        endcase
                    end
                end

                // ---------------- START / 重复 START ----------------
                S_START_A: begin
                    scl_oe_r <= 1'b0;      // 释放 SCL（IDLE 时本来就是高）
                    sda_oe_r <= 1'b0;      // 释放 SDA
                    state    <= S_START_B;
                end
                S_START_B: begin
                    // ⚠️ 必须等 SCL 真的变高才拉低 SDA：否则在"从机延展"期间
                    //    拉低 SDA 就只是数据跳变，不是起始条件（会丢帧）。
                    if (scl_i == 1'b1) begin
                        sda_oe_r <= 1'b1;      // SCL 高时 SDA 下降 = START / Sr
                        state    <= S_START_C;
                    end else if (wcnt >= TIMEOUT_CLK - 1) begin
                        timeout <= 1'b1; scl_oe_r <= 1'b0; sda_oe_r <= 1'b0;
                        busy <= 1'b0; state <= S_IDLE; done <= 1'b1;
                    end
                end
                S_START_C: begin
                    scl_oe_r <= 1'b1;      // 拉低 SCL，开始送位
                    done     <= 1'b1;
                    busy     <= 1'b0;
                    op_count <= op_count + 32'd1;
                    state    <= S_HOLD;
                end

                // ---------------- 发送一位 ----------------
                S_BIT_LOW: begin
                    scl_oe_r <= 1'b1;              // 确保 SCL 低
                    sda_oe_r <= ~tx_shift[7];      // 0 -> 拉低；1 -> 释放
                    state    <= S_BIT_HW;
                end
                S_BIT_HW: begin
                    scl_oe_r <= 1'b0;              // 释放 SCL
                    if (scl_i == 1'b1)       state <= S_BIT_HIGH;
                    else if (wcnt >= TIMEOUT_CLK - 1) begin
                        timeout  <= 1'b1;
                        scl_oe_r <= 1'b0;
                        sda_oe_r <= 1'b0;
                        busy     <= 1'b0;
                        state    <= S_IDLE;
                        done     <= 1'b1;   // 超时也回 done，调用方不会挂死
                    end
                end
                S_BIT_HIGH: begin
                    state <= S_BIT_LB;
                end
                S_BIT_LB: begin
                    scl_oe_r <= 1'b1;              // 拉低 SCL
                    tx_shift <= {tx_shift[6:0], 1'b0};
                    if (bit_cnt == 3'd7) begin
                        bit_cnt <= 3'd0;
                        state   <= S_ACK_HW;
                    end else begin
                        bit_cnt <= bit_cnt + 3'd1;
                        state   <= S_BIT_LOW;
                    end
                end

                // ---------------- 收从机的 ACK ----------------
                S_ACK_HW: begin
                    sda_oe_r <= 1'b0;              // 释放 SDA 让从机拉
                    scl_oe_r <= 1'b0;
                    if (scl_i == 1'b1) state <= S_ACK_HIGH;
                    else if (wcnt >= TIMEOUT_CLK - 1) begin
                        timeout <= 1'b1; scl_oe_r <= 1'b0; sda_oe_r <= 1'b0;
                        busy <= 1'b0; state <= S_IDLE;
                        done <= 1'b1;   // 超时也回 done，调用方不会挂死
                    end
                end
                S_ACK_HIGH: begin
                    nack <= sda_i;                 // 1 = NACK（从机没应答）
                    state <= S_ACK_LB;
                end
                S_ACK_LB: begin
                    scl_oe_r <= 1'b1;
                    done     <= 1'b1;
                    busy     <= 1'b0;
                    op_count <= op_count + 32'd1;
                    state    <= S_HOLD;
                end

                // ---------------- 接收一位 ----------------
                S_RD_LOW: begin
                    sda_oe_r <= 1'b0;              // 释放 SDA 给从机驱动
                    scl_oe_r <= 1'b1;              // SCL 低
                    state    <= S_RD_HW;
                end
                S_RD_HW: begin
                    scl_oe_r <= 1'b0;
                    if (scl_i == 1'b1) state <= S_RD_HIGH;
                    else if (wcnt >= TIMEOUT_CLK - 1) begin
                        timeout <= 1'b1; scl_oe_r <= 1'b0; sda_oe_r <= 1'b0;
                        busy <= 1'b0; state <= S_IDLE;
                        done <= 1'b1;   // 超时也回 done，调用方不会挂死
                    end
                end
                S_RD_HIGH: begin
                    rx_byte <= {rx_byte[6:0], sda_i};   // 移位直接落在输出上
                    state   <= S_RD_LB;
                end
                S_RD_LB: begin
                    scl_oe_r <= 1'b1;
                    if (bit_cnt == 3'd7) begin
                        bit_cnt <= 3'd0;
                        state   <= S_MACK_LOW;
                    end else begin
                        bit_cnt <= bit_cnt + 3'd1;
                        state   <= S_RD_LOW;
                    end
                end

                // ---------------- 主机回 ACK / NACK ----------------
                S_MACK_LOW: begin
                    sda_oe_r <= rd_ack_r;          // ACK = 拉低(1)；NACK = 释放(0)
                    scl_oe_r <= 1'b1;
                    state    <= S_MACK_HW;
                end
                S_MACK_HW: begin
                    scl_oe_r <= 1'b0;
                    if (scl_i == 1'b1) state <= S_MACK_HIGH;
                    else if (wcnt >= TIMEOUT_CLK - 1) begin
                        timeout <= 1'b1; scl_oe_r <= 1'b0; sda_oe_r <= 1'b0;
                        busy <= 1'b0; state <= S_IDLE;
                        done <= 1'b1;   // 超时也回 done，调用方不会挂死
                    end
                end
                S_MACK_HIGH: begin
                    state <= S_MACK_LB;
                end
                S_MACK_LB: begin
                    scl_oe_r <= 1'b1;
                    sda_oe_r <= 1'b0;
                    done     <= 1'b1;
                    busy     <= 1'b0;
                    op_count <= op_count + 32'd1;
                    state    <= S_HOLD;
                end

                // ---------------- STOP ----------------
                S_STOP_A: begin
                    scl_oe_r <= 1'b1;              // 先确保 SCL 低
                    sda_oe_r <= 1'b1;              // SDA 低
                    state    <= S_STOP_B;
                end
                S_STOP_B: begin
                    scl_oe_r <= 1'b0;              // 释放 SCL
                    if (scl_i == 1'b1) state <= S_STOP_C;
                    else if (wcnt >= TIMEOUT_CLK - 1) begin
                        timeout <= 1'b1; scl_oe_r <= 1'b0; sda_oe_r <= 1'b0;
                        busy <= 1'b0; state <= S_IDLE;
                        done <= 1'b1;   // 超时也回 done，调用方不会挂死
                    end
                end
                S_STOP_C: begin
                    sda_oe_r <= 1'b0;              // SCL 高时 SDA 上升 = STOP
                    done     <= 1'b1;
                    busy     <= 1'b0;
                    op_count <= op_count + 32'd1;
                    state    <= S_IDLE;
                end

                default: state <= S_IDLE;
                endcase
            end
        end
    end

// ============================================================
//  用法（写寄存器 → 读回来）
// ------------------------------------------------------------
//   1) start          等 done
//   2) wr (addr<<1|0) 等 done；检查 nack（1 = 器件没应答 ⇒ 接线/供电/地址错）
//   3) wr (reg_addr)  等 done
//   4) start          等 done          <-- 重复起始，不要 stop
//   5) wr (addr<<1|1) 等 done
//   6) rd (rd_ack=0)  等 done；rx_byte 就是数据
//   7) stop           等 done
//   ⇒ 想连续读多个字节：第 6 步 rd_ack=1 保持，最后一个用 rd_ack=0。
// ============================================================
endmodule

`default_nettype wire