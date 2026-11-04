// ============================================================
//  touch_keys.v —— MPR121 12 键触摸 → key_raw[11:0] + 按键事件
//
//  ---- 它做的事 ----
//    周期读 MPR121 的触摸状态两个字节（0x00 / 0x01）→ 拼成 12 位位图
//    → 去抖 → 输出 `key_raw[11:0]`（**高有效，1 = 按下**）+ press/release 单周期脉冲。
//
//  ---- 位序（关键，弄反了就是"按键全是乱的"）----
//    0x00 : bit0..bit7  = ELE0..ELE7
//    0x01 : bit0..bit3  = ELE8..ELE11，bit4 = ELEPROX，bit7 = OVCF(过流)
//    ⇒ raw = { rd1[3:0], rd0 }   ⇒ raw[0] = ELE0 … raw[11] = ELE11
//    再按 P2 的键位约定：**key0 = C，key11 = B（半音上行）**
//    ⇒ 与 `keyboard.v` 的 `key_raw[11:0]` 语义**完全一致，零适配**。
//      （`keyboard.v` 的极性约定是"高有效"，而板上的机械键是低有效上拉 ——
//       那一层的取反在**顶层**做，不在本模块做。这里保持"高 = 按下"。）
//
//  ---- 为什么去抖在 RTL 做 ----
//    MPR121 自己也有硬件去抖（0x5B 的 DT/DR），但 cfg 里把它关成 0：
//    让"去抖时长"只有一处可调，且逻辑上能在仿真里被验证。
//    判据：连续 `DEB_N` 次轮询结果一致才更新。
//
//  ---- 轮询节奏 ----
//    50 MHz 下 `POLL_DIV = 50000` ⇒ 每 1 ms 读一次（≈1 kHz），
//    对人手按键绰绰有余，也不会把 I2C 总线占满（一次 2 字节读约 250 µs @100 kHz）。
//
//  ⚠️ 与 mpr121_cfg 共用同一条 I2C：本模块**只在 init_done 之后**才开始轮询，
//     顶层按 init_done 把 go/reg_addr/... 二选一 mux 过来即可，不会打架。
// ============================================================
`default_nettype none

module touch_keys #(
    parameter [6:0]   DEV      = 7'h5A,
    parameter integer POLL_DIV = 50_000,   // 两次轮询之间的拍数（50MHz ⇒ 1 ms）
    parameter integer DEB_N    = 3,        // 连续几次一致才认（3 次 ≈ 3 ms）
    parameter integer STARTUP  = 1_000     // init_done 之后再等多少拍才开始轮询
)(
    input  wire        clk,
    input  wire        rst_n,

    // 配置完成前不轮询
    input  wire        init_done,

    // ---- 到 i2c_xfer ----
    output reg         go,
    output reg         op_read,
    output reg  [7:0]  reg_addr,
    output reg  [7:0]  wr_data,
    output reg  [1:0]  rd_len,
    input  wire        xf_done,
    input  wire        xf_err,
    input  wire [7:0]  rd0,
    input  wire [7:0]  rd1,

    // ---- 输出 ----
    output reg  [11:0] key_raw,      // 去抖后的位图（高有效）
    output reg  [11:0] key_press,    // 单周期脉冲
    output reg  [11:0] key_release,  // 单周期脉冲
    output reg         ovcf,         // sticky：芯片报过流（接线/电极短路时会出现）
    output reg         i2c_err,      // sticky：轮询过程中出现过 NACK/超时
    output reg         poll_stb      // 每次轮询完成打一拍（给上层做节拍用，可悬空）
);

    localparam [2:0]
        T_IDLE  = 3'd0,   // 等 init_done + STARTUP
        T_WAIT  = 3'd1,   // 等轮询间隔
        T_GO    = 3'd2,   // 发读命令
        T_RUN   = 3'd3;   // 等 xfer 完成

    reg [2:0]  state;
    reg [31:0] wait_cnt;
    reg [7:0]  deb_cnt;      // 连续看到同一个候选值的次数（含本次）
    reg [11:0] cand;         // 候选值
    reg        first_poll;   // 第一次轮询前用 STARTUP 等待，之后用 POLL_DIV

    localparam [7:0] DEB_N8 = DEB_N;   // 参数是 integer，取低 8 位方便比较

    wire [11:0] sample = {rd1[3:0], rd0};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= T_IDLE;
            wait_cnt   <= 32'd0;
            deb_cnt    <= 8'd0;
            cand       <= 12'd0;
            first_poll <= 1'b1;
            go         <= 1'b0;
            op_read    <= 1'b0;
            reg_addr   <= 8'd0;
            wr_data    <= 8'd0;
            rd_len     <= 2'd0;
            key_raw    <= 12'd0;
            key_press  <= 12'd0;
            key_release<= 12'd0;
            ovcf       <= 1'b0;
            i2c_err    <= 1'b0;
            poll_stb   <= 1'b0;
        end else begin
            // 单拍脉冲默认清零
            go         <= 1'b0;
            key_press  <= 12'd0;
            key_release<= 12'd0;
            poll_stb   <= 1'b0;

            case (state)
            T_IDLE: begin
                if (init_done) begin
                    wait_cnt <= 32'd0;
                    state    <= T_WAIT;
                end
            end

            T_WAIT: begin
                // 第一次轮询前多等一会儿（让 MPR121 的基线先收敛），之后按 POLL_DIV
                if (wait_cnt >= (first_poll ? STARTUP[31:0] : POLL_DIV[31:0])) begin
                    wait_cnt <= 32'd0;
                    state    <= T_GO;
                end else wait_cnt <= wait_cnt + 32'd1;
            end

            T_GO: begin
                reg_addr <= 8'h00;       // 触摸状态起始寄存器
                wr_data  <= 8'h00;
                op_read  <= 1'b1;        // 读
                rd_len   <= 2'd2;        // 两个字节：ELE0-7 + ELE8-11
                go       <= 1'b1;
                state    <= T_RUN;
            end

            T_RUN: begin
                if (xf_done) begin
                    if (xf_err) begin
                        i2c_err <= 1'b1;      // 记下来，但继续轮询（偶发 NACK 不该让键盘死掉）
                    end else begin
                        poll_stb <= 1'b1;
                        first_poll <= 1'b0;
                        if (rd1[7]) ovcf <= 1'b1;   // OVCF = 过流标志，置位不清

                        // ---- 去抖：连续 DEB_N 次采样一致才提交 ----
                        //  deb_cnt 记"连续看到同一个值几次（含本次）"。
                        //  ⚠️ 第一版写成"先比后加、再用旧值比阈值"，导致 DEB_N=2 实际要
                        //     3 次才提交（差一拍）。tb 里"抖动只持续 1 次采样不产生事件"
                        //     这条能过，但"两次采样就该响应"这条会红。
                        if (sample == cand) begin
                            if (deb_cnt != 8'hFF) deb_cnt <= deb_cnt + 8'd1;
                        end else begin
                            cand    <= sample;
                            deb_cnt <= 8'd1;          // 本次算第 1 次
                        end

                        // 已连续够 DEB_N 次（本次已计入）且与当前输出不同 ⇒ 提交
                        if ((sample == cand) && ((deb_cnt + 8'd1) >= DEB_N8) &&
                            (sample != key_raw)) begin
                            key_press   <= sample & ~key_raw;
                            key_release <= ~sample & key_raw;
                            key_raw     <= sample;
                            deb_cnt     <= 8'd0;
                        end
                    end
                    state <= T_WAIT;
                end
            end

            default: state <= T_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire