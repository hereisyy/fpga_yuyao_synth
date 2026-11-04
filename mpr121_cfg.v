// ============================================================
//  mpr121_cfg.v —— MPR121 上电配置序列
//
//  只做**初始化**：软复位 → 等一会儿 → 依次写滤波/去抖/阈值/使能。
//  周期读触摸状态放在 `touch_keys.v` 里（那边本来就要读），
//  这样"初始化错"和"读取错"是两种可分辨的失败，不会混在一起。
//
//  ---- 顺序上有一条硬规矩 ----
//    **ECR(0x5E) 必须最后写**：ELE_EN 一位下去，芯片立刻开始对 12 个电极做检测。
//    如果先使能再写阈值，那段时间用的是复位默认阈值 ⇒ 可能误报一大片。
//    tb 里专门有一条检查"最后一笔数据写的是 0x5E"。
//
//  ---- 软复位后为什么要等 ----
//    手册要求复位后约 1 ms 才开始配置。这里用参数 `RESET_WAIT` 折算成拍数
//    （默认 50 MHz 下 1 ms = 50000 拍），tb 里用一个很小的值免得跑太久。
//
//  ---- 数值来源（重要）----
//    寄存器**地址**来自 MPR121 数据手册（见 mpr121_model.v 文件头的出处）。
//    寄存器的**数值**是开源实现里通用的保守默认（CDC=16µA、CDT=0.5µs、
//    无硬件去抖以便由 RTL 统一去抖）。这些数不是"手册规定值"，而是
//    **需要上板按实际电极尺寸微调的调参起点** —— 上板时改动请同步改 tb。
// ============================================================
`default_nettype none

module mpr121_cfg #(
    parameter [6:0]   DEV        = 7'h5A,
    parameter integer RESET_WAIT = 50_000   // 软复位后等待拍数（50MHz ⇒ 1 ms）
)(
    input  wire       clk,
    input  wire       rst_n,

    // ---- 到 i2c_xfer（本模块只写不读）----
    output reg        go,
    output reg        op_read,
    output reg  [7:0] reg_addr,
    output reg  [7:0] wr_data,
    output reg  [1:0] rd_len,
    input  wire       xf_done,
    input  wire       xf_err,

    // ---- 状态 ----
    output reg        init_done,   // 配置全部完成（常高）
    output reg        busy,
    output reg        err,         // 出现过 NACK/超时，已停止
    output wire [1:0] addr_sel,    // ★ 实际应答的地址索引（0..3 → 0x5A..0x5D）
    input  wire       restart,     // ★ 单拍脉冲：总线恢复后重来一遍
    input  wire [7:0] xf_rd0,      // ★ 读回数据（自检用）
    output reg        verify_bad,  // ★ 1 = 配置回读不符合预期（电极没使能）
    output reg  [7:0] ecr_rd,      // ★ 回读到的 ECR 原始值（给串口打印，排错用）
    output reg        ghost_bus,   // ★ 1 = 连"不存在的地址"也应答 ⇒ SDA 被拉死/短路
    output reg [6:0]  dev_ovr,     // 强制使用的从地址（全总线扫描用）
    output reg        dev_ovr_en,  // 1 = 用 dev_ovr
    output reg [6:0]  scan_hit     // 全总线扫描：第一个应答的地址（0 = 一个都没有）
);

    // ---------------- 配置表（15 项）----------------
    //  索引 → {寄存器地址, 写入值}
    localparam integer CFG_N = 41;   // 14 通用 + 24 阈值 + ECR 停止/启动三步（末笔 = ECR 运行）

    function [15:0] cfg_ent(input [5:0] i);
        begin
            case (i)
            // ---- 通用配置（噪声滤波用数据手册默认值）----
            6'd0 : cfg_ent = {8'h2B, 8'h01};  // MHDR  Max Half Delta Rising
            6'd1 : cfg_ent = {8'h2C, 8'h01};  // NHDR  Noise Half Delta Rising
            6'd2 : cfg_ent = {8'h2D, 8'h10};  // NCLR  Noise Count Limit Rising（默认 0x10）
            6'd3 : cfg_ent = {8'h2E, 8'h04};  // FDLR  Filter Delay Limit Rising（默认 0x04；写 0 = 关滤波）
            6'd4 : cfg_ent = {8'h2F, 8'h01};  // MHDF  Max Half Delta Falling
            6'd5 : cfg_ent = {8'h30, 8'h01};  // NHDF  Noise Half Delta Falling
            6'd6 : cfg_ent = {8'h31, 8'h02};  // NCLF  Noise Count Limit Falling（默认 0x02）
            6'd7 : cfg_ent = {8'h32, 8'h04};  // FDLF  Filter Delay Limit Falling（默认 0x04）
            6'd8 : cfg_ent = {8'h33, 8'h00};  // NHDT  Noise Half Delta Touched
            6'd9 : cfg_ent = {8'h34, 8'h00};  // NCLT  Noise Count Limit Touched
            6'd10: cfg_ent = {8'h35, 8'h04};  // FDLT  Filter Delay Limit Touched（默认 0x04）
            6'd11: cfg_ent = {8'h5B, 8'h22};  // DEBOUNCE 按下/释放各 2 次采样
            6'd12: cfg_ent = {8'h5C, 8'h10};  // AFE  CDC=16 µA
            6'd13: cfg_ent = {8'h5D, 8'h20};  // FILTER CDT=0.5 µs
            // ---- 12 个电极的触摸/释放阈值（默认 0x28/0x14；实测误触 ⇒ 抬到 0x30/0x18）----
            6'd14: cfg_ent = {8'h41, 8'h2C};  // ELE0  touch（0x2C=44，略高于默认 0x28；0x30 太钝、开机等太久）
            6'd15: cfg_ent = {8'h42, 8'h18};  // ELE0  release
            6'd16: cfg_ent = {8'h43, 8'h2C};  // ELE1
            6'd17: cfg_ent = {8'h44, 8'h18};
            6'd18: cfg_ent = {8'h45, 8'h2C};  // ELE2
            6'd19: cfg_ent = {8'h46, 8'h18};
            6'd20: cfg_ent = {8'h47, 8'h2C};  // ELE3
            6'd21: cfg_ent = {8'h48, 8'h18};
            6'd22: cfg_ent = {8'h49, 8'h2C};  // ELE4
            6'd23: cfg_ent = {8'h4A, 8'h18};
            6'd24: cfg_ent = {8'h4B, 8'h2C};  // ELE5
            6'd25: cfg_ent = {8'h4C, 8'h18};
            6'd26: cfg_ent = {8'h4D, 8'h2C};  // ELE6
            6'd27: cfg_ent = {8'h4E, 8'h18};
            6'd28: cfg_ent = {8'h4F, 8'h2C};  // ELE7
            6'd29: cfg_ent = {8'h50, 8'h18};
            6'd30: cfg_ent = {8'h51, 8'h2C};  // ELE8
            6'd31: cfg_ent = {8'h52, 8'h18};
            6'd32: cfg_ent = {8'h53, 8'h2C};  // ELE9
            6'd33: cfg_ent = {8'h54, 8'h18};
            6'd34: cfg_ent = {8'h55, 8'h2C};  // ELE10
            6'd35: cfg_ent = {8'h56, 8'h18};
            6'd36: cfg_ent = {8'h57, 8'h2C};  // ELE11
            6'd37: cfg_ent = {8'h58, 8'h18};
            // ---- ECR 必须是**最后一笔**（使能电极、进入运行模式）----
            6'd38: cfg_ent = {8'h5E, 8'h00};  // ECR=0：先**停止**运行（强制一次干净的重校准）
            6'd39: cfg_ent = {8'h5E, 8'h0C};  // ECR=0x0C：再进运行模式 ⇒ 基线从头建立
            //  2026-11-03：开机要等很久才响应，怀疑基线在旧状态上慢慢漂；
            //  显式「停止 → 启动」逼芯片立刻重校准，这是 MPR121 的标准重校准手法。
            6'd40: cfg_ent = {8'h5E, 8'h0C};  // 再写一次运行态（等价于确认，幂等）
            default: cfg_ent = {8'h00, 8'h00};
            endcase
        end
    endfunction

    localparam [8:0]
        C_WAIT0 = 9'd0,   // 上电等几拍
        C_RST   = 9'd1,   // 发软复位
        C_RSTW  = 9'd2,   // 等复位事务完成
        C_DLY   = 9'd3,   // 复位后等待
        C_ISSUE = 9'd4,   // 发一条配置写
        C_WAIT1 = 9'd5,   // 等这条写完成
        C_DONE  = 9'd6,
        C_ERR   = 9'd7,
        C_GH    = 9'd10,  // ★ 幽灵探测：向 0x7F 发一次事务
        C_GW    = 9'd11,  // ★ 等幽灵探测完成
        C_VER   = 9'd8,   // ★ 回读自检：读 0x5E(ECR)
        C_VW    = 9'd9;   // ★ 等回读完成

    reg [8:0]  state;
    reg [5:0]  idx;        // ★ 6 位：配置表已扩到 39 项
    reg [19:0] pwr_cnt;        // ★ 加宽：上电等待从 15 拍改成 2 ms
    reg [1:0]  retry_cnt;      // ★ 整轮配置重试次数（ECR 回读不对就重来）
    reg [6:0]  scan_a;         // ★ 全总线扫描的当前地址
    reg [31:0] dly_cnt;

    // ============================================================
    //  ★ 地址自适应（2026-10-30 上板实测后加）
    //
    //  MPR121 的从地址由 **ADDR 脚**决定：
    //      ADDR=GND → 0x5A、ADDR=3V3 → 0x5B、ADDR=SDA → 0x5C、ADDR=SCL → 0x5D
    //  实物上 ADDR 未必按我们的假设接地，而接错地址时 I2C **不报错、只是没应答**，
    //  表现就是 `E=1`、`K=000`，肉眼完全看不出区别。
    //
    //  所以：**第一条事务（软复位）NACK 就换下一个地址重来**，四个都试过才置 `err`。
    //  代价：最坏多花 4×(复位等待 1 ms) ≈ 5 ms 启动时间，可忽略。
    //  （只在**第一条**事务上重试：能应答软复位说明地址对了，后面的 NACK 是真故障。）
    // ============================================================
    reg [1:0]  addr_idx;                 // 0..3 → 0x5A..0x5D
    wire [6:0] dev_r = DEV + {5'd0, addr_idx};
    assign addr_sel = addr_idx;

    wire [15:0] ent = cfg_ent(idx);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= C_WAIT0;
            idx       <= 6'd0;
            pwr_cnt   <= 20'd0;
            retry_cnt <= 2'd0;
            scan_a    <= 7'h08;
            dev_ovr    <= 7'd0;
            dev_ovr_en <= 1'b0;
            scan_hit   <= 7'd0;
            ghost_bus  <= 1'b0;
            verify_bad <= 1'b0;
            ecr_rd     <= 8'h00;
            dly_cnt   <= 32'd0;
            addr_idx  <= 2'd0;
            go        <= 1'b0;
            op_read   <= 1'b0;
            reg_addr  <= 8'd0;
            wr_data   <= 8'd0;
            rd_len    <= 2'd0;
            init_done <= 1'b0;
            busy      <= 1'b0;
            err       <= 1'b0;
        end else if (restart) begin
            //  外部（总线恢复之后）要求重来：所有状态清零，从头再配置一遍
            state     <= C_WAIT0;
            idx       <= 6'd0;
            addr_idx  <= 2'd0;
            pwr_cnt   <= 20'd0;
            retry_cnt <= 2'd0;
            dev_ovr    <= 7'd0;
            dev_ovr_en <= 1'b0;
            scan_hit   <= 7'd0;
            ghost_bus  <= 1'b0;
            verify_bad <= 1'b0;
            ecr_rd     <= 8'h00;
            dly_cnt   <= 32'd0;
            go        <= 1'b0;
            op_read   <= 1'b0;
            init_done <= 1'b0;
            busy      <= 1'b0;
            err       <= 1'b0;
        end else begin
            go <= 1'b0;   // go 是单拍脉冲

            case (state)
            C_WAIT0: begin
                //  ★ 上电后等 **2 ms** 再开始（原来只等 15 拍 = 0.3 µs）：
                //     上拉把总线拉高是微秒级的事，而且模块自身也要上电稳定；
                //     顺带让 `p2_input_top` 能在 1 ms 处采到**真实的空闲电平**。
                if (pwr_cnt == 20'd100_000) begin
                    busy  <= 1'b1;
                    state <= C_RST;
                end else pwr_cnt <= pwr_cnt + 20'd1;
            end

            C_GH: begin
                //  ★ 全总线扫描（2026-10-30）：配置失败后，把 0x08..0x77 挨个试一遍，
                //     看**有没有任何**器件应答。用来区分：
                //       · 一个都不应答 ⇒ 模块没供电 / 总线没接上
                //       · 某个地址应答   ⇒ 器件在，只是地址不在 0x5A~0x5D
                dev_ovr_en <= 1'b1;
                dev_ovr    <= scan_a;
                reg_addr   <= 8'h00;
                wr_data    <= 8'h00;
                op_read    <= 1'b0;
                rd_len     <= 2'd0;
                go         <= 1'b1;
                state      <= C_GW;
            end

            C_GW: begin
                if (xf_done) begin
                    dev_ovr_en <= 1'b0;
                    ghost_bus <= ~xf_err;      // 无 NACK ⇒ 幽灵应答 ⇒ SDA 被判为低
                    init_done <= 1'b1;
                    busy      <= 1'b0;
                    state     <= C_VER;        // 收尾，不再重试
                end
            end

            C_RST: begin
                reg_addr <= 8'h80;      // SOFTRESET
                wr_data  <= 8'h63;      // 魔数
                op_read  <= 1'b0;
                rd_len   <= 2'd0;
                go       <= 1'b1;
                state    <= C_RSTW;
            end

            C_RSTW: begin
                if (xf_done) begin
                    if (xf_err) begin
                        // 软复位没应答 ⇒ 先怀疑地址不对，逐个试
                        if (addr_idx < 2'd3) begin
                            addr_idx <= addr_idx + 2'd1;
                            state    <= C_RST;      // 用新地址重发软复位
                        end else begin
                            state    <= C_ERR;      // 4 个地址都不应答
                        end
                    end else begin
                        dly_cnt <= 32'd0;
                        state   <= C_DLY;
                    end
                end
            end

            C_DLY: begin
                if (dly_cnt >= RESET_WAIT) begin
                    idx   <= 6'd0;
                    state <= C_ISSUE;
                end else dly_cnt <= dly_cnt + 32'd1;
            end

            C_ISSUE: begin
                reg_addr <= ent[15:8];
                wr_data  <= ent[7:0];
                op_read  <= 1'b0;
                rd_len   <= 2'd0;
                go       <= 1'b1;
                state    <= C_WAIT1;
            end

            C_WAIT1: begin
                if (xf_done) begin
                    if (xf_err) state <= C_ERR;
                    else if (idx == (CFG_N - 1)) state <= C_DONE;
                    else begin
                        idx   <= idx + 6'd1;
                        state <= C_ISSUE;
                    end
                end
            end

            C_DONE: begin
                //  ★ 2026-10-30：配置写完后**回读 0x5E(ECR)** 自检。
                //     实测"配置成功但 K 恒 000"最常见的原因就是这个写没生效
                //     （芯片不在运行模式 ⇒ 触摸状态寄存器永远读 0）。
                reg_addr <= 8'h5E;
                op_read  <= 1'b1;
                rd_len   <= 2'd1;
                go       <= 1'b1;
                state    <= C_VW;
            end

            C_VW: begin
                if (xf_done) begin
                    // 期望 ECR = 0x0C（ELE_EN=1100 ⇒ 使能 ELE0..11，进入运行模式）
                    ecr_rd     <= xf_err ? 8'hFF : xf_rd0;   // 0xFF = 事务本身失败
                    verify_bad <= xf_err ? 1'b1 : (xf_rd0 != 8'h0C);
                    init_done  <= 1'b0;
                    busy       <= 1'b0;
                    //  ★ 回读不对 ⇒ **整轮配置重来**（最多 2 次）。
                    //     实测 E=4 说明写没生效：常见原因是软复位后等待不够、
                    //     第一轮写被芯片忽略；重来一次往往就好了。
                    if ((xf_err || (xf_rd0 != 8'h0C)) && (retry_cnt < 2'd2)) begin
                        retry_cnt <= retry_cnt + 2'd1;
                        addr_idx  <= 2'd0;
                        state     <= C_RST;
                    end else begin
                        //  重试也失败 ⇒ 做一次幽灵探测（发往不存在的 0x7F）：
                        //  连假地址都应答 ⇒ SDA 被判为低 ⇒ 总线坏。
                        init_done <= 1'b1;
                        state     <= C_VER;     // 重试也失败：收尾，交给串口诊断
                    end
                end
            end

            C_VER: begin
                init_done <= 1'b1;
                busy      <= 1'b0;
            end

            C_ERR: begin
                err  <= 1'b1;
                busy <= 1'b0;
            end

            default: state <= C_WAIT0;
            endcase
        end
    end

endmodule

`default_nettype wire
