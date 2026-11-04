`timescale 1ns/1ps
// ============================================================
//  mixer_tree.v —— 混音加法树（P0-6）：把 32 路声音叠起来
//
//  为什么要"树"、为什么要逐级打拍：
//  32 个 16 位有符号数**一次性相加**，组合路径太长，100 MHz 下必挂时序。
//  正确做法是每拍只做一次加法：
//
//      T1：32 → 16   （两两相加）
//      T2：16 →  8
//      T3： 8 →  4
//      T4： 4 →  2
//      T5： 2 →  1    ← 和
//      T6：缩放 + 饱和 + 输出寄存 → mix_out / mix_valid（同一拍）
//
//  ⚠️ **中间量全部用 32 位有符号**（不是"够用就行"的窄位宽）：
//     Verilog 的加法**结果宽度 = 操作数宽度**（不含进位！），
//     两个 17 位数相加只给 17 位结果 → 静默丢掉最高位 → 溢出回绕。
//     要逐级算准该多宽很费劲，而 32 位能装 ±2^31、
//     而这里的最大和只有 ±2^20，**绝对安全**。31 个加法器多花不到 200 个
//     LUT，拿这点资源换"绝不会算错"，非常值。
//
//  ⚠️ 为什么要饱和：多个声部同时响，和会超出 16 位量程。
//     溢出会**回绕**：32767 + 1 → -32768，也就是最大声突然变成最大负声，
//     听起来是刺耳的"啪"（比削顶难听得多）。
//     本项目用**硬饱和**：超了就卡在边界（简单、好验证）。
//
//  接口契约：
//   · `sample_tick` 那一拍，32 路 sample_in 必须**稳定**（上游寄存输出）
//   · LATENCY = 6 拍（5 级加法 + 1 级输出寄存），**恒定**
//   · mix_out 与 mix_valid **同一拍**有效（所以 valid 是 assign 出去的）
//   · 输出给 i2s_tx：OUT_W=16、SCALE_SHIFT=0 时可直接接 data_l/data_r
//
//  验收：无溢出、逐级流水（`tb_mixer.v`：全部输入给满量程 → 卡在边界不回绕）
//  依赖：P0-5 `vca.v`
// ============================================================

module mixer_tree #(
    // 声部数（本实现写死 32 路；单声部场景就把其余 31 路接 0）
    parameter integer NV          = 32,
    // 每路位宽（有符号）
    parameter integer SW          = 16,
    // 输出位宽（有符号）
    parameter integer OUT_W       = 16,
    // 输出前的整体右移（0 = 不缩放；声部全开时可取 1~2 做粗归一化）
    parameter integer SCALE_SHIFT = 0,
    // 1 = 硬饱和（上板用），0 = 直接截断（只用于调试对比）
    parameter integer SAT         = 1,
    // 自 sample_tick 起、到 mix_valid 的拍数（**必须恒定**）
    parameter integer LATENCY     = 6
)(
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    sample_tick,

    // 32 路输入（扁平总线：声部 k 在 [k*SW +: SW]）
    input  wire signed [NV*SW-1:0] sample_in,

    output reg  signed [OUT_W-1:0] mix_out,
    output wire                    mix_valid,
    // 调试用：这一帧发生过饱和（接 LED 或 ILA 一眼看出"削顶了"）
    output reg                     sat_flag
);

    // ============================================================
    //  1) 输入铺开成 32 个 32 位有符号量
    //  ⚠️ part-select 是无符号切片，必须 $signed() 才是负数
    // ============================================================
    wire signed [31:0] x0 [0:NV-1];

    genvar i;
    generate
        for (i = 0; i < NV; i = i + 1) begin : g_expand
            assign x0[i] = $signed(sample_in[i*SW +: SW]);
        end
    endgenerate

    // ============================================================
    //  2) 五级两两相加，每级打一拍
    // ============================================================
    reg signed [31:0] l1 [0:15];    // 32 → 16
    reg signed [31:0] l2 [0:7];     // 16 →  8
    reg signed [31:0] l3 [0:3];     //  8 →  4
    reg signed [31:0] l4 [0:1];     //  4 →  2
    reg signed [31:0] ssum;         //  2 →  1

    integer k;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) for (k = 0; k < 16; k = k + 1) l1[k] <= 32'sd0;
        else        for (k = 0; k < 16; k = k + 1) l1[k] <= x0[2*k] + x0[2*k+1];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) for (k = 0; k < 8; k = k + 1) l2[k] <= 32'sd0;
        else        for (k = 0; k < 8; k = k + 1) l2[k] <= l1[2*k] + l1[2*k+1];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) for (k = 0; k < 4; k = k + 1) l3[k] <= 32'sd0;
        else        for (k = 0; k < 4; k = k + 1) l3[k] <= l2[2*k] + l2[2*k+1];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) for (k = 0; k < 2; k = k + 1) l4[k] <= 32'sd0;
        else        for (k = 0; k < 2; k = k + 1) l4[k] <= l3[2*k] + l3[2*k+1];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) ssum <= 32'sd0;
        else        ssum <= l4[0] + l4[1];
    end

    // ============================================================
    //  3) 缩放 + 饱和 + 输出寄存
    // ============================================================
    wire signed [31:0] sc = ssum >>> SCALE_SHIFT;

    localparam signed [OUT_W-1:0] MAXV = (1 <<< (OUT_W-1)) - 1;   //  32767
    localparam signed [OUT_W-1:0] MINV = -(1 <<< (OUT_W-1));      // -32768

    wire sat_hi = (sc > MAXV);
    wire sat_lo = (sc < MINV);
    wire signed [OUT_W-1:0] satv = sat_hi ? MAXV :
                                  sat_lo ? MINV : sc[OUT_W-1:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mix_out  <= {OUT_W{1'b0}};
            sat_flag <= 1'b0;
        end else begin
            mix_out  <= SAT ? satv : sc[OUT_W-1:0];
            sat_flag <= sat_hi | sat_lo;
        end
    end

    // ============================================================
    //  4) mix_valid：用移位链把 sample_tick 推 LATENCY 拍
    //
    //  ⚠️ 这里必须是 **assign**，不能再用一级寄存器：
    //     数据在 T6 出现在 mix_out 上，valid_sr[LATENCY-1] 也在 T6 变高，
    //     直接接出去才是"数据和标记同一拍"。
    //     若再加一拍，标记会比数据晚一个周期，下游 i2s_tx 就会锁到旧样本
    //     —— 这正是文件头铁律③要防的事。
    // ============================================================
    reg [LATENCY-1:0] valid_sr;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) valid_sr <= {LATENCY{1'b0}};
        else        valid_sr <= {valid_sr[LATENCY-2:0], sample_tick};
    end

    assign mix_valid = valid_sr[LATENCY-1];

endmodule
