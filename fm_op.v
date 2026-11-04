`timescale 1ns/1ps
// ============================================================
//  fm_op.v —— 2 运算器 FM（A2）：用一个人的声音去抖动另一个人的音高
//
//  原理一句话：把**调制器**的输出加到**载波**的相位上，
//             载波频率被推来推去 → 产生边带 → 就是电琴（DX7）的音色。
//
//      freq_mod     → [调制器 osc_dds] → × 调制指数 ──(★打一拍)──→ 相位偏移
//      freq_carrier → [载波   osc_dds] ←──────────────────────────────┘
//
// ------------------------------------------------------------
//  ★★ 铁律：调制量必须**寄存一拍**，而且这一拍算 FM 音色的一部分
// ------------------------------------------------------------
//  若调制器输出**组合直连**到载波相位，就形成组合环（A 算 B、B 算 A），
//  综合工具直接报 `combinational loop`：轻则不收敛，重则被优化成乱东西。
//  同时这一拍也决定了调制器相对载波的相位延迟（20.8 µs @48 kHz，
//  听感无影响，但对时序收敛至关重要）。
//  本模块的做法是：用调制器的 `sample_valid` 当**使能**去锁存调制量，
//  于是"打一拍"和"只在有效样本上更新"一次做完。
//
// ------------------------------------------------------------
//  为什么是"相位调制（PM）"而不是"改频率字"
// ------------------------------------------------------------
//  给 `osc_dds.v` 加了一个 `phase_offset` 输入（`PHASE_MOD_EN=1` 打开）：
//  它只加到**查表用的相位**上，**不进相位累加器**。
//  若把调制量加进累加器（或直接改频率字），相位等于被**积分**一次，
//  那是另一回事（真 FM），频谱与听感都不同。
//  DX7 那一类电琴用的正是 PM —— 所以这里就是 PM。
//
// ------------------------------------------------------------
//  量纲（调制指数怎么定）
// ------------------------------------------------------------
//  相位 1 圈 = 2^32。取
//        phase_offset = mod_sample × mod_index
//  两个 16 位数相乘，量程 ±32767 × 65535 ≈ ±2^31 → **满调制指数时偏移半圈**。
//  想更夸张就调大 mod_index 或用 MOD_SHIFT 左移。
//  ⚠️ mod_sample 是 signed、mod_index 是 unsigned → 显式拓宽再乘
//     （signed × unsigned 陷阱，见 vca.v）。
//
//  验收（`tb_fm.v`）：mod_index=0 时退化成纯正弦；加大后出现调制边带；
//                    载波相位回绕处波形连续；综合无 combinational loop
//  依赖：P0-3 `osc_dds.v` ✅（用它的 PHASE_MOD_EN 端口）
// ============================================================

module fm_op #(
    // ---- 振荡器（与 osc_dds 保持一致）----
    parameter integer PHASE_W  = 32,
    parameter integer TABLE_AW = 10,
    parameter integer WAVE_W   = 2,
    parameter integer BITS     = 12,
    parameter integer FRAC_W   = 18,
    // ---- 调制 ----
    parameter integer MOD_W    = 16,     // 调制指数位宽（0 = 无调制）
    parameter integer MOD_SHIFT= 0,      // 调制量额外左移（加大调制指数用）
    parameter integer OUT_W    = 16
)(
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    sample_tick,

    input  wire [PHASE_W-1:0]      freq_carrier,   // 载波（决定音高）
    input  wire [PHASE_W-1:0]      freq_mod,       // 调制器（决定音色，常与载波成整数比）
    input  wire [MOD_W-1:0]        mod_index,      // =0 时退化成纯正弦

    input  wire                    gate,           // ★ 接 1'b1

    output reg  signed [OUT_W-1:0] sample_out,
    output wire                    sample_valid
);

    // 两个算子都用正弦波表（FM 的标准做法：基波 + 基波调制）
    localparam [WAVE_W-1:0] WAVE_SINE = {WAVE_W{1'b0}};

    // ============================================================
    //  1) 调制器 —— 一个独立的 osc_dds（**有自己的相位累加器**）
    // ============================================================
    wire signed [OUT_W-1:0] mod_s;
    wire                    mod_valid;

    osc_dds #(
        .PHASE_W   (PHASE_W),
        .TABLE_AW  (TABLE_AW),
        .WAVE_W    (WAVE_W),
        .BITS      (BITS),
        .FRAC_W    (FRAC_W),
        .OUT_W     (OUT_W)
    ) u_mod (
        .clk          (clk),
        .rst_n        (rst_n),
        .sample_tick  (sample_tick),
        .gate         (gate),
        .freq_word    (freq_mod),
        .wave_sel     (WAVE_SINE),
        .phase_offset ({PHASE_W{1'b0}}),
        .sample_out   (mod_s),
        .sample_valid (mod_valid)
    );

    // ============================================================
    //  2) 调制量 = mod_s × mod_index，再左移 MOD_SHIFT
    //     ⚠️ 显式拓宽：mod_s 有符号、mod_index 无符号
    // ============================================================
    wire signed [31:0] ma   = $signed(mod_s);
    wire signed [31:0] mb   = $signed({{(32-MOD_W){1'b0}}, mod_index});   // 恒非负
    wire signed [31:0] prod = (ma * mb) <<< MOD_SHIFT;

    // ============================================================
    //  3) ★ 寄存一拍：既是"避开组合环"，也是 FM 音色的一部分
    //     用 mod_valid 当使能 → 只在"调制器出新的有效样本"那拍更新
    // ============================================================
    reg signed [31:0] off_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)            off_r <= 32'sd0;
        else if (mod_valid)    off_r <= prod;
    end

    // ============================================================
    //  4) 载波 —— 另一个独立的 osc_dds，相位偏移接上面的调制量
    //     ⚠️ 相位加法在 osc_dds 内部做（phase_idx = phase + freq_word + offset），
    //        32 位自然回绕，**不要**在外部加 if 判断
    // ============================================================
    wire signed [OUT_W-1:0] car_s;
    wire                    car_valid;

    osc_dds #(
        .PHASE_W      (PHASE_W),
        .TABLE_AW     (TABLE_AW),
        .WAVE_W       (WAVE_W),
        .BITS         (BITS),
        .FRAC_W       (FRAC_W),
        .OUT_W        (OUT_W),
        .PHASE_MOD_EN (1)                 // ★ 打开相位调制输入
    ) u_car (
        .clk          (clk),
        .rst_n        (rst_n),
        .sample_tick  (sample_tick),
        .gate         (gate),
        .freq_word    (freq_carrier),
        .wave_sel     (WAVE_SINE),
        .phase_offset (off_r[PHASE_W-1:0]),
        .sample_out   (car_s),
        .sample_valid (car_valid)
    );

    // ============================================================
    //  5) 输出寄存 + valid 同拍对齐
    // ============================================================
    reg valid_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sample_out <= {OUT_W{1'b0}};
            valid_r    <= 1'b0;
        end else begin
            sample_out <= car_s;
            valid_r    <= car_valid;
        end
    end

    assign sample_valid = valid_r;

endmodule
