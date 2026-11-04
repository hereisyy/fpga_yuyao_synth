`timescale 1ns/1ps
// ============================================================
//  additive.v —— 加法合成（A1）：让一个人同时哼 8 个音
//
//  f0, 2f0, 3f0 … Nf0 各是一个"分音"，每个分音有自己的音量
//  （来自 A3 `partial_amp_tbl.v`），加起来就是音色。
//
// ------------------------------------------------------------
//  ★ 为什么每个分音必须是**独立的 osc_dds 实例**
// ------------------------------------------------------------
//  赛题第 15~16 页：**加法合成按"运算分量"计数**，
//  而**波表查表不管做几个谐波都只算 1 个振荡器**。
//  所以"每个分音一个独立相位累加器"既是技术要求，也是分数来源：
//      8 个分音 = 8 个振荡器（叠加在波表声部之上）。
//  ⚠️ 绝不能共用一条相位 —— 那样所有分音同相，加法等于白做，
//     音色完全不对，评委翻 RTL 也不认。
//
//  实现：直接例化 NP 份 `osc_dds`（它自带相位累加器 + 查表 + 插值），
//        频率字用"移位 + 加法"算倍频（**不用乘法器**）。
//
// ------------------------------------------------------------
//  两条约束
// ------------------------------------------------------------
//  ① **混叠**：第 k 分音 k×f0 必须 < 24 kHz → 由 A3 给 `n_partials`
//     控制，本模块只负责"超出的分音按 0 幅度处理"（等于不发声）。
//  ② **归一化**：A3 的表保证 Σamp ≤ 满量程，所以加权和
//         Σ(sample_k × amp_k) ≤ 32767 × 4095
//     右移 AW 位后 ≤ 32767，**刚好不溢出**。饱和仍是防御性的。
//
//  ⚠️ 加权是 `signed × unsigned`（分音有符号、幅度无符号），
//     照抄 `vca.v` 的显式拓宽写法，否则负半周全毁。
//
//  验收：谱上能数出 N 个峰；各分音相位独立；高音区砍分音后混叠下降
//  依赖：P0-3 `osc_dds.v` ✅、A3 `partial_amp_tbl.v`
//  ⚠️ 资源：每个 osc_dds 内含一份波表 ROM（3 块 BSRAM）→ NP=8 就是 24 块。
//     P1 必须把 ROM 提出来做共享时分复用（见 osc_dds.v 文件头"六"）。
// ============================================================

module additive #(
    parameter integer NP       = 8,      // 分音数
    parameter integer AW       = 12,     // 分音幅度位宽
    // ---- 与 osc_dds 保持一致（别改歪，否则例化对不上）----
    parameter integer PHASE_W  = 32,
    parameter integer TABLE_AW = 10,
    parameter integer WAVE_W   = 2,
    parameter integer BITS     = 12,
    parameter integer FRAC_W   = 18,
    parameter integer OUT_W    = 16
)(
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    sample_tick,

    // 基频字（**不含颤音**）：各分音要乘整数倍，
    // 用含颤音的字会把颤音深度也乘上去（2 分音的颤音是 1 分音的两倍），音乐上不对
    input  wire [PHASE_W-1:0]      freq_base,

    input  wire [NP*AW-1:0]        partial_amp,   // 已归一化（Σ ≤ 满量程）
    input  wire [3:0]              n_partials,    // 本档用几个（混叠红线）
    input  wire [WAVE_W-1:0]       wave_sel,
    input  wire                    gate,          // ★ 接 1'b1（自由运行）

    output reg  signed [OUT_W-1:0] sample_out,
    output wire                    sample_valid
);

    // ============================================================
    //  1) 倍频：第 k 分音的频率字 = freq_base × (k+1)
    //     用移位 + 加法（k 只有 1~8，写成 case 最省事，也不用乘法器）
    // ============================================================
    function [PHASE_W-1:0] mul_partial;
        input [PHASE_W-1:0] f;
        input integer       m;
        begin
            case (m)
                1:       mul_partial = f;
                2:       mul_partial = f << 1;
                3:       mul_partial = f + (f << 1);
                4:       mul_partial = f << 2;
                5:       mul_partial = f + (f << 2);
                6:       mul_partial = (f << 1) + (f << 2);
                7:       mul_partial = f + (f << 1) + (f << 2);
                8:       mul_partial = f << 3;
                default: mul_partial = f;
            endcase
        end
    endfunction

    wire [PHASE_W-1:0] f_k [0:NP-1];

    genvar gk;
    generate
        for (gk = 0; gk < NP; gk = gk + 1) begin : g_freq
            assign f_k[gk] = mul_partial(freq_base, gk + 1);
        end
    endgenerate

    // ============================================================
    //  2) 例化 NP 个振荡器 —— **每个都是独立的相位累加器**
    // ============================================================
    wire signed [OUT_W-1:0] osc_s     [0:NP-1];
    wire                    osc_valid [0:NP-1];

    generate
        for (gk = 0; gk < NP; gk = gk + 1) begin : g_osc
            osc_dds #(
                .PHASE_W   (PHASE_W),
                .TABLE_AW  (TABLE_AW),
                .WAVE_W    (WAVE_W),
                .BITS      (BITS),
                .FRAC_W    (FRAC_W),
                .OUT_W     (OUT_W)
            ) u_osc (
                .clk          (clk),
                .rst_n        (rst_n),
                .sample_tick  (sample_tick),
                .gate         (gate),
                .freq_word    (f_k[gk]),
                .wave_sel     (wave_sel),
                .phase_offset ({PHASE_W{1'b0}}),   // PHASE_MOD_EN=0，用不到
                .sample_out   (osc_s[gk]),
                .sample_valid (osc_valid[gk])
            );
        end
    endgenerate

    // ============================================================
    //  3) 加权求和：Σ(sample_k × amp_k)，其中 k ≥ n_partials 的项不参与
    //     ⚠️ 两边都要显式拓宽成 32 位有符号再乘（signed × unsigned 陷阱）
    // ============================================================
    integer k;
    reg signed [31:0] acc;
    reg signed [31:0] sa;
    reg signed [31:0] ga;

    always @(*) begin
        acc = 32'sd0;
        for (k = 0; k < NP; k = k + 1) begin
            if (k < n_partials) begin
                sa  = $signed(osc_s[k]);
                ga  = $signed({{(32-AW){1'b0}}, partial_amp[k*AW +: AW]});  // 幅度恒非负
                acc = acc + sa * ga;
            end
        end
    end

    // 右移 AW 位回到 16 位量程（算术右移，负数才对）
    wire signed [31:0] scaled = acc >>> AW;

    localparam signed [OUT_W-1:0] MAXW = (1 <<< (OUT_W-1)) - 1;   //  32767
    localparam signed [OUT_W-1:0] MINW = -(1 <<< (OUT_W-1));      // -32768

    wire signed [OUT_W-1:0] satv = (scaled > MAXW) ? MAXW :
                                   (scaled < MINW) ? MINW : scaled[OUT_W-1:0];

    // ============================================================
    //  4) 输出寄存 + sample_valid 同拍对齐
    //     ⚠️ 所有 osc_dds 的延迟相同，所以取第 0 路的 valid 就代表全体
    //        （这个"取一路代表全体"能成立的前提是**各实例参数完全一致**）
    // ============================================================
    reg valid_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sample_out <= {OUT_W{1'b0}};
            valid_r    <= 1'b0;
        end else begin
            sample_out <= satv;
            valid_r    <= osc_valid[0];
        end
    end

    assign sample_valid = valid_r;

endmodule
