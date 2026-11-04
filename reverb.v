// ============================================================
//  reverb.v —— P3：单声道混响（Schroeder / Freeverb-lite）
//
//  规格来源：`P3_HANDOFF.md` §4（Q2/Q3/Q4 已冻结）＋ `docs\adr\0003-effects-are-mono.md`
//  位置：`mixer_tree → reverb → 输出寄存 → i2s_tx`
//
//  ── 结构 ──────────────────────────────────────────────────
//
//      dry ─┬─→ pre-delay(10 ms) ─┬─→ comb1 ─┐
//           │                     ├─→ comb2 ─┤
//           │                     ├─→ comb3 ─┼→ Σ/4 → ap1 → ap2 ─┬→ ×1/2 ─┐
//           │                     └─→ comb4 ─┘                    │        ├→ Σ → 饱和 → out
//           └────────── 干声（延迟 LATENCY 拍对齐）──────────────────────────┘
//
//  · 4 个**并联**梳状：长度互质（差 65/137/75），去相关，尾巴才不像"嗡嗡"
//  · 2 个**串联**全通：扩散（把离散回声抹成连续尾巴）
//  · 每个梳状的反馈路径里有一阶低通（阻尼）：高频衰减更快，消除金属声
//
//  ── 延迟长度（48 kHz，P3_HANDOFF §4 的表）───────────────
//     Freeverb 的经典长度是按 44.1 kHz 设计的，48 kHz 要 ×1.088：
//       梳状：1557/1617/1491/1422 → **1694 / 1759 / 1622 / 1547** 采样
//       全通： 225/ 556         → **245 / 605** 采样
//       pre-delay：10 ms = **480** 采样
//
//  ── ★ 环内延迟必须补偿（P3_HANDOFF 坑 #1）──────────────
//     延时线的读是**同步读**（BRAM 的读口自带 1 拍寄存），所以
//     **有效环路周期 = 缓冲深度 + 1**。若按声学长度直接当深度，
//     每个梳状都会短 1 拍 —— 4 个都短 1 拍听起来不明显，但会破坏
//     "4 个互质"的意图，并在长尾里显出周期性。所以下面统一取
//     `深度 = 声学长度 - 1`，让有效周期正好等于规格值。
//
//  ── RT60 与反馈系数 ──────────────────────────────────────
//     单根延迟线 L 拍、每圈乘 g ⇒ RT60 = -3·L / (fs·log10 g)。
//     反解：  g = 10^(-3·L / (RT60·fs))
//     ★ 固定点截断补偿（2026-10-23 实测标定）：
//        反馈路径每圈有 2~3 处 >>15 / >>DAMP 的截断，实测 RT60 比公式值**短约 21%**
//        （公式给 1.5 s，实测 1.18 s）。所以参数按 `RT60_cal = RT60_target / 0.79`
//        反解 —— 即把目标 1.5 s 当作 1.9 s 代入公式：
//        L=1694 → g=0.8796 → Q15 28822
//        L=1759 → g=0.8753 → Q15 28681
//        L=1622 → g=0.8844 → Q15 28980
//        L=1547 → g=0.8894 → Q15 29145
//        实测（`tb_reverb` 的 T30 外推）：**约 1.5 s** ✔ 落在规格 1.2~1.8 s 内。
//     RT60 = 1500 ms 的**理论值**（不补偿时，供对照）：27853 / 27680 / 28046 / 28248
//     ⚠️ **改 RT60_MS 必须重算这 4 个数**（Verilog 里没有 exp/log）。
//        常用值备查（fs=48k）：
//          RT60=1200 ms：26744 / 26536 / 26976 / 27219
//          RT60=1500 ms：27853 / 27680 / 28046 / 28248
//          RT60=1800 ms：28617 / 28469 / 28783 / 28956
//        `tb_reverb.v` 会独立手算 RT60 并核对（±25%，见验收第 2 条）。
//
//  ── 阻尼 ─────────────────────────────────────────────────
//     `lp += (y - lp) >>> DAMP_SHIFT`，**不用乘法器**（P3_HANDOFF §4）。
//     DAMP_SHIFT=2 ⇒ 转折 ≈ fs/(2π·2^2)… 实际极点 0.75 ⇒ 约 2.2 kHz；
//     =1 时极点 0.5 ⇒ 约 5.3 kHz（更亮）。默认 2（温暖、不刺）。
//     ⚠️ 反馈系数必须 < 1（g ≤ 0.88），否则直流会在环里累积成偏移
//        —— 上电"噗"一声就是这个（P3_HANDOFF 坑 #3）。
//
//  ── fx_bypass（接口契约 §2）──────────────────────────────
//     **整条混响通路始终在跑**，只在最后的求和处用 `fx_bypass` 选干/湿。
//     所以旁路**不可能**改变延迟拍数 —— 这是结构上保证的，不是靠小心。
//
//  ── LATENCY = 9 拍（自 sample_tick 到 out_valid），逐级数清 ──
//     1) xr       输入寄存
//     2) pd_rd    pre-delay 读
//     3) cbX_rd   4 个梳状并行读
//     4) csum_r   4 路求和 + 1/4 缩放
//     5) ap1_rd   全通 1 读（指针也在这拍锁存）
//     6) ap1_y    全通 1 出（写回上一拍的地址）
//     7) ap2_rd   全通 2 读
//     8) ap2_y    全通 2 出
//     9) mix      干湿求和 + 饱和 + 输出寄存
//
//  ── 资源（BSRAM 估算）────────────────────────────────────
//     4 梳状 6622 + 2 全通 850 + pre-delay 480 = **7952 采样 × 16 bit
//     = 124 Kbit ≈ 6.9 块**（18 Kbit/块）。P3_HANDOFF 的预算是 11 块，
//     通得过；但**没综合过**，以 Gowin 报告为准。
//
//  ── 复位与延迟线初值 ─────────────────────────────────────
//     延时线是 BSRAM：Gowin 的 BRAM **上电即 0**（配置比特流的一部分），
//     所以硬件上不需要"清零状态机"。仿真里用 `translate_off` 的 initial
//     显式置 0，保证仿真与上板一致。
//     额外加一道保险：上电后前 `FLUSH_SAMPLES`(=2304 > 最长通路 2174)
//     个采样**只出干声**，万一某块 BRAM 初值不为 0，脏数据也在这段时间
//     里被正常写入的 0 覆盖掉，不会作为"残响"放出来（坑 #4）。
// ============================================================
`default_nettype none

module reverb #(
    parameter integer FS_HZ      = 48_000,
    parameter integer W          = 16,
    // pre-delay 的**声学**长度（采样）。10 ms @48k = 480
    parameter integer PD_SAMPLES = 480,
    // 一阶阻尼的移位量（越大越暗）。见文件头
    parameter integer DAMP_SHIFT = 2,
    // 湿声增益 = 1/2^WET_SHIFT
    parameter integer WET_SHIFT  = 1,
    // 自 sample_tick 到 out_valid 的拍数（**恒定**，见文件头计数）
    parameter integer LATENCY    = 9,
    // RT60=1500 ms @48k 的 4 个梳状反馈系数（Q15，公式见文件头）
    parameter signed [15:0] G_C1 = 16'sd28822,   // L=1694
    parameter signed [15:0] G_C2 = 16'sd28681,   // L=1759
    parameter signed [15:0] G_C3 = 16'sd28980,   // L=1622
    parameter signed [15:0] G_C4 = 16'sd29145,   // L=1547
    // 全通系数（Q15，0.5）
    parameter signed [15:0] G_AP = 16'sd16384,
    // 上电后只出干声的采样数（见文件头）
    parameter integer FLUSH_SAMPLES = 2304
)(
    input  wire              clk,
    input  wire              rst_n,
    input  wire              sample_tick,      // 48 kHz 单周期脉冲

    input  wire signed [W-1:0] dry_in,         // 混音树输出（16 bit 有符号）
    input  wire                fx_bypass,      // 1 = 只出干声（**不改延迟**）

    output reg  signed [W-1:0] out,
    output wire                out_valid
);

    // ============================================================
    //  有效环路周期 = 深度 + 1（同步读自带 1 拍）⇒ 深度 = 声学长 - 1
    // ============================================================
    localparam integer PD_D  = PD_SAMPLES - 1;   // 479 → 有效 480
    localparam integer C1_D  = 1693;             // 有效 1694
    localparam integer C2_D  = 1758;             // 有效 1759
    localparam integer C3_D  = 1621;             // 有效 1622
    localparam integer C4_D  = 1546;             // 有效 1547
    localparam integer A1_D  =  244;             // 有效 245
    localparam integer A2_D  =  604;             // 有效 605

    // ============================================================
    //  参数自检（只仿真）
    // ============================================================
    // synthesis translate_off
    initial begin
        if (PD_SAMPLES < 2) begin
            $display("!! reverb: PD_SAMPLES=%0d 太小", PD_SAMPLES);
            $finish;
        end
        if (W != 16) begin
            $display("!! reverb: 本实现按 W=16 写的（接口契约 §1）");
            $finish;
        end
        //  ⚠️ Q15 有符号装不下 1.0（最大 32767 = 0.99997），所以"≥1"在
        //     数值上不可能出现；这里改成查"不合理区间"：≤0 或 ≥0.95。
        //     （第一版写成 `>= 16'sd32768`，16'sd32768 溢出成 −32768，
        //       于是对任何正数都成立 —— 一上电就自杀，已修。）
        if ((G_C1 <= 0) || (G_C1 >= 16'sd31130) ||
            (G_C4 <= 0) || (G_C4 >= 16'sd31130)) begin
            $display("!! reverb: 反馈系数异常（G_C1=%0d G_C4=%0d）", G_C1, G_C4);
            $display("   RT60 反解公式 g = 10^(-3L/(RT60*fs))，见文件头；合法约 0.80~0.89");
            $finish;
        end
    end
    // synthesis translate_on

    // ============================================================
    //  1) pre-delay 延时线
    // ============================================================
    (* syn_ramstyle = "block_ram" *) reg signed [W-1:0] pd_mem [0:PD_D-1];
    reg [10:0]          pd_wp;
    reg signed [W-1:0]  pd_rd;
    reg signed [W-1:0]  xr;             // 输入寄存（第 1 拍）

    // ============================================================
    //  2) 4 个并联梳状
    //     每个：读(同拍写回) → 一阶阻尼 → 乘 g → 加输入
    // ============================================================
    (* syn_ramstyle = "block_ram" *) reg signed [W-1:0] c1_mem [0:C1_D-1];
    (* syn_ramstyle = "block_ram" *) reg signed [W-1:0] c2_mem [0:C2_D-1];
    (* syn_ramstyle = "block_ram" *) reg signed [W-1:0] c3_mem [0:C3_D-1];
    (* syn_ramstyle = "block_ram" *) reg signed [W-1:0] c4_mem [0:C4_D-1];

    reg [11:0]         c1_wp, c2_wp, c3_wp, c4_wp;
    reg signed [W-1:0] c1_rd, c2_rd, c3_rd, c4_rd;
    reg signed [W-1:0] lp1, lp2, lp3, lp4;      // 阻尼状态

    // 阻尼后的值（组合，1 拍内算完）
    //
    //  ⚠️⚠️ 这里必须**先拓宽到 W+1 位再相减**（2026-10-23 实际踩到）：
    //      `c1_rd - lp1` 两边都是 16 位，Verilog 按 16 位算这个减法，
    //      当两者符号相反时差值会**溢出回绕**（例如 30000 − (−30000) = 60000
    //      在 16 位里有符号数是 −5536）。后果是高电平/高频段被额外砍掉：
    //      实测 RT60 只有设计值的 80%（1.19 s vs 1.5 s），听感偏"干、刺"。
    //      ⇒ 差值用 W+1 位算，移位后再截回 W 位。
    //  ⚠️ 注意：`$signed(a) - $signed(b)` 的**表达式宽度仍是 max(width)**，
    //     赋给 W+1 位的 wire 不会自动加宽！所以必须在这里**显式符号扩展**：
    //     把每个操作数拼成 W+1 位再相减。只改 wire 位宽是无效的
    //     （第一版就是这么改的，波形一模一样 —— 因为减法还是在 16 位里算）。
    wire signed [W:0] cx1 = {c1_rd[W-1], c1_rd};
    wire signed [W:0] cx2 = {c2_rd[W-1], c2_rd};
    wire signed [W:0] cx3 = {c3_rd[W-1], c3_rd};
    wire signed [W:0] cx4 = {c4_rd[W-1], c4_rd};
    wire signed [W:0] lx1 = {lp1[W-1], lp1};
    wire signed [W:0] lx2 = {lp2[W-1], lp2};
    wire signed [W:0] lx3 = {lp3[W-1], lp3};
    wire signed [W:0] lx4 = {lp4[W-1], lp4};

    wire signed [W:0] da1 = lx1 + ((cx1 - lx1) >>> DAMP_SHIFT);
    wire signed [W:0] da2 = lx2 + ((cx2 - lx2) >>> DAMP_SHIFT);
    wire signed [W:0] da3 = lx3 + ((cx3 - lx3) >>> DAMP_SHIFT);
    wire signed [W:0] da4 = lx4 + ((cx4 - lx4) >>> DAMP_SHIFT);

    wire signed [W-1:0] d1 = da1[W-1:0];
    wire signed [W-1:0] d2 = da2[W-1:0];
    wire signed [W-1:0] d3 = da3[W-1:0];
    wire signed [W-1:0] d4 = da4[W-1:0];

    // 反馈量 = g × 阻尼值（Q15）
    wire signed [31:0] fb1 = ($signed(d1) * $signed(G_C1)) >>> 15;
    wire signed [31:0] fb2 = ($signed(d2) * $signed(G_C2)) >>> 15;
    wire signed [31:0] fb3 = ($signed(d3) * $signed(G_C3)) >>> 15;
    wire signed [31:0] fb4 = ($signed(d4) * $signed(G_C4)) >>> 15;

    // ============================================================
    //  3) 2 个串联全通（读、算、写各占 1 拍，指针锁存一拍照抄）
    // ============================================================
    (* syn_ramstyle = "block_ram" *) reg signed [W-1:0] a1_mem [0:A1_D-1];
    (* syn_ramstyle = "block_ram" *) reg signed [W-1:0] a2_mem [0:A2_D-1];

    reg [10:0]         a1_wp, a1_wp_d;
    reg [10:0]         a2_wp, a2_wp_d;
    reg signed [W-1:0] a1_rd, a1_x_d, a1_y;
    reg signed [W-1:0] a2_rd, a2_x_d, a2_y;
    reg signed [W-1:0] csum_r;                  // 4 路求和（已 1/4 缩放）

    // 全通输出（组合）：y = -g·x + 延迟项
    wire signed [31:0] a1_y_c = $signed(a1_rd) - (($signed(a1_x_d) * $signed(G_AP)) >>> 15);
    wire signed [31:0] a2_y_c = $signed(a2_rd) - (($signed(a2_x_d) * $signed(G_AP)) >>> 15);
    // 写回（组合）：mem = x + g·y
    wire signed [31:0] a1_wr  = $signed(a1_x_d) + (($signed(a1_y_c[W-1:0]) * $signed(G_AP)) >>> 15);
    wire signed [31:0] a2_wr  = $signed(a2_x_d) + (($signed(a2_y_c[W-1:0]) * $signed(G_AP)) >>> 15);

    // ============================================================
    //  4) 干声对齐链（深度 = LATENCY，两条路径的延迟**逐拍相同**）
    // ============================================================
    reg signed [W-1:0] dry_sr [0:LATENCY-1];
    reg [LATENCY-1:0]  valid_sr;

    // ============================================================
    //  5) 上电只出干声的计数
    // ============================================================
    reg [15:0] flush_cnt;
    wire       wet_ok = (flush_cnt >= FLUSH_SAMPLES[15:0]);

    // ============================================================
    //  6) 混合与饱和
    // ============================================================
    wire signed [W-1:0] dry_d  = dry_sr[LATENCY-1];
    wire signed [W-1:0] wet_in = a2_y;

    wire signed [31:0] wet32 = $signed(wet_in) >>> WET_SHIFT;
    wire signed [31:0] sum32 = $signed(dry_d) + wet32;

    localparam signed [15:0] MAXS = 16'sd32767;
    localparam signed [15:0] MINS = -16'sd32768;

    wire signed [15:0] mix = (sum32 > MAXS) ? MAXS :
                             (sum32 < MINS) ? MINS : sum32[15:0];

    // ============================================================
    //  7) 时序：全部挂在 sample_tick 上，逐级固定 1 拍
    // ============================================================
    integer q;
    // synthesis translate_off
    initial begin
        for (q = 0; q < PD_D; q = q + 1) pd_mem[q] = {W{1'b0}};
        for (q = 0; q < C1_D; q = q + 1) c1_mem[q] = {W{1'b0}};
        for (q = 0; q < C2_D; q = q + 1) c2_mem[q] = {W{1'b0}};
        for (q = 0; q < C3_D; q = q + 1) c3_mem[q] = {W{1'b0}};
        for (q = 0; q < C4_D; q = q + 1) c4_mem[q] = {W{1'b0}};
        for (q = 0; q < A1_D; q = q + 1) a1_mem[q] = {W{1'b0}};
        for (q = 0; q < A2_D; q = q + 1) a2_mem[q] = {W{1'b0}};
    end
    // synthesis translate_on

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // 第 1 拍
            xr        <= {W{1'b0}};
            // pre-delay
            pd_wp     <= 11'd0;
            pd_rd     <= {W{1'b0}};
            // 梳状
            c1_wp <= 12'd0; c2_wp <= 12'd0; c3_wp <= 12'd0; c4_wp <= 12'd0;
            c1_rd <= {W{1'b0}}; c2_rd <= {W{1'b0}};
            c3_rd <= {W{1'b0}}; c4_rd <= {W{1'b0}};
            lp1   <= {W{1'b0}}; lp2   <= {W{1'b0}};
            lp3   <= {W{1'b0}}; lp4   <= {W{1'b0}};
            // 全通
            a1_wp <= 11'd0; a1_wp_d <= 11'd0;
            a2_wp <= 11'd0; a2_wp_d <= 11'd0;
            a1_rd <= {W{1'b0}}; a1_x_d <= {W{1'b0}}; a1_y <= {W{1'b0}};
            a2_rd <= {W{1'b0}}; a2_x_d <= {W{1'b0}}; a2_y <= {W{1'b0}};
            csum_r <= {W{1'b0}};
            // 对齐链 / 输出
            valid_sr <= {LATENCY{1'b0}};
            for (q = 0; q < LATENCY; q = q + 1) dry_sr[q] <= {W{1'b0}};
            out <= {W{1'b0}};
            flush_cnt <= 16'd0;
        end else if (sample_tick) begin
            // ---------- 第 1 拍：输入寄存 ----------
            xr <= dry_in;

            // ---------- 第 2 拍：pre-delay 读出 + 写回 ----------
            pd_rd     <= pd_mem[pd_wp];
            pd_mem[pd_wp] <= xr;
            pd_wp     <= (pd_wp == PD_D[10:0] - 11'd1) ? 11'd0 : (pd_wp + 11'd1);

            // ---------- 第 3 拍：4 个梳状并行读 + 写回 ----------
            //   写入值 = pre-delay 输出 + g×(阻尼后的上一拍读出)
            c1_rd <= c1_mem[c1_wp];  c1_mem[c1_wp] <= pd_rd + fb1[W-1:0];
            c2_rd <= c2_mem[c2_wp];  c2_mem[c2_wp] <= pd_rd + fb2[W-1:0];
            c3_rd <= c3_mem[c3_wp];  c3_mem[c3_wp] <= pd_rd + fb3[W-1:0];
            c4_rd <= c4_mem[c4_wp];  c4_mem[c4_wp] <= pd_rd + fb4[W-1:0];
            c1_wp <= (c1_wp == C1_D[11:0] - 12'd1) ? 12'd0 : (c1_wp + 12'd1);
            c2_wp <= (c2_wp == C2_D[11:0] - 12'd1) ? 12'd0 : (c2_wp + 12'd1);
            c3_wp <= (c3_wp == C3_D[11:0] - 12'd1) ? 12'd0 : (c3_wp + 12'd1);
            c4_wp <= (c4_wp == C4_D[11:0] - 12'd1) ? 12'd0 : (c4_wp + 12'd1);
            // 阻尼状态随读出值更新（一阶低通）
            lp1 <= d1; lp2 <= d2; lp3 <= d3; lp4 <= d4;

            // ---------- 第 4 拍：4 路求和 + 1/4 缩放 ----------
            csum_r <= ((c1_rd >>> 2) + (c2_rd >>> 2) +
                       (c3_rd >>> 2) + (c4_rd >>> 2));

            // ---------- 第 5 拍：全通 1 读 ----------
            a1_rd   <= a1_mem[a1_wp];
            a1_wp_d <= a1_wp;
            a1_x_d  <= csum_r;
            a1_wp   <= (a1_wp == A1_D[10:0] - 11'd1) ? 11'd0 : (a1_wp + 11'd1);

            // ---------- 第 6 拍：全通 1 出 + 写回 ----------
            a1_y         <= a1_y_c[W-1:0];
            a1_mem[a1_wp_d] <= a1_wr[W-1:0];

            // ---------- 第 7 拍：全通 2 读 ----------
            a2_rd   <= a2_mem[a2_wp];
            a2_wp_d <= a2_wp;
            a2_x_d  <= a1_y;
            a2_wp   <= (a2_wp == A2_D[10:0] - 11'd1) ? 11'd0 : (a2_wp + 11'd1);

            // ---------- 第 8 拍：全通 2 出 + 写回 ----------
            a2_y         <= a2_y_c[W-1:0];
            a2_mem[a2_wp_d] <= a2_wr[W-1:0];

            // ---------- 干声对齐链（与上面同一拍推进）----------
            dry_sr[0] <= dry_in;
            for (q = 1; q < LATENCY; q = q + 1) dry_sr[q] <= dry_sr[q-1];

            // ---------- 第 9 拍：混合 + 饱和 + 输出寄存 ----------
            //   ★ fx_bypass 只在这里生效：延迟由结构保证不变
            out <= fx_bypass ? dry_d : ((!wet_ok && flush_cnt < FLUSH_SAMPLES[15:0])
                                        ? dry_d : mix);
            valid_sr <= {valid_sr[LATENCY-2:0], 1'b1};

            if (flush_cnt < FLUSH_SAMPLES[15:0]) flush_cnt <= flush_cnt + 16'd1;
        end
    end

    assign out_valid = valid_sr[LATENCY-1];

endmodule

`default_nettype wire