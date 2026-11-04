`timescale 1ns/1ps
// ============================================================
//  voice_array.v —— P1-2：32 个独立声部（两套声部池）
//
//  一个"声部" = 1 个（或 2 个）独立相位累加器 + 1 个 ADSR + 1 个音量。
//
//      槽  0..15  = **特殊池**：每声部 2 个 osc_dds
//                     · 加法预设：正弦 f + 正弦 2f，相加后 /2
//                     · FM  预设：锯齿 f 调制 正弦 f（同音高，ratio=1）
//                   两种预设**共用同一份 2-osc 硬件**，只切参数
//      槽 16..31  = **波表池**：每声部 1 个 osc_dds，wave_sel 自由
//
//  资源（ADR-0001/0002 冻结）：16×2×6 + 16×1×6 = **288 / 340** BSRAM。
//
//  ── 五条会翻车的地方（P1_HANDOFF §5），本模块逐条落实 ──────────
//   ① **相位累加器绝不可时分复用**：每个声部（特殊池每声部两个）各实例化
//      一份 osc_dds，累加器在它内部。可以共享的只有乘法器（vca 里分的）
//      和波表 ROM（将来做）。这里一个声部几份 osc_dds 就是几份累加器。
//   ② **`osc_dds.gate` 接 `1'b1`**（自由运行），`adsr.gate` 接各声部
//      `note_on` 电平。两者**绝不接同一个信号** —— 否则松键瞬间振荡器
//      输出被清零，ADSR 的 Release 尾音被吃掉。
//   ③ **signed × unsigned**：adul 加法混合时先各自拓宽；
//      包络增益是无符号、样本是有符号，乘法交给 vca（它内部已显式拓宽）。
//   ④ 数据都寄存过（osc 与 vca 内部都有输出寄存器）。
//   ⑤ 不碰 top_synth.v / keyboard.v。
//
//  ── 每声部音量用**移位**而不是乘法 ─────────────────────────────
//    `voice_amp_shift[k]` 把包络增益右移 0..7 位，等效 1/2^n 音量。
//    这样"32 声部齐响不削顶"（test_mode 的第 2 条约束）只要给
//    voice_amp_shift 全 5，**不需要再多 32 个 DSP**。
//    而且移位是精确的二进制缩放，不会引入幅度误差。
//
//  ── 延迟 ──────────────────────────────────────────────────
//    osc_dds  LATENCY = 5（帧内画面延迟，见参数区的说明）
//    vca      LATENCY = VCA_NV + 1 = 5
//    ⇒ 本模块 LATENCY = **5 拍**（从 sample_tick 到 out_valid）
//    `out_valid` 与 `sample_out` **同一拍**有效。
//    ⚠️ 特殊池的 FM 相位偏移比调制器输出晚 1 拍寄存（避开组合环，
//       与 fm_op.v 同一做法）。它落在两帧之间，**不改变**上面的 5 拍。
// ============================================================
module voice_array #(
    // ---- 声部数 / 特殊池大小 ----
    parameter integer NV       = 32,
    parameter integer SPECIAL  = 16,     // 槽 0..15 特殊池，槽 16..31 波表池
    // ---- 振荡器（与 osc_dds 一致）----
    parameter integer PHASE_W  = 32,
    parameter integer TABLE_AW = 10,
    parameter integer WAVE_W   = 2,
    parameter integer BITS     = 12,
    parameter integer FRAC_W   = 18,
    parameter integer OUT_W    = 16,
    // 波表文件。⚠️ $readmemh 是相对**仿真器工作目录**找的，
    //   所以 tb / 顶层可以从各自的目录传 "./wavetable_all.hex"
    //   或 "../06_keyboard/wavetable_all.hex"，不必把 hex 复制过来。
    parameter         INIT_FILE = "wavetable_all.hex",
    // ---- 包络 / 音量 ----
    parameter integer GAIN_W   = 16,
    parameter integer AMPW     = 3,      // 每声部幅度右移档位宽（0..7）
    // 淡出系数位宽。ATTEN_FULL = 满量 = 不衰减；系数 = atten / ATTEN_FULL。
    //   取 ATTW=7 → ATTEN_FULL = 64：淡出每帧走 2/64，32 帧走完，
    //   正好对应冻结表里"偷声 32 采样淡出"那个 32。
    //   （分辨率比直接用 0..32 高一倍，每帧跳变更小。）
    //   ⚠️ 要与 voice_alloc 的同名参数一致，否则偷声淡出会停在半音量。
    parameter integer ATTW     = 7,
    parameter integer ATTEN_FULL = 64,
    parameter integer ATT_SHIFT = 5,
    parameter integer DEC_SHIFT = 12,
    parameter integer REL_SHIFT = 12,
    parameter integer SUS_PCT   = 70,
    // ---- FM 调制指数 ----
    parameter integer MOD_W    = 16,
    // ---- vca 时分复用路数（32 ÷ 4 = 8 个乘法器）----
    parameter integer VCA_NV   = 4,
    // ---- 延迟 ----
    //  LATENCY = **sample_tick 到 out_valid 的拍数** = VCA_NV + 3 = 7
    //    （vca 的输入锁存在 2026-10-25 推迟了 1 拍，见 vca.v 里 start_lat 的说明）
    //    （out_valid 只由 sample_tick 经 vca 的时隙链产生，与 osc 的深度无关）
    //  OSC_LAT = osc_dds 自己的流水深度（5 拍）。它造成的是**数据画面**
    //    比"这一帧的 tick"晚一帧：vca 在 tick 那一拍锁存的是 osc 上一帧
    //    算完的样本。音频是连续帧序列，整体只多一帧（20.8 µs）延迟，
    //    不会有任何错位或丢帧 —— P0 的 top_synth 就是同一个连法。
    //  ⚠️ 不要在 tb 里把这两个加起来当延迟断言，那会把正确的设计判成错的。
    parameter integer OSC_LAT  = 5,
    parameter integer LATENCY  = VCA_NV + 3,
    // ---- 波形编号（与 wavetable_rom.v 的排布一致）----
    parameter [WAVE_W-1:0] WAVE_SINE = {WAVE_W{1'b0}},   // 0
    parameter [WAVE_W-1:0] WAVE_SAW  = 2'd2              // 2
)(
    input  wire                       clk,
    input  wire                       rst_n,
    input  wire                       sample_tick,      // 48 kHz 单周期脉冲

    // ---- 每声部的演奏参数（来自 voice_alloc / test_mode）----
    input  wire [NV-1:0]              voice_gate,       // 电平：1 = 按住
    input  wire [NV*PHASE_W-1:0]      voice_freq,       // 频率字（已含颤音）
    input  wire [NV*WAVE_W-1:0]       voice_wave,       // 波表池的波形选择
    input  wire [NV*AMPW-1:0]         voice_amp_shift,  // 每声部音量 = 包络 >> n
    // 每声部淡出系数（0..ATTEN_FULL）。
    //   ATTEN_FULL = 不衰减（稳态）；0 = 完全静音。
    //   盗声时 voice_alloc 把它从满量线性拉到 0、换参数、再拉回来，
    //   这就是"单套振荡器硬件上的 cross-fade"（见 voice_alloc.v 文件头）。
    input  wire [NV*ATTW-1:0]         voice_atten,

    // ---- 全局音色参数（只作用于特殊池）----
    input  wire                       special_mode,     // 0 = 加法 1 = FM
    input  wire [MOD_W-1:0]           mod_index,        // FM 调制指数（0 = 纯正弦）

    // ---- 输出 ----
    output wire signed [NV*OUT_W-1:0] sample_out,       // 32 路已过包络的样本
    output wire                       out_valid,        // 与 sample_out 同拍
    output wire [NV-1:0]              voice_active,     // 1 = 该声部正在发声（含 RELEASE）
    output wire [NV-1:0]              voice_releasing   // 1 = 该声部在 RELEASE 段
);

    // ============================================================
    //  0) 参数自检（只仿真）
    // ============================================================
    // synthesis translate_off
    initial begin
        if (SPECIAL > NV) begin
            $display("!! voice_array: SPECIAL=%0d 超过 NV=%0d", SPECIAL, NV);
            $finish;
        end
        if (NV % VCA_NV != 0) begin
            $display("!! voice_array: NV=%0d 必须是 VCA_NV=%0d 的整数倍", NV, VCA_NV);
            $finish;
        end
    end
    // synthesis translate_on

    // ============================================================
    //  1) 振荡器：特殊池 2 个/声部，波表池 1 个/声部
    //     ⚠️ 一段 generate 写死"每声部一份"，不做任何跨声部复用 ——
    //        相位累加器不可共享（赛题按独立累加器计数，评委会翻 RTL）。
    // ============================================================

    // 每声部振荡器输出（扁平总线，避免 unpacked array 的综合差异）
    wire signed [NV*OUT_W-1:0] osc_raw;

    // mod_index 的显式零扩展（无符号 → 有符号容器，恒非负）
    wire signed [31:0] mod_idx_s = $signed({{(32-MOD_W){1'b0}}, mod_index});

    genvar i;
    generate
        // ---- 特殊池：槽 0..SPECIAL-1，每声部 2 个振荡器 ----
        for (i = 0; i < SPECIAL; i = i + 1) begin : g_special
            // 该声部自己的频率字 / 门控
            wire [PHASE_W-1:0] f0 = voice_freq[i*PHASE_W +: PHASE_W];

            // 算子 A：加法时是 2f 的正弦，FM 时是同音高的锯齿（调制器）
            //   ⚠️ freq<<1 不会溢出：真实最高频率字 C5=46.8M，B5=88.4M，
            //      左移一位 176M ≪ 2^32 = 4.29G。加两档八度也安全。
            wire [PHASE_W-1:0] freq_a = special_mode ? f0 : (f0 << 1);
            wire [WAVE_W-1:0]  wave_a = special_mode ? WAVE_SAW : WAVE_SINE;

            wire signed [OUT_W-1:0] a_s;
            wire                    a_valid;

            osc_dds #(
                .PHASE_W   (PHASE_W),
                .TABLE_AW  (TABLE_AW),
                .WAVE_W    (WAVE_W),
                .BITS      (BITS),
                .FRAC_W    (FRAC_W),
                .OUT_W     (OUT_W),
                .INIT_FILE (INIT_FILE)
            ) u_osc_a (
                .clk          (clk),
                .rst_n        (rst_n),
                .sample_tick  (sample_tick),
                .gate         (1'b1),               // ★ 铁律②：自由运行
                .freq_word    (freq_a),
                .wave_sel     (wave_a),
                .phase_offset ({PHASE_W{1'b0}}),
                .sample_out   (a_s),
                .sample_valid (a_valid)
            );

            // ---- FM 调制量：a_s × mod_index，用 a_valid 使能寄存一拍 ----
            //   寄存这一拍既避开"组合环"（调制器→相位→调制器），
            //   也定义了调制器相对载波的固定相位延迟（与 fm_op.v 相同）。
            wire signed [31:0] mod_prod = $signed(a_s) * mod_idx_s;

            reg signed [31:0] off_r;

            always @(posedge clk or negedge rst_n) begin
                if (!rst_n)          off_r <= 32'sd0;
                else if (a_valid)    off_r <= mod_prod;
            end

            // 加法预设下相位偏移必须恒 0，否则 B 也被相位调制了
            wire [PHASE_W-1:0] off_a = special_mode ? off_r[PHASE_W-1:0]
                                                    : {PHASE_W{1'b0}};

            // 算子 B：永远是正弦、永远是 f0（载波 / 基波）
            wire signed [OUT_W-1:0] b_s;
            wire                    b_valid;

            osc_dds #(
                .PHASE_W     (PHASE_W),
                .TABLE_AW    (TABLE_AW),
                .WAVE_W      (WAVE_W),
                .BITS        (BITS),
                .FRAC_W      (FRAC_W),
                .OUT_W       (OUT_W),
                .INIT_FILE   (INIT_FILE),
                .PHASE_MOD_EN(1)                    // ★ 打开相位调制输入
            ) u_osc_b (
                .clk          (clk),
                .rst_n        (rst_n),
                .sample_tick  (sample_tick),
                .gate         (1'b1),               // ★ 铁律②
                .freq_word    (f0),
                .wave_sel     (WAVE_SINE),
                .phase_offset (off_a),
                .sample_out   (b_s),
                .sample_valid (b_valid)
            );

            // ---- 输出选择 ----
            //   加法：(A + B) / 2 —— **先拓宽到 17 位再相加**，
            //         两个 ±32767 相加会溢出 16 位有符号（铁律③的同类陷阱）
            //   FM  ：只取载波 B（A 已经"变成"了相位偏移）
            wire signed [OUT_W:0] sum_ab = $signed(b_s) + $signed(a_s);
            wire signed [OUT_W-1:0] mix_ab = sum_ab >>> 1;   // ±65534/2 = ±32767 ✔

            assign osc_raw[i*OUT_W +: OUT_W] = special_mode ? b_s : mix_ab;

            // 两个振荡器的 valid 必须同拍（同一个 tick、同一个 LATENCY）
            // synthesis translate_off
            always @(posedge clk) begin
                if (rst_n && (a_valid !== b_valid)) begin
                    $display("!! voice_array: 槽 %0d 两算子 valid 不同拍", i);
                    $finish;
                end
            end
            // synthesis translate_on
        end

        // ---- 波表池：槽 SPECIAL..NV-1，每声部 1 个振荡器 ----
        for (i = SPECIAL; i < NV; i = i + 1) begin : g_wave
            wire signed [OUT_W-1:0] w_s;
            wire                    w_valid;

            osc_dds #(
                .PHASE_W   (PHASE_W),
                .TABLE_AW  (TABLE_AW),
                .WAVE_W    (WAVE_W),
                .BITS      (BITS),
                .FRAC_W    (FRAC_W),
                .OUT_W     (OUT_W),
                .INIT_FILE (INIT_FILE)
            ) u_osc (
                .clk          (clk),
                .rst_n        (rst_n),
                .sample_tick  (sample_tick),
                .gate         (1'b1),               // ★ 铁律②
                .freq_word    (voice_freq[i*PHASE_W +: PHASE_W]),
                .wave_sel     (voice_wave[i*WAVE_W +: WAVE_W]),
                .phase_offset ({PHASE_W{1'b0}}),
                .sample_out   (w_s),
                .sample_valid (w_valid)
            );

            assign osc_raw[i*OUT_W +: OUT_W] = w_s;
        end
    endgenerate

    // ============================================================
    //  2) 每声部一个 ADSR（gate = 该声部 note_on 电平）
    //     ⚠️ 铁律②：这个 gate 与 osc_dds 的 gate(1'b1) **不是同一个信号**
    // ============================================================
    wire [NV*GAIN_W-1:0] env_gain;

    generate
        for (i = 0; i < NV; i = i + 1) begin : g_env
            wire [2:0] st;

            adsr #(
                .GAIN_W   (GAIN_W),
                .ATT_SHIFT(ATT_SHIFT),
                .DEC_SHIFT(DEC_SHIFT),
                .REL_SHIFT(REL_SHIFT),
                .SUS_PCT  (SUS_PCT)
            ) u_env (
                .clk        (clk),
                .rst_n      (rst_n),
                .sample_tick(sample_tick),
                .gate       (voice_gate[i]),
                .gain       (env_gain[i*GAIN_W +: GAIN_W]),
                .state      (st),
                .active     (voice_active[i])
            );

            // adsr.v 的状态编码：S_REL = 3'd4（见该文件 localparam）
            //   ⚠️ 用字面量 4 而不是引用 adsr 内部 localparam（它没导出）。
            //     adsr.v 一旦改状态编码，这里要同步；
            //     tb_voice_array 的场景 6 会把"松键后 releasing=1"当成反向校验。
            assign voice_releasing[i] = (st == 3'd4);
        end
    endgenerate

    // ============================================================
    //  3) 每声部音量：包络增益 >> voice_amp_shift（移位，0 个 DSP）
    // ============================================================
    wire [NV*GAIN_W-1:0] gain_scaled;

    //  淡出系数 atten ∈ [0, ATTEN_FULL]，目标是
    //      gain_scaled = (env_gain >> amp_shift) * atten / ATTEN_FULL
    //
    //  不用乘法器（会多出 32 个 DSP），而是用"一位一项的展开式"：
    //
    //      a = S a[i] · 2^i          ⇒   x·a = S a[i] · (x << i)
    //      gain = (x·a) >> ATTW       ⇒   = S a[i] · (x >> (ATTW - i))
    //
    //      恰好 ATTW 项、每项一个右移器 + 一个多路选择器，
    //      约 100 个 LUT，**0 个 DSP**。
    //
    //  ⚠️ 关键约束：a == ATTEN_FULL 时必须逐位恒等于"没有淡出通路"，
    //     否则正常演奏的音量也被改，06_keyboard 现有 468 项回归的波形就变了。
    //     手验（按上面的参数 ATTW=7 ⇒ ATTEN_FULL=64）：
    //         a = 64 = 7'b1000000 ⇒ x·64 >> 7 = x，一位不差。
    //     （这段注释上一版还停在 ATTW=5 / ATTEN_FULL=32 的旧参数，已同步。）
    //
    //  位宽：acc 最大 x·(2^ATTW - 1) = 65535×127 < 2^23 = GAIN_W+ATTW ✔
    //        实际只用到 a ≤ ATTEN_FULL = 64，余量更大。
    //  ⚠️ 2026-10-25 改成**平衡加法树**（数值逐位不变，只是换个加法顺序）：
    //     原来是 `acc = acc + (x<<i)` 的**串行**写法，综合出 **7 级加法链**；
    //     P&R 实测最差路径就落在这条链上（到达 30.9 ns / 要求 26 ns，
    //     slack −4.9 ns，Fmax 只到 40.2 MHz）。
    //     现在 7 个部分积先算出来，再按 4+2+1 的树相加（3 级）。
    //     整数加法可交换 ⇒ 结果与串行版**逐位相同**。
    //     ⚠️ 本树按 ATTW=7 写死；改 ATTW 要同步改这里（下面有仿真期自检）。
    function [GAIN_W-1:0] atten_scale;
        input [GAIN_W-1:0] x;
        input [ATTW-1:0]   a;
        integer i2;
        reg [GAIN_W+ATTW-1:0] p0,p1,p2,p3,p4,p5,p6;
        reg [GAIN_W+ATTW-1:0] s01,s23,s45,s0123;
        reg [GAIN_W+ATTW-1:0] acc;
        begin
            p0 = a[0] ? (x       ) : 0;
            p1 = a[1] ? (x << 1  ) : 0;
            p2 = a[2] ? (x << 2  ) : 0;
            p3 = a[3] ? (x << 3  ) : 0;
            p4 = a[4] ? (x << 4  ) : 0;
            p5 = a[5] ? (x << 5  ) : 0;
            p6 = a[6] ? (x << 6  ) : 0;
            s01   = p0 + p1;          // 1 级
            s23   = p2 + p3;
            s45   = p4 + p5;
            s0123 = s01 + s23;        // 2 级
            //  ★ 2026-11-01：归一化从 >>ATTW 改成 >>(ATTW-1)。
            //     ATTEN_FULL=64=2^6 而 ATTW=7 ⇒ 原来"满衰减"只有一半音量（少 6 dB）。
            //     现在满衰减 = 1.0 倍，并对结果饱和，防止 64~127 的淡入区撑出 16 位。
            acc = ((s0123 + s45) + p6) >> (ATTW - 1);
            atten_scale = (acc > {GAIN_W{1'b1}}) ? {GAIN_W{1'b1}} : acc[GAIN_W-1:0];
        end
    endfunction

    // ============================================================
    //  ★ 控制量本地锁存（2026-10-26，为时序）
    //
    //  病根：`voice_amp_shift` / `voice_atten` 来自 **param_xfer（另一个模块）**，
    //        它们跨模块的那段布线单独占 **7.7 ns**；再叠上"变量移位 + atten_scale 树"，
    //        路径到达 28.26 ns / 要求 25.9（slack −2.32），是当前最差。
    //        在终点后面加寄存器没用（只是挪了终点）；要在**目的地**把控制量先接住。
    //
    //  为什么在"帧中间"锁存，而不是在 tick 沿：
    //        在 tick 沿锁存会晚一帧（包络是当帧、atten 是上一帧），偷声切换那一帧
    //        的 atten==0 就会错位；而 param_xfer 的输出**整帧稳定**（它只在 tick 更新），
    //        所以帧中（tick 后 64 拍）采到的就是**本帧**该用的值，零相位误差。
    // ============================================================
    //  ⚠️ 取"tick 之后第 3 拍"，且计数器**饱和**不再加：
    //     仿真里 sample_tick 周期可能只有 10 拍（tb 为了跑得快），
    //     第一版写成"第 63 拍"⇒ 永远不触发 ⇒ atten/amp 恒 0 ⇒ 全部静音（tb 立刻抓到）。
    //     周期不足 4 拍时这一级不会更新（真实设计 1042~3125 拍，无此风险）。
    reg [6:0]          mid_cnt;
    wire               mid_lat = (mid_cnt == 7'd3);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                 mid_cnt <= 7'd0;
        else if (sample_tick)       mid_cnt <= 7'd0;
        else if (mid_cnt != 7'd63) mid_cnt <= mid_cnt + 7'd1;
    end

    reg [NV*AMPW-1:0]  amp_q;      // 本地锁存的 amp_shift
    reg [NV*ATTW-1:0]  att_q;      // 本地锁存的 atten

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            amp_q <= {(NV*AMPW){1'b0}};
            att_q <= {(NV*ATTW){1'b0}};
        end else if (mid_lat) begin
            amp_q <= voice_amp_shift;
            att_q <= voice_atten;
        end
    end

    // ============================================================
    //  ★ 增益链拆成两级（2026-10-26，第三轮削时序）
    //
    //  实测（第 6 轮 P&R）：`amp_q → gain_q` 的**数据路径 20.78 ns**，
    //  而中间只有"桶形移位 3 级 + 加法树 3 级"共 6 级 LUT ——
    //  说明 20.78 ns 里大头是**布线**（本设计只用 24% 资源，P&R 摊得很散）。
    //  只削逻辑没用，必须把这条链**切成两段**，让每段各自只走一半的线：
    //
    //      第 1 段（tick 沿）：sh_q   <= env_gain >> amp_q        桶形移位
    //      第 2 段（tick 沿）：gain_q <= atten_scale(sh_q, att_q)  加法树
    //      样本同步多推一拍：sample_q2 <= sample_q（保证三者同帧）
    //  ⇒ vca 的锁存再晚 1 拍（`start_lat` 2 级），整链 LATENCY 6 → **7 拍**
    //     （7 × 20 ns = 0.14 µs，对 10 ms 的预算毫无影响）
    // ============================================================
    wire [NV*GAIN_W-1:0] g_amp_sh;   // 第 1 段组合输出（移位）
    wire [NV*GAIN_W-1:0] g_amp_tree; // 第 2 段组合输出（树）
    reg signed [NV*OUT_W-1:0] sample_q2;

    reg signed [NV*OUT_W-1:0] sample_q;   // 样本第一级寄存（声明必须在使用前）
    reg [NV*GAIN_W-1:0] env_sh_q;    // 第 1 段寄存：移位后的包络
    reg [NV*GAIN_W-1:0] gain_q;      // 第 2 段寄存：乘过 atten 的增益

    //  两级都在 tick 沿推进（帧率，一帧 1042 拍，两级绰绰有余）
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            env_sh_q  <= {(NV*GAIN_W){1'b0}};
            gain_q    <= {(NV*GAIN_W){1'b0}};
            sample_q2 <= {(NV*OUT_W){1'b0}};
        end else if (sample_tick) begin
            env_sh_q  <= g_amp_sh;
            gain_q    <= g_amp_tree;
            sample_q2 <= sample_q;
        end
    end

    generate
        for (i = 0; i < NV; i = i + 1) begin : g_amp
            wire [GAIN_W-1:0] att = {att_q[i*ATTW +: ATTW]};
            wire [GAIN_W-1:0] sh  = env_gain[i*GAIN_W +: GAIN_W] >> amp_q[i*AMPW +: AMPW];
            //  第 1 段（本拍移位，tick 沿寄存）
            assign g_amp_sh[i*GAIN_W +: GAIN_W] = sh;
            //  第 2 段（树，组合；由下面的 tick 沿寄存）
            assign g_amp_tree[i*GAIN_W +: GAIN_W] = atten_scale(env_sh_q[i*GAIN_W +: GAIN_W], att);
        end
    endgenerate

    // ============================================================
    //  样本的第一级寄存（增益链的第 2 段已在上面寄存）
    //  样本与增益必须**同帧**到 vca，所以样本也推进一拍；
    //  增益的两级（env_sh_q / gain_q）与样本的一级（sample_q）在
    //  tick 沿一起推进，vca 再晚 2 拍锁存（见 vca.v start_lat）。
    // ============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) sample_q <= {(NV*OUT_W){1'b0}};
        else if (sample_tick) sample_q <= osc_raw;
    end

    // ============================================================
    //  4) 8 个 vca（每个 4 路时分复用）→ 32 路
    //     铁律③的 signed×unsigned 显式拓宽已经在 vca.v 内部做好，
    //     这里只负责把数据送进去。
    // ============================================================
    localparam integer NVCA = NV / VCA_NV;

    wire signed [NV*OUT_W-1:0] vca_out;
    wire [NVCA-1:0]            vca_valid;

    genvar j;
    generate
        for (j = 0; j < NVCA; j = j + 1) begin : g_vca
            vca #(
                .NV     (VCA_NV),
                .SW     (OUT_W),
                .GW     (GAIN_W),
                .LATENCY(VCA_NV + 3)
            ) u_vca (
                .clk        (clk),
                .rst_n      (rst_n),
                .sample_tick(sample_tick),
                .sample_in  (sample_q2 [(j*VCA_NV)*OUT_W  +: VCA_NV*OUT_W]),
                .gain_in    (gain_q    [(j*VCA_NV)*GAIN_W +: VCA_NV*GAIN_W]),
                .sample_out (vca_out   [(j*VCA_NV)*OUT_W  +: VCA_NV*OUT_W]),
                .out_valid  (vca_valid[j])
            );
        end
    endgenerate

    assign sample_out = vca_out;

    // 所有 vca 由同一个 sample_tick 驱动 ⇒ valid 天然同拍。
    // 这里仍然用"与"而不是 [0]，把"某个 vca 掉队"变成显式的 0 而不是幻觉。
    assign out_valid  = &vca_valid;

endmodule
