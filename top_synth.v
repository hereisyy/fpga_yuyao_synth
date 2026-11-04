`timescale 1ns/1ps
// ============================================================
//  top_synth.v —— 顶层（P0-7）：把所有线插上
//
//  本文件**不含新算法**，只有三件事：配置 + 连线 + 上电处理。
//
// ------------------------------------------------------------
//  一、接线图
// ------------------------------------------------------------
//   keyboard.v ─ freq_word ─→ osc_dds.v ─ sample_out ─┐
//      │            (31:0)      (gate 接 1'b1!)        ├→ vca.v → mixer_tree.v
//      │       └─ note_on ────→ adsr.v ─ gain ────────┘     → 软启动 → i2s_tx.v → PT8211
//      └─ sample_tick ──→ 上面每个模块都接（同一条 48 kHz 节拍）
//
//  ★ P0 只打通**一个声部**的通路（验收：仿真导出正弦 + 音准对）。
//    32 声部是 P1 `voice_array.v` 的事。这里 mixer_tree 仍然是 32 路结构，
//    只是第 0 路接发声、其余 31 路接 0 —— 这样上板验证的就是**最终结构**。
//
// ------------------------------------------------------------
//  二、★ 三条铁律（写错任何一条都出诡异问题）
// ------------------------------------------------------------
//  ① `osc_dds.gate` 接 **1'b1**，**绝不和 adsr.gate 接同一个信号**。
//       osc 的 gate=0 → 相位冻结 + 输出立刻清零；
//       adsr 的 gate=0 → 进入 RELEASE，用 314 ms 慢慢衰减。
//       若把同一个 note_on 同时接给两者，松键瞬间振荡器输出就变 0
//       → **释放尾音被吃掉** → ADSR 白做，听感上"松键即静音"。
//  ② `adsr.gate` 接 **`keyboard.note_on`（电平）**，不能接 `key_press`
//       那种单周期脉冲 —— ADSR 只在采样节拍上看一眼 gate，脉冲会被漏掉。
//  ③ 送进 `i2s_tx` 的数据来自**寄存输出**（本文件里的 att_r）。
//       它在 sample_tick 那拍锁存，抓到半更新的值就出咔哒声。
//
// ------------------------------------------------------------
//  三、上板专项
// ------------------------------------------------------------
//  · **`PA_EN` 接 `1'b0`** —— 板载功放使能是**低电平有效**！
//        写 1 = 关功放 = 没声音。这是"代码全对但就是没声音"的头号原因。
//  · **软启动斜坡**：上电后前 RAMP_MS 毫秒，增益从 0 升到 1，
//        不然上电瞬间会"砰"一声（直流冲击）。
//  · **按键极性**：板上 Key_in 是上拉 ⇒ **低有效**（按下 = 0），
//        而 keyboard.v 一律按**高有效**解释 ⇒ 顶层必须翻一次
//        （`KEY_ACTIVE_LOW=1`）。不翻的话上电时所有键都读成"按下"
//        → **一开机就一堆音**，而且看起来像"代码坏了"，极难往极性上想。
//  · **LED 低电平点亮**：要取反。
//  · **PLL 留了占位**：Gowin 的 rPLL 是 IDE 生成的 IP，不能手写。
//        现在先用 sys_clk 直通。上板前务必：
//        ① 用 IP Core Generator 生成 rPLL，把输出接到 clk_sys
//        ② **实测系统时钟频率**，把 SYS_CLK_HZ 改成真值
//           （这个参数错了整机音准就跑调，听起来像"琴坏了"）
//  · `$readmemh` 的路径：综合基准目录与仿真不同，找不到文件时
//        可能**静默填 0** → 上板全静音。综合后确认波表真的进去了。
//
//  验收：`tb_top_stream.v` 导出样本 → `analyze_top.py` 量音准
//  依赖：P0-1…P0-6 全部 ✅
// ============================================================

module top_synth #(
    // ⚠️ 上板实测后改成真值（跟踪表 0.4：系统时钟频率待实测）
    parameter SYS_CLK_HZ     = 100_000_000,
    parameter FS_HZ          = 48_000,
    parameter DEB_US         = 82,       // 按键去抖（µs）
    parameter RAMP_MS        = 10,       // 上电软启动时长（ms）
    // 板载按键上拉 ⇒ 低有效 ⇒ 1；自己画的键盘若高有效就填 0
    parameter KEY_ACTIVE_LOW = 1,
    parameter [1:0] WAVE_SEL = 2'd0,     // 单声部用的波形（0=正弦）
    // 仿真专用：1 = **不例化 i2s_tx**（上板/综合必须保持 0）
    //   理由见文件末尾 i2s_tx 例化处的注释。
    parameter SIM_SKIP_I2S   = 0
)(
    // ---- 系统 ----
    input  wire        sys_clk,       // V22（频率待实测）
    input  wire        sys_rst_n,     // AA13，⚠️ 1.5V，内部要上拉

    // ---- 演奏输入（原始电平，本模块负责翻成高有效）----
    input  wire [11:0] key_raw_pin,   // ⚠️ 上拉 ⇒ 低有效（按下 = 0）
    input  wire [2:0]  mod_raw_pin,   // 3 个档位键（0 档 = 无颤音/平直音）
    input  wire [2:0]  octave_sel,    // 八度：3'd2 → C4~B4

    // ---- 音频通路（PT8211）----
    output wire        PA_EN,         // AB16 ⚠️⚠️ 低有效：1'b0 = 开功放
    output wire        HP_BCK,        // Y17
    output wire        HP_WS,         // AB17
    output wire        HP_DIN,        // AA16

    // ---- 状态 ----
    output wire [7:0]  LEDs,          // ⚠️ 低电平点亮，已取反

    // ---- 仿真/调试观测孔（上板不用；综合器会把未使用的输出优化掉）----
    output wire signed [15:0] dbg_sample,
    output wire               dbg_valid,
    output wire [31:0]        dbg_freq_word,
    output wire [6:0]         dbg_note
);

    // ============================================================
    //  1) 时钟与复位
    //  ⚠️ PLL 占位：上板前用 Gowin IP Core Generator 生成 rPLL，
    //     把 clk_sys 改成它的输出，并把 pll_lock 并进复位。
    // ============================================================
    wire clk_sys = sys_clk;             // TODO(上板): 换成 PLL 输出
    wire rst_n   = sys_rst_n;           // TODO(上板): 加同步器 + pll_lock

    // ============================================================
    //  2) 按键极性翻转（⚠️ 漏了这一步，上电就一堆音）
    // ============================================================
    wire [11:0] kb_key = KEY_ACTIVE_LOW ? ~key_raw_pin : key_raw_pin;
    wire [2:0]  kb_mod = KEY_ACTIVE_LOW ? ~mod_raw_pin : mod_raw_pin;

    // ============================================================
    //  3) 演奏控制 → 频率字 + 包络电平
    // ============================================================
    wire [31:0] freq_word, freq_base;
    wire        freq_valid, sample_tick, note_on;
    wire [6:0]  note_num;
    // 下面这些本设计用不到，但必须接出来（不然就是悬空输出）
    wire [2:0]  octave_out_w;
    wire [3:0]  amp_shift_w, glide_shift_w;
    wire [11:0] key_press_w, key_release_w;
    wire [7:0]  lfo_val_w;
    wire [31:0] vib_dbg_w;

    keyboard #(
        .SYS_CLK_HZ (SYS_CLK_HZ),
        .FS_HZ      (FS_HZ),
        .DEB_US     (DEB_US)
    ) u_kb (
        .clk         (clk_sys),
        .rst_n       (rst_n),
        .key_raw     (kb_key),
        .mod_raw     (kb_mod),
        .octave_sel  (octave_sel),
        .freq_word   (freq_word),
        .freq_base   (freq_base),
        .freq_valid  (freq_valid),
        .sample_tick (sample_tick),      // ★ 全音频链共用这条 48 kHz 节拍
        .note_num    (note_num),
        .octave_out  (octave_out_w),
        .note_on     (note_on),
        .amp_shift   (amp_shift_w),
        .glide_shift (glide_shift_w),
        .key_press   (key_press_w),
        .key_release (key_release_w),
        .lfo_val     (lfo_val_w),
        .vib_debug   (vib_dbg_w)
    );

    // ============================================================
    //  4) 振荡器：gate 恒为 1（自由运行，音的有无交给包络）
    // ============================================================
    wire signed [15:0] osc_s;
    wire               osc_valid;

    osc_dds #(
        .PHASE_W       (32),
        .TABLE_AW      (10),
        .WAVE_W        (2),
        .BITS          (12),
        .FRAC_W        (18),
        .OUT_W         (16),
        .PHASE_MOD_EN  (0)
    ) u_osc (
        .clk          (clk_sys),
        .rst_n        (rst_n),
        .sample_tick  (sample_tick),
        .gate         (1'b1),            // ★★★ 铁律①：绝不能接 note_on
        .freq_word    (freq_word),
        .wave_sel     (WAVE_SEL),
        .phase_offset (32'd0),
        .sample_out   (osc_s),
        .sample_valid (osc_valid)
    );

    // ============================================================
    //  5) 包络：gate 是**电平**（note_on）
    // ============================================================
    wire [15:0] gain;
    wire [2:0]  adsr_state;
    wire        adsr_active;

    adsr #(
        .GAIN_W (16)
    ) u_adsr (
        .clk         (clk_sys),
        .rst_n       (rst_n),
        .sample_tick (sample_tick),
        .gate        (note_on),          // ★★★ 铁律②：电平，不是脉冲
        .gain        (gain),
        .state       (adsr_state),
        .active      (adsr_active)
    );

    // ============================================================
    //  6) VCA：波形 × 包络（1 个乘法器管 4 个声部，这里只用第 0 路）
    // ============================================================
    wire signed [4*16-1:0] vca_out;      // 4 路 × 16 位（只用到第 0 路，其余留 0）
    wire                     vca_valid;

    vca #(
        .NV      (4),
        .SW      (16),
        .GW      (16)
    ) u_vca (
        .clk         (clk_sys),
        .rst_n       (rst_n),
        .sample_tick (sample_tick),
        .sample_in   ({48'b0, osc_s}),    // 声部 0 = 本声部，1~3 = 0
        .gain_in     ({48'b0, gain}),
        .sample_out  (vca_out),
        .out_valid   (vca_valid)
    );

    // ============================================================
    //  7) 混音树：32 路结构（第 0~3 路接 VCA，其余接 0）
    // ============================================================
    wire signed [15:0] mix_out;
    wire               mix_valid;
    wire               sat_flag;

    mixer_tree #(
        .NV          (32),
        .SW          (16),
        .OUT_W       (16),
        .SCALE_SHIFT (0),
        .SAT         (1)
    ) u_mix (
        .clk         (clk_sys),
        .rst_n       (rst_n),
        .sample_tick (sample_tick),
        .sample_in   ({{(28*16){1'b0}}, vca_out}),
        .mix_out     (mix_out),
        .mix_valid   (mix_valid),
        .sat_flag    (sat_flag)
    );

    // ============================================================
    //  8) 软启动斜坡：上电前 RAMP_MS 毫秒，增益 0 → 1（防"砰"声）
    //     全程整数运算，**不做运行时除法**（参数在编译期算好）
    // ============================================================
    localparam integer RAMP_FRAMES = (RAMP_MS * FS_HZ) / 1000;   // 10 ms → 480 帧
    localparam integer RAMP_STEP   = 65536 / RAMP_FRAMES;        // 480 → 136

    reg [16:0] ramp_acc;

    always @(posedge clk_sys or negedge rst_n) begin
        if (!rst_n)                ramp_acc <= 17'd0;
        else if (mix_valid) begin
            if (ramp_acc < (17'd65536 - RAMP_STEP))
                ramp_acc <= ramp_acc + RAMP_STEP;
            else
                ramp_acc <= 17'd65536;      // 到顶就钉住（= 1.0）
        end
    end

    // ⚠️ signed(mix_out) × unsigned(ramp_acc) → 显式拓宽，照抄 vca.v 的写法
    wire signed [31:0] mx32 = $signed(mix_out);
    wire signed [31:0] rg32 = $signed({1'b0, ramp_acc});       // 17 位、恒非负
    wire signed [31:0] at32 = (mx32 * rg32) >>> 16;

    localparam signed [15:0] MAXS = 16'sd32767;
    localparam signed [15:0] MINS = -16'sd32768;

    wire signed [15:0] att = (at32 > MAXS) ? MAXS :
                             (at32 < MINS) ? MINS : at32[15:0];

    // ★ 铁律③：给 i2s_tx 的数据必须**寄存**
    reg signed [15:0] att_r;

    always @(posedge clk_sys or negedge rst_n) begin
        if (!rst_n) att_r <= 16'sd0;
        else        att_r <= att;
    end

    // ============================================================
    //  9) I2S 发送（PT8211）
    // ============================================================
    //  ⚠️ `SIM_SKIP_I2S` 只给仿真用，上板/综合必须保持 0。
    //    为什么需要它：i2s_tx 里有一条仿真自检 ——
    //        BCK 每半周期不足 2 拍（即 SYS_CLK_HZ < 48000×64×2 = 6.144 MHz）时直接 $finish。
    //        （每帧 64 个半周期 —— 见 i2s_tx.v 的 HALF_PER_FRAME。
    //         这里原写 ×128，是按旧的"每帧 64 个 BCK"算的，重写后每帧 32 个 BCK，
    //         半周期数减半，已更正。）
    //    它把仿真时钟逼到 6.144 MHz 以上，于是"导出 4096 点做音准分析"
    //    要跑上千万拍，慢到没法用。
    //    跳过 i2s_tx 后，仿真时钟可以缩到 ×16，整条流程 ~60 万拍。
    //    音频数据通路完全不受影响（音准只取决于 sample_tick 的间隔，
    //    与时钟的绝对频率无关）；I2S 串行化本身由 tb_i2s / tb_params 验证。
    generate
        if (SIM_SKIP_I2S) begin : g_no_i2s
            assign HP_BCK = 1'b0;
            assign HP_WS  = 1'b0;
            assign HP_DIN = 1'b0;
        end else begin : g_i2s
            i2s_tx #(
                .SYS_CLK_HZ (SYS_CLK_HZ),
                .FS_HZ      (FS_HZ),
                .DATA_W     (16)
            ) u_i2s (
                .clk         (clk_sys),
                .rst_n       (rst_n),
                .sample_tick (sample_tick),
                .data_l      (att_r),
                .data_r      (att_r),            // 单声道 → 两声道都送同一份
                .bck         (HP_BCK),
                .ws          (HP_WS),
                .sd          (HP_DIN)
            );
        end
    endgenerate

    // ============================================================
    //  10) 输出引脚
    // ============================================================
    assign PA_EN = 1'b0;                 // ★★★ 低有效：0 = 开功放，别改成 1

    // LED：⚠️ 低电平点亮 → 取反。这里用几个状态做指示，方便上板一眼看
    assign LEDs = ~{4'b0,
                    adsr_active,         // LED3：正在发声
                    sat_flag,            // LED2：混音饱和（削顶）
                    note_on,             // LED1：有键按下
                    freq_valid};         // LED0：频率字有效

    assign dbg_sample    = att_r;
    assign dbg_valid     = mix_valid;
    assign dbg_freq_word = freq_word;
    assign dbg_note      = note_num;

endmodule
