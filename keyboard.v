// ============================================================
//  键盘控制器（顶层）
//
//  职责边界（很重要，避免以后越写越乱）：
//    本模块**只管演奏控制**，不产生声音。
//    输出的是"当前该发什么频率、相位走到哪、颤音加了没"，
//    真正的波形生成由后面的振荡器 + 混音 + I2S 负责。
//
//  两个独立可控维度：
//    维度 1 —— 音高：12 个弹音键 × 八度选择
//    维度 2 —— 调制档位：3 个档位键，控制颤音深度与滑音速度
//
//  相比旧版修掉的问题（都不是笔误，是结构性问题）：
//    ① SAMPLE_DIV 写死 2083，只对 100 MHz 有效
//       -> 改成由 SYS_CLK_HZ 参数自动算
//    ② 松键时 note_num 掉回 0，恰好等于 C4 的合法音符号，
//       导致滑音目标被拉向 C4、轨迹被打断
//       -> 改成"松键时保持上一个有效频率字"
//    ③ glide_shift = 5/8（τ = 0.67/5.33 ms），快到听不出滑音
//       -> 改成 6~13，默认档给 12（τ = 85 ms）
//    ④ vib_shift 最大 15 时相位偏移超出奈奎斯特，混叠成金属噪声，
//       而且默认档 vib_shift=0 等于把颤音关掉
//       -> 上限收到 13，默认档给轻颤音；并改成按频率缩放的调频
//    ⑤ 没有按键同步器（异步输入直接进状态机）
//       -> key_encoder 里做了两级同步
//    ⑥ 没有八度切换
//       -> 加 octave_sel，由 12 个音符号合成 MIDI 音符号后查全音域表
//
//  频率表：freq_table.v（由 gen_freq_table.py 生成，不要手工改）
//          采样率变化时重新生成即可。
//
//  ⚠️ 采样节拍：本模块内部产生 sample_tick（48 kHz 单周期脉冲）。
//     音频链路上的其他模块应当共用同一个 sample_tick（从本模块引出），
//     不要再各写一个分频器，否则会出现拍频。
// ============================================================
module keyboard #(
    // ⚠️ 上板实测后改成 50_000_000 或 100_000_000（见 分工_1_硬件 的 1.7 节）
    parameter SYS_CLK_HZ = 100_000_000,
    // 目标采样率
    parameter FS_HZ      = 48_000,
    // 按键去抖时间（微秒）。82 us 足够滤掉抖动，手感比 655 us 快得多
    parameter DEB_US     = 82,
    // 颤音最低工作频率（Hz）—— ★A0 之后**已废弃，仅作历史记录**。
    //   它原来用来给颤音幅度做下限保护（见下面"删掉一个下限保护"那段），
    //   实测证明那个下限会让 C2 的相对颤音深度变成 C5 的 1.7 倍，
    //   与"调频"的目标（全音区深度一致）直接冲突，所以整段删掉了。
    //   参数保留是为了不让例化它的代码（tb_keyboard.v 等）报端口/参数错，
    //   新代码**不要**再引用它。
    parameter VIB_MIN_HZ = 8_000,
    // LFO（颤音摆动）频率，Hz。音乐上常用 4~6 Hz。
    // 仿真时可调大（例如 2000），这样几个时钟就能观察到一个完整摆动周期。
    parameter LFO_FREQ   = 5
)(
    input  wire        clk,
    input  wire        rst_n,

    // ---- 演奏输入 ----
    //
    //  ⚠️⚠️ **极性约定（全工程最容易接反的地方，务必看清）**
    //
    //    本模块的 key_raw / mod_raw 一律按 **高有效** 解释：
    //        1 = 按下，0 = 松开
    //    内部的 debounce 是"跟随输入电平"的，**不做任何反相**，
    //    key_encoder 也直接按高有效用。
    //
    //    但板载那两个用户按键（`Key_in[0]`=AB13 / `Key_in[1]`=Y12）在官方
    //    `.cst` 里是 `PULL_MODE=UP` —— **上拉 ⇒ 低有效**（按下 = 0）。
    //    自己画的键盘 PCB 用什么极性由你决定（上拉+接地 = 低有效）。
    //
    //    ⇒ **顶层必须把板上的电平翻成"高有效"再送进来**，例如
    //         assign key_raw = ~board_keys;
    //      不翻的话上电时所有键都读成"按下" → **一开机就是一堆音**，
    //      而且看起来像"代码坏了"，极难往极性上想。
    //      （全工程只有 LED 低有效和 PA_EN 低有效写了文档，按键极性以前一个字都没有。）
    input  wire [11:0] key_raw,     // 12 个弹音键（原始、会抖）**高有效：1=按下**
                                    //   key[0]=最低音(C)，key[11]=最高音(B)
    input  wire [2:0]  mod_raw,     // 3 个档位键（原始、会抖）**高有效：1=按下**
    input  wire [2:0]  octave_sel,  // 八度选择。默认建议给 3'd2 -> C4~B4
                                    //   （同样按高有效/直接二进制解释）

    // ---- 送给发声引擎（振荡器）----
    //
    //  ★ 输出的是**频率字**，不是相位。
    //
    //  为什么这样设计（A0 改动，2026-09-30）：
    //    旧版在 keyboard.v 里放**唯一的**相位累加器，输出 phase_out。
    //    但加法合成需要 8 个不同步的相位（f, 2f, 3f…），
    //    FM 需要载波和调制器两个相位 —— 共享一个相位在结构上就做不到。
    //
    //    改成输出频率字后，**相位累加器下沉到每个振荡器实例里**：
    //      · 波表声部：1 个累加器
    //      · 加法声部：N 个累加器（每分量一个）
    //      · FM 声部：2 个累加器（载波 + 调制器）
    //    这正好符合赛题对"独立发声单元"的定义（独立相位累加器 +
    //    频率调控 + 包络控制），所以每个分量/算子都能合法计入振荡器数。
    //
    //    代价：振荡器模块要自己维护累加器（增加约 5 行），
    //    收益：加法/FM 结构上可变现，振荡器计数从 32 涨到数百。
    output wire [31:0] freq_word,   // 口频率字：已含滑音 + 颤音
    output wire [31:0] freq_base,   // 滑音后的基频字（不含颤音）——
                                    //   加法合成算各分量倍频要用它
    output wire        freq_valid,  // 以上频率字是否有效（= 有键按下）
    output wire        sample_tick, // 48 kHz 采样节拍，单周期脉冲
                                    //   音频链路上的其他模块共用这个信号

    // ---- 状态输出 ----
    output wire [6:0]  note_num,    // 当前 MIDI 音符号（含八度）
    output wire [2:0]  octave_out,  // 实际生效的八度（调试用）
    output wire        note_on,     // 是否有键按下
    output wire [3:0]  amp_shift,   // 颤音深度档（0=无颤音）
    output wire [3:0]  glide_shift, // 滑音时间常数档（越大越慢）

    // ---- 事件输出（给声部分配器用，复音链路）----
    output wire [11:0] key_press,   // 按键新按下，单周期脉冲
    output wire [11:0] key_release, // 按键新松开，单周期脉冲

    // ---- 调试 ----
    output wire [7:0]  lfo_val,     // LFO 当前值
    output wire [31:0] vib_debug    // 颤音**频率**偏移量（有符号）
                                    //   注意：改版前这里是"相位偏移量"，
                                    //   A0 之后语义变成频率偏移，抓波形时别搞混
);

    // ============================================================
    //  0) 参数派生
    // ============================================================

    // 采样分频比 = 系统时钟 / 采样率（四舍五入）
    //   ⚠️ 量纲很容易写错。正确写法是两边都先除以 1000，再相除，最后四舍五入：
    //        SAMPLE_DIV = (SYS_CLK_HZ + FS_HZ/2) / FS_HZ      ← 最直观，但整数除法
    //        ↓ 换成不会截断的等价形式：
    //        SAMPLE_DIV = ((SYS_CLK_HZ/1000) + (FS_HZ/2000)) / (FS_HZ/1000)
    //      加 FS_HZ/2000 就是"半个采样率"，即四舍五入到最近的整数分频比。
    //
    //   ⚠️ 曾经写错过一次：SYS_CLK_HZ / (FS_HZ/1000)
    //      分子没同时缩小 1000 倍 -> 结果大 1000 倍 -> 采样率变成 48 Hz。
    //      更坑的是仿真里时钟按同样比例缩小，两个错误互相抵消，
    //      测试"全部通过"但硬件是坏的。所以这里的分母必须和测试台完全一致。
    //
    //   ⚠️ 也不能只写 (SYS_CLK_HZ + FS_HZ/2) / FS_HZ：
    //      当 SYS_CLK_HZ < FS_HZ（仿真加速时）会截断成 0，tick 永不产生。
    //
    //   真实场景：100 MHz -> 2083（实际 48000.0 Hz，零误差）✔
    //             50 MHz -> 1042（实际 47984.6 Hz，-0.55 音分）✔
    //   仿真场景：960 kHz -> 20（比例关系与真实一致）；96 kHz -> 2
    localparam SAMPLE_DIV = ((SYS_CLK_HZ / 1000) + (FS_HZ / 2000)) / (FS_HZ / 1000);

    // 去抖计数上限 = 去抖微秒数 * 每微秒时钟数
    //
    //  ⚠️ 两个陷阱都踩过，现在的写法同时躲开：
    //
    //  ① 不要写成 SYS_CLK_HZ / 1_000_000：那是整数除法，
    //     在低频（如仿真用的 960 Hz）下会得到 0，CNT_MAX 变成 -1，
    //     去抖计数器永远到不了，按键就永远确认不了。
    //
    //  ② **先除后乘**，别写成 (DEB_US * SYS_CLK_HZ) / 1_000_000：
    //     那个中间乘积是 82 × 1e8 = **8.2e9，超出 32 位有符号范围**
    //     （上限 2.147e9）。Icarus 做常量折叠时用了更宽的位宽所以算对了，
    //     但**综合工具不保证**：若 Gowin 按 32 位折叠，结果是负数 →
    //     CNT_MAX 变负 → 去抖永远不确认 → **整机完全没有声音**。
    //     这类"仿真对、综合错"的差异是最难查的，所以直接在写法上避开。
    //
    //  两种写法结果完全一致（都验证过）：
    //      100 MHz : 8199      960 kHz : 77      96 kHz : 6
    localparam CNT_MAX = (DEB_US * (SYS_CLK_HZ / 1_000)) / 1_000 - 1;
    localparam DEB_W   = 16;    // 计数位宽，16 位够到 65535 个时钟

    // 八度输入饱和（详细的映射关系见第 4 节）：
    //   octave_sel > 3 会被钳到 3，避免跑到高频区产生混叠。
    wire [2:0] oct_sel = (octave_sel > 3'd3) ? 3'd3 : octave_sel;
    assign octave_out = oct_sel;

    // ============================================================
    //  1) 采样节拍：48 kHz 单周期脉冲
    //
    //  ★ 只用"计数器 + enable"，绝对不要用门控时钟
    //    （always @(posedge 分频出来的信号) 会让时序收敛炸掉）
    // ============================================================
    reg [11:0] tick_cnt;
    wire       tick = (tick_cnt == SAMPLE_DIV[11:0] - 12'd1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)              tick_cnt <= 12'd0;
        else if (tick)           tick_cnt <= 12'd0;
        else                     tick_cnt <= tick_cnt + 12'd1;
    end

    assign sample_tick = tick;

    // ============================================================
    //  2) 按键去抖
    //     12 个弹音键 + 3 个档位键
    // ============================================================
    wire [11:0] key;        // 去抖后的弹音键
    wire [2:0]  mod_key;    // 去抖后的档位键

    genvar gi;
    generate
        for (gi = 0; gi < 12; gi = gi + 1) begin : g_key
            debounce #(.CNT_WIDTH(DEB_W), .CNT_MAX(CNT_MAX)) u_deb_key (
                .clk(clk), .rst_n(rst_n),
                .key_in(key_raw[gi]), .key_out(key[gi])
            );
        end
        for (gi = 0; gi < 3; gi = gi + 1) begin : g_mod
            debounce #(.CNT_WIDTH(DEB_W), .CNT_MAX(CNT_MAX)) u_deb_mod (
                .clk(clk), .rst_n(rst_n),
                .key_in(mod_raw[gi]), .key_out(mod_key[gi])
            );
        end
    endgenerate

    // ============================================================
    //  3) 键盘编码：位图 + 边沿 + 最高音优先编码
    //     复音链路用 key_state / key_press / key_release；
    //     本模块内部先用最高音优先编码，保证单音也能正常工作。
    // ============================================================
    wire [11:0] key_state;
    wire [3:0]  key_idx;        // 最高音的键号 0~11
    wire        key_valid;
    wire        active, active_press, active_release;

    key_encoder #(
        .SYNC_STAGES(2),
        .REVERSE_BIT_ORDER(0)
    ) u_enc (
        .clk          (clk),
        .rst_n        (rst_n),
        .key          (key),
        .key_state    (key_state),
        .key_press    (key_press),
        .key_release  (key_release),
        .active       (active),
        .active_press (active_press),
        .active_release(active_release),
        .note_num     (key_idx),
        .note_valid   (key_valid)
    );

    assign note_on = key_valid;

    // ============================================================
    //  4) 键内音符号 + 八度 -> MIDI 音符号 -> 频率字
    //
    //  映射关系（design 目标：默认档就是 C4~B4，最符合"电子琴"直觉）：
    //    midi = 12 * (oct_sel + 3) + key_idx
    //      oct_sel = 0 -> MIDI 36 = C2 ~ B2   （低音区）
    //      oct_sel = 1 -> MIDI 48 = C3 ~ B3
    //      oct_sel = 2 -> MIDI 60 = C4 ~ B4   ← 默认档
    //      oct_sel = 3 -> MIDI 72 = C5 ~ B5   （高音区）
    //    覆盖 C2~B5，共 4 个八度、48 个音，对电子乐器足够。
    //
    //  实现用移位加，不用 `*`（避免综合出一个多余的乘法器）：
    //    12*(n+3) = (n+3)*8 + (n+3)*4 = ((n+3)<<3) + ((n+3)<<2)
    //    {n3,3'b000} = n3*8，{n3,2'b00} = n3*4，两式相加即得。
    // ============================================================
    wire [2:0] oct_p3   = oct_sel + 3'd3;                 // n+3，范围 3~6
    wire [6:0] oct_x12  = {oct_p3, 3'b000} + {oct_p3, 2'b00};   // = (n+3)*8 + (n+3)*4
    wire [6:0] midi_calc = oct_x12 + {4'b0000, key_idx};

    assign note_num = midi_calc;

    wire [31:0] freq_target;
    freq_table u_ft (
        .note (midi_calc),
        .freq (freq_target)
    );

    // ============================================================
    //  5) 调制档位 -> 颤音深度 / 滑音速度
    //
    //     重新标定过（旧值的问题见文件头 ③④）：
    //
    //     档位键      听感                 颤音      滑音
    //     ---------------------------------------------------------
    //     000       平直音（默认）         无        τ≈85ms
    //     001       轻颤音                ±10 音分    τ≈85ms
    //     010       中颤音                ±20 音分    τ≈85ms
    //     011       深颤音                ±40 音分    τ≈85ms
    //     100       慢滑音（无颤音）        无        τ≈170ms
    //     101       快滑音（无颤音）        无        τ≈21ms
    //     110       颤音 + 慢滑音           ±20 音分    τ≈170ms
    //     111       颤音 + 快滑音           ±20 音分    τ≈21ms
    //
    //     amp_shift 的含义：颤音相对频率的 2^-amp_shift 倍
    //       amp_shift=7  -> 1/128  -> @A4 约 ±3.4 Hz ≈ ±13 音分
    //       amp_shift=6  -> 1/64   -> @A4 约 ±6.9 Hz ≈ ±27 音分
    //       amp_shift=5  -> 1/32   -> @A4 约 ±13.8 Hz ≈ ±53 音分
    //       amp_shift=4  -> 1/16   -> @A4 约 ±27.5 Hz ≈ ±105 音分（已经很夸张）
    //     amp_shift=0 表示关闭颤音
    // ============================================================
    reg [3:0] amp_shift_r;
    reg [3:0] glide_shift_r;

    always @(*) begin
        case (mod_key)
            3'b000: begin amp_shift_r = 4'd0; glide_shift_r = 4'd12; end // 平直音
            3'b001: begin amp_shift_r = 4'd7; glide_shift_r = 4'd12; end // 轻颤音
            3'b010: begin amp_shift_r = 4'd6; glide_shift_r = 4'd12; end // 中颤音
            3'b011: begin amp_shift_r = 4'd5; glide_shift_r = 4'd12; end // 深颤音
            3'b100: begin amp_shift_r = 4'd0; glide_shift_r = 4'd13; end // 慢滑音
            3'b101: begin amp_shift_r = 4'd0; glide_shift_r = 4'd10; end // 快滑音
            3'b110: begin amp_shift_r = 4'd6; glide_shift_r = 4'd13; end // 颤音+慢滑音
            3'b111: begin amp_shift_r = 4'd6; glide_shift_r = 4'd10; end // 颤音+快滑音
            default: begin amp_shift_r = 4'd0; glide_shift_r = 4'd12; end
        endcase
    end

    assign amp_shift   = amp_shift_r;
    assign glide_shift = glide_shift_r;

    // 有颤音时，滑音目标要等滑音结束才换音色 -> 简化处理：
    // 颤音始终开启（只要 amp_shift_r != 0），滑音只在按键变化时起作用
    wire vib_on = (amp_shift_r != 4'd0);

    // ============================================================
    //  6) LFO：颤音的摆动源（5 Hz）
    //     复用 LFO 自己的正弦表（sine_table.hex，256 项 8 位）
    //     ⚠️ 这张表只给 LFO 用，不要和音频波表（wavetable_*.hex）混用：
    //        那张是 1024×12bit 给振荡器查表用的，用途和精度要求完全不同。
    // ============================================================
    wire [31:0] lfo_phase;

    lfo #(
        .FS_HZ(FS_HZ),
        .LFO_FREQ(LFO_FREQ)
    ) u_lfo (
        .clk       (clk),
        .rst_n     (rst_n),
        .tick      (tick),
        .lfo_phase (lfo_phase)
    );

    reg [7:0] sine_table [0:255];

    initial begin
        $readmemh("sine_table.hex", sine_table);
    end

    wire [7:0] lfo_raw = sine_table[lfo_phase[31 -: 8]];
    assign lfo_val = lfo_raw;

    // 有符号化：把偏移二进制 0~255 变成 -128 ~ +127
    //   sine_table.hex 已核验是标准偏移二进制表（中心 128，与理想
    //   正弦最大偏差 0.5 LSB），所以减 128 后就是干净的双极性正弦。
    wire signed [9:0] lfo_s = $signed({2'b00, lfo_raw}) - 10'sd128;

    // ============================================================
    //  7) 颤音：**调频**（不是调相）
    //
    //  ★ A0 的一个实质性修改（不只是接口改名）：颤音从"调相"改成"调频"。
    //
    //  旧做法（调相）：phase_out = phase_acc + lfo × (freq_cur >> amp_shift)
    //    这时相位偏移量正比于 freq_cur，而相位偏移的**导数**才是频率。
    //    结果是：调制深度（音分数）随音高变化 —— 高音颤得少、低音颤得多。
    //    《四个问题原因分析.md》实测过：@A4 ±90 音分 vs @C4 ±149 音分，
    //    差 1.65 倍，音乐上不自然。
    //
    //  新做法（调频）：freq_word = freq_cur + lfo × (freq_cur >> amp_shift)
    //    相对频偏恒定 = 2^-amp_shift，所以**各音区的颤音音分数一致**：
    //      amp_shift=7 → 1/128 → ±13.5 音分（轻颤音）
    //      amp_shift=6 → 1/64  → ±27 音分（中颤音）
    //      amp_shift=5 → 1/32  → ±54 音分（深颤音）
    //    这才是真正的颤音，也是所有合成器的标准做法。
    //
    //  副作用：频率每秒跳变 48k 次，波形上是"逐样本的阶梯"而不是连续摆动。
    //    但相对跳变率 = 2π×f/fs ≈ 0.4%（A4 时），量化到相位上是渐进的，
    //    听感上就是正常颤音。相位连续性由振荡器里那个累加器保证。
    // ============================================================

    // ---- 滑音：当前频率字朝目标指数逼近（放在颤音之前）----
    reg [31:0] freq_cur;
    reg [31:0] freq_hold;       // 松键时保持的频率字 —— 修掉"松键跳回 C4"

    // 滑音目标：有键时是当前音，无键时是**上一次按下的音**（freq_hold）。
    //   两个分支合成一条通路，避免"步长用的是另一个差值"这类量纲错。
    wire [31:0] glide_tgt  = key_valid ? freq_target : freq_hold;

    wire [31:0] freq_diff = (glide_tgt > freq_cur)
                          ? (glide_tgt - freq_cur)
                          : (freq_cur - glide_tgt);

    // 每次滑音的步长。单独提出来是为了**判断"该收尾了"**：
    //   freq_diff 右移 glide_shift 位后，只要 |freq_diff| < 2^glide_shift，
    //   步长就是 0 —— 原来的代码这时会**永远停在离目标一步之遥的地方**
    //   （指数逼近的经典毛病：无限接近但永不相等）。
    //   听感上无所谓（差 0.03 音分），但它有两个实际害处：
    //     ① 测试里 "频率字 == 期望值" 永远不成立，只能写模糊容差；
    //     ② 切八度后基频差 1/4096，叠加颤音时数值上不好推理。
    //   所以补一条：步长为 0 时直接跳到目标。这不会产生爆音 ——
    //   跳变量 < 2^glide_shift ≤ 8192（相对 5.8 MHz 是 0.14%）。
    wire [31:0] glide_step = freq_diff >> glide_shift_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // 复位时停在 C4（MIDI 60）。具体数值由 freq_table 决定，
            // 这里用一个保守的中间值，避免复位瞬间频率为 0 造成爆音。
            freq_cur  <= 32'd23409859;      // C4 @48kHz
            freq_hold <= 32'd23409859;
        end else if (tick) begin
            // ---- freq_hold 只在有键时更新：记住最后按下的音 ----
            //   这是旧版最隐蔽的 bug：无键时 note_num=0 恰好是 C4，
            //   于是滑音目标被拉向 C4，把滑音轨迹打断。
            freq_hold <= glide_tgt;

            // ---- 一条通路完成指数逼近 + 收尾 ----
            if (glide_tgt > freq_cur)
                freq_cur <= (glide_step == 32'd0) ? glide_tgt
                                                  : (freq_cur + glide_step);
            else if (glide_tgt < freq_cur)
                freq_cur <= (glide_step == 32'd0) ? glide_tgt
                                                  : (freq_cur - glide_step);
            else
                freq_cur <= glide_tgt;
        end
    end

    // ---- 颤音幅度：相对频偏 = 2^-amp_shift，正比于 freq_cur ----
    //
    //  ★★ 这里删掉了一个"下限保护"，它是 A0 最后一个 bug（实测数据在下面）。
    //
    //  原来写的是：
    //      VIB_FLOOR_WORD = (VIB_MIN_HZ<<16)/(FS_HZ/1000) = 10,922,666
    //      vib_src = (freq_cur < VIB_FLOOR_WORD) ? VIB_FLOOR_WORD : freq_cur
    //  本意是"低音区 freq_cur 太小，右移后可能变 0，颤音会消失"。
    //
    //  但代码里偏移是
    //      offset = (lfo_s × (freq_cur >> amp_shift)) >>> 7
    //  它**本来就正比于 freq_cur**，低音区只会等比例变小，不会消失：
    //  最低音 C1 的 freq_cur = 2,926,232，amp_shift 最大 7 时
    //  freq_cur>>7 = 22,861，离 0 差得远，**根本不需要下限**。
    //
    //  而这个下限恰好卡在 C2 和 C3 之间（10,922,666），于是：
    //      C2  freq_cur = 5,852,465  < 下限  → 按 10,922,666 算，被放大 1.866×
    //      C5  freq_cur = 46,819,719 > 下限  → 正常
    //  实测（tb_keyboard 场景 7，amp_shift=5）：
    //      C2 相对频偏 53,878 ppm   ← 应为 31,250
    //      C5 相对频偏 31,460 ppm   ← 正确
    //  即"调频改对了，但低音区又冒出个 1.7 倍"。
    //  这正是 A0 要修的病（各音区深度不一致），必须连根拔掉。
    //
    //  现在直接用 freq_cur：相对频偏严格 = 2^-amp_shift，全音区一致。
    wire [31:0] vib_src = freq_cur;

    // ============================================================
    //  频率偏移量的算法（这一段我改了四版才对，把量纲和位宽写清楚）
    //
    //  目标：偏移 = lfo/128 × (freq_cur >> amp_shift)
    //       即 ±相对频偏，相对频偏 = 2^-amp_shift
    //
    //  ⚠️ 踩过的坑（三个，都记下来免得重犯）：
    //    ① 写成 (lfo × vib_amp) << 16 且 vib_amp = freq[31:16]>>shift：
    //       最大到 3.4e10，**超出 32 位有符号范围**（2.1e9）→ 溢出成负数
    //    ② 想用拼接 {prod[15:0], 16'b0} 绕开溢出：那是把结果乘了 65536，
    //       量纲又错了 —— 低音区偏移变成应有值的 8 倍
    //    ③ **最隐蔽的一个**：`freq_s >> amp_shift_r` 里的 >> 是**算术右移**，
    //       而 Verilog 用"操作数中较宽的那个"来决定位宽，结果被截成 32 位：
    //       右移后高 4 位补 0，32 位放不下，**低位被挤掉** →
    //       等效于 freq × 2^(32-shift) 溢出 → 频率字恰好翻倍。
    //       修法：把操作数**显式左对齐到 48 位**再去乘。
    //
    //  正确写法：把两个操作数都显式做成 48 位有符号，一次乘完再右移
    //      offset = (lfo_s48 × freq48) >>> 7      （>>>7 完成"除以 128"）
    //    位宽核对：
    //      lfo_s48  ∈ [-128, 127]   （48 位有符号）
    //      freq48   = freq_cur >> amp_shift，≤ 2^31>>4 = 2^27
    //      乘积 ≤ 2^7 × 2^27 = 2^34  -> 48 位装得下 ✔
    //      右移 7 位后 ≤ 2^27，放回 32 位绰绰有余 ✔
    //    比例核对：偏移/freq_cur = ±127/128 × 2^-shift ≈ 2^-shift ✔
    //      amp_shift=5 时相对频偏 ≈ 1/32 = 31250 ppm
    // ============================================================
    wire signed [47:0] vib_freq48 = $signed({16'd0, (vib_src >> amp_shift_r)});
    //   {16'd0, X} 把 32 位 X 左对齐到 48 位，再声明为有符号。
    //   注意：这里必须先拼接再加宽，不能写成 $signed(vib_src) >> shift，
    //   否则又回到坑 ③。

    wire signed [47:0] vib_lfo48 = $signed({{38{lfo_s[9]}}, lfo_s[9:0]});

    // 无颤音（amp_shift=0）时必须输出 0，所以要门控
    wire signed [47:0] vib_off_wide = vib_on
                                    ? (vib_lfo48 * vib_freq48)
                                    : 48'sd0;

    wire signed [31:0] vib_offset = vib_off_wide[38:7];
    //   [38:7] 一次完成"除以 128"（右移 7）和"截到 32 位"：
    //   结果绝对值 ≤ 2^27，落在 [38:7] 的有效范围内，不丢信息。

    assign vib_debug = vib_offset;

    // ============================================================
    //  8) 输出：频率字 = 滑音后的基频 + 颤音偏移
    //
    //  ⚠️ 相位累加器**已从这里搬走**，移到各振荡器实例内部。
    //     本模块只负责"算出该用多大频率"，不负责推进相位。
    // ============================================================
    assign freq_base  = freq_cur;                                  // 不含颤音
    assign freq_word  = freq_base + vib_offset[31:0];              // 含颤音
    assign freq_valid = key_valid;

endmodule
