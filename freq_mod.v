// ============================================================
//  freq_mod.v —— 全局 LFO 颤音（P1 只做颤音，滑音推迟 P3）
//
//  ★ 设计冻结口径（跟踪表 P1 冻结表）
//      "颤音 / 滑音：P1 **只做全局 LFO 颤音**；滑音推迟 P3"
//    ⇒ 本模块就是那一个**全局** LFO：所有声部共用同一条调制波，
//      同相、同深、同速。不是每声部一个 LFO（那要 32 份相位累加器）。
//      共用是**规格要求**，不是省资源 —— 合唱式的"每个声部各自快慢"
//      不属于本阶段。
//
//  ★ 颤音怎么作用到频率上（关键，别写错）
//    频率字 fw 是 32 位相位增量：Δφ = fw，输出频率 = fw·fs/2^32。
//    颤音是**频率的百分比**，不是加一个固定 Hz：
//        fw' = fw + (fw >> d) · lfo        d = vib_depth
//    ⇒ 相对偏移 = lfo / 2^d。d=7 时约 ±0.78%，比音乐里的 ±20 音分
//      （±1.16%）略浅，是刻意选的：再深容易听成走音。
//
//    ⚠️ 不能用 fw' = fw + (lfo >> d) —— 那是"加固定 Hz"，
//       低频音会颤得夸张、高频音几乎听不出来（错得很典型）。
//       必须让调制量**跟着 fw 走**，才是等音分的颤音。
//
//  ★ LFO 波形：三角波，不是正弦
//    三角波只要一个加减计数器，0 个乘法器、0 块 BSRAM；
//    而正弦要再占一份波表。颤音听感上三角与正弦几乎无差
//    （调制指数很浅，谐波差被淹没）⇒ 用三角换取资源。
//
//  ★ 位移换算：为什么用 `>> d` 而不是乘
//     签名的 lfo 值 ∈ [-(2^LFO_W-1), +(2^LFO_W-1)]，配 `>> d` 就是
//     "以 2 的幂为单位的百分比"。P1 的档位表（key_scan 的 amp_shift）
//     本来就是 0/5/6/7 这几档，跟这里天然对齐 —— 0 个 DSP。
//
//  ★ 深度 = 0 时必须**完全不动**
//    深度 0 时若仍走乘法/移位，lfo 的噪声会漏进频率字。
//    这里直接旁路：`fw_out = fw_in`，逐位等于输入（tb 逐位比对）。
//
//  ★ 延迟
//    LATENCY = **2 拍**（freq_r 与 freq_out 两级寄存器）。
//    寄存是必须的：fw_in 来自 param_xfer 的输出，若本模块组合相加，
//    就会把乘法器和加法器串进"帧边界提交"那条路径，破坏时序收敛。
//    两级而不是一级：第一级吸收乘法+加法的长路径，第二级把结果与
//    `freq_out` 这个对外端口隔开（端口前必须有寄存器，否则布线绕远）。
//    tb 里按 2 拍断言。
// ============================================================
module freq_mod #(
    parameter integer FW_W    = 32,   // 频率字位宽
    parameter integer LFO_W   = 16,   // LFO 相位累加器高位宽度
    parameter integer DEPTH_W = 3,    // 颤音深度位宽（与 key_scan.amp_shift 同宽）
    parameter integer LATENCY = 2,   // freq_in -> freq_out 的拍数
    //  LFO 每帧的相位步进：LFO 频率 = FS × LFO_STEP / 2^LFO_W
    //   默认 LFO_W=16、LFO_STEP=7 ⇒ 48000×7/65536 ≈ **5.13 Hz**，
    //   正落在颤音的标准速率区间（5~7 Hz）。
    //  ⚠️ 本模块第一版声明了一个 LFO_RATE_SHIFT 参数却**根本没用**
    //     （步进写死成 +1），那样三角周期是 2^16 = 65536 帧 ≈ 1.37 s，
    //     听感上成了"慢速摇摆"而不是颤音，而且参数名骗人。
    //     现在改成显式的 LFO_STEP。
    parameter integer LFO_STEP = 7
)(
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire                 sample_tick,   // 48 kHz 帧边界

    // ---- 单个声部（或已拍平的多声部扁平总线）----
    input  wire [FW_W-1:0]      freq_in,
    // ---- 全局颤音参数 ----
    input  wire [DEPTH_W-1:0]   vib_depth,     // 0 = 完全旁路
    input  wire                 vib_on,        // 总开关（演示态强制关）

    output reg  [FW_W-1:0]      freq_out,
    output wire                 lfo_valid      // 本拍 freq_out 已更新
);

    // ============================================================
    //  1) 全局 LFO 相位累加器（每帧推进一步）
    // ============================================================
    reg [LFO_W-1:0] lfo_phase;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)           lfo_phase <= {LFO_W{1'b0}};
        else if (sample_tick) lfo_phase <= lfo_phase + LFO_STEP[LFO_W-1:0];
    end

    // ============================================================
    //  2) 三角波：用最高位决定方向，其余位决定幅度
    //     mag = (msb ? ~phase[L-2:0] : phase[L-2:0])
    //     ⇒ mag 本身**已经是一个三角**（0 → max → 0），
    //       所以只要把它中心化（减半幅），就得到 [-半幅, +半幅-1] 的三角。
    // ============================================================
    wire                 dir      = lfo_phase[LFO_W-1];
    wire [LFO_W-2:0]     mag      = dir ? ~lfo_phase[LFO_W-2:0] : lfo_phase[LFO_W-2:0];

    //  ⚠️⚠️ 这里踩过一个坑（2026-10 审查修掉，tb_freq_mod 场景 8 钉住）：
    //    原来写成 `dir ? -(mag+1) : mag` —— 把 dir 当**符号位**用。
    //    但 mag 在第二半段本来就在往下走（max→0），再取负就变成
    //    第二半段**继续往上爬**（-max → -1），整条曲线成了
    //    "0 爬到 +max → 跳到 -max → 再爬回 0" = **锯齿波**，不是三角波。
    //    听感后果：每个 LFO 周期有一次满量程的音高跳变（≈27 音分），
    //    是"带硬边的摇摆"，不是平滑揉弦。
    //    正确做法：三角 = mag 减半幅（中心化）——方向已经在 mag 里了。
    localparam integer       LFO_MID = (1 << (LFO_W - 2));
    wire signed [LFO_W-1:0]  lfo_signed;
    assign lfo_signed = $signed({{1'b0}, mag}) - LFO_MID[LFO_W-1:0];
    //  形状核对（LFO_W=8）：mag = 0..127..0 ⇒ lfo = -64..+63..-64，
    //  相邻差恒为 ±1，过顶/过底都连续 —— 这才是三角波。
    //  范围 |lfo| ≤ 2^(LFO_W-2)。

    //  把 lfo_signed 当成"以 2^(LFO_W-1) 为满幅的定点小数"，再乘 fw：
    //      delta = fw · lfo_signed / 2^(LFO_W-2) / 2^depth
    //    （分母取 LFO_W-2，因为三角的半幅正好是 2^(LFO_W-2)；
    //      归一化之后 |delta| 仍是 fw·2^-depth，与旧口径一致。）
    //  两级都是移位：先按 fw 的位宽对齐，再按 depth 缩小。
    //  ⚠️ 全程用 signed，不能让 fw 的高位当符号位丢掉。
    wire signed [FW_W-1:0] fw_s   = $signed(freq_in);
    wire signed [FW_W+LFO_W:0] prod;
    assign prod = fw_s * lfo_signed;      // 这里**确实**用了一个乘法器

    //  除以 2^(LFO_W-2) 是纯移位；再除以 2^depth 还是移位。
    //  ⇒ 共 1 个乘法器、0 个除法器。32 声部若各自一份就是 32 个 DSP；
    //    所以本模块在顶层是**全局一份**、再对 32 路复用（见 README）。
    wire signed [FW_W+LFO_W:0] shifted;
    assign shifted = prod >>> (LFO_W - 2);

    wire signed [FW_W+LFO_W:0] dep_shifted;
    assign dep_shifted = shifted >>> vib_depth;

    //  只取低 FW_W 位并饱和（颤音不该把频率顶到溢出）
    wire signed [FW_W-1:0] delta = dep_shifted[FW_W-1:0];

    // ============================================================
    //  4) 求和 + 旁路 + 寄存
    // ============================================================
    wire        bypass = (!vib_on) || (vib_depth == {DEPTH_W{1'b0}});
    wire signed [FW_W:0] sum_ext = $signed({fw_s[FW_W-1], fw_s}) +
                                   $signed({delta[FW_W-1], delta});

    reg [FW_W-1:0] freq_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            freq_r <= {FW_W{1'b0}};
        else begin
            if (bypass)
                freq_r <= freq_in;                  // ★ 逐位旁路，不加任何东西
            else if (sum_ext[FW_W] != sum_ext[FW_W-1])
                // 加法溢出（正溢到负/负溢到正）⇒ 钳到正向最大值，
                // 绝不回绕：回绕会让频率瞬间跳到 0 附近，听感是"失声"
                freq_r <= {1'b0, {(FW_W-1){1'b1}}};
            else
                freq_r <= sum_ext[FW_W-1:0];
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) freq_out <= {FW_W{1'b0}};
        else        freq_out <= freq_r;
    end

    assign lfo_valid = 1'b1;

endmodule