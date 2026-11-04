// ============================================================
//  param_xfer.v —— 控制域 → 音频域的参数搬运
//
//  ★ 核心约定（P1 设计冻结 + 跟踪表 P1-3）
//      · **每 48 kHz 帧边界只握手一次**：所有参数在 `sample_tick` 那一拍
//        整体提交，一个帧周期里最多一次 `xfer_stb`。
//      · **音频域内不做异步 FIFO**（会引入抖动）。control 与 audio 是
//        **同一个 clk 域**（都是 100 MHz 系统时钟），所以根本不需要异步
//        握手 —— 需要的只是"把随时可变的控制量对齐到帧边界"。
//
//  ★ 为什么必须对齐帧边界
//    参数（尤其频率字）如果在**一帧中间**变化，这一帧的相位增量就不再等于
//    整帧恒定的值：波形会出现一个短的"半边"跳变，听感上是咔哒声，频谱上是
//    两侧的杂散。把更新点钉在帧边界，相位累加器的 Δφ 才能保证「一帧内恒定」，
//    这是 DDS 相位连续性的前提。
//
//  ★ 为什么不做双缓冲（shadow + active 两层）
//    控制域的 `cfg_*` 本身就已经是寄存器输出（`voice_alloc` 的输出），
//    再存一层 shadow 只是白烧一倍触发器（32 声部 × 45 位 ≈ 1440 位）。
//    这里只保留**一层** `xf_*` 输出寄存器：`cfg_*` 在帧内怎么抖，
//    `xf_*` 都只在 `sample_tick` 的那一拍取样。效果等价，资源减半。
//
//  ★ 时序
//    sample_tick  ──┐
//                    └─（同一拍）xf_* 与 xfer_stb 一起更新
//    ⇒ `xfer_stb` 与 `xf_*` 严格同拍，下游（voice_array）可以直接用
//      `xfer_stb` 当作"本帧参数已就绪"。
//
//  ★ 边界行为（tb 专门测）
//    如果 `cfg_*` 恰好**在** `sample_tick` 那一拍变化（同一个时钟沿），
//    `xf_*` 采到的是**变化前**的值，要等下一帧才更新。
//    这是有意的：宁可晚一帧（20.8 µs，人耳不可闻），也不要为了"当拍生效"
//    引入组合路径。代价是**一帧的确定性延迟**，不是随机延迟。
// ============================================================
module param_xfer #(
    parameter integer NV     = 32,   // 声部数
    parameter integer FW_W   = 32,   // 频率字位宽
    parameter integer WAVE_W = 2,    // 波形选择位宽
    parameter integer AMPW   = 3,    // 音量移位位宽
    parameter integer ATTW   = 7,    // 偷声淡出系数位宽
    parameter integer MOD_W  = 16    // FM 调制指数位宽
)(
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire                 sample_tick,   // 48 kHz 帧边界

    // ---- 控制域输入（随时可变，可以在一帧内抖动）----
    input  wire [NV*FW_W-1:0]   cfg_freq,
    input  wire [NV*WAVE_W-1:0] cfg_wave,
    input  wire [NV*AMPW-1:0]   cfg_amp_shift,
    input  wire [NV*ATTW-1:0]   cfg_atten,
    input  wire [NV-1:0]        cfg_gate,      // 每声部 note_on 电平
    input  wire [NV-1:0]        cfg_voice_en,  // 每声部是否参与混音
    input  wire                 cfg_special,   // 特殊池 加法/FM 切换
    input  wire [MOD_W-1:0]     cfg_mod_index, // FM 调制指数
    input  wire [AMPW-1:0]      cfg_vib_depth, // 全局颤音深度（0 = 关）

    // ---- 音频域输出（只在帧边界整体更新，一帧内恒定）----
    output reg  [NV*FW_W-1:0]   xf_freq,
    output reg  [NV*WAVE_W-1:0] xf_wave,
    output reg  [NV*AMPW-1:0]   xf_amp_shift,
    output reg  [NV*ATTW-1:0]   xf_atten,
    output reg  [NV-1:0]        xf_gate,
    output reg  [NV-1:0]        xf_voice_en,
    output reg                  xf_special,
    output reg  [MOD_W-1:0]     xf_mod_index,
    output reg  [AMPW-1:0]      xf_vib_depth,

    // ---- 握手指示 ----
    output reg                  xfer_stb,      // 单周期：本帧参数已提交
    output reg  [31:0]          xfer_cnt,      // 累计提交次数（tb 数握手用）

    // ---- 诊断（tb 与 ILA 用）----
    output reg  [31:0]          hold_cnt       // 帧内"参数变了但没提交"的拍数
);

    // ============================================================
    //  提交：只在 sample_tick 那一拍整体取样
    // ============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            xf_freq      <= {NV*FW_W{1'b0}};
            xf_wave      <= {NV*WAVE_W{1'b0}};
            xf_amp_shift <= {NV*AMPW{1'b0}};
            xf_atten     <= {NV*ATTW{1'b0}};
            xf_gate      <= {NV{1'b0}};
            xf_voice_en  <= {NV{1'b0}};
            xf_special   <= 1'b0;
            xf_mod_index <= {MOD_W{1'b0}};
            xf_vib_depth <= {AMPW{1'b0}};
            xfer_stb     <= 1'b0;
            xfer_cnt     <= 32'd0;
        end else begin
            // xfer_stb 默认拉低：它必须是**单周期**脉冲，
            // 若写成 `xfer_stb <= sample_tick` 就已经对了，
            // 但下面这句赋值保证"不提交就清零"的语义更明确。
            xfer_stb <= 1'b0;

            if (sample_tick) begin
                xf_freq      <= cfg_freq;
                xf_wave      <= cfg_wave;
                xf_amp_shift <= cfg_amp_shift;
                xf_atten     <= cfg_atten;
                xf_gate      <= cfg_gate;
                xf_voice_en  <= cfg_voice_en;
                xf_special   <= cfg_special;
                xf_mod_index <= cfg_mod_index;
                xf_vib_depth <= cfg_vib_depth;

                xfer_stb <= 1'b1;
                xfer_cnt <= xfer_cnt + 32'd1;
            end
        end
    end

    // ============================================================
    //  诊断：本帧内控制域是否在"偷偷"变化
    //  这不是错误 —— 正是本模块存在的理由。计数只是让"帧内变更"变成
    //  可观测的量，tb 用它来确认「变更确实被推迟到帧边界」。
    // ============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            hold_cnt <= 32'd0;
        else if (!sample_tick) begin
            if (cfg_freq !== xf_freq || cfg_gate !== xf_gate ||
                cfg_atten !== xf_atten)
                hold_cnt <= hold_cnt + 32'd1;
        end
    end

endmodule