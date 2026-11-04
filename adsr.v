`timescale 1ns/1ps
// ============================================================
//  adsr.v —— ADSR 包络发生器（P0-4）
//
//  它做什么：把一个"按键按下/松开"的电平，变成一条**增益曲线**。
//  没有它，每个音的起止都是硬切 —— 那种"咔"声（爆音）在频谱上是一
//  大片宽带能量，是评审一眼就能听出来的低级问题。
//
//      gate ──┬─ 上升沿 → ATTACK ─→ DECAY ─→ SUSTAIN
//             └─ 电平 0 → RELEASE（从**当前电平**开始）
//
//  输出 gain ∈ [0, 65535]（无符号 16 位，0 = 静音，满量程 = 1.0），
//  交给 P0-5 的 `vca.v` 去乘采样点。
//
// ------------------------------------------------------------
//  一、三条曲线怎么算 —— **全程零乘法器，只有移位和加减**
// ------------------------------------------------------------
//  这是本模块最重要的设计决定。用"指数逼近"而不是"线性斜坡"：
//
//    ATTACK :  gain += (FULL - gain) >> ATT_SHIFT      （朝满量程指数逼近）
//    DECAY  :  gain -= (gain - SUS)  >> DEC_SHIFT      （朝 sustain 指数逼近）
//    RELEASE:  gain -= (gain >> REL_SHIFT) | 1         （朝 0 指数衰减）
//
//  为什么用指数：**这就是模拟合成器的自然形状**，也是所有数字合成器的
//  标准做法。而且它有个巨大的工程好处 —— 每次逼近的量**正比于剩余距离**，
//  所以不需要知道"要走多少步"，也不需要除法，**一个移位 + 一个加减就够了**。
//  线性斜坡反而要算 步长 = 满量程/总步数（除法，或者预先算好的常数）。
//
//  资源代价：**0 个 DSP、0 个 BSRAM**。32 个声部也就几千个 FF。
//
// ------------------------------------------------------------
//  二、⚠️ 三段都必须"收尾"，否则会出三种经典 bug
// ------------------------------------------------------------
//  指数逼近的本质是**永远到不了目标**（每次只走剩余的一小部分）。
//  不管的话会有三个后果，每个都很难查：
//
//    ① ATTACK 永远差一点点到不了满量程 → 音量比预期小几个百分点
//    ② DECAY 永远悬在 sustain 上面一点 → 稳态音量偏低
//    ③ **RELEASE 永远停在非零**（最严重）：
//       当 gain 变小时 `gain>>REL_SHIFT` 会变成 0，于是 gain 再也不减了。
//       包络**卡在一个小的非零值上永远不结束** → `active` 一直是 1 →
//       在复音系统里**声部永远不释放**，弹几十个音之后所有声部都被占满，
//       **新按的键全都没声音**。
//       现象是"弹一会儿就不响了，重启才好" —— 极其难查。
//
//  所以三段的收尾规则是**必须**的：
//    ① (FULL - gain) < 2^ATT_SHIFT  → 直接跳到 FULL（跳变量 ≤ 31/65535 = 0.05%，听不出来）
//    ② (gain - SUS)  < 2^DEC_SHIFT  → 直接跳到 SUS
//    ③ release 步长写成 `(gain>>REL_SHIFT) | 1` —— **`|1` 保证至少减 1**，
//       所以最后一段是"每拍减 1"的线性尾，一定能精确落到 0。
//       这个 `|1` 是本模块最容易被漏掉、也最要命的一个字符。
//
// ------------------------------------------------------------
//  三、时间常数表（实测值，48 kHz 下）
// ------------------------------------------------------------
//  下面这些数不是估的，是按上面的递推式**逐拍跑出来的**
//  （`analyze_adsr.py` 会独立重算一遍并对拍）：
//
//    ATT_SHIFT :  4 → 140 拍 =   2.92 ms        6 → 478 拍 =   9.96 ms
//                 5 → 260 拍 =   5.42 ms ★默认   7 → 870 拍 =  18.12 ms
//
//    DEC_SHIFT : 10 → 3590 拍 =  74.79 ms       12 → 8329 拍 = 173.52 ms ★默认
//                11 → 5703 拍 = 118.81 ms       13 → 9832 拍 = 204.83 ms
//
//    REL_SHIFT : 10 →  5192 拍 = 108.17 ms      12 → 15086 拍 = 314.29 ms ★默认
//                11 →  8966 拍 = 186.79 ms      13 → 24464 拍 = 509.67 ms
//
//  ⚠️ 时间常数是**离散的**（一档差一倍），不能任意指定毫秒。
//     这是"零乘法器指数逼近"的代价，也是所有硬件合成器的现实做法。
//     想要中间值，可以按段做"移位 + 补一点点"（`(x>>k) + (x>>(k+3))` 之类），
//     现在不需要，先把功能做对。
//
//  ⚠️ RELEASE 的总时间里约 **1/4 是最后那截线性尾**（2^REL_SHIFT 拍）。
//     k=12 时：指数段 9895 拍（206 ms）+ 线性尾 5191 拍（108 ms）= 314 ms。
//     尾段电平已经低于 6%（−24 dB），听感上就是自然的余音，不用担心。
//
// ------------------------------------------------------------
//  四、接口约定（写给 P0-7 / P1-1 的人）
// ------------------------------------------------------------
//  · **gate 必须是电平信号**（按住 = 1），**不能接 `key_press` 那种单周期脉冲**。
//    模块只在采样节拍上看一次 gate，单周期脉冲会被漏掉。
//    正确接法：`gate = keyboard.v 的 note_on`（或声部分配器给的门）。
//  · `gain` 每个采样节拍更新一次，**本拍的值在下一次 sample_tick 才变**。
//  · **从任意阶段松键都立刻进 RELEASE**，而且是从**当前电平**开始衰减
//    （不是从 sustain 开始）—— 这样才不会在松键瞬间跳变出咔声。
//  · **重触发**：RELEASE 期间再次按下 → 回到 ATTACK，同样从当前电平继续，
//    曲线是连续的，没有跳变。
//  · `active` = 正在发声（含 RELEASE 中）。它为 0 时包络已精确归零，
//    声部可以回收。P1-2 的 `voice_alloc.v` 靠这个判"空闲声部"。
//
//  依赖：无（可并行开发）✅
// ============================================================

module adsr #(
    parameter GAIN_W     = 16,      // 增益位宽（满量程 = 2^GAIN_W - 1）
    // ---- 三段的时间常数档（越大越慢；见表）----
    parameter integer ATT_SHIFT = 5,    // 260 拍 = 5.42 ms
    parameter integer DEC_SHIFT = 12,   // 8329 拍 = 173.5 ms
    parameter integer REL_SHIFT = 12,   // 15086 拍 = 314.3 ms
    // ---- sustain 电平（满量程的百分比）----
    parameter integer SUS_PCT   = 70    // 70% → 45874 / 65535
)(
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  sample_tick,   // 48 kHz 节拍（单周期脉冲）

    // ---- 门：电平信号，按住=1；**不要接单周期脉冲** ----
    input  wire                  gate,

    // ---- 输出增益 ----
    //  ⚠️ 本信号是**无符号**（0 ~ 65535，恒非负）。
    //     与 `osc_dds.v` 的 `sample_out`（**有符号**）相乘时，
    //     必须显式把两边都拓宽成有符号，否则负半周会被毁掉 ——
    //     详细说明和正确写法见 `osc_dds.v` 的 `sample_out` 端口注释
    //     （实测：signed(-100) × unsigned(1000) = 65436000，不是 -100000）。
    output reg  [GAIN_W-1:0]     gain,

    // ---- 状态（调试 / 声部分配用）----
    output wire [2:0]            state,
    output wire                  active         // 1 = 正在发声（含 RELEASE 中）
);

    // ============================================================
    //  常量
    //
    //  ⚠️ 算常量时**必须用 32 位的 integer 中间量**：
    //     `(16'd65535 * 70) / 100` 在 16 位上下文里会先把乘法截断，
    //     结果完全错。这是本工程反复踩到的"表达式位宽 = 上下文位宽"陷阱
    //     （keyboard.v 的颤音、tb_osc 的平方和都栽过）。
    //     用 integer（32 位）就没事。
    // ============================================================
    localparam [GAIN_W-1:0] FULL     = {GAIN_W{1'b1}};              // 全 1 = 满量程
    localparam integer      FULL_I   = (1 << GAIN_W) - 1;
    localparam [GAIN_W-1:0] SUS_WORD = (FULL_I * SUS_PCT) / 100;

    // 状态编码
    localparam [2:0] S_IDLE = 3'd0,
                     S_ATT  = 3'd1,
                     S_DEC  = 3'd2,
                     S_SUS  = 3'd3,
                     S_REL  = 3'd4;

    // 参数自检（只仿真，不进综合）
    // synthesis translate_off
    initial begin
        if (ATT_SHIFT >= GAIN_W || DEC_SHIFT >= GAIN_W || REL_SHIFT >= GAIN_W) begin
            $display("!! adsr: 移位档必须小于 GAIN_W=%0d", GAIN_W);
            $finish;
        end
        if (SUS_PCT < 0 || SUS_PCT > 100) begin
            $display("!! adsr: SUS_PCT=%0d 必须在 0~100", SUS_PCT);
            $finish;
        end
    end
    // synthesis translate_on

    reg [2:0] st;
    assign state  = st;
    assign active = (st != S_IDLE);

    // ============================================================
    //  组合：本拍的三个步长
    //
    //  都用当前的 gain 算，交给下面时序块使用。
    //  这样时序块里只有"加法 / 减法 / 比较"，没有中间算术，
    //  读起来清楚，也避免了在时序块里混用阻塞赋值。
    // ============================================================
    wire [GAIN_W-1:0] att_diff = FULL - gain;
    wire [GAIN_W-1:0] att_step = att_diff >> ATT_SHIFT;

    wire [GAIN_W-1:0] dec_diff = gain - SUS_WORD;
    wire [GAIN_W-1:0] dec_step = dec_diff >> DEC_SHIFT;

    // ★ 那个 `| 1` 是保命的：它保证每一步至少减 1，所以一定能到 0。
    //   漏掉它 → 包络卡在非零值 → 声部永不释放 → "弹一会儿就不响了"。
    wire [GAIN_W-1:0] rel_step = (gain >> REL_SHIFT) | {{(GAIN_W-1){1'b0}}, 1'b1};

    // ---- gate 边沿检测：只在采样节拍上比较，检测两个节拍之间的跳变 ----
    reg  gate_d;
    wire gate_rise = gate & ~gate_d;

    // ============================================================
    //  状态机
    // ============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gain   <= {GAIN_W{1'b0}};
            st     <= S_IDLE;
            gate_d <= 1'b0;
        end else if (sample_tick) begin
            gate_d <= gate;

            case (st)
            // ---------------- 空闲：一直输出 0 ----------------
            S_IDLE: begin
                gain <= {GAIN_W{1'b0}};
                if (gate) st <= S_ATT;
            end

            // ---------------- 攻击：朝满量程指数上升 ----------------
            S_ATT: begin
                if (!gate) begin
                    st <= S_REL;              // 中途松键 → 立刻转释放（gain 保持）
                end else if (att_step == 0) begin
                    gain <= FULL;             // 收尾：直接到满量程
                    st   <= S_DEC;
                end else begin
                    gain <= gain + att_step;
                end
            end

            // ---------------- 衰减：朝 sustain 指数下降 ----------------
            S_DEC: begin
                if (!gate) begin
                    st <= S_REL;
                end else if (dec_step == 0) begin
                    gain <= SUS_WORD;         // 收尾：直接到 sustain
                    st   <= S_SUS;
                end else begin
                    gain <= gain - dec_step;
                end
            end

            // ---------------- 保持：钉在 sustain 上 ----------------
            S_SUS: begin
                gain <= SUS_WORD;
                if (!gate) st <= S_REL;
            end

            // ---------------- 释放：朝 0 指数衰减 ----------------
            S_REL: begin
                if (gate_rise) begin
                    st <= S_ATT;              // 重触发：从当前电平继续上升
                end else if (gain <= rel_step) begin
                    gain <= {GAIN_W{1'b0}};   // 收尾：精确归零（这一步不能省）
                    st   <= S_IDLE;
                end else begin
                    gain <= gain - rel_step;
                end
            end

            default: st <= S_IDLE;
            endcase
        end
    end

endmodule
