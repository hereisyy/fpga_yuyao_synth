`timescale 1ns/1ps
// ============================================================
//  timbre_sel.v —— 三种算法切换（A5）：纯波表 / 加法 / FM
//
//       wt_sample  ─┐
//       add_sample ─┼→ [交叉淡入] → sample_out
//       fm_sample  ─┘
//
// ------------------------------------------------------------
//  ★ 切换必须经斜坡：硬切必爆音
// ------------------------------------------------------------
//  两个算法同一时刻的输出数值完全不同（一个可能 +20000，另一个 -18000），
//  直接换源 = 输出瞬间跳 38000 → 耳机里"啪"一声。
//  做法：用 XFADE 帧（默认 256 帧 ≈ 5.3 ms @48 kHz）做线性交叉淡入：
//        out = (旧 × (XFADE - t) + 新 × t) >> log2(XFADE)
//  因为 (XFADE - t) + t = XFADE，这个式子就是"加权平均"，
//  精度也不丢（分子在 32 位里算，最后才右移）。
//  ⚠️ 旧/新样本是 **signed**、t 是 **unsigned** → 又是 signed × unsigned
//     陷阱，必须显式拓宽（见 `vca.v` 的写法）。
//  ⚠️ 淡入期间**三个源都要在跑**（相位继续走）。否则新源从 0 相位起步，
//     听起来像"重新触发"而不是"换音色"。
//
// ------------------------------------------------------------
//  切换时的包络衔接（第二层含义）
// ------------------------------------------------------------
//  跟踪表说"切换必须经包络斜坡"，除了上面这个输出层淡入，还有一层：
//  **换算法时不能重置 ADSR**。ADSR 是按声部走的、共用的，
//  换音色不该让音量重新从 0 起来。
//  ⇒ 本模块只管"选谁"，**绝不碰任何 gate / 包络寄存器**。
//
// ------------------------------------------------------------
//  `algo_act` 是"当前真正生效"的算法
// ------------------------------------------------------------
//  淡入过程中它报告**旧**算法，直到新源完全接管才更新。
//  上板时拿它点 LED，就能一眼看出"面板拨了但没切过去"这类问题。
//
//  验收（`tb_timbre.v`）：
//    ① 三种算法各选一次，输出确实来自对应的源
//    ② **切换那一帧的跳变量小**（无爆音的唯一判据）
//       ⚠️ 判据要同时看**上升和下降**跳变！历史上 tb_adsr 的"无爆音"
//          检查只统计了上升跳变，漏掉了下降 4095（6.25%）的大跳变
//    ③ 淡入结束时输出**完全等于**新源（残留 0）
//
//  依赖：A1 `additive.v`、A2 `fm_op.v`（以及已有的 `osc_dds.v`）
// ============================================================

module timbre_sel #(
    parameter integer OUT_W = 16,
    parameter integer XFADE = 256,      // 交叉淡入的帧数（≈5.3 ms @48 kHz）
    parameter integer XF_W  = 8         // XFADE = 2^XF_W
)(
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    sample_tick,

    // 请求的算法：0=纯波表 1=加法 2=FM（3 未定义 → 退回纯波表）
    input  wire [1:0]              algo_req,

    // 三个引擎的输出（**没选中的那个也要在跑**）
    input  wire signed [OUT_W-1:0] wt_sample,
    input  wire signed [OUT_W-1:0] add_sample,
    input  wire signed [OUT_W-1:0] fm_sample,

    output reg  signed [OUT_W-1:0] sample_out,
    output reg  [1:0]              algo_act
);

    localparam signed [OUT_W-1:0] MAXW = (1 <<< (OUT_W-1)) - 1;
    localparam signed [OUT_W-1:0] MINW = -(1 <<< (OUT_W-1));

    // ---- 按算法号选源（未定义档退回纯波表，避免综合出 latch / x）----
    function signed [OUT_W-1:0] pick;
        input [1:0]              a;
        input signed [OUT_W-1:0] w;
        input signed [OUT_W-1:0] d;
        input signed [OUT_W-1:0] f;
        begin
            case (a)
                2'd1:    pick = d;
                2'd2:    pick = f;
                default: pick = w;
            endcase
        end
    endfunction

    function signed [OUT_W-1:0] satf;
        input signed [31:0] v;
        begin
            if      (v > MAXW) satf = MAXW;
            else if (v < MINW) satf = MINW;
            else               satf = v[OUT_W-1:0];
        end
    endfunction

    wire signed [OUT_W-1:0] src_old = pick(algo_act, wt_sample, add_sample, fm_sample);
    wire signed [OUT_W-1:0] src_new = pick(algo_req, wt_sample, add_sample, fm_sample);

    // ---- 交叉淡入的插值 ----
    //   t 从 0 走到 XFADE：t=0 时全是旧源，t=XFADE 时全是新源
    //   ⚠️ 32 位里算完再右移，别在窄位宽里移（位宽陷阱）
    reg  [XF_W:0] xfd;
    reg           fading;

    wire signed [31:0] a32  = $signed(src_old);
    wire signed [31:0] b32  = $signed(src_new);
    wire signed [31:0] t32  = $signed({1'b0, xfd});          // 0 .. XFADE（恒非负）
    wire signed [31:0] c32  = XFADE - t32;                   // XFADE - t
    wire signed [31:0] mix32 = (a32 * c32 + b32 * t32) >>> XF_W;

    wire signed [OUT_W-1:0] mix_s  = satf(mix32);
    wire signed [OUT_W-1:0] new_s  = satf(b32);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            algo_act   <= 2'd0;
            xfd        <= {(XF_W+1){1'b0}};
            fading     <= 1'b0;
            sample_out <= {OUT_W{1'b0}};
        end else begin
            // ---- 只在采样节拍上推进淡入（每拍推进就成了硬切）----
            if (sample_tick) begin
                if (algo_req != algo_act) begin
                    fading <= 1'b1;
                    if (xfd >= XFADE[XF_W:0]) begin
                        // 淡入走完：正式换算法，输出从这一帧起就是新源
                        algo_act <= algo_req;
                        xfd      <= {(XF_W+1){1'b0}};
                        fading   <= 1'b0;
                    end else begin
                        xfd <= xfd + 1'b1;
                    end
                end else begin
                    // 请求和当前一致：不需要淡入（比如刚上电、或拨回去）
                    fading <= 1'b0;
                    xfd    <= {(XF_W+1){1'b0}};
                end
            end

            // ---- 输出 ----
            //  ⚠️ 这里必须用"马上就要淡入"的条件，**不能只看 fading 寄存器**：
            //     fading 是**这一拍**才被置位的，若输出条件只用它，
            //     切换的第一帧就会直接输出 new_s —— 那就是硬切，
            //     实测跳变 38000（tb_timbre 第一版正是这么报错的）。
            //     条件带上 (algo_req != algo_act) 之后，第一帧输出的是
            //     xfd=0 的插值结果 = 旧源，曲线才连续。
            sample_out <= ((algo_req != algo_act) || fading) ? mix_s : new_s;
        end
    end

endmodule
