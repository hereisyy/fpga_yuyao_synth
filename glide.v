// ============================================================
//  glide.v —— 滑音本体（频率字一阶插值）
//
//  这是 `glide_ctrl.v` 的**另一半**：ctrl 只产生开关电平，本体在这个模块里。
//  分开的理由：开关是"控制域"的事，插值是"音频域/频率字路径"的事，
//  混在一起两边都难单独验证。
//
//  ---- 算法（与 `keyboard.v` 里那份参考实现一致，可对照）----
//    每个 `sample_tick`（48 kHz）：
//      glide_en = 0 → freq <= target            （直接跳变，无滑音）
//      glide_en = 1 → 指数逼近：
//            step = |target - freq| >> SHIFT
//            step == 0 → freq <= target         （**收尾跳变**，避免永远差一步）
//            否则     → freq <= freq ± step
//    时间常数 τ = 2^SHIFT / Fs：
//      SHIFT=10 → 21.3 ms（快）   SHIFT=12 → 85.3 ms（默认）   SHIFT=13 → 171 ms（很慢）
//
//  ---- 为什么"指数逼近"是对的 ----
//    τ 恒定 ⇒ 音程越大滑得越快、越小滑得越慢，听感上接近模拟合成器的 RC 滑音；
//    而"线性步进"会让大音程滑得拖沓、小音程滑得生硬。
//
//  ---- 相位连续性（很重要，别在这里画蛇添足）----
//    本模块只改**频率字**，**绝不碰相位累加器**。
//    DDS 的相位不因频率字改变而复位 ⇒ 换音时波形天然连续，没有爆音。
//    谁要是在这里加"换音时清零相位"，就会换来一声"啪"。
//
//  ---- 延迟记账 ----
//    本模块不进 `sample_tick` 之后的样本流，所以**没有** `LATENCY` 可言：
//    它在 `sample_tick` 那拍更新一次频率字，`freq` 是一直有效的电平信号，
//    不需要 `valid`，也不占用音频链的流水级数。
// ============================================================
`default_nettype none

module glide #(
    parameter integer SHIFT = 12,      // τ = 2^SHIFT / 48000
    parameter [31:0]  INIT_FREQ = 32'd0
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        sample_tick,    // 48 kHz 单周期脉冲（与全链共用）
    input  wire        glide_en,       // 1 = 滑音开
    input  wire [31:0] target,         // 目标频率字
    output reg  [31:0] freq            // 当前频率字（送振荡器）
);

    wire [31:0] diff_up   = target - freq;
    wire [31:0] diff_dn   = freq - target;
    wire [31:0] step_up   = diff_up >> SHIFT;
    wire [31:0] step_dn   = diff_dn >> SHIFT;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            freq <= INIT_FREQ;
        end else if (sample_tick) begin
            if (!glide_en) begin
                freq <= target;                        // 关滑音：直接跳
            end else if (target > freq) begin
                freq <= (step_up == 32'd0) ? target : (freq + step_up);
            end else if (target < freq) begin
                freq <= (step_dn == 32'd0) ? target : (freq - step_dn);
            end
        end
    end

endmodule

`default_nettype wire