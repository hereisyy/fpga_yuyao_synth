// ============================================================
//  低频振荡器 LFO：颤音的"摆动波形"来源
//
//  它就是一个"跑得极慢的 DDS"——相位累加器 + 复用音频的正弦表。
//
//  频率字公式与音频一致：LFO_WORD = round(f_lfo * 2^32 / fs)
//    @48kHz, 5 Hz -> 447392
//
//  ⚠️ 旧版把 447321 写成常量，那是按 5 Hz @48kHz 手算的
//     （实际 4.9992 Hz）。改成由参数推导后，换采样率不会偏。
//     两者的差别听不出来，但规范做法是跟着采样率走。
// ============================================================
module lfo #(
    parameter FS_HZ    = 48_000,   // 采样率，必须与音频链路一致
    parameter LFO_FREQ = 5         // 颤音频率（Hz），音乐上常用 4~6 Hz
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        tick,              // 采样节拍（48kHz 单周期脉冲）
    output wire [31:0] lfo_phase          // LFO 相位，顶层拿去查正弦表
);

    // f * 2^32 / fs。用 64 位表达式做常数折叠，不产生任何硬件乘法/除法。
    localparam [31:0] LFO_WORD = (LFO_FREQ * 64'd4294967296) / FS_HZ;

    reg [31:0] phase;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            phase <= 32'd0;
        else if (tick)
            phase <= phase + LFO_WORD;
    end

    assign lfo_phase = phase;

endmodule
