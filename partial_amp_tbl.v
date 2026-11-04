`timescale 1ns/1ps
// ============================================================
//  partial_amp_tbl.v —— 分音幅度表（A3）
//
//  给 `additive.v` 两样东西：
//    ① `partial_amp`：当前音色下第 1~8 个分音各占多大声（决定音色）
//    ② `n_partials` ：这一档**实际用几个分音**（防混叠）
//
// ------------------------------------------------------------
//  ★ 两条硬约束（改表时必须重新验一遍，否则会出难查的失真）
// ------------------------------------------------------------
//  ① **各分音幅度之和 ≤ 满量程（4095）**
//     因为 additive 把加权和右移 AW 位再塞回 16 位 ——
//     和超了就会溢出回绕（破音）。四张表的和：
//         timbre 0 正弦  : 4095
//         timbre 1 锯齿  : 4094   （1/k 形状）
//         timbre 2 方波  : 4094   （只有奇次，1/k）
//         timbre 3 三角  : 4095   （只有奇次，1/k²）
//  ② **第 k 分音频率 k×f0 必须 < 24 kHz**（fs/2 = 奈奎斯特）
//     超了就折返成听不见的怪声（混叠，金属味/啸叫），
//     所以高音区要**动态砍掉高次分音** —— 这就是 n_partials 的用途。
//     算例：8 个分音时 f0 最大允许 3 kHz（8×3000 = 24 kHz）；
//           C7(2093 Hz) 用 8 个 = 16744 Hz ✓；C8(4186) × 8 = 33488 ✗ → 只给 3 个。
//
//  依赖：无（纯查表，可以最先写）
//  下游：A1 `additive.v`
// ============================================================

module partial_amp_tbl #(
    parameter integer NP      = 8,       // 最多几个分音
    parameter integer AW      = 12,      // 幅度位宽（满量程 = 2^AW-1 = 4095）
    parameter integer NOTE_W  = 7        // MIDI 音符号位宽
)(
    input  wire                  clk,        // 未用（纯组合），为统一例化风格保留
    input  wire                  rst_n,

    input  wire [NOTE_W-1:0]     note_num,   // 当前音高（来自 keyboard.v）
    input  wire [1:0]            timbre,     // 音色档（0/1/2/3）

    output reg  [3:0]            n_partials, // 本档实际用几个
    output reg  [NP*AW-1:0]      partial_amp // 各分音幅度（已归一化）
);

    // ============================================================
    //  四档音色的分音幅度表
    //  ⚠️ 这些数不是随便写的：(1/k)·系数 之类算好后，
    //     再**整体缩放到"和 ≤ 4095"**。改任何一格都要重算和。
    // ============================================================
    function [AW-1:0] amp_of;
        input [1:0]     tb;
        input integer   k;
        begin
            case (tb)
                // ---- 0：纯正弦（只有一个分音，最"干净"）----
                2'd0: amp_of = (k == 0) ? 12'd4095 : 12'd0;

                // ---- 1：锯齿（全部整数倍分音，1/k）----
                2'd1: case (k)
                        0:       amp_of = 12'd1507;
                        1:       amp_of = 12'd753;
                        2:       amp_of = 12'd502;
                        3:       amp_of = 12'd377;
                        4:       amp_of = 12'd301;
                        5:       amp_of = 12'd251;
                        6:       amp_of = 12'd215;
                        7:       amp_of = 12'd188;
                        default: amp_of = 12'd0;
                      endcase

                // ---- 2：方波（只有奇次分音，1/k）----
                2'd2: case (k)
                        0:       amp_of = 12'd2443;
                        2:       amp_of = 12'd814;
                        4:       amp_of = 12'd488;
                        6:       amp_of = 12'd349;
                        default: amp_of = 12'd0;
                      endcase

                // ---- 3：三角（只有奇次分音，1/k²，最柔和）----
                2'd3: case (k)
                        0:       amp_of = 12'd3496;
                        2:       amp_of = 12'd388;
                        4:       amp_of = 12'd140;
                        6:       amp_of = 12'd71;
                        default: amp_of = 12'd0;
                      endcase

                default: amp_of = 12'd0;
            endcase
        end
    endfunction

    // ============================================================
    //  按音高定分音个数（混叠红线）+ 铺开幅度
    // ============================================================
    integer  k;
    reg [3:0] np_r;

    always @(*) begin
        // 音区 → 分音个数。8 个分音在 f0 ≤ 3 kHz 时安全（8×3k = 24k）
        if      (note_num < 7'd96)  np_r = 4'd8;   // C7 以下
        else if (note_num < 7'd108) np_r = 4'd5;   // C7 ~ B7
        else                        np_r = 4'd3;   // C8 以上（只留 1/3/5 次）

        n_partials = np_r;

        // ⚠️ 组合块里**每个输出都要有默认赋值**，否则综合出 latch
        for (k = 0; k < NP; k = k + 1) begin
            if (k < np_r) partial_amp[k*AW +: AW] = amp_of(timbre, k);
            else          partial_amp[k*AW +: AW] = {AW{1'b0}};
        end
    end

endmodule
