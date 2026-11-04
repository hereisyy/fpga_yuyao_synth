// ============================================================
//  键盘编码器（复音版）
//
//  用途：把 12 路"已经去抖"的按键状态，变成
//        ① 按键位图        —— 给复音声部分配器用
//        ② 按下/松开的边沿  —— 事件驱动用
//        ③ 最高音优先编码   —— 单音调试 / 兼容旧接口用
//
//  为什么要改：
//    旧版是纯组合的优先编码器（一层 else if），结构上永远
//    只能输出"一个"音符号 —— 同时按 3 个键也只会响 1 个音，
//    做不了和弦。而赛题基础要求第 3 条明确写着
//    "复音数 ≥4，支持四路及以上独立声部同时并发发声"。
//    所以必须改成"输出位图 + 边沿"，把"哪个音该发声"的决定
//    交给后面的声部分配器，而不是在这里压成一个音。
//
//  按键位序约定（重要）：
//    key[0]  = 最低音（C4, MIDI 60）
//    key[11] = 最高音（B4, MIDI 71）
//    与旧版 key_encoder.v 的约定一致。若触摸键盘面板的实际
//    排线是反的，把 REVERSE_BIT_ORDER 改成 1，不要去改接线。
//
//  时钟域：纯 clk 域逻辑，与 48 kHz 音频节拍无关。
//          输入应当来自 debounce.v 的输出（已去抖）。
// ============================================================
module key_encoder #(
    // 输入同步级数。1 = 只打一拍；2 = 标准两级同步器（推荐）。
    // 输入理论上已过 debounce.v，这是额外保险：将来若有人把
    // 外部信号直接接进来，也不会出现亚稳态。
    parameter SYNC_STAGES      = 2,
    // 0: key[0] 是最低音（默认，与旧版一致）
    // 1: 把 12 位位序反过来（key[0] 当最高音）
    parameter REVERSE_BIT_ORDER = 0
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire [11:0] key,          // 12 路已去抖的按键（1 = 按下）

    // ---- 输出 ①：位图，直接给声部分配器 ----
    output wire [11:0] key_state,    // 当前稳定的按键位图
    output wire [11:0] key_press,    // 新按下，单周期脉冲
    output wire [11:0] key_release,  // 新松开，单周期脉冲

    // ---- 输出 ②：整体活动状态（帧边界握手用）----
    output wire        active,         // 当前是否有任意键按下
    output wire        active_press,   // "全松" -> "有键"，单周期
    output wire        active_release, // "有键" -> "全松"，单周期

    // ---- 输出 ③：最高音优先编码（单音调试 / 兼容旧接口）----
    output wire [3:0]  note_num,     // 按下集合中音最高的，0~11
    output wire        note_valid    // 有没有键按下（= active）
);

    // ============================================================
    //  1) 输入同步
    //
    //     用 generate 逐级显式展开，不用 for 循环给数组赋值：
    //     非阻塞赋值 + 循环变量 + 数组的组合在部分仿真器上会推出
    //     不定态 x（Icarus 就是这样），显式展开最稳，综合结果也确定。
    //     SYNC_STAGES = 2 是标准两级同步器（推荐）。
    // ============================================================
    reg [11:0] sync_pipe [0:SYNC_STAGES-1];

    // 第一级
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) sync_pipe[0] <= 12'd0;
        else        sync_pipe[0] <= key;
    end

    // 中间各级
    genvar gi;
    generate
        for (gi = 1; gi < SYNC_STAGES - 1; gi = gi + 1) begin : g_sync_mid
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) sync_pipe[gi] <= 12'd0;
                else        sync_pipe[gi] <= sync_pipe[gi-1];
            end
        end
    endgenerate

    // 最后一级（SYNC_STAGES == 1 时与第一级重合，所以加条件）
    generate
        if (SYNC_STAGES > 1) begin : g_sync_last
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) sync_pipe[SYNC_STAGES-1] <= 12'd0;
                else        sync_pipe[SYNC_STAGES-1] <= sync_pipe[SYNC_STAGES-2];
            end
        end
    endgenerate

    wire [11:0] key_sync = sync_pipe[SYNC_STAGES-1];

    // 位序可选翻转
    wire [11:0] key_mapped;
    assign key_mapped = REVERSE_BIT_ORDER
                      ? {key_sync[0],  key_sync[1],  key_sync[2],  key_sync[3],
                         key_sync[4],  key_sync[5],  key_sync[6],  key_sync[7],
                         key_sync[8],  key_sync[9],  key_sync[10], key_sync[11]}
                      : key_sync;

    // ============================================================
    //  2) 打一拍得到"上一状态"，再比较出边沿
    //
    //     时序（SYNC_STAGES = 2 时）：
    //       时刻 n   : key 变化
    //       时刻 n+2 : key_state 更新为新值
    //       时刻 n+2 : key_press / key_release 同时拉高一个时钟
    //     事件比物理按键晚 2 个时钟，50~100 MHz 下是 20~40 ns，
    //     相对人耳阈值（10 ms）可忽略。
    // ============================================================
    reg [11:0] key_state_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) key_state_r <= 12'd0;
        else        key_state_r <= key_mapped;
    end

    assign key_state   = key_state_r;
    assign key_press   = key_mapped & ~key_state_r;   // 0 -> 1
    assign key_release = ~key_mapped &  key_state_r;  // 1 -> 0

    // ============================================================
    //  3) 整体活动状态
    // ============================================================
    wire active_now  = |key_mapped;
    wire active_prev = |key_state_r;

    assign active         = active_prev;             // 与 key_state 同拍
    assign active_press   =  active_now & ~active_prev;
    assign active_release = ~active_now &  active_prev;

    // ============================================================
    //  4) 最高音优先编码
    //
    //     与旧版行为一致，只为单音调试和兼容旧接口。
    //     复音链路上请用 key_state / key_press，不要用 note_num。
    // ============================================================
    reg [3:0] note_num_r;
    reg       note_valid_r;

    always @(*) begin
        note_valid_r = 1'b1;
        if      (key_state_r[11]) note_num_r = 4'd11;
        else if (key_state_r[10]) note_num_r = 4'd10;
        else if (key_state_r[9])  note_num_r = 4'd9;
        else if (key_state_r[8])  note_num_r = 4'd8;
        else if (key_state_r[7])  note_num_r = 4'd7;
        else if (key_state_r[6])  note_num_r = 4'd6;
        else if (key_state_r[5])  note_num_r = 4'd5;
        else if (key_state_r[4])  note_num_r = 4'd4;
        else if (key_state_r[3])  note_num_r = 4'd3;
        else if (key_state_r[2])  note_num_r = 4'd2;
        else if (key_state_r[1])  note_num_r = 4'd1;
        else if (key_state_r[0])  note_num_r = 4'd0;
        else begin
            // 注意：这里的 0 是"无键"占位值。旧版就是因为这个 0
            // 恰好等于 C4 的合法音符号，导致松键时音高跳到 C4。
            // 调用方必须用 note_valid（或直接用 key_state）判断，
            // 不要只看 note_num。
            note_num_r   = 4'd0;
            note_valid_r = 1'b0;
        end
    end

    assign note_num   = note_num_r;
    assign note_valid = note_valid_r;

endmodule
