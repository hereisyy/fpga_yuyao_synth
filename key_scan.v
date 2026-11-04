// ============================================================
//  key_scan.v —— P1 控制域：12 键扫描 + 八度 + MIDI 音符号
//
//  从 06_keyboard\keyboard.v 里**派生**出来的（不是复制粘贴后放养）：
//  去抖、同步、边沿、MIDI 映射、档位表这几段与 keyboard.v 逐字同构，
//  tb_key_scan.v 会把本模块和 keyboard.v 挂在**同一组激励**上逐周期
//  对比 —— 这就是"派生"的证据。
//
//  与 keyboard.v 的差别（都是 P1 复音要求的）：
//    ① keyboard.v 内部压成"最高音优先"的单音通路；本模块保住 12 位
//       位图 + 每键边沿，最高音优先只留作调试/兼容输出（note_num）。
//    ② 多输出一个 midi_base = 12*(oct+3)：复音侧对每个按下的键算
//       note = midi_base + key_idx，不必再各算一遍八度。
//    ③ 不再算频率字、不做滑音/颤音 —— 那些下沉到 freq_mod.v。
//       （本模块只输出"哪个键、哪个八度"，不碰音频域。）
//    ④ sample_tick 仍然由本模块产生（与 keyboard.v 同一公式），
//       整条音频链路共用这一个节拍，不许各自分频。
//
//  极性：key_raw / mod_raw 一律**高有效**（1 = 按下），与 keyboard.v 一致。
//        板上按键是上拉（低有效），**顶层负责取反**再送进来。
//
//  按键位序：key[0] = 最低音（C），key[11] = 最高音（B）。
//
//  ⚠️ SAMPLE_DIV / CNT_MAX 两条公式是从 keyboard.v 抄过来的，
//     **必须与它逐字一致**。它们各自躲开过两个坑，改动前先看
//     keyboard.v 里的长注释（整数除法截断、32 位中间乘积溢出）。
// ============================================================
module key_scan #(
    // 系统时钟（真实 100 MHz；仿真里按比例缩小）
    parameter SYS_CLK_HZ = 100_000_000,
    // 目标采样率
    parameter FS_HZ      = 48_000,
    // 按键去抖时间（微秒）
    parameter DEB_US     = 82,
    // 八度上限（**必须与 P2 的 octave_key.OCT_MAX 一致**）
    //   · OCT_MAX=4 ⇒ midi_base = 12*(4+3) = 84（C6），最高键 95（B6）
    //   · freq_table 覆盖 24..107 ⇒ 4 档安全
    //   ⚠️ 这里曾经写死钳到 3，于是 P2 的"第 4 档八度"被**静默吃掉**：
    //      按到第 4 档还是 C5。2026-10-23 集成时发现，改成参数。
    parameter [2:0] OCT_MAX = 3'd4
)(
    input  wire        clk,
    input  wire        rst_n,

    // ---- 演奏输入（全部高有效：1 = 按下）----
    input  wire [11:0] key_raw,      // 12 个弹音键（原始、会抖）
    input  wire [2:0]  mod_raw,      // 3 个档位键（原始、会抖）
    input  wire [2:0]  octave_sel,   // 八度选择，默认建议 3'd2 -> C4~B4

    // ---- 共用的采样节拍 ----
    output wire        sample_tick,  // 48 kHz 单周期脉冲，全链路共用

    // ---- 复音侧（信源）----
    output wire [11:0] key_state,    // 去抖 + 同步后的按键位图
    output wire [11:0] key_press,    // 新按下，单周期脉冲（位图）
    output wire [11:0] key_release,  // 新松开，单周期脉冲（位图）
    output wire        active,       // 当前是否有任意键按下
    output wire [2:0]  octave_out,   // 实际生效的八度（已钳位）
    output wire [6:0]  midi_base,    // 12*(oct+3)：本八度最低音的 MIDI 号
                                     //   某键 idx 的音符号 = midi_base + idx

    // ---- 调试 / 兼容单音通路（与 keyboard.v 同义）----
    output wire [6:0]  note_num,     // 按下集合中音最高的 MIDI 号
                                     //   无键时等于 midi_base（占位值！）
    output wire        note_on,      // 是否有键按下（= active）

    // ---- 调制档位（给 freq_mod.v 用）----
    output wire [3:0]  amp_shift,    // 颤音深度档，0 = 无颤音
    output wire [3:0]  glide_shift   // 滑音时间常数档（P1 只透传，P3 才用）
);

    // ============================================================
    //  0) 参数派生（逐字同 keyboard.v）
    // ============================================================
    localparam SAMPLE_DIV = ((SYS_CLK_HZ / 1000) + (FS_HZ / 2000)) / (FS_HZ / 1000);
    localparam CNT_MAX    = (DEB_US * (SYS_CLK_HZ / 1_000)) / 1_000 - 1;
    localparam DEB_W      = 16;

    // 八度钳位：超过 OCT_MAX 会跑到高频区（插值误差变大、也超出 freq_table）
    wire [2:0] oct_sel = (octave_sel > OCT_MAX) ? OCT_MAX : octave_sel;
    assign octave_out  = oct_sel;

    // ============================================================
    //  1) 采样节拍：48 kHz 单周期脉冲
    //     只用"计数器 + enable"，绝不门控时钟
    // ============================================================
    reg [11:0] tick_cnt;
    wire       tick = (tick_cnt == SAMPLE_DIV[11:0] - 12'd1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)    tick_cnt <= 12'd0;
        else if (tick) tick_cnt <= 12'd0;
        else           tick_cnt <= tick_cnt + 12'd1;
    end

    assign sample_tick = tick;

    // ============================================================
    //  2) 去抖：12 个弹音键 + 3 个档位键（各一个 debounce 实例）
    // ============================================================
    wire [11:0] key;
    wire [2:0]  mod_key;

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
    //  3) 键盘编码：位图 + 边沿 + 最高音优先
    // ============================================================
    wire [3:0] key_idx;

    key_encoder #(
        .SYNC_STAGES(2),
        .REVERSE_BIT_ORDER(0)
    ) u_enc (
        .clk           (clk),
        .rst_n         (rst_n),
        .key           (key),
        .key_state     (key_state),
        .key_press     (key_press),
        .key_release   (key_release),
        .active        (active),
        .active_press  (),              // 复音链路不用整体握手
        .active_release(),
        .note_num      (key_idx),
        .note_valid    (note_on)
    );

    // ============================================================
    //  4) 键内音符号 + 八度 -> MIDI 音符号
    //     midi = 12*(oct+3) + key_idx
    //     用移位加，不用 `*`：12*(n+3) = (n+3)<<3 + (n+3)<<2
    // ============================================================
    wire [2:0] oct_p3   = oct_sel + 3'd3;
    wire [6:0] oct_x12  = {oct_p3, 3'b000} + {oct_p3, 2'b00};

    assign midi_base = oct_x12;
    assign note_num  = oct_x12 + {4'b0000, key_idx};

    // ============================================================
    //  5) 调制档位 -> 颤音深度 / 滑音速度（逐字同 keyboard.v）
    //     P1 只用 amp_shift；glide_shift 透传（滑音推迟到 P3）
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

endmodule