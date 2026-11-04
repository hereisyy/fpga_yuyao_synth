// ============================================================
//  test_mode.v —— P1-4：演示态（现场拿分关键）
//
//  一句话：拿一个按键（`EX_KEY.0` / AA13）把整个合成器切进"数峰模式"，
//  让评委在频谱上**一眼数出** 1/2/4/8/16/32 个峰。
//
//  ── ★ 四条硬约束（Q22，逐条都在下面有对应实现，改之前先想清楚）──────
//
//   ① **强制 `wave_sel = 0`、`mod_index = 0`、特殊池切 FM 态、关颤音**
//      演示频率必须落在**每一根谱线上只有一个峰**。只要混进 FM 边带
//      （mod_index ≠ 0）或锯齿波（wave_sel ≠ 0）的谐波，峰数就不等于
//      声部数了。颤音更糟：它把一根谱线抹成一团，FFT 上直接看不出来。
//      ⇒ 输出 `force_sine` / `mod_zero` / `force_fm` / `no_vibrato`，由顶层 mux 覆盖。
//
//      ⚠️ `force_fm` 这一条最容易漏，漏了峰数直接翻倍：特殊池（槽 0..15）
//         在**加法态**下每声部输出 f 与 2f 两个正弦 ⇒ 32 声部档 = 16×2+16×1
//         = **48 根谱线**。必须切到 FM 态且 mod_index=0，特殊池才只剩载波
//         一根（调制量为 0 就是纯正弦）。顶层接法：
//             special_mode = force_fm ? 1'b1 : timbre_ab
//         （2026-10 审查：这条约束原先只活在 tb 里，没有端口，已补上。）
//
//   ② **每声部幅度按 1/N 缩放**
//      32 路满幅正弦直接相加必削波；削波 = 方波 = 无穷多奇次谐波，
//      峰数立刻炸到几百根。1/N 缩放后总和恒 ≤ 单路满幅，永不削波。
//      ⇒ 输出 `voice_amp_shift` = log2(N) ∈ {0,1,2,3,4,5}。
//        N 只取 2 的幂，所以"1/N"就是右移 log2(N) 位 —— **0 个 DSP**。
//
//   ③ **频率表 = 30 Hz × 前 32 个素数**（60, 90, 150 … 3930 Hz）
//      为什么是素数：任意两个声部的频率比是无理数近似，它们的谐波
//      **永不重合**，FFT 上就是 32 根互不相干的谱线，能一根一根数。
//      如果改用等间距（比如 55×k Hz），谐波会大量重合，数出来远少于
//      32 根 —— 这正是以前踩过的坑。**不要改成 55 Hz。**
//      频率字 = round(f × 2^32 / 48000)，由 Python 独立算出后硬编码。
//
//   ④ **包络停在 sustain**
//      演示态只要"平稳的正弦"，不要起落。⇒ 输出的 `voice_gate` 是
//      **电平**恒 1（不是单周期脉冲），adsr 走到 SUSTAIN 就停住。
//      注意：这里必须区分两个 gate —— `osc_dds.gate` 要恒 1（自由跑），
//      `adsr.gate` 才是这个电平。铁律②，两者绝不接同一个信号。
//
//  ── 档位循环（冻结设计，`EX_KEY.0` 接管式）────────────────────
//     按第 1 下 → 1 声部；第 2 下 → 2；3→4；4→8；5→16；6→32（满）；
//     **第 7 下退出演示**（回到正常演奏 32 声部）。
//     退出后 `demo_en = 0`，顶层把控制权还给 `voice_alloc`。
//
//  ── 演示态必须旁路混响（`接口契约.md` §2）────────────────────
//     32 根稳态正弦进混响，梳状滤波会长出旁瓣，评委可能把峰数数错。
//     ⇒ 输出 `fx_bypass = 1`。⚠️ 旁路只许改变"湿声是否加入"，
//       **绝不许改变任何延迟拍数**，否则切换瞬间会爆音/错位。
//
//  ── 输出是"覆盖层"，不是"取代 voice_alloc"──────────────────
//     本模块不碰分配逻辑，只提供一套覆盖信号，由顶层按 `demo_en` 二选一。
//     这样 32 声部硬件只有一套，正常演奏与演示共用（ADR-0002 的资源
//     预算是按这个前提算的）。
// ============================================================
module test_mode #(
    parameter integer NV      = 32,   // 声部数
    parameter integer FW_W    = 32,   // 频率字位宽
    parameter integer AMPW    = 3    // amp_shift 位宽（要能装下 log2(32)=5）
    //  （曾有一个 PRESS_CYC 参数号称"按键重复保护"，但全文没用到；
    //    按键是边沿触发 + demo_key 是去抖后的电平，按住本来就不会连跳，
    //    重复保护没有存在必要，2026-10 审查时删掉。）
)(
    input  wire                clk,
    input  wire                rst_n,

    // 演示键：**已经去抖的电平**（1 = 按下）。边沿检测在本模块做。
    input  wire                demo_key,

    // ---- 给顶层的覆盖信号 ----
    output reg                 demo_en,       // 1 = 演示态生效
    output reg  [2:0]          demo_sel,      // 0 = 关闭；1..6 = 档位
    output reg  [5:0]          n_voices,      // 实际放出的声部数 1..32

    output wire [NV-1:0]       voice_gate,    // 电平恒 1（前 n_voices 个）
    output wire [NV*FW_W-1:0]  voice_freq,    // 30 Hz × 素数表
    output wire [NV*AMPW-1:0]  voice_amp_shift, // = log2(N)：1/N 缩放

    output wire                force_sine,    // ① 强制 wave_sel = 0
    output wire                mod_zero,      // ① 强制 mod_index = 0
    output wire                force_fm,      // ① 特殊池强制切 FM 态（见文件头）
    output wire                no_vibrato,    // ① 关颤音
    output wire                fx_bypass,     // 旁路混响
    output wire [2:0]          led_rgb        // 档位指示（R/G/B）
);

    // ============================================================
    //  1) 频率表：30 Hz × 前 32 个素数
    //     期望值由 Python 独立算出（round(f * 2^32 / 48000)），
    //     不在这里做任何运算 —— 反算核对见 tb_test_mode.v 场景 3。
    //
    //   idx  素数  频率(Hz)    频率字
    //     0    2      60      0x0051EB85
    //     1    3      90      0x007AE148
    //     2    5     150      0x00CCCCCD
    //     3    7     210      0x011EB852
    //     4   11     330      0x01C28F5C
    //     5   13     390      0x02147AE1
    //     6   17     510      0x02B851EC
    //     7   19     570      0x030A3D71
    //     8   23     690      0x03AE147B
    //     9   29     870      0x04A3D70A
    //    10   31     930      0x04F5C28F
    //    11   37    1110      0x05EB851F
    //    12   41    1230      0x068F5C29
    //    13   43    1290      0x06E147AE
    //    14   47    1410      0x07851EB8
    //    15   53    1590      0x087AE148
    //    16   59    1770      0x0970A3D7
    //    17   61    1830      0x09C28F5C
    //    18   67    2010      0x0AB851EC
    //    19   71    2130      0x0B5C28F6
    //    20   73    2190      0x0BAE147B
    //    21   79    2370      0x0CA3D70A
    //    22   83    2490      0x0D47AE14
    //    23   89    2670      0x0E3D70A4
    //    24   97    2910      0x0F851EB8
    //    25  101    3030      0x1028F5C3
    //    26  103    3090      0x107AE148
    //    27  107    3210      0x111EB852
    //    28  109    3270      0x1170A3D7
    //    29  113    3390      0x12147AE1
    //    30  127    3810      0x1451EB85
    //    31  131    3930      0x14F5C28F
    //
    //  表格是常量函数：idx 在 generate 里是常量，综合时折叠成连线，
    //  **不占任何逻辑**（也不要为了"省"去改成 RAM 查表，那要额外 1 块
    //  BSRAM，而 ADR-0002 的 288/340 已经没有余量了）。
    // ============================================================
    function [FW_W-1:0] prime_freq_word;
        input [4:0] idx;
        begin
            case (idx)
                5'd0 : prime_freq_word = 32'h0051EB85;   //  60 Hz = 30x2
                5'd1 : prime_freq_word = 32'h007AE148;   //  90 Hz = 30x3
                5'd2 : prime_freq_word = 32'h00CCCCCD;   // 150 Hz = 30x5
                5'd3 : prime_freq_word = 32'h011EB852;   // 210 Hz = 30x7
                5'd4 : prime_freq_word = 32'h01C28F5C;   // 330 Hz = 30x11
                5'd5 : prime_freq_word = 32'h02147AE1;   // 390 Hz = 30x13
                5'd6 : prime_freq_word = 32'h02B851EC;   // 510 Hz = 30x17
                5'd7 : prime_freq_word = 32'h030A3D71;   // 570 Hz = 30x19
                5'd8 : prime_freq_word = 32'h03AE147B;   // 690 Hz = 30x23
                5'd9 : prime_freq_word = 32'h04A3D70A;   // 870 Hz = 30x29
                5'd10: prime_freq_word = 32'h04F5C28F;   // 930 Hz = 30x31
                5'd11: prime_freq_word = 32'h05EB851F;   //1110 Hz = 30x37
                5'd12: prime_freq_word = 32'h068F5C29;   //1230 Hz = 30x41
                5'd13: prime_freq_word = 32'h06E147AE;   //1290 Hz = 30x43
                5'd14: prime_freq_word = 32'h07851EB8;   //1410 Hz = 30x47
                5'd15: prime_freq_word = 32'h087AE148;   //1590 Hz = 30x53
                5'd16: prime_freq_word = 32'h0970A3D7;   //1770 Hz = 30x59
                5'd17: prime_freq_word = 32'h09C28F5C;   //1830 Hz = 30x61
                5'd18: prime_freq_word = 32'h0AB851EC;   //2010 Hz = 30x67
                5'd19: prime_freq_word = 32'h0B5C28F6;   //2130 Hz = 30x71
                5'd20: prime_freq_word = 32'h0BAE147B;   //2190 Hz = 30x73
                5'd21: prime_freq_word = 32'h0CA3D70A;   //2370 Hz = 30x79
                5'd22: prime_freq_word = 32'h0D47AE14;   //2490 Hz = 30x83
                5'd23: prime_freq_word = 32'h0E3D70A4;   //2670 Hz = 30x89
                5'd24: prime_freq_word = 32'h0F851EB8;   //2910 Hz = 30x97
                5'd25: prime_freq_word = 32'h1028F5C3;   //3030 Hz = 30x101
                5'd26: prime_freq_word = 32'h107AE148;   //3090 Hz = 30x103
                5'd27: prime_freq_word = 32'h111EB852;   //3210 Hz = 30x107
                5'd28: prime_freq_word = 32'h1170A3D7;   //3270 Hz = 30x109
                5'd29: prime_freq_word = 32'h12147AE1;   //3390 Hz = 30x113
                5'd30: prime_freq_word = 32'h1451EB85;   //3810 Hz = 30x127
                5'd31: prime_freq_word = 32'h14F5C28F;   //3930 Hz = 30x131
                default: prime_freq_word = 32'h0;
            endcase
        end
    endfunction

    // ============================================================
    //  2) 档位状态机
    //     sel: 0 = 正常演奏；1..6 = 1/2/4/8/16/32 声部
    //     第 7 下（从 6 再按一次）回到 0。
    // ============================================================
    //  ⚠️ 边沿检测必须**寄存化**，不能用组合 wire。
    //     本模块第一版写的是：
    //         reg  demo_key_d;
    //         wire demo_rise = demo_key & ~demo_key_d;
    //         always @(posedge clk) begin
    //             demo_key_d <= demo_key;      // 同一块里的非阻塞赋值
    //             if (demo_rise) ...           // 这里读 demo_rise
    //         end
    //     结果 demo_sel 永远不走（0 → 0 → 0）。原因：iverilog 在沿上求值这个
    //     always 块时，同块里 NBA 队列中的 demo_key_d 已经反映到 wire 上了，
    //     phase 组合得到的永远是 "按下之后"的 0。它也不报错，只是静默失效。
    //     → 用一个寄存器 demo_rise_r 把边沿存下来（延一拍，功能不变）。
    reg  demo_key_d;
    reg  demo_rise_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            demo_key_d  <= 1'b0;
            demo_rise_r <= 1'b0;
            demo_sel    <= 3'd0;
            demo_en     <= 1'b0;
        end else begin
            demo_key_d  <= demo_key;
            demo_rise_r <= demo_key & ~demo_key_d;
            if (demo_rise_r) begin
                if (demo_sel == 3'd6) begin
                    demo_sel <= 3'd0;      // 第 7 下：退出演示
                    demo_en  <= 1'b0;
                end else begin
                    demo_sel <= demo_sel + 3'd1;
                    demo_en  <= 1'b1;
                end
            end
        end
    end

    // ============================================================
    //  3) 档位 -> 声部数 / 幅度右移 / LED
    //     声部数只取 2 的幂，所以 amp_shift 就是 log2(N)：
    //        N= 1 -> 0  (>>>0 = 1/1)
    //        N= 2 -> 1
    //        N= 4 -> 2
    //        N= 8 -> 3
    //        N=16 -> 4
    //        N=32 -> 5
    // ============================================================
    always @(*) begin
        case (demo_sel)
            3'd1: begin n_voices = 6'd1;  end
            3'd2: begin n_voices = 6'd2;  end
            3'd3: begin n_voices = 6'd4;  end
            3'd4: begin n_voices = 6'd8;  end
            3'd5: begin n_voices = 6'd16; end
            3'd6: begin n_voices = 6'd32; end
            default: begin n_voices = 6'd0; end
        endcase
    end

    // LED：demo_sel 直接当 RGB 用 —— 0..6 正好是
    //   000 全灭 / 001 R / 010 G / 011 R+G / 100 B / 101 R+B / 110 G+B
    // 六档 + 全灭，天然对应，不需要额外的译码表。
    // （110 之后没有 111，因为最高档就是 6，全亮留给别的用途。）
    assign led_rgb = demo_sel;

    // ============================================================
    //  4) 每声部的 gate / 频率 / 幅度
    // ============================================================
    genvar gi;
    generate
        for (gi = 0; gi < NV; gi = gi + 1) begin : g_voice
            // ① gate 是**电平**：声部号 < n_voices 时恒 1。
            //    恒 1 ⇒ adsr 走完 ATTACK/DECAY 后停在 SUSTAIN（约束④），
            //    不会进 RELEASE，也不会重复起音。
            assign voice_gate[gi] = (gi < n_voices);

            // ③ 每声部一个素数频率
            assign voice_freq[gi*FW_W +: FW_W] = prime_freq_word(gi[4:0]);

            // ② 1/N 缩放 = 右移 log2(N)
            assign voice_amp_shift[gi*AMPW +: AMPW] = amp_shift_of(demo_sel);
        end
    endgenerate

    function [AMPW-1:0] amp_shift_of;
        input [2:0] sel;
        begin
            case (sel)
                3'd1:    amp_shift_of = 3'd0;   // 1/1
                3'd2:    amp_shift_of = 3'd1;   // 1/2
                3'd3:    amp_shift_of = 3'd2;   // 1/4
                3'd4:    amp_shift_of = 3'd3;   // 1/8
                3'd5:    amp_shift_of = 3'd4;   // 1/16
                3'd6:    amp_shift_of = 3'd5;   // 1/32
                default: amp_shift_of = 3'd0;
            endcase
        end
    endfunction

    // ============================================================
    //  5) 四条"强制"信号 + 混响旁路
    //     演示态下全部有效；退出演示后全部放开。
    // ============================================================
    assign force_sine = demo_en;   // ① wave_sel 强制 0（纯正弦）
    assign mod_zero   = demo_en;   // ① mod_index 强制 0（无 FM 边带）
    //  ① **特殊池必须切 FM 态**。这一条以前只写在 tb 里（没有输出端口），
    //    顶层很容易漏 —— 漏了的后果是"32 声部档数出 48 根谱线"：
    //    特殊池在加法态下每声部出 f 与 2f **两个**正弦，16×2 + 16×1 = 48。
    //    接法：顶层 `special_mode = demo_en ? 1'b1 : timbre_ab`。
    assign force_fm   = demo_en;
    assign no_vibrato = demo_en;   // ① 关颤音（谱线不许被抹宽）
    assign fx_bypass  = demo_en;   // 旁路混响（防止梳状旁瓣）

endmodule