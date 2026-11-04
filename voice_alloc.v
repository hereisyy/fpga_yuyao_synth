// ============================================================
//  voice_alloc.v —— P1-3：32 声部分配器 + 偷声 cross-fade
//
//  职责：把 12 个按键的 press / release 事件，变成每声部的
//        `gate`（电平）+ `note`（MIDI 号）+ `atten`（淡出系数）。
//        音高怎么算、波形怎么选都不归它管。
//
//  ── 分配优先级（冻结设计，Q1–Q21）────────────────────────
//    ① **优先空闲**：先槽 0..15（特殊池），再槽 16..31（波表池）。
//       理由是 ADR-0002：正常演奏时波表池基本闲置，优先占满特殊池
//       才能让"和弦里每个音的音色一致"。
//    ② **偷"最老且已 Release"**：正在衰减、马上就消失的槽最不疼。
//    ③ **偷最老 active**：实在没得偷，就偷最早分配的那个。
//       "最老"= `slot_age` 最小（时间戳小的先分配）。
//
//  ── ★ 偷声为什么必须淡出，以及"32 采样 cross-fade"在这里的确切含义 ──
//    ⚠️ 先说清一个物理限制，免得后面误以为这里偷工：
//       每个槽只有**一套**振荡器 + 包络。要"新旧两个音同时交叠淡化"
//       就必须再加一套硬件（ADR-0002 的 288/340 已经把 BSRAM 用满，
//       没有余量）。所以单槽能做到的是**交叉淡化的单硬件近似**：
//
//          淡出 32 帧（atten 64 → 0） → 切换音符 + 重触发包络（1 帧）
//          → 淡入 32 帧（atten 0 → 64）
//
//       切换发生在**atten 已经精确为 0** 的那一帧，此刻输出恒等于 0，
//       所以"频率突变"、"包络从 0 重新起"这两件事在听感上都不存在
//       —— 不会爆音，不会咔哒。代价是被偷的那个音有约 1.3 ms 的空档
//       （32+1+32 帧 @48 kHz）。这是所有单振荡器偷声的通行做法。
//
//       如果将来真要"真交叠"，正确做法是每槽再加一个"过渡振荡器"，
//       那是 P1 之后的资源问题，与本模块接口无关（只要把 atten 的
//       更新换成两条独立曲线即可）。
//
//    ⚠️ 为什么不硬切：旧音可能停在 +20000，新音从 -18000 起，
//       逐样本跳变 38000 —— 耳机里就是"啪"一声，频谱上一片宽带能量。
//       tb_voice_alloc.v 会**逐帧检查跳变量**，硬切的实现一定被抓住。
//
//  ── 时序 ──────────────────────────────────────────────────
//    全部在 `sample_tick` 域（48 kHz），每帧处理一个待分配按键。
//    12 键同时按下 → 12 帧分配完（250 µs），人耳无感。
//    淡出/淡入各 32 帧。
//
//  ── 与 voice_array 的接口 ────────────────────────────────
//    `voice_active`   = 包络非 IDLE（含 RELEASE 中）
//    `voice_releasing`= 包络处于 RELEASE 段 —— 偷声优先级的依据
//    槽回收条件：!held && !voice_active && 不在淡入淡出中
// ============================================================
module voice_alloc #(
    parameter integer NV         = 32,     // 声部数
    parameter integer SPECIAL    = 16,     // 特殊池大小（槽 0..SPECIAL-1）
    parameter integer NB         = 12,     // 弹音键数
    // 淡出系数：atten ∈ [0, ATTEN_FULL]，音量 = atten / ATTEN_FULL
    parameter integer ATTW       = 7,
    parameter integer ATTEN_FULL = 64,
    // 淡出/淡入各占多少帧 ⇒ 步长 = ATTEN_FULL / FADE_FRAMES
    parameter integer FADE_FRAMES = 32
)(
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    sample_tick,     // 48 kHz 单周期脉冲

    // ---- 演奏事件（来自 key_scan.v，高有效）----
    input  wire [NB-1:0]           key_state,       // 当前位图
    input  wire [NB-1:0]           key_press,       // 新按下，单周期
    input  wire [NB-1:0]           key_release,     // 新松开，单周期
    input  wire [6:0]              midi_base,       // 本八度最低音的 MIDI 号

    // ---- 来自 voice_array 的反馈（判偷声优先级 / 槽回收）----
    input  wire [NV-1:0]           voice_active,
    input  wire [NV-1:0]           voice_releasing,

    // ---- 给 voice_array 的控制 ----
    output wire [NV-1:0]           voice_gate,      // 电平：1 = 按住
    output wire [NV*7-1:0]         voice_note,      // 每槽 MIDI 号
    output wire [NV*ATTW-1:0]      voice_atten,     // 每槽淡出系数
    output wire [NV-1:0]           voice_valid,     // 每槽被占用

    // ---- 调试 ----
    output reg  [NV-1:0]           steal_pulse,     // 本帧发生偷声的槽
    output reg  [4:0]              last_steal,      // 最近一次偷的槽号
    output reg  [7:0]              n_used           // 当前占用槽数
);

    // 淡出步长（参数自检见下）
    localparam integer FADE_STEP = ATTEN_FULL / FADE_FRAMES;

    // synthesis translate_off
    initial begin
        if (ATTEN_FULL % FADE_FRAMES != 0) begin
            $display("!! voice_alloc: ATTEN_FULL=%0d 必须能被 FADE_FRAMES=%0d 整除",
                     ATTEN_FULL, FADE_FRAMES);
            $finish;
        end
        if (SPECIAL > NV) begin
            $display("!! voice_alloc: SPECIAL=%0d 超过 NV=%0d", SPECIAL, NV);
            $finish;
        end
    end
    // synthesis translate_on

    // ---- 槽状态 ----
    localparam [1:0] ST_IDLE = 2'd0,   // 正常发声（或空闲）
                     ST_OUT  = 2'd1,   // 偷声：淡出中
                     ST_SW   = 2'd2,   // 偷声：切换帧（gate 强制 0）
                     ST_IN   = 2'd3;   // 偷声：淡入中

    reg [6:0]        slot_note  [0:NV-1];
    reg [31:0]       slot_age   [0:NV-1];   // 分配时间戳，越小越老
    reg              slot_valid [0:NV-1];   // 槽被占用
    reg              slot_held  [0:NV-1];   // 该槽的键还按着
    reg [1:0]        slot_st    [0:NV-1];
    reg [6:0]        pend_note  [0:NV-1];   // 偷声时要切到的新音符
    reg [ATTW-1:0]   slot_atten [0:NV-1];

    // ---- 键 -> 槽 映射 ----
    reg [4:0]        key_slot [0:NB-1];
    reg              key_hit  [0:NB-1];

    // ---- 全局 ----
    reg [31:0]       now;                   // 帧计数器（时间戳源）
    reg [NB-1:0]     pend_press;            // 待分配的按键位图

    // ── ★ 边沿锁存（不能省）────────────────────
    //  key_scan 给的 key_press / key_release 是**任意时刻**的单周期脉冲，
    //  而本模块只在 sample_tick 那一拍看它们。两者不同步：
    //  tick 周期是 1042~3125 拍（50/150 MHz），脉冲只宽 1 拍 ⇒
    //  不锁存就会丢 99.9% 的按键。这个 bug 很隐蔽：大多数时候反正没声音，
    //  只是"偶尔按不响"。
    //
    //  ⚠️⚠️ 锁存必须和"tick 清零"错开写，否则会把**正好落在 tick 那一拍**
    //  的脉冲吞掉（2026-10 审查修掉的竞态，tb_voice_alloc 场景 0b 钉住）：
    //      原写法：press_lat <= press_lat | key_press;   // (A)
    //              if (sample_tick) press_lat <= 0;      // (B) 同块后写的赢
    //              ... 而分配逻辑读的是**旧的** press_lat ...
    //  ⇒ (A) 被 (B) 冲掉，脉冲既没进锁存器、也没被本拍的分配看到。
    //  症状：按下丢 = 偶尔按不响；松开丢 = **音一直响**（要等被偷声）。
    //  概率 = 1/SAMPLE_DIV ≈ 0.03%~0.1%，很难复现，但现场一定会遇到。
    //  现在改成：非 tick 拍只做 OR；tick 拍用 (锁存 | 当前脉冲) 消费。
    reg [NB-1:0]     press_lat, rel_lat;

    // 「本帧可消费的」按下 / 松开 = 已锁存的 ∪ **这一拍正在来的**
    //   后者是修上面那个竞态的关键：脉冲落在 tick 拍时也能被本帧看到。
    wire [NB-1:0]    press_now = press_lat | key_press;
    wire [NB-1:0]    rel_now   = rel_lat   | key_release;

    integer s, k;

    // ============================================================
    //  组合 ①：本帧要分配哪个键（**最低位优先**）
    //    从高往低扫，最后一个命中的就是最低位置位的那个 —— 先弹低音。
    // ============================================================
    reg [3:0] pr_idx;
    reg       pr_ok;

    // 本拍有效的待分配位图：排除同一拍已经松开的键。
    //  不排除的话，"上一拍入队、这一拍松开"的键会被强行分配，
    //  而释放事件已经消耗掉了 ⇒ 那个槽 held=1 永远不会释放，死锁。
    wire [NB-1:0] press_eff = pend_press & ~rel_now;

    always @(*) begin
        pr_idx = 4'd0;
        pr_ok  = 1'b0;
        for (k = NB-1; k >= 0; k = k - 1)
            if (press_eff[k]) begin
                pr_idx = k[3:0];
                pr_ok  = 1'b1;
            end
    end

    // ============================================================
    //  组合 ②：本帧的目标槽（空闲 → 偷 Release → 偷最老）
    //
    //  ⚠️⚠️ 2026-10-25 重写，**为了时序**（功能逐位不变）：
    //     原实现是"一次组合算完"—— 32 个槽 × 32 位比较器串成一条老化链，
    //     综合出 **237 级逻辑**，实测 Fmax 只有 **4.647 MHz**（要求 50 MHz），
    //     最差 setup slack **−195 ns**、2949 个端点违反。
    //
    //     现在改成 **帧内扫描**：每个 sample_tick 之后用 32 个时钟逐槽比较，
    //     每拍只做 1 个 32 位比较 + 几个 mux（约 2~3 ns），扫完把结果
    //     锁进 `tgt_q / tgt_free_q / tgt_ok_q`，供**下一帧**的 tick 使用。
    //
    //     为什么决策依据仍然等价：
    //       `slot_valid / slot_age / voice_releasing` 只在 tick 沿变化，
    //       而扫描**在 tick 之后重新开始**、33 拍内跑完（一帧 1042 拍），
    //       所以每一趟看到的是"上一帧 tick 之后"的**同一个稳定快照**，
    //       与原来在 tick 那一拍做组合判断的输入完全相同。
    //       并列时取低索引（严格 `<` 比较、升序扫描），也与原逻辑一致。
    // ============================================================
    localparam integer NV_LAST = NV - 1;

    localparam [1:0] SC_SCAN = 2'd0, SC_LATCH = 2'd1, SC_IDLE = 2'd2;

    reg [1:0]  sc_st;
    reg [4:0]  sc_i;
    reg [4:0]  sc_free;    reg sc_free_ok;
    reg [4:0]  sc_rel_s;   reg sc_rel_ok;  reg [31:0] sc_rel_age;
    reg [4:0]  sc_any_s;   reg sc_any_ok;  reg [31:0] sc_any_age;

    reg [4:0]  tgt_q;          // 目标槽（下一帧 tick 使用）
    reg        tgt_free_q;     // 1 = 空闲槽，0 = 偷
    reg        tgt_ok_q;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sc_st      <= SC_SCAN;
            sc_i       <= 5'd0;
            sc_free_ok <= 1'b0;  sc_free    <= 5'd0;
            sc_rel_ok  <= 1'b0;  sc_rel_s   <= 5'd0;  sc_rel_age <= 32'hFFFFFFFF;
            sc_any_ok  <= 1'b0;  sc_any_s   <= 5'd0;  sc_any_age <= 32'hFFFFFFFF;
            tgt_q      <= 5'd0;
            tgt_free_q <= 1'b0;
            tgt_ok_q   <= 1'b0;
        end else if (sample_tick) begin
            // ---- 新帧：从头扫一遍（本帧 tick 用的是上一帧扫出来的 tgt_q）----
            sc_st      <= SC_SCAN;
            sc_i       <= 5'd0;
            sc_free_ok <= 1'b0;
            sc_rel_ok  <= 1'b0;  sc_rel_age <= 32'hFFFFFFFF;
            sc_any_ok  <= 1'b0;  sc_any_age <= 32'hFFFFFFFF;
        end else begin
            case (sc_st)
            SC_SCAN: begin
                // ① 空闲槽：升序扫，第一个空闲即最低索引
                //    （特殊池是槽 0..15，低索引天然优先 ⇒ 与原"特殊池优先"一致）
                if (!slot_valid[sc_i] && !sc_free_ok) begin
                    sc_free    <= sc_i;
                    sc_free_ok <= 1'b1;
                end
                // ② 偷"最老且已 Release"（严格小于 ⇒ 并列取低索引）
                if (slot_valid[sc_i] && voice_releasing[sc_i] &&
                    (slot_age[sc_i] < sc_rel_age)) begin
                    sc_rel_s   <= sc_i;
                    sc_rel_age <= slot_age[sc_i];
                    sc_rel_ok  <= 1'b1;
                end
                // ③ 偷最老 active
                if (slot_valid[sc_i] && (slot_age[sc_i] < sc_any_age)) begin
                    sc_any_s   <= sc_i;
                    sc_any_age <= slot_age[sc_i];
                    sc_any_ok  <= 1'b1;
                end

                if (sc_i == NV_LAST[4:0]) sc_st <= SC_LATCH;
                else                      sc_i  <= sc_i + 5'd1;
            end

            SC_LATCH: begin
                // 锁存本帧扫描结果，供下一帧 tick 使用
                tgt_free_q <= sc_free_ok;
                tgt_ok_q   <= sc_free_ok | sc_rel_ok | sc_any_ok;
                tgt_q      <= sc_free_ok ? sc_free
                                         : (sc_rel_ok ? sc_rel_s : sc_any_s);
                sc_st      <= SC_IDLE;      // 等下一个 tick 再扫（一帧只扫一趟）
            end

            default: ;                      // SC_IDLE
            endcase
        end
    end

    wire do_steal = pr_ok && tgt_ok_q && !tgt_free_q;

    // ============================================================
    //  时序：全在 sample_tick 域
    // ============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            now        <= 32'd0;
            pend_press <= {NB{1'b0}};
            press_lat  <= {NB{1'b0}};
            rel_lat    <= {NB{1'b0}};
            steal_pulse<= {NV{1'b0}};
            last_steal <= 5'd0;
            for (s = 0; s < NV; s = s + 1) begin
                slot_note[s]  <= 7'd0;
                slot_age[s]   <= 32'd0;
                slot_valid[s] <= 1'b0;
                slot_held[s]  <= 1'b0;
                slot_st[s]    <= ST_IDLE;
                pend_note[s]  <= 7'd0;
                slot_atten[s] <= ATTEN_FULL[ATTW-1:0];
            end
            for (k = 0; k < NB; k = k + 1) begin
                key_slot[k] <= 5'd0;
                key_hit[k]  <= 1'b0;
            end
        end else begin
            steal_pulse <= {NV{1'b0}};

            if (sample_tick) begin
                // tick 拍：消费 press_now / rel_now（含这一拍正在来的脉冲），
                // 然后把锁存器清零。两个动作分别在两个分支里，
                // **不会**再出现"清零覆盖 OR"的同块竞态。
                press_lat <= {NB{1'b0}};
                rel_lat   <= {NB{1'b0}};

                now <= now + 32'd1;

                // ----------------------------------------------
                //  (1) 新按下 -> 进待分配队列
                //      已在队列里就不再重复入队（避免同一键占两个槽）
                // ----------------------------------------------
                for (k = 0; k < NB; k = k + 1)
                    if (press_now[k] && !key_hit[k])
                        pend_press[k] <= 1'b1;

                // ----------------------------------------------
                //  (2) 松键 -> 该槽进 RELEASE，键-槽映射立刻失效
                //      ⚠️ 槽本身**不收**，等包络 active 掉下来再回收
                //         （否则释放尾音会被掐断）
                // ----------------------------------------------
                //  ⚠️ 还要清 pend_press：如果一个键在**同一个 tick 窗口内**
                //     按下又松开，它会同时在 press_lat 和 rel_lat 里；
                //     先入队再被释放丢棉花的话，那个槽会永远 held=1 卡死。
                for (k = 0; k < NB; k = k + 1)
                    if (rel_now[k]) begin
                        pend_press[k] <= 1'b0;
                        if (key_hit[k]) begin
                            slot_held[key_slot[k]] <= 1'b0;
                            key_hit[k]             <= 1'b0;
                        end
                    end

                // ----------------------------------------------
                //  (3) 分配 / 偷声（每帧最多一个）
                // ----------------------------------------------
                //  偷声时这个槽可能还属于另一个按着的键；
                //  不清掉它的 key_hit，就会两个键指向同一个槽，
                //  后面松掉任一个都会把另一个的音带死。
                //  （本循环写在 (3) 之前；同一目标的后写者赢，
                //    所以 (3) 里对 pr_idx 的 key_hit<=1 依然生效。）
                if (pr_ok && tgt_ok_q && !tgt_free_q)
                    for (k = 0; k < NB; k = k + 1)
                        if (key_hit[k] && (key_slot[k] == tgt_q))
                            key_hit[k] <= 1'b0;

                if (pr_ok && tgt_ok_q) begin
                    pend_press[pr_idx] <= 1'b0;
                    key_slot[pr_idx]   <= tgt_q;
                    key_hit[pr_idx]    <= 1'b1;

                    if (tgt_free_q) begin
                        // 空闲槽：直接起音
                        slot_note[tgt_q]  <= midi_base + {3'b000, pr_idx};
                        slot_age[tgt_q]   <= now;
                        slot_valid[tgt_q] <= 1'b1;
                        slot_held[tgt_q]  <= 1'b1;
                        slot_st[tgt_q]    <= ST_IDLE;
                        slot_atten[tgt_q] <= ATTEN_FULL[ATTW-1:0];
                    end else begin
                        // 偷声：先淡出，切换动作在 ST_SW 那一帧做
                        pend_note[tgt_q]  <= midi_base + {3'b000, pr_idx};
                        slot_held[tgt_q]  <= 1'b1;
                        slot_st[tgt_q]    <= ST_OUT;
                        last_steal      <= tgt_q;
                        steal_pulse[tgt_q]<= 1'b1;
                    end
                end

                // ----------------------------------------------
                //  (4) 每槽的淡出/淡入状态机
                //      ⚠️ 跳过本帧刚被启动偷声的那个槽：否则下面这一拍
                //         就会用**旧的** st 值再算一遍，把 (3) 设的
                //         ST_OUT 覆盖掉（同一个 always 块里后写的赢）。
                // ----------------------------------------------
                for (s = 0; s < NV; s = s + 1)
                    if (!(do_steal && (tgt_q == s[4:0]))) begin
                        case (slot_st[s])
                            ST_OUT: begin
                                if (slot_atten[s] <= FADE_STEP[ATTW-1:0]) begin
                                    slot_atten[s] <= {ATTW{1'b0}};   // 精确到 0
                                    slot_st[s]    <= ST_SW;
                                end else
                                    slot_atten[s] <= slot_atten[s] - FADE_STEP[ATTW-1:0];
                            end

                            ST_SW: begin
                                // 此刻 atten == 0，输出**恒为 0** =>
                                // 改音符、重触发包络都不会产生任何跳变。
                                // （包络真的从 0 起不是必要的：十分钟后 atten 从 0
                                //   淡入，幅度 = gain × atten 仍然从 0 平滑上升。
                                //  gate 在 ST_SW 拉低一帧只是为了让 adsr 从 RELEASE/
                                //  SUSTAIN 重新进 ATTACK，让新音有自己的起音形状。）
                                slot_note[s] <= pend_note[s];
                                slot_age[s]  <= now;
                                slot_st[s]   <= ST_IN;
                            end

                            ST_IN: begin
                                if (slot_atten[s] >= (ATTEN_FULL[ATTW-1:0] - FADE_STEP[ATTW-1:0]))
                                    begin
                                        slot_atten[s] <= ATTEN_FULL[ATTW-1:0];
                                        slot_st[s]    <= ST_IDLE;
                                    end else
                                        slot_atten[s] <= slot_atten[s] + FADE_STEP[ATTW-1:0];
                            end

                            default: ;   // ST_IDLE：不动
                        endcase
                    end

                // ----------------------------------------------
                //  (5) 槽回收：键已松、包络已归零、且不在淡入淡出中
                // ----------------------------------------------
                for (s = 0; s < NV; s = s + 1)
                    if (slot_valid[s] && !slot_held[s] && !voice_active[s]
                        && (slot_st[s] == ST_IDLE))
                        slot_valid[s] <= 1'b0;

                //  (6) 占用槽数不在这里算，见文件末尾的组合求和。
                //  ⚠️ 为什么不能放在 tick 分支里：slot_valid 本身也是非阻塞赋值，
                //     tick 那一拍算出来的是**上一帧**的占用数，再加上
                //     非阻塞赋值本身还要晚一拍，结果就是慢两拍才反映。
                //     直接组合求和，永远和 slot_valid 同步。
            end else begin
                // 非 tick 拍：只收边沿，不清零（清零只在 tick 拍做）
                press_lat <= press_lat | key_press;
                rel_lat   <= rel_lat   | key_release;
            end
        end
    end

    // ============================================================
    //  输出打包
    //  gate：按住 且 不在切换帧（切换帧必须 gate=0 才能产生重触发上升沿）
    // ============================================================
    genvar gi;
    generate
        for (gi = 0; gi < NV; gi = gi + 1) begin : g_out
            assign voice_gate[gi] = slot_held[gi] && (slot_st[gi] != ST_SW);

            assign voice_note[gi*7 +: 7]          = slot_note[gi];
            assign voice_atten[gi*ATTW +: ATTW]   = slot_atten[gi];
            assign voice_valid[gi]                = slot_valid[gi];
        end
    endgenerate

    // ============================================================
    //  占用槽数（组合求和，与 slot_valid 同步）
    // ============================================================
    integer cnt_used;
    always @(*) begin
        cnt_used = 0;
        for (s = 0; s < NV; s = s + 1)
            if (slot_valid[s]) cnt_used = cnt_used + 1;
        n_used = cnt_used[7:0];
    end

    // key_state 目前不参与决策（按键的"当前状态"已经由 press/release
    // 边沿和 key_hit 完整表达）。留着端口是为了将来接"全松时停 LFO"之类的
    // 全局策略，以及让顶层接线时不必再改一次接口。
    // synthesis translate_off
    always @(posedge clk) if (rst_n === 1'b1) begin
        if ((^key_state) === 1'bx)
            $display("!! voice_alloc: key_state 出现 X");
    end
    // synthesis translate_on

endmodule
