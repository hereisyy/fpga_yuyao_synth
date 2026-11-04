`timescale 1ns/1ps
// ============================================================
//  top_console.v —— ★ 全系统顶层（P1-6）：Console 138K 上真正要烧的那一层
//
//  这是一个**只有配置 + 连线 + 上电处理**的文件，不含新算法。
//
//  ── 与 P0 的 `top_synth.v` 的区别（它不是交付顶层，只留作基线）──
//     · 引脚换成 Console 实际的三组：1 颗用户 RGB / 3 个板载键 / PMOD 输入
//     · 32 声部：key_scan → voice_alloc → voice_array → mixer_tree
//     · 输入换成 P2：MPR121 触摸 12 键 + 2 个八度键 + 1 个滑音开关
//     · 演示态（test_mode）真正接进数据通路：4 个覆盖信号全部生效
//     · 补上 P1 缺的那一级：**32 路 MIDI → 频率字**（见 §4）
//     · 补上 P2 的滑音本体：mono/legato 通路（见 §6）
//     · 串口状态行（115200），板上只有 1 颗 RGB，没有它没法调试
//
//  ── 数据通路（一帧 = 一个 sample_tick）────────────────────
//
//   板载键 EX_KEY.0 ─debounce→ test_mode ──┐（演示覆盖信号）
//   EX_KEY.1       ─debounce→ 音色 0/1    │
//                                        ↓
//   MPR121 ─I2C→ p2_input_top ─key_raw→ key_scan ─key_press/release→ voice_alloc
//                    └octave_sel→        │                          │
//                                        └midi_base──────────────────┤
//                                                                    ↓
//        voice_alloc: gate / note(MIDI) / atten          （每槽一套）
//                            │
//               note ─→ §4 32 路 MIDI→频率字（1 个 freq_table 轮扫）
//                            │
//                     §5 32 × freq_mod（全局 LFO 颤音）
//                            │
//        test_mode 覆盖 ──→ param_xfer（帧边界对齐）──→ voice_array
//                                                       （32 osc + 32 env）
//                            └voice_active/releasing→ voice_alloc（偷声依据）
//                                    │
//                            mixer_tree（32→1，可整帧右移）
//                                    │
//                          软启动斜坡（10 ms）→ 寄存 → i2s_tx → MAX98357A
//
//  ── 五条铁律（本文件逐条落实，写错任何一条都出诡异问题）──────
//   ① `osc_dds.gate` 恒 1（在 voice_array 内部接死了，这里管不着也不用管）
//   ② `adsr.gate` 是**电平**，来自 voice_alloc 的 `slot_held`，不是脉冲
//   ③ 送进 `i2s_tx` 的数据必须**寄存过**（下面 att_r）
//   ④ `PA_EN` 在 Console 上接 MAX98357A 的 `SD_MODE`，**高电平才开**
//      （60K 时代是低有效；抄错 = 代码全对但没声音）
//   ⑤ `sys_rst_n = F4` 在 BANK7 = 1.5 V，`.cst` 必须写 `LVCMOS15`（不归本文件管，
//      但它是"上电没反应"的头号原因，记在这里免得有人去改代码）
//
//  ── 上板前必须确认的三件事 ────────────────────────────────
//    1. `SYS_CLK_HZ` = 实测系统时钟（Console = SOM 的 Y1100 50 MHz）
//       ⚠️ 这个参数错了不会报错，只会"整体高/低一个八度"
//    2. 如果要用 rPLL 150 MHz：把 `clk_sys` 换成 PLL 输出，
//       `SYS_CLK_HZ` 改 150_000_000，`pll_lock` 并进复位（见 §1）
//    3. `.cst` 里的 P2 引脚块（PMOD）是**待确认**的接法，见 README
// ============================================================
`default_nettype none

module top_console #(
    // ---- 时钟 / 采样 ----
    parameter integer SYS_CLK_HZ   = 50_000_000,   // ⚠️ 必须与实测一致
    parameter integer FS_HZ        = 48_000,
    // ---- 去抖 ----
    parameter integer DEB_US       = 82,           // 演奏键（key_scan 内部）
    parameter integer DEMO_DEB_US  = 20_000,       // 面板演示键：20 ms，防误触
    // ---- 上电 ----
    parameter integer RAMP_MS      = 10,           // 软启动，防"砰"
    // ---- 音量 / 混音 ----
    //  32 路满幅相加会削顶。MIX_SHIFT=1 → 整帧右移 1 位（−6 dB），
    //  再靠 mixer_tree 的硬饱和兜底（sat_flag 会点灯 + 报串口）。
    //  ★ 2026-11-03：MIX_SHIFT 试过 0（+6 dB）⇒ 干声满幅 + 湿声叠加 ⇒ **每音削波（齿声）**，
    //     已改回 **1**（保留 6 dB 余量）。音量已由 atten_scale 归一化修正拿到 +6 dB。
    //     ⚠️ 想更响请改硬件（MAX98357A 的 GAIN 脚），不要再降 MIX_SHIFT。
    // 正常演奏时波表池用的波形页（0=正弦 1=方波 2=锯齿 3=三角）
    parameter integer MIX_SHIFT    = 1,
    parameter [1:0]   POLY_WAVE    = 2'd0,
    // ---- 颤音（全局 LFO，每声部各乘一次）----
    parameter         VIB_ON       = 1,            // 0 = 彻底关（省 32 个乘法器）
    parameter [3:0]   VIB_SHIFT    = 4'd7,         // 深度：2^-7 ≈ ±0.78%
    // ---- FM 音色（特殊池）----
    parameter [15:0]  MOD_INDEX    = 16'd6000,
    // ---- P2 / I2C ----
    parameter [6:0]   I2C_DEV      = 7'h5A,        // MPR121：ADDR 接 GND ⇒ 0x5A
    parameter integer SCL_HZ       = 25_000,   // ★ 2026-10-30：100k→25k，容忍模块弱上拉
    // ---- 八度上限（与 P2 的 octave_key 一致：1..4）----
    parameter [2:0]   OCT_MAX      = 3'd4,
    // ---- 板载键极性（Console：内部上拉 ⇒ 低有效）----
    parameter         KEY_ACTIVE_LOW = 1,
    // ---- 波表文件 ----
    //  ⚠️ `$readmemh` 的路径是相对**仿真器工作目录**（综合时相对工程目录）找的，
    //     找不到**不报错**、ROM 全 X（仿真里满屏 X，上板全静音）。
    //     上板时 hex 与本文件同目录 ⇒ 保持默认即可；
    //     在 `08_P1\` 下跑仿真时，tb 要显式传 `"../06_keyboard/wavetable_all.hex"`。
    parameter         INIT_FILE = "wavetable_all.hex",
    // ---- 仿真专用：1 = 不例化 i2s_tx（上板/综合必须 0）----
    parameter integer SIM_SKIP_I2S = 0,
    // ---- 串口 ----
    parameter integer BAUD         = 115_200,
    //  ★ 2026-10-30：I2C 时钟从 100 kHz 降到 **25 kHz**。
    //     原因：如果模块板上没有 4.7K 上拉（或只有很弱的），100 kHz 的上升沿根本来不及；
    //     25 kHz 对上升时间的要求宽松 4 倍，能容忍弱上拉。配置时间从 ~5 ms 变 ~20 ms，无所谓。
    parameter integer HEARTBEAT    = 50_000_000    // 状态行心跳（拍）@50MHz = 1 s
)(
    // ---- 系统 ----
    input  wire        sys_clk,        // V22（SOM 的 50 MHz 晶振）
    input  wire        sys_rst_n,      // F4（BANK7，1.5 V，低有效）

    // ---- 板载键（Console：上拉，低有效）----
    //   [0] EX_KEY.0 = 演示档位循环   [1] EX_KEY.1 = 音色（加法 ↔ FM）
    //   ⚠️ 只有 2 个：`EX_KEY.2 = Y12` 原理图上**没接按键**，
    //      所以端口只留 2 位 —— 留着第 3 位会被综合器裁掉，
    //      而 `.cst` 里再约束它就会报"对象找不到"。
    input  wire [1:0]  key_raw_pin,

    // ---- 音频（MAX98357A）----
    output wire        HP_BCK,         // Y17
    output wire        HP_WS,          // AB17
    output wire        HP_DIN,         // AA16
    output wire        PA_EN,          // Y14 = SD_MODE：**1 = 开**

    // ---- 用户 LED（Console 只有 1 颗 RGB，且只有 G 这一路能用，见 §9）----
    //  ⚠️ 2026-10-25 实测（Gowin P&R 报 PR2017/PR2028）：
    //     `R = U12 = ~READY`、`B = G11 = ~DONE` 在 GW5AST-138/PG484 上是
    //     **配置专用脚（dedicated pin）**，直接约束会被拒绝；
    //     只有 `G = V13 = SYS_ACT` 是普通 IO（且直连阴极 ⇒ **低 = 亮**）。
    //     ⇒ 端口从 `LEDs[2:0]` 收成 1 位 `led`，只驱动 V13。
    //       想恢复三色：在 Gowin 的 Bitstream → Dual-Purpose Pin 里把
    //       READY / DONE 勾成 regular IO，然后把这里和 `.cst` 改回 3 位。
    output wire        led,            // V13，低电平点亮

    // ---- P2 输入（⚠️ 引脚待硬件确认，见 constraints\console_138k.cst）----
    //  ⚠️ 2026-10-30：SCL 原来写成 `output`，里面赋 `1'bz` —— 综合器/工具链未必
    //     保留三态，且**读不回来**（`i2c_master.scl_i` 读到的只是驱动值）。
    //     I2C 的 SCL 本来就该是**开漏双向**，改成 `inout` 更正确，也让固件能读到真实引脚电平。
    inout  wire        scl_pad,        // I2C SCL（开漏，靠 4.7K 上拉）
    inout  wire        sda_pad,        // I2C SDA
    input  wire        key_dn_raw,     // 八度 −（机械键，低有效）
    input  wire        key_up_raw,     // 八度 ＋
    input  wire        sw_glide_raw,   // 滑音自锁开关（低有效）

    // ---- 串口（板载 BL616 桥 ⇒ PC 虚拟串口）----
    output wire        uart_tx
);
    // ============================================================
    //  1) 时钟与复位
    //  TODO(rPLL)：用 Gowin IP Core Generator 生成 50→150 MHz，
    //     把下面 clk_sys 换成 PLL 输出、SYS_CLK_HZ 改 150_000_000、
    //     并把 pll_lock 并进复位：
    //        wire rst_n = sys_rst_n & pll_lock;
    //  为什么值得上 150 MHz：① BCLK 边沿抖动从 ±20 ns 降到 ±6.7 ns
    //     ② 每帧从 1042 拍变 3125 拍，32 声部 + 查表更从容。与音准无关。
    // ============================================================
    wire clk_sys = sys_clk;
    wire rst_n   = sys_rst_n;

    // ============================================================
    //  2) 板载键：翻极性 + 去抖（key_raw_pin 是**原始电平、低有效**）
    //     ⚠️ 不翻极性：上电所有键都读成"按下" ⇒ 一开机就进演示态
    // ============================================================
    wire demo_raw = KEY_ACTIVE_LOW ? ~key_raw_pin[0] : key_raw_pin[0];
    //  ⚠️ 2026-10-30：`key_raw_pin[1]`（板载 EX_KEY.1 = AB13）**改作八度循环键**。
    //     原来它接"音色 A/B"，实物上从来没生效过；用户明确要求把这颗板载键给八度。
    //     原始电平直接送 `p2_input_top`（同步器/去抖/边沿都在 `octave_key` 内部做，
    //     这里若再去抖一次会**吃掉短按**）。

    localparam integer DEMO_CNT_MAX = (DEMO_DEB_US * (SYS_CLK_HZ / 1000)) / 1000 - 1;

    wire demo_key;

    debounce #(.CNT_WIDTH(24), .CNT_MAX(DEMO_CNT_MAX)) u_db_demo (
        .clk(clk_sys), .rst_n(rst_n), .key_in(demo_raw), .key_out(demo_key)
    );

    // ============================================================
    //  3) P2 输入：MPR121 触摸 12 键 + 八度 + 滑音开关
    //     只例化这一个模块，端口原样往上传（P2 的交付约定）
    // ============================================================
    wire [11:0] p2_keys, p2_press, p2_release;
    wire [2:0]  octave_sel;
    wire        glide_en;
    wire [3:0]  glide_shift;
    wire        p2_init_done, p2_cfg_err, p2_i2c_err, p2_ovcf, p2_busy;
    wire [3:0]  p2_err_code;      // ★ 诊断码（cfg_err/i2c_err/ECR回读错/开机总线异常）
    wire [7:0]  p2_ecr;           // ★ 回读到的 ECR 值
    wire [1:0]  p2_blvl;          // ★ 实时电平 {SDA, SCL}：3=都高
    wire [3:0]  p2_rise;          // 上升沿时间桶（借 S= 打印）

    // ============================================================
    //  ★ I2C pad 的三态处理（2026-11-01，标准双向结构）
    //     pad 只在这里被驱动/读取；`i2c` 子模块只拿到 i / oe 两条单向信号。
    //     写法是工具一定能识别的形态：`assign pad = oe_low ? 1'b0 : 1'bz;`
    // ============================================================
    wire scl_oe_low, sda_oe_low, scl_i, sda_i;

    assign scl_pad = scl_oe_low ? 1'b0 : 1'bz;
    assign sda_pad = sda_oe_low ? 1'b0 : 1'bz;

    assign scl_i = scl_pad;
    assign sda_i = sda_pad;
    p2_input_top #(
        .DEV        (I2C_DEV),
        .CLK_HZ     (SYS_CLK_HZ),
        .SCL_HZ     (SCL_HZ),
        .TIMEOUT_CLK(SYS_CLK_HZ / 1000),      // 1 ms 超时
        .RESET_WAIT (SYS_CLK_HZ / 200),       // ★ 软复位后 5 ms（1 ms 太短：实测写会被忽略）
        .POLL_DIV   (SYS_CLK_HZ / 1000),      // 1 ms 轮询一次
        .DEB_N      (5),                      // ★ 连续 5 次一致才认（抑制偶发误触）
        .STARTUP    (SYS_CLK_HZ / 5),      // ★ 200 ms（原 500 ms 太保守；开机慢主要由基线漂移引起）
        .OCT_INIT   (3'd2), .OCT_MIN(3'd1), .OCT_MAX(OCT_MAX),
        .MECH_CNT_W (24), .MECH_DEB(SYS_CLK_HZ / 250),   // 4 ms
        .ACTIVE_LOW (1),
        .GLIDE_SHIFT(4'd12)               // τ = 2^12/48000 ≈ 85 ms（P2 交付默认值）
    ) u_p2 (
        .clk        (clk_sys), .rst_n(rst_n),
        .scl_i      (scl_i), .sda_i(sda_i),
        .scl_oe_low (scl_oe_low), .sda_oe_low(sda_oe_low),
        .key_dn_raw (key_dn_raw), .key_up_raw(key_up_raw), .sw_glide_raw(sw_glide_raw),
        .key_cyc_raw(key_raw_pin[1]),      // 板载 EX_KEY.1(AB13) = 八度循环键
        .key_raw    (p2_keys), .key_press(p2_press), .key_release(p2_release),
        .octave_sel (octave_sel), .glide_en(glide_en), .glide_shift(),
        .init_done  (p2_init_done), .cfg_err(p2_cfg_err),
        .i2c_err    (p2_i2c_err), .ovcf(p2_ovcf), .i2c_busy(p2_busy),
        .err_code   (p2_err_code), .ecr_rd(p2_ecr), .bus_lvl(p2_blvl),
        .rise_bkt_o (p2_rise)
    );

    // ============================================================
    //  4) 12 键 → 音符号 + 复音事件（P1 的 key_scan，产生全链共用的 tick）
    //     ⚠️ `mod_raw` 在冻结的 P2 方案里**没有来源**（只有 2 个八度键 +
    //        1 个滑音开关）⇒ 接 0。颤音深度改为顶层参数 VIB_SHIFT。
    // ============================================================
    wire        sample_tick;
    wire [11:0] key_state, key_press, key_release;
    wire        key_active;
    wire [2:0]  octave_out;
    wire [6:0]  midi_base, note_num;
    wire        note_on;
    wire [3:0]  amp_shift_ks, glide_shift_ks;

    //  ⚠️ `glide_en=1` 时把按键事件对分配器屏蔽（滑音走单声部通路，见 §6）
    wire [11:0] press_to_alloc = glide_en ? 12'd0 : key_press;
    wire [11:0] rel_to_alloc   = glide_en ? 12'd0 : key_release;

    key_scan #(
        .SYS_CLK_HZ (SYS_CLK_HZ),
        .FS_HZ      (FS_HZ),
        .DEB_US     (DEB_US),
        .OCT_MAX    (OCT_MAX)
    ) u_ks (
        .clk         (clk_sys),
        .rst_n       (rst_n),
        .key_raw     (p2_keys),
        .mod_raw     (3'b000),
        .octave_sel  (octave_sel),
        .sample_tick (sample_tick),
        .key_state   (key_state),
        .key_press   (key_press),
        .key_release (key_release),
        .active      (key_active),
        .octave_out  (octave_out),
        .midi_base   (midi_base),
        .note_num    (note_num),
        .note_on     (note_on),
        .amp_shift   (amp_shift_ks),
        .glide_shift (glide_shift_ks)
    );

    // ============================================================
    //  5) 声部分配 + 32 声部阵列
    // ============================================================
    localparam integer NV      = 32;
    localparam integer SPECIAL = 16;
    localparam integer ATTW    = 7;
    localparam integer AMPW    = 3;
    localparam integer FW_W    = 32;

    wire [NV-1:0]        v_gate, v_valid;
    wire [NV*7-1:0]      v_note;
    wire [NV*ATTW-1:0]   v_atten;
    wire [NV-1:0]        v_active, v_releasing;

    // 滑音模式退出时给分配器一个复位脉冲：
    //   单声部通路占的是槽 0；退出时若不清掉分配器的状态，
    //   槽 0 会带着"冻结的旧音符"回到复音模式 ⇒ **卡音**。
    //   复位只清分配器（槽全 idle），各声部的包络靠 gate=0 自然释放，不会爆音。
    reg  glide_en_d;
    reg  [1:0] glide_exit_cnt;
    wire glide_exit = glide_en_d & ~glide_en;

    always @(posedge clk_sys or negedge rst_n) begin
        if (!rst_n) begin
            glide_en_d     <= 1'b0;
            glide_exit_cnt <= 2'd0;
        end else begin
            glide_en_d <= glide_en;
            if (glide_exit)                 glide_exit_cnt <= 2'd2;
            else if (glide_exit_cnt != 0)   glide_exit_cnt <= glide_exit_cnt - 2'd1;
        end
    end

    wire alloc_rst_n = rst_n & (glide_exit_cnt == 2'd0);

    voice_alloc #(
        .NV(NV), .SPECIAL(SPECIAL), .NB(12),
        .ATTW(ATTW), .ATTEN_FULL(64), .FADE_FRAMES(32)
    ) u_alloc (
        .clk            (clk_sys),
        .rst_n          (alloc_rst_n),
        .sample_tick    (sample_tick),
        .key_state      (key_state),
        .key_press      (press_to_alloc),
        .key_release    (rel_to_alloc),
        .midi_base      (midi_base),
        .voice_active   (v_active),
        .voice_releasing(v_releasing),
        .voice_gate     (v_gate),
        .voice_note     (v_note),
        .voice_atten    (v_atten),
        .voice_valid    (v_valid),
        .steal_pulse    (),
        .last_steal     (),
        .n_used         ()
    );

    // ============================================================
    //  6) ★ P1 缺的那一级：32 路 MIDI → 32 个 32 位频率字
    //
    //  为什么不能例化 32 个 freq_table：它是 84 项 if/else 链（纯组合），
    //  32 份会烧掉上万 LUT。这里只放 **1 个** freq_table + 一个 32 槽扫描器：
    //  每个时钟换一个槽，32 拍把所有槽刷新一遍 —— 一帧有 1042~3125 拍，
    //  余量 30 倍以上。各级都寄存，组合路径只有"1 个 freq_table"那么长。
    // ============================================================
    //  fw_bank 在下面声明为 reg（要在一个 always 里按槽写）
    wire [FW_W-1:0] fw_lut;

    reg [4:0] scan_s0, scan_s1;
    reg [6:0] scan_note;

    freq_table u_lut (
        .note (scan_note),
        .freq (fw_lut)
    );

    // 扫描地址（自由运行，与 sample_tick 无关；刷新率 = 时钟/32）
    always @(posedge clk_sys or negedge rst_n) begin
        if (!rst_n) begin
            scan_s0   <= 5'd0;
            scan_s1   <= 5'd0;
            scan_note <= 7'd60;
        end else begin
            scan_s0   <= scan_s0 + 5'd1;
            scan_s1   <= scan_s0;
            scan_note <= v_note[scan_s0*7 +: 7];
        end
    end

    genvar gi;
    //  ⚠️ 每个 generate 循环必须用**各自独立的** genvar。
    genvar gj;

    //  频率字寄存器堆：**一个 always + 变量部分选择写**。
    //  ⚠️ 2026-10-23 踩坑：原先用"32 个小 generate 块各写自己那一段 wire"
    //     （`assign fw_bank[gi*32 +: 32] = q;`）—— 各块的 q 都有正确值，
    //     但整条 `fw_bank` 在仿真里一直是 X ⇒ 相位累加器被 X 污染后**永久为 X**
    //     ⇒ 上电第一帧就全机静音。改成一个 always 直接写 reg 向量，干净可靠。
    reg [NV*FW_W-1:0] fw_bank;

    always @(posedge clk_sys or negedge rst_n) begin
        if (!rst_n) fw_bank <= {(NV*FW_W){1'b0}};
        else        fw_bank[scan_s1*FW_W +: FW_W] <= fw_lut;   // 轮到的槽刷新
    end

    // ============================================================
    //  7) 颤音：全局三角波 LFO，但**每个声部各乘一次**（freq_mod 是单声部模块）
    //     幅度按 2^-VIB_SHIFT 相对缩放 ⇒ 与音高无关的等音分颤音。
    //     VIB_ON=0 时整段被优化掉（0 个乘法器）。
    // ============================================================
    wire [NV*FW_W-1:0] fw_vib;

    // ⚠️ 先只声明、在 §10 才赋值：它们的取值要用 test_mode 的 no_vibrato，
//    而 test_mode 实例化在本节之后。Verilog 的连续赋值与声明位置无关，
//    但**声明**必须在使用之前（iverilog 对先用后声明直接报错）。
    wire        vib_on_m;
    wire [3:0]  vib_depth_m;

    generate
        if (VIB_ON != 0) begin : g_vib
            for (gj = 0; gj < NV; gj = gj + 1) begin : g_one
                freq_mod #(
                    .FW_W(FW_W), .DEPTH_W(4), .LFO_STEP(7)
                ) u_fm (
                    .clk        (clk_sys),
                    .rst_n      (rst_n),
                    .sample_tick(sample_tick),
                    .freq_in    (fw_bank[gj*FW_W +: FW_W]),
                    .vib_depth  (vib_depth_m),
                    .vib_on     (vib_on_m),
                    .freq_out   (fw_vib[gj*FW_W +: FW_W]),
                    .lfo_valid  ()
                );
            end
        end else begin : g_novib
            assign fw_vib = fw_bank;
        end
    endgenerate

    // ============================================================
    //  8) 滑音：mono / legato 单声部通路
    //
    //  冻结语义（P2_HANDOFF §1 第 6 条）：`glide_en=1` ⇒ 引擎进 mono/legato
    //  （last-note priority、**不重触发包络**），`glide_en=0` ⇒ 32 声部复音。
    //
    //  做法：
    //    · 最高音 = key_scan 的 `note_num`（它就是"按下集合里最高的那个"）
    //    · target = freq_table(note_num)，经 `glide` 一阶插值（τ = 2^shift/48k）
    //    · 槽 0 被单声部通路**接管**（gate = 还有键按着 → 换音不重触发；
    //      最后松手才落 gate → 走 RELEASE）
    //    · 槽 1..31 在滑音模式下 gate=0（自然释放），atten 归零
    //    · 分配器在滑音期间被屏蔽（§4），退出时复位一次（§5）
    // ============================================================
    wire [FW_W-1:0] fw_mono_tgt, fw_mono;

    freq_table u_lut_mono (
        .note (note_num),
        .freq (fw_mono_tgt)
    );

    //  ⚠️ 2026-10-25：目标频率字**先寄一存**再进 glide。
    //     不加这级的话，路径是 `note_num -> freq_table(128 项 mux) -> glide 的减法/移位/加法
    //     -> freq 寄存器`，实测 slack −0.51 ns（Fmax 48.7 MHz，差一点点）。
    //     寄存一级后两段各占半程。代价：mono 通路的音高晚 1 个采样（20.8 µs），
    //     对 τ≈85 ms 的滑音完全无感。
    reg [FW_W-1:0] fw_mono_tgt_r;

    always @(posedge clk_sys or negedge rst_n) begin
        if (!rst_n) fw_mono_tgt_r <= {FW_W{1'b0}};
        else        fw_mono_tgt_r <= fw_mono_tgt;
    end

    glide #(
        .SHIFT    (12),
        .INIT_FREQ(32'd0)
    ) u_glide (
        .clk        (clk_sys),
        .rst_n      (rst_n),
        .sample_tick(sample_tick),
        .glide_en   (glide_en),
        .target     (fw_mono_tgt_r),
        .freq       (fw_mono)
    );

    // ============================================================
    //  9) 演示态（P1 的 test_mode）：一键 1→2→4→8→16→32 声部
    //     输出一套**覆盖信号**，由下面第 10 节的 mux 接进数据通路。
    // ============================================================
    wire               demo_en;
    wire [2:0]         demo_sel;
    wire [5:0]         n_voices;
    wire [NV-1:0]      tm_gate;
    wire [NV*FW_W-1:0] tm_freq;
    wire [NV*AMPW-1:0] tm_amp;
    wire               force_sine, mod_zero, force_fm, no_vibrato, fx_bypass;
    wire [2:0]         led_demo;

    test_mode #(
        .NV(NV), .FW_W(FW_W), .AMPW(AMPW)
    ) u_tm (
        .clk            (clk_sys),
        .rst_n          (rst_n),
        .demo_key       (demo_key),
        .demo_en        (demo_en),
        .demo_sel       (demo_sel),
        .n_voices       (n_voices),
        .voice_gate     (tm_gate),
        .voice_freq     (tm_freq),
        .voice_amp_shift(tm_amp),
        .force_sine     (force_sine),
        .mod_zero       (mod_zero),
        .force_fm       (force_fm),
        .no_vibrato     (no_vibrato),
        .fx_bypass      (fx_bypass),
        .led_rgb        (led_demo)
    );

    // ⚠️ `fx_bypass` 是给 P3 混响用的（演示态必须旁路湿声）。
    //    P3 的 reverb.v 还没写 ⇒ 暂时只把它引出来，接在
    //    `mixer_tree → reverb → i2s_tx` 那个位置（见 §12 的 TODO）。

    // ============================================================
    //  10) 控制域 → 音频域：所有参数在 param_xfer 里统一对齐到帧边界
    //      （这是"频率字在一帧中间变化 ⇒ 咔哒声"的唯一防线）
    //  （vib_on_m / vib_depth_m 已在第 7 节之前声明，这里赋值）
    // ============================================================
    assign vib_on_m    = no_vibrato ? 1'b0 : (VIB_ON != 0);
    assign vib_depth_m = no_vibrato ? 4'd0 : VIB_SHIFT;

    localparam [ATTW-1:0] ATT_FULL = {ATTW{1'b1}};    // 64 = 满量

    reg  [NV*FW_W-1:0]  cfg_freq;
    reg  [NV*2-1:0]     cfg_wave;
    reg  [NV*AMPW-1:0]  cfg_amp;
    reg  [NV*ATTW-1:0]  cfg_atten;
    reg  [NV-1:0]       cfg_gate;
    reg                 cfg_special;
    reg  [15:0]         cfg_modidx;
    reg  [2:0]          cfg_vibd;

    integer k;
    always @(*) begin
        for (k = 0; k < NV; k = k + 1) begin
            // ---- 频率 ----
            if (demo_en)                   cfg_freq[k*FW_W +: FW_W] = tm_freq[k*FW_W +: FW_W];
            else if (glide_en && (k == 0)) cfg_freq[k*FW_W +: FW_W] = fw_mono;
            else                           cfg_freq[k*FW_W +: FW_W] = fw_vib[k*FW_W +: FW_W];

            // ---- 波形（演示态强制正弦页 0）----
            cfg_wave[k*2 +: 2] = force_sine ? 2'd0 : POLY_WAVE;

            // ---- 每声部音量 ----
            //   演示态用 test_mode 的 1/N；正常演奏满量（整体缩放交给 MIX_SHIFT）
            cfg_amp[k*AMPW +: AMPW] = demo_en ? tm_amp[k*AMPW +: AMPW] : {AMPW{1'b0}};

            // ---- 淡出系数 ----
            if (demo_en)                   cfg_atten[k*ATTW +: ATTW] = ATT_FULL;
            else if (glide_en)             cfg_atten[k*ATTW +: ATTW] = (k == 0) ? ATT_FULL : {ATTW{1'b0}};
            else                           cfg_atten[k*ATTW +: ATTW] = v_atten[k*ATTW +: ATTW];

            // ---- 包络门（电平！）----
            if (demo_en)                   cfg_gate[k] = tm_gate[k];
            else if (glide_en)             cfg_gate[k] = (k == 0) ? key_active : 1'b0;
            else                           cfg_gate[k] = v_gate[k];
        end

        // ---- 全局音色 ----
        //  ⚠️ 2026-10-30：板载第 2 键改作八度循环后，"音色 A/B"这颗键没有了来源。
    //     现在：演示态强制 FM；平时固定 0（= 加法音色）。将来若要把音色键找回来，
    //     建议接 J6 的 pin6（`PMOD0_IO7` = B22，目前空着）。
    cfg_special = force_fm;
        cfg_modidx  = mod_zero     ? 16'd0 : MOD_INDEX;
        cfg_vibd[0 +: 3] = no_vibrato ? 3'd0 : VIB_SHIFT[2:0];
    end

    wire [NV*FW_W-1:0] xf_freq;
    wire [NV*2-1:0]    xf_wave;
    wire [NV*AMPW-1:0] xf_amp;
    wire [NV*ATTW-1:0] xf_atten;
    wire [NV-1:0]      xf_gate, xf_en;
    wire               xf_special;
    wire [15:0]        xf_modidx;
    wire [2:0]         xf_vibd;

    param_xfer #(
        .NV(NV), .FW_W(FW_W), .WAVE_W(2), .AMPW(AMPW),
        .ATTW(ATTW), .MOD_W(16)
    ) u_xf (
        .clk          (clk_sys),
        .rst_n        (rst_n),
        .sample_tick  (sample_tick),
        .cfg_freq     (cfg_freq),
        .cfg_wave     (cfg_wave),
        .cfg_amp_shift(cfg_amp),
        .cfg_atten    (cfg_atten),
        .cfg_gate     (cfg_gate),
        .cfg_voice_en ({NV{1'b1}}),
        .cfg_special  (cfg_special),
        .cfg_mod_index(cfg_modidx),
        .cfg_vib_depth(cfg_vibd),
        .xf_freq      (xf_freq),
        .xf_wave      (xf_wave),
        .xf_amp_shift (xf_amp),
        .xf_atten     (xf_atten),
        .xf_gate      (xf_gate),
        .xf_voice_en  (xf_en),
        .xf_special   (xf_special),
        .xf_mod_index (xf_modidx),
        .xf_vib_depth (),
        .xfer_stb     (),
        .xfer_cnt     (),
        .hold_cnt     ()
    );

    // ============================================================
    //  11) 32 声部阵列
    // ============================================================
    wire signed [NV*16-1:0] v_out;
    wire                    v_valid_out;

    voice_array #(
        .NV      (NV),
        .SPECIAL (SPECIAL),
        .AMPW    (AMPW),
        .ATTW    (ATTW),
        .ATTEN_FULL(64),
        .VCA_NV  (4),
        .INIT_FILE(INIT_FILE)
    ) u_va (
        .clk            (clk_sys),
        .rst_n          (rst_n),
        .sample_tick    (sample_tick),
        .voice_gate     (xf_gate),
        .voice_freq     (xf_freq),
        .voice_wave     (xf_wave),
        .voice_amp_shift(xf_amp),
        .voice_atten    (xf_atten),
        .special_mode   (xf_special),
        .mod_index      (xf_modidx),
        .sample_out     (v_out),
        .out_valid      (v_valid_out),
        .voice_active   (v_active),
        .voice_releasing(v_releasing)
    );

    // ============================================================
    //  12) 混音 → 混响(P3) → 软启动 → 寄存 → I2S
    // ============================================================
    wire signed [15:0] mix_out;
    wire               mix_valid;
    wire               mix_sat;

    mixer_tree #(
        .NV         (NV),
        .SW         (16),
        .OUT_W      (16),
        .SCALE_SHIFT(MIX_SHIFT),
        .SAT        (1),
        .LATENCY    (6)
    ) u_mix (
        .clk        (clk_sys),
        .rst_n      (rst_n),
        .sample_tick(sample_tick),
        .sample_in  (v_out),
        .mix_out    (mix_out),
        .mix_valid  (mix_valid),
        .sat_flag   (mix_sat)
    );

    // ---- P3 混响（单声道 Schroeder，LATENCY=9 帧）----
    //  ⚠️ `fx_bypass` 来自 test_mode：演示态必须旁路湿声，否则 32 根稳态正弦
    //     过混响会因梳状滤波长出旁瓣，评委可能把峰数数错（接口契约 §2）。
    //     旁路**只切湿声**，延迟拍数由 reverb 内部结构保证不变。
    //  ⚠️ 混响自己的 `LATENCY` 是**帧**（48 kHz 采样周期），不是时钟拍：
    //     它内部每帧把 7 条延迟线各读写一次，占用 ~10 个时钟，远小于一帧的
    //     1042 拍（50 MHz），所以不会挤压一帧的处理时间。
    wire signed [15:0] rv_out;
    wire               rv_valid;

    reverb #(
        .FS_HZ        (FS_HZ),
        .W            (16),
        .PD_SAMPLES   (480),        // pre-delay 10 ms
        .DAMP_SHIFT   (2),
        .WET_SHIFT    (2),          // 湿声 −6 dB   // ★ 2026-11-03：1→2（湿声 −12 dB），防干+湿越界
        .LATENCY      (9),
        .FLUSH_SAMPLES(2304)
    ) u_reverb (
        .clk        (clk_sys),
        .rst_n      (rst_n),
        .sample_tick(sample_tick),
        .dry_in     (mix_out),
        .fx_bypass  (fx_bypass),
        .out        (rv_out),
        .out_valid  (rv_valid)
    );

    // 软启动：上电前 RAMP_MS 毫秒增益 0→1（防上电"砰"声）
    //   ⚠️ 用 rv_valid（混响之后的那一拍），否则混响的 9 帧延迟会让
    //      斜坡计数与音频错位。
    localparam integer RAMP_FRAMES = (RAMP_MS * FS_HZ) / 1000;
    localparam integer RAMP_STEP   = 65536 / RAMP_FRAMES;

    reg [16:0] ramp_acc;
    always @(posedge clk_sys or negedge rst_n) begin
        if (!rst_n)          ramp_acc <= 17'd0;
        else if (rv_valid) begin
            if (ramp_acc < (17'd65536 - RAMP_STEP[16:0]))
                ramp_acc <= ramp_acc + RAMP_STEP[16:0];
            else
                ramp_acc <= 17'd65536;
        end
    end

    // ⚠️ signed × unsigned 必须显式拓宽（铁律，见 vca.v）
    wire signed [31:0] mx32 = $signed(rv_out);
    wire signed [31:0] rg32 = $signed({1'b0, ramp_acc});
    wire signed [31:0] at32 = (mx32 * rg32) >>> 16;

    localparam signed [15:0] MAXS = 16'sd32767;
    localparam signed [15:0] MINS = -16'sd32768;

    wire signed [15:0] att = (at32 > MAXS) ? MAXS :
                             (at32 < MINS) ? MINS : at32[15:0];

    // ★ 铁律③：i2s_tx 只在 sample_tick 那一拍锁存 ⇒ 数据必须寄存
    reg signed [15:0] att_r;
    always @(posedge clk_sys or negedge rst_n) begin
        if (!rst_n) att_r <= 16'sd0;
        else        att_r <= att;
    end

    generate
        if (SIM_SKIP_I2S != 0) begin : g_no_i2s
            assign HP_BCK = 1'b0;
            assign HP_WS  = 1'b0;
            assign HP_DIN = 1'b0;
        end else begin : g_i2s
            i2s_tx #(
                .SYS_CLK_HZ(SYS_CLK_HZ),
                .FS_HZ     (FS_HZ),
                .DATA_W    (16)
            ) u_i2s (
                .clk        (clk_sys),
                .rst_n      (rst_n),
                .sample_tick(sample_tick),
                .data_l     (att_r),
                .data_r     (att_r),          // Console 是单声道（SD_MODE 只放左）
                .bck        (HP_BCK),
                .ws         (HP_WS),
                .sd         (HP_DIN)
            );
        end
    endgenerate

    // ★ 铁律④：Console 上 PA_EN 接 MAX98357A 的 SD_MODE，**高电平才开**
    assign PA_EN = 1'b1;

    // ============================================================
    //  13) 用户 LED（只剩 G 一路，V13）
    //  ⚠️ Console 那颗 RGB 的三路里，**只有 G 能用**：
    //       R = U12 = ~READY ⇒ **配置专用脚**，PR2017/PR2028 拒绝
    //       G = V13 = SYS_ACT（直连阴极）⇒ 普通 IO，**低 = 亮**
    //       B = G11 = ~DONE  ⇒ **配置专用脚**，同样被拒
    //     （这是 2026-10-25 实测，见文件头端口注释）
    //
    //  单灯的信息量有限，所以只表达"最该被一眼看到"的三件事：
    //      · 演示态生效（评委按到第几档靠**按的次数**和串口 D=n 看）
    //      · 有键按着
    //      · 出错（I2C 失败 / 混音饱和）
    //    真正靠得住的判据是**串口**（K/O/G/D/E/S 六项）。
    // ============================================================
    wire err_any = p2_cfg_err | p2_i2c_err | p2_ovcf | mix_sat;

    //  低有效：led = 0 时亮
    assign led = ~(demo_en | key_active | err_any);

    // ============================================================
    //  14) 串口状态行（板上只有 1 颗 RGB，没有串口没法调试）
    //
    //  格式固定 27 字节：  K=xxx O=n G=g D=n E=e S=s CR LF
    //    K = 触摸位图（3 位十六进制，同时按 8 键应看到 0FF）
    //    O = 八度（1..4）      G = 滑音开关
    //    D = 演示档位（0=退出；1..6 = 1/2/4/8/16/32 声部）
    //    E = 任何错误（I2C/溢出/饱和）  S = 混音饱和
    //  触发：初始化完成那一刻 + 任何字段变化 + 每秒心跳
    // ============================================================
    reg        u_send;
    reg  [7:0] u_data;
    wire       u_busy;

    uart_tx #(.CLK_HZ(SYS_CLK_HZ), .BAUD(BAUD)) u_uart (
        .clk(clk_sys), .rst_n(rst_n), .send(u_send), .data(u_data),
        .tx(uart_tx), .busy(u_busy)
    );

    function [7:0] hexasc(input [3:0] v);
        hexasc = (v < 4'd10) ? (8'h30 + {4'd0, v}) : (8'h37 + {4'd0, v});
    endfunction

    localparam [5:0] LINE_LAST = 6'd14;   // ★ 2026-11-03 精简：K=xxx O=n D=n（13 字节 + CR LF = 15）
    //  ⚠️ 回滚提示：想看回完整诊断行（含 G/E/S/V/W），把下面 case 里的
    //     6'd9..6'd20 换成旧版本即可；同时把 LINE_LAST 改回 6'd35。
    //     精简原因：用户只关心 K（触摸）、O（八度）、D（演示档）。

    function [7:0] line_byte(
        input [5:0]  i,
        input [11:0] k,
        input [2:0]  o,
        input [2:0]  d
    );
        begin
            case (i)
            6'd0 : line_byte = 8'h4B;                  // K
            6'd1 : line_byte = 8'h3D;                  // =
            6'd2 : line_byte = hexasc(k[11:8]);
            6'd3 : line_byte = hexasc(k[7:4]);
            6'd4 : line_byte = hexasc(k[3:0]);
            6'd5 : line_byte = 8'h20;
            6'd6 : line_byte = 8'h4F;                  // O
            6'd7 : line_byte = 8'h3D;                  // =
            6'd8 : line_byte = 8'h30 + {5'd0, o[2:0]};
            6'd9 : line_byte = 8'h20;
            6'd10: line_byte = 8'h44;                  // D
            6'd11: line_byte = 8'h3D;                  // =
            6'd12: line_byte = 8'h30 + {5'd0, d};
            6'd13: line_byte = 8'h0D;                  // CR
            6'd14: line_byte = 8'h0A;                  // LF
            default: line_byte = 8'h0A;
            endcase
        end
    endfunction

    reg [2:0]  p_state;
    reg [5:0]  p_idx;
    reg [11:0] snap_k;
    reg [2:0]  snap_o, snap_d;
    //  ★ 2026-11-03：状态行只保留 K / O / D，其余快照寄存器已删。

    reg [11:0] k_d;
    reg [2:0]  o_d, d_d;
    reg        g_d, init_d;
    reg [31:0] hb_cnt;

    wire key_chg   = (p2_keys  != k_d);
    wire oct_chg   = (octave_sel != o_d);
    wire demo_chg  = (demo_sel != d_d);
    wire init_rise = (p2_init_done && !init_d);

    //  触发：初始化完成、触摸位图变化、八度变化、演示档变化、每秒心跳
    wire trig = (init_rise || key_chg || oct_chg || demo_chg ||
                 (hb_cnt >= HEARTBEAT[31:0]));

    localparam [2:0] P_IDLE = 3'd0, P_GO = 3'd1, P_WAIT = 3'd2, P_STOP = 3'd3;

    always @(posedge clk_sys or negedge rst_n) begin
        if (!rst_n) begin
            p_state <= P_IDLE;
            p_idx   <= 6'd0;
            snap_k <= 12'd0; snap_o <= 3'd0; snap_d <= 3'd0;
            u_send  <= 1'b0;  u_data <= 8'd0;
            k_d     <= 12'd0; o_d <= 3'd0; d_d <= 3'd0;
            g_d     <= 1'b0;  init_d <= 1'b0;
            hb_cnt  <= 32'd0;
        end else begin
            u_send <= 1'b0;
            k_d    <= p2_keys;
            o_d    <= octave_sel;
            d_d    <= demo_sel;
            g_d    <= glide_en;
            init_d <= p2_init_done;

            if (hb_cnt >= HEARTBEAT[31:0]) hb_cnt <= 32'd0;
            else                           hb_cnt <= hb_cnt + 32'd1;

            case (p_state)
            P_IDLE: if (trig) begin
                snap_k <= p2_keys;
                snap_o <= octave_sel;
                snap_d <= demo_sel;
                p_idx  <= 6'd0;
                hb_cnt <= 32'd0;
                p_state<= P_GO;
            end

            P_GO: if (!u_busy) begin
                u_send  <= 1'b1;
                u_data  <= line_byte(p_idx, snap_k, snap_o, snap_d);
                p_state <= P_WAIT;
            end

            P_WAIT: if (u_busy) p_state <= P_STOP;

            P_STOP: if (!u_busy) begin
                if (p_idx == LINE_LAST) p_state <= P_IDLE;
                else begin
                    p_idx   <= p_idx + 5'd1;
                    p_state <= P_GO;
                end
            end

            default: p_state <= P_IDLE;
            endcase
        end
    end

    // ---- 未被本设计使用的信号：显式接掉，免得综合报"未使用"时被人误改 ----
    wire _unused = &{1'b0, fx_bypass, amp_shift_ks, glide_shift_ks, glide_shift,
                     octave_out, note_num[0], p2_press, p2_release, p2_busy,
                     n_voices, v_valid, xf_en, led_demo};

endmodule

`default_nettype wire
