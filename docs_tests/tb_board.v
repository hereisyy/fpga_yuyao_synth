`timescale 1ns/1ps
// ============================================================
//  tb_board.v —— **上板前的板级自检**（跑的就是要交给 Gowin 的那套 RTL）
//
//  ⚠️ 本文件是 `08_P1\tb_top_console.v` 的**同源副本**，只改了：
//       · 模块名 tb_top_console → tb_board
//       · 波表路径 ../06_keyboard/wavetable_all.hex → ../wavetable_all.hex
//         （因为本目录的上一级就是 Gowin 目录，hex 就在那里）
//     两边的检查项要同步改；改完在两边各跑一次。

//
//  它验的是**接线**，不是算法（算法各自的 tb 已经验过了）：
//    · Console 引脚极性：PA_EN 高有效、板载键低有效 + 去抖
//    · 演示键 → test_mode → 覆盖信号 → param_xfer → voice_array → mixer → I2S
//    · 演示态"1 声部 = 60 Hz 一个峰"真的会出声，且**不削顶**
//    · 32 声部档：1/N 缩放（amp_shift=5）与 32 路 gate 真的送到了 voice_array
//    · 滑音开关 → p2_input_top → glide_en 真的到了顶层
//    · 串口有发送动作（能出字节）、I2S 有翻边
//
//  不验：MPR121 的 I2C（那要 I2C 从机模型，已由 tb_p2_* 147 项覆盖）、
//        音准精度（由 tb_osc / tb_song 覆盖）。
//
//  ⚠️ 为什么用 12.5 MHz 而不是真实 50 MHz：
//     48 kHz 帧在 50 MHz 下是 1042 拍，跑 2000 帧就要 200 万拍；
//     12.5 MHz 下是 260 拍，同样的帧数只要 1/4 时间。
//     功能与时序无关（i2s_tx 仍满足"每半 BCK ≥2 拍"），只是仿真快 4 倍。
//  ⚠️ DEMO_DEB_US 压到 200 µs：真实值 20 ms 会让每次按键要跑 100 万拍。
// ============================================================
`default_nettype none

module tb_board;

    localparam integer CLK_HZ  = 12_500_000;
    localparam real    TCLK    = 1.0e9 / CLK_HZ;      // ns

    reg clk = 1'b0;
    reg rst_n = 1'b0;
    always #(TCLK / 2.0) clk = ~clk;

    // ---- 板载键（低有效：1 = 松开）----
    reg [1:0] key_raw_pin = 2'b11;
    // ---- P2 机械键（低有效）----
    reg key_dn_raw = 1'b1, key_up_raw = 1'b1, sw_glide_raw = 1'b1;

    // ---- I2C：用 pullup 模拟 MPR121 模块板载的 4.7K（否则读到 z/x）----
    wire scl_pad, sda_pad;
    pullup (scl_pad);
    pullup (sda_pad);

    wire       HP_BCK, HP_WS, HP_DIN, PA_EN;
    wire       led;
    wire       uart_tx;

    top_console #(
        .SYS_CLK_HZ  (CLK_HZ),
        .FS_HZ       (48_000),
        .DEB_US      (82),
        .DEMO_DEB_US (200),
        .RAMP_MS     (1),
        .MIX_SHIFT   (1),
        .POLY_WAVE   (2'd0),
        .VIB_ON      (1),
        .VIB_SHIFT   (4'd7),
        .MOD_INDEX   (16'd6000),
        .I2C_DEV     (7'h5A),
        .SCL_HZ      (100_000),
        .OCT_MAX     (3'd4),
        .KEY_ACTIVE_LOW(1),
        .INIT_FILE   ("../wavetable_all.hex"),
        .SIM_SKIP_I2S(0),
        .BAUD        (115_200),
        .HEARTBEAT   (CLK_HZ / 5)
    ) u_top (
        .sys_clk     (clk),
        .sys_rst_n   (rst_n),
        .key_raw_pin (key_raw_pin),
        .HP_BCK      (HP_BCK),
        .HP_WS       (HP_WS),
        .HP_DIN      (HP_DIN),
        .PA_EN       (PA_EN),
        .led         (led),
        .scl_pad     (scl_pad),
        .sda_pad     (sda_pad),
        .key_dn_raw  (key_dn_raw),
        .key_up_raw  (key_up_raw),
        .sw_glide_raw(sw_glide_raw),
        .uart_tx     (uart_tx)
    );

    // ============================================================
    //  记分板
    // ============================================================
    integer npass = 0, nfail = 0;

    task check;
        input string name;
        input        cond;
        begin
            if (cond) begin
                npass = npass + 1;
                $display("  [ ok ] %0s", name);
            end else begin
                nfail = nfail + 1;
                $display("  [FAIL] %0s", name);
            end
        end
    endtask

    task check_eq;
        input string  name;
        input [63:0]  got;
        input [63:0]  exp;
        begin
            if (got === exp) begin
                npass = npass + 1;
                $display("  [ ok ] %0s (=%0d)", name, got);
            end else begin
                nfail = nfail + 1;
                $display("  [FAIL] %0s got=%0d exp=%0d", name, got, exp);
            end
        end
    endtask

    // ---- 帧节拍等待 ----
    //  ⚠️ 用 negedge 采样：posedge 上 sample_tick 还是上一拍的旧值（tcnt 是 NBA）
    task wait_frames;
        input integer n;
        integer q;
        begin
            for (q = 0; q < n; q = q + 1) begin
                @(negedge clk);
                while (!u_top.sample_tick) @(negedge clk);
            end
        end
    endtask

    task wait_us;
        input integer us;
        integer q;
        begin
            for (q = 0; q < us; q = q + 1) repeat (CLK_HZ / 1_000_000) @(posedge clk);
        end
    endtask

    // ---- 按一下演示键（低有效 + 去抖 200 µs）----
    task press_demo;
        begin
            key_raw_pin[0] = 1'b0;
            wait_us(400);
            key_raw_pin[0] = 1'b1;
            wait_us(400);
            wait_frames(2);
        end
    endtask

    // ============================================================
    //  观测：I2S 翻边 / 串口发送 / 音频
    // ============================================================
    reg bck_seen = 1'b0;
    reg uart_seen = 1'b0;
    always @(posedge clk) begin
        if (HP_BCK)   bck_seen  <= 1'b1;
        if (!uart_tx) uart_seen <= 1'b1;      // 起始位 = 拉低
    end

    // 一帧一个样本：记下"上穿零点"发生在第几帧，用它算基频周期
    integer frame_no, cross_frame [0:7], cross_n;
    always @(negedge clk) begin
        if (rst_n && u_top.sample_tick) frame_no <= frame_no + 1;
    end

    reg     sign_d;
    always @(negedge clk) begin
        if (rst_n && u_top.sample_tick) begin
            if (($signed(u_top.att_r) > 0) && !sign_d) begin
                if (cross_n < 8) cross_frame[cross_n] = frame_no;
                cross_n = cross_n + 1;
            end
            sign_d <= ($signed(u_top.att_r) > 0);
        end
    end

    // ============================================================
    //  主流程
    // ============================================================
    integer i;
    integer per, dev_ppm;
    integer gates_one, amp5_all;

    initial begin
        frame_no = 0; cross_n = 0; sign_d = 1'b0;

        $display("");
        $display("==== tb_board (SYS_CLK_HZ=%0d) ====", CLK_HZ);

        // ---- 场景 1：复位后的静态引脚 ----
        $display("-- 场景1: 复位后引脚 --");
        repeat (20) @(posedge clk);
        rst_n = 1'b1;
        repeat (20) @(posedge clk);
        check_eq("PA_EN = 1（Console 的 SD_MODE 高有效）", PA_EN, 1);
        check("复位后音频为 0（软启动还没爬）", u_top.att_r === 16'sd0);
        wait_frames(20);

        // ---- 场景 2：按一下演示键 → 1 声部档 ----
        $display("-- 场景2: 演示键 -> 1 声部档 --");
        check_eq("按之前 demo_en=0", u_top.demo_en, 0);
        press_demo();
        check_eq("按一下后 demo_en=1", u_top.demo_en, 1);
        check_eq("demo_sel=1", u_top.demo_sel, 1);
        check_eq("n_voices=1", u_top.n_voices, 1);

        // ---- 场景 3：真的出声，且是 60 Hz（30 Hz × 素数 2）----
        $display("-- 场景3: 1 声部 = 60 Hz 正弦 --");
        wait_frames(600);                     // 过 attack，进 sustain
        cross_n = 0; frame_no = 0; sign_d = 1'b0;
        wait_frames(2500);                    // 60 Hz ⇒ 800 帧/周期，跑 3 个周期
        check("采到了至少 2 个上穿零点（说明真的在振荡）", cross_n >= 2);
        if (cross_n >= 2) begin
            per = cross_frame[1] - cross_frame[0];
            dev_ppm = ((per - 800) * 1000000) / 800;
            if (dev_ppm < 0) dev_ppm = -dev_ppm;
            $display("    [info] 实测周期 %0d 帧（期望 800），偏差 %0d ppm",
                     per, dev_ppm);
            check("周期在 800±24 帧内（60 Hz ±3%）", (per >= 776) && (per <= 824));
        end
        check("1 声部档没有削顶", u_top.mix_sat === 1'b0);

        // ---- 场景 4：再按到 32 声部档，检查 1/N 缩放真的送到了阵列 ----
        $display("-- 场景4: 32 声部档的 1/N 缩放与 gate --");
        for (i = 0; i < 5; i = i + 1) press_demo();
        check_eq("6 下后 demo_sel=6", u_top.demo_sel, 6);
        check_eq("n_voices=32", u_top.n_voices, 32);
        wait_frames(3);                       // 等 param_xfer 过一帧
        // 每声部 amp_shift 都应 = 5（1/32）
        amp5_all = 1;
        for (i = 0; i < 32; i = i + 1)
            if (u_top.xf_amp[i*3 +: 3] !== 3'd5) amp5_all = 0;
        check("32 路的 amp_shift 全是 5（= 1/32）", amp5_all == 1);
        gates_one = 0;
        for (i = 0; i < 32; i = i + 1)
            if (u_top.xf_gate[i] === 1'b1) gates_one = gates_one + 1;
        check_eq("32 路 gate 全是 1（电平，包络停在 sustain）", gates_one, 32);
        check("演示态特殊池被强制切到 FM 态（否则峰数翻倍）", u_top.xf_special === 1'b1);
        check_eq("演示态 mod_index 被强制为 0", u_top.xf_modidx, 0);
        check("演示态强制正弦页（wave 全 0）",
              u_top.xf_wave === {64{1'b0}});

        // ---- 场景 5：第 7 下退出演示 ----
        $display("-- 场景5: 第 7 下退出演示 --");
        press_demo();
        check_eq("退出后 demo_en=0", u_top.demo_en, 0);
        check_eq("退出后 demo_sel=0", u_top.demo_sel, 0);

        // ---- 场景 6：滑音开关 → glide_en 到顶层 ----
        $display("-- 场景6: 滑音开关 --");
        check_eq("初始 glide_en=0", u_top.glide_en, 0);
        sw_glide_raw = 1'b0;                  // 合上自锁开关
        wait_us(5000);                        // 机械键去抖 4 ms
        check_eq("合上开关后 glide_en=1", u_top.glide_en, 1);
        wait_frames(5);
        check("滑音模式下槽 0 的 gate 跟着“有键按下”，其余 0",
              u_top.xf_gate[0] === u_top.key_active);
        check_eq("滑音模式下槽 1..31 全静音（atten=0）",
                 u_top.xf_atten[7 +: 7] | u_top.xf_atten[31*7 +: 7], 0);
        sw_glide_raw = 1'b1;
        wait_us(5000);
        check_eq("松开开关后 glide_en=0", u_top.glide_en, 0);
        wait_frames(4);
        check("退出滑音后分配器被复位过（alloc_rst_n 回到 1）",
              u_top.alloc_rst_n === 1'b1);

        // ---- 场景 7：I2S 与串口确实在动 ----
        $display("-- 场景7: I2S / 串口 --");
        check("HP_BCK 有翻边", bck_seen == 1'b1);
        check("串口发出过字节（起始位出现过）", uart_seen == 1'b1);

        $display("");
        if (nfail == 0)
            $display("==== tb_board: ALL PASS (%0d checks) ====", npass);
        else
            $display("==== tb_board: %0d FAILED (of %0d) ====",
                     nfail, npass + nfail);
        $finish;
    end

    // 超时保护：正常约 150 万拍，给到 800 万拍
    initial begin
        repeat (8_000_000) @(posedge clk);
        $display("  [FAIL] 仿真超时（顶层没有跑出预期流程）");
        nfail = nfail + 1;
        $display("==== tb_board: TIMEOUT ====");
        $finish;
    end

endmodule

`default_nettype wire