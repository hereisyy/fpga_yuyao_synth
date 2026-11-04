// ============================================================
//  p2_input_top.v —— P2 的**单一入口**
//
//  给 P1 / 顶层负责人的用法：**只例化这一个模块**，
//  你拿到的就是 4 个成品端口：
//      key_raw[11:0]   12 键位图（高有效，key0=C … key11=B，与 keyboard.v 语义一致）
//      key_press / key_release   单周期脉冲
//      octave_sel[2:0] 八度档（直接接 keyboard.v / key_scan.v 的同名端口）
//      glide_en        滑音开关电平（1 = 开滑音；由 test_mode 强制拉 0）
//  外加一组诊断位（init_done / cfg_err / i2c_err / ovcf）给 UART 打印用。
//
//  ---- 为什么内部要 mux ----
//    `mpr121_cfg`（初始化）和 `touch_keys`（周期读）共用同一条 I2C 和同一个
//    `i2c_xfer`。两者不会同时工作：cfg 只在 init_done 之前发命令，
//    touch_keys 只在 init_done 之后才开始轮询。所以按 init_done 二选一即可，
//    不需要仲裁器（也就没有"谁会饿死"的问题）。
//
//  ---- 引脚 ----
//    scl_pad / sda_pad 是**开漏**：只拉低或高阻，上拉由 MPR121 模块板载的 4.7K 提供。
//    ⚠️ 真实硬件上不要把这些脚配成推挽输出（会与模块的上拉打架）。
//
//  ---- glide_shift 是 4 位 ----
//    与 P1 `08_P1\key_scan.v` 的 `output [3:0] glide_shift` 位宽一致，集成时不用适配。
// ============================================================
`default_nettype none

module p2_input_top #(
    // ---- I2C ----
    parameter [6:0]   DEV         = 7'h5A,
    parameter integer CLK_HZ      = 50_000_000,
    parameter integer SCL_HZ      = 25_000,   // 见 top_console 的说明（容忍弱上拉）
    parameter integer TIMEOUT_CLK = 50_000,
    // ---- MPR121 初始化 ----
    parameter integer RESET_WAIT  = 50_000,   // 软复位后等待（50MHz ⇒ 1 ms）
    // ---- 轮询与去抖（触摸）----
    parameter integer POLL_DIV    = 50_000,   // 50MHz ⇒ 1 ms 一次
    parameter integer DEB_N       = 3,        // 连续 3 次一致才认
    parameter integer STARTUP     = 10_000,
    // ---- 机械键 ----
    parameter [2:0]   OCT_INIT    = 3'd2,
    parameter [2:0]   OCT_MIN     = 3'd1,
    parameter [2:0]   OCT_MAX     = 3'd4,
    parameter integer MECH_CNT_W  = 18,
    parameter integer MECH_DEB    = 200_000,  // 4 ms @50MHz（去抖交给 RTL，统一口径）
    parameter         ACTIVE_LOW  = 1,        // 机械键上拉 + 接地 ⇒ 低有效
    // ---- 滑音 ----
    parameter [3:0]   GLIDE_SHIFT = 4'd12     // τ = 2^12/48000 ≈ 85 ms
)(
    input  wire        clk,
    input  wire        rst_n,

    // ---- I2C 开漏焊盘 ----
    //  ★ 2026-11-01：pad 由**顶层**独占（标准双向结构）。本模块只收 i、只出 oe。
    input  wire        scl_i,       // SCL 引脚电平（来自顶层 pad）
    output wire        scl_oe_low,  // 1 = 拉低 SCL
    input  wire        sda_i,       // SDA 引脚电平
    output wire        sda_oe_low,  // 1 = 拉低 SDA


    // ---- 3 个机械键（异步，低有效）----
    input  wire        key_dn_raw,     // 八度 −
    input  wire        key_up_raw,     // 八度 ＋
    input  wire        sw_glide_raw,   // 滑音自锁开关
    input  wire        key_cyc_raw,    // ★ 八度**循环**键（板载 EX_KEY.1 = AB13，低有效）

    // ---- 成品输出（给顶层）----
    output wire [11:0] key_raw,
    output wire [11:0] key_press,
    output wire [11:0] key_release,
    output wire [2:0]  octave_sel,
    output wire        glide_en,
    output wire [3:0]  glide_shift,

    // ---- 诊断（接 UART 打印 / LED）----
    output wire        init_done,
    output wire        cfg_err,
    output wire        i2c_err,
    output wire        ovcf,
    output wire        i2c_busy,
    output wire [3:0]  err_code,       // ★ 诊断码：{开机总线异常, ECR回读错, i2c_err, cfg_err}
    output wire [7:0]  ecr_rd,         // ★ 回读到的 ECR（串口打印用）
    output wire [1:0]  bus_lvl,        // ★ 实时电平 {SDA, SCL}：3=都高、0=都低
    output wire [3:0]  rise_bkt_o      // ★ 上升沿时间桶（判断线上挂了多大电容）
);

    // ---------------- I2C master ----------------
    wire m_start, m_wr, m_rd, m_rd_ack, m_stop;
    wire [7:0] m_tx, m_rx;
    wire m_done, m_nack, m_timeout;
    wire m_scl, m_sda, m_scl_oe, m_sda_oe;

    // 共享 i2c_xfer 的完成/读回（多个模块都要用，**必须先声明再用**；
    // `default_nettype none` 下漏声明会直接报错，这也算一道防呆）
    wire       xf_done, xf_err;
    wire [7:0] xf_rd0, xf_rd1, xf_rd2;

    // ============================================================
    //  ★ 探针模式（2026-10-30，上板实测后加）
    //
    //  背景：`E=1` 表示软复位在 0x5A~0x5D 四个地址上**全都无应答**。
    //        这时只剩三种可能：①模块没供电 ②SDA/SCL 没接到 G21/G22 ③我们的 SCL 没驱动出去。
    //        没有示波器时，用万用表就能把 ②③ 分开 —— 办法是让 FPGA 在两根线上"发信号"：
    //
    //    · **SCL(G22) ≈ 1.5 Hz 方波** ⇒ 万用表 DC 档读数应**稳定在 ≈1.6 V**（3.3V 的一半）
    //    · **SDA(G21) ≈ 0.09 Hz 方波**（约 6 秒低 / 6 秒高）⇒ 读数会**每几秒在 0↔3.3 V 之间跳**
    //
    //  于是：拿万用表一支表笔点 GND，另一支把 J6 的 8 个信号针挨个碰一遍：
    //    · 读到**稳定 1.6 V** 的那根 = 真正的 SCL（G22）
    //    · 读到**慢慢跳变**的那根     = 真正的 SDA（G21）
    //    · 8 根全是死高 3.3V 或死低 0V ⇒ **FPGA 根本没接到这个排针**（引脚/表搞错）
    //  只在 `cfg_err` 生效时输出，正常工作时完全不介入。
    // ============================================================
    //  ★ 总线恢复（2026-10-30）：从机在半个字节处被打断时会**一直拉低 SDA**
    //    （经典 I2C 死锁，SDA/SCL 插反过一次就很容易进入这个状态）。
    //    标准解法：**补 9 个 SCL 脉冲 + 一个 STOP**，让它把那个字节走完并释放 SDA。
    //    做完之后**重试一次配置**（只重试一次，避免和失败时无限重启）。
    localparam integer REC_HALF = 250;    // 半周期拍数：5 µs @50MHz ⇒ ≈100 kHz

    reg        rec_active, rec_retried;
    reg [4:0]  rec_ph;                    // 0..17 SCL 脉冲；18 STOP 前；19 STOP 后
    reg [8:0]  rec_cnt;
    reg        rec_scl;
    reg        cfg_restart;               // 单拍脉冲 → mpr121_cfg 从头再来

    wire rec_sda_low = (rec_ph == 5'd18);   // STOP 的建立相位：SDA 拉低

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rec_active  <= 1'b0;
            rec_retried <= 1'b0;
            rec_ph      <= 5'd0;
            rec_cnt     <= 9'd0;
            rec_scl     <= 1'b1;
            cfg_restart <= 1'b0;
        end else begin
            cfg_restart <= 1'b0;

            if (!rec_active) begin
                // cfg 失败且还没重试过 ⇒ 启动一次总线恢复
                if (cfg_err && !rec_retried) begin
                    rec_active <= 1'b1;
                    rec_ph     <= 5'd0;
                    rec_cnt    <= 9'd0;
                    rec_scl    <= 1'b1;
                end
            end else begin
                if (rec_cnt == (REC_HALF-1)) begin
                    rec_cnt <= 9'd0;
                    if (rec_ph >= 5'd19) begin
                        // 恢复 + STOP 都做完了 ⇒ 重试配置
                        rec_active  <= 1'b0;
                        rec_retried <= 1'b1;
                        cfg_restart <= 1'b1;
                    end else begin
                        rec_ph <= rec_ph + 5'd1;
                        if (rec_ph < 5'd18) rec_scl <= ~rec_scl;   // 9 个脉冲
                        else                 rec_scl <= 1'b1;     // STOP 期间 SCL 保持高
                    end
                end else rec_cnt <= rec_cnt + 9'd1;
            end
        end
    end

    reg [15:0] st_cnt;
    reg        st_drive_low;   // 自检期间拉低
    reg [9:0]  rise_cnt;
    reg        rise_run;
    reg [3:0]  rise_bkt;
    reg [1:0]  st_ok;
    //  ↑ st_cnt/st_drive/st_ok 必须**先声明**（Verilog 不允许先用后声明）

    //  ★ 标准开漏写法（2026-10-30）：只留一个 1 位控制位 + 固定高阻，
    //     工具才能正确推断 IOBUF（带 IBUF）。原来"三元套三元 + 夹 1'b1"被当成纯输出。
    assign scl_oe_low = rec_active ? ~rec_scl : m_scl_oe;
    assign sda_oe_low = rec_active ?  rec_sda_low : m_sda_oe;




    i2c_master #(
        .CLK_HZ(CLK_HZ), .SCL_HZ(SCL_HZ), .TIMEOUT_CLK(TIMEOUT_CLK)
    ) u_i2c (
        .clk(clk), .rst_n(rst_n),
        .start(m_start), .wr(m_wr), .rd(m_rd), .rd_ack(m_rd_ack),
        .stop(m_stop), .tx_byte(m_tx),
        .rx_byte(m_rx),
        .busy(i2c_busy), .done(m_done), .nack(m_nack),
        .timeout(m_timeout), .op_count(),
        .scl(m_scl), .sda(m_sda),
        .scl_oe(m_scl_oe), .sda_oe(m_sda_oe),
        .scl_i(scl_i), .sda_i(sda_i)
    );

    // ---------------- cfg（初始化）----------------
    wire       cfg_go, cfg_op;
    wire [7:0] cfg_reg, cfg_wd;
    wire [1:0] cfg_len;

    wire [1:0]  cfg_addr_sel;
    wire        cfg_verify_bad;   // ★ 1 = ECR 回读不是 0x0C（电极没使能）
    wire [7:0]  cfg_ecr;
    //  ★ 从机地址 = 配置阶段探测到应答的那个（DEV + 0..3）；
    //    配置彻底失败后进入**全总线扫描**，此时用 cfg_dev_ovr 强制地址。
    wire [6:0]  cfg_dev_ovr;
    wire        cfg_dev_ovr_en;
    wire [6:0]  cfg_scan_hit;     // 扫描到的第一个应答地址（0 = 一个都没有）
    wire [6:0]  dev_cur = cfg_dev_ovr_en ? cfg_dev_ovr
                                         : (DEV + {5'd0, cfg_addr_sel});

    mpr121_cfg #(.DEV(DEV), .RESET_WAIT(RESET_WAIT)) u_cfg (
        .clk(clk), .rst_n(rst_n),
        .go(cfg_go), .op_read(cfg_op), .reg_addr(cfg_reg),
        .wr_data(cfg_wd), .rd_len(cfg_len),
        .xf_done(xf_done), .xf_err(xf_err),
        .init_done(init_done), .busy(), .err(cfg_err),
        .addr_sel(cfg_addr_sel), .restart(cfg_restart),
        .xf_rd0(xf_rd0), .verify_bad(cfg_verify_bad), .ecr_rd(cfg_ecr),
        .dev_ovr(cfg_dev_ovr), .dev_ovr_en(cfg_dev_ovr_en), .scan_hit(cfg_scan_hit)
    );

    // ---------------- touch_keys（轮询）----------------
    wire       tk_go, tk_op;
    wire [7:0] tk_reg, tk_wd;
    wire [1:0] tk_len;
    wire [7:0] tk_rd0, tk_rd1;

    touch_keys #(
        .DEV(DEV), .POLL_DIV(POLL_DIV), .DEB_N(DEB_N), .STARTUP(STARTUP)
    ) u_tk (
        .clk(clk), .rst_n(rst_n),
        .init_done(init_done),
        .go(tk_go), .op_read(tk_op), .reg_addr(tk_reg),
        .wr_data(tk_wd), .rd_len(tk_len),
        .xf_done(xf_done), .xf_err(xf_err),
        .rd0(tk_rd0), .rd1(tk_rd1),
        .key_raw(key_raw), .key_press(key_press), .key_release(key_release),
        .ovcf(ovcf), .i2c_err(i2c_err), .poll_stb()
    );

    // ---------------- 共享 i2c_xfer（按 init_done 二选一）----------------
    wire       xf_go = init_done ? tk_go  : cfg_go;
    wire       xf_op = init_done ? tk_op  : cfg_op;
    wire [7:0] xf_reg= init_done ? tk_reg : cfg_reg;
    wire [7:0] xf_wd = init_done ? tk_wd  : cfg_wd;
    wire [1:0] xf_len= init_done ? tk_len : cfg_len;

    i2c_xfer u_xfer (
        .clk(clk), .rst_n(rst_n),
        .go(xf_go), .op_read(xf_op), .dev(dev_cur),
        .reg_addr(xf_reg), .wr_data(xf_wd), .rd_len(xf_len),
        .rd0(xf_rd0), .rd1(xf_rd1), .rd2(xf_rd2),
        .busy(), .done(xf_done), .err(xf_err),
        .m_start(m_start), .m_wr(m_wr), .m_rd(m_rd), .m_rd_ack(m_rd_ack),
        .m_stop(m_stop), .m_tx(m_tx),
        .m_rx(m_rx), .m_done(m_done), .m_nack(m_nack), .m_timeout(m_timeout)
    );

    // ============================================================
    //  ★ 上电自检：SDA 释放时应当是**高**（模块 4.7K 上拉）
    //     若 0.6 ms 后仍读到 0 ⇒ 线被拉死（短地 / 接错脚 / 模块没上电）。
    //     采样点选在 `mpr121_cfg` 发出第一条事务之前。
    // ============================================================
    reg [19:0] bus_chk;      // ★ 加宽：采样点推到 1.8 ms（原来 16 位最大只到 1.3 ms）
    reg        sda_stuck;
    reg [1:0]  bus_lvl_r;      // bit1 = SDA 低, bit0 = SCL 低

    // ============================================================
    //  ★ 引脚自检（2026-10-30）：上电后 **强驱动**两根线为高，再回读。
    //     目的：判断 Gowin 到底有没有把 pad 电平接进 IBUF。
    //     （此前出现自相矛盾的读数：bit3 说"有线低"，W 却说"两根都高"。）
    //     ⚠️ 要**拔掉模块**再测：0.4 ms 的推挽高电平会和模块上拉/输出轻微打架。
    //     结果：{SDA 回读正确, SCL 回读正确}，两个都 1 才算读通路正常。
    // ============================================================

    // ============================================================
    //  ★ 2026-10-30 大幅简化：只保留**实时电平**，不再做定时采样/自检/测电容。
    //     原因：那几版诊断互相矛盾（读数不可信），堆得越多越难判断。
    //     W 字段 = {SDA 电平, SCL 电平}，**每帧都反映当下**：
    //        W=3 两根都高（空闲正常）；W=0 两根都低；1/2 只有一根高。
    // ============================================================

    //  bit3 不再用历史采样：直接看**当前**是否有一根线是低（总线忙时也会是 1，仅供诊断）。
    assign err_code = {(~sda_i | ~scl_i), cfg_verify_bad, i2c_err, cfg_err};
    //  ★ 配置成功时 V = ECR 回读值（应为 0x0C）；
    //    配置失败时 V = **全总线扫描到的第一个应答地址**（0x00 = 整个总线没有任何器件应答）。
    assign ecr_rd   = cfg_verify_bad ? {1'b0, cfg_scan_hit} : cfg_ecr;
    assign bus_lvl   = {bus_lvl_r, st_ok};
    assign rise_bkt_o = rise_bkt;

    // touch_keys 要读的是 i2c_xfer 的读回口
    assign tk_rd0 = xf_rd0;
    assign tk_rd1 = xf_rd1;

    // ---------------- 机械键：八度 + 滑音 ----------------
    octave_key #(
        .OCT_INIT(OCT_INIT), .OCT_MIN(OCT_MIN), .OCT_MAX(OCT_MAX),
        .CNT_WIDTH(MECH_CNT_W), .DEB_CNT(MECH_DEB), .ACTIVE_LOW(ACTIVE_LOW)
    ) u_oct (
        .clk(clk), .rst_n(rst_n),
        .key_dn_raw(key_dn_raw), .key_up_raw(key_up_raw),
        .cyc_raw(key_cyc_raw),
        .octave_sel(octave_sel), .dn_press(), .up_press(), .cyc_press()
    );

    glide_ctrl #(
        .CNT_WIDTH(MECH_CNT_W), .DEB_CNT(MECH_DEB),
        .ACTIVE_LOW(ACTIVE_LOW), .GLIDE_SHIFT(GLIDE_SHIFT)
    ) u_glide (
        .clk(clk), .rst_n(rst_n),
        .sw_raw(sw_glide_raw),
        .glide_en(glide_en), .glide_shift(glide_shift)
    );

endmodule

`default_nettype wire
