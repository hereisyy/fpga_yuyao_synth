// ============================================================
//  octave_key.v —— 2 个机械键（−/＋）→ octave_sel[2:0]
//
//  为什么是机械键而不是触摸通道：
//    MPR121 只有 12 个电极通道，**正好被 12 个琴键占满** ⇒ 八度没有触摸通道可用。
//    这是算术，不是取舍。
//
//  ---- 极性 ----
//    PMOD 上的机械键用 FPGA 内部上拉 + 按键接地 ⇒ **按下 = 低电平**。
//    本模块内部取反成"高 = 按下"（`ACTIVE_LOW=1`），与 `key_raw` 的极性约定一致。
//    顶层不需要再翻一次。
//
//  ---- 档位与音域 ----
//    输出直接驱动 `keyboard.v` / `key_scan.v` 的 `octave_sel[2:0]`。
//    消费侧（`keyboard.v`）的换算是 `oct_p3 = octave_sel + 3`、`midi = oct_p3*12 + key`：
//        sel=0 → C2   sel=2 → C4（默认）   sel=4 → C6   sel=7 → 越界（freq_table 会钳到 B7）
//    所以默认取 **OCT_MIN=1 / OCT_MAX=4** ⇒ 4 档 = **C3~B6**，
//    既覆盖常用音域，又不会踩到 `freq_table`(MIDI 24~107) 的钳位区。
//
//  ---- 去抖 ----
//    复用 `..\06_keyboard\debounce.v`（同步状态机，非 RC）。前面加 2 级同步器，
//    因为按键是异步输入。默认 200000 拍 @50 MHz ≈ 4 ms。
// ============================================================
`default_nettype none

module octave_key #(
    parameter [2:0]   OCT_INIT   = 3'd2,      // 上电默认档（与 keyboard.v 的默认一致）
    parameter [2:0]   OCT_MIN    = 3'd1,      // 最低档（含）
    parameter [2:0]   OCT_MAX    = 3'd4,      // 最高档（含）
    parameter integer CNT_WIDTH  = 18,
    parameter integer DEB_CNT    = 200_000,   // 去抖拍数（50 MHz ⇒ 4 ms）
    parameter         ACTIVE_LOW = 1
)(
    input  wire       clk,
    input  wire       rst_n,

    input  wire       key_dn_raw,   // 八度 −（异步，低有效）
    input  wire       key_up_raw,   // 八度 ＋（异步，低有效）
    input  wire       cyc_raw,      // ★ 八度**循环**键（异步，低有效）—— 板载 EX_KEY.1 = AB13

    output reg  [2:0] octave_sel,   // 送给 keyboard.v / key_scan.v
    output reg        dn_press,     // 单周期脉冲：− 被按下（已去抖）
    output reg        up_press,     // 单周期脉冲：＋ 被按下（已去抖）
    output reg        cyc_press     // 单周期脉冲：循环键被按下（已去抖）
);

    // ---------------- 2 级同步 ----------------
    reg [1:0] dn_sync, up_sync, cyc_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dn_sync  <= 2'b11;
            up_sync  <= 2'b11;
            cyc_sync <= 2'b11;
        end else begin
            dn_sync  <= {dn_sync[0],  key_dn_raw};
            up_sync  <= {up_sync[0],  key_up_raw};
            cyc_sync <= {cyc_sync[0], cyc_raw};
        end
    end

    wire dn_in  = ACTIVE_LOW ? ~dn_sync[1]  : dn_sync[1];   // 高 = 按下
    wire up_in  = ACTIVE_LOW ? ~up_sync[1]  : up_sync[1];
    wire cyc_in = ACTIVE_LOW ? ~cyc_sync[1] : cyc_sync[1];

    // ---------------- 去抖（复用既有模块，不重写）----------------
    wire dn_db, up_db;
    debounce #(.CNT_WIDTH(CNT_WIDTH), .CNT_MAX(DEB_CNT)) u_dn (
        .clk(clk), .rst_n(rst_n), .key_in(dn_in), .key_out(dn_db)
    );
    debounce #(.CNT_WIDTH(CNT_WIDTH), .CNT_MAX(DEB_CNT)) u_up (
        .clk(clk), .rst_n(rst_n), .key_in(up_in), .key_out(up_db)
    );
    wire cyc_db;
    debounce #(.CNT_WIDTH(CNT_WIDTH), .CNT_MAX(DEB_CNT)) u_cyc (
        .clk(clk), .rst_n(rst_n), .key_in(cyc_in), .key_out(cyc_db)
    );

    // ---------------- 边沿 → 换档 ----------------
    reg dn_d, up_d, cyc_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dn_d       <= 1'b0;
            up_d       <= 1'b0;
            cyc_d      <= 1'b0;
            octave_sel <= OCT_INIT;
            dn_press   <= 1'b0;
            up_press   <= 1'b0;
            cyc_press  <= 1'b0;
        end else begin
            dn_d      <= dn_db;
            up_d      <= up_db;
            cyc_d     <= cyc_db;
            dn_press  <= 1'b0;
            up_press  <= 1'b0;
            cyc_press <= 1'b0;

            if (dn_db && !dn_d) begin
                dn_press <= 1'b1;
                // 同时按下两个键视为无效操作（不换档）
                if (!(up_db && !up_d) && (octave_sel > OCT_MIN))
                    octave_sel <= octave_sel - 3'd1;
            end

            if (up_db && !up_d) begin
                up_press <= 1'b1;
                if (!(dn_db && !dn_d) && (octave_sel < OCT_MAX))
                    octave_sel <= octave_sel + 3'd1;
            end

            // ★ 循环键：+1，到上限回最低档（放在最后 ⇒ 同一拍与 dn/up 冲突时它优先）
            if (cyc_db && !cyc_d) begin
                cyc_press  <= 1'b1;
                octave_sel <= (octave_sel >= OCT_MAX) ? OCT_MIN
                                                      : octave_sel + 3'd1;
            end
        end
    end

endmodule

`default_nettype wire