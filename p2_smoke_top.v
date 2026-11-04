// ============================================================
//  p2_smoke_top.v —— **P2 单独上板验证**用的顶层（不是最终交付顶层）
//
//  ⚠️ 定位：这个文件只为"P2 自己能不能在板子上跑"服务。
//     最终设计的顶层是 P1 的 `top_synth.v`，**不要**把它当交付顶层。
//     它存在的理由：P1 的 32 声部还没合进来时，一上板什么都看不见——
//     板子上只有 1 颗 RGB。有了这个顶层，插上串口就能看到：
//         K=0FF O=3 G=1 E=0
//     （键位图 / 八度 / 滑音 / 错误位），正好覆盖 P2 的全部验收项。
//
//  ---- 串口 ----
//    走板载 BL616 桥的虚拟串口：`FPGA_UART.TX = U15`（rx=V14 本文件不用）。
//    默认 115200 8N1。
//
//  ---- LED（可选）----
//    R = init_done  G = 有任何键按下  B = 有错误(cfg_err|i2c_err|ovcf)
//    ⚠️ Console 的这颗 RGB **三路极性不一致**（见 `console_138k.cst` 的 LED 段）：
//         R：经 PMOS 反相器，READY=1 → 亮
//         G：直连阴极，     SYS_ACT=0 → 亮
//         B：经 PMOS 反相器，DONE=1 → 亮
//       所以这里**逐路**按原理图处理，不能用单一"低有效"参数糊过去。
//    ⚠️ 这三根脚在最终设计里判给 P1（见 接口契约.md §4），**本文件仅用于单独验证**。
// ============================================================
`default_nettype none

module p2_smoke_top #(
    // ---- I2C ----
    parameter [6:0]   DEV         = 7'h5A,
    parameter integer CLK_HZ      = 50_000_000,
    parameter integer SCL_HZ      = 100_000,
    parameter integer TIMEOUT_CLK = 50_000,
    // ---- 串口 ----
    parameter integer BAUD        = 115_200,
    // ---- 触摸 ----
    parameter integer RESET_WAIT  = 50_000,
    parameter integer POLL_DIV    = 50_000,
    parameter integer DEB_N       = 3,
    parameter integer STARTUP     = 10_000,
    // ---- 机械键 ----
    parameter [2:0]   OCT_INIT    = 3'd2,
    parameter [2:0]   OCT_MIN     = 3'd1,
    parameter [2:0]   OCT_MAX     = 3'd4,
    parameter integer MECH_DEB    = 200_000,
    parameter [3:0]   GLIDE_SHIFT = 4'd12,
    // ---- 心跳 ----
    parameter integer HEARTBEAT   = 50_000_000   // 1 s @50MHz
)(
    input  wire        clk,
    input  wire        rst_n,

    inout  wire        scl_pad,   // 顶层：pad 由本模块驱动/读取
    inout  wire        sda_pad,

    input  wire        key_dn_raw,
    input  wire        key_up_raw,
    input  wire        sw_glide_raw,

    output wire        uart_tx,

    output wire        dbg_led_r,   // init_done
    output wire        dbg_led_g,   // 有键按下
    output wire        dbg_led_b    // 有错误
);

    //  ★ pad 三态由本模块（顶层）负责
    wire scl_oe_low, sda_oe_low;
    assign scl_pad = scl_oe_low ? 1'b0 : 1'bz;
    assign sda_pad = sda_oe_low ? 1'b0 : 1'bz;

    // ---------------- 被测核心 ----------------
    wire [11:0] key_raw, key_press, key_release;
    wire [2:0]  octave_sel;
    wire        glide_en;
    wire [3:0]  glide_shift;
    wire        init_done, cfg_err, i2c_err, ovcf, i2c_busy;

    p2_input_top #(
        .DEV(DEV), .CLK_HZ(CLK_HZ), .SCL_HZ(SCL_HZ), .TIMEOUT_CLK(TIMEOUT_CLK),
        .RESET_WAIT(RESET_WAIT), .POLL_DIV(POLL_DIV), .DEB_N(DEB_N), .STARTUP(STARTUP),
        .OCT_INIT(OCT_INIT), .OCT_MIN(OCT_MIN), .OCT_MAX(OCT_MAX),
        .MECH_CNT_W(18), .MECH_DEB(MECH_DEB), .ACTIVE_LOW(1), .GLIDE_SHIFT(GLIDE_SHIFT)
    ) u_core (
        .clk(clk), .rst_n(rst_n),
        .scl_i(scl_pad), .sda_i(sda_pad),
        .scl_oe_low(scl_oe_low), .sda_oe_low(sda_oe_low),
        .key_dn_raw(key_dn_raw), .key_up_raw(key_up_raw), .sw_glide_raw(sw_glide_raw),
        .key_cyc_raw(1'b1),               // 冒烟顶层不带板载键 ⇒ 常接"未按下"
        .key_raw(key_raw), .key_press(key_press), .key_release(key_release),
        .octave_sel(octave_sel), .glide_en(glide_en), .glide_shift(glide_shift),
        .init_done(init_done), .cfg_err(cfg_err), .i2c_err(i2c_err),
        .ovcf(ovcf), .i2c_busy(i2c_busy), .err_code()
    );


    // ---------------- LED ----------------
    //  逐路按原理图极性处理（R/B 反相器后接 READY/DONE，G 直连 SYS_ACT）
    assign dbg_led_r = init_done;                       // READY=1 → R 亮
    assign dbg_led_g = ~(|key_raw);                     // SYS_ACT=0 → G 亮（有键 ⇒ 亮）
    assign dbg_led_b = cfg_err | i2c_err | ovcf;        // DONE=1  → B 亮


    // ---------------- 串口打印 ----------------
    //  行格式固定 19 字节：  K=xxx O=n G=g E=e CR LF
    //  'K' 居首：上位机/本 tb 都用它当行首标记
    reg        u_send;
    reg  [7:0] u_data;
    wire       u_busy;

    uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_uart (
        .clk(clk), .rst_n(rst_n), .send(u_send), .data(u_data), .tx(uart_tx), .busy(u_busy)
    );

    function [7:0] hexasc(input [3:0] v);
        hexasc = (v < 4'd10) ? (8'h30 + {4'd0, v}) : (8'h37 + {4'd0, v});   // '0'..'9' / 'A'..'F'
    endfunction

    // 按下标取这一行的第 i 个字节（0..18）
    function [7:0] line_byte(
        input [4:0]  i,
        input [11:0] k,
        input [2:0]  o,
        input        g,
        input        e
    );
        begin
            case (i)
            5'd0 : line_byte = 8'h4B;                 // 'K'
            5'd1 : line_byte = 8'h3D;                 // '='
            5'd2 : line_byte = hexasc(k[11:8]);
            5'd3 : line_byte = hexasc(k[7:4]);
            5'd4 : line_byte = hexasc(k[3:0]);
            5'd5 : line_byte = 8'h20;                 // ' '
            5'd6 : line_byte = 8'h4F;                 // 'O'
            5'd7 : line_byte = 8'h3D;                 // '='
            5'd8 : line_byte = hexasc({1'b0, o});
            5'd9 : line_byte = 8'h20;
            5'd10: line_byte = 8'h47;                 // 'G'
            5'd11: line_byte = 8'h3D;                 // '='
            5'd12: line_byte = g ? 8'h31 : 8'h30;     // '1' / '0'
            5'd13: line_byte = 8'h20;
            5'd14: line_byte = 8'h45;                 // 'E'
            5'd15: line_byte = 8'h3D;                 // '='
            5'd16: line_byte = e ? 8'h31 : 8'h30;
            5'd17: line_byte = 8'h0D;                 // CR
            5'd18: line_byte = 8'h0A;                 // LF
            default: line_byte = 8'h0A;
            endcase
        end
    endfunction

    localparam [4:0] LINE_LAST = 5'd18;

    reg [2:0]  p_state;
    reg [4:0]  p_idx;
    reg [11:0] snap_k;
    reg [2:0]  snap_o;
    reg        snap_g, snap_e;

    // 变化检测
    reg [11:0] k_d;
    reg [2:0]  o_d;
    reg        g_d;
    reg        init_d;

    // 心跳
    reg [31:0] hb_cnt;

    wire key_chg  = (key_raw    != k_d);
    wire oct_chg  = (octave_sel != o_d);
    wire g_chg    = (glide_en   != g_d);
    wire init_rise= (init_done  && !init_d);

    // 触发条件：初始化完成 / 键位变化 / 八度变化 / 滑音变化 / 心跳
    //   ⚠️ 初始化完成之前不打印（那时 UART 上出现的都是垃圾）
    wire trig = init_done && (init_rise || key_chg || oct_chg || g_chg ||
                              (hb_cnt >= HEARTBEAT[31:0]));

    localparam [2:0]
        P_IDLE = 3'd0,
        P_GO   = 3'd1,
        P_WAIT = 3'd2,
        P_STOP = 3'd3;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            p_state   <= P_IDLE;
            p_idx     <= 5'd0;
            snap_k    <= 12'd0;
            snap_o    <= 3'd0;
            snap_g    <= 1'b0;
            snap_e    <= 1'b0;
            u_send    <= 1'b0;
            u_data    <= 8'd0;
            k_d       <= 12'd0;
            o_d       <= 3'd0;
            g_d       <= 1'b0;
            init_d    <= 1'b0;
            hb_cnt    <= 32'd0;
        end else begin
            u_send <= 1'b0;

            // 跟踪上一次的值（用于变化检测）
            k_d    <= key_raw;
            o_d    <= octave_sel;
            g_d    <= glide_en;
            init_d <= init_done;

            // 心跳计数
            if (hb_cnt >= HEARTBEAT[31:0]) hb_cnt <= 32'd0;
            else                           hb_cnt <= hb_cnt + 32'd1;

            case (p_state)
            P_IDLE: begin
                if (trig) begin
                    snap_k <= key_raw;
                    snap_o <= octave_sel;
                    snap_g <= glide_en;
                    snap_e <= cfg_err | i2c_err | ovcf;
                    p_idx  <= 5'd0;
                    hb_cnt <= 32'd0;
                    p_state<= P_GO;
                end
            end

            P_GO: begin
                if (!u_busy) begin
                    u_send <= 1'b1;
                    u_data <= line_byte(p_idx, snap_k, snap_o, snap_g, snap_e);
                    p_state<= P_WAIT;
                end
            end

            P_WAIT: begin
                if (u_busy) p_state <= P_STOP;   // 等它真的开始
            end

            P_STOP: begin
                if (!u_busy) begin               // 这一字节发完了
                    if (p_idx == LINE_LAST) p_state <= P_IDLE;
                    else begin
                        p_idx  <= p_idx + 5'd1;
                        p_state<= P_GO;
                    end
                end
            end

            default: p_state <= P_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
