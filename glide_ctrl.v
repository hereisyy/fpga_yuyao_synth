// ============================================================
//  glide_ctrl.v —— 自锁滑音开关 → glide_en
//
//  ---- 这个模块只做三件事 ----
//    1) 2 级同步（开关是异步输入，不跨时钟域直接用电平会采到亚稳态）
//    2) 去抖（复用 `..\06_keyboard\debounce.v`，与八度键同一套时基）
//    3) 输出 `glide_en` 电平 + `glide_shift` 档位
//
//  ---- 为什么是"自锁开关"而不是"按住"----
//    演奏时双手都在琴键上，没有第三只手一直按住开关。
//    自锁开关给出的是**稳定电平**，所以本模块里没有任何边沿/翻转逻辑。
//
//  ---- glide_shift 为什么是 4 位 ----
//    与 P1 `08_P1\key_scan.v` 的同名端口保持**位宽一致**（那里是 `output [3:0] glide_shift`），
//    否则集成时还要多做一次位宽适配。取值沿用 `keyboard.v` 的档位：10=快 / 12=默认 / 13=慢。
//
//  ---- glide_en 的语义（交接要点，别在本模块里实现滑音本身）----
//    `glide_en = 1` ⇒ 引擎进 **mono/legato**：新音按下时改指同一声部、
//      从前一个音的音高滑过去、**不重触发包络**。
//    `test_mode`（P1 的数峰演示）**必须强制 glide_en = 0**，
//      否则数峰演示会被拉成单声部，评委只看到一根峰。
//    滑音**本体**是 `glide.v`（P2 已交付），本模块只交开关。
//
//  ---- 滑音时间 ----
//    没有模拟量了（X/Y 已删），所以 `glide_shift` 是**编译期参数**。
//    τ = 2^shift / 48000 秒：shift=10 → 21.3 ms / 12 → 85.3 ms / 13 → 171 ms。
// ============================================================
`default_nettype none

module glide_ctrl #(
    parameter integer CNT_WIDTH   = 18,
    parameter integer DEB_CNT     = 200_000,   // 去抖拍数（50 MHz ⇒ 4 ms）
    parameter         ACTIVE_LOW  = 1,         // 开关接地 ⇒ 合上 = 低
    parameter [3:0]   GLIDE_SHIFT = 4'd12      // τ = 2^shift / 48000
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       sw_raw,        // 自锁开关（异步电平）

    output reg        glide_en,      // 去抖后的电平：1 = 滑音开
    output wire [3:0] glide_shift    // 滑音时间档位（编译期常量）
);

    assign glide_shift = GLIDE_SHIFT;

    // ---------------- 2 级同步 ----------------
    reg [1:0] sw_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) sw_sync <= 2'b11;          // 上电按"开关断开"处理
        else        sw_sync <= {sw_sync[0], sw_raw};
    end

    wire sw_in = ACTIVE_LOW ? ~sw_sync[1] : sw_sync[1];   // 高 = 开关合上

    // ---------------- 去抖 ----------------
    wire sw_db;
    debounce #(.CNT_WIDTH(CNT_WIDTH), .CNT_MAX(DEB_CNT)) u_db (
        .clk(clk), .rst_n(rst_n), .key_in(sw_in), .key_out(sw_db)
    );

    // ---------------- 输出 ----------------
    //  去抖器输出上电就是 0（key_out 复位为 0）⇒ **glide_en 默认关**，
    //  这是有意的：默认打开滑音会让"弹什么都在滑"，初次上电就会被当成故障。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) glide_en <= 1'b0;
        else        glide_en <= sw_db;
    end

endmodule

`default_nettype wire