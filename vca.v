`timescale 1ns/1ps
// ============================================================
//  vca.v —— 压控放大器（P0-5）：采样点 × 包络增益
//
//  一句话：osc_dds 吐出一个采样点，adsr 给出一个增益，乘一下 = 发声。
//
//      sample_in[k] (signed 16) × gain_in[k] (unsigned 16)
//           → >>> GW 缩回 16 位量程 → 饱和 → sample_out[k]
//
//  ★ 为什么"1 个乘法器管 4 个声部"：
//     板上只有 118 个 DSP，很贵。不每个声部配一个，而是 1 个乘法器
//     在 4 个时隙里轮流算 4 个声部：
//         100 MHz ÷ 48 kHz = 2083 拍/帧，4 路各占 1 拍，余量 400 倍。
//     验收口径：**8 个 DSP 覆盖 32 声部**（32 ÷ 4 = 8）。
//
// ------------------------------------------------------------
//  ⚠️⚠️ 唯一的坑：signed × unsigned（这里栽过）
// ------------------------------------------------------------
//  `osc_dds.sample_out` 是 signed，`adsr.gain` 是 unsigned。
//  Verilog 规则：只要有一个操作数无符号，整个表达式就按无符号算，
//  有符号操作数被**零扩展**。实测：
//        reg signed [15:0] s = -100;  reg [15:0] u = 1000;
//        m32 = s * u;      // = 65436000，而不是 -100000
//  ⇒ 直接写 `sample_in * gain_in` 会**把负半周全部毁掉**
//    （波形只剩正半周，还叠一个巨大直流）。
//  ✅ 正确写法见下面 sa / ga 两行：**先各自显式拓宽成 32 位再乘**。
//     ⚠️ 注意 part-select（`sample_in[k*SW +: SW]`）的结果是**无符号**的，
//        所以必须 $signed() 一下才能当有符号数用。
//
//  ⚠️ 这个 bug **osc_dds 和 adsr 各自的 tb 都抓不到**（都只单独例化自己），
//     只有连起来才爆 ⇒ 所以 tb_vca.v 专门喂了一个负样本。
//
// ------------------------------------------------------------
//  二、缩放与饱和：其实不会溢出，别自己加复杂度
// ------------------------------------------------------------
//  满量程：|sample| ≤ 32767，gain ≤ 65535
//      |prod| ≤ 32767 × 65535 = 2.147e9 < 2^31     ✔ 32 位有符号装得下
//      prod >>> 16 之后 |值| ≤ 32766               ✔ 16 位装得下
//  所以 gain 不超过满量程时**不会溢出**，饱和只是防御性的
//  （万一以后把 gain 的量程改大，它才起作用）。
//  ⚠️ 右移必须用**算术右移 `>>>`**：`>>` 会给负数补 0，等于毁掉负半周。
//
// ------------------------------------------------------------
//  三、时序（LATENCY = NV + 1 拍，**恒定**）
// ------------------------------------------------------------
//      T0：sample_tick 那一拍锁存 NV 路输入，时隙回到 0
//      T1：算第 0 路    → 写进 sample_out[0]
//      T2：算第 1 路    → 写进 sample_out[1]
//      T3：算第 2 路    → 写进 sample_out[2]
//      T4：算第 3 路    → 写进 sample_out[3]，同时拉高 out_valid
//      T5：NV 路结果与 out_valid **同一拍**出现在端口上
//  ⚠️ 上游契约：`sample_tick` 那一拍 sample_in/gain_in 必须**稳定**
//     （振荡器和 ADSR 在帧内保持值不变），所以上游要**寄存输出**。
//
//  验收：8 个 DSP 覆盖 32 声部，时序收敛
//  验证：`tb_vca.v`（含"负样本"与"gain=0 → 输出恒 0"两条）
// ============================================================

module vca #(
    // 一个乘法器服务几个声部（32 声部 ÷ 4 = 8 个 DSP）
    parameter integer NV       = 4,
    // 采样点位宽（有符号，来自 osc_dds 的 OUT_W）
    parameter integer SW       = 16,
    // 增益位宽（无符号，来自 adsr 的 GAIN_W）
    parameter integer GW       = 16,
    // 自 sample_tick 起、到 out_valid 的拍数（**必须恒定**）
    parameter integer LATENCY  = NV + 3
)(
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    sample_tick,   // 48 kHz 节拍，一帧的起点

    // NV 路声部输入（扁平总线：声部 k 在 [k*SW +: SW] / [k*GW +: GW]）
    input  wire signed [NV*SW-1:0] sample_in,
    input  wire        [NV*GW-1:0] gain_in,

    // NV 路结果（同一拍一起有效）
    output reg  signed [NV*SW-1:0] sample_out,
    output reg                     out_valid
);

    // ============================================================
    //  1) 时隙与输入锁存
    // ============================================================
    reg [15:0] slot;        // 当前服务第几路（0 .. NV-1）
    reg        busy;        // 一帧内"正在逐路算"的标志
    //  ⚠️ 2026-10-25：输入锁存**推迟 1 拍**（tick 的下一拍）。
    //     原因：`gain_in` 来自 voice_array 的 atten_scale，那条组合路径
    //     与"锁存输入"挤在同一拍里，实测 slack −0.32 ns（Fmax 48.7 MHz）。
    //     推迟一拍后，gain 通路有了整一拍的余量；因为 osc/包络的输出
    //     只在 tick 沿变化，帧内保持稳定，晚一拍锁存拿到的仍是同一帧的数据。
    //     代价：LATENCY 从 NV+1 变成 **NV+2**（本模块 5 → 6）。
    reg [1:0]  start_lat;   // tick 之后**两**拍才真正锁存（2026-10-26 再加一级）

    reg signed [NV*SW-1:0] sample_r;
    reg        [NV*GW-1:0] gain_r;

    // ============================================================
    //  2) 当前时隙的乘法 + 缩放 + 饱和
    //
    //  ⚠️ 这里就是本模块唯一容易写错的地方，逐行说明：
    //     · slot*SW +: SW 是**变量 part-select**，拿到的是无符号切片
    //     · $signed() 把它当有符号数读 → 负数才是负数
    //     · ga 前面补 (32-GW) 个 0 变 32 位、再声明有符号
    //       → 最高位恒 0，所以它的有符号值就是它自己，不会变负
    //     · 两边都是 32 位有符号 → Verilog 做**有符号乘法** ✔
    // ============================================================
    wire signed [SW-1:0] s_cur = sample_r[slot*SW +: SW];
    wire        [GW-1:0] g_cur = gain_r  [slot*GW +: GW];

    wire signed [31:0]   sa    = $signed(s_cur);
    wire signed [31:0]   ga    = $signed({{(32-GW){1'b0}}, g_cur});
    wire signed [31:0]   prod  = sa * ga;

    // 缩放：>>> GW 相当于"除以满量程"；用算术右移保住负数的正确性
    wire signed [31:0]   q     = prod >>> GW;

    // 饱和（防御性，见文件头"二"）
    localparam signed [SW-1:0] SAT_MAX = (1 <<< (SW-1)) - 1;   //  32767
    localparam signed [SW-1:0] SAT_MIN = -(1 <<< (SW-1));      // -32768
    wire signed [SW-1:0] y = (q > SAT_MAX) ? SAT_MAX :
                             (q < SAT_MIN) ? SAT_MIN : q[SW-1:0];

    // ============================================================
    //  3) 一帧的时序（见文件头"三"）
    // ============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            slot       <= 16'd0;
            busy       <= 1'b0;
            sample_r   <= {(NV*SW){1'b0}};
            gain_r     <= {(NV*GW){1'b0}};
            sample_out <= {(NV*SW){1'b0}};
            out_valid  <= 1'b0;
            start_lat  <= 2'b00;
        end else begin
            out_valid <= 1'b0;
            //  ⚠️ 把 sample_tick 在两级移位里推进：tick 那拍装 01，
            //     下一拍变 10 ⇒ 再下一拍 start_lat[1]=1 才真正锁存输入。
            //     （第一版写成 `start_lat <= 2'b01` 又去判 [1]，[1] 永远到不了，
            //       整模块不出数 —— 移位写法才对。）
            start_lat <= {start_lat[0], sample_tick};

            if (start_lat[1]) begin
                // tick 之后第 2 拍：锁存这一帧的全部输入，从第 0 路重新开始
                sample_r  <= sample_in;
                gain_r    <= gain_in;
                slot      <= 16'd0;
                busy      <= 1'b1;
            end else if (busy && !sample_tick) begin
                // 一路一拍：算完直接写进输出总线的对应段
                sample_out[slot*SW +: SW] <= y;

                if (slot == NV-1) begin
                    // 最后一路：下一拍结果和 out_valid 同时有效
                    busy      <= 1'b0;
                    slot      <= 16'd0;
                    out_valid <= 1'b1;
                end else begin
                    slot <= slot + 16'd1;
                end
            end
        end
    end

endmodule
