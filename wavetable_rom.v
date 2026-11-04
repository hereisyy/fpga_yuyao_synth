`timescale 1ns/1ps
// ============================================================
//  wavetable_rom.v —— 音频波表 ROM（P0-2）
//
//  它是什么：一块 4096 × 12bit 的 ROM，里面**并排存了四张 1024 点波表**
//            （sine / square / sawtooth / triangle），用地址高位选波形。
//
//      rom 地址 = {wave_sel[1:0], addr[9:0]}
//                        ↑选哪张表      ↑表内第几个点
//
//      波形 0 = sine      地址    0 ~ 1023
//      波形 1 = square    地址 1024 ~ 2047
//      波形 2 = sawtooth  地址 2048 ~ 3071
//      波形 3 = triangle  地址 3072 ~ 4095
//      （顺序由 gen_wavetable.py --also-combined 保证，与 tb_wavetable.v 的
//        WAVE_ID 编号一致，改任何一个都要同步改另一边）
//
//  容量核对：4096 × 12 bit = 48 Kbit。
//            GW5AT-60B 一个 BSRAM 块是 18 Kbit（2124 Kbit / 118 块），
//            所以这块 ROM 占 **3 个 BSRAM 块**（甚至可能是 2~3 块，取决于
//            综合工具的拼块策略）。全片 118 块，完全不心疼。
//
// ------------------------------------------------------------
//  ⚠️ 三个必须记住的性质（后面的 osc_dds.v 全靠它们）
// ------------------------------------------------------------
//
//  ① **输出是寄存的，延迟恰好 1 拍**。
//     地址在第 T 拍给出，数据在第 T+1 拍出现。这不是"多打了一拍"，
//     而是 BSRAM 的硬性要求：块 RAM 内部有输出寄存器，地址→数据
//     不可能在同一个时钟沿内完成。谁要是把它写成组合输出，
//     综合器要么报错，要么偷偷塞进 LUT RAM（→ 占掉几百个 LUT，
//     而且综合报告里看不到 BSRAM）。
//
//     **这条延迟的代价**：osc_dds.v 里取小数部分（相位低 22 位）
//     的那一级流水线**必须跟着延迟 1 拍**，否则插值会把
//     "这个点的小数"配到"上一个点的整数"上 —— 听起来是轻微的
//     沙哑/毛刺，频谱上看不出明显错，但 SINAD 上不去。
//     对齐方法见 osc_dds.v。
//
//  ② **两个读口（port A / port B）是给线性插值用的**。
//     线性插值需要同一时刻拿到 y[n] 和 y[n+1]：
//         port A 给 addr       （即 n）
//         port B 给 addr + 1   （即 n+1，1023+1 自然回绕到 0）
//     两个口是**同一个时钟、同一拍出数据**，所以天生对齐。
//     两个口连到同一个数组，综合出来是"伪双口 ROM"。
//     如果综合报告显示每张表占了 2 个块（共 8 块），也是对的，
//     不要为此去改代码 —— 8/118 依然不是瓶颈。
//
//  ③ **表是周期性的，回绕靠位宽自然完成**。
//     addr_b = addr_a + 1 在 addr_a = 1023 时，10 位加法溢出成 0，
//     正好指向本周期的第一个点 —— 这正是周期波形要的。
//     **不要**在这里写成 `if (addr_a == 1023) addr_b = 0;`，
//     多写一句反而可能在综合时多出一级逻辑。加法器的位宽就是回绕。
//
// ------------------------------------------------------------
//  ⚠️ 一个"仿真通过、上板无声"的经典坑（已在生成脚本里绕开）
// ------------------------------------------------------------
//  四张表本来是四个文件。用 start/end 版本的 $readmemh 分别读：
//      $readmemh("wavetable_sine.hex",     rom,    0, 1023);
//      $readmemh("wavetable_square.hex",   rom, 1024, 2047);
//      ...
//  **带 start/end 的 $readmemh 各家综合工具支持不一致**：有的忽略后两个
//  参数，于是四张表全叠在开头、后面 3/4 是空的。Icarus 支持这个语法，
//  所以仿真是对的 —— 这类 bug 只有上板才暴露。
//
//  所以改用 gen_wavetable.py --also-combined 生成的 **wavetable_all.hex**
//  （4096 行，四张表按顺序拼好），这里只写一句**无参数**的 $readmemh：
//      $readmemh("wavetable_all.hex", rom);
//  仿真和综合的行为完全一致，坑不存在。
//
//  另外：$readmemh 的路径是相对**仿真器的工作目录**找的。
//  在 06_keyboard/ 目录下跑 vvp 才找得到；run_all_sim.ps1 已经处理了。
//  如果文件没找到，ROM 会全是 X（声音是静音/噪声），
//  tb_wavetable_rom.v 的第 1 项检查就是专门抓这个的。
//
//  依赖：wavetable_all.hex（P0-1 的生成脚本产出）✅
// ============================================================

module wavetable_rom #(
    parameter TABLE_LEN = 1024,                 // 每张表的点数
    parameter BITS      = 12,                   // 每个值的位宽（偏移二进制）
    parameter WAVE_NUM  = 4,                    // 表里有几张波形
    parameter INIT_FILE = "wavetable_all.hex",  // 四张表拼好的文件

    // ------------------------------------------------------------
    //  ⚠️ 地址位宽**必须写成独立参数，不能在模块体里用 localparam 推**
    //
    //  踩过的坑：一开始写的是
    //      module wavetable_rom (... port 用 ADDR_W ...);
    //          function integer clog2; ... endfunction
    //          localparam ADDR_W = clog2(TABLE_LEN);
    //  端口列表在**模块体之前**，此时 ADDR_W 还没声明，iverilog 直接报
    //      "Unable to bind parameter `ADDR_W'  (Check for declaration after use)"
    //  然后退化成 1 位端口、把 10 位地址的高 9 位全裁掉
    //  （报错里那句 "Port addr_a expects 1 bit(s), given 10" 就是这个后果）。
    //  改用 $clog2 写在参数列表里也能编译，但 **$clog2 在部分综合工具里
    //  对参数默认值的支持不一致**，而这块板子的综合工具是 Gowin，
    //  不想在这种地方赌。直接写死位数，再用下面的自检挡住改错。
    //
    //  约束：ADDR_W 必须满足 2^ADDR_W = TABLE_LEN，WAVE_W 满足 2^WAVE_W = WAVE_NUM。
    //        改了 TABLE_LEN / WAVE_NUM 就必须同步改这两个（自检会提醒）。
    // ------------------------------------------------------------
    parameter ADDR_W    = 10,                   // 2^10 = 1024 = TABLE_LEN
    parameter WAVE_W    = 2                     // 2^2  = 4    = WAVE_NUM
)(
    input  wire                    clk,

    // ---- 读口 A：主地址（插值里的 n）----
    input  wire [ADDR_W-1:0]       addr_a,
    output reg  [BITS-1:0]         data_a,

    // ---- 读口 B：副地址（插值里的 n+1；不用就接 addr_a）----
    input  wire [ADDR_W-1:0]       addr_b,
    output reg  [BITS-1:0]         data_b,

    // ---- 波形选择：0=sine 1=square 2=sawtooth 3=triangle ----
    input  wire [WAVE_W-1:0]       wave_sel
);

    // ============================================================
    //  0) 位宽自检（**只在仿真期跑，不进综合**）
    //
    //  位宽参数是手写的，就可能和 TABLE_LEN/WAVE_NUM 不同步。
    //  不同步的后果不是报错，而是**静默丢位**（地址高位被裁掉，
    //  波形只跑到一半就回绕），或者 ROM 深度不够、尾部全是 X。
    //  所以在这里放一句断言，仿真一跑就炸，比上板再查便宜得多。
    //
    //  `translate_off` 是给综合工具看的："这段别综合"。
    //  即使某个工具不认这个指令也没关系 —— 一个只含 $display 的
    //  initial 块综合工具会直接忽略。
    // ============================================================
    // synthesis translate_off
    initial begin
        if ((1 << ADDR_W) != TABLE_LEN) begin
            $display("!! wavetable_rom: ADDR_W=%0d 与 TABLE_LEN=%0d 不匹配",
                     ADDR_W, TABLE_LEN);
            $display("   ADDR_W 必须满足 2^ADDR_W = TABLE_LEN，请手工改 ADDR_W");
            $finish;
        end
        if ((1 << WAVE_W) != WAVE_NUM) begin
            $display("!! wavetable_rom: WAVE_W=%0d 与 WAVE_NUM=%0d 不匹配",
                     WAVE_W, WAVE_NUM);
            $display("   WAVE_W 必须满足 2^WAVE_W = WAVE_NUM，请手工改 WAVE_W");
            $finish;
        end
    end
    // synthesis translate_on

    localparam DEPTH = TABLE_LEN * WAVE_NUM;    // 4096

    // ============================================================
    //  1) ROM 本体
    //
    //  (* syn_ramstyle = "block_ram" *) —— **不能省**。
    //  不加这一句，综合器看到"只读、小、两个口"，很可能推断成
    //  分布式 RAM（LUT RAM）：4096×12 要用掉约 4096/64×12 ≈ 700+ 个 LUT，
    //  在 59904 个 LUT4 的预算里是一大块，而且**它本来就不该占 LUT**。
    //  验收标准就是综合报告里这一项落在 BSRAM 一栏。
    // ============================================================
    (* syn_ramstyle = "block_ram" *) reg [BITS-1:0] rom [0:DEPTH-1];

    // ============================================================
    //  2) 地址拼接：高位选表，低位选点
    // ============================================================
    wire [ADDR_W+WAVE_W-1:0] ra = {wave_sel, addr_a};
    wire [ADDR_W+WAVE_W-1:0] rb = {wave_sel, addr_b};

    // ============================================================
    //  3) 两个读口都在时钟沿取数 → 输出寄存、延迟 1 拍
    //
    //  这里**不加复位**：ROM 内容是固定的，复位它没有意义，
    //  而且给 BSRAM 的输出寄存器加复位会**阻止它映射到块 RAM 的
    //  内置寄存器**（综合器只能额外塞一个带复位的 FF，还可能
    //  因此放弃 BSRAM 方案）。"上电初值"由 $readmemh 在配置阶段
    //  写进 BSRAM 完成，不需要也不应该有复位。
    // ============================================================
    always @(posedge clk) begin
        data_a <= rom[ra];
        data_b <= rom[rb];
    end

    // ============================================================
    //  4) 加载波表
    //
    //  综合工具会把 $readmemh 的内容直接**烧进 BSRAM 的初值**
    //  （对 Gowin 就是 .fs 比特流里 BSRAM 的 INIT 段），
    //  所以 FPGA 一上电表就在里面了，不需要从 Flash 搬。
    // ============================================================
    initial begin
        $readmemh(INIT_FILE, rom);
    end

endmodule
