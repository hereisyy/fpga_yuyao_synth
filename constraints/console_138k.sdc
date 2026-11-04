# ============================================================
#  console_138k.sdc —— 全系统构建（目标 B）的时序约束
#
#  ⚠️ 本文件是**模板**，不是已验证的成品：目标 B（全系统顶层）还没写（P1-6）。
#     现在能烧的是目标 A（p2_smoke_top + constraints\p2_smoke.sdc）。
#
#  在 Gowin 里：Project → Add Files → 选本文件（.sdc）。
#  注意 .sdc 是 **Tcl 语法**（不是 Verilog）：
#      · 端口名要加大括号 [get_ports {sys_clk}]
#      · 注释用 #，不能用 //
#
#  系统时钟来源：SOM 的 Y1100 50 MHz 有源晶振 → R973 0R → V22
# ============================================================

# ---- 方案 1（当前默认）：不生成 PLL，直接用 50 MHz ----
#   i2s_tx 的帧锁定在 sample_tick 上，50 MHz 下 SAMPLE_DIV=1042
#   ⇒ 实测采样率 47.985 kHz（−0.53 音分，听不出来）。
#   ⚠️ 此时 P1 的 key_scan 必须例化 SYS_CLK_HZ = 50_000_000（默认是 100M！）
create_clock -name sys_clk -period 20.000 -waveform {0 10.000} [get_ports {sys_clk}]

# ---- 方案 2（推荐上板后再做）：rPLL 50 MHz → 150 MHz ----
#   理由与音准无关，是①抖动余量（±6.7 ns vs ±20 ns）②32 声部的算力。
#   用 IP Core Generator 生成（IDIV=1/FBDIV=12/ODIV=4，VCO=600 MHz），
#   然后把下面两行替换掉上面那行：
#
# create_clock -name sys_clk_50  -period 20.000 -waveform {0 10.000} [get_ports {sys_clk}]
# create_clock -name clk_sys     -period  6.667 -waveform {0  3.333} [get_nets {pll_lock_clk}]
#
#   （PLL 输出的网名以 IP 生成的实际名字为准，别照抄。）

# ---- 音频输出：不用额外约束 ----
#   BCLK ≈ 1.536 MHz 由 sample_tick 均分产生，不是独立时钟域，不需要 create_generated_clock。