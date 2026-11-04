# 时序约束（P2 单独冒烟构建用）
#
# Gowin EDA：Project → Add Files → 选本文件（.sdc）
# 系统时钟来自 SOM 的 Y1100 50 MHz 有源晶振（经 R973 0R 到 V22）。
create_clock -name clk -period 20 -waveform {0 10} [get_ports {clk}]