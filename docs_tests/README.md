# docs_tests —— `11_board` 的文档与测试（**不进 Gowin**）

> 建立：2026-10-25。`11_board\` 里现在**只留要交给 Gowin 的东西**：
> **35 个 `.v` + 2 个 `.hex` + `constraints\`**（4 个 `.cst`/`.sdc`）。
> 其余（文档、脚本、清单、testbench）全部挪到了本目录。
>
> ```
> 11_board\
> ├── *.v  (35)              ← 全部综合源（顶层 = top_console）
> ├── *.hex (2)              ← $readmemh 要读的数据，必须与 .v 同目录
> ├── constraints\           ← 目标 B: console_138k_top_console.cst + console_138k.sdc
> │                            目标 A: p2_smoke.cst + p2_smoke.sdc
> └── docs_tests\            ← 本目录（文档 + 测试 + 同步/自检脚本）
>     ├── README.md          ← 本文件
>     ├── 缺什么.md           ← 与"完整可跑"的差距清单
>     ├── _manifest.csv      ← 逐文件来源 + SHA256（同步脚本的白名单）
>     ├── tb_board.v         ← ★ 板级自检 testbench（跑的就是上一级那套 RTL）
>     ├── run_board_sim.ps1  ← ★ 一键：elaborate 三个顶层 + 跑 tb_board
>     ├── smoke_compile.ps1  ← 只做 elaborate（更快）
>     └── sync_from_sources.ps1 ← 用源目录刷新上一级（禁止把 tb 带过去）
> ```

⚠️ **在 Gowin 里加文件时，千万别把 `docs_tests\` 递归加进去**：
里面有 `tb_board.v`（testbench，含 `pullup` / `$fdisplay`，综合会报错）。
正确做法：只选 `11_board\` **根目录的 `.v`** + `constraints\` 里的两个约束文件。

---

## 一、上板前的一次完整自检（照着敲就行）

### 第 0 步：确认工具在

```powershell
iverilog -V
```

看到 `Icarus Verilog version 14.0` 之类就行。看不到说明没装/没进 PATH ——
本机是 `D:\Self\Softwares\iverilog\bin`，脚本里已内置这个兜底路径。

### 第 1 步：进目录

```powershell
cd D:\Self\Files\Basement\FPGA1\1\1\11_board\docs_tests
```

> 必须在这里跑：`tb_board.v` 里波表路径写的是 `../wavetable_all.hex`
> （`$readmemh` 相对**仿真器工作目录**解析，不是相对源文件）。

### 第 2 步：跑板级自检

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\run_board_sim.ps1
```

正常应看到（实测约 3 分钟，因为要跑 2500 帧音频）：

```
---------- ① 编译（elaborate）----------
  top_console      OK
  p2_smoke_top     OK
  top_synth        OK
---------- ② 仿真（tb_board）----------
  tb_board         PASS  ok=26  fail=0
✅ 全通过：这套 RTL 可以在 Gowin 里做综合 / 布局布线了
```

退出码 0 = 通过；有失败返回 1，并直接打印 `[FAIL]` 行。

### 第 3 步（可选但推荐）：跑三套单元回归

板级自检只看"接线与流程"，下面几套才覆盖算法细节（合计 892 项）：

```powershell
cd D:\Self\Files\Basement\FPGA1\1\1\06_keyboard
powershell -NoProfile -ExecutionPolicy Bypass -File .\run_all_sim.ps1     # 468 项

cd D:\Self\Files\Basement\FPGA1\1\1\08_P1
powershell -NoProfile -ExecutionPolicy Bypass -File .\run_p1.ps1          # 268 项（含 tb_top_console）

cd D:\Self\Files\Basement\FPGA1\1\1\09_P3
powershell -NoProfile -ExecutionPolicy Bypass -File .\run_p3.ps1          # 9 项（tb + Python 判据）

cd D:\Self\Files\Basement\FPGA1\1\1\10_P2
powershell -NoProfile -ExecutionPolicy Bypass -File .\run_p2.ps1          # 147 项
```

每条最后一行都该是 `ALL PASS (...)`。**四条都绿再进 Gowin。**

### 第 4 步：进 Gowin 做 P&R

| 项 | 值 |
|---|---|
| Device | `GW5AST-138B` |
| pn | `GW5AST-LV138PG484AC1/I0` |
| 封装 / 版本 | **PBG484A** / Device Version **B** |
| 顶层 | **`top_console`** |
| 源文件 | `11_board\` 根目录的 **35 个 `.v`**（别选 docs_tests） |
| 约束 | `constraints\console_138k_top_console.cst` + `constraints\console_138k.sdc` |
| 时钟 | 50 MHz（V22），不生成 PLL 也能跑 |

综合后**必看三个数**：
1. **BSRAM ≈ 295/340**（32 声部 288 + 混音/混响 7）——远超就说明哪里被复制了；
2. 日志里搜 `Unable to open` —— 有的话说明波表没读到，**上板会全静音**；
3. 有没有 `IO_LOC ... not found` —— 说明 `.cst` 端口名与顶层不匹配（选错顶层了？）。

---

## 二、三个脚本各管什么

| 脚本 | 干什么 | 何时用 |
|---|---|---|
| `run_board_sim.ps1` | elaborate 三个顶层 + 跑 `tb_board`（26 项） | **每次投 P&R 之前** |
| `smoke_compile.ps1` | 只 elaborate（0.3 秒）：抓漏文件/重名/端口位宽 | 改了 RTL 想快速验证"还能搭起来" |
| `sync_from_sources.ps1` | 从 `06_keyboard`/`08_P1`/`09_P3`/`10_P2` 同步 RTL 与约束到上一级 | 在源目录改完 RTL、跑完回归之后 |

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\sync_from_sources.ps1          # 覆盖有差异的
powershell -NoProfile -ExecutionPolicy Bypass -File .\sync_from_sources.ps1 -Check   # 只报告漂移
```

`sync` 的白名单是 `_manifest.csv`（40 条），所以**不可能**把 `tb_*.v` / `*_model.v`
带进 Gowin 目录；它还会反查上一级有没有混进 `.md/.ps1/.csv/.vvp`。

---

## 三、`tb_board.v` 检查了什么（26 项）

| 组 | 检查 |
|---|---|
| 引脚极性 | `PA_EN=1`（MAX98357A 的 SD_MODE 高有效）、复位后音频为 0 |
| 板载演示键 | 按一下 → `demo_en=1 / demo_sel=1 / n_voices=1`（低有效 + 20 ms 去抖） |
| **真的出声** | 采 3 个周期：**60 Hz 周期 = 800 帧，实测 0 ppm 偏差**、无削顶 |
| 32 声部档 | 6 下后 32 路 `amp_shift` 全 5（1/32）、32 路 gate 全 1、强制 FM 态 / `mod_index=0` / 正弦页 |
| 退出演示 | 第 7 下 → `demo_en=0, demo_sel=0` |
| 滑音 | 合上自锁开关 → `glide_en=1`、槽 0 跟"有键按下"、槽 1..31 静音、退出后分配器复位 |
| 输出通路 | `HP_BCK` 有翻边、串口有字节发出 |

> 这是 `08_P1\tb_top_console.v` 的**同源副本**，只改了模块名与波表路径。
> 改检查项时**两边都要改**，否则两处会漂。

---

## 四、仿真抓不到、必须靠 Gowin / 上板的东西

1. **时序收敛**（295 块 BSRAM、48 个 `osc_dds`、32 个颤音乘法器）——没有时序报告就是没验证。
2. **BSRAM 是否真的被推断成 BSRAM**（而不是 LUT RAM）。
3. **`.cst` 的电平/极性是否与实物一致**（`F4` 是 1.5 V、`PA_EN` 高有效、LED 低有效）。
4. **MPR121 的实际接线与 I2C 地址**（引脚待确认，见 `缺什么.md` §2.1）。
5. **喇叭音色/音量**。

---

## 4.5 ⚠️ Gowin 里已经踩过的三个坑（2026-10-25 实测）

| 现象 | 根因 | 怎么修 |
|---|---|---|
| 综合日志 `NOTE (EX0101) : Current top module is "uart_rx"`，P&R 报告 `<Physical Constraints File>: ---`、`<Timing Constraints File>: ---`，资源只有 49 LUT | **工程没设顶层**，Gowin 自己挑了一个（挑到了 `uart_rx`）；约束也没加进工程 | `Project → Configuration → Synthesis` 的 **Top Module** 填 `top_console`（或在层级树里右键 `top_console` → Set as Top Module）；再用 `Add Files` 把 `.cst`/`.sdc` 加进来 |
| `WARN (TA1132) : 'clk' was determined to be a clock but was not created` | 有 CDC/时钟网络但**没有 `create_clock`**（本例是"顶层错 + 没加 SDC"的并发症） | 加 `constraints\console_138k.sdc`；确认顶层是 `top_console`（它的时钟端口叫 **`sys_clk`**，不是 `clk`） |
| `ERROR (PR2017) : 'LEDs[0]' ... dedicated pin (READY)` + `ERROR (PR2028)` | Console 那颗 RGB 的 **R=U12(~READY)、B=G11(~DONE) 是配置专用脚**，不能当普通 IO | 已改：RTL 的 LED 端口收成 1 位 `led`，只驱动 **V13(SYS_ACT)**。要用三色就在 `Bitstream → Dual-Purpose Pin` 里把 READY/DONE 勾成 regular IO，再把 RTL/`.cst` 改回 3 位 |

> 判据：只要 `rpt.txt` 里那两个 `<... Constraints File>` 还是 `---`，**这次 P&R 的结果就不可信**（引脚乱放、时序没分析）。
## 4.6 ⚠️ 时序：报告里必须看的三个数（2026-10-25 实际踩过）

| 看哪里 | 第一次 P&R（改前） | 含义 |
|---|---|---|
| `fpga_project_3_tr_content.html` → **Max Frequency Summary** | `Actual Fmax = 4.647 MHz`，**Logic Level = 237** | 只有 4.6 MHz，离 50 MHz 差 10 倍 ⇒ **烧上去必坏** |
| 同文件 → **Total Negative Slack Summary** | `Setup: TNS −453,374 ns / 2949 端点`，`Hold: 0` | 2949 个端点不满足；Hold 是好的 |
| `fpga_project_3.timing_paths` | 最差 slack **−195.2 ns**（到达 220.8 vs 要求 25.7），终点**全是** `u_alloc/tgt_2_*` | 病根在 `voice_alloc` 的目标槽搜索 |

**修法（已做）**：`voice_alloc` 的目标槽选择从"一次组合算完"改成**帧内扫描**
（每帧 tick 后用 32 个时钟逐槽比较，32 拍 ≪ 一帧 1042 拍），`freq_table` 的 84 级优先链改成并行 `case`。
**判断标准**：重跑后 `Logic Level` 应降到几十以内、`Fmax` 应 ≥ 50 MHz、Setup 的 `Endpoints` 应为 0。

> ✅ **2026-10-26 已收口**：Fmax **54.797 MHz**、Logic Level **9**、Setup **TNS 0.000 / 0 端点**、Hold **0**。
> 最差路径数据延迟 16.14 ns（周期 20 ns），余量 +1.75 ns。资源：BSRAM **301/340**、DSP 141/298、LUT 21.7K。
> ⚠️ 余量只有 1.75 ns，且各轮 P&R 会抖 ±1~2 MHz ⇒ 任何 RTL 改动后都要重新确认这三个数。

> ⚠️ `WARN (PR1014) : Generic routing resource will be used to clock signal 'sys_clk_d'`：
> `V22` 是 SOM 的 **EMCCLK（配置时钟专用脚）**，不是通用 GCLK 脚 ⇒ 时钟走普通布线、多出偏斜
> （所以 required 显示 25.7 ns 而不是 20）。彻底解决要**上 rPLL**（50→150 MHz，输出接全局时钟网络），
> 但先把逻辑深度修下来——PLL 不解决 237 级。

## 五、出问题怎么查

| 现象 | 先看哪里 |
|---|---|
| 第①步 FAIL | `_build\chk_<顶层>.log`：多半是漏 `.v`、重名、或端口位宽不对 |
| 第②步 `ok=` 少于 26 | `_build\tb_board.log` 里的 `[FAIL]` 行（检查名是中文，能直接读） |
| 卡住不返回 | 脚本有 600 秒看门狗，会打印"看门狗超时"（多半时钟/复位没接对） |
| `sync -Check` 报漂移 | 源目录改过但没同步；跑一次不带 `-Check` 的同步 |
| 综合后 BSRAM 远超 295 | 是不是把 `docs_tests` 或同一模块的两份拷贝加进去了 |