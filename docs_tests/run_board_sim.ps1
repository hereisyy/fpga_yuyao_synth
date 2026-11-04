# ============================================================
#  run_board_sim.ps1 —— ★ 上板前的一键自检（跑的就是交给 Gowin 的那套文件）
#
#  位置：`11_board\docs_tests\`（RTL 在上一级 `11_board\`）
#  用法：双击不行，请在 PowerShell 里：
#      cd D:\Self\Files\Basement\FPGA1\1\1\11_board\docs_tests
#      powershell -NoProfile -ExecutionPolicy Bypass -File .\run_board_sim.ps1
#
#  它分两步，缺一不可：
#    ① **elaborate**（编译）：把上一级 35 个 .v 分别以 `top_console` /
#       `p2_smoke_top` / `top_synth` 为顶层各编一次。
#       抓的是：漏文件、模块重名、端口位宽对不上、顶层名写错、.hex 没放好。
#    ② **tb_board**（仿真）：以 `top_console` 为 DUT 跑板级行为自检
#       （PA_EN 极性、演示键、60 Hz 出声、32 声部 1/N、滑音、I2S、串口）。
#
#  ⚠️ 这一步抓不到的东西（必须靠 Gowin 或上板）：
#       时序收敛、BSRAM 拼块、IO 电平/极性是否与实物一致、喇叭音色。
#
#  ⚠️ 本文件必须存成「UTF-8 带 BOM + CRLF」。
#  退出码：0 = 全通过，1 = 有失败。
# ============================================================

$ErrorActionPreference = 'Stop'
$simdir = (Get-Location).Path
$rtl    = (Resolve-Path (Join-Path $simdir '..')).Path
$build  = Join-Path $simdir '_build'
New-Item -ItemType Directory -Force -Path $build | Out-Null
$env:PYTHONIOENCODING = 'utf-8'          # 本脚本不跑 Python，但同目录脚本会用

# ---------- 找 iverilog / vvp ----------
function Find-Tool($name, $fallback) {
    $c = Get-Command $name -ErrorAction SilentlyContinue
    if ($c) { return $c.Source }
    if (Test-Path $fallback) { return $fallback }
    return $null
}
$iverilog = Find-Tool 'iverilog' 'D:\Self\Softwares\iverilog\bin\iverilog.exe'
$vvp      = Find-Tool 'vvp'      'D:\Self\Softwares\iverilog\bin\vvp.exe'
if (-not $iverilog -or -not $vvp) {
    Write-Host '  !! 找不到 iverilog / vvp。'
    Write-Host '     装好 Icarus Verilog 后，把它的 bin 目录加进 PATH，或改本脚本里的 $fallback 路径。'
    exit 1
}

$vfiles = @(Get-ChildItem $rtl -File -Filter *.v | Select-Object -ExpandProperty Name | Sort-Object)
$hexOk  = @('wavetable_all.hex','sine_table.hex') | Where-Object { Test-Path (Join-Path $rtl $_) }

Write-Host ''
Write-Host '========== 上板前板级自检 =========='
Write-Host ('  RTL 目录 : {0}' -f $rtl)
Write-Host ('  RTL 文件 : {0} 个 .v' -f $vfiles.Count)
Write-Host ('  数据文件 : {0}' -f (($hexOk | ForEach-Object { $_ }) -join ', '))
if ($hexOk.Count -lt 2) { Write-Host '  !! 数据文件不全（.hex 必须在 RTL 目录里，否则综合会静默填 0）' }
Write-Host ''

$fail = 0

# ============================================================
#  第 ① 步：elaborate（三个顶层各编一次）
# ============================================================
Write-Host '---------- ① 编译（elaborate）----------'
Push-Location $rtl
try {
    foreach ($t in @('top_console','p2_smoke_top','top_synth')) {
        $o = Join-Path $build ("chk_" + $t + ".vvp")
        $lg = Join-Path $build ("chk_" + $t + ".log")
        $sw = [Diagnostics.Stopwatch]::StartNew()
        & $iverilog -g2012 -s $t -o $o @vfiles 2> $lg
        $rc = $LASTEXITCODE
        $sw.Stop()
        if ($rc -eq 0) {
            Write-Host ('  {0,-16} OK    {1,5:N1}s' -f $t, $sw.Elapsed.TotalSeconds)
        } else {
            Write-Host ('  {0,-16} FAIL  {1,5:N1}s   -> {2}' -f $t, $sw.Elapsed.TotalSeconds, $lg)
            if (Test-Path $lg) { (Get-Content $lg -Encoding UTF8 | Select-Object -First 15) | ForEach-Object { Write-Host ('      ' + $_) } }
            $fail++
        }
    }
} finally { Pop-Location }

# ============================================================
#  第 ② 步：tb_board（板级行为仿真）
# ============================================================
Write-Host ''
Write-Host '---------- ② 仿真（tb_board）----------'
$tbVvp = Join-Path $build 'tb_board.vvp'
$tbLog = Join-Path $build 'tb_board.log'
$tbCmp = Join-Path $build 'tb_board.compile.log'

Push-Location $rtl
try {
    $srcList = @($vfiles)
    $cmd = 'iverilog -g2012 -o "' + $tbVvp + '" -s tb_board ' + (Join-Path $simdir 'tb_board.v') + ' ' + ($srcList -join ' ')
    cmd /c ('cd /d "' + $rtl + '" && ' + $cmd + ' > "' + $tbCmp + '" 2>&1')
    $rc = $LASTEXITCODE
} finally { Pop-Location }

if ($rc -ne 0) {
    Write-Host ('  tb_board      COMPILE FAILED  -> {0}' -f $tbCmp)
    if (Test-Path $tbCmp) { (Get-Content $tbCmp -Encoding UTF8 | Select-Object -First 20) | ForEach-Object { Write-Host ('      ' + $_) } }
    $fail++
} else {
    # 仿真必须在本目录跑：tb 里 INIT_FILE = "../wavetable_all.hex"
    $sw = [Diagnostics.Stopwatch]::StartNew()
    $psi = New-Object System.Diagnostics.ProcessStartInfo
    $psi.FileName               = $vvp
    $psi.Arguments              = '-l "' + $tbLog + '" "' + $tbVvp + '"'
    $psi.WorkingDirectory       = $simdir
    $psi.UseShellExecute        = $false
    $psi.CreateNoWindow         = $true
    $psi.RedirectStandardOutput = $true
    $psi.RedirectStandardError  = $true
    $proc = [System.Diagnostics.Process]::Start($psi)
    $null = $proc.StandardOutput.ReadToEndAsync()
    $timedOut = $false
    if (-not $proc.WaitForExit(600 * 1000)) { $timedOut = $true; $proc.Kill(); $proc.WaitForExit(3000) | Out-Null }
    try {
        $e = $proc.StandardError.ReadToEnd()
        if ($e -and $e.Trim().Length -gt 0) {
            [System.IO.File]::AppendAllText($tbLog, "`r`n[stderr]`r`n" + $e, (New-Object System.Text.UTF8Encoding($false)))
        }
    } catch { }
    $sw.Stop()

    $txt = ''
    if (Test-Path $tbLog) { $txt = Get-Content $tbLog -Encoding UTF8 -Raw }
    $nOk   = ([regex]::Matches($txt, '\[ ok \]')).Count
    $nFail = ([regex]::Matches($txt, '\[FAIL\]')).Count
    $stat  = 'PASS'
    if ($timedOut) { $stat = 'FAIL'; $why = '看门狗超时' }
    elseif ($nFail -gt 0) { $stat = 'FAIL'; $why = ('fail=' + $nFail) }
    elseif ($nOk -lt 26) { $stat = 'FAIL'; $why = ('只跑了 ' + $nOk + ' 项，应 >= 26') }
    else { $why = '' }
    if ($stat -eq 'FAIL') { $fail++ }
    Write-Host ('  {0,-16} {1}  ok={2}  fail={3}  {4,5:N1}s  {5}' -f 'tb_board', $stat, $nOk, $nFail, $sw.Elapsed.TotalSeconds, $why)
    Write-Host ('      日志：{0}' -f $tbLog)
    if ($stat -eq 'FAIL' -and (Test-Path $tbLog)) {
        (Get-Content $tbLog -Encoding UTF8 | Select-String -Pattern '\[FAIL\]' | Select-Object -First 10) | ForEach-Object { Write-Host ('      ' + $_.Line) }
    }
}

# ============================================================
Write-Host ''
Write-Host '===================================='
if ($fail -eq 0) {
    Write-Host '  ✅ 全通过：这套 RTL 可以在 Gowin 里做综合 / 布局布线了'
    Write-Host ''
    Write-Host '  提醒（这三条仿真验证不了）：'
    Write-Host '    · 器件选 GW5AST-138B / PBG484A / Device Version B，顶层 top_console'
    Write-Host '    · 约束用 constraints\console_138k_top_console.cst + console_138k.sdc'
    Write-Host '    · 综合后看 BSRAM 是否 ≈295/340、有没有 Unable to open（波表没读到就全静音）'
    exit 0
} else {
    Write-Host ('  ❌ 有 {0} 处失败，先别综合：按上面的日志逐个修' -f $fail)
    exit 1
}
Write-Host ''