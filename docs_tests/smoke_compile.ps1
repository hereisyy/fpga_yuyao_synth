# ============================================================
#  smoke_compile.ps1 —— 进 Gowin 之前，先用 iverilog elaborate 一遍
#
#  位置：`11_board\docs_tests\`（脚本体在测试目录里，**RTL 在上一级**）
#  用法：在本目录下
#      powershell -NoProfile -ExecutionPolicy Bypass -File .\smoke_compile.ps1
#      powershell -NoProfile -ExecutionPolicy Bypass -File .\smoke_compile.ps1 -Top all
#
#  它能抓的（都不需要 Gowin）：
#    · 漏拷/删错某个 .v（unknown module）
#    · 两个文件里模块重名（redefinition）
#    · 端口名/位宽对不上（port mismatch）
#    · 顶层名写错（-s 指定的模块不存在）
#    · 数据文件（.hex）不在 RTL 目录里
#  它抓不到：时序、BSRAM 拼块、IO 电平 —— 那些只能在 Gowin 里看。
#
#  ⚠️ 本文件必须存成「UTF-8 带 BOM + CRLF」。
# ============================================================
param(
    [string]$Top = 'top_console'
)

$ErrorActionPreference = 'Stop'
$simdir = (Get-Location).Path                            # 11_board\docs_tests
$rtl    = (Resolve-Path (Join-Path $simdir '..')).Path   # 11_board
$build  = Join-Path $simdir '_build'
New-Item -ItemType Directory -Force -Path $build | Out-Null

$iverilog = $null
$cmd = Get-Command iverilog -ErrorAction SilentlyContinue
if ($cmd) { $iverilog = $cmd.Source }
if (-not $iverilog) {
    $guess = 'D:\Self\Softwares\iverilog\bin\iverilog.exe'
    if (Test-Path $guess) { $iverilog = $guess }
}
if (-not $iverilog) {
    Write-Host '  !! 找不到 iverilog（PATH 里没有，D:\Self\Softwares\iverilog\bin 也没有）'
    exit 1
}

$vfiles = @(Get-ChildItem $rtl -File -Filter *.v | Select-Object -ExpandProperty Name)
if ($vfiles.Count -eq 0) {
    Write-Host ('  !! RTL 目录里没有 .v 文件：' + $rtl)
    exit 1
}

if ($Top -eq 'all') {
    $tops = @('top_console', 'p2_smoke_top', 'top_synth')
} else {
    $tops = @($Top)
}

# 数据文件：$readmemh 找不到时**不报错、静默填 0**（上板全静音）
$needData = @('sine_table.hex', 'wavetable_all.hex')
$missData = @($needData | Where-Object { -not (Test-Path (Join-Path $rtl $_)) })

Write-Host ''
Write-Host ('==== smoke compile：{0} 个 .v（在 {1}），顶层 {2} ====' -f $vfiles.Count, $rtl, ($tops -join ', '))
if ($missData.Count -gt 0) {
    Write-Host ('  !! 缺数据文件：{0}（综合后会静默填 0）' -f ($missData -join ', '))
}

$failed = 0
Push-Location $rtl            # 让 iverilog 的相对路径落在 RTL 目录
try {
    foreach ($t in $tops) {
        $out = Join-Path $build ($t + '.vvp')
        $log = Join-Path $build ($t + '.compile.log')
        $sw  = [Diagnostics.Stopwatch]::StartNew()
        & $iverilog -g2012 -s $t -o $out @vfiles 2> $log
        $rc = $LASTEXITCODE
        $sw.Stop()
        $err = ''
        if (Test-Path $log) { $err = (Get-Content $log -Raw) }
        if ($rc -eq 0) {
            Write-Host ('  {0,-16} OK    {1,6:N1}s' -f $t, $sw.Elapsed.TotalSeconds)
        } else {
            Write-Host ('  {0,-16} FAIL  {1,6:N1}s   {2}' -f $t, $sw.Elapsed.TotalSeconds, $log)
            if ($err) { ($err -split "`r?`n" | Select-Object -First 12) | ForEach-Object { Write-Host ('      ' + $_) } }
            $failed++
        }
    }
} finally {
    Pop-Location
}

Write-Host '--------------------------------------------------------------------'
if ($failed -eq 0 -and $missData.Count -eq 0) {
    Write-Host '  ALL OK'
    Write-Host ''
    exit 0
} else {
    Write-Host ('  {0} 个顶层失败，{1} 个数据文件缺失' -f $failed, $missData.Count)
    Write-Host ''
    exit 1
}