# ============================================================
#  sync_from_sources.ps1 —— 用源目录刷新 `11_board`（RTL 与约束）
#
#  位置：`11_board\docs_tests\`（脚本体在测试目录里，**目标目录是上一级** 11_board）
#  用法：在本目录下
#      powershell -NoProfile -ExecutionPolicy Bypass -File .\sync_from_sources.ps1
#      powershell -NoProfile -ExecutionPolicy Bypass -File .\sync_from_sources.ps1 -Check
#
#  行为：
#    · 只处理 `_manifest.csv` 里登记过的文件 —— 清单是白名单，
#      所以**不可能**把 tb_*.v / *_model.v / 分表 hex 带进 Gowin 目录。
#    · 内容一致就跳过；不一致才覆盖，并打印 [更新]。
#    · -Check 只报告漂移（退出码 1），不写盘。
#    · 源文件缺失时报错退出，不静默跳过。
#
#  ⚠️ 反方向（在 11_board 里改了、要写回源目录）**故意没有自动脚本**：
#     这类双向同步一旦搞错，坏的是回归基线。请改源目录 → 跑回归 → 再跑本脚本。
#
#  ⚠️ 本文件必须存成「UTF-8 带 BOM + CRLF」。
# ============================================================
param(
    [switch]$Check
)

$ErrorActionPreference = 'Stop'
$simdir = (Get-Location).Path                             # 11_board\docs_tests
$dst    = (Resolve-Path (Join-Path $simdir '..')).Path    # 11_board
$root   = (Resolve-Path (Join-Path $simdir '..\..')).Path # 1\1（含 06_keyboard/08_P1/09_P3/10_P2）
$csv    = Join-Path $simdir '_manifest.csv'

if (-not (Test-Path $csv)) {
    Write-Host ('  !! 找不到清单：' + $csv)
    exit 1
}

$rows = @(Import-Csv $csv -Encoding UTF8)
if ($rows.Count -eq 0) {
    Write-Host '  !! 清单是空的'
    exit 1
}

Write-Host ''
Write-Host ('==== 11_board 同步：{0} 个登记文件 ====' -f $rows.Count)
Write-Host ('     源根目录 {0}' -f $root)
Write-Host ('     目标目录 {0}' -f $dst)

$nSame = 0; $nUpd = 0; $nSrcMiss = 0

foreach ($r in $rows) {
    $rel  = $r.'文件'                       # 例：voice_array.v / constraints\console_138k_top_console.cst
    $srcD = $r.'来源'                       # 例：08_P1
    $leaf = Split-Path -Leaf $rel
    $src  = Join-Path (Join-Path $root $srcD) $leaf
    $to   = Join-Path $dst $rel

    if (-not (Test-Path $src)) {
        Write-Host ('  [源缺失] {0,-40} 期望在 {1}\{2}' -f $rel, $srcD, $leaf)
        $nSrcMiss++
        continue
    }

    $hSrc = (Get-FileHash $src -Algorithm SHA256).Hash
    $hDst = ''
    if (Test-Path $to) { $hDst = (Get-FileHash $to -Algorithm SHA256).Hash }

    if ($hSrc -eq $hDst) { $nSame++; continue }

    if ($hDst -eq '') { Write-Host ('  [缺失] {0,-40} <- {1}\{2}' -f $rel, $srcD, $leaf) }
    else              { Write-Host ('  [更新] {0,-40} <- {1}\{2}' -f $rel, $srcD, $leaf) }
    $nUpd++

    if (-not $Check) {
        $dir = Split-Path -Parent $to
        if ($dir -and -not (Test-Path $dir)) { New-Item -ItemType Directory -Force -Path $dir | Out-Null }
        Copy-Item $src $to -Force
    }
}

# ---- 自检：Gowin 目录里不许出现不可综合/非 RTL 的文件 ----
#      （docs_tests 与 _build 是我们自己的东西，跳过）
$bad = Get-ChildItem $dst -File -Recurse |
       Where-Object { $_.FullName -notmatch '\\docs_tests\\' -and $_.FullName -notmatch '\\_build\\' } |
       Where-Object { $_.Name -like 'tb_*.v' -or $_.Name -like '*_model.v' -or
                      $_.Extension -in @('.vvp', '.md', '.ps1', '.csv', '.log') }
if ($bad) {
    Write-Host ''
    Write-Host '  !! Gowin 目录里出现了不该有的文件（请挪进 docs_tests）：'
    $bad | ForEach-Object { Write-Host ('     ' + $_.FullName.Substring($dst.Length + 1)) }
}

Write-Host '--------------------------------------------------------------------'
if ($Check) {
    Write-Host ('  一致 {0} 个，漂移 {1} 个（-Check 未写盘），源缺失 {2} 个' -f $nSame, $nUpd, $nSrcMiss)
} else {
    Write-Host ('  一致 {0} 个，已更新 {1} 个，源缺失 {2} 个' -f $nSame, $nUpd, $nSrcMiss)
}
$badCount = @($bad).Count
if ($badCount -gt 0) { Write-Host ('  !! 另有 {0} 个不该存在的文件' -f $badCount) }
Write-Host ''

if ($nSrcMiss -gt 0) { exit 1 }
if ($Check -and $nUpd -gt 0) { exit 1 }
if ($badCount -gt 0) { exit 1 }
exit 0