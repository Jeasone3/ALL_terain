param(
    [string]$Keil = 'E:/Embedded_Development/KeilMDK/UV4/UV4.exe',
    [string]$BuildDirectory = (Join-Path $env:TEMP ('all-terrain-motion-keil-' + [guid]::NewGuid().ToString('N')))
)

$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$projectDirectory = Join-Path $projectRoot 'MDK-ARM'
$projectPath = Join-Path $projectDirectory 'ALL_terain_For_11.uvprojx'

if (-not (Test-Path -LiteralPath $Keil)) {
    throw "Keil UV4 not found: $Keil"
}
New-Item -ItemType Directory -Path $BuildDirectory -Force | Out-Null
$BuildDirectory = (Resolve-Path -LiteralPath $BuildDirectory).Path

# 复制工程到临时目录，并把源文件与头文件路径改为绝对路径。
# 编译直接读取工作区源码，输出和 RTE 缓存写入临时目录，不覆盖 Git 中的旧构建产物。
[xml]$project = Get-Content -LiteralPath $projectPath -Raw
foreach ($node in $project.SelectNodes('//FilePath')) {
    if ($node.InnerText -and -not [IO.Path]::IsPathRooted($node.InnerText)) {
        $node.InnerText = [IO.Path]::GetFullPath((Join-Path $projectDirectory $node.InnerText))
    }
}
foreach ($node in $project.SelectNodes('//IncludePath')) {
    if (-not $node.InnerText) { continue }
    $paths = foreach ($path in $node.InnerText.Split(';')) {
        if ([IO.Path]::IsPathRooted($path)) {
            $path
        } else {
            [IO.Path]::GetFullPath((Join-Path $projectDirectory $path))
        }
    }
    $node.InnerText = $paths -join ';'
}
foreach ($node in $project.SelectNodes('//OutputDirectory | //ListingPath')) {
    $node.InnerText = $BuildDirectory + '\'
}

$validationProject = Join-Path $BuildDirectory 'ALL_terain_For_11.uvprojx'
$logPath = Join-Path $BuildDirectory 'keil-build.log'
# 当前 uVision 对带 UTF-8 BOM 的临时 XML 返回读取错误；保持原工程的无 BOM 编码。
[IO.File]::WriteAllText($validationProject, $project.OuterXml, [Text.UTF8Encoding]::new($false))
$arguments = @('-r', ('"' + $validationProject + '"'), '-j0', '-t',
               'ALL_terain_For_11', '-o', ('"' + $logPath + '"'))

# 使用隐藏的批处理窗口执行完整重编译，不打开交互式 IDE，不烧录硬件。
$process = Start-Process -FilePath $Keil -ArgumentList $arguments -WindowStyle Hidden -PassThru
while (-not $process.WaitForExit(1000)) {
    # 等待由命令工具异步承接，调用者可以继续处理其他验证工作。
}
$process.WaitForExit()
$process.Refresh()
if (-not (Test-Path -LiteralPath $logPath)) {
    throw "Keil exited with code $($process.ExitCode) without a build log: $logPath"
}
$log = Get-Content -LiteralPath $logPath -Raw
Write-Output $log
Write-Output "Build artifacts: $BuildDirectory"
if ($process.ExitCode -ne 0 -or $log -notmatch '0 Error\(s\), 0 Warning\(s\)') {
    throw "Keil build failed or reported diagnostics: $logPath"
}
