param([string]$KeilPath = 'E:\Embedded_Development\KeilMDK\UV4\UV4.exe')
$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot
$originalProject = Join-Path $repoRoot 'MDK-ARM/ALL_terain_For_11.uvprojx'
$projectDir = Split-Path -Parent $originalProject
$buildDir = Join-Path ([System.IO.Path]::GetTempPath()) ('oled-keil-' + [guid]::NewGuid().ToString('N'))
$outputDir = Join-Path $buildDir 'output'
$listingDir = Join-Path $buildDir 'listing'
[System.IO.Directory]::CreateDirectory($outputDir) | Out-Null
[System.IO.Directory]::CreateDirectory($listingDir) | Out-Null
[xml]$projectXml = Get-Content -LiteralPath $originalProject -Raw
# 短相对路径避免旧版 uVision 的 IncludePath 长度限制。
# 临时目录的 src 联接只用于读取源码；所有构建输出显式指向独立 output/listing。
New-Item -ItemType Junction -Path (Join-Path $buildDir 'src') -Target $repoRoot | Out-Null
$projectText = [System.IO.File]::ReadAllText($originalProject)
function Source-Path([string]$Path) {
    $absolute = [System.IO.Path]::GetFullPath((Join-Path $projectDir $Path))
    return 'src\' + [System.IO.Path]::GetRelativePath($repoRoot, $absolute)
}
$projectText = [regex]::Replace($projectText, '<FilePath>([^<]*)</FilePath>', {
    param($match)
    '<FilePath>' + (Source-Path $match.Groups[1].Value) + '</FilePath>'
})
$projectText = [regex]::Replace($projectText, '<IncludePath>([^<]*)</IncludePath>', {
    param($match)
    $entries = @()
    foreach ($path in $match.Groups[1].Value.Split(';')) {
        if ($path.Trim()) { $entries += Source-Path $path }
    }
    '<IncludePath>' + ($entries -join ';') + '</IncludePath>'
})
$projectText = [regex]::Replace($projectText, '<OutputDirectory>[^<]*</OutputDirectory>', '<OutputDirectory>output\</OutputDirectory>')
$projectText = [regex]::Replace($projectText, '<ListingPath>[^<]*</ListingPath>', '<ListingPath>listing\</ListingPath>')
$temporaryProject = Join-Path $buildDir 'OLED_validation.uvprojx'
[System.IO.File]::WriteAllText($temporaryProject, $projectText, [System.Text.UTF8Encoding]::new($false))
$logPath = Join-Path $buildDir 'build.log'
$targetName = $projectXml.Project.Targets.Target.TargetName
$arguments = @('-r', ('"' + $temporaryProject + '"'), '-j0', '-t', ('"' + $targetName + '"'), '-o', ('"' + $logPath + '"'))
Write-Host "Keil validation directory: $buildDir"
$process = Start-Process -FilePath $KeilPath -ArgumentList $arguments -WorkingDirectory $buildDir -WindowStyle Hidden -PassThru
# 最多等待一分钟，不删除任何产物；用户可在日志中查看失败现场。
if (-not $process.WaitForExit(60000)) { throw "Keil build still running; inspect $logPath" }
if (-not (Test-Path -LiteralPath $logPath)) { throw "Keil did not create log (exit $($process.ExitCode))" }
$logText = [System.IO.File]::ReadAllText($logPath)
Write-Host $logText
if ($process.ExitCode -gt 1 -or $logText -notmatch '0 Error\(s\)') { throw 'Keil rebuild failed' }
Write-Host 'Keil rebuild completed; workspace output files were not replaced.'
