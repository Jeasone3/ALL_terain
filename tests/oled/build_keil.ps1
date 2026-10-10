param(
    [string]$Uv4 = 'E:/Embedded_Development/KeilMDK/UV4/UV4.exe'
)

$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$mdkDirectory = Join-Path $projectRoot 'MDK-ARM'
$sourceProject = Join-Path $mdkDirectory 'ALL_terain_For_11.uvprojx'
$validationDirectory = Join-Path $mdkDirectory '.vscode/oled-validation'
$outputDirectory = Join-Path $validationDirectory 'build'
$validationProject = Join-Path $validationDirectory ([System.IO.Path]::GetFileName($sourceProject))
$logPath = Join-Path $validationDirectory 'build.log'

if (-not (Test-Path -LiteralPath $Uv4)) {
    throw "Keil executable was not found: $Uv4"
}
New-Item -ItemType Directory -Path $outputDirectory -Force | Out-Null
$project = [System.Xml.XmlDocument]::new()
$project.PreserveWhitespace = $true
$project.Load($sourceProject)

# 验证工程使用当前源码和绝对路径，产物放入已忽略目录，避免覆盖已跟踪产物。
foreach ($node in $project.SelectNodes('//FilePath | //IncludePath')) {
    if ([string]::IsNullOrWhiteSpace($node.InnerText)) { continue }
    $absolutePaths = foreach ($relativePath in $node.InnerText.Split(';')) {
        if ([string]::IsNullOrWhiteSpace($relativePath)) { continue }
        if ([System.IO.Path]::IsPathRooted($relativePath)) {
            $relativePath
        } else {
            [System.IO.Path]::GetFullPath((Join-Path $mdkDirectory $relativePath))
        }
    }
    $node.InnerText = $absolutePaths -join ';'
}
foreach ($node in $project.SelectNodes('//OutputDirectory | //ListingPath')) {
    $node.InnerText = $outputDirectory + [System.IO.Path]::DirectorySeparatorChar
}
[System.IO.File]::WriteAllText($validationProject, $project.OuterXml,
                             [System.Text.UTF8Encoding]::new($false))
$targetName = $project.Project.Targets.Target.TargetName

# 只构建，不下载固件；隐藏验证进程，不打断用户的交互窗口。
$arguments = @('-b', ('"' + $validationProject + '"'), '-j0', '-sg', '-t',
               ('"' + $targetName + '"'), '-o', ('"' + $logPath + '"'))
$process = Start-Process -FilePath $Uv4 -ArgumentList $arguments -WindowStyle Hidden -PassThru
$process.WaitForExit()
if (Test-Path -LiteralPath $logPath) {
    Get-Content -LiteralPath $logPath
}
if ($process.ExitCode -ne 0) {
    throw "Keil build failed with exit code $($process.ExitCode); see $logPath"
}
Write-Output "Keil validation output: $outputDirectory"
