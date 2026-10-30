param(
    [string]$Gcc = 'D:/CoderSpcace/IDE/mingw64/bin/gcc.exe',
    [string]$BuildDirectory = (Join-Path $env:TEMP 'all-terrain-sr04-tests')
)

$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path

if (-not (Test-Path -LiteralPath $Gcc)) {
    $Gcc = (Get-Command gcc -ErrorAction Stop).Source
}

# 编译真实驱动，GPIO、DWT、时钟和延时由宿主替身提供；产物仅写临时目录。
New-Item -ItemType Directory -Path $BuildDirectory -Force | Out-Null
$outputPath = Join-Path $BuildDirectory 'test_sr04.exe'
$arguments = @(
    '-std=c99', '-Wall', '-Wextra', '-Werror', '-pedantic', '-O2',
    '-I', (Join-Path $PSScriptRoot 'fake_hal'),
    '-I', (Join-Path $projectRoot 'Int/Int_SR04'),
    '-I', (Join-Path $projectRoot 'Com/Com_Delay_us'),
    (Join-Path $PSScriptRoot 'test_sr04.c'),
    (Join-Path $projectRoot 'Int/Int_SR04/Int_SR04.c'),
    '-lm', '-o', $outputPath
)
& $Gcc @arguments
if ($LASTEXITCODE -ne 0) { throw 'Compilation failed: test_sr04' }
& $outputPath
if ($LASTEXITCODE -ne 0) { throw 'Test failed: test_sr04' }
Write-Output "Build artifacts: $BuildDirectory"
