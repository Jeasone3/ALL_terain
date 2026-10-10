param(
    [string]$Gcc = 'D:/CoderSpcace/IDE/mingw64/bin/gcc.exe',
    [string]$BuildDirectory = (Join-Path $env:TEMP 'all-terrain-oled-tests')
)

$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$oledDirectory = Join-Path $projectRoot 'Int/Int_OLED'
$fakeHalDirectory = Join-Path $PSScriptRoot 'fake_hal'

if (-not (Test-Path -LiteralPath $Gcc)) {
    $Gcc = (Get-Command gcc -ErrorAction Stop).Source
}

# 编译产物保存在临时目录，不向工程目录写入可执行文件，不自动删除任何文件。
New-Item -ItemType Directory -Path $BuildDirectory -Force | Out-Null
$commonArguments = @('-std=c99', '-Wall', '-Wextra', '-Werror', '-pedantic', '-O2', '-I', $oledDirectory)

function Invoke-OledTest {
    param([string]$Name, [string[]]$Arguments)
    $outputPath = Join-Path $BuildDirectory ($Name + '.exe')
    & $Gcc @commonArguments @Arguments '-o' $outputPath
    if ($LASTEXITCODE -ne 0) { throw "Compilation failed: $Name" }
    & $outputPath
    if ($LASTEXITCODE -ne 0) { throw "Test failed: $Name" }
}

Invoke-OledTest -Name 'test_oled_core' -Arguments @(
    (Join-Path $PSScriptRoot 'test_oled_core.c'),
    (Join-Path $oledDirectory 'Int_OLED.c'),
    (Join-Path $oledDirectory 'oledfont.c')
)

$portArguments = @(
    '-I', $fakeHalDirectory,
    (Join-Path $PSScriptRoot 'test_oled_port.c'),
    (Join-Path $oledDirectory 'OLED_Port.c')
)
Invoke-OledTest -Name 'test_oled_port' -Arguments $portArguments
Invoke-OledTest -Name 'test_oled_port_config' -Arguments (@('-DOLED_I2C_ADDRESS=0x40U', '-DOLED_I2C_TIMEOUT_MS=7U') + $portArguments)
Invoke-OledTest -Name 'test_oled_port_invalid_address' -Arguments (@('-DOLED_I2C_ADDRESS=0x80U') + $portArguments)
