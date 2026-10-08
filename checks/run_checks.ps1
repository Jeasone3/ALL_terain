param(
    [string]$CompilerPath = 'D:\CoderSpcace\IDE\mingw64\bin\gcc.exe',
    [string]$ArmCompilerPath = 'E:\Embedded_Development\STM32CubeCLT_1.18.0\GNU-tools-for-STM32\bin\arm-none-eabi-gcc.exe'
)
$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot
$buildDir = Join-Path ([System.IO.Path]::GetTempPath()) ('oled-checks-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $buildDir | Out-Null
Write-Host "Check outputs: $buildDir"

function Compile-And-Run([string]$Name, [string[]]$Sources, [string[]]$Includes, [string[]]$Extra) {
    $binary = Join-Path $buildDir ($Name + '.exe')
    $arguments = @('-std=c99', '-O2', '-Wall', '-Wextra', '-Werror', '-pedantic')
    foreach ($directory in $Includes) { $arguments += @('-I', $directory) }
    $arguments += $Extra
    $arguments += $Sources
    $arguments += @('-lm', '-o', $binary)
    & $CompilerPath @arguments
    if ($LASTEXITCODE -ne 0) { throw "Compile failed: $Name" }
    & $binary
    if ($LASTEXITCODE -ne 0) { throw "Check failed: $Name" }
}

$core = Join-Path $repoRoot 'Com/Com_OLED'
$coreSources = @((Join-Path $core 'oled.c'), (Join-Path $core 'oled_font.c'))
Compile-And-Run 'oled' ($coreSources + (Join-Path $PSScriptRoot 'test_oled.c')) @($core) @()
Compile-And-Run 'oled-debug' ($coreSources + @(
    (Join-Path $repoRoot 'App/OLED_Debug/OLED_Debug.c'), (Join-Path $PSScriptRoot 'test_oled_debug.c')
)) @((Join-Path $PSScriptRoot 'stubs'), $core, (Join-Path $repoRoot 'App/OLED_Debug'),
    (Join-Path $repoRoot 'App/Mode_FSM')) @()

# 原 tests 被用户删除：只把 Git 中旧回归样例放入临时目录，不恢复工作区文件。
$oldTests = Join-Path $buildDir 'regression'
$oldFiles = @(git -C $repoRoot ls-tree -r --name-only HEAD tests)
if ($LASTEXITCODE -eq 0 -and $oldFiles.Count -gt 0) {
foreach ($file in $oldFiles) {
    $relative = $file.Substring(6)
    $target = Join-Path $oldTests $relative
    [System.IO.Directory]::CreateDirectory((Split-Path -Parent $target)) | Out-Null
    $content = (git -C $repoRoot show "HEAD:$file") -join "`n"
    if ($LASTEXITCODE -ne 0) { throw "Cannot read $file from HEAD" }
    [System.IO.File]::WriteAllText($target, $content + "`n", [System.Text.UTF8Encoding]::new($false))
}

# 新的观测只读取 PWM 和 DWT；给旧硬件替身补充这些字段，不修改控制断言。
$stubMain = Join-Path $oldTests 'stubs/main.h'
$stubText = [System.IO.File]::ReadAllText($stubMain)
$timingDeclarations = @'
typedef struct { uint32_t CYCCNT; } TestDWT;
extern TestDWT test_dwt;
#define DWT (&test_dwt)
extern uint32_t SystemCoreClock;
extern uint32_t test_control_cycles;
'@
$stubText = $stubText.Replace('#endif', $timingDeclarations + "`n#endif")
[System.IO.File]::WriteAllText($stubMain, $stubText, [System.Text.UTF8Encoding]::new($false))
$stubMotor = Join-Path $oldTests 'stubs/motor.h'
$motorText = [System.IO.File]::ReadAllText($stubMotor).Replace('uint8_t side;', 'uint8_t side; int16_t speed;')
[System.IO.File]::WriteAllText($stubMotor, $motorText, [System.Text.UTF8Encoding]::new($false))
foreach ($name in @('test_mode_fsm.c', 'test_imu_task.c')) {
    $path = Join-Path $oldTests $name
    $text = [System.IO.File]::ReadAllText($path).Replace('motorLeft = {0}', 'motorLeft = {0, 0}').Replace('motorRight = {1}', 'motorRight = {1, 0}')
    $text = $text.Replace("void Motor_SetSpeed(Motor_Struct *motor, int16_t speed)`n{", "void Motor_SetSpeed(Motor_Struct *motor, int16_t speed)`n{`n    motor->speed = speed;")
    if ($name -eq 'test_mode_fsm.c') {
        $text = $text.Replace("void Read_All_Track(uint16_t *sensor_values)`n{", "void Read_All_Track(uint16_t *sensor_values)`n{`n    test_dwt.CYCCNT += test_control_cycles;")
        $extraCases = [System.IO.File]::ReadAllText((Join-Path $PSScriptRoot 'test_mode_diagnostics.inc'))
        $text = $text.Replace('int main(void)', $extraCases + "`nint main(void)")
        $text = $text.Replace('const Test_Case cases[] = {', "const Test_Case cases[] = {`n        {`"diagnostic_freshness_and_evidence`", test_diagnostic_freshness_and_evidence},`n        {`"control_timing_callback`", test_control_timing_callback},")
    }
    [System.IO.File]::WriteAllText($path, $text, [System.Text.UTF8Encoding]::new($false))
}
$shim = Join-Path $buildDir 'timing_stub.c'
[System.IO.File]::WriteAllText($shim, "#include `"main.h`"`nTestDWT test_dwt;`nuint32_t SystemCoreClock = 72000000u;`nuint32_t test_control_cycles;`n", [System.Text.UTF8Encoding]::new($false))
foreach ($polarity in @(1, 0)) {
    Compile-And-Run "fsm-$polarity" @((Join-Path $repoRoot 'App/Mode_FSM/Mode_FSM.c'),
        (Join-Path $oldTests 'test_mode_fsm.c'), $shim) @((Join-Path $oldTests 'stubs'),
        (Join-Path $repoRoot 'App/Mode_FSM')) @("-DLINE_RAW_VALUE=$polarity")
}
Compile-And-Run 'imu' @((Join-Path $repoRoot 'App/IMU_Task/IMU_Task.c'),
    (Join-Path $repoRoot 'Com/Com_PID/Com_pid.c'), (Join-Path $oldTests 'test_imu_task.c')) @(
    (Join-Path $repoRoot 'App/IMU_Task'), (Join-Path $oldTests 'stubs'),
    (Join-Path $repoRoot 'Com/Com_PID'), (Join-Path $repoRoot 'Com/Com_Limit')) @()
} else {
    Write-Host 'SKIP archived control regression: HEAD no longer contains the old tests. OLED and ARM checks continue.'
}

# 用工程真实头文件检查 ARM 源码，检查程序不使用主机替身。
if (Test-Path -LiteralPath $ArmCompilerPath) {
    $project = Join-Path $repoRoot 'MDK-ARM/ALL_terain_For_11.uvprojx'
    [xml]$xml = Get-Content -LiteralPath $project -Raw
    $includeDirs = @()
    foreach ($node in $xml.SelectNodes('//Cads/VariousControls/IncludePath')) {
        foreach ($entry in $node.InnerText.Split(';')) {
            if ($entry.Trim()) { $includeDirs += [System.IO.Path]::GetFullPath((Join-Path (Split-Path $project) $entry)) }
        }
    }
    $arguments = @('-std=c99', '-mcpu=cortex-m3', '-mthumb', '-DUSE_HAL_DRIVER', '-DSTM32F103xE',
        '-fsyntax-only', '-Wall', '-Wextra', '-Werror')
    foreach ($dir in ($includeDirs | Select-Object -Unique)) { $arguments += @('-I', $dir) }
    foreach ($source in @('Com/Com_OLED/oled.c', 'Com/Com_OLED/oled_font.c',
        'Int/Int_OLED/Int_OLED.c', 'App/OLED_Debug/OLED_Debug.c', 'App/Mode_FSM/Mode_FSM.c', 'Core/Src/main.c')) {
        & $ArmCompilerPath @arguments (Join-Path $repoRoot $source)
        if ($LASTEXITCODE -ne 0) { throw "ARM check failed: $source" }
        Write-Host "ARM check passed: $source"
    }
}
git -C $repoRoot diff --check
if ($LASTEXITCODE -ne 0) { throw 'Whitespace check failed' }
Write-Host 'All checks passed; original tests deletions are preserved.'
