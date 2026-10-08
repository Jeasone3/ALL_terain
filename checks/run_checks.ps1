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
$archiveRef = 'HEAD'
$oldFiles = @(git -C $repoRoot ls-tree -r --name-only $archiveRef tests)
if ($LASTEXITCODE -ne 0 -or $oldFiles.Count -eq 0) {
    # HEAD 已删除样例时，定位最近一次添加或修改状态机样例的提交。
    $candidateRef = @(git -C $repoRoot log -1 --diff-filter=AM --format=%H -- tests/test_mode_fsm.c)
    if ($LASTEXITCODE -eq 0 -and $candidateRef.Count -gt 0 -and $candidateRef[0].Trim()) {
        $archiveRef = $candidateRef[0].Trim()
        $oldFiles = @(git -C $repoRoot ls-tree -r --name-only $archiveRef tests)
    } else {
        $oldFiles = @()
    }
}
if ($LASTEXITCODE -eq 0 -and $oldFiles.Count -gt 0) {
Write-Host "Archived control regression source: $archiveRef"
foreach ($file in $oldFiles) {
    $relative = $file.Substring(6)
    $target = Join-Path $oldTests $relative
    [System.IO.Directory]::CreateDirectory((Split-Path -Parent $target)) | Out-Null
    $content = (git -C $repoRoot show "${archiveRef}:$file") -join "`n"
    if ($LASTEXITCODE -ne 0) { throw "Cannot read $file from $archiveRef" }
    [System.IO.File]::WriteAllText($target, $content + "`n", [System.Text.UTF8Encoding]::new($false))
}

# 给旧硬件替身补充 PWM 和 DWT；状态阶段调整仅适配临时样例，保留其他断言。
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
        # 旧助手把候选和推进合并计时；新助手逐帧验证独立推进再等待原转弯断言。
        $turnHelperPattern = '(?s)static int enter_turn\(uint8_t mask, Run_State expected_state, float target\).*?(?=static int finish_exit\(float target\))'
        $newTurnHelper = @'
static int enter_turn(uint8_t mask, Run_State expected_state, float target)
{
    unsigned int i;
    steps(JUNCTION_CONFIRM_FRAMES, mask, 0.0f);
    CHECK(snapshot().state == STATE_APPROACH_TURN);
    CHECK(snapshot().event_count == 1u && snapshot().state_frames == 0u);
    CHECK(close_float(snapshot().target_yaw, 0.0f));
    CHECK(close_float(s_last_target, 0.0f));
    CHECK(s_turn_calls == 0);
    for (i = 0u; i < TURN_APPROACH_FRAMES; ++i) {
        step(mask, 0.0f);
        CHECK(snapshot().state != STATE_FAULT_STOP);
        CHECK(s_turn_calls == 0);
        if (i + 1u < TURN_APPROACH_FRAMES) {
            CHECK(snapshot().state == STATE_APPROACH_TURN);
            CHECK(close_float(snapshot().target_yaw, 0.0f));
            CHECK(close_float(s_last_target, 0.0f));
        }
    }
    CHECK(snapshot().state == expected_state);
    CHECK(snapshot().event_count == 1u);
    CHECK(close_float(snapshot().target_yaw, target));
    CHECK(close_float(s_last_target, target));
    CHECK(s_target_calls > 0);
    CHECK(s_forward_calls == (int)(JUNCTION_CONFIRM_FRAMES + TURN_APPROACH_FRAMES));
    CHECK(close_float(s_last_forward_speed, TURN_APPROACH_BASE_PWM));
    CHECK(s_pid_clear_calls == 0);
    return 1;
}

'@
        $turnHelperRegex = [regex]::new($turnHelperPattern)
        if ($turnHelperRegex.Matches($text).Count -ne 1) {
            throw 'Archived enter_turn helper changed; cannot adapt the approach phase safely.'
        }
        $text = $turnHelperRegex.Replace($text, $newTurnHelper)

        # 过期左证据应仍归类为右侧；确认时目标保持零，推进完成后才设置负九十度。
        $expiredStart = $text.IndexOf('static int test_old_left_evidence_cannot_form_cross(void)')
        if ($expiredStart -lt 0) {
            throw 'Archived evidence-age test changed; cannot find its start safely.'
        }
        $expiredEnd = $text.IndexOf('static int test_pair_evidence_age_eight_accepts_cross(void)', $expiredStart)
        if ($expiredEnd -lt 0) {
            throw 'Archived evidence-age test changed; cannot adapt its approach assertions safely.'
        }
        $expiredTest = $text.Substring($expiredStart, $expiredEnd - $expiredStart)
        $oldExpiredAssertions = @'
    CHECK(close_float(snapshot().target_yaw, -90.0f));
    CHECK(snapshot().state == STATE_TURN_RIGHT || snapshot().state == STATE_APPROACH_TURN);
'@
        $newExpiredAssertions = @'
    CHECK(snapshot().state == STATE_APPROACH_TURN);
    CHECK(close_float(snapshot().target_yaw, 0.0f));
    CHECK(close_float(s_last_target, 0.0f));
    steps(TURN_APPROACH_FRAMES, 0xf0u, 0.0f);
    CHECK(snapshot().state == STATE_TURN_RIGHT);
    CHECK(close_float(snapshot().target_yaw, -90.0f));
    CHECK(snapshot().event_count == 1u && snapshot().last_event == JUNCTION_RIGHT);
'@
        $oldExpiredAssertions = $oldExpiredAssertions.Replace("`r`n", "`n")
        $newExpiredAssertions = $newExpiredAssertions.Replace("`r`n", "`n")
        if (-not $expiredTest.Contains($oldExpiredAssertions)) {
            throw 'Archived evidence-age assertion block changed; cannot adapt it safely.'
        }
        $adaptedExpiredTest = $expiredTest.Replace($oldExpiredAssertions, $newExpiredAssertions)
        $text = $text.Remove($expiredStart, $expiredEnd - $expiredStart).Insert($expiredStart, $adaptedExpiredTest)
        $extraCases = [System.IO.File]::ReadAllText((Join-Path $PSScriptRoot 'test_mode_diagnostics.inc'))
        $approachCases = [System.IO.File]::ReadAllText((Join-Path $PSScriptRoot 'test_turn_approach.inc'))
        $text = $text.Replace('int main(void)', $extraCases + "`n" + $approachCases + "`nint main(void)")
        $caseEntries = @'
const Test_Case cases[] = {
        {"diagnostic_freshness_and_evidence", test_diagnostic_freshness_and_evidence},
        {"control_timing_callback", test_control_timing_callback},
        {"turn_approach_finishes_before_rotation", test_turn_approach_finishes_before_rotation},
        {"confirmed_approach_allows_center_loss_and_ignores_new_width", test_confirmed_approach_allows_center_loss_and_ignores_new_width},
        {"turn_approach_imu_failure_stops_same_frame", test_turn_approach_imu_failure_stops_same_frame},
        {"turn_approach_heading_limit_stops_same_frame", test_turn_approach_heading_limit_stops_same_frame},
        {"cross_skips_extra_turn_approach", test_cross_skips_extra_turn_approach},
'@
        $text = $text.Replace('const Test_Case cases[] = {', $caseEntries)
    } elseif ($name -eq 'test_imu_task.c') {
        $extraHeadingCases = [System.IO.File]::ReadAllText((Join-Path $PSScriptRoot 'test_imu_heading.inc'))
        if (-not $text.Contains('int main(void)') -or -not $text.Contains('    IMUTask_Init();')) {
            throw 'Archived IMU test layout changed; cannot inject heading regression checks safely.'
        }
        # 先修改归档 main，避免把调用插入新增检查函数自身而形成递归。
        $firstInit = $text.IndexOf('    IMUTask_Init();', $text.IndexOf('int main(void)'))
        if ($firstInit -lt 0) { throw 'Archived IMU main has no initialization insertion point.' }
        $text = $text.Insert($firstInit, "    if (check_early_heading_correction()) return 1;`n")
        $text = $text.Replace('int main(void)', $extraHeadingCases + "`nint main(void)")
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
    Write-Host 'SKIP archived control regression: no readable tests found in HEAD or an adding/modifying commit. OLED and ARM checks continue.'
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
        'Int/Int_OLED/Int_OLED.c', 'App/OLED_Debug/OLED_Debug.c', 'App/Mode_FSM/Mode_FSM.c',
        'App/IMU_Task/IMU_Task.c', 'Core/Src/main.c')) {
        & $ArmCompilerPath @arguments (Join-Path $repoRoot $source)
        if ($LASTEXITCODE -ne 0) { throw "ARM check failed: $source" }
        Write-Host "ARM check passed: $source"
    }
}
git -C $repoRoot diff --check
if ($LASTEXITCODE -ne 0) { throw 'Whitespace check failed' }
Write-Host 'All checks passed; original tests deletions are preserved.'
