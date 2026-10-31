param(
    [string]$Gcc = 'D:/CoderSpcace/IDE/mingw64/bin/gcc.exe',
    [string]$BuildDirectory = (Join-Path $env:TEMP 'all-terrain-motion-tests'),
    [switch]$ImuRetryOnly,
    [switch]$PlatformPitchOnly,
    [switch]$RouteFinishOnly,
    [switch]$RouteReverseOnly,
    [switch]$RouteStartOnly,
    [switch]$RouteAfterPlatformOnly,
    [switch]$SonarDiagnosticsOnly
)

$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$fakeHalDirectory = Join-Path $PSScriptRoot 'fake_hal'

if (-not (Test-Path -LiteralPath $Gcc)) {
    $Gcc = (Get-Command gcc -ErrorAction Stop).Source
}

# 编译结果仅写入 TEMP，不向源码目录写入可执行文件，也不自动删除任何文件。
New-Item -ItemType Directory -Path $BuildDirectory -Force | Out-Null
$includeDirectories = @(
    $fakeHalDirectory,
    (Join-Path $projectRoot 'App/Mode_FSM'),
    (Join-Path $projectRoot 'App/Route'),
    (Join-Path $projectRoot 'App/Debug_Log'),
    (Join-Path $projectRoot 'App/Track_Task'),
    (Join-Path $projectRoot 'App/IMU_Task'),
    (Join-Path $projectRoot 'Com/Com_Attitude'),
    (Join-Path $projectRoot 'Com/Com_IMU_Config'),
    (Join-Path $projectRoot 'Com/Com_PID'),
    (Join-Path $projectRoot 'Com/Com_Limit'),
    (Join-Path $projectRoot 'Int/Int_Track'),
    (Join-Path $projectRoot 'Int/Int_OLED'),
    (Join-Path $projectRoot 'Int/Int_Motor')
    (Join-Path $projectRoot 'Int/Int_Beep')
    (Join-Path $projectRoot 'Int/Int_SR04')
)
$commonArguments = @('-std=c99', '-Wall', '-Wextra', '-Werror', '-pedantic', '-O2')
foreach ($includeDirectory in $includeDirectories) {
    $commonArguments += @('-I', $includeDirectory)
}

<#
@brief 编译并运行指定的宿主测试，非零编译或测试退出码立即终止脚本。
@param Name 测试名称，作为 TEMP 下可执行文件的名称。
@param Arguments 需要编译的真实源码、测试源码与额外链接参数。
@return 无；成功测试结果由可执行文件直接输出，失败抛出异常。
@note 公共参数使用 C99 和严格警告；调用者不得把源码修改命令混入参数。
#>
function Invoke-MotionTest {
    param([string]$Name, [string[]]$Arguments)
    $outputPath = Join-Path $BuildDirectory ($Name + '.exe')
    & $Gcc @commonArguments @Arguments '-lm' '-o' $outputPath
    if ($LASTEXITCODE -ne 0) { throw "Compilation failed: $Name" }
    & $outputPath
    if ($LASTEXITCODE -ne 0) { throw "Test failed: $Name" }
}

$motionSources = @(
    (Join-Path $PSScriptRoot 'test_motion_route.c'),
    (Join-Path $projectRoot 'App/Mode_FSM/Mode_FSM.c'),
    (Join-Path $projectRoot 'App/Route/Route.c'),
    (Join-Path $projectRoot 'App/Debug_Log/Debug_Log.c'),
    (Join-Path $projectRoot 'App/Track_Task/Track_Task.c'),
    (Join-Path $projectRoot 'App/IMU_Task/IMU_Task.c'),
    (Join-Path $projectRoot 'Com/Com_PID/Com_pid.c'),
    (Join-Path $projectRoot 'Com/Com_Limit/Com_Limit.c'),
    '-Wl,--wrap=PID_calc'
)

if ($PlatformPitchOnly) {
    # 使用真实第 9 项到达高台，分别检查默认时长、可调时长和非整周期向上取整。
    foreach ($stableMs in @(100, 200, 105)) {
        Invoke-MotionTest -Name "test_platform_pitch_$stableMs" -Arguments ($motionSources + @(
            '-DTEST_PLATFORM_PITCH_ONLY', '-DROUTE_START_STEP=9u',
            "-DROUTE_REVERSE_DRIVE_STABLE_MS=$($stableMs)u", "-DTEST_PLATFORM_STABLE_MS=$($stableMs)u"))
    }
    # 截止在第 9 项时仍完整离台，但不能越过截止项进入第 10 项。
    Invoke-MotionTest -Name 'test_platform_pitch_limit9' -Arguments ($motionSources + @(
        '-DTEST_PLATFORM_PITCH_ONLY', '-DROUTE_START_STEP=9u', '-DROUTE_DEBUG_STEP_LIMIT=9u'))
    return
}

if ($SonarDiagnosticsOnly) {
    # 单独验证测距拒绝、超时保留和 OLED 距离，不受旧路线 PWM 断言影响。
    Invoke-MotionTest -Name 'test_sonar_diagnostics' -Arguments ($motionSources + @(
        '-DTEST_ROUTE_AFTER_PLATFORM_ONLY', '-DTEST_SONAR_DIAGNOSTICS_ONLY', '-DROUTE_START_STEP=12u'))
    return
}

if ($ImuRetryOnly) {
    # 链接真实 MPU6050 驱动验证总线重试，再链接真实状态机验证暂停和故障边界。
    Invoke-MotionTest -Name 'test_imu_retry_driver' -Arguments @(
        (Join-Path $PSScriptRoot 'test_imu_bias.c'),
        (Join-Path $projectRoot 'Int/Int_MPU6050/Int_MPU6050.c'),
        (Join-Path $projectRoot 'Com/Com_Attitude/Attitude.c'),
        # 当前姿态算法的既有 ez 未使用警告不影响总线回归，保留可见警告。
        '-Wno-error=unused-but-set-variable'
    )
    Invoke-MotionTest -Name 'test_imu_retry_policy' -Arguments ($motionSources + '-DTEST_IMU_RETRY_ONLY')
    return
}

if ($RouteAfterPlatformOnly) {
    # 验证两次右转后接第 12 项左转，以及仅本次左转使用 10°容差。
    Invoke-MotionTest -Name 'test_route_after_platform' -Arguments ($motionSources + '-DTEST_ROUTE_AFTER_PLATFORM_ONLY')
    # 单独从两个新表项启动，检查被跳过的高台和前一右转不会影响当前动作。
    foreach ($startStep in 10..20) {
        Invoke-MotionTest -Name "test_route_after_platform_start$startStep" -Arguments ($motionSources + @(
            '-DTEST_ROUTE_AFTER_PLATFORM_ONLY', "-DROUTE_START_STEP=$($startStep)u"))
    }
    # 0 表示全部 12 项；9、10、11 分别覆盖离台和两处右转后的调试截止。
    foreach ($limit in @(0, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19)) {
        Invoke-MotionTest -Name "test_route_after_platform_limit$limit" -Arguments ($motionSources + @(
            '-DTEST_ROUTE_AFTER_PLATFORM_ONLY', '-DROUTE_START_STEP=9u', "-DROUTE_DEBUG_STEP_LIMIT=$($limit)u"))
    }
    return
}

if ($RouteStartOnly) {
    # 按需逐项验证起始配置、完整高台离台时序及末项左转后持续循迹，另外覆盖旧截止配置。
    foreach ($startStep in 1..12) {
        Invoke-MotionTest -Name "test_route_start_$startStep" -Arguments ($motionSources + @(
            '-DTEST_ROUTE_START_ONLY', "-DROUTE_START_STEP=$($startStep)u"))
    }
    Invoke-MotionTest -Name 'test_route_start6_all' -Arguments ($motionSources + @(
        '-DTEST_ROUTE_START_ONLY', '-DROUTE_START_STEP=6u', '-DROUTE_DEBUG_STEP_LIMIT=0u'))
    Invoke-MotionTest -Name 'test_route_start6_limit8' -Arguments ($motionSources + @(
        '-DTEST_ROUTE_START_ONLY', '-DROUTE_START_STEP=6u', '-DROUTE_DEBUG_STEP_LIMIT=8u'))
    return
}

if ($RouteReverseOnly) {
    # 验证第 8 项的倒退、入口、通行计时及取消边界，覆盖完整路线与截止 8。
    Invoke-MotionTest -Name 'test_route_reverse' -Arguments ($motionSources + '-DTEST_ROUTE_REVERSE_ONLY')
    Invoke-MotionTest -Name 'test_route_reverse_all' -Arguments ($motionSources + @('-DTEST_ROUTE_REVERSE_ONLY', '-DROUTE_DEBUG_STEP_LIMIT=0u'))
    Invoke-MotionTest -Name 'test_route_reverse_limit8' -Arguments ($motionSources + @('-DTEST_ROUTE_REVERSE_ONLY', '-DROUTE_DEBUG_STEP_LIMIT=8u'))
    return
}

if ($RouteFinishOnly) {
    # 单独验证完整路线的收尾及截止 8 的兼容性，不依赖旧参数的全量调参断言。
    Invoke-MotionTest -Name 'test_route_finish' -Arguments ($motionSources + '-DTEST_ROUTE_FINISH_ONLY')
    Invoke-MotionTest -Name 'test_route_finish_all' -Arguments ($motionSources + @('-DTEST_ROUTE_FINISH_ONLY', '-DROUTE_DEBUG_STEP_LIMIT=0u'))
    Invoke-MotionTest -Name 'test_route_finish_limit8' -Arguments ($motionSources + @('-DTEST_ROUTE_FINISH_ONLY', '-DROUTE_DEBUG_STEP_LIMIT=8u'))
    return
}

# 先运行独立 IMU 回归，避免旧路线调参断言阻断姿态与零偏检查。
Invoke-MotionTest -Name 'test_attitude' -Arguments @(
    (Join-Path $PSScriptRoot 'test_attitude.c'),
    (Join-Path $projectRoot 'Com/Com_Attitude/Attitude.c')
)

Invoke-MotionTest -Name 'test_imu_bias' -Arguments @(
    (Join-Path $PSScriptRoot 'test_imu_bias.c'),
    (Join-Path $projectRoot 'Int/Int_MPU6050/Int_MPU6050.c'),
    (Join-Path $projectRoot 'Com/Com_Attitude/Attitude.c')
)

Invoke-MotionTest -Name 'test_motion_route' -Arguments $motionSources
# 单段截止必须允许未填写的后续段；0 则表示执行全部已编写段。
Invoke-MotionTest -Name 'test_motion_route_limit1' -Arguments ($motionSources + '-DROUTE_DEBUG_STEP_LIMIT=1u')
Invoke-MotionTest -Name 'test_motion_route_limit2' -Arguments ($motionSources + '-DROUTE_DEBUG_STEP_LIMIT=2u')
Invoke-MotionTest -Name 'test_motion_route_limit3' -Arguments ($motionSources + '-DROUTE_DEBUG_STEP_LIMIT=3u')
Invoke-MotionTest -Name 'test_motion_route_limit4' -Arguments ($motionSources + '-DROUTE_DEBUG_STEP_LIMIT=4u')
Invoke-MotionTest -Name 'test_motion_route_limit5' -Arguments ($motionSources + '-DROUTE_DEBUG_STEP_LIMIT=5u')
Invoke-MotionTest -Name 'test_motion_route_limit6' -Arguments ($motionSources + '-DROUTE_DEBUG_STEP_LIMIT=6u')
Invoke-MotionTest -Name 'test_motion_route_limit7' -Arguments ($motionSources + '-DROUTE_DEBUG_STEP_LIMIT=7u')
Invoke-MotionTest -Name 'test_motion_route_limit8' -Arguments ($motionSources + '-DROUTE_DEBUG_STEP_LIMIT=8u')
Invoke-MotionTest -Name 'test_motion_route_all' -Arguments ($motionSources + '-DROUTE_DEBUG_STEP_LIMIT=0u')

Invoke-MotionTest -Name 'test_debug_log' -Arguments @(
    (Join-Path $PSScriptRoot 'test_debug_log.c'),
    (Join-Path $projectRoot 'App/Debug_Log/Debug_Log.c')
)
