param(
    [string]$Gcc = 'D:/CoderSpcace/IDE/mingw64/bin/gcc.exe',
    [string]$BuildDirectory = (Join-Path $env:TEMP 'all-terrain-motion-tests'),
    [switch]$RouteFinishOnly,
    [switch]$RtosOnly
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

if ($RtosOnly) {
    # 内核 API 替身只驱动任务边界，三个入口及 SysTick/TIM4 回调均编译生产源码。
    Invoke-MotionTest -Name 'test_rtos_tasks' -Arguments @(
        '-iquote', (Join-Path $PSScriptRoot 'fake_rtos'),
        '-I', (Join-Path $projectRoot 'App/RTOS'),
        '-I', (Join-Path $projectRoot 'Core/Inc'),
        (Join-Path $PSScriptRoot 'test_rtos_tasks.c'),
        (Join-Path $projectRoot 'App/RTOS/App_RTOS.c'),
        (Join-Path $projectRoot 'App/RTOS/Control_Task.c'),
        (Join-Path $projectRoot 'App/RTOS/Display_Task.c'),
        (Join-Path $projectRoot 'App/RTOS/Recovery_Task.c'),
        (Join-Path $projectRoot 'Core/Src/stm32f1xx_it.c')
    )
    Invoke-MotionTest -Name 'test_rtos_motion' -Arguments ($motionSources + '-DTEST_RTOS_MIGRATION_ONLY')
    Invoke-MotionTest -Name 'test_rtos_motion_all' -Arguments ($motionSources + @('-DTEST_RTOS_MIGRATION_ONLY', '-DROUTE_DEBUG_STEP_LIMIT=0u'))
    Invoke-MotionTest -Name 'test_rtos_motion_limit8' -Arguments ($motionSources + @('-DTEST_RTOS_MIGRATION_ONLY', '-DROUTE_DEBUG_STEP_LIMIT=8u'))
    Invoke-MotionTest -Name 'test_rtos_debug_log' -Arguments @(
        (Join-Path $PSScriptRoot 'test_debug_log.c'),
        (Join-Path $projectRoot 'App/Debug_Log/Debug_Log.c')
    )
    Invoke-MotionTest -Name 'test_rtos_attitude' -Arguments @(
        (Join-Path $PSScriptRoot 'test_attitude.c'),
        (Join-Path $projectRoot 'Com/Com_Attitude/Attitude.c')
    )
    return
}

if ($RouteFinishOnly) {
    # 单独验证完整路线的收尾及截止 8 的兼容性，不依赖旧参数的全量调参断言。
    Invoke-MotionTest -Name 'test_route_finish' -Arguments ($motionSources + '-DTEST_ROUTE_FINISH_ONLY')
    Invoke-MotionTest -Name 'test_route_finish_all' -Arguments ($motionSources + @('-DTEST_ROUTE_FINISH_ONLY', '-DROUTE_DEBUG_STEP_LIMIT=0u'))
    Invoke-MotionTest -Name 'test_route_finish_limit8' -Arguments ($motionSources + @('-DTEST_ROUTE_FINISH_ONLY', '-DROUTE_DEBUG_STEP_LIMIT=8u'))
    return
}

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

Invoke-MotionTest -Name 'test_attitude' -Arguments @(
    (Join-Path $PSScriptRoot 'test_attitude.c'),
    (Join-Path $projectRoot 'Com/Com_Attitude/Attitude.c')
)
