param(
    [string]$Gcc = 'D:/CoderSpcace/IDE/mingw64/bin/gcc.exe',
    [string]$BuildDirectory = (Join-Path $env:TEMP 'all-terrain-motion-tests')
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

Invoke-MotionTest -Name 'test_motion_route' -Arguments @(
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

Invoke-MotionTest -Name 'test_debug_log' -Arguments @(
    (Join-Path $PSScriptRoot 'test_debug_log.c'),
    (Join-Path $projectRoot 'App/Debug_Log/Debug_Log.c')
)

Invoke-MotionTest -Name 'test_attitude' -Arguments @(
    (Join-Path $PSScriptRoot 'test_attitude.c'),
    (Join-Path $projectRoot 'Com/Com_Attitude/Attitude.c')
)
