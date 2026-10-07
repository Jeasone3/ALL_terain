param(
    [string]$CompilerPath = "",
    [string]$ArmCompilerPath = ""
)

$ErrorActionPreference = "Stop"
$repoRoot = Split-Path -Parent $PSScriptRoot

if ([string]::IsNullOrWhiteSpace($CompilerPath)) {
    $compilerCommand = Get-Command gcc -ErrorAction Stop
    $CompilerPath = $compilerCommand.Source
}
if (-not (Test-Path -LiteralPath $CompilerPath -PathType Leaf)) {
    throw "Compiler not found: $CompilerPath"
}

# 构建产物放在系统临时目录，保留失败现场；脚本不删除任何文件。
$buildName = "mode-fsm-tests-{0}-{1}" -f (Get-Date -Format "yyyyMMdd-HHmmss"), ([guid]::NewGuid().ToString("N"))
$buildDir = Join-Path ([System.IO.Path]::GetTempPath()) $buildName
New-Item -ItemType Directory -Path $buildDir | Out-Null
Write-Host "Build directory: $buildDir"

# 必须编译真实状态机源码，替身只提供传感器、电机、姿态和中断接口。
foreach ($lineRawValue in @(1, 0)) {
    $binaryPath = Join-Path $buildDir ("mode-fsm-tests-active-{0}.exe" -f $lineRawValue)
    $compilerArgs = @(
        "-std=c99", "-Wall", "-Wextra", "-Werror", "-pedantic", "-O2",
        ("-DLINE_RAW_VALUE={0}" -f $lineRawValue),
        "-I", (Join-Path $PSScriptRoot "stubs"),
        "-I", (Join-Path $repoRoot "App\Mode_FSM"),
        (Join-Path $repoRoot "App\Mode_FSM\Mode_FSM.c"),
        (Join-Path $PSScriptRoot "test_mode_fsm.c"),
        "-lm", "-o", $binaryPath
    )
    & $CompilerPath @compilerArgs
    if ($LASTEXITCODE -ne 0) {
        throw "Compilation failed for LINE_RAW_VALUE=$lineRawValue (exit $LASTEXITCODE)."
    }

    & $binaryPath
    if ($LASTEXITCODE -ne 0) {
        throw "Tests failed for LINE_RAW_VALUE=$lineRawValue (exit $LASTEXITCODE)."
    }
}

# 角度控制单独链接真实 IMU_Task 和 PID，只有电机输出与姿态输入使用替身。
$imuBinaryPath = Join-Path $buildDir "imu-forward-tests.exe"
$imuCompilerArgs = @(
    "-std=c99", "-Wall", "-Wextra", "-Werror", "-pedantic", "-O2",
    "-I", (Join-Path $repoRoot "App\IMU_Task"),
    "-I", (Join-Path $PSScriptRoot "stubs"),
    "-I", (Join-Path $repoRoot "Com\Com_PID"),
    "-I", (Join-Path $repoRoot "Com\Com_Limit"),
    (Join-Path $repoRoot "App\IMU_Task\IMU_Task.c"),
    (Join-Path $repoRoot "Com\Com_PID\Com_pid.c"),
    (Join-Path $PSScriptRoot "test_imu_task.c"),
    "-lm", "-o", $imuBinaryPath
)
& $CompilerPath @imuCompilerArgs
if ($LASTEXITCODE -ne 0) {
    throw "IMU control test compilation failed (exit $LASTEXITCODE)."
}
& $imuBinaryPath
if ($LASTEXITCODE -ne 0) {
    throw "IMU control tests failed (exit $LASTEXITCODE)."
}

if (-not [string]::IsNullOrWhiteSpace($ArmCompilerPath)) {
    if (-not (Test-Path -LiteralPath $ArmCompilerPath -PathType Leaf)) {
        throw "ARM compiler not found: $ArmCompilerPath"
    }

    # 从 Keil 工程提取真实头文件目录，ARM 检查不使用主机替身。
    $projectPath = Join-Path $repoRoot "MDK-ARM\ALL_terain_For_11.uvprojx"
    $projectDir = Split-Path -Parent $projectPath
    [xml]$projectXml = Get-Content -LiteralPath $projectPath -Raw
    $includeDirs = @(
        (Join-Path $repoRoot "Core\Inc"),
        (Join-Path $repoRoot "Drivers\STM32F1xx_HAL_Driver\Inc"),
        (Join-Path $repoRoot "Drivers\STM32F1xx_HAL_Driver\Inc\Legacy"),
        (Join-Path $repoRoot "Drivers\CMSIS\Device\ST\STM32F1xx\Include"),
        (Join-Path $repoRoot "Drivers\CMSIS\Include")
    )
    foreach ($includeNode in $projectXml.SelectNodes("//Cads/VariousControls/IncludePath")) {
        foreach ($includePath in $includeNode.InnerText.Split(';')) {
            if (-not [string]::IsNullOrWhiteSpace($includePath)) {
                $includeDirs += [System.IO.Path]::GetFullPath((Join-Path $projectDir $includePath))
            }
        }
    }
    $armCompilerArgs = @(
        "-std=c99", "-mcpu=cortex-m3", "-mthumb",
        "-DUSE_HAL_DRIVER", "-DSTM32F103xE",
        "-fsyntax-only", "-Wall", "-Wextra"
    )
    foreach ($includeDir in ($includeDirs | Select-Object -Unique)) {
        $armCompilerArgs += @("-I", $includeDir)
    }
    $changedSources = @(
        "App\Mode_FSM\Mode_FSM.c",
        "App\IMU_Task\IMU_Task.c",
        "Core\Src\main.c",
        "Int\Int_MPU6050\Int_MPU6050.c",
        "App\Track_Task\Track_Task.c"
    )
    foreach ($sourcePath in $changedSources) {
        Write-Host "ARM syntax check: $sourcePath"
        & $ArmCompilerPath @armCompilerArgs (Join-Path $repoRoot $sourcePath)
        if ($LASTEXITCODE -ne 0) {
            throw "ARM syntax check failed: $sourcePath (exit $LASTEXITCODE)."
        }
    }
}
