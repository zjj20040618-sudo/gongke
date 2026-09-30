# Host-only regression checks. They do not flash or exercise the robot.
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$compiler = 'D:\mingw64\bin\gcc.exe'
if (-not (Test-Path -LiteralPath $compiler)) { throw "GCC not found: $compiler" }

function Invoke-HostCase {
    param([string]$Name, [string[]]$Sources, [string[]]$Includes, [string[]]$ExtraFlags = @())
    $output = Join-Path ([IO.Path]::GetTempPath()) "$Name.exe"
    $arguments = @('-std=c11', '-Wall', '-Wextra', '-Werror')
    foreach ($include in $Includes) { $arguments += "-I$include" }
    $arguments += $Sources
    $arguments += $ExtraFlags
    $arguments += @('-o', $output)
    & $compiler @arguments
    if ($LASTEXITCODE -ne 0) { throw "$Name compilation failed" }
    & $output
    if ($LASTEXITCODE -ne 0) { throw "$Name failed" }
}

Push-Location $projectRoot
try {
    Invoke-HostCase 'eod_arm_task_flow_test' @('tests/arm_task_flow_test.c', 'App/task_eod.c', 'App/task_rescue.c') @('App')
    Invoke-HostCase 'eod_anti_task_flow_test' @('tests/anti_task_flow_test.c', 'App/task_anti.c') @('App')
    Invoke-HostCase 'eod_obstacle_abort_test' @('tests/obstacle_abort_test.c', 'App/auto_steps.c') @('tests/stubs', 'App')
    Invoke-HostCase 'eod_mission_start_abort_test' @('tests/mission_start_abort_test.c') @('tests/stubs', 'App')
    Invoke-HostCase 'eod_proto_frame_test' @('tests/proto_frame_test.c', 'App/proto.c') @('tests/stubs', 'App')
    Invoke-HostCase 'eod_rotate_continuous_test' @('tests/rotate_continuous_test.c', 'App/steps.c') @('tests/stubs', 'App') @('-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections', '-lm')
    Invoke-HostCase 'eod_align_timeout_test' @('tests/align_timeout_test.c', 'App/steps.c') @('tests/stubs', 'App') @('-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections', '-lm')

    $mission = Get-Content 'App/mission.c' -Raw -Encoding utf8
    $test = Get-Content 'App/test.c' -Raw -Encoding utf8
    $start = [regex]::Match($mission, 'int mission_start\(void\)\s*\{(?<body>.*?)\n\}', 'Singleline').Groups['body'].Value
    $main = [regex]::Match($mission, 'void mission_main\(void\)\s*\{(?<body>.*)\n\}', 'Singleline').Groups['body'].Value
    $abort = [regex]::Match($test, 'static void cmd_abort\(void\)\s*\{(?<body>.*?)\n\}', 'Singleline').Groups['body'].Value
    if (-not $start -or -not $main -or -not $abort) { throw 'Start/abort source contract: function not found' }
    if ($start.IndexOf('run_reset();') -lt 0 -or $start.IndexOf('run_reset();') -ge $start.IndexOf('s_start_req = 1;')) {
        throw 'Start/abort source contract: reset must precede start latch'
    }
    if ($main.Contains('run_reset();')) { throw 'Start/abort source contract: MissionTask must not clear abort' }
    if ($main.IndexOf('if (run_aborted()) goto failed;') -le $main.IndexOf('s_start_req = 0;') -or
        $main.IndexOf('if (run_aborted()) goto failed;') -ge $main.IndexOf('to_state(MS_READ_QR);')) {
        throw 'Start/abort source contract: abort must be checked before QR motion'
    }
    if ($abort.IndexOf('run_abort();') -lt 0 -or $abort.IndexOf('run_abort();') -ge $abort.IndexOf('motion_brake();')) {
        throw 'Start/abort source contract: latch abort before braking'
    }
    Write-Output 'start/abort source contract: 4 checks passed'

    $routeCalls = $mission -split "`n" | Where-Object { $_ -match '^\s*if \(!route_(straight|strafe|strafe_to)\(' }
    $straight = @($routeCalls | Where-Object { $_ -match 'route_straight\(' })
    $strafe = @($routeCalls | Where-Object { $_ -match 'route_strafe(_to)?\(' })
    if ($straight.Count -ne 3 -or @($straight | Where-Object { $_ -notmatch 'ROUTE_FWD_V_MMS' }).Count -ne 0) {
        throw 'Route speed contract: every forward leg must use ROUTE_FWD_V_MMS'
    }
    if ($strafe.Count -ne 6 -or @($strafe | Where-Object { $_ -notmatch 'ROUTE_STRAFE_V_MMS' }).Count -ne 0) {
        throw 'Route speed contract: every lateral leg must use ROUTE_STRAFE_V_MMS'
    }
    if ($mission -notmatch 'if \(ROUTE_FWD_V_MMS <= 0\.0f\)' -or
        $mission -notmatch 'if \(ROUTE_STRAFE_V_MMS <= 0\.0f\)') {
        throw 'Route speed contract: both speed gates are required'
    }
    Write-Output 'route speed source contract: 3 forward + 6 lateral calls, 2 gates checked'

    $benchConfig = Get-Content 'App/test_config.h' -Raw -Encoding utf8
    if ($benchConfig -notmatch '(?m)^#define BENCH_AUTO\s+0u\s*$') {
        throw 'Bench safety contract: power-on auto motion must remain disabled'
    }
    if ($benchConfig -notmatch '(?m)^#define T_LATERAL_DEFAULT_V_MMS\s+300\.0f\s*$' -or
        $benchConfig -notmatch '(?m)^#define T_LATERAL_DEFAULT_D_MM\s+1500\.0f\s*$') {
        throw 'Bench default contract: lateral trial defaults changed'
    }
    Write-Output 'bench safety/default source contract: 3 checks passed'
} finally {
    Pop-Location
}
