# Host-only regression checks. They do not flash or exercise the robot.
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$compiler = 'D:\mingw64\bin\gcc.exe'
if (-not (Test-Path -LiteralPath $compiler)) { throw "GCC not found: $compiler" }
$hostPython = 'D:\mingw64\bin\python.exe'
if (-not (Test-Path -LiteralPath $hostPython)) { throw "Python not found: $hostPython" }
$hostBuildId = [Guid]::NewGuid().ToString('N')
$previousHostCompiler = $env:EOD_HOST_CC
$env:EOD_HOST_CC = $compiler

function Invoke-HostCase {
    param([string]$Name, [string[]]$Sources, [string[]]$Includes, [string[]]$ExtraFlags = @())
    $output = Join-Path ([IO.Path]::GetTempPath()) "$Name-$hostBuildId.exe"
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

function Invoke-HostPythonCase {
    param([string]$Name, [string]$Script)
    & $hostPython -B $Script
    if ($LASTEXITCODE -ne 0) { throw "$Name failed" }
}

Push-Location $projectRoot
try {
    # Grab XY uses the single MissionTask. Its float printf paths include
    # indirect calls which Keil's reported known depth cannot fully bound.
    $taskRtos = Get-Content 'Src/freertos.c' -Raw -Encoding utf8
    $taskIoc = Get-Content 'jiejie.ioc' -Raw -Encoding utf8
    $taskAttr = [regex]::Match($taskRtos, 'MissionTask_attributes\s*=\s*\{(?<body>.*?)\};', 'Singleline').Groups['body'].Value
    $taskStack = [regex]::Match($taskAttr, '\.stack_size\s*=\s*(?<words>\d+)\s*\*\s*4')
    $taskIocStack = [regex]::Match($taskIoc, 'MissionTask,32,(?<words>\d+),StartTask03,')
    if (-not $taskStack.Success -or -not $taskIocStack.Success -or
        [int]$taskStack.Groups['words'].Value -lt 512 -or
        $taskStack.Groups['words'].Value -ne $taskIocStack.Groups['words'].Value) {
        throw 'Grab XY stack contract: MissionTask needs at least 2 KB and matching IOC words'
    }
    Write-Output 'grab XY task stack contract: at least 2 KB and matching IOC configuration'
    Invoke-HostPythonCase 'dedicated laser pin/init/fault contracts' 'tests/test_laser_tb_contract.py'
    Invoke-HostCase 'eod_laser_tb6612_test' @('tests/laser_tb6612_test.c') @('tests/laser_stubs', 'App')
    Invoke-HostCase 'eod_arm_task_flow_test' @('tests/arm_task_flow_test.c', 'App/task_eod.c', 'App/task_rescue.c') @('App')
    Invoke-HostCase 'eod_anti_task_flow_test' @('tests/anti_task_flow_test.c', 'App/task_anti.c') @('App')
    Invoke-HostCase 'eod_target_fire_timing_test' @('tests/target_fire_timing_test.c', 'App/steps.c') @('tests/stubs', 'App') @('-ffunction-sections', '-fdata-sections', '-fno-asynchronous-unwind-tables', '-fno-unwind-tables', '-Wl,--gc-sections', '-lm')
    Invoke-HostCase 'eod_obstacle_abort_test' @('tests/obstacle_abort_test.c', 'App/auto_steps.c') @('tests/stubs', 'App')
    Invoke-HostCase 'eod_mission_start_abort_test' @('tests/mission_start_abort_test.c') @('tests/stubs', 'App')
    # Real legacy test.c has an unrelated unused cmd_reset local; report it without changing that code.
    Invoke-HostCase 'eod_g_command_stop_test' @('tests/g_command_stop_test.c') @('tests/stubs', 'App') @('-Wno-error=unused-variable', '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections', '-lm')
    # Real IK/ctrl target plumbing: keep the legacy unused pose local warning visible.
    Invoke-HostCase 'eod_forward_ff_ik_test' @('tests/forward_ff_ik_test.c', 'App/motion.c') @('tests/stubs', 'App') @('-Wno-error=unused-variable', '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections', '-lm')
    Invoke-HostCase 'eod_precise_velocity_test' @('tests/precise_velocity_test.c') @('tests/stubs', 'App') @('-Wno-error=unused-variable', '-lm')
    Invoke-HostCase 'eod_speed_yaw_tuning_test' @('tests/speed_yaw_tuning_test.c') @('tests/stubs', 'App') @('-Wno-error=unused-variable', '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections', '-lm')
    Invoke-HostCase 'eod_translation_post_yaw_test' @('tests/translation_post_yaw_test.c') @('tests/stubs', 'App') @('-Wno-error=unused-variable', '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections', '-lm')
    Invoke-HostCase 'eod_translation_feedforward_test' @('tests/translation_feedforward_test.c') @('tests/stubs', 'App') @('-Wno-error=unused-variable', '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections', '-lm')
    Invoke-HostCase 'eod_speed_yaw_heading_test' @('tests/speed_yaw_heading_test.c', 'App/steps.c', 'App/motion.c') @('tests/stubs', 'App') @('-Wno-error=unused-variable', '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections', '-lm')
    Invoke-HostCase 'eod_route_softstop_test' @('tests/route_softstop_test.c') @('tests/stubs', 'App') @('-Wno-error=unused-variable', '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections', '-lm')
    Invoke-HostCase 'eod_route31_tuning_scope_test' @('tests/route31_tuning_scope_test.c') @('tests/stubs', 'App') @('-Wno-error=unused-variable', '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections', '-lm')
    Invoke-HostCase 'eod_route31_qr_gate_test' @('tests/route31_qr_gate_test.c') @('tests/stubs', 'App') @('-Wno-error=unused-variable', '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections', '-lm')
    Invoke-HostCase 'eod_route34_no_qr_test' @('tests/route34_no_qr_test.c') @('tests/stubs', 'App') @('-Wno-error=unused-variable', '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections', '-lm')
    Invoke-HostCase 'eod_bucket36_route_test' @('tests/bucket36_route_test.c') @('tests/stubs', 'App') @('-Wno-error=unused-variable', '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections', '-lm')
    Invoke-HostCase 'eod_cross37_sequence_test' @('tests/cross37_sequence_test.c') @('tests/stubs', 'App') @('-Wno-error=unused-variable', '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections', '-lm')
    Invoke-HostCase 'eod_route36_no_bucket_test' @('tests/route36_no_bucket_test.c') @('tests/stubs', 'App') @('-Wno-error=unused-variable', '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections', '-lm')
    Invoke-HostCase 'eod_route31_bucket_integration_test' @('tests/route31_bucket_integration_test.c') @('tests/stubs', 'App') @('-Wno-error=unused-variable', '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections', '-lm')
    Invoke-HostCase 'eod_target35_trial_test' @('tests/target35_trial_test.c') @('tests/stubs', 'App') @('-Wno-error=unused-variable', '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections', '-lm')
    Invoke-HostCase 'eod_target35_parser_replay_test' @('tests/target35_parser_replay_test.c') @('tests/stubs', 'App') @('-Wno-error=unused-variable', '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections', '-lm')
    Invoke-HostCase 'eod_mission_departure_route_test' @('tests/mission_departure_route_test.c') @('tests/stubs', 'App') @('-lm')
    Invoke-HostCase 'eod_mission_trial_plan_test' @('tests/mission_trial_plan_test.c', 'App/mission_trial_plan.c') @('App') @('-lm')
    Invoke-HostCase 'eod_mission_trial_flow_test' @('tests/mission_trial_flow_test.c', 'App/mission_trial.c', 'App/mission_trial_plan.c') @('tests/stubs', 'App') @('-lm')
    Invoke-HostCase 'eod_proto_frame_test' @('tests/proto_frame_test.c', 'App/proto.c') @('tests/stubs', 'App')
    Invoke-HostCase 'eod_vision_task_select_test' @('tests/vision_task_select_test.c', 'App/proto.c') @('tests/stubs', 'App')
    Invoke-HostCase 'eod_uart_error_rearm_test' @('tests/uart_error_rearm_test.c') @('tests/uart_stubs', 'App') @('-ffunction-sections', '-fdata-sections', '-fno-asynchronous-unwind-tables', '-fno-unwind-tables', '-Wl,--gc-sections')
    Invoke-HostCase 'eod_vision_target_slot_test' @('tests/vision_target_slot_test.c') @('tests/stubs', 'App') @('-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections', '-lm')
    Invoke-HostCase 'eod_vision_scene_wait_test' @('tests/vision_scene_wait_test.c') @('tests/stubs', 'App') @('-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections', '-lm')
    Invoke-HostCase 'eod_rotate_continuous_test' @('tests/rotate_continuous_test.c', 'App/steps.c') @('tests/stubs', 'App') @('-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections', '-lm')
    Invoke-HostCase 'eod_align_timeout_test' @('tests/align_timeout_test.c', 'App/steps.c') @('tests/stubs', 'App') @('-DVISION_CX_FWD_SIGN=1', '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections', '-lm')
    Invoke-HostCase 'eod_grab_xy_alignment_test' @('tests/grab_xy_alignment_test.c') @('tests/stubs', 'App') @('-ffunction-sections', '-fdata-sections', '-fno-asynchronous-unwind-tables', '-fno-unwind-tables', '-Wl,--gc-sections', '-lm')
    Invoke-HostCase 'eod_align_gate_test' @('tests/align_boundary_test.c') @('tests/stubs', 'App') @('-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections', '-lm')
    foreach ($pixelSign in @(1, -1)) {
        Invoke-HostCase "eod_align_left_axis_$pixelSign" @('tests/align_boundary_test.c') @('tests/stubs', 'App') @("-DVISION_CX_FWD_SIGN=$pixelSign", '-DSWEEP_FWD_MMS=100.0f', '-DSWEEP_BALL_DELTA_MM=10.0f', '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections', '-lm')
    }
    Invoke-HostPythonCase 'vision camera main loop' 'tests/test_vision_control_main.py'
    Invoke-HostPythonCase 'full binary packet replay' 'tests/test_vision_binary_replay.py'
    Invoke-HostPythonCase 'current QR53 and four selected tasks' 'tests/test_vision_qr53_replay.py'
    Invoke-HostPythonCase 'receive-only vision IRQ/stop contract' 'tests/test_vision_diag_contract.py'

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
    $gBody = [regex]::Match($test, 'if \(strcmp\(buf, "g"\) == 0\)\s*\{(?<body>.*?)\n    \}', 'Singleline').Groups['body'].Value
    $gStop = $gBody.IndexOf('if (s_go || mission_state() != MS_BOOT) { cmd_abort(); return; }')
    if ($gStop -lt 0 -or $gStop -ge $gBody.IndexOf('if (s_msel != R_FREE)') -or
        $gStop -ge $gBody.IndexOf('if (!bench_ok())') -or $gBody.Contains('run_reset();')) {
        throw 'g-stop source contract: pending/running mission stop must precede bench selection/gate; never reset abort'
    }
    Write-Output 'g-stop source contract: stop precedes mode/BOOT guards; abort is never cleared'

    # The checks above deliberately cover the complete start/abort dispatcher.
    # The legacy route contracts below start after the independent mode32 branch;
    # its earlier DONE must not be mistaken for the formal route's terminal state.
    $formalStart = $main.IndexOf('s_qr_ok = 0;')
    if ($formalStart -lt 0) { throw 'Formal route source contract: entry not found' }
    $main = $main.Substring($formalStart)

    $qrGate = $main.IndexOf('if (!s_qr_ok) goto failed;')
    $leftTurn = $main.IndexOf('if (!route_pre_cross_turn()) goto failed;')
    $preCross = $main.IndexOf('route_straight(R_PRE_CROSS_FWD_MM')
    $cross = $main.IndexOf('step_cross_obstacle(CROSS_V_MMS')
    if ($qrGate -lt 0 -or $leftTurn -le $qrGate -or $preCross -le $leftTurn -or $cross -le $preCross) {
        throw 'Departure route contract: valid QR -> left90 -> forward approach -> obstacle'
    }
    $routeSelect = [regex]::Match($test, 'static void cmd_route_leg\(int32_t leg\)\s*\{(?<body>.*?)\n\}', 'Singleline').Groups['body'].Value
    if (-not $routeSelect.Contains('cmd_select(leg == 1 ? 17 : (leg == 2 ? 16 : 15), 1);') -or
        -not $routeSelect.Contains('s_d = -1.0f;') -or
        $routeSelect.Contains('step_rotate_deg(')) {
        throw 'Departure bench contract: r1 left, r2 back, r3 forward only; explicit distance required'
    }
    Write-Output 'departure source contracts: QR/left90/approach/cross order and r1/r2/r3 selection checked'

    $entrySequence = @(
        'if (!route_straight(R_CROSS_REST_FWD_MM, ROUTE_FWD_V_MMS)) goto failed;',
        'if (!route_strafe(-R_CROSS_EXIT_LEFT_MM, -ROUTE_STRAFE_V_MMS)) goto failed;',
        'if (!route_straight(R_CROSS_EXIT_FWD_MM, ROUTE_FWD_V_MMS)) goto failed;',
        'if (!step_nav_leg(R_EOD_ENTRY_RIGHT_TURN_DEG, 0.0f, 0.0f, 0)) goto failed;',
        'if (!route_straight(R_EOD_ENTRY_FWD_MM, ROUTE_FWD_V_MMS)) goto failed;',
        'to_state(MS_EOD);'
    )
    $previousEntryIndex = $cross
    foreach ($entryAction in $entrySequence) {
        $entryIndex = $main.IndexOf($entryAction)
        if ($entryIndex -le $previousEntryIndex) {
            throw 'Task approach contract: finish third road -> left strafe -> forward -> right90 -> forward entry -> EOD'
        }
        $previousEntryIndex = $entryIndex
    }
    if ($mission.Contains('R_EOD_ENTRY_RIGHT_MM') -or
        $mission -notmatch '(?m)^#define R_EOD_ENTRY_RIGHT_TURN_DEG\s+90\.0f\s' -or
        $mission -notmatch '(?m)^#define R_EOD_ENTRY_FWD_MM\s+0\.0f\s' -or
        -not $mission.Contains('if (R_EOD_ENTRY_FWD_MM <= 0.0f) return "R_EOD_ENTRY_FWD_MM";')) {
        throw 'Task approach contract: right turn is 90 degrees, not a lateral distance'
    }
    Write-Output 'task approach source contract: full third road / left / forward / right90 / forward entry; zero distance gated'

    $eodBasis = $main.IndexOf('eod_entry_fwd_odo = motion_odo_mm();')
    $eodTask = $main.IndexOf('if (task_eod_run(s_targets.ball_color) != TASK_OK) goto failed;')
    $antiTravel = $main.IndexOf('if (!route_straight_to(eod_entry_fwd_odo + R_EOD_TO_ANTI_FWD_MM, ROUTE_FWD_V_MMS)) goto failed;')
    if ($eodBasis -le $main.IndexOf('if (!route_straight(R_EOD_ENTRY_FWD_MM') -or
        $eodBasis -ge $previousEntryIndex -or $eodTask -le $previousEntryIndex -or
        $antiTravel -le $eodTask -or $main.IndexOf('to_state(MS_ANTI);') -le $antiTravel -or
        $mission.Contains('R_EOD_TO_ANTI_RIGHT_MM') -or
        $mission -notmatch '(?m)^#define R_EOD_TO_ANTI_FWD_MM\s+0\.0f\s' -or
        -not $mission.Contains('if (R_EOD_TO_ANTI_FWD_MM <= 0.0f) return "R_EOD_TO_ANTI_FWD_MM";')) {
        throw 'EOD-to-ANTI contract: fore basis -> EOD -> fore remaining distance -> ANTI; zero distance gated'
    }
    Write-Output 'EOD-to-ANTI source contract: forward basis and remaining-distance call; zero distance gated'

    $antiExitSequence = @(
        'anti_entry_fwd_odo = motion_odo_mm();',
        'to_state(MS_ANTI);',
        'if (task_anti_run(s_targets.target_color) != TASK_OK) goto failed;',
        'if (!route_straight_to(anti_entry_fwd_odo + R_ANTI_EXIT_FWD_MM, ROUTE_FWD_V_MMS)) goto failed;',
        'if (!step_nav_leg(R_RESCUE_RIGHT_TURN_DEG, 0.0f, 0.0f, 0)) goto failed;'
    )
    $previousAntiIndex = $antiTravel
    foreach ($antiAction in $antiExitSequence) {
        $antiIndex = $main.IndexOf($antiAction)
        if ($antiIndex -le $previousAntiIndex) {
            throw 'ANTI exit contract: fore entry basis -> task -> remaining forward distance -> right90'
        }
        $previousAntiIndex = $antiIndex
    }
    if ($mission.Contains('R_ANTI_UP_MM') -or
        $mission -notmatch '(?m)^#define R_ANTI_EXIT_FWD_MM\s+0\.0f\s' -or
        -not $mission.Contains('if (R_ANTI_EXIT_FWD_MM <= 0.0f) return "R_ANTI_EXIT_FWD_MM";') -or
        $mission -notmatch '(?m)^#define R_RESCUE_RIGHT_TURN_DEG\s+90\.0f\s' -or
        -not $mission.Contains('if (R_RESCUE_RIGHT_TURN_DEG != 90.0f) return "R_RESCUE_RIGHT_TURN_DEG";')) {
        throw 'ANTI exit contract: pending forward distance gated; right turn is exactly 90 degrees'
    }
    Write-Output 'ANTI exit source contract: fore basis / task / forward remainder / right90 ordered and gated'

    $rescueSequence = @(
        'if (!route_straight(R_RESCUE_ENTRY_FWD_MM, ROUTE_FWD_V_MMS)) goto failed;',
        'rescue_entry_fwd_odo = motion_odo_mm();',
        'to_state(MS_RESCUE);',
        'if (task_rescue_run(s_targets.hostage_shape) != TASK_OK) goto failed;',
        'if (!route_straight_to(rescue_entry_fwd_odo + R_RESCUE_TO_HOME_FWD_MM, ROUTE_FWD_V_MMS)) goto failed;',
        'to_state(run_aborted() ? MS_ABORT : MS_DONE);'
    )
    $previousRescueIndex = $previousAntiIndex
    foreach ($rescueAction in $rescueSequence) {
        $rescueIndex = $main.IndexOf($rescueAction)
        if ($rescueIndex -le $previousRescueIndex) {
            throw 'RESCUE route contract: right90 -> forward entry -> new fore basis -> task -> forward home -> DONE'
        }
        $previousRescueIndex = $rescueIndex
    }
    foreach ($param in @('R_RESCUE_ENTRY_FWD_MM', 'R_RESCUE_TO_HOME_FWD_MM')) {
        if ($mission -notmatch "(?m)^#define $param\s+0\.0f\s" -or
            -not $mission.Contains("if ($param <= 0.0f) return `"$param`";")) {
            throw "RESCUE route contract: $param must remain zero and gated"
        }
    }
    $rescueTail = $main.Substring($main.IndexOf('to_state(MS_RESCUE);'))
    if ($mission.Contains('R_RESCUE_ENTRY_MM') -or $mission.Contains('R_HOME_DIST_MM') -or
        $mission.Contains('route_strafe_to(') -or $rescueTail.Contains('step_nav_leg(') -or
        $rescueTail.Contains('step_rotate_deg(') -or $rescueTail.Contains('route_strafe(')) {
        throw 'RESCUE route contract: no stale lateral route or extra turn after rescue'
    }
    Write-Output 'RESCUE route source contract: forward entry / new fore basis / task / forward home; no extra turn; zero distances gated'

    $routeCalls = $mission -split "`n" | Where-Object { $_ -match '^\s*if \(!route_(straight|strafe)(_to)?\(' }
    $straight = @($routeCalls | Where-Object { $_ -match 'route_straight(_to)?\(' })
    $strafe = @($routeCalls | Where-Object { $_ -match 'route_strafe(_to)?\(' })
    if ($straight.Count -ne 8 -or @($straight | Where-Object { $_ -notmatch 'ROUTE_FWD_V_MMS' }).Count -ne 0) {
        throw 'Route speed contract: every forward leg must use ROUTE_FWD_V_MMS'
    }
    if ($strafe.Count -ne 1 -or @($strafe | Where-Object { $_ -notmatch 'ROUTE_STRAFE_V_MMS' }).Count -ne 0) {
        throw 'Route speed contract: every lateral leg must use ROUTE_STRAFE_V_MMS'
    }
    if ($mission -notmatch 'if \(ROUTE_FWD_V_MMS <= 0\.0f\)' -or
        $mission -notmatch 'if \(ROUTE_STRAFE_V_MMS <= 0\.0f\)') {
        throw 'Route speed contract: both speed gates are required'
    }
    Write-Output 'route speed source contract: 8 forward + 1 lateral calls, 2 gates checked'

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
    $env:EOD_HOST_CC = $previousHostCompiler
    Pop-Location
}
