# Focused current31/43/44 software replay; never flashes or drives hardware.
# Historical run_host_tests.ps1 fixtures also cover older frozen tuning snapshots.
$ErrorActionPreference = 'Stop'
$taskProjectRoot = Split-Path -Parent $PSScriptRoot
$taskCompiler = 'D:/mingw64/bin/gcc.exe'
if (-not (Test-Path -LiteralPath $taskCompiler)) { throw "GCC not found: $taskCompiler" }
$taskBuildId = [Guid]::NewGuid().ToString('N')
Push-Location $taskProjectRoot
try {
    $taskRtos = Get-Content -LiteralPath 'Src/freertos.c' -Raw -Encoding utf8
    $taskIoc = Get-Content -LiteralPath 'jiejie.ioc' -Raw -Encoding utf8
    $taskAttr = [regex]::Match($taskRtos, 'defaultTask_attributes\s*=\s*\{(?<body>.*?)\};', 'Singleline').Groups['body'].Value
    $taskStack = [regex]::Match($taskAttr, '\.stack_size\s*=\s*(?<words>\d+)\s*\*\s*4')
    $taskIocStack = [regex]::Match($taskIoc, 'defaultTask,24,(?<words>\d+),StartDefaultTask,')
    if (-not $taskStack.Success -or -not $taskIocStack.Success -or
        [int]$taskStack.Groups['words'].Value -lt 1024 -or
        $taskStack.Groups['words'].Value -ne $taskIocStack.Groups['words'].Value) {
        throw 'Current route stack contract: DefaultTask needs at least 4 KB and matching IOC words'
    }
    foreach ($taskCase in @('route31_slow_clearance_test', 'route43_current_step_test',
            'route31_revised_tail_test', 'route44_current_test')) {
        foreach ($taskVariant in @('default', 'optimized')) {
            $taskOutput = Join-Path ([IO.Path]::GetTempPath()) "$taskCase-$taskVariant-$taskBuildId.exe"
            $taskArguments = @('-std=c11', '-Wall', '-Wextra', '-Werror', '-Wno-error=unused-variable',
                '-Itests/stubs', '-IApp', '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections')
            if ($taskVariant -eq 'optimized') { $taskArguments += '-O2' }
            $taskArguments += "tests/$taskCase.c"
            if ($taskCase -in @('route31_slow_clearance_test', 'route43_current_step_test')) {
                $taskArguments += 'tests/stubs/motion_left_scaled_compat.c'
            }
            $taskArguments += @('-lm', '-o', $taskOutput)
            & $taskCompiler @taskArguments
            if ($LASTEXITCODE -ne 0) { throw "$taskCase $taskVariant compilation failed" }
            & $taskOutput
            if ($LASTEXITCODE -ne 0) { throw "$taskCase $taskVariant failed" }
        }
    }
    Write-Output 'Current31/43/44 focused replay and RTOS/IOC stack contract passed; not physical acceptance.'
} finally {
    Pop-Location
}
