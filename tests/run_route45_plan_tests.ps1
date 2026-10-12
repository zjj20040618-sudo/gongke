# Preparation-only planner replay: does not build/overwrite Keil or register45.
$ErrorActionPreference = 'Stop'
$taskProjectRoot = Split-Path -Parent $PSScriptRoot
$taskCompiler = 'D:/mingw64/bin/gcc.exe'
if (-not (Test-Path -LiteralPath $taskCompiler)) { throw "GCC not found: $taskCompiler" }
$taskBuildId = [Guid]::NewGuid().ToString('N')
Push-Location $taskProjectRoot
try {
    foreach ($taskVariant in @('default', 'optimized')) {
        $taskOutput = Join-Path ([IO.Path]::GetTempPath()) "route45-plan-$taskVariant-$taskBuildId.exe"
        $taskArguments = @('-std=c11', '-Wall', '-Wextra', '-Werror', '-IApp')
        if ($taskVariant -eq 'optimized') { $taskArguments += '-O2' }
        $taskArguments += @('tests/route45_plan_test.c', 'App/route45_plan.c', 'App/mission_trial_plan.c', '-lm', '-o', $taskOutput)
        & $taskCompiler @taskArguments
        if ($LASTEXITCODE -ne 0) { throw "route45 $taskVariant compilation failed" }
        & $taskOutput
        if ($LASTEXITCODE -ne 0) { throw "route45 $taskVariant planner replay failed" }
    }
    Write-Output 'Route45 isolated planner replay passed; no runtime/Keil/flash/physical acceptance.'
} finally {
    Pop-Location
}
