# Verify every official input against its recorded Flash trace, using an already-built `hw` CLI.
# Each input is verified against its richest shipped trace: the listeners stream when present, else
# the per-frame hash diagnostic stream. Inputs with neither are listed as skipped. The repository ships
# only a subset of the listeners traces; -RequireAll fails unless every input has its listeners trace.
param(
    [string]$Executable = 'target/i686-pc-windows-msvc/release/hw.exe',
    [switch]$RequireAll
)
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
Push-Location $root
try {
    $exe = (Resolve-Path -LiteralPath $Executable).Path
    $cases = Get-Content -LiteralPath 'reference/tas/index.json' -Raw | ConvertFrom-Json
    $full = 0
    $diagnostic = 0
    $skipped = @()
    foreach ($case in $cases) {
        $listeners = $case.listeners_state_trace
        $diag = $case.diagnostic_trace
        if ($listeners -and (Test-Path -LiteralPath $listeners)) {
            $trace = $listeners; $level = 'listeners'
        } elseif ($diag -and (Test-Path -LiteralPath $diag)) {
            $trace = $diag; $level = 'diagnostic'
        } else {
            $skipped += $case.file; continue
        }
        Write-Output "$($case.file) [$level]"
        & $exe verify --trace $trace
        if ($LASTEXITCODE -ne 0) { throw "Verification failed for $($case.file) ($level trace)" }
        if ($level -eq 'listeners') { $full++ } else { $diagnostic++ }
    }
    foreach ($m in $skipped) { Write-Warning "SKIPPED (no trace present): $m" }
    Write-Output "PASS: $($full + $diagnostic) of $($cases.Count) official inputs verified ($full with the listeners stream, $diagnostic with the diagnostic stream), $($skipped.Count) skipped"
    if ($RequireAll -and ($diagnostic -gt 0 -or $skipped.Count -gt 0)) {
        throw "-RequireAll: $diagnostic inputs have only the diagnostic stream and $($skipped.Count) have no trace"
    }
} finally {
    Pop-Location
}
