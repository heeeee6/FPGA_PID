$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
# Keep Vivado's generated simulation database in the user temp directory.  A
# checked-out project may be read-only (for example when it lives on D:), and
# XSim needs write access to .Xil and xsim.dir while compiling.
$runDir = Join-Path ([System.IO.Path]::GetTempPath()) 'pid_swing_sim_check'
$rtlFiles = @(Get-ChildItem (Join-Path $projectRoot 'rtl') -Filter '*.v' -File | Sort-Object Name | ForEach-Object FullName)
$simFiles = @(Get-ChildItem (Join-Path $projectRoot 'sim') -Filter '*.sv' -File | Sort-Object Name | ForEach-Object FullName)
$testbenches = @(Get-ChildItem (Join-Path $projectRoot 'sim') -Filter 'tb_*.sv' -File | Sort-Object Name | ForEach-Object BaseName)

if ($rtlFiles.Count -eq 0 -or $testbenches.Count -eq 0) {
    throw 'RTL or testbench files are missing.'
}

$xvlog = (Get-Command xvlog.bat -ErrorAction Stop).Source
$xelab = (Get-Command xelab.bat -ErrorAction Stop).Source
$xsim = (Get-Command xsim.bat -ErrorAction Stop).Source
New-Item -ItemType Directory -Force -Path $runDir | Out-Null

Push-Location $runDir
try {
    & $xvlog --sv --nolog @rtlFiles @simFiles
    if ($LASTEXITCODE -ne 0) { throw 'Vivado HDL compilation failed.' }

    foreach ($top in $testbenches) {
        $snapshot = "${top}_snapshot"
        & $xelab --nolog "work.$top" -s $snapshot
        if ($LASTEXITCODE -ne 0) { throw "Elaboration failed for $top." }

        $simOutput = & $xsim $snapshot --runall --nolog --onerror quit --onfinish quit 2>&1
        $simOutput | Write-Output
        $expectedPass = "${top}: PASS"
        if ($LASTEXITCODE -ne 0 -or (($simOutput -join "`n") -notmatch [regex]::Escape($expectedPass))) {
            throw "Simulation failed for $top."
        }
    }
} finally {
    Pop-Location
}
