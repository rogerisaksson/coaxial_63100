<#
.SYNOPSIS
    The 3D engine's demo and bench: watch it render live, or hold it against
    its oracle and the CAD exporter's references.
.DESCRIPTION
    With no switches this opens the live console: the exporter's cube (or
    the board, -Model board) rendered by the staged engine, steered by hand
    - x/y/z step the pose, SPACE spins, M swaps model, the wheel zooms, Q
    leaves.
.EXAMPLE
    .\render_demo.ps1                # the live console, cube
#>
param(
    [switch]$Test,
    [switch]$Calibration,
    [switch]$Show,
    [string]$Pose = 'x45y45z45',
    [string]$Model = 'cube',
    [switch]$Fit
)

$root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$hostDir = Join-Path $root 'host'
$code = 0

if (-not ($Test -or $Calibration -or $Show -or $Fit)) {
    python -X utf8 (Join-Path $hostDir 'terminal\views\show_render.py') `
        --model $Model
    exit $LASTEXITCODE
}

if ($Test) {
    python -X utf8 (Join-Path $hostDir 'tests\test_render.py')
    $code = $LASTEXITCODE
}

if ($Test -or $Calibration) {
    Write-Host ''
    Write-Host '-- calibration against the exporter''s renders --------------'
    python -X utf8 (Join-Path $hostDir 'tools\render\facecheck.py')
}

if ($Show) {
    Write-Host ''
    python -X utf8 (Join-Path $hostDir 'tools\render\rendershow.py') `
        --model $Model --pose $Pose
}

if ($Fit) {
    Write-Host ''
    Write-Host '-- fitting the shading constants ---------------------------'
    python -X utf8 (Join-Path $hostDir 'tools\render\lightfit.py')
}

exit $code
