<#
.SYNOPSIS
    The 3D engine's demo and bench: watch it render live, or hold it
    against its oracle and the CAD exporter's references.

.DESCRIPTION
    With no switches this opens the live console: the exporter's cube
    (or the board, -Model board) rendered by the staged engine, steered
    by hand - x/y/z step the pose, SPACE spins, M swaps model, the
    wheel zooms, Q leaves.

    The bench behind it:

      -Test         host/tests/test_render.py - every engine stage
                    against exact expectations and the full chain
                    against an analytic ray-cast oracle; the exit code.
      -Calibration  host/tools/facecheck.py - silhouette and light
                    agreement against the exporter's reference renders.
                    Informational: their pose conventions are only
                    partly known.
      -Show         a still: engine, oracle and exporter side by side.
      -Fit          tools/lightfit.py - refits the shading constants
                    against the references and prints what to bake into
                    coaxial/wireframe.py. Changes nothing on disk.

    -Pose x30y0z0 and -Model cube|board steer -Show and the console.

.EXAMPLE
    .\render_demo.ps1                # the live console, cube
    .\render_demo.ps1 -Model board   # the live console, board
    .\render_demo.ps1 -Test          # the gate, then the table
    .\render_demo.ps1 -Show -Pose x30y0z0
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
    python -X utf8 (Join-Path $hostDir 'tools\show_render.py') `
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
    python -X utf8 (Join-Path $hostDir 'tools\facecheck.py')
}

if ($Show) {
    Write-Host ''
    python -X utf8 (Join-Path $hostDir 'tools\rendershow.py') `
        --model $Model --pose $Pose
}

if ($Fit) {
    Write-Host ''
    Write-Host '-- fitting the shading constants ---------------------------'
    python -X utf8 (Join-Path $hostDir 'tools\lightfit.py')
}

exit $code
