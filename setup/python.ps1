# python.ps1 - host/requirements.txt at the versions host/constraints.txt pins, the editable install of host/,
# the notebook kernel.

function Install-PythonDeps {
    param([string]$Python)
    Write-Head 'python packages'
    $script:Packages = $false
    if (-not $Python) { return }   # [string] makes $null ''

    $requirements = Join-Path $Host_ 'requirements.txt'
    if (-not (Test-Path $requirements)) {
        Write-Item 'requirements.txt' 'failed' $requirements
        return
    }

    # The list to probe comes from requirements.txt itself, never a copy here:
    # a copy drifted once - numpy and rich joined the file and this probe kept
    # saying 'all present' on a machine that had neither. The probe is
    # host/tools/dev/depends.py's, the lines the pre-commit runs: each distribution
    # asked of the interpreter's metadata by the name the file writes - a module
    # name guessed from it said pillow and fonttools missing where both stood -,
    # the standard library alone, so a python with no package runs it.
    $probe = @('-X', 'utf8', 'tools/dev/depends.py', '--absent')
    # One version of each distribution on every machine and on CI (host/constraints.txt, tools/dev/pinned.py): the
    # probe asks the requirements at the versions it pins and the distributions here it pins at another, and pip
    # installs through it.
    $constraints = Join-Path $Host_ 'constraints.txt'
    $pinned, $through = @(), @()
    if (Test-Path $constraints) { $pinned, $through = @('--pinned', $constraints), @('-c', $constraints) }
    $absent = Invoke-Host -Python $Python -Arguments ($probe + $requirements + $pinned)
    $probed = ($LASTEXITCODE -eq 0)

    $present = $probed -and [string]::IsNullOrWhiteSpace($absent)
    if (-not $probed) {
        Write-Item 'requirements' 'failed' $absent
        Add-Todo 'python tools/dev/depends.py --absent requirements.txt failed - run it in host/ and read the output'
    } elseif ($present) {
        Write-Item 'requirements' 'ok' 'all present'
    } else {
        Write-Item 'requirements' 'missing' $absent
        # Beside them what the package's own build asks (pyproject's build-system), as CI's environment and the Linux
        # containers' hold it: the lock pins it too.
        $build = @(Invoke-Python -Python $Python -Code @'
import tomllib
print('\n'.join(tomllib.load(open(r'HOST_PATH\pyproject.toml', 'rb'))['build-system']['requires']))
'@.Replace('HOST_PATH', $Host_) | Where-Object { $_ })
        if (Confirm-Step "pip install -r host/requirements.txt $($build -join ' ') -c host/constraints.txt ?") {
            & $Python -m pip install --disable-pip-version-check -r $requirements @build @through
            if ($LASTEXITCODE -eq 0) {
                Write-Item 'requirements' 'done' 'installed'
                $present = $true
            } else {
                Write-Item 'requirements' 'failed' "pip exit $LASTEXITCODE"
                Add-Todo "pip install -r host/requirements.txt failed - read the output above"
            }
        } else {
            Add-Todo "python -m pip install -r host/requirements.txt -c host/constraints.txt"
        }
    }

    # Her eyes on a CUDA card (host/requirements-gpu.txt) are asked where a card's driver stands and
    # never installed unasked: gigabytes a bench PC does not use.
    $gpu = Join-Path $Host_ 'requirements-gpu.txt'
    if ($present -and (Test-Path $gpu) -and ($null -ne (Get-Tool 'nvidia-smi'))) {
        $lacks = Invoke-Host -Python $Python -Arguments ($probe + $gpu)
        if (($LASTEXITCODE -eq 0) -and [string]::IsNullOrWhiteSpace($lacks)) {
            Write-Item 'requirements-gpu' 'ok' 'all present'
        } else {
            Write-Item 'requirements-gpu' 'missing' ('optional - her eyes on the card: ' + $lacks)
            Add-Todo -Optional 'python -m pip install -r host/requirements-gpu.txt -c host/constraints.txt   (her eyes on the CUDA card here)'
        }
    }

    # Required: every script, test and view imports the installed packages
    # (nothing sets sys.path); pyproject also hands out `coaxial`, `coaxial-dbg`
    # and `coaxial-mcp` as commands.
    $installed = Invoke-Python -Python $Python -Code @'
try:
    from importlib.metadata import version
    print(version('coaxial63100'))
except Exception:
    print('')
'@
    # An editable install maps the packages pyproject listed when it ran: one
    # added since (machine, motor, terminal, tools) needs it again.
    $stale = Invoke-Python -Python $Python -Code @'
import fnmatch, importlib.util as util, pathlib, re
host = pathlib.Path(r'HOST_PATH').resolve()
git = host.parent / '.git'
roots = {host}
if git.is_file():
    roots.add((host.parent / git.read_text(encoding='utf-8').split(':', 1)[1].strip()).resolve().parents[2] / 'host')
text = (host / 'pyproject.toml').read_text(encoding='utf-8')
include = re.findall(r'"([^"]+)"', re.search(r'^include\s*=\s*\[([^\]]*)\]', text, re.M).group(1))
stale = []
for d in sorted(p for p in host.iterdir() if p.is_dir() and any(p.glob('*.py'))):
    if any(fnmatch.fnmatch(d.name, p) for p in include):
        spec = util.find_spec(d.name)
        where = [pathlib.Path(p).resolve() for p in (spec.submodule_search_locations or [])] if spec else []
        if not any((r / d.name) in where for r in roots):
            stale.append(d.name)
print(','.join(stale))
'@.Replace('HOST_PATH', $Host_)
    $editable = (-not [string]::IsNullOrWhiteSpace($installed)) -and [string]::IsNullOrWhiteSpace($stale)
    if ($editable) {
        Write-Item 'pip install -e host/' 'ok' ('coaxial63100 ' + $installed)
    } else {
        $why = 'required: scripts, tests and views import it'
        if (-not [string]::IsNullOrWhiteSpace($installed)) { $why = 'stale: ' + $stale + ' not on the path' }
        Write-Item 'pip install -e host/' 'missing' $why
        if (Confirm-Step 'pip install -e host/ ?  (editable: the checkout stays the source)') {
            & $Python -m pip install --disable-pip-version-check --no-deps -e $Host_
            if ($LASTEXITCODE -eq 0) {
                Write-Item 'pip install -e host/' 'done' 'installed'
                $editable = $true
            } else {
                Write-Item 'pip install -e host/' 'failed' "pip exit $LASTEXITCODE"
                Add-Todo 'pip install -e host/ failed - read the output above'
            }
        } else {
            Add-Todo 'python -m pip install -e host/'
        }
    }
    # The host suite and the model probe import host/'s packages: before them they wait.
    $script:Packages = $present -and $editable

    # The notebooks name their kernel, `coaxial_63100`, registered on this
    # interpreter by make_notebooks.py, so an editor holding two CPythons of
    # the same version opens them on the one the packages are in.
    $maker = @('-m', 'tools.notebooks.make_notebooks', '--kernel')
    $kernel = Invoke-Host -Python $Python -Arguments ($maker + 'status')
    if ($LASTEXITCODE -eq 0) {
        Write-Item 'notebook kernel' 'ok' $kernel
    } elseif ($script:Packages -and ($kernel -match '^\w+(Error|Exception)\b')) {
        Write-Item 'notebook kernel' 'failed' $kernel                 # a crash, not a kernel missing
        Add-Todo 'python -m tools.notebooks.make_notebooks --kernel status failed - run it in host/'
    } else {
        Write-Item 'notebook kernel' 'missing' $kernel
        if (Confirm-Step 'register the notebook kernel on this python ?  (one kernel.json under %APPDATA%\jupyter)') {
            $kernel = Invoke-Host -Python $Python -Arguments ($maker + 'install')
            if ($LASTEXITCODE -eq 0) {
                Write-Item 'notebook kernel' 'done' $kernel
            } else {
                Write-Item 'notebook kernel' 'failed' $kernel
                Add-Todo 'python -m tools.notebooks.make_notebooks --kernel install failed - run it in host/'
            }
        } else {
            Add-Todo 'python -m tools.notebooks.make_notebooks --kernel install   (in host/)'
        }
    }
}
