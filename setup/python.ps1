# python.ps1 - host/requirements.txt, the editable install of host/, the notebook kernel.

function Install-PythonDeps {
    param([string]$Python)
    Write-Head 'python packages'
    if ($null -eq $Python) { return }

    $requirements = Join-Path $Host_ 'requirements.txt'
    if (-not (Test-Path $requirements)) {
        Write-Item 'requirements.txt' 'failed' $requirements
        return
    }

    # The list to probe comes from requirements.txt itself, never a copy here:
    # a copy drifted once - numpy and rich joined the file and this probe kept
    # saying 'all present' on a machine that had neither.
    $absent = Invoke-Python -Python $Python -Code @'
import importlib.util as util
import re

RENAMED = {'pyserial': 'serial', 'pyyaml': 'yaml'}
missing = []
for line in open(r'REQUIREMENTS_PATH', encoding='utf-8'):
    line = line.split('#')[0].strip()
    if not line:
        continue
    dist = re.split(r'[<>=!~\[ ]', line)[0]
    module = RENAMED.get(dist.lower(), dist.lower().replace('-', '_'))
    if util.find_spec(module) is None:
        missing.append(dist)
print(','.join(missing))
'@.Replace('REQUIREMENTS_PATH', $requirements)

    if ([string]::IsNullOrWhiteSpace($absent)) {
        Write-Item 'requirements' 'ok' 'all present'
    } else {
        Write-Item 'requirements' 'missing' $absent
        if (Confirm-Step "pip install -r host/requirements.txt ?") {
            & $Python -m pip install --disable-pip-version-check -r $requirements
            if ($LASTEXITCODE -eq 0) {
                Write-Item 'requirements' 'done' 'installed'
            } else {
                Write-Item 'requirements' 'failed' "pip exit $LASTEXITCODE"
                Add-Todo "pip install -r host/requirements.txt failed - read the output above"
            }
        } else {
            Add-Todo "python -m pip install -r host/requirements.txt"
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
    # added since (machine, motor, terminal, tools on 2026-09-24) needs it again.
    $stale = Invoke-Python -Python $Python -Code @'
import fnmatch, importlib.util as util, pathlib, re
host = pathlib.Path(r'HOST_PATH').resolve()
text = (host / 'pyproject.toml').read_text(encoding='utf-8')
include = re.findall(r'"([^"]+)"', re.search(r'^include\s*=\s*\[([^\]]*)\]', text, re.M).group(1))
stale = []
for d in sorted(p for p in host.iterdir() if p.is_dir() and any(p.glob('*.py'))):
    if any(fnmatch.fnmatch(d.name, p) for p in include):
        spec = util.find_spec(d.name)
        where = [pathlib.Path(p).resolve() for p in (spec.submodule_search_locations or [])] if spec else []
        if d not in where:
            stale.append(d.name)
print(','.join(stale))
'@.Replace('HOST_PATH', $Host_)
    if ((-not [string]::IsNullOrWhiteSpace($installed)) -and [string]::IsNullOrWhiteSpace($stale)) {
        Write-Item 'pip install -e host/' 'ok' ('coaxial63100 ' + $installed)
    } else {
        $why = 'required: scripts, tests and views import it'
        if (-not [string]::IsNullOrWhiteSpace($installed)) { $why = 'stale: ' + $stale + ' not on the path' }
        Write-Item 'pip install -e host/' 'missing' $why
        if (Confirm-Step 'pip install -e host/ ?  (editable: the checkout stays the source)') {
            & $Python -m pip install --disable-pip-version-check -e $Host_
            if ($LASTEXITCODE -eq 0) {
                Write-Item 'pip install -e host/' 'done' 'installed'
            } else {
                Write-Item 'pip install -e host/' 'failed' "pip exit $LASTEXITCODE"
                Add-Todo 'pip install -e host/ failed - read the output above'
            }
        } else {
            Add-Todo 'python -m pip install -e host/'
        }
    }

    # The notebooks name their kernel, `coaxial_63100`, registered on this
    # interpreter by make_notebooks.py, so an editor holding two CPythons of
    # the same version opens them on the one the packages are in.
    $maker = Join-Path $Host_ 'tools\notebooks\make_notebooks.py'
    $kernel = (& $Python $maker --kernel status) -join ' '
    if ($LASTEXITCODE -eq 0) {
        Write-Item 'notebook kernel' 'ok' $kernel
    } else {
        Write-Item 'notebook kernel' 'missing' $kernel
        if (Confirm-Step 'register the notebook kernel on this python ?  (one kernel.json under %APPDATA%\jupyter)') {
            $kernel = (& $Python $maker --kernel install) -join ' '
            if ($LASTEXITCODE -eq 0) {
                Write-Item 'notebook kernel' 'done' $kernel
            } else {
                Write-Item 'notebook kernel' 'failed' $kernel
                Add-Todo 'python tools/notebooks/make_notebooks.py --kernel install failed - run it from host/ and read the output'
            }
        } else {
            Add-Todo 'python tools/notebooks/make_notebooks.py --kernel install   (from host/)'
        }
    }
}
