#!/usr/bin/env python3
"""The pre-commit hook: a file past the token budget, or setup.ps1 broken on a new machine, stops it.

.githooks/pre-commit runs it; setup.ps1 points git at .githooks.

    python tools/dev/pre_commit.py         # what `git commit` runs; 1 stops it

The staged text is judged, not the working tree's: a file over tools/dev/token_budget.BUDGET,
or one of its HEAVY grown past its cap, is named with the way to split it; setup.ps1 -Check
runs as staged (STAGED checked out apart) on two new machines at once (machines()): its stderr,
a traceback, a failed line, an exit past 1 or no summary is a fault. Only a new machine meets
the paths without host/'s packages: two tracebacks stood there unseen (2026-09-29).
"""
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
from concurrent.futures import ThreadPoolExecutor

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools import REPO  # noqa: E402
from tools.dev import token_budget  # noqa: E402

#: How to take a file of each kind under the budget.
SPLIT = {
    'tests': 'split it by subject: python tools/dev/split_suite.py plan.json - its tests into '
             'files, its helpers where they are used',
    '.py': 'move a subject to its own module, every user repointed (no shim)',
    '.c': 'move a step and its state to their own file, the header its API',
    '.h': 'move a subject\'s structs and prototypes to their own header',
    '.md': 'keep what holds now, one line a finding; move a subject to its own page',
}


def _git(*args):
    return subprocess.run(['git'] + list(args), capture_output=True, text=True,
                          encoding='utf-8', errors='replace').stdout


def staged():
    """(path, tokens) of every staged file the budget reads, as staged."""
    for path in _git('diff', '--cached', '--name-only', '--diff-filter=ACMR').splitlines():
        if path.endswith(token_budget.EXTENSIONS) and not any(
                part.lower() in token_budget.SKIP for part in path.split('/')[:-1]):
            text = _git('show', ':' + path)
            if token_budget.GENERATED in text.split('\n', 1)[0]:
                continue
            yield path, len(text) / 4.0


def how(path):
    """The way to split `path`."""
    if '/tests/' in '/' + path:
        return SPLIT['tests']
    return SPLIT.get(os.path.splitext(path)[1], SPLIT['.py'])


#: setup.ps1 as the hook runs it, report only; what it reads and runs, as staged; its time, s.
SETUP = ('powershell', '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File')
STAGED, SETUP_S = ('setup.ps1', 'setup', '.github', 'host'), 180

#: A PATH entry holding one of these hands setup a python: gone from both machines, as is every
#: entry under the real LOCALAPPDATA (a new machine has nothing installed per user).
PYTHONS = tuple(n + e for n in ('python', 'python3', 'py')
                for e in os.environ.get('PATHEXT', '.EXE').lower().split(';') + ['.ps1'] if e)


def gate():
    """[(major, minor)] CI runs host/ on, read as setup.ps1's Get-PythonGate reads them."""
    with open(os.path.join(REPO, '.github', 'workflows', 'host.yml'), encoding='utf-8') as f:
        m = re.search(r'''python:[ \t]*(\[[^\]]*\]|(\s*-[ \t]*['"]?\d+\.\d+['"]?)+)''', f.read())
    return sorted(tuple(map(int, v.split('.'))) for v in re.findall(r'\d+\.\d+', m[1] if m else ''))


def machines(scratch):
    """(name, the line its report must hold, environment) of each new machine setup meets.

    No python on PATH and an empty LOCALAPPDATA; a bare venv alone on PATH - python, none of
    host/'s packages, refused where this python is not one CI runs. A machine set up runs
    setup's other paths whenever setup runs; this one's took 12.8 s, 9.1 of them the host
    suite (2026-09-29).
    """
    bare = os.path.join(scratch, 'bare')
    subprocess.run([sys.executable, '-m', 'venv', '--without-pip', bare], check=True)
    empty = os.path.join(scratch, 'localappdata')
    os.mkdir(empty)
    local = os.path.normcase(os.environ.get('LOCALAPPDATA', '?')) + os.sep
    none = [d for d in os.environ['PATH'].split(os.pathsep) if d
            and not os.path.normcase(d).startswith(local)
            and not any(os.path.lexists(os.path.join(d, p)) for p in PYTHONS)]
    ci = gate()
    taken = bool(ci) and ci[0] <= sys.version_info[:2] <= ci[-1]
    return [
        ('no python', '  missing python ',
         dict(os.environ, PATH=os.pathsep.join(none), LOCALAPPDATA=empty)),
        ('a bare python', '  missing requirements ' if taken else '  missing python ',
         dict(os.environ, PATH=os.pathsep.join([os.path.join(bare, 'Scripts')] + none),
              LOCALAPPDATA=empty)),
    ]


def staged_tree(scratch):
    """STAGED checked out as staged under `scratch`: the root setup.ps1 runs from."""
    tree = os.path.join(scratch, 'tree')
    paths = subprocess.run(['git', 'ls-files', '-z', '--'] + list(STAGED), cwd=REPO,
                           capture_output=True, check=True).stdout
    subprocess.run(['git', 'checkout-index', '-z', '--stdin',
                    '--prefix=' + tree.replace(os.sep, '/') + '/'],
                   input=paths, cwd=REPO, capture_output=True, check=True)
    return tree


def _setup(tree, env):
    """One setup.ps1 -Check from `tree`: past SETUP_S its whole process tree is killed - a
    child still holding the pipes kept `subprocess.run`'s timeout from ever ending."""
    run = subprocess.Popen(SETUP + (os.path.join(tree, 'setup.ps1'), '-Check'), env=env, cwd=tree,
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, encoding='oem',
                           errors='replace', creationflags=subprocess.CREATE_NO_WINDOW)
    try:
        out, err = run.communicate(timeout=SETUP_S)
    except subprocess.TimeoutExpired:
        subprocess.run(['taskkill', '/F', '/T', '/PID', str(run.pid)], capture_output=True)
        out, err = run.communicate()
        err += 'did not finish in %d s' % SETUP_S
    return subprocess.CompletedProcess(run.args, run.returncode, out, err)


def faults(name, must, run):
    """What in one run says setup is broken, not that the machine is unfinished."""
    out = run.stdout.splitlines()
    found = ['stderr: ' + line for line in run.stderr.splitlines() if line.strip()][:12]
    found += [line.strip() for line in out if line.startswith('  failed ')]
    if any('Traceback (most recent call last)' in line for line in out):
        found.append('a traceback in its report')
    if run.returncode not in (0, 1):
        found.append('exit %d' % run.returncode)
    if not any(line.startswith('-- next ') for line in out):
        found.append('stopped before its summary: ' + ' | '.join(out[-3:]))
    if must and not any(line.startswith(must) for line in out):
        found.append('no "%s" line: setup did not meet %s' % (must.strip(), name))
    return ['%s: %s' % (name, f) for f in found]


def setup_faults():
    """(faults, s) of setup.ps1 -Check on each of MACHINES at once; none where no powershell is."""
    if os.name != 'nt' or shutil.which('powershell') is None:
        return [], 0.0
    t = time.perf_counter()
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as scratch:
        tree, runs = staged_tree(scratch), machines(scratch)
        with ThreadPoolExecutor(len(runs)) as pool:
            done = list(pool.map(lambda r: _setup(tree, r[2]), runs))
    return [f for r, d in zip(runs, done) for f in faults(r[0], r[1], d)], time.perf_counter() - t


def main():
    code = 0
    past = [(p, t, token_budget.HEAVY.get(p, token_budget.BUDGET)) for p, t in staged()
            if t > token_budget.HEAVY.get(p, token_budget.BUDGET)]
    if past:
        print('pre-commit: %d file%s past what a model reads whole - split before committing:'
              % (len(past), '' if len(past) == 1 else 's'))
        for path, tokens, cap in past:
            print('  %s  %.1f k tokens of %.1f k: %s' % (path, tokens / 1e3, cap / 1e3, how(path)))
        code = 1
    found, seconds = setup_faults()
    if found:
        print('pre-commit: setup.ps1 -Check broke - fix it before committing:')
        for fault in found:
            print('  ' + fault)
        code = 1
    elif seconds:
        print('pre-commit: setup.ps1 -Check clean with no python and a bare one (%.1f s)' % seconds)
    return code


if __name__ == '__main__':
    sys.exit(main())
