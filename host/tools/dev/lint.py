"""The editor's linters on the command line: markdownlint, and pyright as Pylance runs it.

markdownlint for .md; pyright in basic mode for .py and a notebook's code cells.

    python host/tools/dev/lint.py FILE ...      # findings, a line each; exit 1 on any
    python host/tools/dev/lint.py --changed     # every changed or new file git sees
    python host/tools/dev/lint.py --hook post   # Claude Code hooks: the payload on stdin
    python host/tools/dev/lint.py --hook stop

The tools live in %LOCALAPPDATA%/coaxial_63100/lint, fetched on first use: pyright from PyPI,
markdownlint-cli2 through npm, run by node from PATH or the one Autodesk Fusion ships.
One pyright at a time: a lock in that folder.
"""
import argparse
import glob
import json
import os
import re
import shutil
import subprocess
import sys
import tarfile
import tempfile
import time
import urllib.request

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
HOST = os.path.join(REPO, 'host')
CACHE = os.path.join(os.environ.get('LOCALAPPDATA') or os.path.expanduser('~/.cache'),
                     'coaxial_63100', 'lint')
KINDS = ('.py', '.md', '.ipynb')
NPM = 'https://registry.npmjs.org/npm/-/npm-10.9.2.tgz'
MDLINT = os.path.join(CACHE, 'mdlint', 'node_modules', 'markdownlint-cli2',
                      'markdownlint-cli2-bin.mjs')
PYRIGHT = os.path.join(CACHE, 'pyright')
LOCK_S = 30.0


def _node():
    found = shutil.which('node') or sorted(glob.glob(os.path.join(
        os.environ.get('LOCALAPPDATA', ''), 'Autodesk', 'webdeploy', 'production', '*', 'NODEJS',
        'node.exe')))
    if not found:
        raise SystemExit('lint: no node on PATH and no Autodesk Fusion node')
    return found if isinstance(found, str) else found[-1]


def _ensure_mdlint():
    if os.path.exists(MDLINT):
        return
    base = os.path.join(CACHE, 'mdlint')
    os.makedirs(base, exist_ok=True)
    tgz = os.path.join(base, 'npm.tgz')
    urllib.request.urlretrieve(NPM, tgz)
    with tarfile.open(tgz) as t:
        t.extractall(os.path.join(base, 'npmpkg'), filter='data')
    subprocess.run([_node(), os.path.join(base, 'npmpkg', 'package', 'bin', 'npm-cli.js'),
                    'install', '--no-audit', '--no-fund', 'markdownlint-cli2@0.23'],
                   cwd=base, capture_output=True, check=True)


def _ensure_pyright():
    if os.path.exists(os.path.join(PYRIGHT, 'pyright', '__main__.py')):
        return
    subprocess.run([sys.executable, '-m', 'pip', 'install', '-q', '--target', PYRIGHT,
                    'pyright'], capture_output=True, check=True)


def lint_md(paths):
    if not paths:
        return []
    _ensure_mdlint()
    done = subprocess.run([_node(), MDLINT] + [os.path.relpath(p, REPO).replace('\\', '/')
                                               for p in paths],     # globs: \ escapes
                          cwd=REPO, capture_output=True, text=True, encoding='utf-8',
                          errors='replace')
    out = done.stdout + done.stderr
    return [line.strip() for line in out.splitlines()
            if re.match(r'\S+\.md:\d+(:\d+)? error MD\d+', line.strip())]


def _cells(path):
    """A notebook's code cells as one module, each cell's first line marked."""
    cells = json.load(open(path, encoding='utf-8'))['cells']
    text, starts = [], []
    for i, cell in enumerate(c for c in cells if c['cell_type'] == 'code'):
        starts.append((len(text) + 1, i + 1))
        text += ''.join(cell['source']).splitlines() + ['']
    return '\n'.join(text) + '\n', starts


class _Lock:
    """One pyright at a time across hooks and agents; a stale lock (2 min) is taken over."""

    def __init__(self):
        self.path = os.path.join(CACHE, 'pyright.lock')

    def __enter__(self):
        os.makedirs(CACHE, exist_ok=True)
        until = time.monotonic() + LOCK_S
        while True:
            try:
                os.close(os.open(self.path, os.O_CREAT | os.O_EXCL))
                return self
            except FileExistsError:
                if time.time() - os.path.getmtime(self.path) > 120.0:
                    os.remove(self.path)
                elif time.monotonic() > until:
                    raise TimeoutError('another pyright holds %s' % self.path)
                time.sleep(0.5)

    def __exit__(self, *exc):
        try:
            os.remove(self.path)
        except FileNotFoundError:
            pass


def lint_py(paths):
    """pyright over .py files and notebooks' code cells, with the tree's import roots."""
    if not paths:
        return []
    _ensure_pyright()
    work = tempfile.mkdtemp(prefix='coaxial_lint_')
    try:
        files, cells = [], {}
        for p in paths:
            if p.endswith('.ipynb'):
                text, starts = _cells(p)
                twin = os.path.join(work, os.path.basename(p)[:-6] + '_cells.py')
                open(twin, 'w', encoding='utf-8').write(text)
                cells[os.path.normcase(twin)] = (p, starts)
                files.append(twin)
            else:
                files.append(p)
        json.dump({'extraPaths': [HOST, os.path.join(HOST, 'tools'), os.path.join(HOST, 'tests')],
                   'typeCheckingMode': 'basic'},
                  open(os.path.join(work, 'pyrightconfig.json'), 'w'))
        with _Lock():
            done = subprocess.run([sys.executable, '-m', 'pyright', '-p', work] + files,
                                  cwd=work, env=dict(os.environ, PYTHONPATH=PYRIGHT),
                                  capture_output=True, text=True, encoding='utf-8',
                                  errors='replace')
        found = []
        lines = done.stdout.splitlines()
        for i, line in enumerate(lines):
            m = re.match(r'\s*(.+\.py):(\d+):(\d+) - error: (.*)', line)
            if not m:
                continue
            path, row, msg = m.group(1), int(m.group(2)), m.group(4)
            while i + 1 < len(lines) and not re.match(r'\s*\S.*\.py:\d+:\d+ - ', lines[i + 1]) \
                    and lines[i + 1][:1].isspace() and lines[i + 1].strip():
                i += 1
                msg += ' ' + lines[i].strip()
            twin = cells.get(os.path.normcase(path))
            if twin:
                start, cell = max(s for s in twin[1] if s[0] <= row)
                where = '%s: code cell %d, line %d' % (os.path.relpath(twin[0], REPO), cell,
                                                        row - start + 1)
            else:
                where = '%s:%d' % (os.path.relpath(path, REPO), row)
            found.append('%s pyright %s' % (where, msg))
        return found
    finally:
        shutil.rmtree(work, ignore_errors=True)


def lint(paths):
    paths = [os.path.abspath(p) for p in paths
             if p.endswith(KINDS) and os.path.isfile(p)
             and os.path.normcase(os.path.abspath(p)).startswith(os.path.normcase(REPO))]
    return (lint_md([p for p in paths if p.endswith('.md')])
            + lint_py([p for p in paths if p.endswith(('.py', '.ipynb'))]))


def changed():
    """Every changed, staged or new file git sees, under the repo."""
    done = subprocess.run(['git', 'status', '--porcelain', '-uall'], cwd=REPO,
                          capture_output=True, text=True, encoding='utf-8', errors='replace')
    names = [line[3:].split(' -> ')[-1].strip('"') for line in done.stdout.splitlines()
             if line[:2].strip() != 'D']
    return [os.path.join(REPO, n) for n in names if n.endswith(KINDS)]


def hook(event):
    """A Claude Code hook: findings go back to the model as the reason to go on."""
    payload = json.load(sys.stdin)
    if event == 'post':
        given = payload.get('tool_input') or {}
        path = given.get('file_path') or given.get('notebook_path') or ''
        found = lint([path])
        if found:
            print(json.dumps({'decision': 'block', 'reason': 'lint, %s:\n%s' % (
                os.path.relpath(path, REPO), '\n'.join(found[:40]))}))
        return 0
    found = lint(changed())
    if not found:
        return 0
    text = '%d lint findings in the changed files:\n%s' % (len(found), '\n'.join(found[:60]))
    if payload.get('stop_hook_active'):
        print(json.dumps({'systemMessage': text}))
    else:
        print(json.dumps({'decision': 'block', 'reason': text + '\nFix them before stopping.'}))
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description='The editor linters on the command line.')
    parser.add_argument('files', nargs='*')
    parser.add_argument('--changed', action='store_true', help='every changed or new file')
    parser.add_argument('--hook', choices=('post', 'stop'), help='a Claude Code hook, stdin JSON')
    args = parser.parse_args(argv)
    if args.hook:
        return hook(args.hook)
    found = lint(args.files + (changed() if args.changed else []))
    print('\n'.join(found) or 'lint: clean')
    return 1 if found else 0


if __name__ == '__main__':
    sys.exit(main())
