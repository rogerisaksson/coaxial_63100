#!/usr/bin/env python3
"""The recordings' own CI: each carries HEAD; a commit through the hook removes them, folder or none.

    python -X utf8 tools/dev/recordings_ci.py      # from host/ of a worktree; 1 on a fault

Run in a worktree only (it commits and removes recordings), on a branch of its own (BRANCH), dropped
after, its first branch back: recordings made by the page's recorder and by the gate's own writer
process carry HEAD; a commit through the tree's .githooks removes every one, a subfolder's too, and
keeps one held open; a recording after it carries the new HEAD; an empty folder and none commit
clean; a commit the hook stops keeps them all; the main checkout's recordings are never touched.
"""
import json
import os
import pickle
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from machine.process import commit
from machine.process.introspect import RECORDINGS
from terminal.ui import recorder
from tools import HOST, REPO
from tools.dev import token_budget

#: The branch the commits land on, dropped after; a staged file the hook stops on, past the budget.
BRANCH, PAST = 'recordings-ci', os.path.join(HOST, 'tools', 'dev', 'recordings_ci_past.py')
RESULTS: list = []


def git(*args):
    """git in this checkout: its stdout, stripped."""
    return subprocess.run(['git'] + list(args), cwd=REPO, capture_output=True, text=True, encoding='utf-8',
                          errors='replace').stdout.strip()


def committed(message):
    """An empty commit through this checkout's own hooks: (whether HEAD moved, the hook's recording lines)."""
    was = git('rev-parse', 'HEAD')
    run = subprocess.run(['git', '-c', 'core.hooksPath=' + os.path.join(REPO, '.githooks'), 'commit',
                          '--allow-empty', '-q', '-m', message], cwd=REPO, capture_output=True, text=True,
                         encoding='utf-8', errors='replace')
    lines = [line for line in (run.stdout + run.stderr).splitlines() if 'recording' in line or 'kept' in line]
    return git('rev-parse', 'HEAD') != was, lines


def check(name, ok, detail: object = ''):
    """One check, told PASS or FAIL with what it saw."""
    RESULTS.append(bool(ok))
    print('%s  %s  %s' % ('PASS' if ok else 'FAIL', name, detail))


def paged(name):
    """A page's recording (R, R): its path."""
    rec = recorder.started(name, ['t'])
    recorder.added(rec, [[0.0]])
    return recorder.saved(rec)


def gated(name):
    """A gate file written by the gate's own writer process (`python -m machine.rig.gate --writes`): its
    path and the commit on its metadata line."""
    path = os.path.join(RECORDINGS, name + '.jsonl')
    subprocess.run([sys.executable, '-X', 'utf8', '-m', 'machine.rig.gate', '--writes'], cwd=HOST,
                   input=pickle.dumps((0, path, {'name': 'ci'}, {}, {'held_off': 0})), capture_output=True)
    with open(path, encoding='utf-8') as f:
        return path, json.loads(f.readline())['gate'].get('commit')


def listed(folder):
    """The files under `folder`, relative, sorted; None where it is not."""
    if not os.path.isdir(folder):
        return None
    return sorted(os.path.relpath(os.path.join(r, n), folder) for r, _d, names in os.walk(folder) for n in names)


def cases(main_recordings):
    """Every case in order on BRANCH."""
    main_was = listed(main_recordings)
    head = git('rev-parse', 'HEAD')
    page, (gate, stamped) = paged('ci_with'), gated('ci_with')
    os.makedirs(os.path.join(RECORDINGS, 'sub'), exist_ok=True)
    with open(os.path.join(RECORDINGS, 'sub', 'kept.jsonl'), 'w', encoding='utf-8') as f:
        f.write('{}\n')
    check('a page recording named with HEAD', page.endswith('_%s.csv' % head[:commit.SHORT]), os.path.basename(page))
    check("a gate file's metadata carries HEAD", stamped == head, stamped)
    landed, lines = committed('Recordings CI: a commit with recordings in the folder')
    check('with files: the commit lands, all three removed, the folder kept empty',
          landed and listed(RECORDINGS) == [] and any(line.startswith('pre-commit: 3 recordings') for line in lines),
          lines)
    after = paged('ci_after')
    new = git('rev-parse', 'HEAD')
    check('a recording after the commit carries the new HEAD', new != head and after.endswith(
        '_%s.csv' % new[:commit.SHORT]), os.path.basename(after))
    if os.name == 'nt':
        with open(after, encoding='utf-8'):
            landed, lines = committed('Recordings CI: a commit with a recording held open')
        check('held open: the commit lands, the file kept and named', landed and os.path.exists(after)
              and any(os.path.basename(after) in line for line in lines), lines)
    if os.path.exists(after):
        os.remove(after)
    landed, lines = committed('Recordings CI: a commit with an empty folder')
    check('empty folder: the commit lands, nothing told, the folder kept', landed and lines == []
          and listed(RECORDINGS) == [], lines)
    os.rmdir(RECORDINGS)
    landed, lines = committed('Recordings CI: a commit with no folder')
    check('no folder: the commit lands, nothing told, none made', landed and lines == []
          and listed(RECORDINGS) is None, lines)
    stopped = paged('ci_stopped')
    with open(PAST, 'w', encoding='utf-8', newline='\n') as f:
        f.write('# padding\n' * int(token_budget.BUDGET * 4 / 10 + 10))
    git('add', PAST)
    landed, lines = committed('Recordings CI: a commit the hook stops')
    check('stopped commit: HEAD stays, the recording kept', not landed and os.path.exists(stopped), lines)
    git('reset', '-q', '--hard')
    os.remove(stopped)
    check("the main checkout's recordings untouched", listed(main_recordings) == main_was, main_was)


def main():
    common = git('rev-parse', '--path-format=absolute', '--git-common-dir')
    if git('rev-parse', '--path-format=absolute', '--git-dir') == common:
        print('recordings CI: run it in a worktree, not the main checkout - it commits and removes recordings')
        return 2
    if os.path.dirname(RECORDINGS) != REPO or git('status', '--porcelain', '-uno'):
        print('recordings CI: a clean worktree whose host/ is imported (PYTHONPATH) is needed')
        return 2
    start = git('branch', '--show-current')
    main_recordings = os.path.join(os.path.dirname(common), 'recordings')
    git('switch', '-q', '-c', BRANCH)
    try:
        cases(main_recordings)
    finally:
        git('reset', '-q', '--hard')
        git('switch', '-q', start)
        git('branch', '-q', '-D', BRANCH)
    print('recordings CI: %d passed, %d failed' % (sum(RESULTS), len(RESULTS) - sum(RESULTS)))
    return 1 if not all(RESULTS) else 0


if __name__ == '__main__':
    sys.exit(main())
