"""A write of her texts held on the suites it reaches, in a worktree of its own.

    broken = gate.held(texts)     # [(suite, check)] failing with them alone: none, write

The texts' files that differ from the tree's (`ortho.source`) pick their suites by the path map
(`scope.pick`); a detached worktree at HEAD runs them on the relay as the files are, then with
`texts` written there: a check failing with them alone breaks the write. The shared tree is never
written to try one - its suites read her files from disk (her viewport's bed was broken by a step
held on her suite's roots and the bench).
"""
import os
import re
import shutil
import subprocess
import sys
import tempfile

#: The repository's root, and a failing check's line in the relay's tally: its suite and its
#: label to the label's first colon - what follows is the run's numbers.
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
FAILED = re.compile(r'^  (test_[^:\s]+): ([^:]+)', re.M)


def changed(texts):
    """{repository path: text} of `texts` unlike their files."""
    from machine.texts import ortho
    return {os.path.relpath(path, ROOT).replace(os.sep, '/'): text for path, text in texts.items()
            if text != ortho.source(path)}


def failed(host, suites):
    """{(suite, check)} failing of `suites` run on the relay in `host`'s tree."""
    argv = [sys.executable, '-X', 'utf8', os.path.join(host, 'tools', 'dev', 'run_tests.py')]
    out = subprocess.run(argv + [a for s in sorted(suites) for a in ('--file', s)], cwd=host, text=True, errors='replace',
                         capture_output=True, env=dict(os.environ, PYTHONPATH=host)).stdout
    return set(FAILED.findall(out[out.rfind('Total:'):]))


def held(texts):
    """[(suite, check)] of the suites the changed files reach (`scope.pick`) that pass as the files
    are and fail with `texts`, both runs in a worktree at HEAD: compared by name - a count hid the
    viewport's bed behind a check the bare worktree fails."""
    from tools.dev import scope
    files = changed(texts)
    if not files:
        return []
    suites = scope.pick(list(files))[0]
    tree = tempfile.mkdtemp(prefix='tune_gate_')
    shutil.rmtree(tree)
    subprocess.run(['git', 'worktree', 'add', '--detach', tree, 'HEAD'], cwd=ROOT, check=True, capture_output=True)
    try:
        host = os.path.join(tree, 'host')
        was = failed(host, suites)
        for path, text in files.items():
            with open(os.path.join(tree, path), 'w', encoding='utf-8', newline='') as f:
                f.write(text)
        now = failed(host, suites)
    finally:
        subprocess.run(['git', 'worktree', 'remove', '--force', tree], cwd=ROOT, capture_output=True)
    out = sorted(now - was)
    print('held on %d suites (%d failing as the files are): %s' % (
        len(suites), len(was), '; '.join('%s: %s' % o for o in out) or 'none fails with them'), flush=True)
    return out
