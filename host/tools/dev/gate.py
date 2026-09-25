#!/usr/bin/env python3
"""The offline gate - or the coverage run - on a commit, in a worktree of its own.

    python tools/dev/gate.py            # HEAD: the offline suites, the tally
    python tools/dev/gate.py abc1234    # that commit
    python tools/dev/gate.py --cover    # tools/dev/cover.py there instead

The tree the work goes on in keeps moving while this runs: the commit is checked out
at build/gate/tree (detached, reused), its host/ first on the path so its code is the
one imported, not the editable install's; the log is build/gate/<sha>.log. Exit 0
only when the run was green.
"""
import argparse
import os
import subprocess
import sys

from tools import REPO

WHERE = os.path.join(REPO, 'build', 'gate')
TREE = os.path.join(WHERE, 'tree')


def git(*args, cwd=REPO):
    return subprocess.run(['git'] + list(args), cwd=cwd, capture_output=True, text=True,
                          check=True).stdout.strip()


def checkout(ref):
    """The worktree at `ref`, made on first use; its sha."""
    sha = git('rev-parse', ref)
    if not os.path.isdir(os.path.join(TREE, '.git')) and not os.path.isfile(
            os.path.join(TREE, '.git')):
        os.makedirs(WHERE, exist_ok=True)
        git('worktree', 'add', '--detach', '--force', TREE, sha)
    else:
        git('checkout', '--detach', '--force', sha, cwd=TREE)
    return sha


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('ref', nargs='?', default='HEAD')
    parser.add_argument('--cover', action='store_true', help='tools/dev/cover.py instead')
    args = parser.parse_args(argv)
    sha = checkout(args.ref)
    host = os.path.join(TREE, 'host')
    script = ['tools/dev/cover.py', '--files'] if args.cover else [
        'tools/dev/run_tests.py', '--offline']
    log = os.path.join(WHERE, '%s%s.log' % (sha[:7], '.cover' if args.cover else ''))
    env = dict(os.environ, PYTHONPATH=host, PYTHONUTF8='1')
    with open(log, 'w', encoding='utf-8') as out:
        done = subprocess.run([sys.executable] + script, cwd=host, env=env, stdout=out,
                              stderr=subprocess.STDOUT)
    with open(log, encoding='utf-8') as f:
        lines = f.read().splitlines()
    failed = [line for line in lines if line.startswith('  FAIL')]
    total = [line for line in lines if line.startswith('Total:')]
    print('%s %s: %s' % (sha[:7], 'cover' if args.cover else 'gate',
                         total[-1] if total else 'no tally - see %s' % log))
    for line in failed[:20]:
        print(line)
    return 0 if done.returncode == 0 and not failed else 1


if __name__ == '__main__':
    sys.exit(main())
