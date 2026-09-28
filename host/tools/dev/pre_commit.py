#!/usr/bin/env python3
"""The pre-commit hook: a staged file grown past the token budget stops the commit until split.

.githooks/pre-commit runs it; setup.ps1 points git at .githooks.

    python tools/dev/pre_commit.py         # what `git commit` runs; 1 stops it

The staged text is judged, not the working tree's: a file over tools/dev/token_budget.BUDGET,
or one of its HEAVY grown past its cap, is named with the way to split it.
"""
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

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
            yield path, len(_git('show', ':' + path)) / 4.0


def how(path):
    """The way to split `path`."""
    if '/tests/' in '/' + path:
        return SPLIT['tests']
    return SPLIT.get(os.path.splitext(path)[1], SPLIT['.py'])


def main():
    past = [(p, t, token_budget.HEAVY.get(p, token_budget.BUDGET)) for p, t in staged()
            if t > token_budget.HEAVY.get(p, token_budget.BUDGET)]
    if not past:
        return 0
    print('pre-commit: %d file%s past what a model reads whole - split before committing:'
          % (len(past), '' if len(past) == 1 else 's'))
    for path, tokens, cap in past:
        print('  %s  %.1f k tokens of %.1f k: %s' % (path, tokens / 1e3, cap / 1e3, how(path)))
    return 1


if __name__ == '__main__':
    sys.exit(main())
