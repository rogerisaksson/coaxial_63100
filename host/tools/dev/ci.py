#!/usr/bin/env python3
"""What GitHub's last host run left red leads the suites that run first here (suites.FAILED_FIRST).

    python tools/dev/ci.py        # GitHub asked (gh): the suites its last host run on main left red

The gate on GitHub stops at its first red suite and leaves its log's tail as a comment on its commit;
`FAILED_FIRST` schedules suites right after the structural ones, so a run here says first whether the
tree at hand still has them red. That tuple was kept by hand. The pre-commit asks GitHub at every commit
(two calls of gh, 1.8 s beside its other checks) and lays what the last finished host run left red at the
tuple's head, staged with the commit; no gh, no network or a green run: nothing changed, nothing said.
"""
import json
import re
import subprocess
import sys
import textwrap

from tools.dev.suites import ROOT

#: The file that holds FAILED_FIRST, from the repository's top; the workflow and the branch asked.
SUITES, WORKFLOW, BRANCH = 'host/tools/dev/suites.py', 'host.yml', 'main'

#: A red suite in a gate's log tail: one that failed a check, or crashed.
RED = re.compile(r'^(test_\w+\.py)\s+(?:\d+ passed, [1-9]\d* failed|CRASHED)', re.M)

#: The tuple in suites.py's text, and how wide its lines are laid.
FIRST, WIDTH, INDENT = re.compile(r'^FAILED_FIRST = \((.*?)\)$', re.M | re.S), 100, ' ' * 16


def red(tail):
    """The suites a gate's log tail names red, in its order, each once."""
    return list(dict.fromkeys(RED.findall(tail)))


def _gh(*words):
    """gh's answer read as JSON; None where gh is not here, not signed in, or GitHub is not reached."""
    try:
        done = subprocess.run(['gh', *words], capture_output=True, text=True, encoding='utf-8',
                              errors='replace', timeout=20, cwd=ROOT)
        return json.loads(done.stdout) if done.returncode == 0 else None
    except (OSError, ValueError, subprocess.TimeoutExpired):
        return None


def asked():
    """(its commit, the suites it left red) of GitHub's last finished host run on main - none where it was
    green -, read off the comments it left on its commit; None where GitHub could not be asked."""
    runs = _gh('run', 'list', '--branch', BRANCH, '--workflow', WORKFLOW, '--status', 'completed',
               '--limit', '1', '--json', 'conclusion,headSha')
    if not runs:
        return None
    commit = runs[0]['headSha']
    if runs[0]['conclusion'] != 'failure':
        return commit, []
    notes = _gh('api', 'repos/{owner}/{repo}/commits/%s/comments' % commit) or []
    return commit, red('\n'.join(str(note.get('body', '')) for note in notes))


def led(source, suites):
    """`source`, suites.py's text, with `suites` at the head of its FAILED_FIRST and the others after them in
    their order; `source` itself where they lead already, or it holds no such tuple."""
    found = FIRST.search(source)
    names = re.findall(r"'(test_\w+\.py)'", found.group(1)) if found else []
    order = list(dict.fromkeys(list(suites) + names))
    if not found or order == names:
        return source
    lines = textwrap.wrap(', '.join("'%s'" % name for name in order), WIDTH - len(INDENT))
    return source[:found.start()] + 'FAILED_FIRST = (' + ('\n' + INDENT).join(lines) + ')' + source[found.end():]


def main():
    """GitHub asked, its answer said: 1 where it could not be asked."""
    got = asked()
    if got is None:
        print('ci: GitHub could not be asked (gh, its login, the network)')
        return 1
    print("ci: the last host run on %s (%s) left %s red" % (BRANCH, got[0][:8], ', '.join(got[1]) or 'nothing'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
