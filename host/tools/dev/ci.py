#!/usr/bin/env python3
"""What GitHub's last host run told: its red suites run first here, its seconds the table's work.

    python tools/dev/ci.py        # GitHub asked (gh): the suites its last host run on main left red

Its red suites lead suites.FAILED_FIRST; what it measured raises tests/work.json (`machine_detector.py
--keep`).

The gate on GitHub stops at its first red suite and leaves its log's tail as a comment on its commit;
`FAILED_FIRST` schedules suites right after the structural ones, so a run here says first whether the
tree at hand still has them red. That tuple was kept by hand. The pre-commit asks GitHub at every commit
(two calls of gh, 1.8 s beside its other checks) and lays what the last finished host run left red at the
tuple's head, staged with the commit; no gh, no network or a green run: nothing changed, nothing said.
The same tail says each suite's seconds on the runner and the runner's pace: its work there, which the
table holds for a host with no record of its own - GitHub's runner itself - where this desk's runs alone
had told it: test_sight renders on a card here, on the runner's processor there, and was cut at 300 s.
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

#: A suite's line in a tail with its seconds - its tally or its crash -, and a runner's pace a baton.
TIMED = re.compile(r'^(test_\w+\.py)\s+(?:\d+ passed, \d+ failed|CRASHED exit=\S+)\s+([\d.]+)s$')
PACE = re.compile(r'^host: a runner\b.*?; ([\d.]+) kernels a second a baton')

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


def measured(tail):
    """{suite: its work, kernels} a gate's log tail tells: each suite's seconds by the pace of the runner
    that ran it (its host line before them) - a suite cut at its limit by the time it ran, the most of two
    runs; none under no runner's pace."""
    pace, out = None, {}
    for line in tail.splitlines():
        host, timed = PACE.match(line.strip()), TIMED.match(line.strip())
        if host:
            pace = float(host.group(1))
        elif timed and pace:
            out[timed.group(1)] = max(out.get(timed.group(1), 0.0), round(float(timed.group(2)) * pace, 1))
    return out


def told(runs):
    """Of finished runs, the newest first, the newest that ended green or red - a cancelled one tells
    neither -; None where none did."""
    return next((run for run in runs if run.get('conclusion') in ('success', 'failure')), None)


def last():
    """(its commit, its gate's log tail - none where it was green -) of GitHub's last host run on main that
    ended green or red, read off the comments it left on its commit; None where GitHub could not be asked
    or no run told."""
    run = told(_gh('run', 'list', '--branch', BRANCH, '--workflow', WORKFLOW, '--status', 'completed',
                   '--limit', '10', '--json', 'conclusion,headSha') or [])
    if run is None:
        return None
    if run['conclusion'] != 'failure':
        return run['headSha'], ''
    notes = _gh('api', 'repos/{owner}/{repo}/commits/%s/comments' % run['headSha']) or []
    return run['headSha'], '\n'.join(str(note.get('body', '')) for note in notes)


def asked():
    """(its commit, the suites it left red) of GitHub's last host run on main that ended green or red - none
    where it was green -; None where GitHub could not be asked."""
    got = last()
    return None if got is None else (got[0], red(got[1]))


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
