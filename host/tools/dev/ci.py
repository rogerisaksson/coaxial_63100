#!/usr/bin/env python3
"""What GitHub's last gate run told: its red suites run first here, its seconds the table's work.

    python tools/dev/ci.py          # GitHub asked (gh): the suites its last gate run on main left red
    python tools/dev/ci.py --green  # the last commit on main whose host, firmware and arch jobs, and whose gate whole, were green

The run's red suites lead suites.FAILED_FIRST; what it measured raises tests/work.json (`machine_detector.py
--keep`). `--green` names the reference tree: the newest commit whose host jobs, whose firmware jobs, whose arch
jobs and whose gate whole ended green (`jobs_green`, `green`, `whole`; the workflows that held each before the
gate was one count on), each with how many commits HEAD is past it (`behind`) and its subject; a
red suite is A/B'd against a worktree at that commit and the fault read in the diff, not derived from first
principles.

The gate on GitHub runs in parts, each stopping at its first red suite and leaving its log's tail as a comment on
its commit;
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
SUITES, WORKFLOW, BRANCH = 'host/tools/dev/suites.py', 'gate.yml', 'main'

#: The gate's jobs whose greens are each a reference - the host's suites, the target's firmware, the robot's C on
#: another target -, each with the workflow that held it before the gate was one (gate.yml): their greens count
#: on, the host's last one of host.yml's (f169f839).
JOBS = {'host': 'host.yml', 'firmware': 'firmware.yml', 'arch': 'host.yml'}

#: How many of a workflow's green runs are asked at most, the newest first; `whole` meets in them.
GREENS = 100

#: A red suite in a gate's log tail: one that failed a check, or crashed.
RED = re.compile(r'^(test_\w+\.py)\s+(?:\d+ passed, [1-9]\d* failed|CRASHED)', re.M)

#: A suite's line in a tail with its seconds - its tally or its crash -, and a runner's pace a baton.
TIMED = re.compile(r'^(test_\w+\.py)\s+(?:\d+ passed, \d+ failed|CRASHED exit=\S+)\s+([\d.]+)s$')
PACE = re.compile(r'^host: a runner\b.*?; ([\d.]+) kernels a second a baton')
#: What a job's log (gh run view --log) sets before each line: its job, its step, its time.
LOGGED = re.compile(r'^(?:[^\t]*\t){0,2}\d{4}-\d\d-\d\dT[\d:.]+Z ')

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
    """{suite: its work, kernels} a gate's log tells - its tail as the comment holds it, or the job's
    whole log (gh run view --log) -: each suite's seconds by the pace of the runner that ran it (its host
    line before them) - a suite cut at its limit by the time it ran, the most of two runs; none under no
    runner's pace."""
    pace, out = None, {}
    for line in tail.splitlines():
        line = LOGGED.sub('', line).strip()
        host, timed = PACE.match(line), TIMED.match(line)
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
    """(its commit, its gate's log tail - none where it was green -) of GitHub's last gate run on main that
    ended green or red - the host workflow's while the gate has none -, read off the comments it left on its
    commit; None where GitHub could not be asked or no run told."""
    run = None
    for workflow in (WORKFLOW, JOBS['host']):
        run = told(_gh('run', 'list', '--branch', BRANCH, '--workflow', workflow, '--status', 'completed',
                       '--limit', '10', '--json', 'conclusion,headSha') or [])
        if run is not None:
            break
    if run is None:
        return None
    if run['conclusion'] != 'failure':
        return run['headSha'], ''
    notes = _gh('api', 'repos/{owner}/{repo}/commits/%s/comments' % run['headSha']) or []
    return run['headSha'], '\n'.join(str(note.get('body', '')) for note in notes)


def asked():
    """(its commit, the suites it left red) of GitHub's last gate run on main that ended green or red - none
    where it was green -; None where GitHub could not be asked."""
    got = last()
    return None if got is None else (got[0], red(got[1]))


def _git(*words):
    """git's answer, stripped; None where git says no (a commit not here, no repository)."""
    try:
        done = subprocess.run(['git', *words], capture_output=True, text=True, encoding='utf-8',
                              errors='replace', timeout=20, cwd=ROOT)
        return done.stdout.strip() if done.returncode == 0 else None
    except (OSError, subprocess.TimeoutExpired):
        return None


def green(workflow, limit=GREENS):
    """[(commit, when, run)] of the workflow's runs on main that ended green, the newest first, `limit` at
    most; None where GitHub could not be asked."""
    runs = _gh('run', 'list', '--branch', BRANCH, '--workflow', workflow, '--status', 'success',
               '--limit', str(limit), '--json', 'headSha,createdAt,databaseId')
    return None if runs is None else [(run['headSha'], run['createdAt'], run['databaseId']) for run in runs]


def whole(*greens):
    """The newest commit green in every list given (`green`'s, each the newest first): the first run of the
    first list whose commit the others hold too; None where there is none."""
    first, *others = greens
    held = [{run[0] for run in other} for other in others]
    return next((run for run in first if all(run[0] in shas for shas in held)), None)


def behind(sha):
    """How many commits HEAD is past `sha`, 0 at HEAD itself; None where the commit is not here (fetch)."""
    count = _git('rev-list', '--count', '%s..HEAD' % sha)
    return None if count is None else int(count)


def covered(job, jobs):
    """Whether the gate's `job` (host, firmware, arch) ended green in a run's `jobs` (gh run view --json jobs): each
    of its name - 'host (3.12, 3)', 'firmware (Release)', 'arch (ubuntu-latest, avx512)' - completed green, one
    at least."""
    own = [j for j in jobs if str(j.get('name', '')).split(' (')[0] == job]
    return bool(own) and all(j.get('status') == 'completed' and j.get('conclusion') == 'success' for j in own)


def jobs_green(limit=GREENS):
    """{job: (commit, when, run) of the gate's newest run on main where it ended green, or None} for JOBS, read
    off the newest `limit` finished runs: a run green whole says so of every job, another is asked for its jobs
    (one call of gh a run, till each job has its green); None where GitHub could not be asked."""
    runs = _gh('run', 'list', '--branch', BRANCH, '--workflow', WORKFLOW, '--status', 'completed',
               '--limit', str(limit), '--json', 'headSha,createdAt,databaseId,conclusion')
    if runs is None:
        return None
    found = dict.fromkeys(JOBS)
    for run in runs:
        left = [job for job, got in found.items() if got is None]
        if not left:
            break
        jobs = None if run.get('conclusion') == 'success' else \
            (_gh('run', 'view', str(run['databaseId']), '--json', 'jobs') or {}).get('jobs', [])
        for job in left:
            if jobs is None or covered(job, jobs):
                found[job] = (run['headSha'], run['createdAt'], run['databaseId'])
    return found


def newest(*runs):
    """The newest of `runs` ((commit, when, run) or None) by when; None where all are None."""
    return max((run for run in runs if run is not None), key=lambda run: run[1], default=None)


def reference():
    """{job: its newest green run on main or None} for JOBS - the gate's (`jobs_green`) or the workflow's that held
    the job before -, and 'gate': the newest commit the gate left green whole - a gate run green, or a commit the
    host's and the firmware's workflows both left green (`whole`); None where GitHub could not be asked."""
    asked = {workflow: green(workflow) for workflow in dict.fromkeys(JOBS.values())}
    if any(runs is None for runs in asked.values()):
        return None
    legacy = {workflow: runs or [] for workflow, runs in asked.items()}
    # the gate's own, none while GitHub holds no run of its name yet (gh says no such workflow)
    own, whole_ = jobs_green() or dict.fromkeys(JOBS), green(WORKFLOW) or []
    named = {job: newest(own[job], legacy[workflow][0] if legacy[workflow] else None) for job, workflow in JOBS.items()}
    named['gate'] = newest(whole_[0] if whole_ else None, whole(*legacy.values()))
    return named


def said(named):
    """The lines that tell {name: its newest green run or None}: its commit, how far HEAD is past it
    (`behind`) and its subject cut to a line; 'none green' where none ended green."""
    width, lines = max(map(len, named)), []
    for name, run in named.items():
        if run is None:
            lines.append('ci: %-*s  none green on %s' % (width, name, BRANCH))
            continue
        past, subject = behind(run[0]), _git('log', '-1', '--format=%s', run[0]) or ''
        where = 'not here (git fetch)' if past is None else '%d commits behind HEAD' % past
        lines.append('ci: %-*s  green last at %s, %s: %s'
                     % (width, name, run[0][:8], where, textwrap.shorten(subject, 72)))
    return lines


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


def red_said():
    """The line that tells what GitHub's last gate run on main left red (`asked`); None where GitHub could
    not be asked."""
    got = asked()
    if got is None:
        return None
    return ["ci: the last gate run on %s (%s) left %s red" % (BRANCH, got[0][:8], ', '.join(got[1]) or 'nothing')]


def green_said():
    """The lines that tell the last commit each job and the gate whole left green (`reference`, `said`); None
    where GitHub could not be asked."""
    named = reference()
    return None if named is None else said(named)


def main(argv=None):
    """GitHub asked, its answer said: the suites its last gate run left red, or with --green the last commit
    each job and the gate whole left green; 1 where it could not be asked, 2 on another word."""
    words = sys.argv[1:] if argv is None else list(argv)
    if words not in ([], ['--green']):
        print('usage: ci.py [--green]')
        return 2
    lines = green_said() if words else red_said()
    if lines is None:
        print('ci: GitHub could not be asked (gh, its login, the network)')
        return 1
    print('\n'.join(lines))
    return 0


if __name__ == '__main__':
    sys.exit(main())
