"""One suite in its own process: run, time out, kill the tree, read the tally - on the relay."""
import os
import re
import subprocess
import sys
import time

from tools.dev import counts
from tools.dev.focus import TALLY_RE, Job, kill_tree, relay
from tools.dev.suites import ALONE, LIVE, OLLAMA, ROOT


# The whole line after FAIL, detail included: a check's detail is the compiler
# warning, the wrong value, the reason - and on a runner the summary (relayed
# as a commit comment) is the only place it surfaces. The line's first marker,
# escapes stripped: a page drawn in-process leaves its last write - a footer, a
# screen clear - where the next PASS or FAIL lands (CI's 3.12 counted a FAIL on
# 332a3fc and named none).
MARK_RE = re.compile(r'(?:^|\s)(PASS|FAIL|SKIP)\s+(\S.*?)\s*$')
ANSI_RE = re.compile(r'\x1b\[[0-9;?]*[A-Za-z]')

# The ollama suites under --tags say what they left out.
GROUPS_RE = re.compile(r'^ran \d+ of \d+ groups: .*$')

#: A suite's commit on the relay, GB: the stand-in's own process; the views suite starts a
#: page's processes by the dozen.
SUITE_GB = 0.3
HEAVY_GB = {'test_views.py': 2.0}


def run_captured(argv, timeout, cwd=None):
    """`argv` run with its output captured, or None once `timeout` has
    passed - the whole tree killed, not the child alone.
    """
    env = dict(os.environ, PYTHONIOENCODING='utf-8')
    group = {'start_new_session': True} if os.name != 'nt' else {}
    proc = subprocess.Popen(argv, cwd=cwd, env=env, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, text=True,
                            encoding='utf-8', errors='replace', **group)
    try:
        out, err = proc.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        kill_tree(proc.pid)
        proc.communicate()
        return None
    return subprocess.CompletedProcess(argv, proc.returncode, out, err)


def _parse(out, code, elapsed, timeout):
    """(tally, code, failing, elapsed, crash-or-None, groups-line-or-None) out of a suite's
    output; `code` None is out of time."""
    if code is None:
        return None, None, [], elapsed, 'TIMEOUT after %ss' % timeout, None
    lines = (out or '').splitlines()
    tally = None
    for line in reversed(lines):
        m = TALLY_RE.match(line.strip())
        if m:
            tally = (int(m.group(1)), int(m.group(2)),
                     int(m.group(3) or 0), '~' in line)
            break
    marks = (MARK_RE.search(ANSI_RE.sub('', line)) for line in lines)
    failing = [m.group(2).strip() for m in marks if m and m.group(1) == 'FAIL']
    groups = next((l.strip() for l in reversed(lines)
                   if GROUPS_RE.match(l.strip())), None)
    if tally is None:
        # The suite crashed before printing its own tally - a traceback, an
        # import error.
        return None, code, failing, elapsed, (out or '').strip()[-1500:], groups
    return tally, code, failing, elapsed, None, groups


def run_one(path, timeout=300, extra=()):
    """(tally, code, failing, elapsed, crash-or-None, groups-line-or-None)."""
    started = time.monotonic()
    done = run_captured([sys.executable, str(path)] + list(extra), timeout,
                        cwd=str(ROOT))
    if done is None:
        return _parse('', None, time.monotonic() - started, timeout)
    return _parse((done.stdout or '') + (done.stderr or ''), done.returncode,
                  time.monotonic() - started, timeout)


def _extra_for(name, args, tags, live_sections):
    """The flags one suite takes from the plan: the model, its sections and
    match for the live suite; the picked tests or the tags and coverage
    for the ollama ones.
    """
    extra = ['-m', args.model] if name == LIVE else []
    if name == LIVE and live_sections:
        extra += ['--sections', live_sections]
    if name == LIVE and args.match:
        extra += ['--match', args.match]
    if name not in OLLAMA:
        return extra
    if args.only:
        return extra + ['--only', args.only]
    if tags:
        extra += ['--tags', tags]
    if tags and args.coverage:
        extra += ['--coverage', str(args.coverage)]
    return extra


def _timeout(name):
    return 1200 if name == LIVE else 300


def _job(name, args, tags, live_sections):
    """One suite run - `run_one`'s tuple, or None where the file is not."""
    path = ROOT / 'tests' / name
    if not path.exists():
        return None
    return run_one(path, timeout=_timeout(name),
                   extra=_extra_for(name, args, tags, live_sections))


def _results(suites, args, tags, live_sections):
    """(suite, its result) in the order the report lists them: the suites that share the host
    on the relay - the longest first, a baton a physical core - then the ones that want it
    alone, one after another.
    """
    took = counts.load().get('seconds') or {}
    sharing = [name for name in suites if name not in ALONE]
    jobs = [Job(name, [sys.executable, str(ROOT / 'tests' / name)]
                + _extra_for(name, args, tags, live_sections),
                HEAVY_GB.get(name, SUITE_GB), _timeout(name))
            for name in sorted(sharing, key=lambda n: -took.get(n, float('inf')))
            if (ROOT / 'tests' / name).exists()]
    got = {job.name: _parse(out, code, seconds, job.timeout)
           for job, out, code, seconds in relay(jobs, args.jobs)}
    for name in sharing:
        yield name, got.get(name)
    for name in suites:
        if name in ALONE:
            yield name, _job(name, args, tags, live_sections)
