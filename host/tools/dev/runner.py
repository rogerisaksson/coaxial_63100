"""The suites as jobs on one relay, their tallies read and merged a suite each.

The long ones as shards and the emulator's groups a job each: run, timed out, their trees
killed."""
import math
import os
import re
import subprocess
import sys
import time

from tools.dev import counts
from tools.dev.focus import TALLY_RE, WORKER_GB, Job, kill_tree, relay
from tools.dev.suites import (ALONE, EMULATOR, EMULATOR_GROUPS, LIVE, OLLAMA, PORT, REAL_TIME,
                              FRESH_S, ROOT, SHARDED)


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

#: A suite's commit on the relay, GB: the stand-in's own process; the front page's views
#: suite starts a page's processes by the dozen.
SUITE_GB = 0.3
HEAVY_GB = {'test_views_front.py': 2.0}

#: A job's share of a run is half the run's work over the batons, no less than SLICE_S s: a
#: suite past it goes on as shards side by side, at most MAX_SHARDS - few on the laptop's 8
#: cores, many on 32. One job a suite, the offline gate's three longest - 99 to 137 s - left it
#: on 13 % of the cores for its last minute and a half, the suites kept alone one after another
#: behind them: 544 s. Shards at the whole work over the batons, 246 s, the views whole the
#: last two minutes of it (2026-09-28).
SLICE_S = 10.0
MAX_SHARDS = 16


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


def _emulated_here():
    """Whether the emulator's groups run here: Renode and both images, as the suite asks."""
    from tools.emu.emulator import BOOT_ELF, ELF, find_renode
    return find_renode() is not None and os.path.exists(ELF) and os.path.exists(BOOT_ELF)


def _lock(name, batons):
    """A suite's lock on the relay: the host alone, the board's port, or none."""
    if name in ALONE or (name in REAL_TIME and batons <= 4):
        return 'alone'
    return 'port' if name in PORT else None


def _jobs(name, args, tags, live_sections, took, share, words=()):
    """The jobs one suite runs as: its words' alone, the emulator's groups where it runs here,
    shards past `share` s, else the suite whole."""
    argv = ([sys.executable, str(ROOT / 'tests' / name)]
            + _extra_for(name, args, tags, live_sections))
    gb, lock, timeout = HEAVY_GB.get(name, SUITE_GB), _lock(name, args.jobs), _timeout(name)
    if words:
        return [Job(name, argv + list(words), gb, timeout, lock)]
    if name == EMULATOR and _emulated_here():
        return [Job('%s %s' % (name, group), argv + [group], WORKER_GB, seconds, lock)
                for group, seconds in EMULATOR_GROUPS.items()]
    shards = (min(MAX_SHARDS, math.ceil(took.get(name, FRESH_S.get(name, 0.0)) / share))
              if name in SHARDED else 1)
    if shards <= 1:
        return [Job(name, argv, gb, timeout, lock)]
    return [Job('%s %d/%d' % (name, k, shards), argv + ['--shard', '%d/%d' % (k, shards)], gb,
                timeout, lock) for k in range(1, shards + 1)]


def _merged(parts):
    """One suite's result out of its jobs': the tallies summed, the failing lines in order, the
    seconds summed - its work, which the next run's shards are planned on. A job with no tally
    is the suite's crash."""
    crash = next((p for p in parts if p[0] is None), None)
    code = next((p[1] for p in parts if p[1]), parts[0][1])
    failing = [line for p in parts for line in p[2]]
    elapsed = sum(p[3] for p in parts)
    groups = next((p[5] for p in parts if p[5]), None)
    if crash is not None:
        return None, crash[1], failing, elapsed, crash[4], groups
    tally = tuple(sum(p[0][k] for p in parts) for k in range(3)) + (any(p[0][3] for p in parts),)
    return tally, code, failing, elapsed, None, groups


def _results(suites, args, tags, live_sections):
    """(suite, its result) in the order the report lists them, once every job has ended: all of
    them on one relay, the longest expected first, a baton a physical core - the port's suites
    one at a time beside the rest, the quiet ones with the host to themselves.
    """
    known = counts.load()
    took, jobs_took = known.get('seconds') or {}, known.get('jobs') or {}
    words = getattr(args, 'words', None) or {}
    share = max(SLICE_S, sum(took.get(name, SLICE_S) for name in suites)
                / (2.0 * max(1, args.jobs)))
    plan = {name: _jobs(name, args, tags, live_sections, took, share, words.get(name, ()))
            for name in suites if (ROOT / 'tests' / name).exists()}

    def expected(job):
        suite = job.name.split()[0]
        return jobs_took.get(job.name, took.get(suite, float('inf')) / len(plan[suite]))
    queue = sorted((job for jobs in plan.values() for job in jobs), key=lambda j: -expected(j))
    got, seconds = {}, {}
    for job, out, code, took_s in relay(queue, args.jobs):
        got[job.name] = _parse(out, code, took_s, job.timeout)
        seconds[job.name] = round(took_s, 1)
    counts.record('jobs', seconds)
    for name in suites:
        yield name, (_merged([got[job.name] for job in plan[name]]) if name in plan else None)
