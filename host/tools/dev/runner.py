"""One suite in its own process: run, time out, kill the tree, read the tally."""
import os
import re
import signal
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from contextlib import suppress

from tools.dev import counts
from tools.dev.suites import ALONE, LIVE, OLLAMA, ROOT


TALLY_RE = re.compile(r'^(\d+) passed, (\d+) failed(?:, ~?(\d+) skipped)?$')

# The whole line after FAIL, detail included: a check's detail is the compiler
# warning, the wrong value, the reason - and on a runner the summary (relayed
# as a commit comment) is the only place it surfaces.
FAIL_RE = re.compile(r'^\s{1,8}FAIL\s+(\S.*?)\s*$')

# The ollama suites under --tags say what they left out.
GROUPS_RE = re.compile(r'^ran \d+ of \d+ groups: .*$')


def kill_tree(pid):
    """The process and every descendant, gone."""
    if os.name == 'nt':
        subprocess.run(['taskkill', '/F', '/T', '/PID', str(pid)],
                       capture_output=True)
        return
    with suppress(OSError):
        os.killpg(os.getpgid(pid), signal.SIGKILL)


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


def run_one(path, timeout=300, extra=()):
    """(tally, code, failing, elapsed, crash-or-None, groups-line-or-None)."""
    started = time.monotonic()
    done = run_captured([sys.executable, str(path)] + list(extra), timeout,
                        cwd=str(ROOT))
    if done is None:
        return (None, None, [], time.monotonic() - started,
                'TIMEOUT after %ss' % timeout, None)

    elapsed = time.monotonic() - started
    lines = (done.stdout or '').splitlines()
    tally = None
    for line in reversed(lines):
        m = TALLY_RE.match(line.strip())
        if m:
            tally = (int(m.group(1)), int(m.group(2)),
                     int(m.group(3) or 0), '~' in line)
            break
    failing = [m.group(1).strip() for m in (FAIL_RE.match(l) for l in lines) if m]
    groups = next((l.strip() for l in reversed(lines)
                   if GROUPS_RE.match(l.strip())), None)

    if tally is None:
        # The suite crashed before printing its own tally - a traceback, an
        # import error.
        detail = (done.stderr or done.stdout or '').strip()
        return None, done.returncode, failing, elapsed, detail[-1500:], groups
    return tally, done.returncode, failing, elapsed, None, groups


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


def _job(name, args, tags, live_sections):
    """One suite run - `run_one`'s tuple, or None where the file is not."""
    path = ROOT / 'tests' / name
    if not path.exists():
        return None
    return run_one(path, timeout=1200 if name == LIVE else 300,
                   extra=_extra_for(name, args, tags, live_sections))


def _results(suites, args, tags, live_sections):
    """(suite, its result) in the order the report lists them: the suites
    that share the machine, in the plan's order, then the ones that want
    it alone.
    """
    took = counts.load().get('seconds') or {}
    sharing = [name for name in suites if name not in ALONE]
    pool = ThreadPoolExecutor(max_workers=max(1, args.jobs))
    try:
        started = {name: pool.submit(_job, name, args, tags, live_sections)
                   for name in sorted(
                       sharing, key=lambda n: -took.get(n, float('inf')))}
        for name in sharing:
            yield name, started[name].result()
    finally:
        pool.shutdown(wait=True, cancel_futures=True)
    for name in suites:
        if name in ALONE:
            yield name, _job(name, args, tags, live_sections)
