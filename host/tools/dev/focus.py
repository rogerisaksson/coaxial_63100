"""A suite's reach and time: the tests a command line names, a watchdog, the relay of batons.

    python -X utf8 tests/test_simulated.py sto gate     # the tests with those words in their names
    python -X utf8 tests/test_emulator.py               # its groups on the relay, a Renode each
    python -X utf8 tests/test_emulator.py board sto     # one group, one test of it

The relay: a baton a physical core and a queue of jobs, the longest first. A baton takes the
next job whose commit fits what is left and whose lock is free, and runs it in a process of its
own; a job ending hands its baton on at once, so no core idles while a job waits, whatever each
takes. `emulator://` starts one Renode a process: jobs side by side are emulators side by side.
A suite past a slice of the run goes on as shards (`--shard k/n`, `chosen`), side by side.
"""
import collections
import ctypes
import faulthandler
import functools
import os
import queue
import re
import signal
import subprocess
import sys
import threading
import time
from contextlib import suppress

from tools.dev.suites import ROOT

#: A suite's closing line: what it passed and failed, and what it skipped.
TALLY_RE = re.compile(r'^(\d+) passed, (\d+) failed(?:, ~?(\d+) skipped)?$')

#: A job's commit where it names none, and what the host keeps free for the rest, GB: an
#: emulator's Renode 0.6 and its Python 0.3 with OpenBLAS on one thread; eight at once left 1.3
#: of 23.7 GB with no page file, the desktop holding 17.9 (2026-09-27).
WORKER_GB, RESERVE_GB = 0.9, 2.5

#: One job on the relay: its name, its command, its commit (GB), its time (s), and its lock -
#: jobs holding the same one never run together (a board's port), 'alone' none beside it.
Job = collections.namedtuple('Job', 'name argv gb timeout lock',
                             defaults=(WORKER_GB, 600.0, None))

#: The shard a suite's command line asks for: the k-th of every n of its tests, from 1.
SHARD_RE = re.compile(r'^(\d+)/(\d+)$')


def kill_tree(pid):
    """The process and every descendant, gone."""
    if os.name == 'nt':
        subprocess.run(['taskkill', '/F', '/T', '/PID', str(pid)], capture_output=True)
        return
    with suppress(OSError):
        os.killpg(os.getpgid(pid), signal.SIGKILL)


@functools.lru_cache(maxsize=None)
def physical_cores():
    """The machine's physical cores: psutil's count where it is installed, Windows' own
    (Win32_Processor), else the logical count."""
    try:
        import psutil
        return psutil.cpu_count(logical=False) or os.cpu_count() or 1
    except ImportError:
        pass
    if os.name == 'nt':
        with suppress(OSError, ValueError, subprocess.TimeoutExpired):
            said = subprocess.run(
                ['powershell', '-NoProfile', '-Command',
                 '(Get-CimInstance Win32_Processor | Measure-Object -Property NumberOfCores '
                 '-Sum).Sum'], capture_output=True, text=True, timeout=30).stdout
            return max(1, int(said.strip()))
    return os.cpu_count() or 1


def free_commit_gb():
    """The commit charge still free, GB; None off Windows."""
    if os.name != 'nt':
        return None

    class Status(ctypes.Structure):
        _fields_ = [('length', ctypes.c_uint32), ('load', ctypes.c_uint32)] + [
            (name, ctypes.c_uint64) for name in ('phys', 'phys_free', 'commit', 'commit_free',
                                                 'virtual', 'virtual_free', 'extended')]
    status = Status(length=ctypes.sizeof(Status))
    if not ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(status)):
        return None
    return status.commit_free / 2 ** 30


def pick(tests, names):
    """The `tests` whose names hold one of `names` - `sto` picks
    test_the_sto_chain_follows_the_pilot - in the suite's order; all for no names. A name no
    test holds exits, listing them."""
    if not names:
        return list(tests)
    words = {test: test.__name__[5:] for test in tests}
    unknown = [name for name in names if not any(name in w for w in words.values())]
    if unknown:
        sys.exit('no test named %s here: %s' % (', '.join(unknown), ', '.join(words.values())))
    return [test for test in tests if any(name in words[test] for name in names)]


def chosen(tests, argv):
    """The `tests` a suite's command line asks for, in the suite's order: those its words name
    (`pick`), and of them the share `--shard k/n` gives - every n-th from the k-th, so n shards
    side by side run each test once."""
    words, shard, args = [], None, list(argv)
    while args:
        arg = args.pop(0)
        if arg == '--shard' and args:
            arg = '--shard=' + args.pop(0)
        if arg.startswith('--shard='):
            m = SHARD_RE.match(arg.split('=', 1)[1])
            if not m or not 1 <= int(m.group(1)) <= int(m.group(2)):
                sys.exit('--shard k/n, 1 <= k <= n: %s' % arg)
            shard = (int(m.group(1)), int(m.group(2)))
        else:
            words.append(arg)
    tests = pick(tests, words)
    return tests[shard[0] - 1::shard[1]] if shard else tests


def watchdog(seconds):
    """Every thread's stack printed and the process gone `seconds` on: a hung suite says where,
    and its emulators go with it (tools.emu.emulator's job object)."""
    faulthandler.dump_traceback_later(seconds, exit=True)


def run(job, env=None):
    """One job in a process of its own, stdout and stderr in one, killed with its tree at its
    time: (output, exit code - None out of time -, seconds)."""
    started = time.monotonic()
    group = {'start_new_session': True} if os.name != 'nt' else {}
    proc = subprocess.Popen(job.argv, cwd=str(ROOT), env=env, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, encoding='utf-8',
                            errors='replace', **group)
    try:
        out, _ = proc.communicate(timeout=job.timeout)
        code = proc.returncode
    except subprocess.TimeoutExpired:
        kill_tree(proc.pid)
        out, _ = proc.communicate()
        code = None
    return out or '', code, time.monotonic() - started


def relay(jobs, batons=None):
    """`jobs` from a queue in their order, the longest first where the caller knows: `batons` -
    a physical core each - each taking the first job whose commit fits the free commit over
    RESERVE_GB, measured at the start and again as each job ends (one runs whatever it needs),
    OpenBLAS on one thread in each; a job ending hands its baton to the next at once. (job,
    output, exit code - None out of time -, seconds) as each ends."""
    waiting = collections.deque(jobs)
    count = len(waiting)
    free = free_commit_gb()
    held = {'gb': 0.0, 'running': 0, 'locks': set(),
            'budget': float('inf') if free is None else free - RESERVE_GB}
    turn = threading.Condition()
    ended = queue.Queue()
    env = dict(os.environ, PYTHONIOENCODING='utf-8')
    env.setdefault('OPENBLAS_NUM_THREADS', '1')

    def fits(job):
        """Its commit within the budget - anything fits an idle host - and its lock free: an
        'alone' job waits for the host to empty, and holds it."""
        if 'alone' in held['locks']:
            return False
        if job.lock == 'alone':
            return not held['running']
        if job.lock is not None and job.lock in held['locks']:
            return False
        return not held['running'] or held['gb'] + job.gb <= held['budget']

    def take():
        """The next job that fits; None once none is left. Waits while those left do not."""
        with turn:
            while waiting:
                job = next((j for j in waiting if fits(j)), None)
                if job is not None:
                    waiting.remove(job)
                    held['gb'] += job.gb
                    held['running'] += 1
                    if job.lock is not None:
                        held['locks'].add(job.lock)
                    return job
                turn.wait()
            return None

    def baton():
        while (job := take()) is not None:
            try:
                ended.put((job,) + run(job, env))
            except OSError as exc:
                ended.put((job, '%s: %s' % (type(exc).__name__, exc), 1, 0.0))
            finally:
                with turn:
                    held['gb'] -= job.gb
                    held['running'] -= 1
                    held['locks'].discard(job.lock)
                    # The host's own share moves too: what is free now, and what the jobs
                    # still running hold of it.
                    free = free_commit_gb()
                    if free is not None:
                        held['budget'] = free + held['gb'] - RESERVE_GB
                    turn.notify_all()

    for _ in range(max(1, min(count, batons or physical_cores()))):
        threading.Thread(target=baton, name='baton', daemon=True).start()
    for _ in range(count):
        yield ended.get()


def run_groups(path, groups, timeout, batons=None):
    """A suite's `groups` on the relay, `python path group` each, the longest first as given:
    each group's lines as it ends, and one tally over them, a group out of time or ending
    without a tally one failure. `timeout`, s: one for all, or a group's own by name. The exit
    code."""
    passed = failed = 0
    jobs = [Job(group, [sys.executable, '-X', 'utf8', str(path), group], WORKER_GB,
                timeout[group] if isinstance(timeout, dict) else timeout)
            for group in groups]
    for job, out, code, took in relay(jobs, batons):
        lines = out.splitlines()
        tally = next((m for m in (TALLY_RE.match(line.strip()) for line in reversed(lines)) if m),
                     None)
        print('\n== %s, %.0f s ==' % (job.name, took))
        print('\n'.join(line for line in lines if not TALLY_RE.match(line.strip())))
        if code is None or tally is None:
            failed += 1
            print('  FAIL  %s: %s' % (job.name, 'out of time at %.0f s' % job.timeout
                                      if code is None else 'ended without a tally'))
        else:
            passed += int(tally.group(1))
            failed += int(tally.group(2))
        sys.stdout.flush()
    print('\n%d passed, %d failed' % (passed, failed))
    return 1 if failed else 0
