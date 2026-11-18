"""A suite's reach and time: the tests a command line names, a watchdog, the relay of batons.

    python -X utf8 tests/test_simulated.py sto gate     # the tests with those words in their names
    python -X utf8 tests/test_emulator.py               # its groups on the relay, a Renode each
    python -X utf8 tests/test_emulator.py board sto     # one group, one test of it

The relay: a baton a physical core and a queue of jobs, the longest first. A baton takes the
next job whose commit fits what is left and whose lock is free, and runs it in a process of its
own; a job ending hands its baton on at once, so no core idles while a job waits, whatever each
takes. `emulator://` starts one Renode a process: jobs side by side are emulators side by side.
A suite that starts one asks for its cores (`suites.asked`): they go along with its job (`run`).
A suite past a slice of the run goes on as shards (`--shard k/n`, `chosen`), side by side.
"""
import collections
import ctypes
import faulthandler
import json
import os
import queue
import re
import signal
import subprocess
import sys
import threading
import time
from contextlib import suppress

from tools.dev import background, census
from tools.dev.suites import ROOT, asked

if os.name == 'nt':
    # No Windows Error Reporting dialog on a child's unhandled exception.
    ctypes.windll.kernel32.SetErrorMode(0x8003)

#: A suite's closing line: what it passed and failed, and what it skipped.
TALLY_RE = re.compile(r'^(\d+) passed, (\d+) failed(?:, ~?(\d+) skipped)?$')

#: A job's commit where it names none, and what the host keeps free for the rest, GB: an
#: emulator's Renode 0.6 and its Python 0.3 with OpenBLAS on one thread; eight at once left 1.3
#: of 23.7 GB with no page file, the desktop holding 17.9.
WORKER_GB, RESERVE_GB = 0.9, 2.5
#: Every relay's running jobs, machine-wide: a file a job, its commit and its process. A job a
#: relay took is counted against all of them at what it has still to commit - two sessions'
#: relays each sized their batons on the same free commit and the desktop went out of memory
#: (Windows' event 2004, 10.4 GB of 23.7 free before they started).
LEDGER = os.path.join(os.environ.get('LOCALAPPDATA') or os.path.expanduser('~'), 'coaxial_63100', 'relay')
#: How often a relay asks the ledger while its jobs wait, and a booking waiting for room, s.
POLL_S = 1.0

#: One job on the relay: its name, its command, its commit (GB), its time (s), its lock -
#: jobs holding the same one never run together (a board's port), 'alone' none beside it -
#: and its cores: its own and, where its suite asked (`suites.asked`), a core a machine of the
#: Renode it starts, sent along with it (`run`).
Job = collections.namedtuple('Job', 'name argv gb timeout lock cores',
                             defaults=(WORKER_GB, 600.0, None, 1))
#: The cores sent along with a job, a mask in its environment: Renode is bound to them where it
#: starts (tools.emu.probes.Cored).
SENT = 'COAXIAL_RENODE'

#: The shard a suite's command line asks for: the k-th of every n of its tests, from 1.
SHARD_RE = re.compile(r'^(\d+)/(\d+)$')


def kill_tree(pid, known=None):
    """The process and every descendant, gone. Windows': by a taskkill in a console of its own - started in one
    that answers no more (`census.console`) the taskkill ended at once. Elsewhere the group by its own id, its
    leader reaped or not, then the live of `known` - its tree read before its leader went (`last_words`), read now
    where none is given - with what they started: a child in a session of its own is no member of the group, and
    an orphan is init's."""
    if os.name == 'nt':
        subprocess.run(['taskkill', '/F', '/T', '/PID', str(pid)], capture_output=True,
                       creationflags=getattr(subprocess, 'CREATE_NO_WINDOW'))
        return
    with suppress(OSError):
        os.killpg(pid, signal.SIGKILL)
    for each in census.Snapshot().tree([pid] if known is None else known):
        with suppress(OSError):
            os.kill(each, signal.SIGKILL)


#: A job out of time: its Python processes' stacks said this long before its tree is killed, s.
SAY_S = 10.0


def last_words(proc):
    """(stdout, stderr) of `proc` out of time, its tree killed: on POSIX each Python process of its
    group says its threads' stacks first (faulthandler on SIGABRT), SAY_S s at the most, its tree read before - a
    grandchild in a session of its own, the job's pipe in hand, held a baton 25 s past a limit of 3."""
    known = None
    if os.name != 'nt':
        known = census.Snapshot().tree([proc.pid])
        with suppress(OSError):
            os.killpg(proc.pid, signal.SIGABRT)
        with suppress(subprocess.TimeoutExpired):
            proc.wait(timeout=SAY_S)
    kill_tree(proc.pid, known)
    return proc.communicate()


def environment():
    """A job's environment: the relay's own, `ROOT` on `PYTHONPATH`, OpenBLAS on one thread where not
    told, its output unbuffered and each of its processes saying its threads' stacks on a fatal signal
    - a job cut at its time says where it was: test_sight's 300 s on GitHub's runner said nothing, its
    prints held in a pipe's buffer."""
    env = dict(os.environ, PYTHONIOENCODING='utf-8', PYTHONPATH=str(ROOT), PYTHONUNBUFFERED='1', PYTHONFAULTHANDLER='1')
    env.setdefault('OPENBLAS_NUM_THREADS', '1')
    return env


class _Locked:

    """The ledger held by this process alone while a relay measures, decides and books."""

    def __enter__(self):
        os.makedirs(LEDGER, exist_ok=True)
        self.f = open(os.path.join(LEDGER, '.lock'), 'a+b')
        if os.name == 'nt':
            import msvcrt
            while True:
                try:
                    msvcrt.locking(self.f.fileno(), msvcrt.LK_NBLCK, 1)
                    break
                except OSError:
                    time.sleep(0.02)
        else:
            import fcntl
            fcntl.flock(self.f.fileno(), fcntl.LOCK_EX)
        return self

    def __exit__(self, *exc):
        if os.name == 'nt':
            import msvcrt
            with suppress(OSError):
                self.f.seek(0)
                msvcrt.locking(self.f.fileno(), msvcrt.LK_UNLCK, 1)
        self.f.close()


def booked(path, gb, pid=None, owner=None, cores=0):
    """A job in the ledger: `gb` its commit, `pid` its process once started, held while `owner`
    (this relay) lives; `cores` a container's, or a job's past one, counted as so many processes in place of its
    own (tools.dev.ledger); the tree it runs."""
    folder = os.path.dirname(path) or LEDGER
    os.makedirs(folder, exist_ok=True)
    part = os.path.join(folder, '.%s.part' % os.path.basename(path))
    with open(part, 'w', encoding='utf-8') as f:
        json.dump({'relay': owner or os.getpid(), 'gb': gb, 'pid': pid, 'cores': cores, 'tree': str(ROOT.parent)}, f)
    os.replace(part, path)


def waiting(path, count=0, since=None, gb=0.0):
    """A ticket in the ledger beside the bookings while a relay or a container waits for room - `count` jobs or
    cores and `gb` of commit, waiting since `since` (now where none is told: laid anew at every poll, a ticket read
    0 s for ever) -, gone when `count` is 0: the queue as `tools.dev.ledger.said` lists it (`.wait-<relay>-<name>`)."""
    if not count:
        with suppress(OSError):
            os.remove(path)
        return
    with suppress(OSError):
        with open(path + '.part', 'w', encoding='utf-8') as f:
            json.dump({'relay': os.getpid(), 'count': count, 'since': since or time.time(), 'gb': gb}, f)
        os.replace(path + '.part', path)


def queued(ledger=LEDGER, mine=None, before=None):
    """The cores the machine's wide requests wait for - a relay's job of more than one core, a container: their
    tickets (`waiting`), a relay's count of jobs apart - of owners that live, less the ticket `mine` and, `before`
    given, those waiting since later: what a narrower or a later request leaves free, so a wide one is not passed
    by for ever - every core let go went to the next narrow job that asked."""
    total = 0
    with suppress(OSError):
        for name in os.listdir(ledger):
            path = os.path.join(ledger, name)
            if not name.startswith('.wait-') or name.endswith(('-jobs', '.part')) or path == mine:
                continue
            with suppress(OSError, ValueError, KeyError), open(path, encoding='utf-8') as f:
                held = json.load(f)
                if census.alive(held['relay']) and (before is None or held['since'] < before):
                    total += int(held['count'])
    return total


def tally(ledger=LEDGER):
    """(what is taken of the width - every relay's live job processes and the cores booked: a container's, an
    emulator's, a job's sent along, each counted once -; the commit every relay's running jobs have still to
    take, GB; how many run; the cores booked), machine-wide, the machine's processes read once
    (`census.Snapshot`): a job counted at its booking less what its tree holds, a container at its booking; one
    whose relay is gone, dropped."""
    snap = census.Snapshot()
    owed, running, pids, cores, own = 0.0, 0, [], 0, []
    with suppress(OSError):
        for name in os.listdir(ledger):
            if name.startswith('.'):
                continue
            path = os.path.join(ledger, name)
            try:
                with open(path, encoding='utf-8') as f:
                    job = json.load(f)
            except (OSError, ValueError):
                continue
            # Asked live, not off the snapshot: a booking laid after it was read must not pass for dead.
            if not census.alive(job['relay']) or (job['pid'] is not None and not census.alive(job['pid'])):
                with suppress(OSError):
                    os.remove(path)
                continue
            held = snap.private_gb(snap.tree([job['pid']])) if job['pid'] is not None else None
            owed += max(0.0, job['gb'] - (held or 0.0))
            if job.get('cores'):
                cores += int(job['cores'])
                own += [job['pid']] * (job['pid'] is not None)
                continue
            running += 1
            pids += [job['pid']] * (job['pid'] is not None)
    return len(snap.tree(pids, own)) + running - len(pids) + cores, owed, running, cores


def admitted(gb, start, name, busy=None):
    """A process begun outside a relay (`start()`) where the machine's commit holds `gb` over every
    relay's owed and RESERVE_GB and `busy()` is not, booked in the ledger as `name` under its lock
    while it lives: (the process, its booking), or None."""
    with _Locked():
        free = census.free_commit_gb()
        if (busy is not None and busy()) or (free is not None and free - tally()[1] - RESERVE_GB < gb):
            return None
        proc = start()
        booking = os.path.join(LEDGER, '%s-%d-%d' % (name, os.getpid(), proc.pid))
        with suppress(OSError):
            booked(booking, gb, proc.pid, proc.pid)
        return proc, booking


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
    (`pick`) less those a `--but=word` names - a suite's rest beside a job of their own -, and of
    them the share `--shard k/n` gives - every n-th from the k-th, so n shards side by side run
    each test once."""
    words, shard, without, args = [], None, [], list(argv)
    while args:
        arg = args.pop(0)
        if arg == '--shard' and args:
            arg = '--shard=' + args.pop(0)
        if arg.startswith('--shard='):
            m = SHARD_RE.match(arg.split('=', 1)[1])
            if not m or not 1 <= int(m.group(1)) <= int(m.group(2)):
                sys.exit('--shard k/n, 1 <= k <= n: %s' % arg)
            shard = (int(m.group(1)), int(m.group(2)))
        elif arg.startswith('--but='):
            without.append(arg.split('=', 1)[1])
        else:
            words.append(arg)
    tests = [test for test in pick(tests, words) if not any(word in test.__name__[5:] for word in without)]
    return tests[shard[0] - 1::shard[1]] if shard else tests


def watchdog(seconds):
    """Every thread's stack printed and the process gone `seconds` on: a hung suite says where,
    and its emulators go with it (tools.emu.emulator's job object)."""
    faulthandler.dump_traceback_later(seconds, exit=True)


def run(job, env=None, ledger=None, bench=False, procs=None):
    """One job in a process of its own on a core of its own while it runs (`placed.claim`: moved among the batch's
    cores by the system, a job of 28 touched 40 in 5 s, nine moves a second, a tenth less work done), the rest of
    `job.cores` taken with it and sent along (SENT) for the Renode it starts, all of them or none (on its host's
    one core Renode answered late: tools.emu.probes.Cored), stdout and stderr in one, killed with its tree at its
    time: (output, exit code - None out of time -, seconds); `ledger` its booking, its process and the cores
    sent named in it while it runs; `bench`, above normal priority from its start (Windows')."""
    from machine.process import placed
    started = time.monotonic()
    group = {'start_new_session': True} if os.name != 'nt' else {}
    if bench and os.name == 'nt':
        group['creationflags'] = getattr(subprocess, 'ABOVE_NORMAL_PRIORITY_CLASS')
    claims = [placed.claim() for _ in range(max(1, job.cores))]
    if not all(mask for mask, _held in claims[1:]):
        for _mask, held in claims[1:]:
            placed.free(held)
        del claims[1:]
    sent = {SENT: '%x' % sum(mask for mask, _held in claims[1:])}
    out, code = '', 1
    try:
        proc = subprocess.Popen(job.argv, cwd=str(ROOT), env=dict(os.environ if env is None else env, **sent),
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, encoding='utf-8',
                                errors='replace', **group)
        placed.bind(proc.pid, claims[0][0])
        if procs is not None:
            procs.add(proc.pid)
        if ledger is not None:
            with suppress(OSError), _Locked():
                booked(ledger, job.gb, proc.pid, cores=len(claims) if len(claims) > 1 else 0)
        try:
            out, _ = proc.communicate(timeout=job.timeout)
            code = proc.returncode
        except subprocess.TimeoutExpired:
            out, _ = last_words(proc)
            code = None
        finally:
            if procs is not None:
                procs.discard(proc.pid)
    finally:
        for _mask, held in claims:
            placed.free(held)
    return out or '', code, time.monotonic() - started


waiting_ticket = waiting


def relay(jobs, batons=None, bench=False, ledger=LEDGER):
    """`jobs` from a queue in their order, the longest first where the caller knows: `batons` -
    a physical core each - each taking the first job whose commit fits the free commit less what
    every relay's running jobs have still to take (the ledger, machine-wide), over RESERVE_GB -
    an idle host fits anything -, while every relay's jobs with their children, and the cores every container of
    the gate holds (tools.dev.ledger), stay under the batch's widest (`tally`, `background.widths`), OpenBLAS
    on one thread in each; a job ending
    hands its baton to the next at once. `bench` a benchmark's, on a desk: each job above normal priority
    from its start and `placed.BENCH` set for every process of the robot's own it starts (`placed.raised`),
    its wall times less at the mercy of the host's other loads - a runner has none, its jobs run as they are.
    `ledger` the machine's, or a folder of its own for a relay that asks nothing of the machine (a test's).
    (job, output, exit code - None out of time -, seconds) as each ends. A relay whose console answers no more
    (`census.console`: who started the relay stopped it) ends as stopped, KeyboardInterrupt, its jobs' trees
    killed and none told: every job started after ended at once, 110 suites burnt as red in 11 s."""
    bench, console = bench and not background.runner(), census.console()
    waiting = collections.deque(jobs)
    count = len(waiting)
    held = {'running': 0, 'locks': set(), 'n': 0, 'asked': 0.0, 'idle': 0.0, 'since': 0.0}
    tickets = [os.path.join(ledger, '.wait-%d-%s' % (os.getpid(), kind)) for kind in ('jobs', 'wide')]
    turn = threading.Condition()
    ended = queue.Queue()
    env = environment()
    if bench:
        from machine.process.placed import BENCH
        env[BENCH] = '1'

    def fits(job, room):
        """The job's lock free - an 'alone' job waits for this relay to empty, and holds it - and its
        commit within `room` (None: anything)."""
        if 'alone' in held['locks']:
            return False
        if job.lock == 'alone':
            return not held['running']
        if job.lock is not None and job.lock in held['locks']:
            return False
        return room is None or job.gb <= room

    ticket = waiting_ticket

    def take():
        """(the next job that fits, its booking); None once none is left. Waits while those left do not, one
        baton asking the ledger a second for all that wait - each baton asking, a relay of 28 held the ledger's
        lock without a gap - and at once when a job of this relay ends: another relay's job may end. The relay's
        first job of more than one core waits its turn among the machine's wide requests (`queued`), and every
        other job leaves their cores free."""
        with turn:
            while waiting:
                if time.monotonic() - held['asked'] >= POLL_S:
                    with _Locked():
                        free = census.free_commit_gb()
                        used, owed, running, cores = tally(ledger)
                        room = None if free is None or not (running or cores) else free - owed - RESERVE_GB
                        width = background.widths()[-1]
                        wide = next((j for j in waiting if j.cores > 1), None)
                        ahead = queued(ledger), queued(ledger, tickets[1], held['since'] or time.time())
                        # A job's cores within the width beside what is taken and what the wide requests ahead
                        # of it wait for; one wider than the width, the width.
                        job = next((j for j in waiting if fits(j, room)
                                    and used + min(j.cores, width) + ahead[j is wide] <= width), None)
                        if job is not None:
                            waiting.remove(job)
                            held['running'] += 1
                            held['n'] += 1
                            if job.lock is not None:
                                held['locks'].add(job.lock)
                            path = os.path.join(ledger, '%d-%d' % (os.getpid(), held['n']))
                            with suppress(OSError):
                                booked(path, job.gb, cores=job.cores if job.cores > 1 else 0)
                        # The tickets: the jobs that wait, since the relay last took one; its first wide job's
                        # cores, since that job came first - the next one's turn begins when the first has gone.
                        held['idle'] = 0.0 if job else held['idle'] or time.time()
                        ticket(tickets[0], 0 if job else len(waiting), held['idle'],
                               min((j.gb for j in waiting), default=0.0))
                        if job is wide:
                            held['since'] = 0.0
                        wide = next((j for j in waiting if j.cores > 1), None)
                        held['since'] = (held['since'] or time.time()) if wide else 0.0
                        ticket(tickets[1], min(wide.cores, width) if wide else 0, held['since'], wide.gb if wide else 0.0)
                    if job is not None:
                        return job, path
                    held['asked'] = time.monotonic()
                turn.wait(timeout=POLL_S)
            for path in tickets:
                ticket(path, 0)
            return None

    def baton():
        while (taken := take()) is not None:
            job, path = taken
            try:
                ended.put((job,) + run(job, env, path, bench, procs))
            except OSError as exc:
                ended.put((job, '%s: %s' % (type(exc).__name__, exc), 1, 0.0))
            finally:
                with suppress(OSError):
                    os.remove(path)
                with turn:
                    held['running'] -= 1
                    held['locks'].discard(job.lock)
                    held['asked'] = 0.0             # room where the job ran: the ledger asked at once
                    turn.notify_all()

    procs = set()
    for _ in range(max(1, min(count, batons or background.cores()))):
        threading.Thread(target=baton, name='baton', daemon=True).start()
    try:
        for _ in range(count):
            got = None
            while got is None:
                with suppress(queue.Empty):
                    got = ended.get(timeout=POLL_S)
                if console and not census.console():
                    raise KeyboardInterrupt("the relay's console is gone: who started the relay stopped it")
            yield got
    finally:
        with turn:
            waiting.clear()
            turn.notify_all()
        for pid in list(procs):
            kill_tree(pid)


def run_groups(path, groups, timeout, batons=None):
    """A suite's `groups` on the relay, `python path group` each, the longest first as given:
    each group's lines as it ends, and one tally over them, a group out of time or ending
    without a tally one failure. `timeout`, s: one for all, or a group's own by name. The exit
    code."""
    passed = failed = 0
    needs = asked(path)
    jobs = [Job(group, [sys.executable, '-X', 'utf8', '-u', str(path), group], WORKER_GB,
                timeout[group] if isinstance(timeout, dict) else timeout, cores=1 + needs.get(group, 0))
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
            sys.stdout.flush()
            break
        else:
            passed += int(tally.group(1))
            failed += int(tally.group(2))
        sys.stdout.flush()
        if failed or code != 0:
            break
    print('\n%d passed, %d failed' % (passed, failed))
    return 1 if failed else 0
