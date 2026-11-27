#!/usr/bin/env python3
"""The robot's processes placed by its host's topology (`machine.process.placed`), on hosts of other shapes than this one.

The placement policy is of masks alone: asked here of a host of one cache for all its cores, of a cache a core, of
fast and slow cores each cluster its own cache, of two memory nodes of two caches each; Linux's sysfs read off a tree
laid in a folder; and live where this host tells its topology - two of the robot's loops each take a cache group none
else holds, another process leaves both; a job on the relay is sent its emulator's cores and Renode bound to them
(`tools.dev.focus.run`, `tools.emu.probes.Cored`), and a relay whose console is gone stops (`tools.dev.focus.relay`)."""
import json
import os
import signal
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen
from gynoid_kit import Report

#: A child of this suite: placed as told, its processors and its loop's said, alive till its stdin ends.
CHILD = ('import sys; from machine.process import placed; ok = placed.placed(%s); '
         'print("%%d %%x %%x" %% (ok, placed.now(), placed._MINE[1] if len(placed._MINE) > 1 else 0), flush=True); '
         'sys.stdin.read()')
#: A relay of two batons over eight jobs that sleep, 'ready' once up and begun on a line of its stdin: each job's
#: exit code told as the job ends, 'stopped' where the relay is.
RELAY = ('import sys, tempfile; from tools.dev import focus; print("ready", flush=True); input()\n'
         'jobs = [focus.Job("j%d" % k, [sys.executable, "-c", "import time; time.sleep(4)"], 0.1, 60.0) for k in range(8)]\n'
         'try:\n'
         '    with tempfile.TemporaryDirectory() as folder:\n'
         '        for _job, _out, code, _took in focus.relay(jobs, batons=2, ledger=folder):\n'
         '            print("ended %s" % code, flush=True)\n'
         'except KeyboardInterrupt as stop:\n'
         '    print("stopped: %s" % stop, flush=True)\n')


def _host(cores, threads, caches):
    """([(class, a core's mask)], [a cache's mask]) of a host told: `cores` [class a core], each of `threads`
    logical processors side by side, `caches` [[core, ...] a cache]."""
    wide = (1 << threads) - 1
    found = [(klass, wide << (threads * k)) for k, klass in enumerate(cores)]
    return found, [sum(found[k][1] for k in group) for group in caches]


def test_the_robots_loop_takes_a_cache_of_its_own_on_hosts_of_other_shapes(report):
    """`placed.apart`, `chosen` and `left` on four hosts: the robot's loop's group, its first core, another
    process's."""
    from machine.process import placed
    every = lambda _group: True
    none = lambda _group: False
    # one cache for all of eight cores of two threads: the robot's loop's process on all its cores,
    # another off its first
    found, caches = _host([0] * 8, 2, [range(8)])
    own, fast = placed.apart(found)
    mine = placed.chosen(caches, own, every, fast)
    report.check('one cache for all the robot\'s cores: its loop on them all, another process off its first core alone',
                 own == fast == 0x5554 and mine == own and placed.left(caches, own, every) == own & ~(mine & -mine)
                 and placed.left(caches, own, none) == own, '%x, another %x' % (mine, placed.left(caches, own, every)))
    # two cores or fewer: leaving the first core leaves at most one processor, so the robot's processes are left
    # unplaced
    two = _host([0] * 2, 2, [range(2)])
    report.check('two cores or fewer: leaving the first core leaves one processor at most, nothing bound',
                 placed.apart(two[0]) == (0x4, 0x4) and not placed.placed(True, two) and not placed.placed(False, two))
    # a cache a core: no group holds two of the robot's processors, its loop is left among them all
    found, caches = _host([0] * 4, 2, [[k] for k in range(4)])
    own, fast = placed.apart(found)
    report.check('a cache a core: none taken, every process left on the robot\'s cores',
                 placed.chosen(caches, own, every, fast) == 0 and placed.left(caches, own, none) == own == 0x54)
    # four slow cores then four fast, a cache a cluster, one thread each: the robot's loop's group the fast
    # cores'
    found, caches = _host([1, 1, 1, 1, 4, 4, 4, 4], 1, [range(4), range(4, 8)])
    own, fast = placed.apart(found)
    mine = placed.chosen(caches, own, every, fast)
    report.check('fast and slow cores, a cache a cluster: the robot\'s loop on the fast ones, the slow left to the rest',
                 fast == 0xf0 and mine == 0xf0 and placed.left(caches, own, lambda g: g == 0xf0) == 0x0e,
                 '%x of fast %x' % (mine, fast))
    # fast cores first and one cache for all: the robot's loop's thread on the first fast core of its own, not
    # the lowest
    found, caches = _host([1, 1, 4, 4], 2, [range(4)])
    own, fast = placed.apart(found)
    mine = placed.chosen(caches, own, every, fast)
    report.check('fast and slow cores on one cache: the robot\'s loop\'s thread on its first fast core',
                 mine == own == 0x54 and (mine & fast) & -(mine & fast) == 0x10, '%x' % ((mine & fast) & -(mine & fast)))
    # two memory nodes of two caches of four cores: the robot's loops take the last first, each its own, a
    # fifth none
    found, caches = _host([0] * 16, 2, [range(4), range(4, 8), range(8, 12), range(12, 16)])
    own, fast = placed.apart(found)
    held = []
    for _ in range(5):
        held.append(placed.chosen(caches, own, lambda g: g & own not in held, fast))
    report.check('two nodes of two caches: the robot\'s loops each a cache of its own, the last first, a fifth none',
                 held == [0x55000000, 0x00550000, 0x00005500, 0x00000054, 0]
                 and placed.left(caches, own, lambda g: g & own in held[:2]) == 0x00005554,
                 ' '.join('%x' % m for m in held))


def test_each_process_takes_a_core_of_its_own_on_hosts_of_other_shapes(report):
    """`placed.core` on hosts laid here: each process a core of its own, every cache's first before any's second,
    the last cache first, the fast before the slow, none where all are taken; a host held to a batch's processors
    keeps every core of them - only the system's first, processor 0's, is left -; who starts the robot tells them
    (`told`)."""
    from machine.process import placed
    found, caches = _host([0] * 16, 2, [range(4), range(4, 8), range(8, 12), range(12, 16)])
    own, _fast = placed.apart(found)
    taken = []
    for _ in range(16):
        taken.append(placed.core(found, caches, own, lambda mask: mask not in taken))
    cores = [next(k for k, (_c, m) in enumerate(found) if m == mask) if mask else None for mask in taken]
    report.check('four caches of four cores: each cache\'s first core, the last cache first, then each\'s second; '
                 'the system\'s first core none\'s, a sixteenth none',
                 cores == [12, 8, 4, 1, 13, 9, 5, 2, 14, 10, 6, 3, 15, 11, 7, None]
                 and taken[0] == 0x3000000, ' '.join(map(str, cores)))
    found, caches = _host([1, 1, 1, 1, 4, 4, 4, 4], 1, [range(4), range(4, 8)])
    own, _fast = placed.apart(found)
    taken = []
    for _ in range(8):
        taken.append(placed.core(found, caches, own, lambda mask: mask not in taken))
    report.check('fast and slow cores: the fast taken first, the slow after, the first none\'s',
                 taken == [0x10, 0x20, 0x40, 0x80, 0x2, 0x4, 0x8, 0], ' '.join('%x' % m for m in taken))
    found, caches = _host([0] * 8, 2, [range(4), range(4, 8)])
    held = placed.held(found, caches, 0xfff0)
    report.check("a host held to a batch's processors, the pages' first two cores kept: all six of its cores the robot's own",
                 placed.apart(held[0])[0] == 0x5550 and placed.apart(found)[0] == 0x5554,
                 '%x' % placed.apart(held[0])[0])
    report.check('who starts the robot tells its processors in hex; none told or none read: 0',
                 placed.told({placed.GIVEN: 'fff0'}) == 0xfff0 and placed.told({}) == 0
                 and placed.told({placed.GIVEN: 'none'}) == 0)


def test_a_batch_binds_its_jobs_each_to_a_core(report):
    """Live, where this host tells its topology and binds a process: two cores claimed at once are two, a process
    started bound to each holds its core's processors alone; a claim held is told held, let go none's - asked in a
    name of this run's own, a batch's batons claiming the host's cores beside it."""
    from machine.process import placed
    first, held_first = placed.claim()
    second, held_second = placed.claim()
    if not (first and second):
        placed.free(held_first)
        placed.free(held_second)
        report.check('a batch binds its jobs each to a core (this host tells none to bind, or none is free)', True,
                     '%x %x' % (first, second))
        return
    env = dict(os.environ, PYTHONPATH=os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    said = []
    try:
        for mask in (first, second):
            child = subprocess.Popen([sys.executable, '-X', 'utf8', '-c', 'import sys; from machine.process import '
                                      'placed; sys.stdin.readline(); print("%x" % placed.now(), flush=True)'],
                                     stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, env=env)
            bound = placed.bind(child.pid, mask)
            said.append((bound, int(child.communicate('\n')[0].strip() or '0', 16)))
    finally:
        placed.free(held_first)
        placed.free(held_second)
    kind = 'probe-%d' % os.getpid()
    probe = placed._take(first, kind)
    told = probe is not None and placed._taken(first, kind) and placed._take(first, kind) is None
    placed.free(probe)
    report.check("two claimed at once, two cores; each process bound to its own alone; a claim held told held and "
                 "none's once let go",
                 first != second and not first & second and said == [(True, first), (True, second)]
                 and told and not placed._taken(first, kind), '%x %x %s held %s' % (first, second, said, told))


def test_linux_tells_its_topology_in_sysfs(report):
    """`placed.sysfs` on a tree laid in a folder: four cores of two threads, the last two of twice the capacity, a
    last-level cache a pair of cores; its lists of processors read (`listed`)."""
    from machine.process import placed
    with tempfile.TemporaryDirectory() as root:
        for cpu in range(8):
            core = cpu % 4
            for path, text in (('topology/thread_siblings_list', '%d,%d' % (core, core + 4)),
                               ('cpu_capacity', '%d' % (1024 if core >= 2 else 512)),
                               ('cache/index0/level', '1'), ('cache/index0/shared_cpu_list', '%d,%d' % (core, core + 4)),
                               ('cache/index3/level', '3'),
                               ('cache/index3/shared_cpu_list', '0-1,4-5' if core < 2 else '2-3,6-7')):
                full = os.path.join(root, 'cpu%d' % cpu, *path.split('/'))
                os.makedirs(os.path.dirname(full), exist_ok=True)
                with open(full, 'w', encoding='ascii') as f:
                    f.write(text + '\n')
        found, caches = placed.sysfs(root)
        empty = placed.sysfs(os.path.join(root, 'none'))
    own, fast = placed.apart(found)
    report.check("Linux's sysfs: the robot's cores, their classes and their last-level caches as it tells them",
                 found == [(512, 0x11), (512, 0x22), (1024, 0x44), (1024, 0x88)] and caches == [0x33, 0xcc]
                 and (own, fast) == (0x0e, 0x0c) and placed.chosen(caches, own, lambda g: True, fast) == 0x0c
                 and empty == ([], []), '%s %s' % (found, ['%x' % c for c in caches]))
    report.check("a sysfs list of processors read", placed.listed('0-3,8,10-11\n') == 0xd0f and placed.listed('5') == 0x20)


def test_a_container_s_processors_are_its_cpuset(report):
    """`placed.given` on control groups laid in a folder - cgroup v2's at a container's root, a group telling none
    walked up to one that does, v1's cpuset, none at all -; `placed.held`: a host of four cores of two threads held
    to two of them, both threads, their cache with them."""
    from machine.process import placed
    told = []
    for files in ({'proc/self/cgroup': '0::/', 'sys/fs/cgroup/cpuset.cpus.effective': '2-3,6-7'},
                  {'proc/self/cgroup': '0::/user.slice/a.scope', 'sys/fs/cgroup/user.slice/cpuset.cpus.effective': '0-1'},
                  {'proc/self/cgroup': '4:cpuset:/docker/x\n3:memory:/docker/x',
                   'sys/fs/cgroup/cpuset/cpuset.effective_cpus': '1,3'},
                  {'proc/self/cgroup': '0::/'}):
        with tempfile.TemporaryDirectory() as root:
            for path, text in files.items():
                full = os.path.join(root, *path.split('/'))
                os.makedirs(os.path.dirname(full), exist_ok=True)
                with open(full, 'w', encoding='ascii') as f:
                    f.write(text + '\n')
            told.append(placed.given(root))
    report.check("a control group's processors: v2's at a container's root, walked up to, v1's; none told: 0",
                 told == [0xcc, 0x3, 0xa, 0], str(['%x' % mask for mask in told]))
    found, caches = [(0, 0x11), (0, 0x22), (0, 0x44), (0, 0x88)], [0x33, 0xcc]
    report.check('a host held to two of its cores, both threads, their cache with them; told none: as it is',
                 placed.held(found, caches, 0xcc) == ([(0, 0x44), (0, 0x88)], [0xcc])
                 and placed.held(found, caches, 0) == (found, caches), str(placed.held(found, caches, 0xcc)))


def test_two_loops_take_each_a_cache_and_another_process_leaves_both(report):
    """Live, where this host tells its topology and holds a thread or a process to its processors: two processes
    placed as the robot's loop's, a third as another, all alive at once - each loop's processors within one
    cache group and no other's, the third's off both."""
    from machine.process import placed
    found, caches = placed.topology()
    own, fast = placed.apart(found) if found else (0, 0)
    fits = [g for g in caches if (g & own) & ((g & own) - 1)]
    if len(fits) < 3:
        report.check('its loops each take a cache of their own (this host tells fewer than three such)', True,
                     '%d caches of two of the robot\'s cores' % len(fits))
        return
    env = dict(os.environ, PYTHONPATH=os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    alive = []
    try:
        for role in ('True', 'True', ''):
            child = subprocess.Popen([sys.executable, '-X', 'utf8', '-c', CHILD % role], stdin=subprocess.PIPE,
                                     stdout=subprocess.PIPE, text=True, env=env)
            said = child.stdout.readline() if child.stdout is not None else ''
            alive.append((child, [int(x, 16) for x in said.split()]))
    finally:
        for child, _said in alive:
            child.communicate('')
    (_a, (ok_a, now_a, mine_a)), (_b, (ok_b, now_b, mine_b)), (_c, (ok_c, now_c, _none)) = alive
    if not (ok_a and ok_b and ok_c and mine_a and mine_b):
        report.check("the robot's loops each take a cache of their own (this run's job holds its processes, or none is free)",
                     not (mine_a or mine_b) or (ok_a and ok_b), 'placed %d %d %d' % (ok_a, ok_b, ok_c))
        return
    cores = [mask for _klass, mask in found]
    no_smt_c = all(((now_c & core) & ((now_c & core) - 1)) == 0 for core in cores)
    report.check('two of the robot\'s loops each on a cache group of its own, another process off both without SMT '
                 'sibling sharing',
                 ok_a and ok_b and ok_c and mine_a and mine_b and now_a == mine_a and now_b == mine_b
                 and any(mine_a == g & own for g in caches) and any(mine_b == g & own for g in caches)
                 and not mine_a & mine_b and not now_c & (mine_a | mine_b) and now_c and no_smt_c,
                 'loops %x %x, another %x' % (now_a, now_b, now_c))


def test_a_job_is_sent_its_emulator_s_cores(report):
    """`focus.run`: a job of three cores - its suite asked for a Renode of two machines - is held to a whole core
    of its own where one is free, sent the two others (`SENT`, a mask in its environment) where they are, and
    booked as three in the ledger with them; a job sent none - of one core, or the two not free - is booked as its
    process. `probes.Cored` binds a Renode to the cores sent it, unbooked; with none sent, to as many it takes
    itself where they are free, else to any of the processors given, booked in the ledger while it runs - and
    counted there by its cores alone, its process left out of its job's (`focus.tally`)."""
    from machine.process import placed
    from tools.dev import census, focus, ledger
    from tools.emu.probes import Cored
    cores = [mask for _klass, mask in placed.topology()[0]]
    if len(cores) < 2:
        report.check("a job is sent its emulator's cores (this host tells no two cores to bind to)", True, str(cores))
        return

    def whole(mask):
        """The cores `mask` holds whole; none where it splits one."""
        return [] if any(core & mask not in (0, core) for core in cores) else [c for c in cores if c & mask]

    env = dict(os.environ, PYTHONPATH=os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    probe = [sys.executable, '-X', 'utf8', '-c', 'import os; from machine.process import placed; '
             'print("%%x %%s" %% (placed.now(), os.environ["%s"]))' % focus.SENT]
    waits = [sys.executable, '-X', 'utf8', '-c', 'import sys; from machine.process import placed; '
             'sys.stdin.readline(); print("%x" % placed.now(), flush=True)']
    with tempfile.TemporaryDirectory() as folder:
        paths = [os.path.join(folder, '%d-%d' % (os.getpid(), k)) for k in (1, 2)]
        said = [focus.run(focus.Job(name, probe, cores=n), env, path)[0].split()
                for name, n, path in zip(('asked', 'plain'), (3, 1), paths)]
        booked = []
        for path in paths:
            with open(path, encoding='utf-8') as f:
                booked.append(json.load(f)['cores'])

        def bound(given, port):
            """A stand-in for a Renode of two machines bound as one is, `given` sent it: (its processors, the
            cores it took, whether it lay in the ledger while it ran and is gone after, the processes the ledger
            leaves out of this one's tree, booked as a job, for the cores booked, the view's lines of it)."""
            child = subprocess.Popen(waits, stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, env=env)
            cored = Cored(child.pid, 2, port, sent=given, ledger=folder)
            booking, took = cored.booking, len(cored.holds)
            lay = booking is not None and os.path.exists(booking)
            job = os.path.join(folder, '%d-9' % os.getpid())
            focus.booked(job, 0.0, os.getpid())
            left = len(census.Snapshot().tree([os.getpid()])) + 2 * lay - focus.tally(folder)[0]
            os.remove(job)
            view = [line for line in ledger.said(folder) if line.startswith('emulator')]
            mask = int(child.communicate('\n')[0].strip() or '0', 16)
            cored.free()
            return mask, took, lay and booking is not None and not os.path.exists(booking), left, view

        pair = cores[-1] | cores[-2]
        with_sent, with_none = bound(pair, 1), bound(0, 2)
    (mine, sent), (_plain, none) = [(int(mask, 16), int(rest, 16)) for mask, rest in said]
    held = len(whole(mine)) == 1 or mine == placed.now()
    along = sent == 0 or (len(whole(sent)) == 2 and not sent & mine)
    itself = (with_none[1] == 2 and len(whole(with_none[0])) == 2) or (with_none[1] == 0 and with_none[0] == sum(cores))
    listed = [line for line in with_none[4] if line.startswith('emulator renode-2 (process %d, ' % os.getpid())
              and line.endswith('): 2 cores, 0.0 GB')]
    report.check('a job of three cores held to one, sent two apart from it where they are free and booked as three '
                 'with them; a job of one sent none; a Renode bound to the cores sent it, unbooked; with none sent to '
                 'two it takes itself or to any given, booked while it runs, counted by its cores alone and listed '
                 'with the process that started it',
                 held and along and none == 0 and booked == [3 if sent else 0, 0]
                 and with_sent == (pair, 0, False, 0, []) and itself and with_none[2] and with_none[3] >= 1
                 and len(listed) == 1,
                 'mine %x, sent %x, plain sent %x, booked %s; sent %x: %s; none sent: %s of %x'
                 % (mine, sent, none, booked, pair, with_sent, with_none, sum(cores)))


def test_a_relay_whose_console_is_gone_stops(report):
    """`focus.relay` begun in a console whose host is then killed, as with who started the relay: the relay ends as
    stopped, its running jobs killed and none told - the relay ran on before, each job started after ending at once
    (0xC0000142), 110 suites told red in 11 s (`census.console`; Windows' consoles alone)."""
    from tools.dev import census, focus
    if os.name != 'nt':
        report.check("a relay whose console is gone stops (Windows' consoles alone)", True)
        return
    import psutil
    proc = subprocess.Popen([sys.executable, '-X', 'utf8', '-c', RELAY], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, env=focus.environment(),
                            creationflags=getattr(subprocess, 'CREATE_NO_WINDOW'))
    assert proc.stdin is not None and proc.stdout is not None
    proc.stdout.readline()                            # up and waiting, a console of its own
    hosts =[pid for pid in census.Snapshot().kids.get(proc.pid, []) if psutil.Process(pid).name().lower() == 'conhost.exe']
    proc.stdin.write('\n')
    proc.stdin.flush()
    time.sleep(2.0)                                   # two jobs running
    for pid in hosts:
        os.kill(pid, signal.SIGTERM)
    out, _ = proc.communicate(timeout=60)
    report.check('a relay whose console is gone ends as stopped, its jobs killed, none told as ended',
                 len(hosts) == 1 and 'stopped' in out and 'ended' not in out, '%d hosts: %r' % (len(hosts), out[-300:]))


ROSTER = (test_the_robots_loop_takes_a_cache_of_its_own_on_hosts_of_other_shapes,
          test_each_process_takes_a_core_of_its_own_on_hosts_of_other_shapes, test_a_batch_binds_its_jobs_each_to_a_core,
          test_linux_tells_its_topology_in_sysfs, test_a_container_s_processors_are_its_cpuset,
          test_two_loops_take_each_a_cache_and_another_process_leaves_both,
          test_a_job_is_sent_its_emulator_s_cores, test_a_relay_whose_console_is_gone_stops)


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
