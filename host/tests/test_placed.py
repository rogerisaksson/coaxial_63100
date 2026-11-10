#!/usr/bin/env python3
"""Her processes placed by her host's topology (`machine.process.placed`), on hosts of other shapes than this one.

Its policy is of masks alone: asked here of a host of one cache for all its cores, of a cache a core, of fast and
slow cores each cluster its own cache, of two memory nodes of two caches each; Linux's sysfs read off a tree laid in
a folder; and live where this host tells its topology - two of her loops each take a cache group none else holds,
another process leaves both."""
import os
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen
from gynoid_kit import Report

#: A child of this suite: placed as told, its processors and its loop's said, alive till its stdin ends.
CHILD = ('import sys; from machine.process import placed; ok = placed.placed(%s); '
         'print("%%d %%x %%x" %% (ok, placed.now(), placed._MINE[1] if len(placed._MINE) > 1 else 0), flush=True); '
         'sys.stdin.read()')


def _host(cores, threads, caches):
    """([(class, a core's mask)], [a cache's mask]) of a host told: `cores` [class a core], each of `threads`
    logical processors side by side, `caches` [[core, ...] a cache]."""
    wide = (1 << threads) - 1
    found = [(klass, wide << (threads * k)) for k, klass in enumerate(cores)]
    return found, [sum(found[k][1] for k in group) for group in caches]


def test_her_loop_takes_a_cache_of_its_own_on_hosts_of_other_shapes(report):
    """`placed.apart`, `chosen` and `left` on four hosts: her loop's group, its first core, another process's."""
    from machine.process import placed
    every = lambda _group: True
    none = lambda _group: False
    # one cache for all of eight cores of two threads: her loop's process on all her cores, another off its first
    found, caches = _host([0] * 8, 2, [range(8)])
    own, fast = placed.apart(found)
    mine = placed.chosen(caches, own, every, fast)
    report.check('one cache for all her cores: her loop on them all, another process off its first core alone',
                 own == fast == 0x5554 and mine == own and placed.left(caches, own, every) == own & ~(mine & -mine)
                 and placed.left(caches, own, none) == own, '%x, another %x' % (mine, placed.left(caches, own, every)))
    # a cache a core: no group holds two of her processors, her loop is left among them all
    found, caches = _host([0] * 4, 2, [[k] for k in range(4)])
    own, fast = placed.apart(found)
    report.check('a cache a core: none taken, every process left on her cores',
                 placed.chosen(caches, own, every, fast) == 0 and placed.left(caches, own, none) == own == 0x54)
    # four slow cores then four fast, a cache a cluster, one thread each: her loop's group the fast cores'
    found, caches = _host([1, 1, 1, 1, 4, 4, 4, 4], 1, [range(4), range(4, 8)])
    own, fast = placed.apart(found)
    mine = placed.chosen(caches, own, every, fast)
    report.check('fast and slow cores, a cache a cluster: her loop on the fast ones, the slow left to the rest',
                 fast == 0xf0 and mine == 0xf0 and placed.left(caches, own, lambda g: g == 0xf0) == 0x0e,
                 '%x of fast %x' % (mine, fast))
    # fast cores first and one cache for all: her loop's thread on the first fast core of hers, not the lowest
    found, caches = _host([1, 1, 4, 4], 2, [range(4)])
    own, fast = placed.apart(found)
    mine = placed.chosen(caches, own, every, fast)
    report.check('fast and slow cores on one cache: her loop\'s thread on her first fast core',
                 mine == own == 0x54 and (mine & fast) & -(mine & fast) == 0x10, '%x' % ((mine & fast) & -(mine & fast)))
    # two memory nodes of two caches of four cores: her loops take the last first, each its own, a fifth none
    found, caches = _host([0] * 16, 2, [range(4), range(4, 8), range(8, 12), range(12, 16)])
    own, fast = placed.apart(found)
    held = []
    for _ in range(5):
        held.append(placed.chosen(caches, own, lambda g: g & own not in held, fast))
    report.check('two nodes of two caches: her loops each a cache of its own, the last first, a fifth none',
                 held == [0x55000000, 0x00550000, 0x00005500, 0x00000054, 0]
                 and placed.left(caches, own, lambda g: g & own in held[:2]) == 0x00005554,
                 ' '.join('%x' % m for m in held))


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
    report.check("Linux's sysfs: her cores, their classes and their last-level caches as it tells them",
                 found == [(512, 0x11), (512, 0x22), (1024, 0x44), (1024, 0x88)] and caches == [0x33, 0xcc]
                 and (own, fast) == (0x0e, 0x0c) and placed.chosen(caches, own, lambda g: True, fast) == 0x0c
                 and empty == ([], []), '%s %s' % (found, ['%x' % c for c in caches]))
    report.check("a sysfs list of processors read", placed.listed('0-3,8,10-11\n') == 0xd0f and placed.listed('5') == 0x20)


def test_two_loops_take_each_a_cache_and_another_process_leaves_both(report):
    """Live, where this host tells its topology and holds a thread or a process to its processors: two processes
    placed as her loop's, a third as another, all alive at once - each loop's processors within one cache group
    and no other's, the third's off both."""
    from machine.process import placed
    found, caches = placed.topology()
    own, fast = placed.apart(found) if found else (0, 0)
    fits = [g for g in caches if (g & own) & ((g & own) - 1)]
    if len(fits) < 3:
        report.check('her loops each take a cache of their own (this host tells fewer than three such)', True,
                     '%d caches of two of her cores' % len(fits))
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
        report.check("her loops each take a cache of their own (this run's job holds its processes, or none is free)",
                     not (mine_a or mine_b) or (ok_a and ok_b), 'placed %d %d %d' % (ok_a, ok_b, ok_c))
        return
    report.check('two of her loops each on a cache group of its own, another process off both',
                 ok_a and ok_b and ok_c and mine_a and mine_b and now_a == mine_a and now_b == mine_b
                 and any(mine_a == g & own for g in caches) and any(mine_b == g & own for g in caches)
                 and not mine_a & mine_b and not now_c & (mine_a | mine_b) and now_c,
                 'loops %x %x, another %x' % (now_a, now_b, now_c))


ROSTER = (test_her_loop_takes_a_cache_of_its_own_on_hosts_of_other_shapes, test_linux_tells_its_topology_in_sysfs,
          test_two_loops_take_each_a_cache_and_another_process_leaves_both)


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
