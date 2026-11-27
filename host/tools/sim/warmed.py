#!/usr/bin/env python3
"""The machines' control systems compiled for their sources before their pages lay them.

A pull leaves the robot's parts' C behind. A change under the robot's machine, its cores or its texts restamps every
part its law keeps (`tools.cores.kept`); a change of what their libraries are built of (the glue's `quick.h`, the
cores' headers) rebuilds each. The robot's page then laid the robot on numpy's own while a thread beside its loop
traced, built and held a part at a time: after a pull 37 builds over 16 minutes and many sessions, the robot's world
at x0.42-0.56 of real time meanwhile. Here the robot is laid as its worker lays it (`pace.Run`), its eyes the world's
boxes (`blind`: flat out, the camera's looks held the robot's loop 93 % of the wall at x0.09, and the robot never
walked), its processes where the system moves them over every processor it is given, not its page's core
(`placed.FLOAT`, `spread`), stood and walked (ERRANDS), each build told as it lands (`built`): each part its law
meets traced, their builds as wide as the host's batch (`held.widened`) and waited for, each held bit for bit by the
passes after (`bodied.met`) - a shape met once and not again left to hold where it is met, its library built. No build
left, the stamp is marked (`kept.marked`) and the robot's page's first pass takes them all. A craft (`pace.CRAFTS`) is
flown FLOWN_S as its page flies it (`pace.Flown`) and its parts compiled the same way, each machine its own mark.

    python -X utf8 -m tools.sim.warmed [MACHINE ...]   # each named, or every one, compiled where behind: a line a step
    warmed.current('quad')                              # whether its parts are its sources' (`kept.current`)
    warmed.ensure(step)                                 # the robot's compiled where behind, apart; `step(share, text)`
    run = warmed.started(machines); warmed.told(run, step)   # several apart, `step(machine, share, text)` as they go
"""
import argparse
import functools
import os
import subprocess
import sys
import time

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.sim import pace  # noqa: E402

#: What the robot does while its parts are met: (what, its twist, the robot's seconds of it).
ERRANDS = (('standing', (0.0, 0.0, 0.0), 1.0), ('walking', pace.WALK, 3.0))
#: The robot's time a round of holds is given once its builds are done, s; the rounds an errand has at the most.
HOLD_S, ROUNDS = 0.5, 6
#: The seconds between the compile's lines while a build is under way: its page's strip told it goes on.
BEAT_S = 1.0
#: The machines whose control systems compile, as their pages name them (`terminal/pages`, COMPILED): the robot,
#: laid and walked, and each craft, flown.
ROBOT = 'robot'
MACHINES = (ROBOT,) + pace.CRAFTS
#: A craft's flight while its parts are met, s of its time.
FLOWN_S = 2.0
#: A line the compile prints for its page: STEP machine share text, then DONE machine held numpy under seconds.
STEP, DONE = 'step', 'done'


def _mark(machine):
    """`machine`'s mark in the kept store (`kept.current`): the robot's its active build's (None), a craft's its kind."""
    return None if machine == ROBOT else machine


def current(machine=ROBOT):
    """Whether `machine`'s parts were compiled whole under its sources' stamp (`kept.current`), the stamp read
    anew: the terminal outlives the sources it opened on."""
    from tools.cores import kept
    kept.stamp.cache_clear()
    return kept.current(_mark(machine))


def behind(machines=MACHINES):
    """Those of `machines` behind their sources' stamp (`current`), the stamp read anew once, in MACHINES' order."""
    from tools.cores import kept
    kept.stamp.cache_clear()
    return [machine for machine in MACHINES if machine in machines and not kept.current(_mark(machine))]


def spread():
    """Place this compile on one logical processor per free physical core (no SMT sibling contention, off processor 0
    and off cores/caches held by the terminal page, `Ahead` or live loops), at below-normal priority on a desk so
    background GCC builds never starve the interactive UX."""
    import ctypes
    from machine.process import placed
    from tools.dev import background
    if not background.runner():
        if sys.platform == 'win32':
            kernel = ctypes.windll.kernel32
            kernel.GetCurrentProcess.restype = ctypes.c_void_p
            kernel.SetPriorityClass.argtypes = (ctypes.c_void_p, ctypes.c_uint32)
            kernel.SetPriorityClass(kernel.GetCurrentProcess(), background.BELOW_NORMAL)
        elif hasattr(os, 'nice'):
            try:
                os.nice(background.NICE)
            except OSError:
                pass
    return placed.bind(os.getpid(), placed.free_processors())


def built(what, low, high, jobs, beat=BEAT_S):
    """`jobs` waited for: (share, text) as each lands, in any order, and every `beat` s while none does - WHAT: N OF M
    BUILT ON K CORES, the share from `low` to `high` as they land. Told only as the list's first landed, the page's
    strip stood on its errand's name 57 s."""
    from tools.cores import held
    while jobs:
        waiting = [job for job in jobs if not job['done'].is_set()]
        done = len(jobs) - len(waiting)
        yield (low + (high - low) * done / len(jobs),
               '%s: %d OF %d BUILT ON %d CORES' % (what.upper(), done, len(jobs), held.WIDTH[0]))
        if not waiting:
            return
        waiting[0]['done'].wait(beat)


def _errand(r, twist, seconds):
    from machine.law import barrier
    g = r.s['g']
    while g.get('ringed_ahead'):
        barrier.ahead(g['ringed'], g['ringed_ahead'], paced=False)
    r.tell(twist=twist)
    r.go(seconds)


def _hold_hulls(r):
    """Hold and persist any `barrier.HULLS` shapes on `r` whose C libraries finished building."""
    if not hasattr(r, 's'):
        return
    from machine.law import barrier
    ringed = r.s['g'].get('ringed')
    if ringed is None:
        return
    pending = []
    for k in barrier.HULLS:
        part = ringed.get(((3,), (3,), (3,), (), (k, 2), ()))
        if part is None or (part.c is None and part.why is None):
            pending.append(k)
    if pending:
        barrier.ahead(ringed, pending, paced=False)


def _flown(r, seconds):
    r.go(seconds)


def _laid(machine):
    """`machine` laid as its page lays it - the robot blind (`pace.Run`), a craft flown (`pace.Flown`) - and what it
    does while its parts are met, [(what, fn(laid))]."""
    if machine == ROBOT:
        return pace.Run(blind=True), [(what, functools.partial(_errand, twist=twist, seconds=seconds))
                                      for what, twist, seconds in ERRANDS]
    return pace.Flown(machine, blind=True), [('flying', functools.partial(_flown, seconds=FLOWN_S))]


def compiled(machine=ROBOT, width=None):
    """`machine` laid, each part it meets doing what it does compiled: (share, text) as it goes; its end (held,
    numpy's, under way, seconds) - its stamp marked where no build is left."""
    from tools.cores import bodied, held, kept
    from tools.dev import background
    began, since = time.perf_counter(), bodied.made()
    held.widened(width or background.widths()[0])
    yield 0.02, 'LAYING THE %s' % machine.upper()
    r, doings = _laid(machine)
    try:
        for k, (what, do) in enumerate(doings):
            share, half = (k + 0.5) / len(doings), 0.25 / len(doings)
            yield share - half, what.upper()
            do(r)
            for _ in range(ROUNDS):
                yield from built(what, share - half, share, held.unbuilt())
                _hold_hulls(r)
                r.go(HOLD_S)
                got, numpy, under = bodied.met(since)
                yield share + half, '%s: HELD %d OF %d' % (what.upper(), got, got + numpy + under)
                if not held.unbuilt() and not under:
                    break
    finally:
        r.close()
    got, numpy, under = bodied.met(since)
    if not held.unbuilt() and not under:
        kept.marked(_mark(machine))
    return got, numpy, under, time.perf_counter() - began


def started(machines):
    """The compile of `machines` begun in a process of its own (its robot's processes unplaced, this tree's modules):
    its process, its lines for `told`."""
    from tools import REPO
    from machine.process import placed
    host = os.path.join(REPO, 'host')
    env = dict(os.environ, **{placed.FLOAT: '1',
                              'PYTHONPATH': os.pathsep.join(p for p in (host, os.environ.get('PYTHONPATH')) if p)})
    return subprocess.Popen([sys.executable, '-X', 'utf8', '-m', 'tools.sim.warmed', *machines], cwd=host, env=env,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, encoding='utf-8',
                            errors='replace')


def told(run, step=lambda machine, share, text: None, done=lambda machine: None):
    """`run`'s (`started`'s) lines read till it ends: `step(machine, share, text)` each step, `done(machine)` as each
    finishes; {machine: what its end says}. The process waited for, ended where it still runs."""
    said, fault = {}, ''
    try:
        for raw in run.stdout or ():
            line = raw.strip()
            word, machine, rest = (line.split(' ', 2) + ['', ''])[:3]
            if word == STEP:
                share, _, text = rest.partition(' ')
                step(machine, float(share), text)
            elif word == DONE:
                got, numpy, under, seconds = rest.split()
                said[machine] = ("%s parts held, %s numpy's, %s under way, %.0f s"
                                 % (got, numpy, under, float(seconds)))
                done(machine)
            elif line:
                fault = line
    finally:
        if run.poll() is None:
            run.kill()
        run.wait()
    if not said and fault:
        said['_fault'] = fault
    return said


def ensure(step=lambda share, text: None, machines=(ROBOT,)):
    """`machines` (the robot) compiled where behind, in a process of its own (`started`): `step(share, text)` told
    as it goes. (whether each is current now, what they say)."""
    late = behind(machines)
    if not late:
        return True, 'current'
    said = told(started(late), lambda _machine, share, text: step(share, text))
    fault = said.pop('_fault', '')
    return not behind(late), '; '.join(said.values()) or fault or 'ended without a word'


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('machines', nargs='*', help='those compiled where behind (%s), every one unless named'
                        % ', '.join(MACHINES))
    parser.add_argument('--width', type=int, default=0, help="builds at once, the host's batch unless given")
    args = parser.parse_args(argv)
    unknown = [machine for machine in args.machines if machine not in MACHINES]
    if unknown:
        parser.error('no machine %s: %s' % (', '.join(unknown), ', '.join(MACHINES)))
    from machine.process import placed
    late = behind(args.machines or MACHINES)
    os.environ.setdefault(placed.FLOAT, '1')
    if late:
        spread()
    for machine in late:
        steps = compiled(machine, args.width or None)
        try:
            while True:
                print('%s %s %.3f %s' % ((STEP, machine) + next(steps)), flush=True)
        except StopIteration as end:
            got, numpy, under, seconds = end.value
        print('%s %s %d %d %d %.1f' % (DONE, machine, got, numpy, under, seconds), flush=True)
    return 0 if not behind(late) else 1


if __name__ == '__main__':
    sys.exit(main())
