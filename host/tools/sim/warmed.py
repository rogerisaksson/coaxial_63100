#!/usr/bin/env python3
"""The robot's control system compiled for its sources before its page lays it.

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
left, the stamp is marked (`kept.marked`) and the robot's page's first pass takes them all.

    python -X utf8 -m tools.sim.warmed        # compiled where behind: a line a step
    warmed.current()                           # whether its parts are its sources' (`kept.current`)
    warmed.ensure(step)                        # compiled where behind, apart; `step(share, text)` told as it goes
"""
import argparse
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
#: A line the compile prints for its page: STEP share text, then DONE held numpy under seconds.
STEP, DONE = 'step', 'done'


def current():
    """Whether the robot's parts were compiled whole under its sources' stamp (`kept.current`), the stamp read
    anew: the terminal outlives the sources it opened on."""
    from tools.cores import kept
    kept.stamp.cache_clear()
    return kept.current()


def spread():
    """This process on every processor it is given (`placed.cores`), not on its starter's: started by the robot's
    page, which holds a core of its own (`terminal.ui.stage`), its builds, the robot's world and its law shared that
    core's 2 processors of 64 - its first build 57 s, the whole 123 s for 44. Whether it now is."""
    from machine.process import placed
    return placed.bind(os.getpid(), sum(placed.cores()))


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


def compiled(width=None):
    """The robot laid, stood and walked, each part it meets compiled: (share, text) as it goes; its end
    (held, numpy's, under way, seconds) - the stamp marked where none is under way."""
    from tools.cores import bodied, held, kept
    from tools.dev import background
    began = time.perf_counter()
    held.widened(width or background.widths()[0])
    yield 0.02, 'LAYING THE ROBOT'
    r = pace.Run(blind=True)
    try:
        for k, (what, twist, seconds) in enumerate(ERRANDS):
            share = (k + 0.5) / len(ERRANDS)
            yield share - 0.25 / len(ERRANDS), what.upper()
            r.tell(twist=twist)
            r.go(seconds)
            for _ in range(ROUNDS):
                yield from built(what, share - 0.25 / len(ERRANDS), share, held.unbuilt())
                r.go(HOLD_S)
                got, numpy, under = bodied.met()
                yield share + 0.25 / len(ERRANDS), '%s: HELD %d OF %d' % (what.upper(), got, got + numpy + under)
                if not held.unbuilt():
                    break
    finally:
        r.close()
    got, numpy, under = bodied.met()
    if not held.unbuilt():
        kept.marked()
    return got, numpy, under, time.perf_counter() - began


def ensure(step=lambda share, text: None):
    """Compiled where behind, in a process of its own (its robot's processes unplaced, this tree's modules):
    `step(share, text)` told as it goes. (whether current now, what it says)."""
    from tools import REPO
    from machine.process import placed
    if current():
        return True, 'current'
    host = os.path.join(REPO, 'host')
    env = dict(os.environ, **{placed.FLOAT: '1',
                              'PYTHONPATH': os.pathsep.join(p for p in (host, os.environ.get('PYTHONPATH')) if p)})
    run = subprocess.Popen([sys.executable, '-X', 'utf8', '-m', 'tools.sim.warmed'], cwd=host, env=env,
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, encoding='utf-8',
                           errors='replace')
    said = 'ended without a word'
    try:
        for line in run.stdout or ():
            word, _, rest = line.strip().partition(' ')
            if word == STEP:
                share, _, text = rest.partition(' ')
                step(float(share), text)
            elif word == DONE:
                got, numpy, under, seconds = rest.split()
                said = '%s parts held, %s numpy\'s, %s under way, %.0f s' % (got, numpy, under, float(seconds))
                break
    finally:
        if run.poll() is None:
            run.kill()
        run.wait()
    return current(), said


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--width', type=int, default=0, help="builds at once, the host's batch unless given")
    args = parser.parse_args(argv)
    from machine.process import placed
    if current():
        print('%s 0 0 0 0.0' % DONE, flush=True)
        return 0
    os.environ.setdefault(placed.FLOAT, '1')
    spread()
    steps = compiled(args.width or None)
    try:
        while True:
            print('%s %.3f %s' % ((STEP,) + next(steps)), flush=True)
    except StopIteration as end:
        got, numpy, under, seconds = end.value
    print('%s %d %d %d %.1f' % (DONE, got, numpy, under, seconds), flush=True)
    return 0 if current() else 1


if __name__ == '__main__':
    sys.exit(main())
