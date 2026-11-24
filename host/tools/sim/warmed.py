#!/usr/bin/env python3
"""The robot's control system compiled for its sources before its page lays it: a pull leaves its parts' C behind.

A change under the robot's machine, its cores or its texts restamps every part its law keeps (`tools.cores.kept`); a
change of what their libraries are built of (the glue's `quick.h`, the cores' headers) rebuilds each. Its page then
laid the robot on numpy's own while a thread beside its loop traced, built and held a part at a time: after a pull 37
builds over 16 minutes and many sessions, the robot's world at x0.42-0.56 of real time meanwhile. Here the robot is
laid as its worker lays it (`pace.Run`), its processes where the system moves them (`placed.FLOAT`), stood and walked
(ERRANDS): each part its law meets traced, their builds as wide as the host's batch (`held.widened`) and waited for,
each held bit for bit by the passes after (`bodied.met`); none left under way, the stamp is marked (`kept.marked`) and
the robot's page's first pass takes them all.

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
#: A line the compile prints for its page: STEP share text, then DONE held numpy under seconds.
STEP, DONE = 'step', 'done'


def current():
    """Whether the robot's parts were compiled whole under its sources' stamp (`kept.current`), the stamp read
    anew: the terminal outlives the sources it opened on."""
    from tools.cores import kept
    kept.stamp.cache_clear()
    return kept.current()


def compiled(width=None):
    """The robot laid, stood and walked, each part it meets compiled: (share, text) as it goes; its end
    (held, numpy's, under way, seconds) - the stamp marked where none is under way."""
    from tools.cores import bodied, held, kept
    from tools.dev import background
    began = time.perf_counter()
    held.widened(width or background.widths()[0])
    yield 0.02, 'LAYING THE ROBOT'
    r = pace.Run()
    try:
        for k, (what, twist, seconds) in enumerate(ERRANDS):
            share = (k + 0.5) / len(ERRANDS)
            yield share - 0.25 / len(ERRANDS), what.upper()
            r.tell(twist=twist)
            r.go(seconds)
            for _ in range(ROUNDS):
                waiting = held.unbuilt()
                for i, job in enumerate(waiting):
                    job['done'].wait()
                    yield share, '%s: BUILT %d OF %d' % (what.upper(), i + 1, len(waiting))
                r.go(HOLD_S)
                got, numpy, under = bodied.met()
                yield share + 0.25 / len(ERRANDS), '%s: HELD %d OF %d' % (what.upper(), got, got + numpy + under)
                if not under and not held.unbuilt():
                    break
    finally:
        r.close()
    got, numpy, under = bodied.met()
    if not under:
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
    steps = compiled(args.width or None)
    try:
        while True:
            print('%s %.3f %s' % ((STEP,) + next(steps)), flush=True)
    except StopIteration as end:
        got, numpy, under, seconds = end.value
    print('%s %d %d %d %.1f' % (DONE, got, numpy, under, seconds), flush=True)
    return 0 if not under else 1


if __name__ == '__main__':
    sys.exit(main())
