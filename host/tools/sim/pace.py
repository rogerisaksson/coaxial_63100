#!/usr/bin/env python3
"""Her loop's pace as the worker runs her, flat out: her world's seconds a wall second, an errand.

    python tools/sim/pace.py                   # her errands one at a time on the relay, a benchmark's: a line each
    python tools/sim/pace.py walk door         # those named, here (COAXIAL_BENCH=1: above normal priority too)
    python tools/sim/pace.py walk --profile    # and where her main thread spends its time, sampled
    python tools/sim/pace.py door --time machine.law.motion.control tools.cores.bodied.Bound.__call__
    python tools/sim/pace.py door --here --time machine.eyes.registry.seen   # her vision in this process

Laid as `worker.run` lays her - this process placed (`placed`), her lodge, her fake camera, her
planner and her vision apart -, her state said each SAY_S of her world's time, run flat out: a line
an errand, each second's pace (her world's seconds a wall second, 1.0 real time), the worst and what
she was doing, the mean, her qpos's hash at its end. `--profile` samples her main thread's stack
every SAMPLE_S from a thread of its own (cProfile on this interpreter mixes threads): each function's
own share of the samples and its share with what it calls - a sample waits on the interpreter's lock,
so a C call holding it is under-counted. `--time` wraps each function named (a module's, or a class's
in it, where her code calls it through its module or its object) in a wall clock: its calls, its
seconds with what it calls, its share of the errand's wall time; `--here` keeps her vision in this
process (`vision.Here`, a run's), its look's own parts timed so - its bits the same either way.
"""
import argparse
import collections
import hashlib
import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

#: W's twist; a sit or a lie held this long once there, s.
WALK, HELD_S = (0.69, 0.0, 0.0), 3.0
#: The sampler's period, s; the functions a profile lists, by own and by inclusive share.
SAMPLE_S, LISTED = 0.0005, 24


class Run:
    """Her session laid as the worker lays her (`worker.run`), stepped flat out a pass at a time as
    `worker.Her` steps her, each second's wall time kept with what she was doing."""

    def __init__(self, here=False):
        from machine.eyes import vision
        from machine.process import placed, sprung, worker, worn
        from machine.way import planner
        from machine.world import physics
        placed.placed()
        # her eyes hers, as her page's worker lays her (`worker.run`, since 0808daf1): her registry on her stereo's map
        physics.ROOM, physics.CAMERA, physics.SIGHT = 1.0, 1.0, 1.0
        s = self.s = worker.session(sprung.STACK_EVERY, False, None)
        s['g']['planner'], s['peeker'] = planner.Planner(), planner.Planner()
        s['g']['vision'] = vision.Here() if here else vision.Apart()
        s['g']['planner'].start()
        s['peeker'].start()
        if not here:
            s['g']['vision'].start()
        s['worn'] = worn.Apart(worker.DT).start()
        s['worn'].laid(s['wear'])
        worker.readied(s)
        self.seconds: list = []
        self.said, self.wall, self.second = 0.0, time.perf_counter(), math.floor(self.t)

    @property
    def t(self):
        return self.s['g']['t']

    def doing(self):
        from machine.process import sprung
        return sprung.doing(self.s['g'])

    def go(self, seconds, until=None):
        """`seconds` of passes, or to her fall or `until(self)`: the seconds until then, or None."""
        from machine.process import sprung, worker
        from machine.way import peek
        t0 = self.t
        while self.t - t0 < seconds and self.s['fallen_at'] is None:
            worker.stepped(self.s)
            peek.peeked(self.s)
            if self.t - self.said >= sprung.SAY_S:
                self.said = self.t
                worker.told(self.s)
            if math.floor(self.t) > self.second:
                now = time.perf_counter()
                self.seconds.append((self.doing(), 1.0 / (now - self.wall)))
                self.wall, self.second = now, math.floor(self.t)
            if until is not None and until(self):
                return round(self.t - t0, 2)
        return None

    def tell(self, **command):
        from machine.process import worker
        worker.apply(self.s, command)

    def close(self):
        for apart in (self.s['g']['planner'], self.s['peeker'], self.s['g']['vision'], self.s['worn']):
            apart.close()


def _stand(r):
    r.go(8.0)


def _walk(r):
    r.tell(twist=WALK)
    r.go(8.0)


def _door(r):
    from machine.world import room
    r.tell(say='exit_door')
    r.go(30.0, lambda q: float(q.s['g']['world'].data.qpos[2]) > room.TO)


def _chair(r):
    from machine.world import room
    r.tell(say='sit_chair', at=room.CHAIR_AT)
    r.go(25.0, lambda q: q.doing() == 'seated')
    r.go(HELD_S)


def _bed(r):
    from machine.world import room
    r.tell(say='lie_bed', at=room.BED_AT)
    r.go(30.0, lambda q: q.doing() == 'bed_rest')
    r.go(HELD_S)


ERRANDS = {'stand': _stand, 'walk': _walk, 'door': _door, 'chair': _chair, 'bed': _bed}


def paced(name, here=False):
    """Errand `name` run flat out from her laying, her vision in this process where `here`: its line."""
    r = Run(here)
    try:
        r.go(0.5)
        r.seconds.clear()
        ERRANDS[name](r)
    finally:
        r.close()
    xs = [x for _d, x in r.seconds] or [0.0]
    worst = min(r.seconds, key=lambda d: d[1]) if r.seconds else ('-', 0.0)
    h = hashlib.sha1(r.s['g']['world'].data.qpos.tobytes()).hexdigest()[:12]
    return ('%-6s worst x%.2f (%s) mean x%.2f | %s | qpos %s at t %.2f%s'
            % (name, worst[1], worst[0][:14], sum(xs) / len(xs), ' '.join('%.2f' % x for x in xs), h, r.t,
               '' if r.s['fallen_at'] is None else ', fell at %.2f s' % r.s['fallen_at']))


def sampled(fn):
    """fn() run while a thread samples this thread's stack every SAMPLE_S: (its result, {function:
    own samples}, {function: samples it was on the stack}, samples)."""
    own: collections.Counter = collections.Counter()
    inclusive: collections.Counter = collections.Counter()
    main, done, count = threading.get_ident(), threading.Event(), [0]

    def sample():
        while not done.wait(SAMPLE_S):
            frame = sys._current_frames().get(main)
            if frame is None:
                continue
            count[0] += 1
            seen = set()
            own[_named(frame)] += 1
            while frame is not None:
                seen.add(_named(frame))
                frame = frame.f_back
            inclusive.update(seen)
    switch = sys.getswitchinterval()
    sys.setswitchinterval(SAMPLE_S / 5.0)
    sampler = threading.Thread(target=sample, daemon=True)
    sampler.start()
    try:
        got = fn()
    finally:
        done.set()
        sampler.join()
        sys.setswitchinterval(switch)
    return got, own, inclusive, count[0]


def timed(names):
    """Each function of its dotted name in `names` (a module's, or a class's in it) wrapped in a wall
    clock where it is found: {name: [calls, seconds]}, counted as her code calls them."""
    kept = {}
    for name in names:
        parts = name.split('.')
        owner, k = _module(parts)
        for part in parts[k:-1]:
            owner = getattr(owner, part)
        tally = kept[name] = [0, 0.0]

        def wrapped(*a, _fn=getattr(owner, parts[-1]), _tally=tally, **kw):
            t0 = time.perf_counter()
            try:
                return _fn(*a, **kw)
            finally:
                _tally[0] += 1
                _tally[1] += time.perf_counter() - t0
        setattr(owner, parts[-1], wrapped)
    return kept


def _module(parts):
    """(the longest module `parts` begin with, imported; how many of them it takes)."""
    import importlib
    for k in range(len(parts) - 1, 0, -1):
        try:
            return importlib.import_module('.'.join(parts[:k])), k
        except ModuleNotFoundError:
            continue
    raise ModuleNotFoundError('.'.join(parts))


def _named(frame):
    code = frame.f_code
    path = code.co_filename.replace('\\', '/')
    return '%s:%s' % ('/'.join(path.split('/')[-2:]), code.co_name)


def relayed(names):
    """[(errand, its line)] of `names`, each its own process on the relay's batons at a benchmark's priority
    (`focus.relay`'s bench: hers above normal, the host's other loads under them), alone - her pace is wall time,
    and a job beside it, with its four processes apart, takes its cores -, in order."""
    from tools.dev import focus
    jobs = [focus.Job(name, [sys.executable, '-X', 'utf8', os.path.abspath(__file__), name], 1.5, 900.0, 'alone')
            for name in names]
    got = {job.name: (text or '').strip().splitlines()[-1:] for job, text, _code, _s in focus.relay(jobs, bench=True)}
    return [(name, (got.get(name) or ['%-6s no line' % name])[0]) for name in names]


def main(argv=None):
    p = argparse.ArgumentParser(description=(__doc__ or '').split('\n\n')[0])
    p.add_argument('names', nargs='*', help='errands: %s' % ', '.join(ERRANDS))
    p.add_argument('--profile', action='store_true', help='where her main thread spends its time, sampled')
    p.add_argument('--time', nargs='+', default=(), help='functions by dotted name, each its wall time')
    p.add_argument('--here', action='store_true', help='her vision in this process, not apart')
    args = p.parse_args(argv)
    if not args.names:
        for _name, line in relayed(list(ERRANDS)):
            print(line, flush=True)
        return 0
    for name in args.names:
        if not args.profile:
            kept, t0 = timed(args.time), time.perf_counter()
            line = paced(name, args.here)
            wall = time.perf_counter() - t0
            for fn, (calls, s) in kept.items():
                print('  %5.1f%%  %8.1f us x %6d  %s' % (100.0 * s / wall, 1e6 * s / max(calls, 1), calls, fn))
            print(line, flush=True)
            continue
        line, own, inclusive, n = sampled(lambda: paced(name, args.here))
        for title, counts in (('own', own), ('inclusive', inclusive)):
            print('%s, %d samples:' % (title, n))
            for fn, k in counts.most_common(LISTED):
                print('  %5.1f%%  %s' % (100.0 * k / max(n, 1), fn))
        print(line, flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
