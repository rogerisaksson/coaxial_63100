#!/usr/bin/env python3
"""Her loop's pace as her worker runs her, a craft's as its page flies it: flat out, an errand.

    python tools/sim/pace.py                   # her errands and the crafts' flights one at a time on the relay: a line each
    python tools/sim/pace.py walk door         # those named, here (COAXIAL_BENCH=1: above normal priority too)
    python tools/sim/pace.py quad helicopter ornithopter aerobat   # a craft's flight: its card's first FLOWN_S
    python tools/sim/pace.py walk --profile    # and where her main thread spends its time, sampled
    python tools/sim/pace.py door --time machine.law.motion.control tools.cores.bodied.Bound.__call__
    python tools/sim/pace.py door --here --time machine.eyes.registry.seen   # her vision in this process
    python tools/sim/pace.py door --blind      # her eyes not hers: a sha1 over every pass, her bits' oracle

Laid as `worker.run` lays her - this process placed (`placed`), her lodge, her fake camera, her
planner and her vision apart -, her state said each SAY_S of her world's time, run flat out: a line
an errand, each second's pace (her world's seconds a wall second, 1.0 real time), the worst and what
she was doing, the mean, her qpos's hash at its end - her hot pace: its first second, cold, is its
tail's with her laying's time, a cold start to cut apart from a hot pace to raise; a profile's samples
and a clock's calls the hot ones, `--cold` the cold ones (WARM). `--float` leaves her processes
where the system moves them, unplaced. `--profile` samples her main thread's stack
every SAMPLE_S from a thread of its own (cProfile on this interpreter mixes threads): each function's
own share of the samples and its share with what it calls - a sample waits on the interpreter's lock,
so a C call holding it is under-counted. `--time` wraps each function named (a module's, or a class's
in it, where her code calls it through its module or its object) in a wall clock: its calls, its
seconds with what it calls, its share of the errand's wall time; `--here` keeps her vision in this
process (`vision.Here`, a run's), its look's own parts timed so - its bits the same either way. `--blind` lays
her with her eyes not hers - her registry on the world's boxes, no camera - and tells a sha1 over every pass's
qpos, qvel and ctrl: her bits the same at any pace and on any host's load, as her errands' ends with her eyes
hers are not - the card's multisampling tells her camera's frames by the wall's time (`machine.eyes.camera`).
A craft (CRAFTS) is laid as its page lays it (`terminal.views.quad.flying`: its stand-in boards, its sky, its law, its
card), placed as a loop's as her worker is, each step said as its page hears it; `--blind` hashes its every step.
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
#: Set once an errand's first second has passed (`warmed`). Before it her cold start - her laying, its imports a tenth
#: of a profile's samples, her first looks, her planner's first answer, cold caches -, after it her hot pace: her line
#: tells each apart (its seconds hot, its tail cold: laid in, the first second's pace), and a profile's samples
#: (`sampled`) and a clock's calls (`timed`) are the hot ones, the cold ones where COLD is set (`--cold`). The first
#: second was the worst of every errand, x0.74-1.00.
WARM, WARM_AT, COLD = threading.Event(), [0.0], [False]
#: The crafts flown as errands, each its page's flight; how long of its card each is flown, s.
CRAFTS, FLOWN_S = ('quad', 'helicopter', 'ornithopter', 'aerobat'), 20.0


class Run:
    """Her session laid as the worker lays her (`worker.run`), stepped flat out a pass at a time as
    `worker.Her` steps her, each second's wall time kept with what she was doing."""

    def __init__(self, here=False, blind=False):
        from machine.eyes import vision
        from machine.process import placed, sprung, worker, worn
        from machine.way import planner
        from machine.world import physics
        placed.placed(True)
        # her eyes hers, as her page's worker lays her (`worker.run`, since 0808daf1): her registry on her stereo's map
        physics.ROOM, physics.CAMERA, physics.SIGHT = 1.0, float(not blind), float(not blind)
        self.passed = hashlib.sha1() if blind else None
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

    @property
    def data(self):
        return self.s['g']['world'].data

    @property
    def ended(self):
        return '' if self.s['fallen_at'] is None else ', fell at %.2f s' % self.s['fallen_at']

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
            if self.passed is not None:
                d = self.s['g']['world'].data
                for a in (d.qpos, d.qvel, d.ctrl):
                    self.passed.update(a.tobytes())
            if self.t - self.said >= sprung.SAY_S:
                self.said = self.t
                worker.told(self.s)
            if math.floor(self.t) > self.second:
                now = time.perf_counter()
                self.seconds.append((self.doing(), 1.0 / (now - self.wall)))
                self.wall, self.second = now, math.floor(self.t)
                warmed(now)
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


class Flown:

    """A craft's flight laid as its page lays it (`terminal.views.quad.flying.run`), stepped flat out a step at a
    time as `apart.paced` steps it - each step said as its page hears it -, each second's wall time kept with its
    card's stage; this process placed as a loop's, as `apart.looped` places its page's."""

    def __init__(self, kind, blind=False):
        import importlib
        from machine.process import placed
        from terminal.views.quad import flying
        placed.placed(True)
        craft = importlib.import_module('machine.aircraft.' + kind)
        flown = importlib.import_module('terminal.views.%s.flight' % kind)
        self.flight = flying.Flight(craft, flown, len(getattr(flown, 'ROTORS', None) or craft.ROTOR_AT))
        self.passed = hashlib.sha1() if blind else None
        self.seconds: list = []
        self.wall, self.second = time.perf_counter(), math.floor(self.t)

    t = property(lambda self: self.flight.t)
    data = property(lambda self: self.flight.sky.data)
    ended = property(lambda self: ', wrecked' if self.flight.flight.get('wreck') else '')

    def go(self, seconds):
        """`seconds` of its flight's steps, each said (`Flight.said`) as its page's loop says it."""
        t0 = self.t
        while self.t - t0 < seconds:
            self.flight.step()
            self.flight.said(1.0)
            if self.passed is not None:
                for a in (self.data.qpos, self.data.qvel, self.data.xfrc_applied):
                    self.passed.update(a.tobytes())
            if math.floor(self.t) > self.second:
                now = time.perf_counter()
                self.seconds.append((str(self.flight.flight.get('stage')), (math.floor(self.t) - self.second) / (now - self.wall)))
                self.wall, self.second = now, math.floor(self.t)
                warmed(now)

    def close(self):
        self.flight.close()


def _flown(r):
    r.go(FLOWN_S)


ERRANDS = dict({'stand': _stand, 'walk': _walk, 'door': _door, 'chair': _chair, 'bed': _bed}, **dict.fromkeys(CRAFTS, _flown))


def warmed(now):
    """An errand's first second past at wall time `now`: what is timed from here (WARM)."""
    if not WARM.is_set():
        WARM_AT[0] = now
        WARM.set()


def paced(name, here=False, blind=False):
    """Errand `name` run flat out from her laying, her vision in this process where `here`, her eyes not hers and
    every pass hashed where `blind` - a craft's flight (CRAFTS) from its card's start, its every step hashed -: its line,
    its seconds hot - its first, cold, apart (WARM) -, its tail its cold start: laid in, its first second's pace."""
    WARM.clear()
    began = time.perf_counter()
    r = Flown(name, blind) if name in CRAFTS else Run(here, blind)
    try:
        r.go(0.5)
        r.seconds.clear()
        WARM.clear()
        laid = time.perf_counter() - began
        ERRANDS[name](r)
    finally:
        r.close()
    hot = r.seconds[1:]
    xs = [x for _d, x in hot] or [0.0]
    worst = min(hot, key=lambda d: d[1]) if hot else ('-', 0.0)
    h = hashlib.sha1(r.data.qpos.tobytes()).hexdigest()[:12]
    return ('%-6s worst x%.2f (%s) mean x%.2f | %s | qpos %s at t %.2f%s'
            % (name, worst[1], worst[0][:14], sum(xs) / len(xs), ' '.join('%.2f' % x for x in xs), h, r.t,
               r.ended)
            + ('' if r.passed is None else ', passes %s' % r.passed.hexdigest()[:16])
            + ', cold: laid in %.2f s, first second x%.2f' % (laid, r.seconds[0][1] if r.seconds else 0.0))


def sampled(fn):
    """fn() run while a thread samples this thread's stack every SAMPLE_S: (its result, {function:
    own samples}, {function: samples it was on the stack}, samples)."""
    own: collections.Counter = collections.Counter()
    inclusive: collections.Counter = collections.Counter()
    main, done, count = threading.get_ident(), threading.Event(), [0]

    def sample():
        while not done.wait(SAMPLE_S):
            frame = sys._current_frames().get(main)
            if frame is None or WARM.is_set() == COLD[0]:
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
                if WARM.is_set() != COLD[0]:
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
    p.add_argument('--blind', action='store_true', help='her eyes not hers, no camera: a sha1 over every pass told')
    p.add_argument('--float', action='store_true',
                   help='her processes where the system schedules them, none held to a core or a cache (FLOAT)')
    p.add_argument('--cold', action='store_true',
                   help="--profile's samples and --time's calls her cold start's - her laying, her first second - "
                        'not her hot pace\'s after it (COLD)')
    args = p.parse_args(argv)
    from machine.process.placed import BENCH, FLOAT
    COLD[0] = args.cold
    if args.float:
        os.environ[FLOAT] = '1'
    if not args.names:
        for _name, line in relayed(list(ERRANDS)):
            print(line, flush=True)
        return 0
    from tools.dev import background
    if not background.runner():
        # an errand run here is a benchmark's or a profile's: on a desk her processes above normal priority
        # (`placed.raised`), the user's other loads under what is timed; a runner has none
        os.environ.setdefault(BENCH, '1')
    for name in args.names:
        if not args.profile:
            kept, t0 = timed(args.time), time.perf_counter()
            line = paced(name, args.here, args.blind)
            end = time.perf_counter()
            # a clock's share of the phase it counts: her cold start's wall time to her first second, her hot pace's after
            wall = max((WARM_AT[0] - t0 if COLD[0] else end - WARM_AT[0]) if WARM.is_set() else end - t0, 1e-9)
            for fn, (calls, s) in kept.items():
                print('  %5.1f%%  %8.1f us x %6d  %s' % (100.0 * s / wall, 1e6 * s / max(calls, 1), calls, fn))
            print(line, flush=True)
            continue
        line, own, inclusive, n = sampled(lambda: paced(name, args.here, args.blind))
        for title, counts in (('own', own), ('inclusive', inclusive)):
            print('%s, %d samples:' % (title, n))
            for fn, k in counts.most_common(LISTED):
                print('  %5.1f%%  %s' % (100.0 * k / max(n, 1), fn))
        print(line, flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
