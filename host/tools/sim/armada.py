#!/usr/bin/env python3
"""An armada of her, each walking steady: a law swapped in under way, gone back from at a fall.

    python tools/sim/armada.py                        # the walk as built, each pace's measures
    python tools/sim/armada.py --set stance.PRE_SWING_DEG=30
    python tools/sim/armada.py --grid gait.KNEE_SOFT_DEG=3,6,9
    python tools/sim/armada.py --search stance.PRE_SWING_DEG=20:45 gait.LIFT_M=0.02:0.05
    python tools/sim/armada.py --search .. --aim 'knee at landing=12'   # past the form
    python tools/sim/armada.py --up                   # the armada kept walking, until --down
    python tools/sim/armada.py --serve DIR            # one robot of it: the relay's job

A robot is armed once, begun standing, walked to its steady stride at each pace and marked there
(`tools.sim.replay`). A candidate - constants {name: value} - is swapped in as she walks
(`knobs.swap`), walked SETTLE strides and measured over STRIDES whole ones (`strides.measured`),
and she goes back to her mark: a fall ends its trial, not the robot. Every candidate starts from
the same stride of the same walk, so two differ by their constants alone; its starts - standing
and from the squat, from marks taken there (STARTS) - are tried beside it. The law's own code is
swapped in the same way: a robot reloads the law's modules (LAW) when their files change and
walks on from its marks under them (`Robot.relaw`); a law her state does not fit, she walks in
again. The robots are the relay's jobs (`focus.relay`), fed from a directory: todo/ taken by
renaming, done/ a result each. Kept up (`--up`), a walk's smoke test is a pace's 4 s of her time;
from the squat it took 16-24 s of hers and 60-90 of ours.
"""
import argparse
import importlib
import json
import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools import REPO  # noqa: E402
from tools.dev import background  # noqa: E402
from tools.sim import cmaes, knobs, looks  # noqa: E402

#: The paces she is marked at, strides/s, the page's first; the strides a candidate walks before
#: it is measured and the whole strides it is measured over; a robot's commit, GB.
PACES, SETTLE, STRIDES, ROBOT_GB = (0.85, 0.65, 1.0), 2, 3, 1.2

#: A robot marks a pace STEADY strides after its ramp or its glide to it; a trial is cut at
#: TRIAL_S of her time; her rows a second.
STEADY, TRIAL_S, ROWS_HZ = 3, 14.0, 60.0

#: A candidate's starts, (way, pace, seconds of hers): standing and from the squat, begun at
#: her own pace and glided to it as the director has her - each from a mark taken there, its
#: parries counted. Steady alone, a search's best landed its knee at 9 degrees and fell in 2
#: rises of 9 and 2 walks of 12 (2026-10-05).
STARTS = (('stand', 0.65, 9.0), ('stand', 0.882, 9.0), ('stand', 1.02, 9.0),
          ('squat', 0.85, 13.0), ('squat', 1.0, 13.0))

#: A start's parry, in a price: START_K each.
START_K = 2.0

#: The law's modules (`machine.`), reloaded when a file of theirs changes: what the director and
#: the walker are made of, not her body, her boards or the loop.
LAW = ('curves', 'figure', 'gait', 'walkplan', 'capture', 'pendulum', 'parry', 'bearing', 'dcm',
       'landing', 'stance', 'stand', 'falls', 'down', 'getup', 'arrival', 'walker', 'director')

#: The armada kept up: its directory, and how stale a robot's sign of life may be, s.
ROOT, ALIVE_S = os.path.join(REPO, 'build', 'armada'), 30.0


def _stamp():
    """The law's files' newest change."""
    import machine
    root = os.path.dirname(machine.__file__)
    return max(os.path.getmtime(os.path.join(root, name + '.py')) for name in LAW)


class Robot:

    """Her, walking, marked at each pace."""

    def __init__(self, paces=PACES):
        from machine import Machine
        from machine.modes import DYNAMIC
        from tools.sim import replay
        knobs.set_({'ENVELOPE': 0.0})
        self.replay, self.paces = replay, tuple(paces)
        self.body = Machine.discover('gynoid', execution_mode=DYNAMIC)
        self.body.arm()
        self.world = self.body.nodes['pelvis'].world
        self.base, self.law = {}, _stamp()
        self._walk_in()

    def _walk_in(self):
        """Begun standing, walked to her steady stride at each pace and marked there; each pace
        glided to from the first's mark as the director glides (`director.PACE_RATE`)."""
        from machine.director import Director
        self.director = Director(self.body, self.paces[0], stand_s=0.0)
        self.marks, self.steady, self.begun = {}, {}, {}
        for way in ('squat', 'stand'):
            self.director.begin(drop=0.002, stage=way)
            self.body.loop.step(0.0)
            self.begun[way] = self.replay.mark(self.body, self.director)
        while not (self.director.stage == 'walk' and self.director.walker.held is None):
            self._pass()
        for pace in self.paces:
            if self.marks:
                self.replay.back(self.marks[self.paces[0]])
            self.director.cadence = pace
            walked = True
            while walked and abs(self.director.walker.cadence - pace) > 1e-9:
                walked = self._strides(1)
            self.steady[pace] = walked and self._strides(STEADY)
            self.marks[pace] = self.replay.mark(self.body, self.director)

    def relaw(self):
        """The law's modules loaded anew from their files, she and her marks walking on under
        them: each of her objects made its class's new self."""
        for name in LAW:
            importlib.reload(sys.modules['machine.' + name])
        for mark in self.marks.values():
            for o, _attrs in mark['held'].values():
                now = getattr(sys.modules.get(type(o).__module__), type(o).__name__, None)
                if isinstance(now, type) and now is not type(o):
                    o.__class__ = now
        knobs.set_({'ENVELOPE': 0.0})
        self.base, self.law = {}, _stamp()

    def _pass(self):
        self.body.loop.write(**self.director.step(0.001))
        self.body.loop.step(0.001)

    def _strides(self, count, each=None):
        """`count` whole strides on - the left leg's phase round -, `each()` after every pass;
        whether she walked them: not fallen, not out of time."""
        bus, walker = self.body.loop.bus, self.director.walker
        until, was = bus['t'] + TRIAL_S, walker.phase
        while count > 0:
            self._pass()
            if self.director.stage in ('falling', 'fallen') or bus['t'] > until:
                return False
            if each is not None:
                each()
            count -= walker.phase < was - 0.5
            was = walker.phase
        return True

    def tried(self, values, pace, way='steady', seconds=0.0):
        """{measure: value} of the walk `values` makes of hers at `pace`: from her mark, swapped
        in, SETTLE strides on, over STRIDES whole ones - or, `way` 'stand' or 'squat', of her
        start under it (`_started`). The law's files changed, they are loaded and she walks in
        under them first: a mark is its own law's walk - kept, a walk braked another way stood
        in for this one's and fell at every pace (2026-10-05)."""
        run = ((lambda: self._tried(values, pace)) if way == 'steady'
               else (lambda: self._started(values, way, pace, seconds)))
        if _stamp() != self.law:
            self.relaw()
            self._walk_in()
        return run()

    def _started(self, values, way, pace, seconds):
        """{fell, catches} of her start under `values`: from the mark where she began standing or
        in the squat, to `seconds` of hers on, toward `pace`."""
        for name in values:
            self.base.setdefault(name, knobs.now(name))
        knobs.put(self.base)
        self.replay.back(self.begun[way])
        knobs.set_(values)
        self.director.cadence = pace
        bus, parries, was = self.body.loop.bus, 0, self.director.stage
        until = bus['t'] + seconds
        while bus['t'] < until:
            self._pass()
            now = self.director.stage
            if now in ('falling', 'fallen'):
                return {'fell': 1.0, 'catches': parries}
            parries += now == 'catch' != was
            was = now
        return {'fell': 0.0, 'catches': parries}

    def _tried(self, values, pace):
        from tools.sim import look, strides
        if not self.steady[pace]:
            return {'fell': 1.0, 'unmarked': 1.0}
        for name in values:
            self.base.setdefault(name, knobs.now(name))
        knobs.put(self.base)
        self.replay.back(self.marks[pace])
        knobs.swap(values)
        bus, world = self.body.loop.bus, self.world
        if not self._strides(SETTLE):
            return {'fell': 1.0}
        rows, said, joules = [], [-1.0], [0.0]
        t0, z0 = bus['t'], bus['pelvis.pose.z']

        def each():
            joules[0] += world.drawn() * 0.001
            if bus['t'] - said[0] >= 1.0 / ROWS_HZ:
                said[0] = bus['t']
                rows.append(look.sample(bus, self.director, world))
        if not self._strides(STRIDES, each):
            return {'fell': 1.0}
        out = {k: v for k, v in strides.measured(rows).items() if v == v}
        on = bus['pelvis.pose.z'] - z0
        out.update(fell=0.0, speed=on / (bus['t'] - t0), power=joules[0] / (bus['t'] - t0),
                   energy=joules[0] / on if on > 0.1 else math.inf)
        return out

    def close(self):
        knobs.put(self.base)
        self.body.close()


#: A search's aims past the form, {measure: at most}: FORM_K a measure's share past it.
AIMS = {}


def price(results):
    """A candidate's price from its {trial: measures}: each pace's steady walk (`looks.priced`)
    and what it is past the search's AIMS, meaned, and each start's fall or parries (START_K)."""
    steady = [m for k, m in results.items() if ' ' not in k]
    starts = [m for k, m in results.items() if ' ' in k]
    return (sum(looks.priced(m) + sum(looks.FORM_K * max(0.0, m.get(name, top) - top)
                                      / max(1.0, top) for name, top in AIMS.items()
                                      if not m.get('fell')) for m in steady) / len(steady)
            + sum(looks.FELL if m.get('fell') else START_K * m.get('catches', 0) for m in starts))


def _write(path, what):
    """`what` as JSON at `path`, whole or not at all: written beside it and renamed - again
    while the host holds the new file (its scanner did, a robot died of it, 2026-10-05)."""
    with open(path + '.tmp', 'w') as f:
        json.dump(what, f)
    for k in range(100):
        try:
            return os.replace(path + '.tmp', path)
        except OSError:
            if k == 99:
                raise
            time.sleep(0.05)


def serve(root, paces=PACES):
    """A robot of the armada: candidates taken from `root`/todo by renaming, their results written
    to `root`/done, a sign of life in `root`/alive, until `root`/stop."""
    robot = Robot(paces)
    todo, doing, done, alive = (os.path.join(root, d) for d in ('todo', 'doing', 'done', 'alive'))
    mine, beat = '%d.json' % os.getpid(), 0.0
    while not os.path.exists(os.path.join(root, 'stop')):
        took = None
        try:
            # A file the host or a reader holds is tried again next round: a sign of life
            # written every pass, 8 robots of 12 died of it in a minute (2026-10-05).
            if time.time() - beat >= 1.0:
                with open(os.path.join(alive, mine), 'w') as f:
                    f.write('%f' % time.time())
                beat = time.time()
            for name in sorted(n for n in os.listdir(todo) if n.endswith('.json')):
                try:
                    os.rename(os.path.join(todo, name), os.path.join(doing, name + '.' + mine))
                    took = name
                    break
                except OSError:
                    continue
            if took is not None:
                with open(os.path.join(doing, took + '.' + mine)) as f:
                    task = json.load(f)
        except OSError:
            took = None
        if took is None:
            time.sleep(0.05)
            continue
        began = time.time()
        try:
            result = robot.tried(task['values'], task['pace'], task.get('way', 'steady'),
                                 task.get('seconds', 0.0))
        except Exception as e:                # a candidate's own: its trial lost, not the robot
            result = {'fell': 1.0, 'error': '%s: %s' % (type(e).__name__, e)}
        _write(os.path.join(done, took), dict(task, result=result, seconds=time.time() - began))
        for path in (os.path.join(doing, took + '.' + mine), ):
            try:
                os.remove(path)
            except OSError:
                pass
    try:
        os.remove(os.path.join(alive, mine))
    except OSError:
        pass
    robot.close()
    return 0


def robots(root):
    """How many robots serve `root` now: their signs of life under ALIVE_S old."""
    alive = os.path.join(root, 'alive')
    if not os.path.isdir(alive) or os.path.exists(os.path.join(root, 'stop')):
        return 0
    return sum(1 for name in os.listdir(alive)
               if time.time() - os.path.getmtime(os.path.join(alive, name)) < ALIVE_S)


class Armada:

    """Robots on the relay, fed from a directory: the one kept up (`--up`, ROOT) where it
    walks, else `count` of its own under build/, down again with it."""

    def __init__(self, count, paces=PACES, root=None):
        from tools.dev import focus
        self.own = root is not None or not robots(ROOT)
        self.root = root or (os.path.join(REPO, 'build', 'armada_%d' % os.getpid())
                             if self.own else ROOT)
        self.paces, self.ended, self.thread = paces, [], None
        self.tag = '%d_%d_' % (os.getpid(), int(time.time() * 1e3) % 10 ** 9)
        self.sent = 0
        if not self.own:
            return
        for d in ('todo', 'doing', 'done', 'alive'):
            os.makedirs(os.path.join(self.root, d), exist_ok=True)
        if os.path.exists(os.path.join(self.root, 'stop')):
            os.remove(os.path.join(self.root, 'stop'))
        jobs = [focus.Job('robot%d' % k, [sys.executable, '-X', 'utf8', os.path.abspath(__file__),
                                          '--serve', self.root, '--paces']
                          + ['%g' % p for p in paces], ROBOT_GB, 30 * 86400.0)
                for k in range(count)]
        self.thread = threading.Thread(target=lambda: self.ended.extend(focus.relay(jobs)),
                                       daemon=True)
        self.thread.start()

    def tried(self, candidates, starts=STARTS):
        """[{trial: measures}] for `candidates` [{name: value}], in their order: a pace's steady
        walk under its pace ('0.85'), a start under its way and pace ('squat 0.85')."""
        trials = [('%g' % pace, {'pace': pace}) for pace in self.paces] + [
            ('%s %g' % (way, pace), {'pace': pace, 'way': way, 'seconds': seconds})
            for way, pace, seconds in starts]
        names = []
        for values in candidates:
            for _key, task in trials:
                name = '%s%06d.json' % (self.tag, self.sent)
                self.sent += 1
                _write(os.path.join(self.root, 'todo', name), dict(task, values=values))
                names.append(name)
        out = {}
        while len(out) < len(names):
            for name in names:
                path = os.path.join(self.root, 'done', name)
                if name not in out and os.path.exists(path):
                    try:                      # held by the host's scanner: next round
                        with open(path) as f:
                            out[name] = json.load(f)['result']
                        os.remove(path)
                    except (OSError, ValueError):
                        pass
            if self.thread is not None and self.ended and not self.thread.is_alive():
                raise RuntimeError('the armada ended: %s' % self.ended[-1][1][-400:])
            self._requeue()
            time.sleep(0.05)
        each = len(trials)
        return [{key: out[names[k * each + i]] for i, (key, _task) in enumerate(trials)}
                for k in range(len(candidates))]

    def _requeue(self):
        """A task a robot took and died over, back in the queue."""
        doing, alive = os.path.join(self.root, 'doing'), os.path.join(self.root, 'alive')
        for name in os.listdir(doing):
            task, _dot, robot = name.partition('.json.')
            try:
                stale = time.time() - os.path.getmtime(os.path.join(alive, robot)) > ALIVE_S
            except OSError:
                stale = time.time() - os.path.getmtime(os.path.join(doing, name)) > ALIVE_S
            if stale:
                try:
                    os.replace(os.path.join(doing, name),
                               os.path.join(self.root, 'todo', task + '.json'))
                except OSError:
                    pass

    def close(self):
        if self.thread is not None:
            open(os.path.join(self.root, 'stop'), 'w').close()
            self.thread.join(60.0)


def shown(values, results):
    print('%-60s price %7.2f' % (' '.join('%s=%g' % kv for kv in values.items()) or 'as it is',
                                 price(results)))
    starts = ['%s %s' % (k, 'fell' if m.get('fell') else '%d' % m.get('catches', 0))
              for k, m in results.items() if ' ' in k]
    if starts:
        print('    starts, parries: ' + ' | '.join(starts))
    for pace, m in results.items():
        if ' ' in pace:
            continue
        if m.get('fell'):
            print('    %-5s fell' % pace)
            continue
        off = looks.broken(m)
        print('    %-5s %4.0f J/m %4.0f W %.2f m/s | knee %4.1f behind plumb %4.1f landing | leg '
              '%4.1f behind | hip %3.1f | toes back %4.1f mm | %d parries | head %2.0f %2.0f %2.0f '
              'mm | strike %3.0f N%s' % (
                  pace, m['energy'], m['power'], m['speed'], m.get('knee behind plumb', math.nan),
                  m.get('knee at landing', math.nan), m.get('leg behind plumb', math.nan),
                  m.get('hip over stance', math.nan), m.get('toes back at lift', math.nan),
                  m.get('catches', 0), m.get('head bob', math.nan),
                  m.get('head fore-aft', math.nan), m.get('head aside', math.nan),
                  m.get('strike', math.nan),
                  ' | off: ' + ', '.join('%s %.1f' % b[:2] for b in off) if off else ''))


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--set', nargs='*', default=[], metavar='NAME=V')
    parser.add_argument('--grid', nargs='*', default=[], metavar='NAME=V,V,..')
    parser.add_argument('--search', nargs='*', default=[], metavar='NAME=LOW:HIGH')
    parser.add_argument('--generations', type=int, default=12)
    parser.add_argument('--population', type=int, default=16)
    parser.add_argument('--sigma', type=float, default=0.15)
    parser.add_argument('--robots', type=int, default=16)
    parser.add_argument('--paces', nargs='*', type=float, default=list(PACES))
    parser.add_argument('--log', default=os.path.join(REPO, 'build', 'armada.jsonl'))
    parser.add_argument('--aim', nargs='*', default=[], metavar='MEASURE=MOST',
                        help='a search aims past the form: a measure at most this')
    parser.add_argument('--up', action='store_true', help='the armada kept walking')
    parser.add_argument('--down', action='store_true', help='the armada kept up, stopped')
    parser.add_argument('--serve', metavar='DIR', help='a robot of the armada')
    args = parser.parse_args(argv)
    background.lower()
    paces = tuple(args.paces)
    if args.serve:
        return serve(args.serve, paces)
    if args.down:
        open(os.path.join(ROOT, 'stop'), 'w').close()
        return 0
    if args.up:
        armada = Armada(args.robots, paces, ROOT)
        print('%d robots walking in at %s' % (args.robots, ROOT), flush=True)
        if armada.thread is not None:
            armada.thread.join()
        return 0
    AIMS.update((k, float(v)) for k, v in (a.split('=') for a in args.aim))
    fixed = {k: float(v) for k, v in (a.split('=') for a in args.set)}
    spans, cands = {}, [fixed]
    if args.search:
        spans = {k: tuple(float(x) for x in v.split(':'))
                 for k, v in (a.split('=') for a in args.search)}
    elif args.grid:
        import itertools
        axes = [(k, [float(x) for x in v.split(',')])
                for k, v in (a.split('=') for a in args.grid)]
        cands = [dict(fixed, **dict(zip([k for k, _v in axes], combo)))
                 for combo in itertools.product(*[v for _k, v in axes])]
    began = time.time()
    armada = Armada(min(args.robots, (len(paces) + len(STARTS))
                        * (args.population if spans else len(cands))), paces)
    try:
        if spans:
            def runs(_pool, candidates):
                got = armada.tried([dict(fixed, **c) for c in candidates])
                return [(price(r), sum(1.0 - m.get('fell', 0.0) for m in r.values()) / len(r),
                         math.nan, r) for r in got]
            with open(args.log, 'a') as log:
                cost, values = cmaes.search(None, spans, args.generations, args.population, log,
                                            runs, knobs.now, args.sigma)
            print('BEST %.2f %s' % (cost, json.dumps(values)))
            cands = [dict(fixed, **(values or {}))]
        for values, results in zip(cands, armada.tried(cands)):
            shown(values, results)
    finally:
        armada.close()
    print('%.0f s, %s' % (time.time() - began, 'its own robots' if armada.own
                          else 'the armada kept up'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
