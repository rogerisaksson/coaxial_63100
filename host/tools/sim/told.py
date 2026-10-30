#!/usr/bin/env python3
"""Her scenarios, each alone on her loop as the page runs her (`worker.session`): to iterate on one.

    python tools/sim/told.py                     # every scenario, side by side on the relay
    python tools/sim/told.py chair bed           # those named, here
    python tools/sim/told.py chair --trace       # and its phases, a line each

A scenario a line: what a player's keys, F or a click send, the seconds from it to what it asks -
past 0.3 m/s, still, seated, lying, up, out of the door -, the flips of her doing meanwhile,
whether she fell; her arms' fastest swing and widest sweep while it runs, a sit's or a lie's
furniture's share of her held, a sit's measures off a human's (`seats.SEAT`) where out of its band.
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

#: The keys' twists: W, S's stop, A, W with D.
WALK, STOP, LEFT, TURN = (0.69, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.5, 0.0), (0.69, 0.0, -0.7)
#: Past this she moves, under this for STILL_S she is still, m/s.
MOVING_M_S, STILL_M_S, STILL_S = 0.3, 0.05, 0.5
#: Her arms' joints, measured.
ARMS = ('left_shoulder', 'right_shoulder', 'left_elbow', 'right_elbow')
#: A sit or a lie held this long before its rise, s.
HELD_S = 3.0


class Run:
    """Her session laid on `scene` (`scenarios.placed`) or in her room (`room`), stepped, her doing's
    flips counted, her arms and her support logged and a sit's joints kept (`seats.measured`)."""

    def __init__(self, scene='flat', room=False):
        from machine.world import physics
        from machine.process import worker
        from tools.sim import scenarios
        physics.ROOM, physics.SIGHT = float(room), 0.0
        self.s = worker.session()
        if not room:
            scenarios.placed(self.s, scene)
        self.last, self.flips, self.calm, self.log, self.rows, self.told = None, 0, None, [], [], -1.0
        self.go(0.5)

    @property
    def t(self):
        return self.s['g']['t']

    def speed(self):
        v = self.s['g']['est']['vel']
        return math.hypot(float(v[0]), float(v[2]))

    def go(self, seconds, until=None):
        """`seconds` of passes, or to her fall: the seconds until `until(self)` first holds, or None."""
        from machine.process import sprung, worker
        t0, got = self.t, None
        while self.t - t0 < seconds and self.s['fallen_at'] is None:
            worker.stepped(self.s)
            doing = sprung.doing(self.s['g'])
            self.flips += doing != self.last and self.last is not None
            self.last = doing
            self._logged(doing)
            if got is None and until is not None and until(self):
                got = round(self.t - t0, 2)
        return got

    def _logged(self, doing):
        from machine.law import modal
        from machine.process import worker
        from terminal.views.humanoid_keys import HEADER, row
        from tools.sim import look
        g = self.s['g']
        if self.t - self.told < 1.0 / look.RATE_HZ:
            return
        self.told, now = self.t, worker.told(self.s)
        self.log.append((self.t, doing, [float(now['angles'].get(j, 0.0)) for j in ARMS],
                         sum(g['loads']) / float(g['fr'][modal.WEIGHT]), float(g['y'][modal.PROP]),
                         float(g['world'].data.qpos[1])))
        self.rows.append(dict(zip(HEADER, row(dict(now, stage='sit'), 60.0))))

    def tell(self, **command):
        from machine.process import worker
        self.flips = 0
        worker.apply(self.s, command)

    def arms(self, t0=0.0, t1=math.inf):
        """(fastest swing deg/s, widest sweep deg) of her arms' joints between `t0` and `t1` s."""
        from coaxial.model.blocks import numpy as np
        at = [r for r in self.log if t0 <= r[0] <= t1]
        if len(at) < 3:
            return 0.0, 0.0
        t, a = np.array([r[0] for r in at]), np.array([r[2] for r in at])
        return float(np.abs(np.gradient(a, t, axis=0)).max()), float((a.max(axis=0) - a.min(axis=0)).max())

    def borne(self, t0, t1):
        """The furniture's share of her, mean, between `t0` and `t1` s."""
        at = [r[4] for r in self.log if t0 <= r[0] <= t1]
        return sum(at) / max(len(at), 1)

    def sat(self, t0, t1):
        """A sit's measures off a human's between `t0` and `t1` s (`seats.out`): [(name, value, lo, hi)]."""
        from tools.sim import fbx, seats
        rows = [r for r in self.rows if t0 <= float(r['t']) <= t1]
        return seats.out(seats.measured(*fbx.joints(rows))) if len(rows) > 10 else []

    def trace(self):
        """Her phases a line each: span, her arms' fastest swing and widest sweep, her soles' and
        the furniture's share, her pelvis's height at its end."""
        lines, k0 = [], 0
        for k in range(1, len(self.log) + 1):
            if k == len(self.log) or self.log[k][1] != self.log[k0][1]:
                t0, t1 = self.log[k0][0], self.log[k - 1][0]
                fast, wide = self.arms(t0, t1)
                last = self.log[k - 1]
                lines.append('  %-28s %6.2f %5.2f s | arms %5.0f deg/s %5.1f deg | soles %4.2f prop %4.2f pelvis %.3f'
                             % (self.log[k0][1][:28], t0, t1 - t0, fast, wide, last[3], last[4], last[5]))
                k0 = k
        return lines


def moving(r):
    return r.speed() > MOVING_M_S


def still(r):
    r.calm = (r.calm if r.calm is not None else r.t) if r.speed() < STILL_M_S else None
    return r.calm is not None and r.t - r.calm >= STILL_S


def doing(*names):
    from machine.process import sprung
    return lambda r: sprung.doing(r.s['g']) in names


def _arms(r, t0=0.0):
    return 'arms %.0f deg/s %.0f deg' % r.arms(t0)


def errand_walk():
    r = Run()
    r.tell(twist=WALK)
    went = r.go(3.0, moving)
    r.go(5.0)
    return 'W from a stand: moving at %s s, %d flips, %s' % (went, r.flips, _arms(r, 2.0)), r


def errand_stop():
    r = Run()
    r.tell(twist=WALK)
    r.go(3.0)
    r.tell(twist=STOP)
    stood = r.go(6.0, still)
    return 'stopped after 3 s walking: still at %s s, %d flips' % (stood, r.flips), r


def errand_walk_again():
    times = []
    for after in (0.0, 0.5, 1.0, 2.0, 3.0, 4.0):
        r = Run()
        r.tell(twist=WALK)
        r.go(3.0)
        r.tell(twist=STOP)
        r.go(after)
        r.tell(twist=WALK)
        times.append('%.1f: %s' % (after, r.go(3.0, moving)))
    return 'W again into a stop, s to moving: ' + ', '.join(times), r


def errand_sidestep():
    r = Run()
    r.tell(twist=LEFT)
    hd = r.s['g']['heading']

    def aside(q):
        v = q.s['g']['est']['vel']
        return abs(float(v[0]) * math.cos(hd) - float(v[2]) * math.sin(hd)) > 0.2
    side = r.go(3.0, aside)
    return 'A from a stand: aside past 0.2 m/s at %s s, %d flips' % (side, r.flips), r


def errand_turn():
    r = Run()
    r.tell(twist=WALK)
    r.go(2.0)
    h0 = r.s['g']['heading']
    r.tell(twist=TURN)
    turned = r.go(3.0, lambda q: abs(q.s['g']['heading'] - h0) > 0.3)
    return 'W and D walking: turned 0.3 rad at %s s, %d flips' % (turned, r.flips), r


def _held(r, verb, rest, rise, seconds):
    """`verb` said, the seconds to `rest`, held HELD_S, `rise` said, the seconds to up: a line."""
    t0 = r.t
    r.tell(say=verb)
    there = r.go(seconds, doing(rest))
    flips, t1 = r.flips, r.t
    r.go(HELD_S)
    t2 = r.t
    r.tell(say=rise)
    up = r.go(8.0, doing('stand', 'rest'))
    out = ', '.join('%s %.2f (%g..%g)' % m for m in r.sat(t0, t2)) if verb == 'sit_chair' else ''
    return ('%s at %s s (%d flips), held %.2f of her, up at %s s (%d flips); arms down %.0f deg/s %.0f deg, '
            'held %.0f deg/s %.0f deg, up %.0f deg/s %.0f deg%s'
            % (rest, there, flips, r.borne(t1, t2), up, r.flips, *r.arms(t0, t1), *r.arms(t1, t2), *r.arms(t2),
               '; off a human: ' + out if out else ''))


def errand_chair():
    r = Run('chair_behind')
    return 'F at a chair: ' + _held(r, 'sit_chair', 'seated', 'rise_chair', 6.0), r


def errand_bed():
    r = Run('bed_behind')
    return 'F at a bed: ' + _held(r, 'lie_bed', 'bed_rest', 'rise_bed', 8.0), r


def errand_chair_errand():
    from machine.world import room
    r = Run(room=True)
    t0 = r.t
    r.tell(say='sit_chair', at=room.CHAIR_AT)
    seated = r.go(25.0, doing('seated'))
    r.go(HELD_S)
    return 'clicked the chair: seated at %s s, %s' % (seated, _arms(r, t0)), r


def errand_bed_errand():
    from machine.world import room
    r = Run(room=True)
    t0 = r.t
    r.tell(say='lie_bed', at=room.BED_AT)
    lying = r.go(30.0, doing('bed_rest'))
    r.go(HELD_S)
    return 'clicked the bed: lying at %s s, %s' % (lying, _arms(r, t0)), r


def errand_door():
    from machine.world import room
    r = Run(room=True)
    t0 = r.t
    r.tell(say='exit_door')
    out = r.go(30.0, lambda q: float(q.s['g']['world'].data.qpos[2]) > room.TO)
    return 'told the door: out at %s s, %s' % (out, _arms(r, t0)), r


def errand_hop():
    r = Run()
    r.tell(say='hop')
    stood = r.go(6.0, lambda q: still(q) and doing('rest', 'stand')(q))
    return 'hop from a stand: still at %s s, %d flips' % (stood, r.flips), r


ERRANDS = {name[7:]: f for name, f in sorted(globals().items()) if name.startswith('errand_')}


def _relayed(names):
    """Each scenario its own process on the relay's batons (`focus.relay`), its lines in order."""
    from tools.dev import focus
    jobs = [focus.Job(name, [sys.executable, '-X', 'utf8', os.path.abspath(__file__), name], 1.5, 900.0, None)
            for name in names]
    got = {job.name: (text or '').strip().splitlines()[-1:] for job, text, _code, _s in focus.relay(jobs)}
    for name in names:
        print((got.get(name) or ['%-14s no line' % name])[0], flush=True)


def main(argv=None):
    p = argparse.ArgumentParser(description=(__doc__ or '').split('\n\n')[0])
    p.add_argument('names', nargs='*', help='scenarios: %s' % ', '.join(ERRANDS))
    p.add_argument('--trace', action='store_true', help='each its phases, a line each')
    args = p.parse_args(argv)
    if not args.names:
        _relayed(list(ERRANDS))
        return 0
    for name in args.names:
        line, r = ERRANDS[name]()
        fell = r.s['fallen_at']
        lines = r.trace() if args.trace else []
        print('\n'.join(lines + ['%-14s %s%s' % (name, line, '' if fell is None else ', fell at %.2f s' % fell)]), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
