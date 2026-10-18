#!/usr/bin/env python3
"""How she answers what she is told, on her loop as the page runs her (`worker.session`).

    python tools/sim/told.py                 # every errand
    python tools/sim/told.py walk_again bed  # those named

An errand a line: what a player's keys and F send, and the seconds from it to what it asks -
past 0.3 m/s, still (under 0.05 m/s for 0.5 s), seated, lying, up -, the flips of her doing
meanwhile, whether she fell.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

#: The keys' twists: W, S's stop, A, W with D.
WALK, STOP, LEFT, TURN = (0.69, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.5, 0.0), (0.69, 0.0, -0.7)
#: Past this she moves, under this for STILL_S she is still, m/s.
MOVING_M_S, STILL_M_S, STILL_S = 0.3, 0.05, 0.5


class Run:
    """Her session laid on `scene` (`scenarios.placed`), stepped, her doing's flips counted."""

    def __init__(self, scene='flat'):
        from machine import worker
        from tools.sim import scenarios
        self.s = worker.session()
        scenarios.placed(self.s, scene)
        self.last, self.flips, self.calm = None, 0, None
        self.go(0.5)

    @property
    def t(self):
        return self.s['g']['t']

    def speed(self):
        v = self.s['g']['est']['vel']
        return math.hypot(float(v[0]), float(v[2]))

    def go(self, seconds, until=None):
        """`seconds` of passes, or to her fall: the seconds until `until(self)` first holds, or None."""
        from machine import sprung, worker
        t0, got = self.t, None
        while self.t - t0 < seconds and self.s['fallen_at'] is None:
            worker.stepped(self.s)
            doing = sprung.doing(self.s['g'])
            self.flips += doing != self.last and self.last is not None
            self.last = doing
            if got is None and until is not None and until(self):
                got = round(self.t - t0, 2)
        return got

    def tell(self, **command):
        from machine import worker
        self.flips = 0
        worker.apply(self.s, command)


def moving(r):
    return r.speed() > MOVING_M_S


def still(r):
    r.calm = (r.calm if r.calm is not None else r.t) if r.speed() < STILL_M_S else None
    return r.calm is not None and r.t - r.calm >= STILL_S


def doing(*names):
    from machine import sprung
    return lambda r: sprung.doing(r.s['g']) in names


def errand_walk():
    r = Run()
    r.tell(twist=WALK)
    went = r.go(3.0, moving)
    return 'W from a stand: moving at %s s, %d flips' % (went, r.flips), r


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


def errand_chair():
    r = Run('chair_behind')
    r.tell(say='sit_chair')
    seated = r.go(6.0, doing('seated'))
    sat_flips = r.flips
    r.go(1.0)
    r.tell(say='rise_chair')
    up = r.go(7.0, doing('stand', '#201', 'rest'))
    return 'F at a chair: seated at %s s (%d flips), up at %s s (%d flips)' % (seated, sat_flips, up, r.flips), r


def errand_bed():
    r = Run('bed_behind')
    r.tell(say='lie_bed')
    lying = r.go(8.0, doing('bed_rest'))
    lay_flips = r.flips
    r.go(1.0)
    r.tell(say='rise_bed')
    up = r.go(8.0, doing('stand', '#201', 'rest'))
    return 'F at a bed: lying at %s s (%d flips), up at %s s (%d flips)' % (lying, lay_flips, up, r.flips), r


def errand_hop():
    r = Run()
    r.tell(say='hop')
    stood = r.go(6.0, lambda q: still(q) and doing('rest', '#201', 'stand')(q))
    return 'hop from a stand: still at %s s, %d flips' % (stood, r.flips), r


ERRANDS = {name[7:]: f for name, f in sorted(globals().items()) if name.startswith('errand_')}


def main(argv=None):
    names = (sys.argv[1:] if argv is None else argv) or list(ERRANDS)
    for name in names:
        line, r = ERRANDS[name]()
        fell = r.s['fallen_at']
        print('%-11s %s%s' % (name, line, '' if fell is None else ', fell at %.2f s' % fell), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
