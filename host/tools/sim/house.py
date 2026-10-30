#!/usr/bin/env python3
"""Her errands from every spot of the lodge: sat or lain, held, her joints against her own walk's.

    python tools/sim/house.py                      # sit_chair from every spot and heading, as F says it
    python tools/sim/house.py lie_bed --said       # said as the prompt says it, seen or not
    python tools/sim/house.py --one 0.32 1.01 1.37 sit_chair [--said] [--rise]   # one spot, here
    python tools/sim/house.py --docks              # the chair's sit and rise laid at its dock, off by a grid

A spot is a cell of a GRID_M grid on the lodge's free floor, CLEAR_M clear of every furniture box, at
each of four headings; what F offers there (`gym.offered`) is said as F says it (`gym.chose`), or
with --said as a prompt says it, whatever she sees. A run: seconds to the goal borne and held REST_S
(its rise's to up with --rise), her fall, what she bumped in which phase, the dock (along its normal,
along its edge, her heading off it, s), and each joint group's fastest swing, widest sweep, when the
fastest came and the seconds her master bus's passivity controller held it (`machine.law.supervisor`) - a
spasm is a group swung faster than her own walk swings it (`walked`), a held one caught by that tank.
"""
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

#: Each verb's goal, the rise said after it held REST_S, the seconds it may take.
GOALS = {'sit_chair': ('seated', 'rise_chair', 35.0), 'lie_bed': ('bed_rest', 'rise_bed', 45.0)}
#: The furniture her bumps are counted on, and the lodge's own boxes that are not.
FURNITURE = ('seat', 'back', 'chair_leg', 'bed', 'table', 'lamp', 'loft', 'stair')
SCENERY = ('floor', 'wall', 'roof', 'deck', 'door', 'pane', 'mullion', 'rail', 'post', 'stone', 'switch', 'shade')
#: Her joint groups a spasm is read on, and her walk's seconds they are judged against.
GROUPS = {'arms': ('left_shoulder', 'right_shoulder', 'left_elbow', 'right_elbow'), 'neck': ('neck', 'head'),
          'trunk': ('spine', 'spine_roll', 'waist')}
WALK_S = 8.0
#: The groups a spasm is judged on: her trunk's motion is the maneuver's own lean.
SPASM = ('arms', 'neck')
#: Her spots: a grid this fine, this clear of the furniture, four headings; a goal held this long; her
#: joints read this often, s.
GRID_M, CLEAR_M, REST_S, EVERY_S = 0.6, 0.45, 3.0, 0.01
HEADINGS = tuple(0.5 * math.pi * k for k in range(4))
#: The chair's dock sweep: off along its edge, her root off it, her heading off it.
DOCK_GRID = ((-0.06, -0.03, 0.0, 0.03, 0.06), (0.14, 0.19), (-0.1, -0.05, 0.0, 0.05, 0.1))


def _named(m, i, heads):
    return any(m.geom(i).name.startswith(h) for h in heads)


class Watch:

    """Her session stepped: each joint group's fastest swing (deg/s) and widest sweep (deg), her phases,
    what of hers bumped the furniture in which."""

    def __init__(self, x, z, yaw):
        from machine.world import physics
        from machine.process import sprung, worker
        from machine.body.figure import JOINTS
        physics.ROOM, physics.SIGHT = 1.0, 0.0
        self.s = s = worker.session()
        sprung.relaid(s['g'], (x, z), yaw)
        self.g, self.world = s['g'], s['g']['world']
        m = self.world.model
        self.furn = {i for i in range(m.ngeom) if _named(m, i, FURNITURE) and float(self.world.data.geom_xpos[i][1]) > -0.5}
        self.hers = {i for i in range(m.ngeom) if int(m.geom_bodyid[i]) > 0 and i not in self.furn and not _named(m, i, SCENERY)}
        self.last, self.read, self.fast, self.lo, self.hi, self.when, self.held = None, -1.0, {}, {}, {}, {}, {}
        self.phases, self.bumps = [], {}
        self.driven = [JOINTS[j] for j in self.g['joints']]
        self.go(0.5)
        self.fast, self.lo, self.hi, self.when, self.held = {}, {}, {}, {}, {}

    def go(self, seconds, until=None):
        """`seconds` of passes, or to her fall or `until(self)`: whether `until` came."""
        from machine.process import sprung, worker
        g, t0 = self.g, self.g['t']
        while g['t'] - t0 < seconds and self.s['fallen_at'] is None:
            worker.stepped(self.s)
            self._read()
            er = g.get('_errand') or {}
            tag = '%s/%s' % (er.get('phase'), sprung.doing(g))
            q = self.world.data.qpos
            self.phases += [(round(g['t'], 2), tag, round(float(q[0]), 2), round(float(q[2]), 2))] if not self.phases or self.phases[-1][1] != tag else []
            if until is not None and until(self):
                return True
        return False

    def _read(self):
        from machine.process import sprung, worker
        g, d, m = self.g, self.world.data, self.world.model
        if g['t'] - self.read < EVERY_S:
            return
        angles, dt = worker.told(self.s)['angles'], g['t'] - self.read
        for name, joints in GROUPS.items():
            for j in joints:
                v = float(angles.get(j, 0.0))
                if self.last is not None and abs(v - self.last.get(j, v)) / dt > self.fast.get(name, 0.0):
                    self.fast[name] = abs(v - self.last.get(j, v)) / dt
                    self.when[name] = (round(g['t'], 2), j, sprung.doing(g), (g.get('_errand') or {}).get('phase'))
                self.lo[name], self.hi[name] = min(self.lo.get(name, v), v), max(self.hi.get(name, v), v)
        held = dict(zip(self.driven, g['bus'][2]['alpha']))
        for name, joints in GROUPS.items():
            self.held[name] = self.held.get(name, 0.0) + dt * float(max((held.get(j, 0.0) for j in joints), default=0.0) > 0.0)
        self.last, self.read = {j: float(angles.get(j, 0.0)) for js in GROUPS.values() for j in js}, g['t']
        phase = (g.get('_errand') or {}).get('phase')
        for k in range(d.ncon):
            a, b = int(d.contact[k].geom1), int(d.contact[k].geom2)
            for f, o in ((a, b), (b, a)):
                if f in self.furn and o in self.hers:
                    self.bumps.setdefault('%s@%s' % (m.geom(f).name, phase), round(g['t'], 2))

    def spasm(self):
        return {name: (round(self.fast.get(name, 0.0)), round(self.hi.get(name, 0.0) - self.lo.get(name, 0.0)),
                       self.when.get(name), round(self.held.get(name, 0.0), 2)) for name in GROUPS}


def walked():
    """Each joint group's fastest swing and widest sweep over WALK_S of her walk: a spasm's bound."""
    from machine.process import worker
    w = Watch(0.0, -1.0, 0.0)
    worker.apply(w.s, {'twist': (0.69, 0.0, 0.0)})
    w.go(WALK_S)
    return w.spasm()


def _goal(w, goal, rise, limit, t0):
    """(seconds to `goal` borne, held REST_S, seconds to up once `rise` is said - up held REST_S -, None
    unsaid) from `t0`. Counted up the moment she stood, 7 chair rises of 58 that fell within 3 s were risen."""
    from machine.law import modal
    from machine.process import sprung, worker
    g = w.g
    def borne(q):
        return sprung.doing(g) == goal and float(g['y'][modal.PROP]) > 0.45
    reached = round(g['t'] - t0, 2) if w.go(limit - (g['t'] - t0), borne) else None
    if reached is not None:
        w.go(REST_S)
    held = reached is not None and w.s['fallen_at'] is None and sprung.doing(g) == goal
    up = None
    if held and rise:
        worker.apply(w.s, {'say': rise})
        t2 = g['t']
        if w.go(12.0, lambda q: sprung.doing(g) in ('stand', 'rest') and float(g['y'][modal.PROP]) < 0.05):
            up = round(g['t'] - t2, 2)
            w.go(REST_S)
    return reached, held, up


def one(x, z, yaw, verb, said=False, rise=False):
    """Her errand `verb` from (x, z, `yaw`), as F says it or `said` as a prompt does: a line's fields."""
    from types import SimpleNamespace as SN
    from machine.senses import edge
    from machine.process import worker
    from terminal.views import gym, scanner, viewpoint
    w = Watch(x, z, yaw)
    g = w.g
    st = {'body': SN(send=lambda **cmd: worker.apply(w.s, cmd)), 'scanner': scanner.VisionScanner(),
          'now': worker.told(w.s), 'yaw': 180.0, 'tour': None, **viewpoint.HOME}
    offers = gym.offered(st)
    out = {'x': x, 'z': z, 'yaw': round(yaw, 3), 'verb': verb, 'said': said}
    pick = [k for k, o in enumerate(offers) if o[1] == verb]
    if not said and not pick:
        return dict(out, result='not offered')
    if said:
        worker.apply(w.s, {'say': verb})
    else:
        out['key'] = offers[pick[0]][0]
        gym.chose(st, pick[0])
    t0, dock = g['t'], []
    goal, rise_verb, limit = GOALS[verb]

    def docked(q):
        er = g.get('_errand') or {}
        if er.get('phase') == 'done' and not dock:
            p = w.world.data.qpos
            dx, dz = float(p[0]) - er['x_dock'], float(p[2]) - er['z_dock']
            dock.extend((round(dx * er['nx'] + dz * er['nz'], 3), round(dx * er['tx'] + dz * er['tz'], 3),
                         round(math.remainder(float(edge.yaw(g['sense'])) - er['hdd'], 2.0 * math.pi), 3),
                         round(g['t'] - t0, 2)))
        return not g.get('_errand')
    w.go(limit, docked)
    reached, held, up = _goal(w, goal, rise_verb if rise else None, limit, t0)
    fell = None if w.s['fallen_at'] is None else round(w.s['fallen_at'] - t0, 2)
    ok = held and fell is None and (up is not None or not rise)
    return dict(out, dock=dock or None, reached=reached, held=held, up=up, fell=fell, spasm=w.spasm(), bumps=w.bumps,
                phases=w.phases[-12:], result='ok' if ok else ('fell' if fell is not None else 'short'))


def dock(dx, spar, dyaw):
    """The chair's sit laid at its dock `dx` along its edge, her root `spar` off it, `dyaw` turned off
    it; held, risen: a line's fields."""
    from machine.world import room
    from machine.process import worker
    w = Watch(room.CHAIR_AT[0] + dx, room.CHAIR_AT[1] - room.SEAT[2] - spar, math.pi + dyaw)
    worker.apply(w.s, {'say': 'sit_chair'})
    reached, held, up = _goal(w, 'seated', 'rise_chair', 15.0, w.g['t'])
    fell = None if w.s['fallen_at'] is None else round(w.s['fallen_at'], 2)
    return {'dx': dx, 'spar': spar, 'dyaw': dyaw, 'reached': reached, 'held': held, 'up': up, 'fell': fell,
            'spasm': w.spasm(), 'result': 'ok' if up is not None and fell is None else ('fell' if fell is not None else 'short')}


def spots():
    """(x, z) on the lodge's free floor: a GRID_M grid CLEAR_M clear of every furniture box."""
    from machine.world import physics, room
    from machine.process import worker
    physics.ROOM, physics.SIGHT = 1.0, 0.0
    world = worker.session()['g']['world']
    m, d = world.model, world.data
    boxes = [(float(d.geom_xpos[i][0]), float(d.geom_xpos[i][2]), float(m.geom_size[i][0]), float(m.geom_size[i][2]))
             for i in range(m.ngeom) if _named(m, i, FURNITURE) and float(d.geom_xpos[i][1]) > -0.5]
    xs = [round(-room.WIDE + 0.5 + GRID_M * k, 2) for k in range(int((2.0 * room.WIDE - 1.0) / GRID_M) + 1)]
    zs = [round(room.FROM + 0.5 + GRID_M * k, 2) for k in range(int((room.TO - room.FROM - 1.0) / GRID_M) + 1)]
    return [(x, z) for x in xs for z in zs
            if all(abs(x - bx) > hx + CLEAR_M or abs(z - bz) > hz + CLEAR_M for bx, bz, hx, hz in boxes)]


def _relayed(argvs):
    """Each run its own process on the relay's batons (`focus.relay`): their lines' fields."""
    from tools.dev import focus
    jobs = [focus.Job('house_%d' % k, [sys.executable, '-X', 'utf8', os.path.abspath(__file__)] + a, 0.4, 900.0, None)
            for k, a in enumerate(argvs)]
    lines = [[ln for ln in (text or '').splitlines() if ln.startswith('{')] for _job, text, _code, _s in focus.relay(jobs)]
    return [dict(json.loads(ln[-1])) if ln else dict(result='crashed', spasm={}) for ln in lines]


def main(argv=None):
    p = argparse.ArgumentParser(description=(__doc__ or '').split('\n\n')[0])
    p.add_argument('verbs', nargs='*', help='errands: %s' % ', '.join(GOALS))
    p.add_argument('--one', nargs=4, metavar=('X', 'Z', 'YAW', 'VERB'), help='one spot, here')
    p.add_argument('--dock', nargs=3, type=float, metavar=('DX', 'SPAR', 'DYAW'), help='one dock, here')
    p.add_argument('--docks', action='store_true', help='the chair at its dock, off by DOCK_GRID, on the relay')
    p.add_argument('--said', action='store_true', help='said as a prompt says it, seen or not')
    p.add_argument('--rise', action='store_true', help='its rise said once it held')
    p.add_argument('--walked', action='store_true', help='her walk\'s fastest swings, here')
    args = p.parse_args(argv)
    flags = ['--said'] * args.said + ['--rise'] * args.rise
    if args.walked:
        print(json.dumps(walked()), flush=True)
        return 0
    if args.one:
        print(json.dumps(one(float(args.one[0]), float(args.one[1]), float(args.one[2]), args.one[3], args.said, args.rise)), flush=True)
        return 0
    if args.dock:
        print(json.dumps(dock(*args.dock)), flush=True)
        return 0
    runs = ([['--dock', str(a), str(b), str(c)] for a in DOCK_GRID[0] for b in DOCK_GRID[1] for c in DOCK_GRID[2]]
            if args.docks else [['--one', str(x), str(z), str(h), v] + flags for x, z in spots() for h in HEADINGS
                                for v in (args.verbs or ['sit_chair'])])
    bound = _relayed([['--walked']])[0]
    rows = _relayed(runs)
    counts = {}
    spasms = {}
    for k, r in enumerate(rows):
        got = r.get('spasm')
        spasms[k] = [n for n, (fast, *_more) in (got if isinstance(got, dict) else {}).items() if n in SPASM and fast > bound.get(n, (math.inf,))[0]]
        spasm = spasms[k]
        key = '%s%s' % (r.get('result'), ' spasm' if spasm and r.get('result') == 'ok' else '')
        counts[key] = counts.get(key, 0) + 1
    print('walk %s | %s' % (bound, '  '.join('%s %d' % kv for kv in sorted(counts.items()))), flush=True)
    for k, r in enumerate(rows):
        if r.get('result') not in ('ok', 'not offered') or spasms[k]:
            print(json.dumps(dict(r, spasms=spasms[k])), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
