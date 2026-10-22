#!/usr/bin/env python3
"""Her errands from every spot of the lodge: the thing a verb goes to said as the page's F says it.

    python tools/sim/house.py                      # sit_chair from every spot and heading, on the relay
    python tools/sim/house.py lie_bed sit_chair    # those verbs
    python tools/sim/house.py --one 0.32 1.01 1.37 sit_chair    # one spot, its line and its phases
    python tools/sim/house.py --docks              # the chair's sit and rise laid at its dock, off by a grid

A spot is a cell of a GRID_M grid on the lodge's free floor, CLEAR_M clear of every furniture box, at
each of four headings; what F offers there (`gym.offered`, her identified things) is said as F says
it (`gym.chose`). A line: the key, the dock (along its normal, along its edge, her heading off it, s),
seconds to the goal held REST_S, its rise's seconds to up, her fall, what she bumped in which phase.
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
#: Her spots: a grid this fine, this clear of the furniture, four headings; a goal held this long.
GRID_M, CLEAR_M, REST_S = 0.6, 0.45, 3.0
HEADINGS = tuple(0.5 * math.pi * k for k in range(4))
#: The chair's dock sweep: off along its edge, her root off it, her heading off it.
DOCK_GRID = ((-0.06, -0.03, 0.0, 0.03, 0.06), (0.14, 0.19), (-0.1, -0.05, 0.0, 0.05, 0.1))


def _named(m, i, heads):
    return any(m.geom(i).name.startswith(h) for h in heads)


def _laid(x, z, yaw):
    """Her session in the lodge, her laid at (x, z) facing `yaw` and stood half a second."""
    from machine import physics, sprung, worker
    physics.ROOM, physics.SIGHT = 1.0, 0.0
    s = worker.session()
    sprung.relaid(s['g'], (x, z), yaw)
    for _ in range(int(round(0.5 / worker.DT))):
        worker.stepped(s)
    return s


def _held(s, goal, rise, limit, t0):
    """(seconds to `goal` borne, held REST_S, seconds to up once `rise` is said) from `t0`."""
    from machine import modal, sprung, worker
    g, reached, held, up = s['g'], None, None, None
    while g['t'] - t0 < limit and s['fallen_at'] is None and reached is None:
        worker.stepped(s)
        reached = round(g['t'] - t0, 2) if sprung.doing(g) == goal and float(g['y'][modal.PROP]) > 0.45 else None
    t1 = g['t']
    while reached is not None and g['t'] - t1 < REST_S and s['fallen_at'] is None:
        worker.stepped(s)
    held = reached is not None and s['fallen_at'] is None and sprung.doing(g) == goal
    if held:
        worker.apply(s, {'say': rise})
    t2 = g['t']
    while held and up is None and g['t'] - t2 < 12.0 and s['fallen_at'] is None:
        worker.stepped(s)
        up = round(g['t'] - t2, 2) if sprung.doing(g) in ('stand', 'rest') and float(g['y'][modal.PROP]) < 0.05 else None
    return reached, held, up


def one(x, z, yaw, verb):
    """Her errand `verb` from (x, z, `yaw`) as F says it: a line's fields."""
    from types import SimpleNamespace as SN
    from machine import edge, sprung, worker
    from terminal.views import gym, scanner, viewpoint
    s = _laid(x, z, yaw)
    g, world = s['g'], s['g']['world']
    m = world.model
    furn = {i for i in range(m.ngeom) if _named(m, i, FURNITURE) and float(world.data.geom_xpos[i][1]) > -0.5}
    hers = {i for i in range(m.ngeom) if int(m.geom_bodyid[i]) > 0 and i not in furn and not _named(m, i, SCENERY)}
    st = {'body': SN(send=lambda **cmd: worker.apply(s, cmd)), 'scanner': scanner.VisionScanner(),
          'now': worker.told(s), 'yaw': 180.0, 'tour': None, **viewpoint.HOME}
    offers = gym.offered(st)
    out = {'x': x, 'z': z, 'yaw': round(yaw, 3), 'verb': verb, 'offers': [(o[0], o[1], o[4]) for o in offers]}
    pick = [k for k, o in enumerate(offers) if o[1] == verb]
    if not pick:
        return dict(out, result='not offered')
    out['key'] = offers[pick[0]][0]
    gym.chose(st, pick[0])
    t0, phases, bumps, dock = g['t'], [], {}, None
    goal, rise, limit = GOALS[verb]
    while g.get('_errand') and g['t'] - t0 < limit and s['fallen_at'] is None:
        worker.stepped(s)
        er, q = g.get('_errand') or {}, world.data.qpos
        tag = '%s/%s' % (er.get('phase'), sprung.doing(g))
        phases += [(round(g['t'] - t0, 2), tag, round(float(q[0]), 2), round(float(q[2]), 2))] if not phases or phases[-1][1] != tag else []
        dx, dz = float(q[0]) - er.get('x_dock', 0.0), float(q[2]) - er.get('z_dock', 0.0)
        dock = dock or ((round(dx * er['nx'] + dz * er['nz'], 3), round(dx * er['tx'] + dz * er['tz'], 3),
                         round(math.remainder(float(edge.yaw(g['sense'])) - er['hdd'], 2.0 * math.pi), 3),
                         round(g['t'] - t0, 2)) if er.get('phase') == 'done' else None)
        for k in range(world.data.ncon):
            c = world.data.contact[k]
            a, b = int(c.geom1), int(c.geom2)
            for f, o in ((a, b), (b, a)):
                if f in furn and o in hers:
                    bumps.setdefault('%s@%s' % (m.geom(f).name, er.get('phase')), round(g['t'] - t0, 2))
    reached, held, up = _held(s, goal, rise, limit, t0)
    fell = None if s['fallen_at'] is None else round(s['fallen_at'] - t0, 2)
    result = 'ok' if up is not None and fell is None else ('fell' if fell is not None else 'short')
    return dict(out, dock=dock, reached=reached, held=held, up=up, fell=fell, bumps=bumps, phases=phases[-12:], result=result)


def dock(dx, spar, dyaw):
    """The chair's sit laid at its dock `dx` along its edge, her root `spar` off it, `dyaw` turned off
    it; held, risen: a line's fields."""
    from machine import room, worker
    s = _laid(room.CHAIR_AT[0] + dx, room.CHAIR_AT[1] - room.SEAT[2] - spar, math.pi + dyaw)
    worker.apply(s, {'say': 'sit_chair'})
    reached, held, up = _held(s, 'seated', 'rise_chair', 15.0, s['g']['t'])
    fell = None if s['fallen_at'] is None else round(s['fallen_at'], 2)
    return {'dx': dx, 'spar': spar, 'dyaw': dyaw, 'reached': reached, 'held': held, 'up': up, 'fell': fell,
            'result': 'ok' if up is not None and fell is None else ('fell' if fell is not None else 'short')}


def spots():
    """(x, z) on the lodge's free floor: a GRID_M grid CLEAR_M clear of every furniture box."""
    from machine import physics, room, worker
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
    return [json.loads(ln[-1]) if ln else {'result': 'crashed'} for ln in lines]


def main(argv=None):
    p = argparse.ArgumentParser(description=(__doc__ or '').split('\n\n')[0])
    p.add_argument('verbs', nargs='*', help='errands: %s' % ', '.join(GOALS))
    p.add_argument('--one', nargs=4, metavar=('X', 'Z', 'YAW', 'VERB'), help='one spot, here')
    p.add_argument('--dock', nargs=3, type=float, metavar=('DX', 'SPAR', 'DYAW'), help='one dock, here')
    p.add_argument('--docks', action='store_true', help='the chair at its dock, off by DOCK_GRID, on the relay')
    args = p.parse_args(argv)
    if args.one:
        print(json.dumps(one(float(args.one[0]), float(args.one[1]), float(args.one[2]), args.one[3])), flush=True)
        return 0
    if args.dock:
        print(json.dumps(dock(*args.dock)), flush=True)
        return 0
    runs = ([['--dock', str(a), str(b), str(c)] for a in DOCK_GRID[0] for b in DOCK_GRID[1] for c in DOCK_GRID[2]]
            if args.docks else [['--one', str(x), str(z), str(h), v] for x, z in spots() for h in HEADINGS
                                for v in (args.verbs or ['sit_chair'])])
    rows = _relayed(runs)
    counts = {}
    for r in rows:
        counts[r.get('result')] = counts.get(r.get('result'), 0) + 1
    print('  '.join('%s %d' % kv for kv in sorted(counts.items(), key=lambda kv: str(kv[0]))), flush=True)
    for r in rows:
        if r.get('result') not in ('ok', 'not offered'):
            print(json.dumps(r), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
