#!/usr/bin/env python3
"""Her errands from every spot of the lodge: sat or lain, held, her joints against her own walk's.

    python tools/sim/house.py                      # sit_chair from every spot and heading, as F says it
    python tools/sim/house.py lie_bed --said       # said as the prompt says it, seen or not
    python tools/sim/house.py --one 0.32 1.01 1.37 sit_chair [--said] [--rise] [--trace]   # one spot, here
    python tools/sim/house.py --one 0.5 0.2 4.712 exit_door --said --record NAME           # and her loop recorded
    python tools/sim/house.py --docks              # the chair's sit and rise laid at its dock, off by a grid
    python tools/sim/house.py lie_bed --docked --said --rise   # at her dock on each face, her back to it

A spot is a cell of a GRID_M grid on the lodge's free floor, CLEAR_M clear of every furniture box, at
each of four headings, or with --docked her dock at each face a verb docks her at, of each thing it
goes to that her walk reaches (`dock.docked`), her back to it; what F offers there (`gym.offered`) is
said as F says it (`gym.chose`), or
with --said as a prompt says it, whatever she sees. A run: seconds to the goal borne, held REST_S and
her joints' program over (its rise's to up with --rise), her fall, what she bumped in which phase, the dock (along its normal,
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

from tools.sim.house_watch import FURNITURE, Watch, named

#: Each verb's goal, the rise said after it held REST_S, the seconds it may take and its rise may (from the bed's
#: end she stands 18-22 s after it is said; F's errand test gives it 30); the door's out past its line (`_out`).
GOALS = {'sit_chair': ('seated', 'rise_chair', 35.0, 12.0), 'lie_bed': ('bed_rest', 'rise_bed', 45.0, 30.0),
         'exit_door': ('out', None, 40.0, 0.0)}
#: Her walk's seconds her joint groups' swings are judged against (`house_watch.GROUPS`).
WALK_S = 8.0
#: The groups a spasm is judged on: her trunk's motion is the maneuver's own lean.
SPASM = ('arms', 'neck')
#: Her spots: a grid this fine, this clear of the furniture, four headings; a goal held this long, s.
GRID_M, CLEAR_M, REST_S = 0.6, 0.45, 3.0
HEADINGS = tuple(0.5 * math.pi * k for k in range(4))
#: The chair's dock sweep: off along its edge, her root off it, her heading off it.
DOCK_GRID = ((-0.06, -0.03, 0.0, 0.03, 0.06), (0.14, 0.19), (-0.1, -0.05, 0.0, 0.05, 0.1))


def walked():
    """Each joint group's fastest swing and widest sweep over WALK_S of her walk: a spasm's bound."""
    from machine.process import worker
    w = Watch(0.0, -1.0, 0.0)
    worker.apply(w.s, {'twist': (0.69, 0.0, 0.0)})
    w.go(WALK_S)
    return w.spasm()


def _lain(w):
    """Where she lies on the bed nearest her as it stands: her pelvis off its middle along it toward her feet (m, +
    short of it), across it to her left (m), her body's turn off its length (deg, left +); None with none."""
    from machine.way import furniture
    d, box = w.world.data, furniture.nearest(w.g, 'lie_bed')[6]
    if box is None:
        return None
    _kind, (px, _py, pz), half, turn = box[:4]
    j = 0 if half[0] > half[2] else 2
    ux, uz = turn[0][j], turn[2][j]
    head = d.xpos[w.world.model.body('head').id]
    bx, bz, ox, oz = float(head[0] - d.qpos[0]), float(head[2] - d.qpos[2]), float(d.qpos[0]) - px, float(d.qpos[2]) - pz
    s = math.copysign(1.0, bx * ux + bz * uz)
    return [round(-s * (ox * ux + oz * uz), 3), round(s * (ox * uz - oz * ux), 3),
            round(math.degrees(math.atan2(bx * uz - bz * ux, s * (bx * ux + bz * uz))), 1)]


def _on(w):
    """Whether the bed bears all of her as the world has it: her pelvis, head and feet over its mattress (the room's
    `bed` box, its top's footprint) and the lodge's scenery - its floor - bearing none of her. Judged by her prop
    alone, a lie from the long side was held with her pelvis 0.18 m past the edge and her feet on the floor."""
    import numpy as np
    m, d = w.world.model, w.world.data
    bed = m.geom('bed').id
    turn, half = d.geom_xmat[bed].reshape(3, 3), m.geom_size[bed]
    ends = [d.qpos[0:3]] + [d.xpos[m.body(n).id] for n in ('head', 'left_foot', 'right_foot')]
    over = all(abs(float(a)) <= float(h) for p in ends
               for a, h in zip((turn.T @ (np.asarray(p, float) - d.geom_xpos[bed]))[0::2], half[0::2]))
    # hers by her body's root (a rug under the bed on its own body, on the floor, read as the floor bearing her)
    mine, force, floor = m.body_rootid[m.geom_bodyid] == m.body_rootid[w.world.pelvis], np.zeros(6), 0.0
    for k in range(d.ncon):
        a, b = int(d.contact[k].geom1), int(d.contact[k].geom2)
        if mine[a] != mine[b] and a not in w.furn and b not in w.furn:
            w.world._mj.mj_contactForce(m, d, k, force)
            floor += abs(float(force[0]))
    return bool(over and floor == 0.0)


def _goal(w, goal, rise, limit, rise_s, t0, judged=None):
    """(seconds to `goal` borne, held REST_S and her joints' program over - and where `judged`, where she ended as it
    judges her (`_on`) -, seconds to up within `rise_s` once `rise`
    is said - up held REST_S -, None unsaid) from `t0`. Counted up the moment she stood, 7 chair rises of 58 that
    fell within 3 s were risen; said as her pull up the bed still played, it was cut and she rose from 0.49 m in, not
    1.11, her knees past its end (F's errand test waits for it too)."""
    from machine.law import modal
    from machine.process import sprung, worker
    g = w.g
    def borne(q):
        return sprung.doing(g) == goal and float(g['y'][modal.PROP]) > 0.45
    reached = round(g['t'] - t0, 2) if w.go(limit - (g['t'] - t0), borne) else None
    if reached is not None:
        w.go(REST_S)
        w.go(limit, lambda q: g.get('played') is None)
        w.lain, w.on = _lain(w), (judged(w) if judged is not None else True)
    held = reached is not None and w.s['fallen_at'] is None and sprung.doing(g) == goal and getattr(w, 'on', True)
    up = None
    if held and rise:
        worker.apply(w.s, {'say': rise})
        t2 = g['t']
        if w.go(rise_s, lambda q: sprung.doing(g) in ('stand', 'rest') and float(g['y'][modal.PROP]) < 0.05):
            up = round(g['t'] - t2, 2)
            w.go(REST_S)
    return reached, held, up


def _out(w, limit, t0):
    """(seconds to her pelvis out past the door's line `room.TO` and her errand ended, whether she is out and up REST_S
    on, None: no rise) from `t0` - the door's goal."""
    from machine.world import room
    g = w.g
    out = w.go(limit - (g['t'] - t0), lambda q: float(q.world.data.qpos[2]) > room.TO and not g.get('_errand'))
    reached = round(g['t'] - t0, 2) if out else None
    if out:
        w.go(REST_S)
    return reached, out and w.s['fallen_at'] is None, None


def one(x, z, yaw, verb, said=False, rise=False, trace=False, record=None):
    """Her errand `verb` from (x, z, `yaw`), as F says it or `said` as a prompt does: a line's fields, her rise's
    rows (`house_watch._row`) with them where `trace`; her loop recorded from her say to the run's end where `record` names it
    (`watch.recorded`: recordings/NAME.jsonl)."""
    from types import SimpleNamespace as SN
    from machine.senses import edge
    from machine.process import worker
    from terminal.views import gym, scanner, viewpoint
    w = Watch(x, z, yaw)
    g, w.trace = w.g, [] if trace else None
    st = {'body': SN(send=lambda **cmd: worker.apply(w.s, cmd)), 'scanner': scanner.VisionScanner(),
          'now': worker.told(w.s), 'yaw': 180.0, 'tour': None, **viewpoint.HOME}
    offers = gym.offered(st)
    out = {'x': x, 'z': z, 'yaw': round(yaw, 3), 'verb': verb, 'said': said}
    pick = [k for k, o in enumerate(offers) if o[1] == verb]
    if not said and not pick:
        return dict(out, result='not offered')
    if record:
        worker.apply(w.s, {'record': record})
    if said:
        worker.apply(w.s, {'say': verb})
    else:
        out['key'] = offers[pick[0]][0]
        gym.chose(st, gym.DIGITS.index(offers[pick[0]][0][1:]))
    t0, dock = g['t'], []
    goal, rise_verb, limit, rise_s = GOALS[verb]

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
    lies = verb.startswith('lie')
    reached, held, up = (_out(w, limit, t0) if goal == 'out' else
                         _goal(w, goal, rise_verb if rise else None, limit, rise_s, t0, _on if lies else None))
    fell = None if w.s['fallen_at'] is None else round(w.s['fallen_at'] - t0, 2)
    ok = held and fell is None and (up is not None or not rise)
    if record:
        from tools.sim import scenarios
        scenarios.ended(w.s)
    return dict(out, dock=dock or None, reached=reached, held=held, up=up, fell=fell, spasm=w.spasm(), bumps=w.bumps,
                phases=w.phases[-12:], stops=w.stops, lain=getattr(w, 'lain', None) if lies else None, on=getattr(w, 'on', None) if lies else None,
                result='ok' if ok else ('fell' if fell is not None else 'short'), **({'trace': w.trace} if trace else {}))


def dock(dx, spar, dyaw):
    """The chair's sit laid at its dock `dx` along its edge, her root `spar` off it, `dyaw` turned off
    it; held, risen: a line's fields."""
    from machine.world import room
    from machine.process import worker
    w = Watch(room.CHAIR_AT[0] + dx, room.CHAIR_AT[1] - room.SEAT[2] - spar, math.pi + dyaw)
    worker.apply(w.s, {'say': 'sit_chair'})
    reached, held, up = _goal(w, 'seated', 'rise_chair', 15.0, GOALS['sit_chair'][3], w.g['t'])
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
             for i in range(m.ngeom) if named(m, i, FURNITURE) and float(d.geom_xpos[i][1]) > -0.5]
    xs = [round(-room.WIDE + 0.5 + GRID_M * k, 2) for k in range(int((2.0 * room.WIDE - 1.0) / GRID_M) + 1)]
    zs = [round(room.FROM + 0.5 + GRID_M * k, 2) for k in range(int((room.TO - room.FROM - 1.0) / GRID_M) + 1)]
    return [(x, z) for x in xs for z in zs
            if all(abs(x - bx) > hx + CLEAR_M or abs(z - bz) > hz + CLEAR_M for bx, bz, hx, hz in boxes)]


def docked(verb):
    """(x, z, heading) of her dock on each face `verb` docks her at (`furniture.GOES`), of each thing of the
    lodge it goes to, that her walk reaches (`dock.docked`'s reach): her heels its stunt's standoff off the
    face's edge - her soles' middle `poise.HALF_M` and the standoff out of it as she stands on them, the sit's
    own reading (`furniture.seen`), not where she is laid 3 cm behind them: so laid, the bed's sit rocked her off her
    heels and she perched on its end -, her back to it: a bed's end and its sides, a seat's front."""
    from machine.law import modal, poise
    from machine.texts import maneuver
    from machine.process import sprung
    from machine.way import dock, furniture
    at = (0.0, -1.0)
    w = Watch(at[0], at[1], 0.0)
    sprung.looked(w.g)
    things, standoff = list(w.g['things']), maneuver.standoff(verb, w.g['suite'])
    floor = min(float(f['sole'][1]) for f in w.g['s']['feet'])
    laid = [dock.docked(at, things, floor, verb, box, standoff) for box in things
            if len(box) >= 4 and furniture.GOES[verb][0](box, things)]
    # her soles' middle as the sit reads it ahead of where she was laid, facing +z
    ahead = float(w.g['y'][modal.SOLE].reshape(2, 3)[:, 2].mean() + w.world.data.qpos[2] - w.g['est']['pos'][2]) - at[1]
    out = poise.HALF_M + standoff - ahead
    return [(round(x + out * math.sin(h), 3), round(z + out * math.cos(h), 3), round(h % math.tau, 3))
            for d in laid if d is not None for h, (x, z) in d['reach']]


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
    p.add_argument('--docked', action='store_true', help='from her dock at each face each verb docks her at')
    p.add_argument('--said', action='store_true', help='said as a prompt says it, seen or not')
    p.add_argument('--rise', action='store_true', help='its rise said once it held')
    p.add_argument('--walked', action='store_true', help='her walk\'s fastest swings, here')
    p.add_argument('--every', action='store_true', help='every run\'s line, not only the short, fallen or spasmed')
    p.add_argument('--trace', action='store_true', help='her rise traced (house_watch.TRACED, each TRACE_S)')
    p.add_argument('--record', help='with --one: her loop recorded from her say, recordings/NAME.jsonl (`watch.recorded`)')
    args = p.parse_args(argv)
    flags = ['--said'] * args.said + ['--rise'] * args.rise + ['--trace'] * args.trace
    if args.walked:
        print(json.dumps(walked()), flush=True)
        return 0
    if args.one:
        print(json.dumps(one(float(args.one[0]), float(args.one[1]), float(args.one[2]), args.one[3], args.said, args.rise,
                             args.trace, args.record)), flush=True)
        return 0
    if args.dock:
        print(json.dumps(dock(*args.dock)), flush=True)
        return 0
    runs = ([['--dock', str(a), str(b), str(c)] for a in DOCK_GRID[0] for b in DOCK_GRID[1] for c in DOCK_GRID[2]]
            if args.docks else [['--one', str(x), str(z), str(h), v] + flags for x, z, h, v in (
                [(x, z, h, v) for v in args.verbs or ['sit_chair'] for x, z, h in docked(v)] if args.docked
                else [(x, z, h, v) for x, z in spots() for h in HEADINGS for v in args.verbs or ['sit_chair']])])
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
        if r.get('result') not in ('ok', 'not offered') or spasms[k] or (args.every and r.get('result') == 'ok'):
            print(json.dumps(dict(r, spasms=spasms[k])), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
