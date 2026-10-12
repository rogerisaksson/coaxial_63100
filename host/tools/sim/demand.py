#!/usr/bin/env python3
"""Her drives against what her loop asks: each one's amps, speed and heat beside its parts' limits.

Her walk, her hop and a shove on her loop as the page runs it, a relay job each.

    python tools/sim/demand.py                 # every scene, a relay job each: a row a drive
    python tools/sim/demand.py --scene walk    # one, here: its row as JSON

A drive's amps are its joint's torque over its N m an amp (`World.motor`, `drives.kt`); a
parallel pair's two give the first joint's share plus and less its roll's (`physics.paired`),
and turn at its speed plus and less its roll's the same way. Its heat is `machine.heat`'s nodes
on those amps, HASTE times the clock. Its parts' limits: its board's amps (`Size.amps`), the
amps it holds for ever (`heat.held`), its gearbox's momentary torque (`Size.shock`), its motor's
speed with no load at the pack's volts (`drives.speed`), a node's span to its ceiling. A share
past PAST of any is over. A shove she falls of is not a scene: a fall's demand is the fall's.
"""
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

#: What a part may be asked of its limit.
PAST = 2.0 / 3.0
#: A scene: (the gym's entries, each at its second - 'walk' her walk from rest -, a shove's N,
#: its seconds); a shove comes at SHOVE_AT s from each of WAYS, deg round her.
SCENES = {'walk': ('walk', 0.0, 51.0), 'hop': ((('hop', 1.0),), 0.0, 6.0),
          'nudge': ((), 38.0, 6.0), 'push': ((), 80.0, 6.0)}
SHOVE_AT, WAYS = 1.0, tuple(range(0, 360, 45))
#: The shares a row holds, as printed: of its board's amps at its peak, of what it holds by its
#: rms, of its gearbox's momentary torque, of its no-load speed, of a node's span.
SHARES = ('peak', 'rms', 'box', 'speed', 'heat')


def laid(world):
    """The drives of `world`: (names, first, second, sign) - a joint's own drive, or a parallel
    pair's two: the first joint's share plus (sign 1) and less (-1) its roll's."""
    import numpy as np
    from machine.figure import JOINTS
    paired = {int(i) for pair in world.pairs for i in pair}
    rows = [(JOINTS[i], i, i, 0.0) for i in np.flatnonzero(world.driven) if int(i) not in paired]
    for p, r in world.pairs:
        rows += [(JOINTS[p] + ' +', int(p), int(r), 1.0), (JOINTS[p] + ' -', int(p), int(r), -1.0)]
    names, first, second, sign = zip(*rows)
    return names, np.array(first), np.array(second), np.array(sign)


def ran(scene, way=0):
    """{scene, way, stood, t, names, amps (peak), peak, rms, box, speed, heat - a share of its
    part's limit a drive -, often (the share of the passes its amps are past PAST of its
    board's), celsius}: `scene` on her loop as the page runs it, a shove from `way` deg."""
    import numpy as np
    from machine import drives, heat, sprung
    from machine.figure import JOINTS
    asked, shove, seconds = SCENES[scene]
    g = sprung.laid(sprung.STACK_EVERY)
    by = {entry[0]: entry for entry in sprung.entries()}
    for label, at in (sprung.WALK if asked == 'walk' else asked):
        sprung.said(g, *by[label], at=at)
    world = g['world']
    names, first, second, sign = laid(world)
    joints = [JOINTS[i] for i in first]
    sizes = [drives.of(j)[1] for j in joints]
    kt = world.kt
    free = np.radians([drives.speed(j) if world.driven[i] else 1.0 for i, j in enumerate(JOINTS)])
    board = np.array([s.amps for s in sizes])
    holds = np.array([heat.held(j)[0] / drives.kt(j) for j in joints])
    box = np.array([s.kt_motor * drives.RATIO * s.efficiency / s.shock for s in sizes])
    warm, ones = heat.Heat([drives.heat(j) for j in joints]), [1.0] * len(names)
    warm.envelope = False
    peak, square, fast, spent, often = (np.zeros(len(names)) for _ in range(5))
    n, shoved = 0, shove == 0.0
    while g['t'] < seconds and sprung.tilt(g) < sprung.DOWN_DEG and world.data.qpos[1] > sprung.DOWN_M:
        if not shoved and g['t'] >= SHOVE_AT:
            world.push((shove * math.cos(math.radians(way)), 0.0, shove * math.sin(math.radians(way))),
                       sprung.SHOVE_S)
            shoved = True
        sprung.passed(g)
        share, turns = world.motor / kt * world.driven, world.data.qvel[world.vadr] / free
        amps = share[first] + sign * share[second]
        peak, square = np.maximum(peak, np.abs(amps)), square + amps * amps
        often += np.abs(amps) > PAST * board
        fast = np.maximum(fast, np.abs(turns[first] + sign * turns[second]))
        for k, a in enumerate(amps):
            warm.load(k, a * warm.kt[k])
        n += 1
        if n % 10 == 0:
            warm.step(0.01, ones, ones)
            spent = np.maximum(spent, warm.spent)
    rms = np.sqrt(square / max(n, 1))
    return {'scene': scene, 'way': way, 'stood': bool(g['t'] >= seconds - 1e-6), 't': round(g['t'], 3),
            'names': names, 'amps': np.round(peak, 1).tolist(), 'peak': (peak / board).tolist(),
            'rms': (rms / holds).tolist(), 'box': (peak * box).tolist(), 'speed': fast.tolist(),
            'heat': spent.tolist(), 'often': (often / max(n, 1)).tolist(),
            'celsius': [round(max(t), 1) for t in warm.t]}


def kinds(rows):
    """{kind: {share: its worst over `rows` and the sides}}, the kinds by their worst share."""
    worst = {}
    for row in rows:
        for k, name in enumerate(row['names']):
            kind = name.split('_', 1)[1] if name.startswith(('left_', 'right_')) else name
            mine = worst.setdefault(kind, dict.fromkeys(SHARES + ('amps', 'often', 'celsius'), 0.0))
            for share in mine:
                mine[share] = max(mine[share], row[share][k])
    return dict(sorted(worst.items(), key=lambda item: -max(item[1][s] for s in SHARES)))


def said(scene, rows):
    """A scene's table over the runs she stood, and its shares past PAST: [(kind, share,
    value)]."""
    stood = [row for row in rows if row['stood']]
    print('%s: stood %d of %d, %.0f s' % (scene, len(stood), len(rows), max(row['t'] for row in rows)))
    print('  %-12s %6s %9s %8s %9s %8s %6s %5s %6s' % (
        'drive', 'A', 'of board', 'past, %', 'rms/held', 'gearbox', 'speed', 'heat', 'C'))
    over = []
    for kind, mine in kinds(stood).items():
        print('  %-12s %6.1f %9.2f %8.2f %9.2f %8.2f %6.2f %5.2f %6.1f' % (
            kind, mine['amps'], mine['peak'], 100.0 * mine['often'], mine['rms'], mine['box'],
            mine['speed'], mine['heat'], mine['celsius']))
        over += [(kind, share, mine[share]) for share in SHARES if mine[share] > PAST]
    return over


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--scene', choices=sorted(SCENES), help='one scene, here: its row as JSON')
    parser.add_argument('--way', type=int, default=0, help="a shove's way, deg")
    args = parser.parse_args(argv)
    if args.scene:
        print(json.dumps(ran(args.scene, args.way)))
        return 0
    from tools.dev import focus
    jobs = [focus.Job('%s@%d' % (scene, way), [sys.executable, '-X', 'utf8', os.path.abspath(__file__),
                                              '--scene', scene, '--way', str(way)], 1.5, 1800.0)
            for scene, (_asked, shove, _s) in SCENES.items() for way in (WAYS if shove else (0,))]
    rows = {}
    for job, text, code, _seconds in focus.relay(jobs):
        line = next((ln for ln in reversed(text.splitlines()) if ln.startswith('{')), None)
        if code != 0 or line is None:
            print('%s failed: %s' % (job.name, text.strip().splitlines()[-1:] or code))
            return 1
        row = json.loads(line)
        rows.setdefault(row['scene'], []).append(row)
    over = [(scene,) + item for scene in SCENES for item in said(scene, rows[scene])]
    print("past %.2f of a part's limit:" % PAST)
    for scene in SCENES:
        mine = ['%s %s %.2f' % item[1:] for item in over if item[0] == scene]
        print('  %s: %s' % (scene, ', '.join(mine) or 'none'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
