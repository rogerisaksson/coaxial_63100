"""Test 1's doorway (docs/todo/protocol.md): her exit through the lodge's door held ajar, its sill.

    python tools/sim/doorway.py [--gap 0.8] [--seconds 20]

The lodge's door locked open as far as leaves `gap` of her shoulders (`wayfinder.NOMINAL`) between its
free edge and the far post, her exit said from inside; each 10 ms: how far from the doorway's line,
her trunk's twist off her heading (her waist), her gauge, her forearms' and upper arms' least clearance
to the door, the post and the sill, her swing sole over the sill; her fall.
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))


def ajar(world, gap, nominal):
    """The door's hinge locked at the angle leaving `gap` x `nominal` m between its free edge and the far
    post: rad."""
    from machine import room
    width = room.DOOR_X[1] - room.DOOR_X[0]
    angle = math.acos(max(-1.0, min(1.0, 1.0 - (gap * nominal - (width - 2.0 * room.DOOR[0])) / (2.0 * room.DOOR[0]))))
    m, d = world.model, world.data
    j = m.joint('door').id
    m.jnt_range[j] = (angle, angle)
    d.qpos[m.jnt_qposadr[j]] = angle
    world._mj.mj_forward(m, d)
    return angle


def run(gap, seconds, x=0.75, z=2.2):
    from machine import bemf, capsules, room, schema, wayfinder
    from tools.sim.house import Watch
    w = Watch(x, z, 0.0)
    angle = ajar(w.world, gap, wayfinder.NOMINAL)
    from machine import worker
    worker.apply(w.s, {'say': 'exit_door'})
    rows, g = [], w.g
    t0 = g['t']

    def read(_w):
        held = (g.get('schema') or {}).get('room')
        pairs, dist = (held['pairs'], held['d']) if held else ([], [])
        arms = [float(dd) for (c, _b), dd in zip(pairs, dist) if capsules.NAMES[c].startswith(('forearm', 'upper-arm'))]
        q = w.world.data.qpos
        rows.append((round(g['t'] - t0, 2), round(room.TO - float(q[2]), 3),
                     round(math.degrees(float(g['s']['q'][bemf.DRIVEN.index('waist')])), 1),
                     round(min(arms, default=math.inf), 4)))
        return float(q[2]) > room.TO + 0.6
    passed = w.go(seconds, read)
    fell = w.s['fallen_at']
    return {'angle_deg': math.degrees(angle), 'passed': passed, 'fell': fell, 'rows': rows, 'bumps': w.bumps}


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').partition('.')[0])
    parser.add_argument('--gap', type=float, default=0.8)
    parser.add_argument('--seconds', type=float, default=20.0)
    args = parser.parse_args(argv)
    got = run(args.gap, args.seconds)
    print('door at %.1f deg, passed %s, fell %s, bumps %s' % (got['angle_deg'], got['passed'], got['fell'], got['bumps']))
    rows, last = [], -1.0
    for r in got['rows']:
        if r[0] - last >= 0.5:
            rows.append(r)
            last = r[0]
    for r in rows:
        print('  t %5.2f  to the door %+.3f m  waist %+6.1f deg  arms %.4f m' % r)
    return 0


if __name__ == '__main__':
    sys.exit(main())
