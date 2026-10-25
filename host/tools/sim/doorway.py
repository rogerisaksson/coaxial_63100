"""Test 1's doorway (docs/todo/protocol.md): her exit through the lodge's door held ajar, its sill.

    python tools/sim/doorway.py [--gap 0.8] [--seconds 20]

The lodge's door locked open as far as leaves `gap` of her shoulders (`wayfinder.NOMINAL`) between its
free edge and the far post, her exit said from inside; each 10 ms: how far from the doorway's line,
her trunk's twist off her heading (her waist), her forearms' and upper arms' least clearance to the
door; her fall.
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

#: The points along a capsule's axis its clearance is read at.
STEPS = 50


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


def clearance(caps, k, box):
    """m capsule `k` of `caps` (`capsules.laid`) clears `box` (centre, half sizes, rotation): its axis
    sampled STEPS times, each point's distance to the box less the radius there, the least."""
    import numpy as np
    centre, half, R = (np.asarray(v, float) for v in box)
    s = np.linspace(0.0, 1.0, STEPS)[:, None]
    p = caps['p0'][k] + s * (caps['p1'][k] - caps['p0'][k])
    local = (p - centre) @ R
    out = local - np.clip(local, -half, half)
    return float((np.sqrt((out * out).sum(1)) - (caps['r0'][k] + s[:, 0] * (caps['r1'][k] - caps['r0'][k]))).min())


def run(gap, seconds, x=0.75, z=2.2):
    from machine import bemf, capsules, room, wayfinder
    from tools.sim.house import Watch
    w = Watch(x, z, 0.0)
    angle = ajar(w.world, gap, wayfinder.NOMINAL)
    from machine import worker
    worker.apply(w.s, {'say': 'exit_door'})
    rows, g = [], w.g
    t0 = g['t']
    door = next(p[1:4] for p in w.world.props() if p[0] == 'door')

    def read(_w):
        own = g['schema_body']
        caps = capsules.laid(capsules.posed(own, own.d, g['s']), g['schema_at'])
        arms = [clearance(caps, k, door) for k, n in enumerate(capsules.NAMES) if n.startswith(('forearm', 'upper-arm'))]
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
