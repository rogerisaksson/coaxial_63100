#!/usr/bin/env python3
"""A scenario traced a pass at a time: her law's floor, stance and loads beside the world's floor.

    python tools/sim/trace.py stairs_down 1.8 4.0          # from 1.8 to 4.0 s of it, a row every 20 ms
    python tools/sim/trace.py slope --every 0.05 > ramp.txt
    python tools/sim/trace.py snow --wobble -2e-4           # laid turned, as the bench's wobbles

A row a pass (`--every` s apart): her centre of mass's height and speed along her way and up,
the floor her law reads (`modal.FLOOR`), its plane under each sole (`modal.under`), the world's
floor under each sole's middle (`arrows.surface_at`, the referee's), each sole's middle and lowest
corner, its load, whether it stands and swings, its lead, what her swings are raised, her going,
and when she fell. A scenario is `tools/sim/scenarios.py`'s, its commands at their seconds.
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

COLUMNS = ('t com_y v_along v_up floor plane_l plane_r world_l world_r sole_l sole_r low_l low_r load_l load_r '
           'st_l st_r sw_l sw_r lead_l lead_r raise going fell')


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('scenario')
    parser.add_argument('start', type=float, nargs='?', default=0.0)
    parser.add_argument('stop', type=float, nargs='?', default=1e9)
    parser.add_argument('--every', type=float, default=0.02, help='s between rows')
    parser.add_argument('--wobble', type=float, default=0.0, help='laid turned this rad about up')
    args = parser.parse_args(argv)
    from coaxial.graphics import arrows
    from machine.law import modal
    from machine.world import physics
    from machine.process import worker
    from tools.sim import scenarios
    physics.ROOM, physics.SIGHT = 0.0, 0.0
    scene, plan, seconds, _goal, _aim = scenarios.SCENARIOS[args.scenario]
    s = worker.session()
    scenarios.placed(s, scene)
    world, props = s['g']['world'], None
    c, h = math.cos(args.wobble / 2.0), math.sin(args.wobble / 2.0)
    q = world.data.qpos[3:7].copy()
    world.data.qpos[3:7] = (q[0] * c - q[2] * h, q[1] * c + q[3] * h, q[2] * c + q[0] * h, q[3] * c - q[1] * h)
    world._mj.mj_forward(world.model, world.data)
    plan, k, last = sorted(plan, key=lambda c: c[0]), 0, -1e9
    print(COLUMNS)
    while s['g']['t'] < min(seconds, args.stop) and s['epoch'] == 0:
        while k < len(plan) and s['g']['t'] >= plan[k][0]:
            worker.apply(s, plan[k][1])
            props, k = None, k + 1
        worker.stepped(s)
        g = s['g']
        if g['t'] < args.start or g['t'] - last < args.every - 1e-9:
            continue
        last, y, a, fr = g['t'], g['y'], g['a'], g['fr']
        props = world.props() if props is None else props
        soles = y[modal.SOLE].reshape(2, 3)
        plane = modal.under(y, fr, a[modal.STANCE])[0]
        v = fr[modal.ROT].reshape(3, 3) @ y[modal.VCOM]
        under = [arrows.surface_at(float(p[0]), float(p[2]), props)[0] for p in soles]
        print('%.2f %.3f %.2f %.2f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %4.0f %4.0f %d %d %.2f %.2f %.2f %.2f %.3f %.2f %s' % (
            g['t'], y[modal.COM][1], v[0], y[modal.VCOM][1], fr[modal.FLOOR], plane[0], plane[1], under[0], under[1],
            soles[0][1], soles[1][1], y[modal.LOWEST][0], y[modal.LOWEST][1], g['loads'][0], g['loads'][1],
            a[modal.STANCE][0], a[modal.STANCE][1], a[modal.SWINGS][0], a[modal.SWINGS][1], a[modal.LEAD][0], a[modal.LEAD][1],
            a[modal.RAISE], a[modal.GOING], '-' if s['fallen_at'] is None else '%.2f' % s['fallen_at']))
        if s['fallen_at'] is not None and g['t'] - s['fallen_at'] > 0.3:
            break
    return 0


if __name__ == '__main__':
    sys.exit(main())
