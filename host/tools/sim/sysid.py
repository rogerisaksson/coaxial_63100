"""Her identification campaign on the stand, run and tabled; its corrections on her build written.

Each drive's friction and armature, the base parameters trusted, the corrections on her build.

    python tools/sim/sysid.py [--seconds 24] [--payload 0.5] [--boxed] [--write machine/robots/X.identified.toml]
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from machine import mjcf, physics

mjcf.STAND = 1.0


def main(argv=None):
    from machine import identified, sysid, wbc
    p = argparse.ArgumentParser(description='Her identification campaign on the stand, run and tabled.')
    p.add_argument('--seconds', type=float, default=24.0, help='the campaign, s')
    p.add_argument('--payload', type=float, default=0.0, help='kg in her left hand, the world\'s')
    p.add_argument('--every', type=int, default=10, help='every how many steps a row is read')
    p.add_argument('--seed', type=int, default=1)
    p.add_argument('--write', help='the corrections on her build, TOML (`identified.IDENTIFIED` for her law)')
    p.add_argument('--boxed', action='store_true', help='the world with its gearboxes: each box read too')
    args = p.parse_args(argv)
    physics.BOXED = float(args.boxed)
    body = wbc.Body(identified=False)
    lay = sysid.laid(body)
    world = physics.World()
    if args.payload:
        world.model.body_mass[world.model.body('left_hand').id] += args.payload
        world._mj.mj_setConst(world.model, world.data)
    world.reset(sysid.pose(), where=(0.0, mjcf.STAND_M, 0.0))
    ys, taus, touched = sysid.excite(world, sysid.setpoints(world, args.seconds, np.random.default_rng(args.seed)))
    smooth = sysid.smoothed(ys, taus, physics.STEP_S) if args.boxed else (ys, taus)
    Y, tau = sysid.rows(lay, body, smooth[0], smooth[1], physics.STEP_S, args.every)
    base = sysid.base(lay, Y)
    got = sysid.identify(base, Y, tau)
    nominal = sysid.truth(lay, body.m)
    was, now = sysid.projected(base, nominal), sysid.projected(base, sysid.truth(lay, world.model))
    print('%d s on the stand, touched %d steps; %d base parameters of %d columns, condition %.1e, '
          '%d trusted' % (args.seconds, touched, len(base['ind']), len(lay['names']), got['condition'],
                          got['trusted'].sum()))
    print('\n%-20s %10s %10s %10s %8s' % ('drive', 'J kg m^2', 'b N m s', 'f N m', 'rms N m'))
    drives = [n[:-2] for n in lay['names'][lay['drive']::3]]
    for i, name in enumerate(drives):
        row = []
        for q in sysid.FRICTION:
            k = got['names'].index('%s.%s' % (name, q))
            row.append('%7.4f%s' % (got['theta'][k], '  ' if got['trusted'][k] else ' ?'))
        print('%-20s %10s %10s %10s %8.3f' % (name, row[0], row[1], row[2], got['residual'][i]))
    if args.boxed:
        from machine.figure import JOINTS
        armature = np.array([got['theta'][got['names'].index(n + '.J')] for n in drives])
        read = sysid.boxes(body, ys, taus, physics.STEP_S, armature)
        print('\n%-20s %9s %9s %8s %8s' % ('gearbox', 'k N m/rad', 'world', 'play rad', 'world'))
        for i, name in enumerate(drives):
            k = JOINTS.index(name)
            print('%-20s %9.0f %9.0f %8.4f %8.4f' % (name, read[i, 0], world.boxes.k[k], read[i, 1], world.boxes.gap[k]))
    print('\n%-24s %10s %10s %10s %8s' % ('base parameter', 'build', 'world', 'read', '+- %'))
    for k in np.argsort(got['uncertainty']):
        if got['trusted'][k] and not got['names'][k].endswith(('.J', '.b', '.f')):
            print('%-24s %10.4f %10.4f %10.4f %8.1f' % (got['names'][k], was[k], now[k], got['theta'][k],
                                                       100 * got['uncertainty'][k]))
    if args.write:
        deltas = identified.save(args.write, identified.correction(lay, body, Y, tau), got)
        applied = identified.correct(wbc.Body(identified=False), deltas)
        print('\n%d corrections to %s; laid over her law\'s model: %d links and drives'
              % (len(deltas), args.write, len(applied)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
