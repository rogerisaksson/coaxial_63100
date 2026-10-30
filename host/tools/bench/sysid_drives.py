#!/usr/bin/env python3
"""Each drive on a bus identified on its board: held, swung by its loop, currents and encoder read.

    python tools/bench/sysid_drives.py --port native://?nodes=4&world=humanoid_leg --units 1-4
    python tools/bench/sysid_drives.py --port COM4 --gear 30 --profile outrunner_63100_14p

J, b, f and the gravity on each shaft (`coaxial.control.identify`, `machine.identify.sysid.motor`); on a
native world, the world's own beside.
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))


def _units(text):
    a, _, b = text.partition('-')
    return list(range(int(a), int(b or a) + 1))


def main(argv=None):
    from coaxial import EMULATED, HARDWARE, Coaxial63100
    from coaxial.control import identify
    from coaxial.devices.drive import PROFILES
    p = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    p.add_argument('--port', default='native://?nodes=4&world=humanoid_leg')
    p.add_argument('--units', default='1', help='1 or 1-4')
    p.add_argument('--seconds', type=float, default=16.0)
    p.add_argument('--amps', type=float, default=identify.HOLD_A, help='the hold\'s current')
    p.add_argument('--swing', type=float, default=identify.SWING_RAD, help='the shaft\'s swing, rad')
    p.add_argument('--gear', type=float, default=None, help='motor turns a joint turn (a native world\'s unless given)')
    p.add_argument('--profile', help='the motor profile the record is commissioned as, its drive part')
    args = p.parse_args(argv)
    native = args.port.startswith('native://')
    limb = None
    if native:
        from tools.cores.native import limb_for
        limb = limb_for(args.port)
    print('%-5s %10s %10s %10s %10s %9s %s' % ('unit', 'J kg m^2', 'b N m s', 'f N m', 'gravity', 'left/rms', 'the world\'s J, b, gravity'))
    for unit in _units(args.units):
        rig = Coaxial63100(port=args.port, unit=unit, execution_mode=EMULATED if native else HARDWARE).open()
        try:
            node = limb.world.spec['nodes'][unit - 1] if limb and limb.world else {}
            name = args.profile or node.get('motor')
            model = {}
            if name:
                with open(os.path.join(PROFILES, name + '.json'), encoding='utf-8') as f:
                    profile = json.load(f)
                rig.drive.configure(**profile['drive'])
                model = profile.get('model', {})
            load = node.get('load', {})
            gear = args.gear or load.get('gear', 1.0)
            got = identify.fitted(identify.campaign(rig, args.seconds, args.amps, args.swing), gear)
        finally:
            rig.close()
        world = ''
        if node:
            world = '%.3e %.3e %.3e' % (model.get('j', 0.0) + load.get('inertia', 0.0) / gear ** 2,
                                       model.get('b', 0.0) + load.get('damping', 0.0) / gear ** 2,
                                       load.get('mass', 0.0) * 9.81 * load.get('arm', 0.0) / gear)
        print('%-5d %10.3e %10.3e %10.3e %10.3e %9.2f %s' % (unit, got['J'], got['b'], got['f'], got['gravity'],
                                                           got['residual'] / max(got['rms'], 1e-12), world))
    return 0


if __name__ == '__main__':
    sys.exit(main())
