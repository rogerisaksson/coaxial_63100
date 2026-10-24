"""Her loop's return ratio at the kernel's probe frequencies, a joint's torque probed standing.

    python tools/sim/margin.py [joint ..] [--amps 0.5] [--boxed] [--walk]

Her standing on her loop - walking with --walk, told her walk after her settling -, the probe's sine
in one drive's torque fed forward at its board (u = c + d), each frequency in its turn after a
second's settling: L = -C / U at each of the probe's own response (the same passes with the probe off
taken out), its gain, its phase, its margin, and the text's rollback of K_d and gamma; --boxed with
her gearboxes' wind-up and resonances (`physics.BOXED`).
"""
import argparse
import sys

#: Her settling before the probe, s; its sine's amplitude by default, N m.
SETTLE_S, AMPS = 1.0, 0.5


def _passes(joint, amps, walk):
    """[(t, dt, u, c)] each law pass of `joint`'s loop from SETTLE_S on, the probe's sine at `amps` N m
    fed forward at its board - her walk told after the settling with `walk` -: the torque her drive was
    given and it less the sine."""
    import numpy as np
    from machine import edge, margin, sprung
    from machine.figure import JOINTS
    g = sprung.laid(sprung.STACK_EVERY)
    j, kept = JOINTS.index(joint), edge.drive
    hot = np.zeros(len(g['joints']))
    hot[list(g['joints']).index(j)] = 1.0
    given, out = [0.0], []

    def drive(world, joints, target=None, feed=None, kp=None, limit=None):
        on = feed is not None and joints is g['joints'] and g['t'] >= SETTLE_S
        given[0] = margin.injected(margin.laid(), g['t'] - SETTLE_S, amps) * on
        return kept(world, joints, target=target, feed=None if feed is None else feed + given[0] * hot, kp=kp, limit=limit)

    edge.drive = drive
    if walk:
        sprung.walked(g)
    try:
        while g['t'] < SETTLE_S + len(margin.HZ) * margin.DWELL_S + 0.01:
            t = g['t']
            sprung.passed(g)
            u = float(g['world'].motor[j])
            out.append((max(0.0, t - SETTLE_S), g['t'] - t, u, u - given[0]))
    finally:
        edge.drive = kept
    return out


def probed(joint, amps=AMPS, walk=False):
    """[{hz, gain_db, phase_deg, margin_deg}] of `joint`'s loop standing - walking with `walk` -, each
    probe frequency its turn: the lock-in on the probe's own response, each pass's u and c less the
    same pass's with the probe off (with it off u = c and L reads -1, 0 dB at 180 deg, whatever her
    torque: walking, her gait's own swamped a 1 N m probe at 30 Hz). Her session is deterministic: a
    pass count or a time differing between the two is refused."""
    from machine import margin
    from machine.errors import MachineError
    on, off = _passes(joint, amps, walk), _passes(joint, 0.0, walk)
    if len(on) != len(off) or any(abs(a[0] - b[0]) > 1e-9 for a, b in zip(on, off)):
        raise MachineError('refused: her session ran %d passes with the probe off, %d with it on' % (len(off), len(on)))
    probe = margin.laid()
    for (t, dt, u, c), (_t, _dt, u0, c0) in zip(on, off):
        margin.step(probe, u - u0, c - c0, t, dt)
    return margin.read(probe)


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').partition('.')[0])
    parser.add_argument('joints', nargs='*', default=['left_ankle', 'left_knee', 'left_hip', 'left_shoulder', 'spine'])
    parser.add_argument('--amps', type=float, default=AMPS)
    parser.add_argument('--boxed', action='store_true', help="her gearboxes' wind-up (`physics.BOXED`)")
    parser.add_argument('--walk', action='store_true', help='her walk told after her settling')
    args = parser.parse_args(argv)
    from machine import margin, physics
    physics.BOXED = float(args.boxed)
    for joint in args.joints:
        got = probed(joint, args.amps, args.walk)
        print('%-14s %s  rollback K_d %.2f gamma %.2f' % (joint, '  '.join(
            '%2.0f Hz %+6.1f dB %+7.1f deg (margin %+6.1f)' % (r['hz'], r['gain_db'], r['phase_deg'], r['margin_deg'])
            for r in got), *margin.derated(got)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
