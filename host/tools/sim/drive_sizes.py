#!/usr/bin/env python3
"""Her drives against what she asks of them: each joint's size, its peak, rms and speed asked.

    python tools/sim/drive_sizes.py              # the rise and 20 s of walk, simulated
    python tools/sim/drive_sizes.py --to 60 RATIO_L=48

A row a joint kind (the worse side), from the squat through the walk as the page runs her: the
peak torque and the walk's rms torque asked, N m, against the size's peak at its board's amps
and the torque it holds for ever at its envelope's throttle point (`machine.heat`, its steady
state); the peak speed asked against the unloaded speed at the pack's volts, deg/s; the rotor
seen through the cycloid against the armature the model carries, kg m^2 (`physics.REFLECTED`).
Then each segment's assemblies' mass against its own, kg: a segment's first joint's assembly
rides its parent, the chain's later ones the segment itself (the hip's roll and pitch on the
thigh, after the yaw), one mounted rides where it is mounted.
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))


def held(joint):
    """The torque a joint's drive holds for ever at its envelope's THROTTLE_AT, N m: its nodes'
    steady state (no hold left to count) bisected on the torque."""
    from machine import drives, heat
    kt, r, scale, _wj, winding_k_w, laminate_k_w = drives.heat(joint)

    def spent(torque):
        amps = torque / kt
        board = (amps * scale) ** 2
        switch = 1.5 * board * heat.RDS_OHM + heat.SWITCHING_W
        laminate = heat.AMBIENT_C + (switch + 1.5 * board * heat.SHUNT_OHM
                                     + heat.HOUSEKEEPING_W) * laminate_k_w
        nodes = (laminate + switch * heat.INTO_K_W, laminate,
                 heat.AMBIENT_C + r * amps * amps * winding_k_w)
        return max((t - heat.AMBIENT_C) / (top - heat.AMBIENT_C)
                   for t, top in zip(nodes, heat.CEILING_C))
    lo, hi = 0.0, drives.peak(joint) * 4.0
    for _ in range(50):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if spent(mid) < heat.THROTTLE_AT else (lo, mid)
    return lo


def asked(to_s, values, cadence=0.85):
    """{joint: (peak N m, the walk's rms N m, peak deg/s)} from the squat, `to_s` seconds."""
    from tools.sim.gait_montecarlo import _set
    _set(values)
    from machine import Machine
    from machine.director import Director
    from machine.figure import JOINTS
    from machine.modes import DYNAMIC
    body = Machine.discover('gynoid', execution_mode=DYNAMIC)
    body.arm()
    director = Director(body, cadence)
    director.begin()
    body.loop.step(0.0)
    world, bus = body.nodes['pelvis'].world, body.loop.bus
    n = len(JOINTS)
    peak, speed, square, count = [0.0] * n, [0.0] * n, [0.0] * n, 0
    while bus['t'] < to_s and director.stage != 'fallen':
        body.loop.write(**director.step(0.001))
        body.loop.step(0.001)
        walking = director.stage in ('walk', 'catch')
        count += walking
        for i in range(n):
            tau = float(world.data.ctrl[i])
            peak[i] = max(peak[i], abs(tau))
            speed[i] = max(speed[i], abs(math.degrees(world.data.qvel[world.vadr[i]])))
            square[i] += tau * tau if walking else 0.0
    body.disarm()
    return {j: (peak[i], math.sqrt(square[i] / max(1, count)), speed[i])
            for i, j in enumerate(JOINTS)}


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--to', type=float, default=28.0, help='seconds simulated from the squat')
    parser.add_argument('knobs', nargs='*', metavar='NAME=V', help='constants moved')
    args = parser.parse_args(argv)
    values = {k: float(v) for k, v in (kv.split('=') for kv in args.knobs)}
    got = asked(args.to, values)
    from machine import drives, physics
    print('%-11s %-4s %-24s | %-15s | %-15s | %-14s | %s' % (
        'kind', 'size', 'where', 'peak asked/has', 'rms asked/holds', 'deg/s asked/has',
        'rotor seen / modelled'))
    kinds = {}
    for joint, row in got.items():
        k = drives.kind(joint)
        was = kinds.get(k)
        kinds[k] = (joint, tuple(max(a, b) for a, b in zip(row, was[1]))) if was else (joint, row)
    for k, (joint, (peak, rms, speed)) in kinds.items():
        name, size = drives.of(joint)
        where = drives.mount(joint)
        print('%-11s %-4s %-24s | %6.1f / %6.1f | %6.1f / %6.1f | %5.0f / %5.0f | %.3f / %.3f' % (
            k, name, 'on its axis' if where is None else 'in the %s' % where[0].split('_')[-1],
            peak, drives.peak(joint), rms, held(joint), speed, drives.speed(joint),
            drives.armature(joint), physics.SERVO[k][3]))
    from machine.figure import JOINTS, MASS_KG, SEGMENTS
    carried = {}
    for seg in SEGMENTS:
        for k, (joint, _axis, _sign) in enumerate(seg[2]):
            where = drives.mount(joint)
            rides = where[0] if where else seg[1] if k == 0 else seg[0]
            carried[rides] = carried.get(rides, 0.0) + drives.of(joint)[1].mass
    print('assemblies in a segment, kg: ' + ', '.join(
        '%s %.2f of %.2f' % (seg[0], carried[seg[0]], seg[5] * MASS_KG)
        for seg in SEGMENTS if seg[0] in carried and not seg[0].startswith('right_')))
    for name, s in drives.SIZES.items():
        print('%s: %.0f A, Kt %.4f N m/A, R %.3f ohm, KV %.0f, 1:%.0f, %.0f x %.0f mm, %.2f kg - %s'
              % (name, s.amps, s.kt_motor, s.r, s.kv,
                 {'L': drives.RATIO_L, 'M': drives.RATIO_M, 'S': drives.RATIO_S}[name],
                 s.diameter * 1e3, s.length * 1e3, s.mass, s.source))
    return 0


if __name__ == '__main__':
    sys.exit(main())
