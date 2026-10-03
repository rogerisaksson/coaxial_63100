#!/usr/bin/env python3
"""Her drives against what she asks of them: each joint's numbers, each 1 at a part's limit.

    python tools/sim/drive_sizes.py              # the rise and 24 s of walk, simulated
    python tools/sim/drive_sizes.py --cached RATIO=36   # the last run's demand, the stacks as set

A row a joint kind (the worse side), from the squat through the walk as the page runs her, the
drive's numbers at MARGIN times what she asked, each 1 where its part binds (docs/findings/drives.md):

    P  power      M max(tau w) / (eta sqrt3/2 V I)       the inverter - under 1 a winding exists
    T  heat       (M T_rms / T_held)^2                    the copper at this ratio (`held`)
    J  inertia    N^2 J_rotor / J_load                    the rotor felt through the ratio
    Q  frame      T J                                     the ratio cancels: the frame alone
    S  box        M T_peak / T_momentary                  the gearbox
    V  volts      w_peak / w_unloaded                     the winding against the pack

Under 1 everywhere the drive is sized; the ratio's window is sqrt(T) N .. N / sqrt(J). Then
the BOM: distinct parts, the drives' kg of hers, each stack's diameter. The demand - the peak
and rms torque, the peak speed and power, her own inertia about the joint (the mass matrix's
diagonal less the armature) - is kept in build/drive_demand.json for `--cached` iterations.
"""
import argparse
import importlib
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools import REPO  # noqa: E402

#: Each number at this times what she asked: heavy lifts off the floor at half again, nothing
#: tuned at its limit (the user, 2026-10-03).
MARGIN = 1.5
CACHE = os.path.join(REPO, 'build', 'drive_demand.json')


def held(joint):
    """(N m, node) a joint's drive holds for ever at its envelope's THROTTLE_AT: its nodes'
    steady state (no hold left to count) bisected on the torque, the node that binds there
    (`heat.NODES`)."""
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
        return max(((t - heat.AMBIENT_C) / (top - heat.AMBIENT_C), name)
                   for t, top, name in zip(nodes, heat.CEILING_C, heat.NODES))
    lo, hi = 0.0, drives.peak(joint) * 4.0
    for _ in range(50):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if spent(mid)[0] < heat.THROTTLE_AT else (lo, mid)
    return lo, spent(hi)[1]


def asked(to_s, values, cadence=0.85):
    """{joint: (peak N m, the walk's rms N m, peak deg/s, peak W motoring, J_load kg m^2)} from
    the squat, `to_s` seconds; the load's inertia the mass matrix's diagonal less the armature,
    meaned every 0.1 s."""
    from tools.sim.gait_montecarlo import _set
    _set(values)
    mujoco = importlib.import_module('mujoco')
    from coaxial.model.blocks import numpy as np
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
    m, d, v = world.model, world.data, np.array(world.vadr)
    full, n = np.zeros((m.nv, m.nv)), len(JOINTS)
    peak, speed, watts, square = np.zeros(n), np.zeros(n), np.zeros(n), np.zeros(n)
    load, count, steps = np.zeros(n), 0, 0
    while bus['t'] < to_s and director.stage != 'fallen':
        body.loop.write(**director.step(0.001))
        body.loop.step(0.001)
        tau, w = np.array(d.ctrl[:n]), np.array(d.qvel[v])
        peak, speed = np.maximum(peak, np.abs(tau)), np.maximum(speed, np.abs(w))
        watts = np.maximum(watts, tau * w)
        if director.stage in ('walk', 'catch'):
            square += tau * tau
            count += 1
        steps += 1
        if steps % 100 == 0:
            mujoco.mj_fullM(m, d, full)
            load += np.diag(full)[v] - m.dof_armature[v]
    body.disarm()
    return {j: (float(peak[i]), math.sqrt(square[i] / max(1, count)), math.degrees(speed[i]),
                float(watts[i]), float(load[i] / max(1, steps // 100))) for i, j in enumerate(JOINTS)}


def numbers(joint, peak, rms, speed, watts, load):
    """(P, T, J, Q, S, V) of a joint's drive at MARGIN times its demand (the module's brief)."""
    from machine import drives
    size = drives.of(joint)[1]
    p = MARGIN * watts / (drives.motors(joint) * size.efficiency * math.sqrt(3.0) / 2.0
                          * drives.PACK_V * size.amps)
    t = (MARGIN * rms / held(joint)[0]) ** 2
    j = drives.armature(joint) / max(load, 1e-9)
    return p, t, j, t * j, MARGIN * peak / drives.shock(joint), speed / drives.speed(joint)


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--to', type=float, default=28.0, help='seconds simulated from the squat')
    parser.add_argument('--cached', action='store_true', help="the last run's demand")
    parser.add_argument('knobs', nargs='*', metavar='NAME=V', help='constants moved')
    args = parser.parse_args(argv)
    values = {k: float(v) for k, v in (kv.split('=') for kv in args.knobs)}
    if args.cached:
        from tools.sim.gait_montecarlo import _set
        _set(values)
        got = json.load(open(CACHE))
    else:
        got = asked(args.to, values)
        os.makedirs(os.path.dirname(CACHE), exist_ok=True)
        json.dump(got, open(CACHE, 'w'), indent=1)
    from machine import drives
    from machine.figure import JOINTS, MASS_KG
    print('%-11s %-3s %-26s | %-11s %-10s %-10s | %5s %5s %5s %5s %5s %5s | %s' % (
        'kind', 'frm', 'stack', 'peak/has', 'rms/holds', 'deg/s/has', 'P', 'T', 'J', 'Q', 'S',
        'V', 'ratio window'))
    kinds = {}
    for joint, row in got.items():
        k = drives.kind(joint)
        was = kinds.get(k)
        kinds[k] = (joint, tuple(max(a, b) for a, b in zip(row, was[1]))) if was else (joint, row)
    for k, (joint, (peak, rms, speed, watts, load)) in kinds.items():
        if drives.passive(joint):
            continue
        name, size = drives.of(joint)
        p, t, j, q, s, v = numbers(joint, peak, rms, speed, watts, load)
        n, (holds, node) = drives.ratio(joint), held(joint)
        print('%-11s %-3s %-26s | %5.0f/%-5.0f %4.0f/%-4.0f%s %4.0f/%-5.0f | %5.2f %5.2f %5.2f '
              '%5.2f %5.2f %5.2f | %.0f..%.0f' % (
                  k, name, size.source.split(',')[0][6:], peak, drives.peak(joint), rms, holds,
                  node[0], speed, drives.speed(joint), p, t, j, q, s, v, math.sqrt(t) * n,
                  n / math.sqrt(j)))
    stacks = [drives.STACKS[drives.kind(j)] for j in JOINTS if not drives.passive(j)]
    kg = sum(drives.mass(j) + drives.of(j)[1].board[0] for j in JOINTS if not drives.passive(j))
    print('BOM: %d frames, %d boxes, %d inverters, %d windings; drives %.1f kg of %.0f; stacks '
          '%s mm round' % (len({s[0] for s in stacks}), len({s[2] for s in stacks}),
                           len({s[3] for s in stacks}), len({s[:2] for s in stacks}), kg, MASS_KG,
                           '/'.join('%.0f' % (1e3 * d) for d in sorted(
                               {drives.SIZES[s].diameter for s in drives.STACKS}, reverse=True))))
    return 0


if __name__ == '__main__':
    sys.exit(main())
