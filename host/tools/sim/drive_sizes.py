#!/usr/bin/env python3
"""Her drives against what she asks of them: each joint's numbers, each 1 at a part's limit.

    python tools/sim/drive_sizes.py              # the rise and 24 s of walk, simulated
    python tools/sim/drive_sizes.py --cached RATIO=36   # the last run's demand, the stacks as set
    python tools/sim/drive_sizes.py --cached --run      # with a run's demand folded in (RUN)

A row a joint kind (the worse side), from the squat through the walk as the page runs her, the
drive's numbers at MARGIN times what she asked, each 1 where its part binds (docs/findings/drives.md):

    A  amps       M T_peak / T_at_I                       the inverter's amps at stall
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

from machine import events  # noqa: E402
from tools import REPO  # noqa: E402

#: Each number at this times what she asked: heavy lifts off the floor at half again, nothing
#: tuned at its limit (the user, 2026-10-03).
MARGIN = 1.5
CACHE = os.path.join(REPO, 'build', 'drive_demand.json')
#: The demand's scenes, each from the squat: the walk alone, then a slip, a nudge, a hole and
#: the page's shove laid at SCENE_S - her parries and falls ask the peaks (one walk's knee asked
#: 1231 deg/s with a catch in it, 561 without, 2026-10-03) - and the gym's, standing on each rig
#: of `events.STANDING` after 'stand:' (the user, 2026-10-04).
SCENES = (None, 'slip', 'nudge', 'hole', 'shove') + tuple('stand:' + e for e in events.STANDING)
SCENE_S = 8.0
#: A scene's margin where it is not MARGIN: the parries' and the falls' 1.2 (the user's call,
#: taken 2026-10-04: a catch's 171 N m and 1259 deg/s at the hip bound the hips and knees at
#: 1.5 - 1.26 of A's amps, 1.19 of its volts - where the walk, the gym and the run ask less).
#: Each scene's peaks, speeds and watts go into the cache at its margin over MARGIN.
MARGINS = {'slip': 1.2, 'nudge': 1.2, 'hole': 1.2, 'shove': 1.2}
#: A run's demand a kind at the 2 m/s the running item asks (peak N m, peak deg/s, peak W) a kg,
#: the rms RUN_RMS of the peak; folded in with `--run`. Estimated from the literature's 3-3.5 m/s
#: peaks (Novacheck 1998, Schache 2011, Dorn 2012: hip 2.7, knee 3.0, ankle 3.6 N m; 450, 650,
#: 850 deg/s; 7, 10, 12 W): the hip's moment near-linear in speed, the knee's and the ankle's
#: nearly flat, the speeds 0.75 of them. At those peaks and 1.5x, amps x volts outran the pack's
#: volts x the inverter's amps - 1.26 x 1.19 at 48 V, 1.07 x 1.07 at 63 V - with no ratio or KV
#: to move it (2026-10-04).
RUN = {'hip': (1.6, 330.0, 4.0), 'knee': (2.6, 500.0, 6.0), 'ankle': (3.0, 650.0, 8.0),
       'hip_roll': (1.4, 220.0, 2.0), 'hip_yaw': (0.5, 220.0, 0.7), 'ankle_roll': (0.7, 220.0, 1.0),
       'spine': (1.2, 150.0, 1.5), 'spine_roll': (1.0, 150.0, 1.0), 'waist': (0.4, 150.0, 0.7)}
RUN_RMS = 0.4


def running(got):
    """`got` {joint: (peak, rms, speed, watts, load)} with a run's demand (RUN) folded in: each
    the larger, the load hers."""
    from machine import drives
    from machine.figure import MASS_KG
    out = {}
    for joint, (peak, rms, speed, watts, load) in got.items():
        nm_kg, deg_s, w_kg = RUN.get(drives.kind(joint), (0.0, 0.0, 0.0))
        out[joint] = (max(peak, nm_kg * MASS_KG), max(rms, RUN_RMS * nm_kg * MASS_KG),
                      max(speed, deg_s), max(watts, w_kg * MASS_KG), load)
    return out


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


def asked(to_s, values, scene=None, cadence=0.85):
    """{joint: (peak N m, the walk's rms N m, peak deg/s, peak W motoring, J_load kg m^2)} from
    the squat, `to_s` seconds, a `scene` laid at SCENE_S - a floor event (`machine.events`),
    'shove', the page's P, or 'stand:' a rig she stands on (`events.STANDING`) -; the load's
    inertia the mass matrix's diagonal less the armature, meaned every 0.1 s."""
    from tools.sim import knobs
    knobs.set_(values)
    mujoco = importlib.import_module('mujoco')
    from coaxial.model.blocks import numpy as np
    from machine import Machine, drives, events, linkage
    from machine.director import Director
    from machine.figure import JOINTS
    from machine.modes import DYNAMIC
    body = Machine.discover('gynoid', execution_mode=DYNAMIC)
    body.arm()
    world = body.nodes['pelvis'].world
    rig = scene[6:] if scene and scene.startswith('stand:') else None
    director = Director(body, cadence, stand_s=math.inf if rig else 0.0)
    if rig:
        up, stagger = events.rigged(rig)
        director.begin(0.002, up=up, stagger=stagger)
        events.rig(rig, director, world)
    else:
        director.begin()
    body.loop.step(0.0)
    bus = body.loop.bus
    laid, was = scene is None, 0.0
    m, d, v = world.model, world.data, np.array(world.vadr)
    full, n = np.zeros((m.nv, m.nv)), len(JOINTS)
    peak, speed, watts, square = np.zeros(n), np.zeros(n), np.zeros(n), np.zeros(n)
    load, count, steps = np.zeros(n), 0, 0
    # A joint on a rod or a four-bar: its torque and speed as its rest ratio sees them, the
    # lever's change along the stroke taken out (`physics.World`'s s).
    stroked = [(i, j) for i, j in enumerate(JOINTS)
               if drives.kind(j) in linkage.RODS or drives.kind(j) in linkage.PLANAR]
    while bus['t'] < to_s and (not laid or director.stage != 'fallen'):
        if not laid and bus['t'] >= SCENE_S:
            if rig:
                events.befall(rig, director, world)
                laid = True
            elif scene == 'shove':
                world.push((events.SHOVES['shove'], 0.0, 0.0), events.SHOVE_S)
                laid = True
            elif was < events.at(scene) <= director.walker.phase:
                events.lay(scene, director, world)
                laid = True
            was = director.walker.phase
        body.loop.write(**director.step(0.001))
        body.loop.step(0.001)
        tau, w = np.array(d.ctrl[:n]), np.array(d.qvel[v])
        for i, j in stroked:
            s = drives.ratio(j, math.degrees(d.qpos[world.qadr[i]])) / drives.ratio(j)
            tau[i], w[i] = tau[i] / s, w[i] * s
        peak, speed = np.maximum(peak, np.abs(tau)), np.maximum(speed, np.abs(w))
        watts = np.maximum(watts, tau * w)
        # The walk's rms alone: a stand of milliseconds before a fall made the rms the peak,
        # T 3.75 (2026-10-04).
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
    """(A, P, T, J, Q, S, V) of a joint's drive at MARGIN times its demand (the module's
    brief)."""
    from machine import drives
    size = drives.of(joint)[1]
    p = MARGIN * watts / (drives.motors(joint) * size.efficiency * math.sqrt(3.0) / 2.0
                          * drives.PACK_V * size.amps)
    t = (MARGIN * rms / held(joint)[0]) ** 2
    j = drives.armature(joint) / max(load, 1e-9)
    return (MARGIN * peak / drives.peak(joint), p, t, j, t * j,
            MARGIN * peak / drives.shock(joint), speed / drives.speed(joint))


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--to', type=float, default=28.0, help='seconds simulated from the squat')
    parser.add_argument('--cached', action='store_true', help="the last run's demand")
    parser.add_argument('--run', action='store_true', help="a run's demand folded in (RUN)")
    parser.add_argument('knobs', nargs='*', metavar='NAME=V', help='constants moved')
    args = parser.parse_args(argv)
    values = {k: float(v) for k, v in (kv.split('=') for kv in args.knobs)}
    if args.cached:
        from tools.sim import knobs
        knobs.set_(values)
        got = json.load(open(CACHE))
        got = running(got) if args.run else got
    else:
        # The scenes' demands folded: the peaks, the rms and the power their most, the load
        # its mean.
        runs = [{j: (p * k, rms, s * k, w * k, load) for j, (p, rms, s, w, load) in
                 asked(args.to, values, scene).items()}
                for scene in SCENES for k in (MARGINS.get(scene or '', MARGIN) / MARGIN,)]
        got = {j: [max(r[j][i] for r in runs) if i != 4 else sum(r[j][4] for r in runs) / len(runs)
                   for i in range(5)] for j in runs[0]}
        os.makedirs(os.path.dirname(CACHE), exist_ok=True)
        json.dump(got, open(CACHE, 'w'), indent=1)
    from machine import drives
    from machine.figure import JOINTS, MASS_KG
    print('%-11s %-3s %-26s | %-11s %-10s %-10s | %5s %5s %5s %5s %5s %5s %5s | %s' % (
        'kind', 'frm', 'stack', 'peak/has', 'rms/holds', 'deg/s/has', 'A', 'P', 'T', 'J', 'Q',
        'S', 'V', 'ratio window'))
    kinds = {}
    for joint, row in got.items():
        k = drives.kind(joint)
        was = kinds.get(k)
        kinds[k] = (joint, tuple(max(a, b) for a, b in zip(row, was[1]))) if was else (joint, row)
    for k, (joint, (peak, rms, speed, watts, load)) in kinds.items():
        if drives.passive(joint):
            continue
        name, size = drives.of(joint)
        a, p, t, j, q, s, v = numbers(joint, peak, rms, speed, watts, load)
        n, (holds, node) = drives.ratio(joint), held(joint)
        print('%-11s %-3s %-26s | %5.0f/%-5.0f %4.0f/%-4.0f%s %4.0f/%-5.0f | %5.2f %5.2f %5.2f '
              '%5.2f %5.2f %5.2f %5.2f | %.0f..%.0f' % (
                  k, name, size.source.split(',')[0][6:], peak, drives.peak(joint), rms, holds,
                  node[0], speed, drives.speed(joint), a, p, t, j, q, s, v, math.sqrt(t) * n,
                  n / math.sqrt(j)))
    stacks = [drives.STACKS[drives.kind(j)] for j in JOINTS if not drives.passive(j)]
    kg = sum(drives.mass(j) + drives.of(j)[1].board[0] for j in JOINTS if not drives.passive(j))
    print('BOM: %d drives - %d frames (a winding each), %d boxes, %d inverters; %.1f kg of %.0f; '
          'stacks %s mm round' % (len(stacks), len({s[0] for s in stacks}),
                                  len({s[1] for s in stacks}), len({s[2] for s in stacks}), kg,
                                  MASS_KG, '/'.join('%.0f' % (1e3 * d) for d in sorted(
                                      {drives.size(s).diameter for s in drives.STACKS},
                                      reverse=True))))
    return 0


if __name__ == '__main__':
    sys.exit(main())
