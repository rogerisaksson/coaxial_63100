#!/usr/bin/env python3
"""Her run, simulated and measured: a row a step, then her drives against what it asked.

    python tools/sim/run.py                      # 2 m/s, 6 s, from a flight
    python tools/sim/run.py --speed 2.5 --to 10 runner.CONTACT_S=0.2

A row a landing: when, the foot, her speed and sink as it came down, the pelvis's height, the
ball ahead of the hip, the stance's and the flight's seconds, her trunk's pitch. Then each leg
joint: its peak and rms torque, its peak speed and power, and its drive's numbers at MARGIN
(`drive_sizes.numbers`: A amps, P power, T heat, S box, V volts - 1 where the part binds).
"""
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

#: Her run's form on a flat floor, (measure, least, most), held by test_gynoid_run.py: her speed's
#: share off the one asked, her shortest flight, s, her trunk's tip, deg, a sole's load, N. A
#: minute at 1.25-2.0 m/s asked she ran 1.27-1.94, her flights 43 ms at least, tipped 9.0 deg,
#: a sole 888 N (2026-10-05).
FORM = (('off', None, 0.10), ('flight', 0.02, None), ('pitch', None, 12.0), ('load', None, 1000.0))

#: Fallen: the pelvis under FELL_M or her trunk tipped past FELL_DEG.
FELL_M, FELL_DEG = 0.55, 55.0
#: The demand is taken from SETTLE_S on.
SETTLE_S = 1.0
#: The speed asked goes to another at RAMP m/s^2.
RAMP = 0.25
KINDS = ('hip_yaw', 'hip_roll', 'hip', 'knee', 'ankle', 'ankle_roll', 'spine', 'spine_roll',
         'waist', 'shoulder', 'elbow')


def ran(speed, to_s, values, swap=None, swap_s=4.0, to_speed=None):
    """({measure: value}, [step rows], {joint: (peak, rms, deg/s, W, load)}) of `to_s` s of her
    run at `speed` from a flight - `swap` {name: value} set under way at `swap_s`, the speed
    asked going to `to_speed` from then, and all of it measured from SETTLE_S after that."""
    import importlib
    from tools.sim import knobs
    knobs.set_(dict(values))
    since = SETTLE_S if swap is None and to_speed is None else swap_s + SETTLE_S + (
        0.0 if to_speed is None else abs(to_speed - speed) / RAMP)
    mujoco = importlib.import_module('mujoco')
    from coaxial.model.blocks import numpy as np
    from machine import Machine, drives, linkage
    from machine.figure import JOINTS
    from machine.modes import DYNAMIC
    from machine.runner import Runner
    body = Machine.discover('gynoid', execution_mode=DYNAMIC)
    body.arm()
    world = body.nodes['pelvis'].world
    runner = Runner(body, speed)
    runner.start()
    body.loop.step(0.0)
    bus = body.loop.bus
    m, d, v = world.model, world.data, np.array(world.vadr)
    n = len(JOINTS)
    full = np.zeros((m.nv, m.nv))
    peak, top, watts, square = np.zeros(n), np.zeros(n), np.zeros(n), np.zeros(n)
    load, count, passes, work, loads = np.zeros(n), 0, 0, 0.0, 0.0
    stroked = [(i, j) for i, j in enumerate(JOINTS)
               if drives.kind(j) in linkage.RODS or drives.kind(j) in linkage.PLANAR]
    fell, z0, t0, pitch = None, None, None, []
    while bus['t'] < to_s:
        if swap is not None and bus['t'] >= swap_s:
            knobs.put(swap)
            swap = None
        if to_speed is not None and bus['t'] >= swap_s:
            runner.speed += max(-RAMP * 0.001, min(RAMP * 0.001, to_speed - runner.speed))
        body.loop.write(**runner.step(0.001))
        body.loop.step(0.001)
        up = 1.0 - 2.0 * (bus['pelvis.pose.qx'] ** 2 + bus['pelvis.pose.qz'] ** 2)
        tip = math.degrees(math.acos(max(-1.0, min(1.0, up))))
        if bus['pelvis.pose.y'] < FELL_M or tip > FELL_DEG:
            fell = bus['t']
            break
        if bus['t'] < since:
            continue
        if z0 is None:
            z0, t0 = bus['pelvis.pose.z'], bus['t']
        tau, w = np.array(d.ctrl[:n]), np.array(d.qvel[v])
        for i, j in stroked:
            s = drives.ratio(j, math.degrees(d.qpos[world.qadr[i]])) / drives.ratio(j)
            tau[i], w[i] = tau[i] / s, w[i] * s
        peak, top = np.maximum(peak, np.abs(tau)), np.maximum(top, np.abs(w))
        power = tau * w
        watts = np.maximum(watts, power)
        work += float(power[power > 0.0].sum()) * 1e-3
        loads = max(loads, bus['pelvis.pose.left_load'], bus['pelvis.pose.right_load'])
        square += tau * tau
        count += 1
        pitch.append(tip)
        passes += 1
        if passes % 100 == 0:
            mujoco.mj_fullM(m, d, full)
            load += np.diag(full)[v] - m.dof_armature[v]
    gone = (bus['pelvis.pose.z'] - z0) if z0 is not None else 0.0
    span = (bus['t'] - t0) if t0 is not None else 0.0
    steps = runner.steps
    whole = [s for s in steps if 'off' in s and s['t'] >= since]
    flights = [b['t'] - a['off'] for a, b in zip(steps, steps[1:]) if 'off' in a and a['t'] >= since]
    speeds = [s['v'] for s in whole]
    asked = speed if to_speed is None else to_speed
    measures = {
        'fell': fell, 'm': gone, 's': span, 'speed': gone / span if span else 0.0,
        'off': abs(gone / span / asked - 1.0) if span else 1.0,
        'steps': len(steps), 'step_hz': (len(whole) / span) if span else 0.0,
        'contact': sum(s['off'] - s['t'] for s in whole) / max(1, len(whole)),
        'flight': sum(flights) / max(1, len(flights)),
        'pitch': (max(pitch) if pitch else 0.0), 'load': loads,
        'uneven': (max(speeds) - min(speeds)) if speeds else 0.0,
        'flights': (min(flights), max(flights)) if flights else (0.0, 0.0),
        'energy': work / gone if gone > 0.1 else float('inf'), 'watts': work / span if span else 0.0}
    demand = {j: (float(peak[i]), math.sqrt(square[i] / max(1, count)), math.degrees(top[i]),
                  float(watts[i]), float(load[i] / max(1, passes // 100)))
              for i, j in enumerate(JOINTS)}
    body.disarm()
    body.close()
    return measures, steps, demand


def shown(measures, steps, asked, rows=True):
    from machine import drives
    from tools.sim import drive_sizes
    if rows:
        print('      t  foot   speed   sink  pelvis  ahead | stance  flight   rise')
        for a, b in zip(steps, steps[1:] + [None]):
            print('  %5.2f  %-5s  %5.2f  %+5.2f  %6.3f  %+5.3f | %s  %s  %s' % (
                a['t'], a['side'], a['v'], a['sink'], a['y'], a['ahead'],
                '%6.3f' % (a['off'] - a['t']) if 'off' in a else '     -',
                '%6.3f' % (b['t'] - a['off']) if b and 'off' in a else '     -',
                '%+5.2f' % a['rise'] if 'rise' in a else '    -'))
    m = measures
    print('%s: %.1f m in %.1f s, %.2f m/s (%.2f apart a landing), %.2f steps/s, stance %.3f s, '
          'flight %.3f s (%.3f-%.3f), tipped %.1f deg at most, a sole %.0f N at most, %.0f J/m, '
          '%.0f W' % (
              'fell at %.2f s' % m['fell'] if m['fell'] else 'ran', m['m'], m['s'], m['speed'],
              m['uneven'], m['step_hz'], m['contact'], m['flight'], m['flights'][0],
              m['flights'][1], m['pitch'], m['load'], m['energy'], m['watts']))
    print('kind        | peak/has    rms/holds  deg/s/has   peak W |     A     P     T     S     V')
    for kind in KINDS:
        both = [j for j in asked if drives.kind(j) == kind]
        if not both:
            continue
        j = max(both, key=lambda j: asked[j][0])
        peak, rms, speed, watts, load = asked[j]
        a, p, t, _j, _q, s, v = drive_sizes.numbers(j, peak, rms, speed, watts, load)
        print('%-11s | %5.0f/%-5.0f %5.0f/%-5.0f %5.0f/%-5.0f %6.0f | %5.2f %5.2f %5.2f %5.2f %5.2f' % (
            kind, peak, drives.peak(j), rms, drive_sizes.held(j)[0], speed, drives.speed(j), watts,
            a, p, t, s, v))


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--speed', type=float, default=2.0, help='m/s asked')
    parser.add_argument('--to', type=float, default=6.0, help='seconds simulated')
    parser.add_argument('--json', action='store_true', help='the measures and the demand, a line')
    parser.add_argument('--swap', nargs='*', default=None, metavar='NAME=V',
                        help='constants set under way at --swap-at, measured from there')
    parser.add_argument('--swap-at', type=float, default=4.0)
    parser.add_argument('--to-speed', type=float,
                        help='the speed asked goes to this from --swap-at on, RAMP m/s^2')
    parser.add_argument('knobs', nargs='*', metavar='NAME=V', help='constants moved')
    args = parser.parse_args(argv)
    values = {k: float(v) for k, v in (kv.split('=') for kv in args.knobs)}
    swap = (None if args.swap is None
            else {k: float(v) for k, v in (kv.split('=') for kv in args.swap)})
    measures, steps, asked = ran(args.speed, args.to, values, swap, args.swap_at, args.to_speed)
    if args.json:
        print('RESULT ' + json.dumps({'measures': measures, 'asked': asked}))
    else:
        shown(measures, steps, asked)
    return 0


if __name__ == '__main__':
    sys.exit(main())
