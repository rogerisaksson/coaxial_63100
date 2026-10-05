#!/usr/bin/env python3
"""Takes' walks measured as tools/sim/look.py measures hers, beside a woman's normal walk.

    python tools/sim/mocap.py TAKE.fbx [..]         # a column a take, the first the reference
    python tools/sim/mocap.py TAKE.fbx --joints     # each joint's first and last place, m

A take (`fbx.take`: binary or ASCII, a mocap's, an animation's or her own, `look.py --fbx`):
its longest straight walk only - a turn at the runway's end and a pose left out - each measure
against its path, not the room: a stride's running mean is the path. Its line in look.py's
names, then `normal.BAND`'s measures beside the band, how far off it the take is and how far
from the first.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from coaxial.model.blocks import numpy as np  # noqa: E402   behind the OpenBLAS cap
from tools.sim import normal  # noqa: E402
from tools.sim.fbx import take  # noqa: E402

#: The straight walk: at least WALKING of the take's top speed, its heading within TURN_DEG of
#: the walk's; TRIM_S off each end, where she gathers pace or slows. A take travelling less than
#: IN_PLACE_M walks on the spot: all of it is measured, ahead of her hips' line.
WALKING, TURN_DEG, TRIM_S, IN_PLACE_M = 0.6, 15.0, 0.5, 1.0


def _mean(a, n):
    """`a`'s running mean over n samples, the ends held."""
    pad = np.pad(a, [(n // 2, n - 1 - n // 2)] + [(0, 0)] * (a.ndim - 1), mode='edge')
    return np.stack([np.convolve(pad[:, j], np.ones(n) / n, 'valid')
                     for j in range(a.shape[1])], 1) if a.ndim > 1 else \
        np.convolve(pad, np.ones(n) / n, 'valid')


def straight(times, pelvis):
    """The frames of the take's longest straight walk (a slice)."""
    rate = (len(times) - 1) / (times[-1] - times[0])
    path = _mean(pelvis[:, [0, 2]], int(rate))
    v = np.gradient(path, times, axis=0)
    speed, heading = np.hypot(v[:, 0], v[:, 1]), np.degrees(np.arctan2(v[:, 0], v[:, 1]))
    fast = speed >= WALKING * speed.max()
    best, start = (0, 0), None
    for k in range(len(times) + 1):
        if k < len(times) and fast[k] and (start is None or abs(
                (heading[k] - heading[start] + 180.0) % 360.0 - 180.0) < TURN_DEG):
            start = k if start is None else start
            continue
        if start is not None and k - start > best[1] - best[0]:
            best = (start, k)
        start = k if k < len(times) and fast[k] else None
    trim = int(TRIM_S * rate)
    return slice(best[0] + trim, best[1] - trim)


def walked(times, joints):
    """[(measure, value, unit)] of the take's straight walk: look.py's walk names where the same."""
    up = np.array([0.0, 1.0, 0.0])
    moved = joints['Hips'][-1] - joints['Hips'][0]
    cut = straight(times, joints['Hips']) if np.hypot(moved[0], moved[2]) > IN_PLACE_M else slice(None)
    t, j = times[cut], {k: p[cut] for k, p in joints.items()}
    if len(t) < 2 or t[-1] - t[0] < 2.0:
        raise ValueError('no straight walk of 2 s in the take')
    rate = (len(t) - 1) / (t[-1] - t[0])
    hips = j['LeftUpLeg'] - j['RightUpLeg']                    # to her left
    travel = j['Hips'][-1] - j['Hips'][0]
    if cut == slice(None):                                     # on the spot: ahead of her hips
        travel = np.cross(hips.mean(0), up)
    ahead = (travel - up * travel[1]) / np.linalg.norm(travel - up * travel[1])
    left = np.cross(up, ahead)
    if np.median(hips @ left / np.linalg.norm(hips, axis=1)) < 0.7:
        raise ValueError("her hips' line is not across her path: a turning take, or a rig read wrong")
    roll = np.degrees(np.unwrap(np.arctan2(hips @ up, hips @ left)))      # + her left hip up
    turn = np.degrees(np.unwrap(np.arctan2(hips @ ahead, hips @ left)))   # + her left hip ahead
    spectrum = np.abs(np.fft.rfft((roll - roll.mean()) * np.hanning(len(roll))))
    freqs = np.fft.rfftfreq(len(roll), 1.0 / rate)
    stride = freqs[np.argmax(np.where(freqs > 0.3, spectrum, 0.0))]
    n = int(round(rate / stride))

    def off(p, axis):
        """Along `axis` off the path, the ends' half strides dropped: the mean lags there."""
        return ((p - _mean(p, n)) @ axis)[n // 2:len(p) - n // 2]
    shoulders = (j['LeftArm'] + j['RightArm']) / 2.0
    feet = [j['LeftFoot'] @ left, j['RightFoot'] @ left]
    low = [p[:, 1] < p[:, 1].min() + 0.03 for p in (j['LeftFoot'], j['RightFoot'])]
    both = low[0] & low[1]
    thighs = np.array([min(np.linalg.norm((j['LeftUpLeg'][k] + (j['LeftLeg'][k] - j['LeftUpLeg'][k]) * u)
                                          - (j['RightUpLeg'][k] + (j['RightLeg'][k] - j['RightUpLeg'][k]) * w))
                           for u in (0.15, 0.4, 0.7, 1.0) for w in (0.15, 0.4, 0.7, 1.0))
                       for k in range(0, len(t), 4)])
    lean = np.degrees(np.arctan2((j['Neck'] - j['Spine']) @ ahead, (j['Neck'] - j['Spine']) @ up))
    return [('walked', t[-1] - t[0], 's'), ('speed', np.linalg.norm(travel) / (t[-1] - t[0]), 'm/s'),
            ('steps', 2.0 * stride, '/s'),
            ('hips wag', np.ptp(off(j['Hips'], left)) * 1e3, 'mm'),
            ('shoulders wag', np.ptp(off(shoulders, left)) * 1e3, 'mm'),
            ('pelvis roll', np.ptp(roll), 'deg'), ('pelvis turn', np.ptp(turn), 'deg'),
            ('head bob', np.ptp(j['Head'][:, 1]) * 1e3, 'mm'),
            ('pelvis fore-aft', np.ptp(off(j['Hips'], ahead)) * 1e3, 'mm'),
            ('torso pitch', np.ptp(lean), 'deg'),
            ('feet apart', float(np.mean(np.abs(feet[0] - feet[1])[both])) * 1e3
             if both.any() else float('nan'), 'mm'),
            ('hip joints apart', float(np.mean(np.linalg.norm(hips, axis=1))) * 1e3, 'mm'),
            ('thighs closest', float(thighs.min()) * 1e3, 'mm')]


def walk(times, joints):
    """(times, joints) of the take's straight walk: all of it where it walks on the spot."""
    moved = joints['Hips'][-1] - joints['Hips'][0]
    if np.hypot(moved[0], moved[2]) <= IN_PLACE_M:
        return times, joints
    cut = straight(times, joints['Hips'])
    if len(times[cut]) < 2 or times[cut][-1] - times[cut][0] < TRIM_S:
        return times, joints                     # a short take: a loop, all of it
    return times[cut], {k: p[cut] for k, p in joints.items()}


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('take', nargs='+', help='an FBX with a skeleton; the first the reference')
    parser.add_argument('--joints', action='store_true', help="each joint's first and last place")
    args = parser.parse_args(argv)
    walks, failed = [], 0
    for k, path in enumerate(args.take):
        try:
            times, joints = take(path)
        except (KeyError, ValueError, StopIteration, OSError) as e:
            print('%s: not read: %s' % (os.path.basename(path), e))
            failed = 1
            continue
        print('%d %s: %d frames, %.2f s, %.0f Hz, %d joints' % (
            k + 1, os.path.basename(path), len(times), times[-1] - times[0],
            (len(times) - 1) / (times[-1] - times[0]), len(joints)))
        if args.joints:
            for name, p in joints.items():
                print('  %-16s %s .. %s' % (name, np.round(p[0], 3), np.round(p[-1], 3)))
        try:
            print('  ' + ' | '.join(('%s %.2f %s' if unit in ('s', 'm/s', '/s') else '%s %.1f %s')
                                    % (name, v, unit) for name, v, unit in walked(times, joints)))
        except (KeyError, ValueError) as e:
            print('  mocap: %s' % e)
        try:
            found = normal.measured(*walk(times, joints))
            if not found['strides']:                       # a turning take: all of it
                found = normal.measured(times, joints)
        except (KeyError, ValueError) as e:
            print('  normal: %s' % e)
            failed = 1
            continue
        print('  %d strides%s, a leg %.2f m, %.2f m/s' % (
            found['strides'], ', a loop' if found['loop'] else '', found['leg'],
            found.get('speed', float('nan'))))
        if found['strides']:
            walks.append((str(k + 1), found))
    for a in range(0, len(walks), 10):
        print()
        print('\n'.join(normal.table(walks[:1] * (a > 0) + walks[a:a + 10])))
    return failed


if __name__ == '__main__':
    sys.exit(main())
