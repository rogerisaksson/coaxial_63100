#!/usr/bin/env python3
"""A mocap take's walk, measured as tools/sim/look.py measures hers: a binary FBX skeleton.

    python tools/sim/mocap.py TAKE.fbx              # its straight walking, look.py's walk names
    python tools/sim/mocap.py TAKE.fbx --joints     # each joint's first and last place, m

The take's longest straight walk only - a turn at the runway's end and a pose left out - each
measure against its path, not the room: a stride's running mean is the path. Joints are named
by their last ':' part (Hips, LeftUpLeg, LeftLeg, LeftFoot, Spine, Neck, Head, LeftArm, ..).
"""
import argparse
import os
import struct
import sys
import zlib

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from coaxial.model.blocks import numpy as np  # noqa: E402   behind the OpenBLAS cap

#: FBX time units a second; the orders of `RotationOrder`, the first axis applied first.
KTIME = 46186158000.0
ORDERS = ('xyz', 'xzy', 'yzx', 'yxz', 'zxy', 'zyx')

#: The straight walk: at least WALKING of the take's top speed, its heading within TURN_DEG of
#: the walk's; TRIM_S off each end, where she gathers pace or slows. A take travelling less than
#: IN_PLACE_M walks on the spot: all of it is measured, ahead of her hips' line.
WALKING, TURN_DEG, TRIM_S, IN_PLACE_M = 0.6, 15.0, 0.5, 1.0

#: The joints by another rig's name (Rokoko's, the 01-07 takes of WALK-RUN-CYCLES-MOCAP).
ALIASES = {'LeftThigh': 'LeftUpLeg', 'RightThigh': 'RightUpLeg', 'LeftShin': 'LeftLeg',
           'RightShin': 'RightLeg', 'Spine1': 'Spine'}


def nodes(data):
    """[(name, properties, children)] of a binary FBX (7.x; 64-bit headers from 7500)."""
    head = '<QQQB' if struct.unpack_from('<I', data, 23)[0] >= 7500 else '<IIIB'
    size = struct.calcsize(head)

    def prop(pos):
        code, pos = chr(data[pos]), pos + 1
        if code in 'YCIFDL':
            fmt = {'Y': '<h', 'C': '<?', 'I': '<i', 'F': '<f', 'D': '<d', 'L': '<q'}[code]
            return struct.unpack_from(fmt, data, pos)[0], pos + struct.calcsize(fmt)
        if code in 'fdlib':
            n, packed, length = struct.unpack_from('<III', data, pos)
            raw = data[pos + 12:pos + 12 + length]
            kind = {'f': '<f4', 'd': '<f8', 'l': '<i8', 'i': '<i4', 'b': '?'}[code]
            return (np.frombuffer(zlib.decompress(raw) if packed else raw, kind, n),
                    pos + 12 + length)
        n = struct.unpack_from('<I', data, pos)[0]
        raw = data[pos + 4:pos + 4 + n]
        return (raw.decode('utf-8', 'replace') if code == 'S' else raw), pos + 4 + n

    def node(pos):
        end, count, _length, n = struct.unpack_from(head, data, pos)
        if end == 0:
            return None, pos + size + n
        name, pos = data[pos + size:pos + size + n].decode('ascii', 'replace'), pos + size + n
        props = []
        for _ in range(count):
            value, pos = prop(pos)
            props.append(value)
        kids = []
        while pos < end:
            kid, pos = node(pos)
            if kid is None:
                break
            kids.append(kid)
        return (name, props, kids), end

    out, pos = [], 27
    while pos + size < len(data):
        top, pos = node(pos)
        if top is None:
            break
        out.append(top)
    return out


def _props(n):
    """{name: values} of a node's Properties70."""
    block = next((k for k in n[2] if k[0] == 'Properties70'), None)
    return {} if block is None else {p[1][0]: p[1][4:] for p in block[2] if p[0] == 'P'}


def _turns(deg, order):
    """(n, 3, 3) rotations from Euler degrees (n, 3), `order`'s first axis applied first."""
    c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg))
    one, nil = np.ones(len(deg)), np.zeros(len(deg))
    by = {'x': (one, nil, nil, nil, c[:, 0], -s[:, 0], nil, s[:, 0], c[:, 0]),
          'y': (c[:, 1], nil, s[:, 1], nil, one, nil, -s[:, 1], nil, c[:, 1]),
          'z': (c[:, 2], -s[:, 2], nil, s[:, 2], c[:, 2], nil, nil, nil, one)}
    out = np.broadcast_to(np.eye(3), (len(deg), 3, 3))
    for axis in order:
        out = np.stack(by[axis], 1).reshape(-1, 3, 3) @ out
    return out


def take(path):
    """(times s, {joint: (n, 3) places, m, y up}) of a take, every joint at every key's time.

    A joint's place: its parent's, then Lcl Translation, PreRotation, Lcl Rotation and the
    inverse of PostRotation (offsets, pivots and scaling taken as none, as a mocap's are)."""
    with open(path, 'rb') as f:
        top = {n[0]: n for n in nodes(f.read())}
    settings = _props(top['GlobalSettings'])
    metres = float(settings.get('UnitScaleFactor', [1.0])[0]) / 100.0
    up = int(settings.get('UpAxis', [1])[0])
    objects = {n[1][0]: n for n in top['Objects'][2]}
    models = {i: n for i, n in objects.items() if n[0] == 'Model'}
    parent, channel, curve = {}, {}, {}
    for c in top['Connections'][2]:
        kind, a, b = c[1][:3]
        what = objects.get(a, ('',))[0]
        if kind == 'OO' and a in models and b in models:
            parent[a] = b
        elif kind == 'OP' and what == 'AnimationCurveNode' and b in models:
            channel[(b, c[1][3])] = a
        elif kind == 'OP' and what == 'AnimationCurve':
            curve[(b, c[1][3][-1])] = a
    keys = {}
    for (node, axis), i in curve.items():
        t = next(k for k in objects[i][2] if k[0] == 'KeyTime')[1][0] / KTIME
        v = next(k for k in objects[i][2] if k[0] == 'KeyValueFloat')[1][0]
        keys[(node, axis)] = (t, v.astype(float))
    times = np.unique(np.concatenate([t for t, _v in keys.values()]))

    def lcl(i, name):
        node, rest = channel.get((i, name)), _props(models[i]).get(name, [0.0, 0.0, 0.0])
        if node is not None:                                   # an axis without keys: its default
            given = _props(objects[node])
            rest = [given.get('d|' + a, [rest[k]])[0] for k, a in enumerate('XYZ')]
        return np.stack([np.interp(times, *keys[(node, a)]) if (node, a) in keys
                         else np.full(len(times), float(rest[j]))
                         for j, a in enumerate('XYZ')], 1)

    placed = {}

    def place(i):
        if i not in placed:
            props = _props(models[i])
            pre = _turns(np.array([props.get('PreRotation', [0, 0, 0])], float), 'xyz')
            post = _turns(np.array([props.get('PostRotation', [0, 0, 0])], float), 'xyz')
            order = ORDERS[int(props.get('RotationOrder', [0])[0])]
            turn = pre @ _turns(lcl(i, 'Lcl Rotation'), order) @ np.transpose(post, (0, 2, 1))
            at = lcl(i, 'Lcl Translation')
            if i in parent:
                up_turn, up_at = place(parent[i])
                turn, at = up_turn @ turn, np.einsum('nij,nj->ni', up_turn, at) + up_at
            placed[i] = (turn, at)
        return placed[i]

    joints = {n[1][1].split('\x00')[0].split(':')[-1]: place(i)[1] * metres
              for i, n in models.items()}
    joints.update({ours: joints[theirs] for theirs, ours in ALIASES.items()
                   if theirs in joints and ours not in joints})
    if up == 2:
        joints = {k: np.stack([p[:, 0], p[:, 2], -p[:, 1]], 1) for k, p in joints.items()}
    return times, joints


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
            ('hips across', np.ptp(off(j['Hips'], left)) * 1e3, 'mm'),
            ('shoulders across', np.ptp(off(shoulders, left)) * 1e3, 'mm'),
            ('pelvis roll', np.ptp(roll), 'deg'), ('pelvis turn', np.ptp(turn), 'deg'),
            ('head bob', np.ptp(j['Head'][:, 1]) * 1e3, 'mm'),
            ('pelvis fore-aft', np.ptp(off(j['Hips'], ahead)) * 1e3, 'mm'),
            ('torso pitch', np.ptp(lean), 'deg'),
            ('feet apart', float(np.mean(np.abs(feet[0] - feet[1])[both])) * 1e3
             if both.any() else float('nan'), 'mm'),
            ('hip joints apart', float(np.mean(np.linalg.norm(hips, axis=1))) * 1e3, 'mm'),
            ('thighs closest', float(thighs.min()) * 1e3, 'mm')]


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('take', help='a binary FBX with a skeleton')
    parser.add_argument('--joints', action='store_true', help="each joint's first and last place")
    args = parser.parse_args(argv)
    times, joints = take(args.take)
    print('%s: %d frames, %.2f s, %.0f Hz, %d joints' % (
        os.path.basename(args.take), len(times), times[-1] - times[0],
        (len(times) - 1) / (times[-1] - times[0]), len(joints)))
    if args.joints:
        for name, p in joints.items():
            print('  %-16s %s .. %s' % (name, np.round(p[0], 3), np.round(p[-1], 3)))
    try:
        measured = walked(times, joints)
    except (KeyError, ValueError) as e:
        print('mocap: %s' % e)
        return 1
    print(' | '.join(('%s %.2f %s' if unit in ('s', 'm/s', '/s') else '%s %.1f %s') % (k, v, unit)
                     for k, v, unit in measured))
    return 0


if __name__ == '__main__':
    sys.exit(main())
