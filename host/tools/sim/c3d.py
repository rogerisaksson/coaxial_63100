#!/usr/bin/env python3
"""A C3D marker take as a get-up's timeline: what of her is on the floor, and where.

    python tools/sim/c3d.py TAKE.c3d [EVERY_S]

Every EVERY_S (0.25): the pelvis's height and place, the trunk's lean, the hips' and knees' bend.

The markers are Vicon's 41 (CMU's set, any 'Subject:' prefix off), z up, mm. A marker under
FLOOR_MM over the lowest the take's feet reach is on the floor; a hand, knee or the seat on it
is placed ahead (+) of the feet's middle along the pelvis's forward, mm.

Found (2026-09-30), six get-up takes: face down, the push-up onto hands and knees on tucked toes
(the knees 0.42 m ahead of the toes, the hands 1.1), a hand walked back to 0.7, the hips back
over the feet at 0.45-0.48 m lifting the knees, a squat on the toes (the knees 126-144 deg), up
with the heels coming down, 2.5 s from the push-up; on the back or the side, onto the back and
sat up (the trunk 86 -> 2-7 deg in 1.25 s), a hand beside the seat bearing most of her as the
seat went 0.66 -> 0.12 m behind the feet in 1 s and 0.13 -> 0.37 m up, the deep squat, and up.
"""
import math
import struct
import sys

#: A marker this near the floor touches it, mm.
FLOOR_MM = 60.0

#: Her parts by their markers.
PARTS = {'head': ('LFHD', 'RFHD', 'LBHD', 'RBHD'), 'l.hand': ('LFIN', 'LWRA', 'LWRB'),
         'r.hand': ('RFIN', 'RWRA', 'RWRB'), 'l.elbow': ('LELB',), 'r.elbow': ('RELB',),
         'l.knee': ('LKNE',), 'r.knee': ('RKNE',), 'l.toe': ('LTOE', 'LMT5'),
         'r.toe': ('RTOE', 'RMT5'), 'l.heel': ('LHEE',), 'r.heel': ('RHEE',),
         'seat': ('LBWT', 'RBWT'), 'back': ('T10', 'RBAC'), 'l.shin': ('LSHN',), 'r.shin': ('RSHN',),
         'l.thigh': ('LTHI',), 'r.thigh': ('RTHI',), 'chest': ('STRN', 'CLAV')}
PLACED = ('l.hand', 'r.hand', 'l.knee', 'r.knee', 'seat')


def read(path):
    """(rate Hz, [{marker: (x, y, z) mm or None}] a frame) of a C3D, its header's and POINT's."""
    raw = open(path, 'rb').read()
    h = struct.unpack('<256H', raw[:512])
    points, analog, first, last, start, per = h[1], h[2], h[3], h[4], h[8], h[9]
    scale, rate = struct.unpack_from('<f', raw, 12)[0], struct.unpack_from('<f', raw, 20)[0]
    labels = _labels(raw, raw[0])
    floats = scale < 0
    size = (16 if floats else 8) * points + (4 if floats else 2) * analog * per
    frames, at = [], (start - 1) * 512
    for _ in range(last - first + 1):
        vals = struct.unpack_from('<%d%s' % (4 * points, 'f' if floats else 'h'), raw, at)
        at += size
        k = 1.0 if floats else scale
        frames.append({name: None if vals[4 * i + 3] < 0 else
                       (vals[4 * i] * k, vals[4 * i + 1] * k, vals[4 * i + 2] * k)
                       for i, name in enumerate(labels)})
    return rate, frames


def _labels(raw, block):
    """POINT:LABELS, the prefix off."""
    at, groups = (block - 1) * 512 + 4, {}
    while True:
        n, gid = struct.unpack_from('bb', raw, at)
        if n == 0:
            break
        name = raw[at + 2:at + 2 + abs(n)].decode('latin-1')
        body = at + 4 + abs(n)
        step = struct.unpack_from('<h', raw, at + 2 + abs(n))[0]
        if gid < 0:
            groups[-gid] = name
        elif groups.get(gid) == 'POINT' and name == 'LABELS':
            dims = raw[body + 2:body + 2 + raw[body + 1]]
            text = raw[body + 2 + len(dims):body + 2 + len(dims) + dims[0] * dims[1]]
            return [text[i:i + dims[0]].decode('latin-1').strip().split(':')[-1]
                    for i in range(0, len(text), dims[0])]
        if step == 0:
            break
        at += 2 + abs(n) + step
    raise ValueError('no POINT:LABELS')


def _mean(points):
    got = [p for p in points if p is not None]
    return tuple(sum(c) / len(got) for c in zip(*got)) if got else None


def _bend(a, b, c):
    """The bend at b between a-b and c-b, deg: 0 straight."""
    if a is None or b is None or c is None:
        return float('nan')
    u, v = [x - y for x, y in zip(a, b)], [x - y for x, y in zip(c, b)]
    cos = sum(x * y for x, y in zip(u, v)) / math.sqrt(sum(x * x for x in u) * sum(x * x for x in v))
    return 180.0 - math.degrees(math.acos(max(-1.0, min(1.0, cos))))


def timeline(rate, frames, every=0.25):
    """[str] a row every `every` s."""
    floor = min(p[2] for f in frames for n in ('LTOE', 'RTOE', 'LHEE', 'RHEE') if (p := f.get(n)))
    rows = []
    for i in range(0, len(frames), max(1, round(every * rate))):
        f = frames[i]
        pelvis = _mean([f.get(n) for n in ('LFWT', 'RFWT', 'LBWT', 'RBWT')])
        neck = _mean([f.get('C7'), f.get('CLAV')])
        front, back = _mean([f.get('LFWT'), f.get('RFWT')]), _mean([f.get('LBWT'), f.get('RBWT')])
        feet = _mean([f.get(n) for n in ('LHEE', 'RHEE', 'LTOE', 'RTOE')])
        if pelvis is None or neck is None or front is None or back is None or feet is None:
            continue
        way = (front[0] - back[0], front[1] - back[1])
        n = math.hypot(*way) or 1.0

        def ahead(p, feet=feet):
            return ((p[0] - feet[0]) * way[0] + (p[1] - feet[1]) * way[1]) / n
        up = [a - b for a, b in zip(neck, pelvis)]
        bends = []
        for s in 'LR':
            hip = _mean([f.get(s + 'FWT'), f.get(s + 'BWT')])
            bends += [_bend(neck, hip, f.get(s + 'KNE')), _bend(hip, f.get(s + 'KNE'), f.get(s + 'ANK'))]
        on = []
        for part, names in PARTS.items():
            where = _mean([p for m in names if (p := f.get(m)) is not None and p[2] - floor < FLOOR_MM])
            if where is not None:
                on.append(part + ('@%+.0f' % ahead(where) if part in PLACED else ''))
        rows.append('%5.2f pelvis %4.0f mm @%+5.0f  trunk %3.0f deg  hip %3.0f/%3.0f knee %3.0f/%3.0f | %s'
                    % (i / rate, pelvis[2] - floor, ahead(pelvis),
                       math.degrees(math.atan2(math.hypot(up[0], up[1]), up[2])), bends[0], bends[2],
                       bends[1], bends[3], ' '.join(on)))
    return rows


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if not argv:
        print(__doc__)
        return 1
    rate, frames = read(argv[0])
    print('%s: %d frames at %.0f Hz' % (argv[0], len(frames), rate))
    print('\n'.join(timeline(rate, frames, float(argv[1]) if len(argv) > 1 else 0.25)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
