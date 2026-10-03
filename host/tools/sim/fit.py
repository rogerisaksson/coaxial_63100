#!/usr/bin/env python3
"""Her mechanism checked by geometry, nothing drawn.

Each rod's transmission over its stroke, each drive's reach past her skin standing, and the least
clearance between her drums, her boards, her ankles' rods and her bones over the poses she is put in - the walk's cycle, the squat, the get-up's keyframes,
the fall's catch, crouch and tuck.

    python tools/sim/fit.py
    python tools/sim/fit.py --worst 20       # that many of the closest pairs

A drum is her drive's assembly as drawn (`coaxial.graphics.drums`), a cylinder; the hip's turn with
the yokes they ride - the pitch's with the yaw and the roll, the roll's with the yaw - about the
hip. A bone is its carbon tube from its joint to the next; a rod its tube from its crank's pin to
its ball (`machine.linkage`). Parts in segments neither the same nor neighbours are not a pair -
their shells keep them apart, the physics colliding those - nor parts bolted together: a drum
and its own segment's bone, a bone and its neighbour's at their joint, a rod within a ball of its
ends.
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from coaxial.model.blocks import numpy as np  # noqa: E402

#: Each segment's carbon tube's radius, m (`mechanism._bones`; a rod's `linkage.ROD_R`).
TUBE_R = {'pelvis': 0.015, 'torso': 0.02, 'neck': 0.013, 'upper_arm': 0.012, 'forearm': 0.01,
          'thigh': 0.016, 'shank': 0.014, 'foot': 0.006}
END_KEEP_M = 0.02

#: The shoulders' girdle off the torso's top, m up it (`mechanism.wires`); the femur and the
#: tibia through `skeleton.HUNG`'s points.
GIRDLE_Y = 0.325


def poses(csv=None, every=10):
    """[(name, {joint: deg})]: where she is put - standing, the walk's cycle, the squat, the
    get-up's keyframes, the fall's catch, crouch and tuck - or where a recording (`csv`, R's or
    `look.py`'s rows) had her, every `every`th row: a keyframe is asked, not reached."""
    from machine import arrival, falls, figure, gait, getup
    if csv:
        from tools.sim.look import recorded
        rows = recorded(csv)[::every]
        return [('%s %.2f s' % (r['stage'], float(r['t'])), {j: float(r[j]) for j in figure.JOINTS})
                for r in rows]
    out = [('stand', gait.stand())]
    out += [('walk %.2f' % p, gait.walk(0.0, phase=p)) for p in np.arange(0.0, 1.0, 0.05)]
    out.append(('squat', arrival.angles_of(arrival._squat())))
    for table in ('UNFOLD', 'TO_FRONT', 'KNEES_UNDER', 'SIT_BACK', 'ONTO_FEET'):
        out += [('%s %s' % (table.lower(), step[1]), step[3]) for step in getattr(getup, table)
                if len(step) > 3 and isinstance(step[3], dict)]
    stand = gait.stand()
    out += [('catch ' + k, dict(stand, **v)) for k, v in falls.CATCH.items()]
    out += [('crouch %+.0f' % tip, dict(stand, **falls.crouch(tip))) for tip in (-90.0, 0.0, 90.0)]
    out.append(('tuck', dict(stand, **falls.TUCK)))
    return out


def rods():
    """[(joint kind, stroke, worst transmission in and out deg, lever least and most)]."""
    from machine import linkage
    out = []
    for kind, (rod, (lo, hi)) in linkage.RODS.items():
        up, ahead, r, _t0, b, _beta = rod
        length, branch = linkage._laid(kind)
        worst_in = worst_out = 90.0
        levers = []
        for d in np.arange(lo, hi + 0.5, 1.0):
            q = math.radians(d)
            t = linkage._root(kind, length, q, 0.0, branch)
            py, pz = up - r * math.cos(t), ahead + r * math.sin(t)
            _x, by, bz = linkage.ball(kind, q)
            wy, wz = by - py, bz - pz
            wl = math.hypot(wy, wz)
            worst_in = min(worst_in, math.degrees(math.asin(min(1.0, abs(
                (py - up) * wz - (pz - ahead) * wy) / (r * wl)))))
            worst_out = min(worst_out, math.degrees(math.asin(min(1.0, abs(by * wz - bz * wy)
                                                                   / (b * wl)))))
            levers.append(linkage.lever('left_' + linkage.ROD_AT[kind][0], float(d)))
        out.append((kind, (lo, hi), worst_in, worst_out, min(levers), max(levers)))
    return out


def outputs():
    """[(joint, what its output turns, [what is wrong])], her left's and her trunk's: each drum's
    axis that of its joint, its rod's crank or its belt's pulley (x) - else across it a bevel
    pair's (`linkage.BEVELS`) -, its spur pair's pinion (z);
    a crank or a belt past its gearbox's end; on its joint's axis, the pivot on the drum's line
    and, the drum along the segment it turns, its gearbox's end toward that."""
    from coaxial.graphics import drums
    from machine import drives, figure, linkage
    placed = {name[len('drive_'):]: (parent, at) for name, parent, at, _m in drums.drums()}
    segs = {j: seg for seg in figure.SEGMENTS for j, _ax, _s in seg[2]}
    out = []
    for joint, (parent, at) in placed.items():
        if joint.startswith('right_'):
            continue
        kind, seg = drives.kind(joint), segs[joint]
        axis, half = drums.AXES[joint]
        k = [abs(a) for a in axis].index(1.0)
        letter, end, place, wrong = 'xyz'[k], axis[k], None, []
        if kind in linkage.RODS:
            want, what, place = 'x', 'crank', linkage.ROD_AT[kind][1]
        elif kind in linkage.BELTS and kind in linkage.BEVELS:
            want, what = ('y' if letter == 'x' else letter), 'bevel'
        elif kind in linkage.BELTS:
            want, what, place = 'x', 'belt', linkage.BELTS[kind][2]
        elif kind in linkage.GEARS or kind in linkage.PLANAR:
            want, what = 'z', 'pinion' if kind in linkage.GEARS else 'crank'
        else:
            want, what = next(ax for j, ax, _s in seg[2] if j == joint), 'joint'
            pivot = seg[3] if parent == seg[1] else (0.0, 0.0, 0.0)
            off = [p - a for p, a in zip(pivot, at)]
            if math.hypot(*[off[i] for i in range(3) if i != k]) > 1e-6:
                wrong.append('%.0f mm off its joint, nothing between' % (
                    1e3 * math.hypot(*[off[i] for i in range(3) if i != k])))
            elif letter == 'y' and parent == seg[1] and seg[6][1] * end < 0.0:
                wrong.append("its gearbox's end away from the segment it turns")
        if letter != want:
            wrong.append('along %s, its %s about %s' % (letter, what, want))
        if place is not None and letter == 'x' and (place - at[0]) * end < half:
            wrong.append("its %s at x %+.3f, its gearbox's end at %+.3f" % (
                what, place, at[0] + end * half))
        out.append((joint, what, wrong))
    return out


def _frames(angles):
    from machine import figure
    return {k: (np.array(R), np.array(p)) for k, (p, R) in figure.frames(
        angles, (0.0, 0.0, 0.0), ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))).items()}


def parts(angles):
    """{name: ('drum', centre, axis, radius, half) | ('tube', a, b, radius)}, world, and the
    segment each rides."""
    from coaxial.graphics import drums
    from machine import drives, figure, linkage, skeleton
    fr = _frames(angles)
    out, rides = {}, {}
    for name, parent, at, *_mesh in drums.drums():
        joint = name[len('drive_'):]
        axis, _half = drums.AXES[joint]
        R, p = fr[parent]
        c, a = p + R @ np.array(at, float), R @ np.array(axis, float)
        # Its stack's parts, each its own drum (`drives.along`): the motor the drive's name.
        for part, radius, along, long in drives.along(joint):
            named = {'board': 'inv_', 'motor': 'drive_', 'gear': 'gear_'}[part] + joint
            out[named] = ('drum', c + a * along, a, radius, long / 2.0)
            rides[named] = parent
    R, p = fr['pelvis']
    boom = [p + R @ np.array(q, float) for q in skeleton.BOOM[1]]
    for k, (a, b) in enumerate(zip(boom, boom[1:])):
        out['bone_pelvis>pelvis' + '+' * k] = ('tube', a, b, skeleton.BOOM[0])
        rides['bone_pelvis>pelvis' + '+' * k] = 'pelvis'
    for side in ('left_', 'right_'):
        s = 1.0 if side == 'left_' else -1.0
        for stage in ('hip_yaw', 'hip_roll'):
            R, p = fr[side + stage]
            tubes, rings = skeleton.gimbal(stage)
            on = [('tube', p + R @ np.array((s * a[0],) + a[1:]), p + R @ np.array(
                (s * b[0],) + b[1:]), r) for a, b, r in tubes] + [
                ('drum', p + R @ np.array((s * c[0],) + c[1:]), R @ np.eye(3)['xyz'.index(axis)],
                 r, half) for c, axis, r, half in rings]
            for k, part in enumerate(on):
                out['gimbal_' + side + stage + '+' * k] = part
                rides['gimbal_' + side + stage + '+' * k] = side + stage
    for seg in figure.SEGMENTS:
        if seg[1] is None or seg[1] in skeleton.TRUNK:
            continue
        bare = seg[1].split('_', 1)[-1] if seg[1].startswith(('left_', 'right_')) else seg[1]
        if bare in TUBE_R:
            Rp, pp = fr[seg[1]]
            if seg[0].endswith('upper_arm'):
                pp = pp + Rp @ np.array([0.0, GIRDLE_Y, 0.0])
            name = 'bone_%s>%s' % (seg[1], seg[0])
            if bare in skeleton.HUNG:
                s = 1.0 if seg[1].startswith('left_') else -1.0
                for k, (a, b, r) in enumerate(skeleton.runs(bare, s)):
                    out[name + '+' * k] = ('tube', pp + Rp @ np.array(a), pp + Rp @ np.array(b), r)
                    rides[name + '+' * k] = seg[1]
            else:
                out[name] = ('tube', pp, fr[seg[0]][1], TUBE_R[bare])
                rides[name] = seg[1]
    for side in ('left_', 'right_'):
        for kind, (rod, _stroke) in linkage.RODS.items():
            joint = side + kind
            seg = next(s for s in figure.SEGMENTS
                       if side + linkage.ROD_AT[kind][0] in [j for j, *_ in s[2]])
            up, ahead, r, t0 = rod[:4]
            s, (x, by, bz) = (1.0 if side == 'left_' else -1.0), linkage.ball(kind)
            Rp, pp = fr[seg[1]]
            hub = pp + Rp @ (np.array(seg[3], float)
                             + np.array([s * linkage.ROD_AT[kind][1], up, ahead]))
            t = t0 + linkage.crank(joint, *linkage.turns(joint, angles))
            pin = hub + Rp @ np.array([0.0, -r * math.cos(t), r * math.sin(t)])
            Rj, pj = fr[seg[0]]
            ball = pj + Rj @ np.array([s * x, by, bz])
            bend = np.array(linkage.bent(kind, pin, ball, s * Rp[:, 0], s))
            out['rod_' + joint], out['rod_' + joint + '+'] = (
                ('tube', pin, bend, linkage.ROD_R), ('tube', bend, ball, linkage.ROD_R))
            rides['rod_' + joint] = rides['rod_' + joint + '+'] = seg[1]
    for joint, (seg, at, _kg, radius, faces) in drives.boards().items():
        R, p = fr[seg]
        out['board_' + joint] = ('drum', p + R @ np.array(at, float),
                                 R @ np.eye(3)['xyz'.index(faces)], radius, 0.006)
        rides['board_' + joint] = seg
    for joint in [j for kind in linkage.PLANAR for j in linkage.joints(kind)]:
        R, p = fr[(drives.mount(joint) or ('pelvis',))[0]]
        crank, horn, pin, ball = (p + R @ np.array(q) for q in linkage.four_bar(joint, angles))
        out['rod_' + joint] = ('tube', pin, ball, linkage.ROD_R)
        out['crank_' + joint] = ('tube', crank, pin, linkage.PLATE_R)
        out['horn_' + joint] = ('tube', horn, ball, linkage.PLATE_R)
        rides['rod_' + joint] = rides['crank_' + joint] = (drives.mount(joint) or ('pelvis',))[0]
        rides['horn_' + joint] = joint
    for k, (frame, shape) in enumerate(skeleton.trunk()):
        R, p = fr[frame]
        key = 'frame_%s%s' % (frame, '+' * k)
        out[key] = (('drum', p + R @ np.array(shape[1]), R @ np.array(shape[2]), shape[3],
                     shape[4]) if shape[0] == 'ring' else
                    ('tube', p + R @ np.array(shape[1]), p + R @ np.array(shape[2]), shape[3]))
        rides[key] = frame
    for side in ('left_', 'right_'):
        for k, (name, on, shape) in enumerate(skeleton.held(side)):
            R, p = fr[on]
            key = 'held_%s%s' % (name, '+' * k)
            out[key] = (('drum', p + R @ np.array(shape[1]), R @ np.array(shape[2]), shape[3],
                         shape[4]) if shape[0] == 'ring' else
                        ('tube', p + R @ np.array(shape[1]), p + R @ np.array(shape[2]), shape[3]))
            rides[key] = on
        for seg in skeleton.HUNG:
            R, p = fr[side + seg]
            for k, (at, r, half) in enumerate(skeleton.collars(side, seg)):
                key = 'collar_%s%s%s' % (side, skeleton.HUNG[seg][2][k][0], '+' * k)
                out[key] = ('drum', p + R @ np.array(at), R[:, 0], r, half)
                rides[key] = side + seg
    return out, rides


def _points(part, n=12):
    """Points over a part's surface or axis, and the radius they stand for."""
    if part[0] == 'tube':
        _k, a, b, r = part
        return a + np.linspace(0.0, 1.0, n)[:, None] * (b - a), r
    _k, c, ax, r, half = part
    w = np.array([1.0, 0.0, 0.0]) if abs(ax[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = np.cross(ax, w)
    u /= np.linalg.norm(u)
    v = np.cross(ax, u)
    t = np.linspace(0.0, 2.0 * np.pi, 16, endpoint=False)
    ring = np.cos(t)[:, None] * u + np.sin(t)[:, None] * v
    return np.concatenate([c + s * half * ax + f * r * ring for s in (-1.0, 0.0, 1.0)
                           for f in (1.0, 0.5)]), 0.0


def _gap(points, part):
    """The least distance of `points` outside `part`, m: under 0 inside."""
    if part[0] == 'tube':
        _k, a, b, r = part
        ab = b - a
        t = np.clip(((points - a) @ ab) / max(ab @ ab, 1e-12), 0.0, 1.0)
        return float((np.linalg.norm(points - (a + t[:, None] * ab), axis=1) - r).min())
    _k, c, ax, r, half = part
    d = points - c
    along = d @ ax
    radial = np.linalg.norm(d - along[:, None] * ax, axis=1)
    out_a, out_r = np.abs(along) - half, radial - r
    return float(np.where((out_a > 0) & (out_r > 0), np.hypot(out_a, out_r),
                          np.maximum(out_a, out_r)).min())


def _bolted(a, b, rides, parts_):
    """Whether two parts are not a pair: in segments neither the same nor neighbours - their
    shells keep them apart, and the physics collides those - unless one is a rod, which no shell
    holds; a bone and a drum on its segment or on a joint at either of its ends, two bones meeting
    at a joint."""
    from machine import figure, skeleton
    if _stack(a) == _stack(b):
        return True
    a, b = _stack(a), _stack(b)
    joints = {s[0]: [j for j, *_ in s[2]] for s in figure.SEGMENTS}
    parent = {s[0]: s[1] for s in figure.SEGMENTS}
    # A stage is its segment's here; a gimbal's members and its hip's drums bolted together.
    stage = {j: s[0] for s in figure.SEGMENTS for j, *_ in s[2][:-1]}
    sa, sb = stage.get(rides[a], rides[a]), stage.get(rides[b], rides[b])
    if a.startswith('rod_') and a.rstrip('+') == b.rstrip('+'):
        return True
    if {a.split('_', 1)[0], b.split('_', 1)[0]} <= {'crank', 'horn', 'rod'} and (
            a.split('_', 1)[1].rstrip('+') == b.split('_', 1)[1].rstrip('+')):
        return True
    link = [x for x in (a, b) if x.startswith(('crank_', 'horn_'))]
    if link and (rides[a] == rides[b] or (b if link[0] == a else a)
                 == 'drive_' + link[0].split('_', 1)[1]):
        return True
    held = [x for x in (a, b) if x.startswith(('held_', 'collar_', 'frame_'))]
    if held and (rides[a] == rides[b] or (b if held[0] == a else a)
                 == 'drive_' + held[0].split('_', 1)[1].rstrip('+')):
        return True
    hip = [x for x in (a, b) if x.startswith('gimbal_')]
    if hip and (a.startswith('gimbal_') == b.startswith('gimbal_') or any(
            x.startswith('drive_') and drives_hip(x) for x in (a, b))) and sa == sb:
        return True
    if (not (sa == sb or parent.get(sa) == sb or parent.get(sb) == sa)
            and not a.startswith('rod_') and not b.startswith('rod_')):
        return True
    kinds = (a.split('_', 1)[0], b.split('_', 1)[0])
    if 'bone' in kinds and 'drive' in kinds:
        bone, drum = (a, b) if kinds[0] == 'bone' else (b, a)
        root, tip = bone[len('bone_'):].rstrip('+').split('>')
        bare = root.split('_', 1)[-1] if root.startswith(('left_', 'right_')) else root
        if bare in skeleton.HUNG:
            # Hung, a bone holds only the drums it clamps and those on its own segment.
            side = root[:root.index('_') + 1]
            return (rides[drum] == root or drum[len('drive_'):] in
                    [side + j for j, _part, _y in skeleton.HUNG[bare][2]])
        return (rides[drum] == root or drum[len('drive_'):] in joints[root] + joints[tip])
    cross = [x for x in (a, b) if x.startswith('held_') and x.rstrip('+').endswith('_cross')]
    if cross and 'bone' in kinds:
        # The ankle's cross: its pins in the tibia's clevis and the foot's cheeks.
        return True
    if kinds == ('bone', 'bone'):
        ends_a, ends_b = (set(x[len('bone_'):].rstrip('+').split('>')) for x in (a, b))
        return bool(ends_a & ends_b)
    return False


def _stack(name):
    """A stack's part (`drives.along`) by its drive's name: its gearbox and its inverter its
    motor's."""
    return 'drive_' + name.split('_', 1)[1] if name.startswith(('gear_', 'inv_')) else name


def drives_hip(name):
    """Whether a part is a hip's drive's drum, its gimbal's to carry."""
    return name.startswith(('drive_left_hip', 'drive_right_hip'))


def clearances(worst=10, csv=None):
    """[(gap m, a, b, pose)]: each pair's least gap over the poses, closest first."""
    best = {}
    for name, angles in poses(csv):
        parts_, rides = parts(angles)
        names = sorted(parts_)
        for i, a in enumerate(names):
            pa, ra = _points(parts_[a])
            if a.startswith('rod_'):
                pa = pa[np.linalg.norm(pa - pa[-1 if a.endswith('+') else 0], axis=1)
                        > END_KEEP_M]
            for b in names[i + 1:]:
                if _bolted(a, b, rides, parts_):
                    continue
                pb = pa
                if b.startswith('rod_'):
                    end = parts_[b][2 if b.endswith('+') else 1]
                    pb = pa[np.linalg.norm(pa - end, axis=1) > END_KEEP_M]
                g = _gap(pb, parts_[b]) - ra if len(pb) else 1.0
                if (a, b) not in best or g < best[(a, b)][0]:
                    best[(a, b)] = (g, name)
    return sorted((g, a, b, pose) for (a, b), (g, pose) in best.items())[:worst]


def _rings(mesh):
    """(axis, [(at, cx, cc, rx, rc)]) of a loft's rings, its part's frame (`shapes.loft`): up
    it (1) or forward (2, a shoe's), the other across its rings; None not one - forward the
    sneaker's read as up, its toes' drive stood 119 mm out of it."""
    from coaxial.graphics.shapes import AROUND
    c = np.asarray(mesh[0])
    n = (len(c) - 2) // AROUND
    if n < 2 or n * AROUND + 2 != len(c):
        return None
    rows = [c[k * AROUND:(k + 1) * AROUND] for k in range(n)]
    means = np.array([r.mean(0) for r in rows])
    axis = 1 if np.ptp(means[:, 1]) >= np.ptp(means[:, 2]) else 2
    other = 3 - axis
    out = []
    for ring in rows:
        lo, hi = ring.min(0), ring.max(0)
        out.append((float(ring[:, axis].mean()), (lo[0] + hi[0]) / 2.0,
                    (lo[other] + hi[other]) / 2.0, max((hi[0] - lo[0]) / 2.0, 1e-4),
                    max((hi[other] - lo[other]) / 2.0, 1e-4)))
    return axis, sorted(out)


def _excess(points, rings):
    """Each point's reach past a loft's rings (`_rings`), m, by its ellipse where it stands along
    the loft (inf past its ends)."""
    axis, rings = rings
    other = 3 - axis
    ys = np.array([r[0] for r in rings])
    out = np.full(len(points), np.inf)
    inside = (points[:, axis] >= ys[0]) & (points[:, axis] <= ys[-1])
    if not inside.any():
        return out
    q = points[inside]
    cols = [np.interp(q[:, axis], ys, [r[i] for r in rings]) for i in range(1, 5)]
    cx, cz, rx, rz = cols
    norm = np.hypot((q[:, 0] - cx) / rx, (q[:, other] - cz) / rz)
    out[inside] = (norm - 1.0) * np.minimum(rx, rz)
    return out


def drawn(dressed):
    """{joint: m}: each drum's, board's and rod's worst reach past her drawn shell (`dressed`
    False) or past her clothes and her skin where they leave it bare, standing."""
    from coaxial.graphics import gynoid
    from machine import gait
    stand = gait.stand()
    body = gynoid.body(dressed=dressed)
    frames = body._frames(stand)
    worn = [(rings, turn, spot) for part, frame in zip(body.parts, frames) if frame is not None
            for rings in [_rings(part[5])] if rings is not None for turn, spot in [frame]]
    parts_, _rides = parts(stand)
    out = {}
    for name, part in parts_.items():
        if not name.startswith(('drive_', 'gear_', 'inv_', 'board_', 'rod_', 'gimbal_', 'frame_')):
            continue
        name = _stack(name)
        points, radius = _points(part)
        best = np.full(len(points), np.inf)
        for rings, turn, spot in worn:
            best = np.minimum(best, _excess((points - spot) @ turn, rings))
        key = name.split('_', 1)[1] if name.startswith('drive_') else name
        out[key] = max(out.get(key, -1.0), float(best.max() + radius))
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--worst', type=int, default=12, help='the closest pairs shown')
    parser.add_argument('--csv', help='poses from a recording (R, build/recordings/*.csv)')
    args = parser.parse_args(argv)
    from machine import drives, figure
    from tools.sim.strides import _out
    wrong = [(j, w) for j, _what, w in outputs() if w]
    print('drives turning what they drive from their gearbox: %d wrong' % len(wrong))
    for joint, w in wrong:
        print('  %-16s %s' % (joint, '; '.join(w)))
    print('rods: stroke, transmission at worst in / out, lever')
    for kind, (lo, hi), mi, mo, l0, l1 in rods():
        print('  %-10s %+4.0f..%+3.0f deg  %4.1f / %4.1f deg  %.2f-%.2f' % (kind, lo, hi, mi, mo,
                                                                         l0, l1))
    stand = {j: v for j, v in __import__('machine.gait', fromlist=['stand']).stand().items()}
    row = dict({j: stand.get(j, 0.0) for j in figure.JOINTS},
               **{'%s_%s' % (s[0], a): 0.0 for s in figure.SEGMENTS for a in 'xyz'})
    reach = sorted(((_out(row, [j]), drives.kind(j), drives.of(j)[0]) for j in figure.JOINTS
                    if j.startswith(('left_', 'spine', 'waist', 'neck', 'head'))
                    and not drives.passive(j)), reverse=True)
    print('drives past her skin standing, mm: ' + ', '.join(
        '%s %s %+.0f' % (k, s, r) for r, k, s in reach if r > -1e9))
    for dressed, what in ((False, 'her shell'), (True, 'her clothes')):
        got = drawn(dressed)
        print('drives past %s standing, mm: ' % what + ', '.join(
            '%s %+.0f' % (j, v * 1e3) for j, v in sorted(got.items(), key=lambda kv: -kv[1])
            if j.startswith(('left_', 'board_left', 'rod_left', 'spine', 'waist', 'neck', 'head'))
            and v > -0.03))
    print('closest pairs over %d poses, mm:' % len(poses(args.csv)))
    for g, a, b, pose in clearances(args.worst, args.csv):
        print('  %+6.0f  %-26s %-26s %s' % (g * 1e3, a, b, pose))
    return 0


if __name__ == '__main__':
    sys.exit(main())
