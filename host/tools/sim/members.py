#!/usr/bin/env python3
"""Her members under their drives' peaks: each tube's, plate's and rod's stress and flex.

    python tools/sim/members.py             # every member, the ones past a limit starred
    python tools/sim/members.py --family 3  # her tubes cut from three stock sizes

A member is a run of her skeleton (`machine.skeleton`: the bones, the trunk's frame, the boom,
the hip's gimbal, the holders' struts) or a linkage's part (`machine.linkage`: a crank, a horn, a
rod). Its load is the peak its drive delivers (`drives.peak`, through the lever) as a bending
moment, a twist, or a rod's pull on a crank's arm; its section a carbon tube (WALL of its radius),
a printed PAHT-CF plate loaded in its plane, or a solid carbon rod. Each is judged on stress
against its material's allowable and on flex over its length at FLEX_AT of the load against
FLEX_DEG (docs/findings/body.md: 0.25 deg a member at 0.6). No safety factor beyond ALLOW_MPA.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

#: Materials: E GPa, G GPa, allowable MPa. Roll-wrapped carbon tube (its ultimate ~600 in
#: bending) and the high-modulus one - every tube of hers since 2026-10-03, one grade: the
#: femur 50 -> 40 mm in it -, pultruded carbon rod, printed PAHT-CF in its layers'
#: plane (92 tensile, 44 across: parts loaded in-plane, printed flat, the user, 2026-10-03),
#: steel plate. Printed parts hold and join: short and stout, every one an extrusion along its
#: print axis (the user: 2.5D, no overhang in any print pose); the long members carbon tubes
#: cut to length (TUBES).
CARBON, HM_CARBON, ROD, PAHT_CF, STEEL = ((70.0, 20.0, 300.0), (130.0, 30.0, 300.0),
                                           (130.0, 5.0, 600.0), (4.5, 1.6, 50.0),
                                           (200.0, 80.0, 250.0))
#: A tube's wall, of its radius; the flex judged at FLEX_AT of the load, FLEX_DEG at most; the
#: clamp's margin (the user's 1.5x).
WALL, FLEX_AT, FLEX_DEG, MARGIN = 0.1, 0.6, 0.25, 1.5
#: A four-bar's plates: PLATE_R thick (`linkage`), this wide in their plane, printed PAHT-CF; the
#: hip fork's crown a carbon plate CROWN_R thick (`skeleton`), this wide.
PLATE_W, CROWN_W = 0.050, 0.040
#: Stock roll-wrapped carbon tubes, (outer radius, wall) m: 1 mm walls to 25 mm round, 2 mm from
#: 30 (the makers' lists); her tubes cut from as few of them as her members allow (the user,
#: 2026-10-03: minimal variants), each within ROOM of its radius now, carbon CF_KG_M3.
TUBES = tuple((d / 2e3, 0.001 if d < 30 else 0.002)
              for d in (8, 10, 12, 14, 16, 18, 20, 22, 25, 30, 35, 40, 50))
ROOM, CF_KG_M3 = 0.0025, 1550.0
#: Each joint's wind-up: the members its torque passes on its way to the limb, their flex a
#: N m summed in series (`wind`); a rod's its stretch over its horn.
PATH = {'spine': ('spine frame 0', 'pelvis frame 0'), 'spine_roll': ('spine_roll crank',
                                                                      'spine_roll horn',
                                                                      'spine_roll rod'),
        'waist': ('torso frame 0',), 'neck': ('neck frame 0',), 'head': ('neck frame 0',),
        'shoulder': ('torso frame 1', 'upper_arm run 0'), 'elbow': ('upper_arm run 2',
                                                                    'forearm run 0'),
        'wrist': ('forearm run 1',), 'hip_yaw': ('hip fork 0', 'hip fork 1', 'hip fork 2'),
        'hip_roll': ('hip cradle 0', 'hip fork 3'), 'hip': ('thigh run 0', 'thigh run 1'),
        'knee': ('shank run 0', 'shank run 1'), 'ankle': ('ankle rod', 'foot run 2'),
        'ankle_roll': ('ankle_roll rod',), 'foot': ('foot run 2',)}


def _tube(r, t):
    """(I, J, A) m^4, m^4, m^2 of a tube r round, t thick."""
    ri = r - t
    return (math.pi * (r ** 4 - ri ** 4) / 4.0, math.pi * (r ** 4 - ri ** 4) / 2.0,
            math.pi * (r * r - ri * ri))


def tube(r):
    """(I, J, A) of a carbon tube r round: a stock size's wall (TUBES), else WALL of r (1 mm at
    least)."""
    return _tube(r, next((t for rr, t in TUBES if math.isclose(rr, r)), max(WALL * r, 0.001)))


def plate(t, w):
    """I m^4 of a plate t thick, w wide, bent in its plane."""
    return t * w ** 3 / 12.0


def members():
    """[(name, kind, length m, load, (I, J, A), material, c)]: kind 'bend' with a moment N m,
    'twist' with one, 'pull' with a rod's newtons; c the section's outer fibre, m."""
    from machine import drives, linkage, skeleton
    from machine.physics import SERVO
    out = []
    # The design load: the drive's deliverable peak, or MARGIN times the controller's clamp where
    # that is less (one frame on the wrist delivers 120 N m it is never asked).
    peak = {k: min(drives.peak(k if k in ('spine', 'spine_roll', 'waist', 'neck', 'head')
                               else 'left_' + k), MARGIN * SERVO[k][0]) for k in drives.STACKS}
    # The bones: a bone's main runs bent by the larger of its ends' joints; the shank's yoke and
    # cheeks and the foot's keel carry the ankle's rods' pull (`linkage.RODS`), as does the
    # tibia's lower run along with the knee's bending.
    ends = {'thigh': ('hip', 'knee'), 'shank': ('knee', 'ankle'), 'upper_arm': ('shoulder', 'elbow'),
            'forearm': ('elbow', 'wrist'), 'foot': ('ankle', 'foot')}
    rods_n = sum(peak[k] / drives.motors('left_' + k) / min(g[2], g[4])
                 for k, (g, _s) in linkage.RODS.items())
    for seg, (a, b) in ends.items():
        for k, (p, q, r) in enumerate(skeleton.runs(seg)):
            length = math.dist(p, q)
            if seg == 'foot' or (seg == 'shank' and k >= 2):
                out.append(('%s run %d' % (seg, k), 'pull', length, rods_n, tube(r), CARBON, r))
                continue
            out.append(('%s run %d' % (seg, k), 'bend', length, max(peak[a], peak[b]), tube(r),
                        HM_CARBON, r))
    # The trunk's frame: the pitch's torque is a couple on the roll's bearings 66 mm out, each
    # fork leg a cantilever under its share of it (the front bearing's two legs, the back's one),
    # the roll's trunnions steel pins under it over their 9 mm; the pitch's bracket takes the
    # pitch's torque, the column the waist's and a shoulder's, the girdle a shoulder's over its
    # half, the neck's bracket the neck's.
    couple = peak['spine'] / 0.132
    for frame, (runs, _rings) in skeleton.TRUNK.items():
        for k, pts in enumerate(runs):
            r = skeleton.TRUNK_R[frame][min(k, len(skeleton.TRUNK_R[frame]) - 1)]
            length = sum(math.dist(p, q) for p, q in zip(pts, pts[1:]))
            mat = STEEL if frame == 'spine_roll' else HM_CARBON
            i, j, area = tube(r) if frame != 'spine_roll' else (
                math.pi * r ** 4 / 4.0, math.pi * r ** 4 / 2.0, math.pi * r * r)
            if frame == 'pelvis':
                length = math.dist(pts[0], pts[-1])     # a cantilever's arm to its bearing
            load = {'pelvis': couple * (0.5 if k < 2 else 1.0) * length,
                    'spine_roll': couple * length, 'spine': peak['spine'],
                    'torso': peak['waist'] + peak['shoulder'] if k == 0 else peak['shoulder'],
                    'neck': peak['neck']}[frame]
            if frame == 'torso' and k == 1:
                length /= 2.0
            out.append(('%s frame %d' % (frame, k), 'bend', length, load, (i, j, area), mat, r))
    # The boom twisted by a hip's pitch; the hip's gimbal: the steerer twisted by the yaw, the
    # legs bent by the pitch's couple on the roll's bearings, the arms by the roll's.
    r, pts = skeleton.BOOM
    out.append(('boom', 'twist', math.dist(pts[0], pts[1]), peak['hip'], tube(r), HM_CARBON, r))
    tubes, _rings = skeleton.gimbal('hip_yaw')
    for k, (a, b, r) in enumerate(tubes):
        mode, load_nm = ('twist', peak['hip_yaw']) if k == 0 else (
            'bend', peak['hip'] * 0.5 if r >= 0.01 else peak['hip_yaw'])
        if k and r <= skeleton.CROWN_R:
            # The crown's plate, drawn as tubes of its half thickness.
            i = plate(2.0 * r, CROWN_W)
            out.append(('hip fork %d' % k, mode, math.dist(a, b), load_nm,
                        (i, i, 2.0 * r * CROWN_W), HM_CARBON, CROWN_W / 2.0))
            continue
        out.append(('hip fork %d' % k, mode, math.dist(a, b), load_nm, tube(r), CARBON, r))
    # The cradle's trunnions: steel pins under the pitch's couple on the roll's bearings 60 mm
    # out, bent over their length.
    tubes, _rings = skeleton.gimbal('hip_roll')
    for k, (a, b, r) in enumerate(tubes):
        length = math.dist(a, b)
        out.append(('hip cradle %d' % k, 'bend', length, peak['hip'] / 0.12 * length,
                    (math.pi * r ** 4 / 4.0, math.pi * r ** 4 / 2.0, math.pi * r * r), STEEL, r))
    # The holders' struts, printed solids: a drum's torque as a couple over the strut's lever
    # from the axis.
    for name, _rides, shape in skeleton.held('left_'):
        if shape[0] != 'tube':
            continue
        kind = drives.kind(name.replace('left_', '').rstrip('+'))
        _t, a, b, r = shape
        arm = max(0.02, math.hypot(a[1], a[2]))
        out.append((name.replace('left_', '') + ' strut', 'pull', math.dist(a, b),
                    peak.get(kind, 0.0) / arm,
                    (math.pi * r ** 4 / 4.0, 0.0, math.pi * r * r), PAHT_CF, r))
    # The four-bars' cranks and horns, plates in their plane; their rods.
    for kind, (a, b, r_c, _t0, horn, _b0, _z, _s) in linkage.PLANAR.items():
        pull = peak[kind] / horn
        for part, arm in (('crank', r_c), ('horn', horn)):
            i = plate(2.0 * linkage.PLATE_R, PLATE_W)
            out.append(('%s %s' % (kind, part), 'bend', arm, pull * arm,
                        (i, i, 2.0 * linkage.PLATE_R * PLATE_W), PAHT_CF, PLATE_W / 2.0))
        length = math.dist(linkage.planar(kind, 0.0)[0], linkage.planar(kind, 0.0)[1])
        out.append(('%s rod' % kind, 'pull', length, pull, tube(linkage.ROD_R), ROD, 0.0))
    for kind, (geo, _stroke) in linkage.RODS.items():
        length, _x, crank, _t, horn, _b = geo
        pull = peak[kind] / drives.motors('left_' + kind) / min(crank, horn)
        out.append(('%s rod' % kind, 'pull', length, pull,
                    (math.pi * linkage.ROD_R ** 4 / 4.0, 0.0, math.pi * linkage.ROD_R ** 2),
                    ROD, 0.0))
    return out


def judge(name, kind, length, load, section, material, c):
    """(stress ratio, flex deg): bending sigma = M c / I, twist tau = T c / J, a pull its
    axial stress and Euler's buckling as the ratio; flex the angle over the length at FLEX_AT."""
    i, j, area = section
    e, g, allow = material[0] * 1e9, material[1] * 1e9, material[2] * 1e6
    if kind == 'bend':
        return load * c / i / allow, math.degrees(FLEX_AT * load * length / (e * i))
    if kind == 'twist':
        return load * c / j / (allow * 0.6), math.degrees(FLEX_AT * load * length / (g * j))
    euler = math.pi ** 2 * e * i / max(length, 0.02) ** 2
    return max(load / area / allow, load / euler), 0.0


def compliance(name, kind, length, load, section, material, c):
    """rad a N m of torque at the joint this member winds up: bending L/(E I), twist L/(G J),
    a rod's stretch over its horn, L/(E A h^2) - the horn PATH's joint's (`linkage`)."""
    from machine import linkage
    i, j, area = section
    e, g, _allow = material[0] * 1e9, material[1] * 1e9, material[2]
    if kind == 'bend':
        return length / (e * i)
    if kind == 'twist':
        return length / (g * j)
    horn = min(linkage.RODS[name.split()[0]][0][2:5:2]) if name.split()[0] in linkage.RODS \
        else linkage.PLANAR.get(name.split()[0], (0, 0, 0, 0, 0.02))[4]
    return length / (e * area * horn * horn)


def wind(rows):
    """{joint kind: rad a N m} the structure on its path winds up (PATH), its members' in
    series."""
    by = {m[0]: compliance(*m) for m, _j in rows}
    return {kind: sum(by.get(n, 0.0) for n in names) for kind, names in PATH.items()}


def tubes(rows):
    """The members that are carbon tubes (a round section, carbon), each (row, judged)."""
    return [(m, j) for m, j in rows if m[5] in (CARBON, HM_CARBON)
            and math.isclose(m[4][1], 2.0 * m[4][0])]


def _passes(row, r, t):
    name, kind, length, load, _s, material, _c = row
    stress, flex = judge(name, kind, length, load, _tube(r, t), material, r)
    return stress <= 1.0 and flex <= FLEX_DEG


def spans(rows):
    """[(name, length, (first, last) of TUBES passing it, or None)] a tube member: the stock sizes
    passing `judge` no more than ROOM over its radius now - a bigger one passes too."""
    out = []
    for row, _j in tubes(rows):
        ok = [k for k, (r, t) in enumerate(TUBES) if r <= row[6] + ROOM and _passes(row, r, t)]
        out.append((row[0], row[2], (min(ok), max(ok)) if ok else None))
    return out


def families(served, most=6):
    """[(TUBES' indices, kg)] the lightest family of 1..`most` stock sizes cutting every member
    of `served` [(name, length, (first, last))] - the smallest of the family within its span -,
    None where none can."""
    import itertools
    out = []
    for n in range(1, most + 1):
        best = None
        for family in itertools.combinations(range(len(TUBES)), n):
            kg = 0.0
            for _name, length, (first, last) in served:
                k = next((k for k in family if first <= k <= last), None)
                if k is None:
                    break
                kg += CF_KG_M3 * _tube(*TUBES[k])[2] * length
            else:
                best = (family, kg) if best is None or kg < best[1] else best
        out.append(best)
    return out


def main(argv=None):
    import argparse
    from machine import drives
    from machine.physics import SERVO
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--family', type=int, default=0, metavar='N',
                        help='the tubes cut from the lightest family of N stock sizes')
    args = parser.parse_args(argv)
    rows = [(m, judge(*m)) for m in members()]
    if args.family:
        served = [s for s in spans(rows) if s[2] is not None]
        for name, _length, span in spans(rows):
            if span is None:
                print('%-20s no stock size within %.0f mm of its radius' % (name, 1e3 * ROOM))
        now = sum(CF_KG_M3 * m[4][2] * m[2] for m, _j in tubes(rows))
        print('%d tube members, %.2f kg as they are; the lightest family a count of sizes:'
              % (len(served), now))
        found = families(served, args.family)
        for n, best in enumerate(found, 1):
            print('  %d  %-32s %s' % (n, ' '.join('%gx%g' % (2e3 * r, 1e3 * t) for r, t in (
                TUBES[k] for k in best[0])) if best else '-', '%.2f kg' % best[1] if best else ''))
        if found[-1]:
            for name, _length, (first, last) in served:
                k = next(k for k in found[-1][0] if first <= k <= last)
                print('  %-20s %gx%g' % (name, 2e3 * TUBES[k][0], 1e3 * TUBES[k][1]))
        return 0
    print('%-22s %-6s %5s %7s %9s | %6s %6s' % ('member', 'load', 'mm', 'N m / N', 'section',
                                                 'stress', 'flex'))
    for (name, kind, length, load, section, material, c), (stress, flex) in rows:
        bad = stress > 1.0 or flex > FLEX_DEG
        print('%s %-20s %-6s %5.0f %7.0f %9s | %6.2f %6.2f%s' % (
            '*' if bad else ' ', name, kind, 1e3 * length, load,
            'r %.0f' % (1e3 * c) if kind != 'pull' else 'rod', stress, flex,
            ' deg' if kind != 'pull' else ''))
    print('%d of %d past a limit (stress 1, flex %.2f deg at %.1f of the peak)' % (
        sum(s > 1.0 or f > FLEX_DEG for _m, (s, f) in rows), len(rows), FLEX_DEG, FLEX_AT))
    print('wind-up a joint, its structure then its gearbox, mrad a N m, and deg at its clamp:')
    for kind, w in wind(rows).items():
        box = 1.0 / drives.BOX_K[drives.STACKS[kind][1]]
        clamp = min(drives.peak(kind if kind in ('spine', 'spine_roll', 'waist', 'neck', 'head')
                                else 'left_' + kind), SERVO[kind][0])
        print('  %-11s %6.3f + %5.3f mrad/N m  %5.2f deg at %3.0f N m' % (
            kind, 1e3 * w, 1e3 * box, math.degrees((w + box) * clamp), clamp))
    return 0


if __name__ == '__main__':
    sys.exit(main())
