#!/usr/bin/env python3
"""What she buys, counted: drives, tubes, rods and their ends, belts, gear pairs, pins, bearings.

    python tools/sim/bom.py

Each line a part type and its count, from the drives as stacked (`drives.STACKS`, the held and
sprung joints' left out), the tubes by stock size (`tools/sim/members.py`), the transmissions
(`machine.linkage`) and the gimbals' and the ankles' pins and bearings (`machine.skeleton`).
The last line counts the types: the fewest variants (the user, 2026-10-04).
"""
import collections
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))


def parts():
    """[(part, count)] of her purchased parts, the drives first."""
    from machine import drives, linkage, skeleton
    from machine.figure import JOINTS
    from tools.sim import members
    out = collections.Counter()
    driven = [j for j in JOINTS if not drives.passive(j)]
    for j in driven:
        frame, box, inverter = drives.STACKS[drives.kind(j)]
        out['motor %s KV %.0f' % (frame, drives.KV[frame])] += 1
        out['gearbox %s, 1:%.0f' % (box, drives.RATIO)] += 1
        out['inverter %d mm' % inverter] += 1
    rows = [(m, members.judge(*m)) for m in members.members()]
    for (name, _kind, length, *_rest, r), _j in members.tubes(rows):
        size = next(((rr, t) for rr, t in members.TUBES if math.isclose(rr, r)), None)
        sides = 1 if name.startswith(('pelvis', 'spine', 'torso', 'neck', 'boom')) else 2
        out['tube %gx%g mm' % ((2e3 * size[0], 1e3 * size[1]) if size else (2e3 * r, 0))
            ] += sides
        out['tube %gx%g mm, m' % ((2e3 * size[0], 1e3 * size[1]) if size else (2e3 * r, 0))
            ] += sides * length
    kinds = {drives.kind(j) for j in driven}
    for kind in linkage.RODS:
        if kind in kinds:
            out['rod %g mm, bent' % (2e3 * linkage.ROD_R)] += 2
            out['rod end'] += 4
            out['crank, printed'] += 2
    for kind in linkage.PLANAR:
        if kind in kinds:
            out['rod %g mm' % (2e3 * linkage.ROD_R)] += 1
            out['rod end'] += 2
            out['crank and horn, printed plates'] += 2
    for kind, (r1, r2, _off) in linkage.BELTS.items():
        if kind in kinds:
            out['belt, pulleys %g/%g mm' % (2e3 * r1, 2e3 * r2)] += 2
            out['pulley'] += 4
    for kind, r in linkage.BEVELS.items():
        if kind in kinds:
            out['bevel pair %g mm' % (2e3 * r)] += 2
    for kind, ratio in linkage.GEARS.items():
        if kind in kinds:
            out['spur pair 1:%.2f' % ratio] += 2
    r, half, _z = skeleton.BEARING
    out['bearing %gx%g mm, the hips\' roll' % (2e3 * r, 2e3 * half)] += 4
    out['pin %g mm, the hips\' trunnions' % (2e3 * (r - 0.002))] += 4
    out['pin %g mm, the ankles\' crosses' % (2e3 * skeleton.CROSS[1])] += 4
    out['pin %g mm, the trunk\'s roll' % (2e3 * skeleton.TRUNK_R['spine_roll'][0])] += 2
    return sorted(out.items())


def main():
    got = parts()
    for part, count in got:
        print('%5s  %s' % ('%.2f' % count if part.endswith(', m') else count, part))
    print('%d part types' % sum(1 for p, _c in got if not p.endswith(', m')))
    return 0


if __name__ == '__main__':
    sys.exit(main())
