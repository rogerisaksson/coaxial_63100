#!/usr/bin/env python3
"""What holds each of her drives, standing.

A bone, a gimbal, the boom or the trunk's frame touching its drum, or its own collars and struts
between (`machine.skeleton.HELD`); a drum held by nothing floats (the user, 2026-10-03: the hip's
roll drum, the ankles', the toes').

    python tools/sim/held.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from coaxial.model.blocks import numpy as np  # noqa: E402
from tools.sim import fit  # noqa: E402


def held_by(touch=0.001):
    """{joint: [parts]}: standing, her left's and her trunk's drums and the structure holding
    each - a bone, a gimbal, the boom, the trunk's frame, a bone's collar - touching it or through
    its own collars and struts, within `touch`; never a rod, a drum, a board, nor a wire through a
    segment's middle (a straight bone where `skeleton.HUNG` has none)."""
    from machine import gait, skeleton
    parts_, _rides = fit.parts(gait.stand())

    def structure(name):
        root = name[len('bone_'):].rstrip('+').split('>')[0]
        return (name.startswith(('gimbal_', 'collar_', 'frame_', 'bone_pelvis>pelvis'))
                or name.startswith('bone_') and root.split('_', 1)[-1] in skeleton.HUNG)

    def dense(part):
        if part[0] == 'tube':
            return fit._points(part, 24)[0]
        _k, c, ax, r, half = part
        along = [c + t * half * ax for t in np.linspace(-1.0, 1.0, 15)]
        return np.concatenate([fit._points(('drum', at, ax, r, 0.0))[0] for at in along]
                              + [np.array(along)])
    out = {}
    for name, part in parts_.items():
        if not name.startswith('drive_') or name.startswith('drive_right_'):
            continue
        joint = name[len('drive_'):]
        # Its stack's parts one drum (`drives.along`): its gearbox's and its inverter's with it.
        stack = [parts_[k + joint] for k in ('drive_', 'gear_', 'inv_') if k + joint in parts_]
        pts = np.concatenate([dense(q) for q in stack])
        mine = [h for h in parts_ if h.startswith('held_' + joint) and h[len('held_' + joint):]
                .strip('+') == '']
        reach = [p for p in parts_ if structure(p) and fit._gap(pts, parts_[p]) <= touch]
        for h in mine:
            if any(fit._gap(dense(parts_[h]), q) <= touch for q in stack) or any(
                    fit._gap(dense(parts_[h]), parts_[g]) <= touch for g in mine if g != h):
                reach += [p for p in parts_ if structure(p)
                          and fit._gap(dense(parts_[h]), parts_[p]) <= touch]
        out[joint] = sorted({p.rstrip('+') for p in reach})
    return out


def main():
    for joint, by in held_by().items():
        print('%-18s %s' % (joint, ', '.join(by) if by else 'nothing'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
