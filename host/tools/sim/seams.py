#!/usr/bin/env python3
"""Her shell past her clothes, standing: where a seam would catch.

    python tools/sim/seams.py

Each covered part of her drawn shell (`coaxial.graphics.gynoid`, bare) against the clothes over
it (COVERED, a side's its own): the worst reach of its corners past a cloth's rings
(`fit._excess`), m - a seam catches where a cloth is tighter than the shell beneath; a corner no
cloth covers does not count. The cheeks stood 19 mm out of the jeans between the seat and the
thighs' legs (2026-10-04).
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from coaxial.model.blocks import numpy as np  # noqa: E402
from tools.sim.fit import _excess, _rings  # noqa: E402

#: Her shell's parts and the clothes over each (`gynoid._wear`).
COVERED = {'pelvis': ('seat',), 'torso': ('tee', 'bust'), 'upper_arm': ('sleeve',),
           'thigh': ('thigh', 'seat'), 'shank': ('shin', 'leg'), 'bust': ('bust', 'tee'),
           'cheek': ('seat', 'thigh'), 'inner': ('inner', 'thigh')}


def _sided(name):
    """(side or None, the rest) of a part's name."""
    side, _dot, rest = name.partition('_')
    return (side, rest) if side in ('left', 'right') else (None, name)


def seams():
    """{shell part: m}: each covered part's worst reach past the clothes over it, standing."""
    from coaxial.graphics import gynoid
    from machine import gait
    stand = gait.stand()
    bare, dressed = gynoid.body(dressed=False), gynoid.body(dressed=True)
    worn = [(_sided(part[0][len('cloth_'):]), rings, turn, spot)
            for part, frame in zip(dressed.parts, dressed._frames(stand))
            if frame is not None and part[0].startswith('cloth_')
            for rings in [_rings(part[5])] if rings is not None for turn, spot in [frame]]
    out = {}
    for part, frame in zip(bare.parts, bare._frames(stand)):
        side, name = _sided(part[0])
        if frame is None or name not in COVERED:
            continue
        turn, spot = frame
        points = np.asarray(part[5][0]) @ turn.T + spot
        best = np.full(len(points), np.inf)
        for (side_c, kind), rings, turn_c, spot_c in worn:
            if kind in COVERED[name] and side_c in (None, side):
                best = np.minimum(best, _excess((points - spot_c) @ turn_c, rings))
        if np.isfinite(best).any():
            out[name] = max(out.get(name, -1.0), float(best[np.isfinite(best)].max()))
    return out


def main():
    print('her shell past her clothes standing, mm: ' + ', '.join(
        '%s %+.0f' % (p, v * 1e3) for p, v in sorted(seams().items(), key=lambda kv: -kv[1])))
    return 0


if __name__ == '__main__':
    sys.exit(main())
