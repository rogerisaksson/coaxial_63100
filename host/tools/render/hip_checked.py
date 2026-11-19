"""The hip's sketch held to what is asked: its links' air through every pose, its loads, its hoses.

    COAXIAL_ROBOT=gynoid_mesomorph_medium_coax python -X utf8 tools/render/hip_checked.py [--clear]

`clear` is the least air between two of the hip's links (`hip_clamped.links`) through every pose the scenarios ask
(`hip_laid.POSED`, a cell more of roll and yaw each way) - minutes; `near` names the parts that meet at a pose;
`checked` is what the suite holds: the hand checks, the hoses' bends, the air at the poses' corners.
"""
import argparse
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import hip_clamped as hc  # noqa: E402
from tools.render import hip_laid as hl  # noqa: E402
from tools.render import hung  # noqa: E402
from tools.render import knee_mount as km  # noqa: E402
from tools.sim import legload  # noqa: E402

#: The least air kept between two links, m; a cloud's points' spacing and its cells, m.
CLEAR_M, STEP_M, CELL_M = 0.001, 0.0015, 0.006
#: What touches by design, left out of a pair's clouds: a gearbox's ring in its housing, the pin in its bearings,
#: the pinion in the sector.
MATED = ('yaw ring', 'pitch ring', 'kingpin', 'pinion')


def _cloud(parts, but=MATED):
    import numpy as np
    return np.concatenate([km._points(tris, STEP_M) for name, tris in parts.items() if tris and name not in but])


def _hashed(points):
    import numpy as np
    keys = km._key(np.floor(points / CELL_M).astype(np.int64))
    order = np.argsort(keys)
    return keys[order], points[order]


def clear(log=None, corners=False):
    """{(still link, turning link): (least air m - CELL_M where none nearer -, (yaw, roll, pitch) deg there)} over the
    poses the scenarios ask - from the deepest fold to the farthest swing back (`hip_laid.TURNED`, a degree more),
    a cell more of roll and yaw each way: the yaw's on the pelvis by its yaw, the cradle on the yaw's by its roll, the
    thigh on the cradle by its pitch, the thigh on the yaw's by its pitch and roll, the thigh on the pelvis by all
    three - only where its box comes within a cell of the pelvis's. With `corners`, the fold's and the swing's ends,
    the hip straight and where it turns out and in the most, alone, each at its roll's and its yaw's ends."""
    import numpy as np
    clouds = {name: _cloud(parts) for name, parts in hc.links().items()}
    hashed = {name: _hashed(points) for name, points in clouds.items()}
    pad = legload.CELL_DEG

    def rolls(cell):
        lo, hi = hl.POSED[cell][0] - pad, hl.POSED[cell][1] + pad
        return (lo, hi) if corners else range(lo, hi + pad, pad)

    def yaws(cell):
        lo, hi = hl.POSED[cell][2] - pad, hl.POSED[cell][3] + pad
        return (lo, hi) if corners else range(lo, hi + pad, pad)
    fold, back = hl.TURNED[0] - 1.0, hl.TURNED[1] + 1.0
    pitches = [fold] + [float(c) for c in sorted(hl.POSED) if fold < c < back] + [back]
    if corners:
        wide = max(hl.POSED, key=lambda c: hl.POSED[c][1]), min(hl.POSED, key=lambda c: hl.POSED[c][0])
        pitches = sorted({fold, back, 0.0} | {max(float(c), fold) for c in wide})
    out = {}

    def tried(pair, moving, matrix, pose):
        gap = km._least(*hashed[pair[0]], moving @ matrix.T, CELL_M)
        if gap < out.get(pair, (CELL_M + 1.0, None))[0]:
            out[pair] = (gap, pose)
    for y in sorted({y for c in hl.POSED for y in yaws(c)}):
        tried(('pelvis', 'yaw'), clouds['yaw'], hc.turn(y)[0], (y, 0, 0))
    for r in sorted({r for c in hl.POSED for r in rolls(c)}):
        tried(('yaw', 'roll'), clouds['roll'], hc.turn(0.0, r)[1], (0, r, 0))
    box = clouds['pelvis'].min(axis=0) - CELL_M, clouds['pelvis'].max(axis=0) + CELL_M
    for p in pitches:
        cell = min(hl.POSED, key=lambda c: abs(c - p))
        tried(('roll', 'thigh'), clouds['thigh'], hc.turn(0.0, 0.0, p)[2], (0, 0, p))
        for r in rolls(cell):
            tried(('yaw', 'thigh'), clouds['thigh'], hc.turn(0.0, r, p)[2], (0, r, p))
            for y in yaws(cell):
                moved = clouds['thigh'] @ hc.turn(y, r, p)[2].T
                close = moved[np.all((moved > box[0]) & (moved < box[1]), axis=1)]
                if len(close):
                    gap = km._least(*hashed['pelvis'], close, CELL_M)
                    if gap < out.get(('pelvis', 'thigh'), (CELL_M + 1.0, None))[0]:
                        out[('pelvis', 'thigh')] = (gap, (y, r, p))
        if log:
            log('pitch %4d: %s' % (p, '  '.join('%s-%s %.1f' % (a, b, 1e3 * g) for (a, b), (g, _at) in out.items())))
    return out


def checked():
    """{'held': {member: what it may bear over what it bears}, 'bends': {hose: [its least bend, the hose's own, m]},
    'clear': {pair: least air, m}, 'drawn': [the least radius any hose drawn bends at, the longest stretch one laid
    still lies free between holds, the longest that may be, m], 'fold': the least air between the thigh's neck and
    tube and the shank's side with the knee at its travel's end, m}: the hip's hand checks (`hip_laid.checks`), its
    two hoses hung through their travel, its links' air at the corners of the poses asked, its hoses as drawn at
    rest (`tees.tightest`; `tees.loosest` of its links alone, the two hung ones apart), its thigh in the leg
    (`leg_clamped.folded`)."""
    from machine.body import mechatronics
    from tools.render import leg_clamped, tees
    own = float(mechatronics.materials()['hoses']['bus'][4])
    links = [t for parts in hc.links().values() for tris in parts.values() for t in tris]
    return {'held': {k: round(may / got, 3) for k, (got, may) in hl.checks().items()},
            'bends': {name: [round(hung.least(span), 5), own] for name, (span, _laid) in hc.hoses().items()},
            'clear': {'%s-%s' % pair: round(gap, 5) for pair, (gap, _at) in clear(corners=True).items()},
            'drawn': [round(q, 5) for q in (tees.tightest(hc.posed()), tees.loosest(links), tees.unheld())],
            'fold': round(leg_clamped.folded()[0], 5)}


def near(still, turning, pose, top=4):
    """[(air m, still's part, turning's part)], the `top` least: which parts of two links come nearest at `pose`
    (yaw, roll, pitch) deg."""
    import numpy as np
    got = hc.links()
    turns = dict(zip(('yaw', 'roll', 'thigh'), hc.turn(*pose)))
    out = []
    for a, mine in got[still].items():
        if a in MATED or not mine:
            continue
        hashed = _hashed(km._points(mine, STEP_M) @ turns.get(still, np.eye(3)).T)
        for b, other in got[turning].items():
            if other and b not in MATED:
                out.append((km._least(*hashed, km._points(other, STEP_M) @ turns[turning].T, CELL_M), a, b))
    return sorted(out)[:top]


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--clear', action='store_true')
    args = parser.parse_args(argv)
    if args.clear:
        for pair, (gap, at) in clear(log=print).items():
            print('%-6s - %-6s %5.1f mm at yaw %d roll %d pitch %d' % (pair + (1e3 * gap,) + tuple(at)))
    else:
        for kind, told in checked().items():
            print(kind, told)
    return 0


if __name__ == '__main__':
    sys.exit(main())
