"""The foot through its ankle's travel, its levers and leaves following: the air kept off the shank.

    COAXIAL_ROBOT=gynoid_mesomorph_medium_coax python -X utf8 tools/render/foot_swung.py [OUT.png] [--near]

`posed` lays what the ankle turns - the foot on its cross, its heel bar, each drive's lever and its leaf between
their balls (`leg_clamped.linked`) - at a pitch and a roll; `met` tells what of it then stands nearest what holds
still on the shank - the fork, the tube, the calf's drives and their clamps - and nearest the foot's own; `swung` is
the least over the travel's ends and what lies between them: what the suite holds (test_views_clothes).
"""
import argparse
import functools
import math
import os
import sys
from typing import Any

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import knee_clamped as kc  # noqa: E402
from tools.render import knee_mount as km  # noqa: E402
from tools.render import leaves  # noqa: E402
from tools.render import leg_clamped as lc  # noqa: E402
from tools.render import mounting as m  # noqa: E402

#: What the ankle carries, by how it turns: with the foot - the pitch and the roll -, with the cross - the pitch.
FOOT, CROSS = ('foot', 'plate', 'soles', 'heel bar'), ('cross',)
#: The rolls tried at each pitch, shares of the roll's travel's ends; the pitches between the travel's ends, deg
#: apart - the tool's and the suite's; a cloud's points' spacing and how near a pair is told, m.
ROLLS, STEP_DEG, HELD_DEG, STEP_M, NEAR_M = (-1.0, 0.0, 1.0), 5.0, 15.0, 0.0015, 0.012
#: What touches by design, left out: the cross in the fork's bosses and the foot's cheeks, a lever's band on its
#: drive's ring.
MATED = ('cross', 'bands')


def _turn(pitch, roll):
    """The ankle's turn, a matrix: the roll about the cross's own pin, then the pitch (`linkage.ball`), deg."""
    import numpy as np
    c, s = math.cos(math.radians(roll)), math.sin(math.radians(roll))
    return km._rx(pitch) @ np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


@functools.lru_cache(maxsize=None)
def _rest():
    """({name: tris} of the shank's side at rest (`leg_clamped.shank`), {name: points} of what the ankle carries
    there, a cloud each)."""
    low = lc.shank()
    return low, {name: km._points(low[name], STEP_M) for name in FOOT + CROSS}


def _carried(parts, pitch, roll, moved):
    """{name: `moved`(its part, the ankle's centre, its turn)} of what the ankle carries (`parts` by name) at
    `pitch` and `roll`: the foot's own turned with both, the cross with the pitch."""
    import numpy as np
    from machine.body.figure import SHANK
    at = np.array((0.0, -SHANK, 0.0))
    return {name: moved(parts[name], at, turn) for names, turn in ((FOOT, _turn(pitch, roll)), (CROSS, km._rx(pitch)))
            for name in names}


def posed(pitch=0.0, roll=0.0):
    """{name: tris} of what the ankle turns, the knee's frame, the ankle at `pitch` and `roll`, deg: the foot, its
    plate, soles and heel bar turned with it, the cross with its pitch, each lever and its leaf where the foot's
    balls then hold them ('bands', 'lugs', 'leaf KIND')."""
    out = _carried(_rest()[0], pitch, roll, lambda tris, at, turn: [((tri - at) @ turn.T + at, c) for tri, c in tris])
    return dict(out, **lc.linked(pitch, roll))


@functools.lru_cache(maxsize=None)
def _still():
    """{name: (points, their tree, their box)} of what holds still on the shank: its fork, its flange and tube, the
    knee's shank's side - a cloud each, STEP_M apart."""
    from scipy.spatial import KDTree
    low = lc.shank()
    _p, _c, _thigh, turning = kc.bodies(ahead=lc.stated())
    parts = dict({'knee ' + k: v for k, v in turning.items() if v},
                 **{k: low[k] for k in ('fork', 'fork flange', 'shank tube', 'crank covers')})
    clouds = {name: km._points(tris, STEP_M) for name, tris in parts.items()}
    return {name: (pts, KDTree(pts), (pts.min(axis=0) - NEAR_M, pts.max(axis=0) + NEAR_M))
            for name, pts in clouds.items()}


def _gap(tree, box, pts):
    """The least distance from `pts` to a cloud's `tree`, NEAR_M where none comes that near its `box`."""
    import numpy as np
    by = pts[np.all((pts > box[0]) & (pts < box[1]), axis=1)]
    return float(tree.query(by, distance_upper_bound=NEAR_M)[0].min()) if len(by) else NEAR_M


def met(pitch, roll, top=6, own=False):
    """[(air m, still part, turned part)], the `top` least and under NEAR_M: what of the foot's side (`posed`) stands
    nearest what holds still on the shank, the ankle at `pitch` and `roll`; with `own`, a leaf nearest the foot's
    own, the other leaf and the levers' lugs - not where it rides them: its eye on the heel's shaft, its fork round
    its lug's pin."""
    import numpy as np
    from scipy.spatial import KDTree
    moving = _carried(_rest()[1], pitch, roll, lambda pts, at, turn: (pts - at) @ turn.T + at)
    moving.update({k: km._points(v, STEP_M) for k, v in lc.linked(pitch, roll, leaves.stops()['push'][0]).items()})
    moving = {k: v for k, v in moving.items() if k not in MATED}
    out = []
    if not own:
        for name, (_pts, tree, box) in _still().items():
            out += [(_gap(tree, box, pts), name, k) for k, pts in moving.items()]
        return sorted(g for g in out if g[0] < NEAR_M)[:top]
    for kind in leaves.KINDS:
        leaf, pin = moving['leaf ' + kind], np.array(leaves.ends(kind, pitch, roll)[0])
        off = leaf[np.hypot(*(leaf[:, 1:] - pin[1:]).T) > leaves.FORK_R + 0.003]
        for name in FOOT[:-1] + ('lugs',) + (('leaf ankle_roll',) if kind == 'ankle' else ()):
            pts = moving[name]
            if name == 'lugs':
                pts = pts[np.hypot(*(pts[:, 1:] - pin[1:]).T) > leaves.FORK_R + 0.003]
            box = (pts.min(axis=0) - NEAR_M, pts.max(axis=0) + NEAR_M)
            out.append((_gap(KDTree(pts), box, off if name == 'lugs' else leaf), name, 'leaf ' + kind))
    return sorted(g for g in out if g[0] < NEAR_M)[:top]


def poses(step=STEP_DEG):
    """[(pitch, roll)], deg: the ankle's travel's ends and every `step` between, each at its roll's ends and
    middle."""
    import numpy as np
    from machine.body import drives
    (p0, p1), (r0, r1) = drives.TRAVEL['ankle'], drives.TRAVEL['ankle_roll']
    pitches = sorted({p0, p1} | {float(p) for p in np.arange(math.ceil(p0 / step) * step, p1, step)})
    return [(p, 0.5 * (r0 + r1) + 0.5 * (r1 - r0) * share) for p in pitches for share in ROLLS]


def swung(step=HELD_DEG):
    """{'shank': [air m, still part, turned part, [pitch, roll]], 'own': likewise, 'held': {check: what a rod's
    part may bear over what it bears}}: the least air the foot's side keeps through the ankle's travel (`poses`,
    `step` deg apart) off what holds still on the shank, and its rods - their leaves bowed as far as their stops
    let them - off the foot's own, each other and the levers' lugs; the rods' hand checks (`leaves.held`)."""
    out: Any = {'held': {k: round(v, 3) for k, v in leaves.held().items()}}
    for pitch, roll in poses(step):
        for key, own in (('shank', False), ('own', True)):
            got = met(pitch, roll, 1, own)
            if got and got[0][0] < out.get(key, [1.0])[0]:
                out[key] = [round(got[0][0], 5), got[0][1], got[0][2], [pitch, roll]]
    return out


def corners():
    """[[pitch, roll, shank_m, own_m]] at the four extreme ankle travel corners."""
    from machine.body import drives
    (p0, p1), (r0, r1) = drives.TRAVEL['ankle'], drives.TRAVEL['ankle_roll']
    return [[p, r, round((met(p, r, 1, False) or [(NEAR_M,)])[0][0], 5),
             round((met(p, r, 1, True) or [(NEAR_M,)])[0][0], 5)] for p in (p0, p1) for r in (r0, r1)]


def main(argv=None):
    from PIL import Image, ImageDraw
    from machine.body import drives
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('out', nargs='?')
    parser.add_argument('--near', action='store_true')
    args = parser.parse_args(argv)
    if args.near:
        for pitch, roll in poses():
            print('pitch %5.1f roll %5.1f' % (pitch, roll))
            for own in (False, True):
                print('   ' + ' | '.join('%.1f %s - %s' % (1e3 * g, a, b) for g, a, b in met(pitch, roll, 4, own)))
        return 0
    told: Any = swung(STEP_DEG)
    print('held', told.pop('held'), 'stops', leaves.stops(), 'worst %.0f N' % leaves.worst())
    for key, (gap, still, turned, at) in told.items():
        print('%-5s %.1f mm, %s - %s, at pitch %.0f roll %.0f' % (key, 1e3 * gap, still, turned, at[0], at[1]))
    if args.out:
        px, views, still = 700, [], [t for v in _still().values() for t in v]
        (p0, p1), roll = drives.TRAVEL['ankle'], drives.TRAVEL['ankle_roll'][1]
        for name, pitch, r in (('toes up %.0f deg' % -p0, p0, 0.0), ('at rest', 0.0, 0.0),
                               ('toes down %.0f deg' % p1, p1, 0.0), ('toes down, rolled %.0f deg' % roll, p1, roll)):
            parts = still + [t for v in posed(pitch, r).values() for t in v]
            centre, half = kc._framed([t for v in posed(0.0, 0.0).values() for t in v])
            for yaw in (90.0, -150.0):
                views.append((name, m.draw(parts, yaw=yaw, pitch=8.0, px=px, half=1.15 * half, centre=centre)))
        sheet = Image.new('RGB', (4 * px, 2 * (px + 30)), (18, 18, 18))
        pen = ImageDraw.Draw(sheet)
        for i, (name, img) in enumerate(views):
            x, y = px * (i // 2), (px + 30) * (i % 2)
            sheet.paste(img, (x, y + 30))
            pen.text((x + 12, y + 10), name, fill=(200, 200, 200))
        sheet.save(args.out)
        print(args.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
