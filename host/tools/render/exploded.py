"""The leg drawn apart: each part slid off along the axis it goes on by - a drive's, a tube's, a rod's.

    COAXIAL_ROBOT=gynoid_mesomorph_medium_coax python -X utf8 tools/render/exploded.py OUT.png

The leg's sketches' parts (`leg_clamped`, `knee_clamped`, `hip_clamped`) in bundles, each with the axis its parts go
on by (`bundles`): the hip's three drives' - the yaw's upright, the roll's fore and aft, the pitch's across -, the
thigh's along its tube, the knee's and the calf's across along their drives', the shank's along its tube down to
the foot, each ankle rod's along its own line. A drive is told board, motor and gearbox apart, a part of two inks
by its inks - a clamp from its seat, a tube from its head. Within a bundle the parts keep their order along the
axis and move apart until no two overlap along it, GAP_M between them (`apart`); the bundles move apart along the
leg, the rods back behind the calf.
"""
import argparse
import functools
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import elbow_mount as em  # noqa: E402
from tools.render import knee_clamped as kc  # noqa: E402
from tools.render import leaves  # noqa: E402
from tools.render import leg_clamped as lc  # noqa: E402
from tools.render import mounting as m  # noqa: E402

#: The air left between two parts along their bundle's axis, how far apart the bundles move as a share of how far
#: each stands from the knee, how far back the rods are drawn, m.
GAP_M, LEG, BACK_M = 0.014, 0.55, 0.11
X, Y, Z = (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)
#: What goes with another bundle than its sketch's: the thigh's seat and foot on the knee's clamp, the knee's
#: output's on the calf's side, the shank's seat and head under the cage, the levers and covers on the drives.
THIGH, KNEE, SHANK, CALF = ('thigh seat', 'thigh tube'), ('output ring', 'output band', 'web'), (
    'shank seat', 'shank tube'), ('bands', 'lugs', 'crank covers')


def _inked(name, tris):
    """[(name, tris)]: a part by its inks where it is a printed piece and something else - a seat, a tube."""
    by = {}
    if not any(word in name for word in ('tube', 'seat', 'fork')):
        return [(name, tris)]
    for tri, c in tris:
        by.setdefault(tuple(c) if tuple(c) in (em.PLATE, em.CARBON) else None, []).append((tri, c))
    return [(name if ink is None else '%s %s' % (name, 'seat' if ink == em.PLATE else 'tube'), part)
            for ink, part in by.items()]


def _stacked(name, tris, stack):
    """[(name, tris)]: a drive told apart along x by its stack {part: (x0, x1, r)} - board, motor, gearbox."""
    import numpy as np
    out = {}
    for tri, c in tris:
        x = float(np.mean(tri[:, 0]))
        part = min(stack, key=lambda k: abs(x - 0.5 * (stack[k][0] + stack[k][1])))
        out.setdefault('%s %s' % (name, part), []).append((tri, c))
    return list(out.items())


@functools.lru_cache(maxsize=None)
def bundles():
    """[(name, its axis, [(part, tris)])]: the leg's parts, the knee's frame, the leg straight, in the bundles they
    go together in."""
    from tools.render import hip_clamped as hc
    _p, _c, still, turning = kc.bodies(ahead=lc.stated())
    lay, low = kc.laid(), lc.shank()
    (m0, m1, _can), (g0, g1, _ring), *_rest = kc._stack()
    knee = [(k, v) for k, v in still.items() if v and k not in THIGH + ('knee drive',)]
    knee += _stacked('knee drive', still['knee drive'], {'board': (m0 - 0.02, m0, 0.0), 'motor': (m0, m1, 0.0),
                                                          'gearbox': (g0, g1, 0.0)})
    knee += [(k, turning[k]) for k in KNEE]
    calf = [(k, v) for k, v in turning.items() if v and k not in KNEE + SHANK + ('pitch drive', 'roll drive')]
    for which in ('pitch', 'roll'):
        calf += _stacked(which + ' drive', turning[which + ' drive'], lay[which][0])
    calf += [(k, low[k]) for k in CALF]
    shank = [part for k in SHANK for part in _inked('knee ' + k, turning[k])]
    shank += [part for k, v in low.items() if not k.startswith('leaf ') and k not in CALF for part in _inked(k, v)]
    thigh = [part for k in THIGH for part in _inked('knee ' + k, still[k])]
    hip = {link: [part for k, v in lc._lifted(parts).items() for part in _inked(k, v)]
           for link, parts in hc.links().items()}
    out = [('hip yaw', Y, hip['pelvis']), ('hip roll', Z, hip['yaw']), ('hip pitch', X, hip['roll']),
           ('thigh', Y, hip['thigh'] + thigh), ('knee', X, knee), ('calf', X, calf), ('shank', Y, shank)]
    for kind in leaves.KINDS:
        _at, axes, _span = leaves.frame(kind)
        out.append(('rod ' + kind, tuple(axes[1]), list(zip(('leaf', 'stem', 'head'), leaves.laid(
            kind, 0.0, 0.0, lc.lever(kind)[2])))))
    return out


def apart():
    """[(tri, colour)]: the leg's parts drawn apart - each bundle's along its axis in their order, none overlapping
    another along it, the bundles along the leg from the knee, the rods back."""
    import numpy as np
    out = []
    for name, axis, parts in bundles():
        way = np.asarray(axis, float)
        spans = [(np.concatenate([tri for tri, _c in tris]) @ way, tris) for _part, tris in parts if tris]
        spans.sort(key=lambda span: float(span[0].mean()))
        at, moved = -np.inf, []
        for along, tris in spans:
            shift = max(0.0, at + GAP_M - float(along.min())) if np.isfinite(at) else 0.0
            at = float(along.max()) + shift
            moved.append((shift, tris))
        middle = 0.5 * moved[-1][0]                           # the bundle's own middle stays where it stood
        centre = np.concatenate([tri for _s, tris in moved for tri, _c in tris]).mean(axis=0)
        push = np.array((0.0, LEG * centre[1], -BACK_M if name.startswith('rod ') else 0.0))
        out += [(tri + (shift - middle) * way + push, c) for shift, tris in moved for tri, c in tris]
    return out


def main(argv=None):
    from PIL import Image, ImageChops, ImageDraw
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('out')
    args = parser.parse_args(argv)
    parts, px, views = apart(), 2200, []
    centre, half = kc._framed(parts)
    for yaw in (40.0, -140.0):                             # from ahead and outside, from behind: each cut to the leg
        view = m.draw(parts, yaw=yaw, pitch=12.0, px=px, half=half, centre=centre)
        box = ImageChops.difference(view, Image.new('RGB', view.size, view.getpixel((0, 0)))).getbbox()
        views.append(view.crop(box) if box else view)
    sheet = Image.new('RGB', (sum(v.width for v in views) + 120, max(v.height for v in views) + 70), (18, 18, 18))
    pen, x = ImageDraw.Draw(sheet), 40
    for view in views:
        sheet.paste(view, (x, 50))
        x += view.width + 40
    pen.text((12, 8), 'the leg apart, from ahead and from behind: %s' % ', '.join(
        '%s %d' % (b, len(p)) for b, _a, p in bundles()), fill=(220, 220, 220))
    sheet.save(args.out)
    print(args.out, sum(len(p) for _b, _a, p in bundles()), 'parts in', len(bundles()), 'bundles')
    return 0


if __name__ == '__main__':
    sys.exit(main())
