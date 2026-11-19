"""A leg's printed pieces, each turned onto its bed face and laid out on the printer's plates.

    COAXIAL_ROBOT=gynoid_mesomorph_medium_coax python -X utf8 tools/render/prints.py [OUT.png]

PIECES names each printed piece of one leg - its hip, its knee, its shank and foot -: the sketch's parts it is made
of, the way it grows off its bed in that sketch's frame (None: the way its section is roundest about, its wider
end down; a tube's foot or head up its tube, its flange down) and how many a leg takes; the seats between a clamp
and a tube's foot are pieces of their own (INKS). `laid` turns a piece onto its bed; `plates` packs their
footprints - the least rectangle round each, AIR_M apart, the deepest first - on plates the printer's size (PLATE_M
square and TALL_M high, its front left corner CORNER_M kept free); `fitted` tells what the suite holds: every piece
within a plate and under its height. `cut` lists the leg's carbon tubes, cut to length, not printed. The other
leg's are these mirrored. A piece is what the sketches draw of it - a drawing; `solids` fills it.
"""
import argparse
import functools
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import chain  # noqa: E402
from tools.render import covered as cv  # noqa: E402
from tools.render import elbow_mount as em  # noqa: E402
from tools.render import mounting as m  # noqa: E402

#: The printer's plate: its side and the height over it, the corner at its front left kept free (across, deep) -
#: its nozzle's wiper's -, the air between two pieces and off the plate's edge, m.
PLATE_M, TALL_M, CORNER_M, AIR_M = 0.256, 0.256, (0.018, 0.028), 0.005
#: The inks printed: the frame's, the guards' and their first layers'; the bed's.
PRINTED, BED = (em.FRAME, cv.BLACK, chain.ACCENT), (70, 74, 84)
X, Y, Z, IN = (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (-1.0, 0.0, 0.0)
#: (name, the sketch it is drawn in, its parts there, the way it grows off its bed or None, how many a leg takes).
#: A rod's three prints lie in the rod's own frame: back from its line, up it, out across it.
PIECES = (
    ('yaw band', 'pelvis', ('yaw band',), Y, 1), ('yaw guard', 'pelvis', ('yaw guard',), None, 1),
    ('yaw lid', 'pelvis', ('yaw lid',), None, 1), ('yoke', 'yaw', ('yoke',), Y, 1),
    ('block', 'yaw', ('block',), (0.0, 0.0, -1.0), 1),
    ('roll guard', 'yaw', ('roll guard',), None, 1), ('roll lid', 'yaw', ('roll lid',), None, 1),
    ('cradle', 'roll', ('cradle band', 'saddle'), X, 1), ('pitch guard', 'roll', ('pitch guard',), None, 1),
    ('pitch lid', 'roll', ('pitch lid',), None, 1), ('thigh arm', 'thigh', ('neck',), IN, 1),
    ('thigh knuckle', 'thigh', ('knuckle',), None, 1), ('thigh foot', 'still', ('thigh tube',), None, 1),
    ('thigh shim', 'still', ('thigh seat',), None, 1), ('stator clamp', 'still', ('stator band', 'saddle'), X, 1),
    ('knee guard', 'still', ('board cover',), None, 1), ('knee lid', 'still', ('board lid',), None, 1),
    ('multi-clamp', 'turning', ('output band', 'web', 'pitch band', 'cage', 'shank seat'), IN, 1),
    ('shank shim', 'turning', ('shank seat',), None, 1), ('collar', 'turning', ('collar',), IN, 1),
    ('shank head', 'turning', ('shank tube',), None, 1), ('shank foot', 'shank', ('fork flange',), None, 1),
    ('fork shim', 'shank', ('fork',), None, 1), ('fork', 'shank', ('fork',), (0.0, -1.0, 0.0), 1),
    ('pitch lever', 'shank', ('lever ankle',), X, 1), ('roll lever', 'shank', ('lever ankle_roll',), IN, 1),
    ('pitch cover', 'shank', ('cover ankle',), IN, 1), ('roll cover', 'shank', ('cover ankle_roll',), X, 1),
    ('pitch leaf head', 'rods', ('head ankle',), IN, 1), ('roll leaf head', 'rods', ('head ankle_roll',), IN, 1),
    ('pitch leaf', 'rods', ('leaf ankle',), IN, 1), ('roll leaf', 'rods', ('leaf ankle_roll',), IN, 1),
    ('pitch leaf stem', 'rods', ('stem ankle',), Z, 1), ('roll leaf stem', 'rods', ('stem ankle_roll',), Z, 1),
    ('foot half', 'shank', ('foot half',), X, 2))
#: The pieces told by other inks than PRINTED: the seats' own, and the clamps' without their seats.
INKS = {'thigh shim': (em.PLATE,), 'shank shim': (em.PLATE,), 'fork shim': (em.PLATE,)}
#: The tubes' feet and heads, each grown up its tube: (the sketch, the part its tube is drawn in).
ON_TUBE = {'thigh knuckle': ('thigh', 'thigh tube'), 'thigh foot': ('thigh', 'thigh tube'),
           'shank head': ('shank', 'shank tube'), 'shank foot': ('shank', 'shank tube')}


@functools.lru_cache(maxsize=None)
def _drawn():
    """{sketch: {part: tris}}: the leg's sketches' parts - the knee's two sides, the shank's (each lever and each
    drive's cover apart, the foot's one half), each rod's three prints in the rod's own frame, the hip's four
    links."""
    import numpy as np
    from tools.render import hip_clamped as hc
    from tools.render import knee_clamped as kc
    from tools.render import leaves
    from tools.render import leg_clamped as lc
    _p, _c, still, turning = kc.bodies(ahead=lc.stated())
    shank, rods = dict(lc.shank()), {}
    for kind in leaves.KINDS:
        band, lug, faces = lc.lever(kind)
        shank['lever ' + kind] = band + lug
        way = math.copysign(1.0, faces[1])
        shank['cover ' + kind] = [(tri, c) for tri, c in lc.covers() if way * tri[:, 0].mean() > 0.0]
        at, axes, _span = leaves.frame(kind)
        for word, part in zip(('leaf ', 'stem ', 'head '), leaves.laid(kind, 0.0, 0.0, faces)):
            rods[word + kind] = [((tri - at) @ axes.T, c) for tri, c in part]
    shank['foot half'] = [(np.maximum(tri, (0.0, -np.inf, -np.inf)), c) for tri, c in shank['foot']]
    return dict(hc.links(), still=still, turning=turning, shank=shank, rods=rods)


def _way(points, way=None):
    """The way a piece grows off its bed, a unit vector, its wider end down: along `way` where that is given, else
    of the three axes its own spread names the one its section across is roundest about - a guard's or a lid's
    face, a seat's."""
    import numpy as np
    centre = points.mean(axis=0)
    if way is None:
        _w, axes = np.linalg.eigh(np.cov((points - centre).T))
        size = np.ptp((points - centre) @ axes, axis=0)
        round_ = [min(size[i - 1], size[i - 2]) / max(size[i - 1], size[i - 2]) for i in range(3)]
        way = axes[:, int(np.argmax(round_))]
    along = (points - centre) @ way
    fifth = 0.2 * np.ptp(along)
    wide = [np.linalg.norm((points - centre - np.outer(along, way))[pick], axis=1).max(initial=0.0)
            for pick in (along < along.min() + fifth, along > along.max() - fifth)]
    return way if wide[0] >= wide[1] else -way


def _tube(name):
    """(a tube's middle, its way, its points): the carbon tube a piece of ON_TUBE stands on, or one of `cut`'s."""
    import numpy as np
    sketch, part = ON_TUBE[name]
    points = np.concatenate([tri for tri, c in _drawn()[sketch][part] if tuple(c) == em.CARBON])
    middle = points.mean(axis=0)
    return middle, np.linalg.svd(points - middle)[2][0], points


def cut():
    """[(name, its diameter, its length, m)]: the leg's carbon tubes, cut to length - not printed."""
    import numpy as np
    out = []
    for name, foot in (('thigh tube', 'thigh foot'), ('shank tube', 'shank foot')):
        middle, way, points = _tube(foot)
        along = (points - middle) @ way
        out.append((name, 2.0 * float(np.linalg.norm(points - middle - np.outer(along, way), axis=1).max()),
                    float(np.ptp(along))))
    return out


@functools.lru_cache(maxsize=None)
def laid(name):
    """(tris, (across, deep, tall) m): a piece on its bed - the plane y = 0, the piece grown up it -, its footprint
    the least rectangle round it, its sides along x and z, a corner at their zero."""
    import numpy as np
    _name, sketch, parts, way, _count = next(p for p in PIECES if p[0] == name)
    inks = INKS.get(name, PRINTED)
    tris = [(tri, c) for part in parts for tri, c in _drawn()[sketch][part] if tuple(c) in inks]
    points = np.concatenate([tri for tri, _c in tris])
    up = np.asarray(way, float) if way else _way(points, _tube(name)[1] if name in ON_TUBE else None)
    side = np.cross(up, X if abs(up[0]) < 0.9 else Y)
    side /= np.linalg.norm(side)
    turn = np.array([side, up, np.cross(side, up)])
    flat, best = points @ turn.T, (math.inf, turn, np.zeros(3))
    for deg in range(0, 90, 3):                            # the least rectangle round its footprint
        c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
        spin = np.array([[c, 0.0, -s], [0.0, 1.0, 0.0], [s, 0.0, c]])
        size = np.ptp(flat @ spin.T, axis=0)
        if size[0] * size[2] < best[0]:
            best = (size[0] * size[2], spin @ turn, size)
    _area, turn, size = best
    low = (points @ turn.T).min(axis=0)
    return [(tri @ turn.T - low, c) for tri, c in tris], (float(size[0]), float(size[2]), float(size[1]))


def plates():
    """[[(name, x, z, across, deep)]], a plate each: every piece of a leg (PIECES, as many as it takes), its longer
    side along a row, laid the deepest first in the first row of the first plate with room for it - rows from the
    plate's front, AIR_M apart and off its edges, clear of its front left corner."""
    todo = sorted(((sorted(laid(name)[1][:2], reverse=True), name) for name, _s, _p, _w, count in PIECES
                   for _k in range(count)), key=lambda piece: (-piece[0][1], -piece[0][0]))
    out, rows = [], []                                     # rows: a plate's [[z, deep, the next x]]
    for (a, d), name in todo:
        for plate, lines in zip(out, rows):
            for line in lines:
                x = max(line[2], CORNER_M[0] + AIR_M) if line[0] < CORNER_M[1] + AIR_M else line[2]
                if d <= line[1] + 1e-9 and x + a <= PLATE_M - AIR_M:
                    break
            else:
                z = lines[-1][0] + lines[-1][1] + AIR_M
                if z + d > PLATE_M - AIR_M:
                    continue                               # no row with room, no room for a row: the next plate
                lines.append([z, d, AIR_M])
                line, x = lines[-1], AIR_M
            plate.append((name, x, line[0], a, d))
            line[2] = x + a + AIR_M
            break
        else:
            x, z = CORNER_M[0] + AIR_M, AIR_M
            if x + a > PLATE_M - AIR_M:                    # too long beside the corner: its row behind it
                x, z = AIR_M, CORNER_M[1] + AIR_M
            out.append([(name, x, z, a, d)])
            rows.append([[z, d, x + a + AIR_M]])
    return out


def fitted():
    """{'pieces': how many a leg prints, 'plates': how many plates they take, 'over': [the pieces that lie past a
    plate's edges or its height]}."""
    got = plates()
    over = [name for name, _s, _p, _w, _c in PIECES if laid(name)[1][2] > TALL_M]
    over += [name for plate in got for name, x, z, a, d in plate if x + a > PLATE_M - AIR_M + 1e-9
             or z + d > PLATE_M - AIR_M + 1e-9]
    return {'pieces': sum(len(plate) for plate in got), 'plates': len(got), 'over': sorted(set(over))}


def drawn(plate):
    """[(tri, colour)]: a plate and the pieces on it (`plates`), each where it lies."""
    import numpy as np
    square = np.array([(0.0, 0.0), (PLATE_M, 0.0), (PLATE_M, PLATE_M), (0.0, PLATE_M)])
    out = em._rounded(square, -0.003, 0.0, BED, r=0.001, steps=1)
    for name, x, z, a, _d in plate:
        tris, (across, _deep, _tall) = laid(name)
        if abs(a - across) > 1e-9:                         # turned a quarter: its depth along the row
            tris = [(tri[:, (2, 1, 0)] * (1.0, 1.0, -1.0) + (0.0, 0.0, across), c) for tri, c in tris]
        out += [(tri + (x, 0.0, z), c) for tri, c in tris]
    return out


def main(argv=None):
    from PIL import Image, ImageDraw
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('out', nargs='?')
    args = parser.parse_args(argv)
    got = plates()
    for k, plate in enumerate(got):
        for name, x, z, a, d in plate:
            print('plate %d  %-16s %5.0f x %5.0f mm, %5.0f high, at %5.0f, %5.0f' % (
                k + 1, name, 1e3 * a, 1e3 * d, 1e3 * laid(name)[1][2], 1e3 * x, 1e3 * z))
    print(fitted())
    for name, wide, long in cut():
        print('cut, not printed: %-12s %.0f mm round, %.0f mm long' % (name, 1e3 * wide, 1e3 * long))
    if args.out:
        px, wide = 760, min(3, len(got))
        sheet = Image.new('RGB', (px * wide, (px + 30) * -(-len(got) // wide)), (18, 18, 18))
        pen = ImageDraw.Draw(sheet)
        for k, plate in enumerate(got):
            view = m.draw(drawn(plate), yaw=25.0, pitch=50.0, px=px, half=0.62 * PLATE_M,
                          centre=(0.5 * PLATE_M, 0.02, 0.5 * PLATE_M))
            at = (px * (k % wide), (px + 30) * (k // wide))
            sheet.paste(view, (at[0], at[1] + 30))
            pen.text((at[0] + 12, at[1] + 8), 'plate %d: %s' % (k + 1, ', '.join(name for name, *_at in plate)),
                     fill=(220, 220, 220))
        sheet.save(args.out)
        print(args.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
