"""A leg's printed pieces, each turned onto its bed face and laid out on the printer's plates.

    COAXIAL_ROBOT=gynoid_mesomorph_medium_coax python -X utf8 tools/render/prints.py [OUT.png]

PIECES names each printed piece of one leg - its hip, its knee, its shank and foot -: the sketch's parts it is made
of, the way it grows off its bed in that sketch's frame (None: the way its section is roundest about, its wider
end down; a tube's foot or head up its tube, its flange down) and how many a leg takes; the seats between a clamp
and a tube's foot are pieces of their own (INKS). `laid` turns a piece onto its bed; `plates` packs their
footprints - the least rectangle round each, AIR_M apart, the deepest first - on plates the printer's size (PLATE_M
square and TALL_M high, its front left corner CORNER_M kept free); `fitted` tells what the suite holds: every piece
within a plate and under its height. `cut` lists the leg's carbon tubes, cut to length, not printed. The other
leg's are these mirrored. A piece is what the sketches draw of it - a drawing; `solids` fills it. The trunk's
printed parts are pieces too (`trunk`: `trunk_clamped.PRINTS`, a first draft's), each 'trunk ' and its part's name.
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
    ('yaw band', 'pelvis', ('yaw band',), Y, 1), ('yaw guard', 'pelvis', ('yaw guard',), (0.0, -1.0, 0.0), 1),
    ('yaw lid', 'pelvis', ('yaw lid',), (0.0, -1.0, 0.0), 1), ('yoke', 'yaw', ('yoke',), Y, 1),
    ('block', 'yaw', ('block',), (0.0, 0.0, -1.0), 1),
    ('roll guard', 'yaw', ('roll guard',), Z, 1), ('roll lid', 'yaw', ('roll lid',), Z, 1),
    ('cradle', 'roll', ('cradle band', 'saddle'), X, 1), ('pitch guard', 'roll', ('pitch guard',), X, 1),
    ('pitch lid', 'roll', ('pitch lid',), X, 1), ('thigh arm', 'thigh', ('neck',), IN, 1),
    ('thigh knuckle', 'thigh', ('knuckle',), None, 1), ('thigh foot', 'still', ('thigh tube',), None, 1),
    ('thigh shim', 'still', ('thigh seat',), None, 1), ('stator clamp', 'still', ('stator band', 'saddle'), IN, 1),
    ('knee guard', 'still', ('board cover',), X, 1), ('knee lid', 'still', ('board lid',), X, 1),
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
INKS = {'thigh shim': (em.PLATE,), 'shank shim': (em.PLATE,), 'fork shim': (em.PLATE,),
        'upper bay puck': (em.FRAME,)}
#: The tubes' feet and heads, each grown up its tube: (the sketch, the part its tube is drawn in).
ON_TUBE = {'thigh knuckle': ('thigh', 'thigh tube'), 'thigh foot': ('thigh', 'thigh tube'),
           'shank head': ('shank', 'shank tube'), 'shank foot': ('shank', 'shank tube')}


def trunk():
    """The trunk's printed pieces, rows as PIECES': `trunk_clamped.PRINTS`, each grown the way stated there - one a
    side where it is drawn on the left."""
    from tools.render import trunk_clamped as tc
    return tuple(('trunk ' + name, 'trunk', (name,), way, 2 if link.startswith('left ') else 1)
                 for name, (link, way) in tc.PRINTS.items())


@functools.lru_cache(maxsize=None)
def _trunk():
    """{part: tris}: the trunk's printed parts as its sketch draws them."""
    from tools.render import trunk_clamped as tc
    return {name: tc.links()[link][name] for name, (link, _way) in tc.PRINTS.items()}


#: (name, way, count) of each printed PAHT-CF piece of the arms, shoulders, neck and torso bay (`upper`).
UPPER_PIECES = (
    ('elbow stator band', IN, 2), ('elbow stator saddle', (0.0, -1.0, 0.0), 2), ('elbow rotor clamp', IN, 2),
    ('elbow cover', X, 2), ('elbow lid', X, 2), ('elbow foot', Y, 2), ('forearm head', (0.0, -1.0, 0.0), 2),
    ('crossbar foot', X, 2), ('shoulder block', IN, 2), ('shoulder housing', X, 2), ('shoulder guard', X, 2),
    ('shoulder lid', X, 2), ('abduction band', (0.0, 0.0, -1.0), 2), ('abduction guard', Y, 2),
    ('abduction post', X, 2), ('abduction saddle', X, 2), ('torso block', X, 1), ('neck band', X, 1),
    ('spine foot', (0.0, -1.0, 0.0), 1), ('bay puck', Z, 1), ('bay clamp', Y, 1), ('bay guard', Z, 1),
    ('bay lid', Z, 1), ('neck cheek', X, 1), ('neck crown', Y, 1), ('neck shoe', X, 1))


def upper():
    """The arms', shoulders' and neck's printed pieces, rows as PIECES'."""
    return tuple(('upper ' + name, 'upper', (name,), way, count) for name, way, count in UPPER_PIECES)


@functools.lru_cache(maxsize=None)
def _upper():
    """{part: tris}: the upper body's printed parts as their sketches draw them."""
    from tools.render import boards, clamped as cl, knee_mount as km, neck_laid as nl, neck_mount as nm
    from tools.render import shoulder_laid as sl, shoulder_mount as sm, shoulder_rolled as sr, sized
    (_sp, (_g0, g1, ring), arm_in, arm_out, _can, motor, _brd, (narrow, reach), _split, (back, x1, band), cover,
     (_face, end, outside), over, (first, _rot), swing) = em._laid()
    fit, top, low = em._fitted(), over + em.SADDLE_M, over + em.ROTOR_AIR
    turn = cv.turned(fit[4])
    stator_b, stator_s = em._stator(motor, back, x1, band, end, outside, over, top, _rot, narrow, reach, swing)
    wall_r, waist_r = 1e-3 * sized.band(swing, 1e3 * ring, 1e3 * (g1 - first)), 1e-3 * sized.neck(1e3 * (reach + g1), 'elbow')
    rotor_c = em._band(ring, ring + wall_r, first, g1, em.SLIT)
    for a, b, r_in in [(first, g1, ring + wall_r - 0.001), (-reach, first, low)]:
        rotor_c += em._rounded(cl.filleted(cl.necked((0.0, 0.0), r_in, -(low + em.ROTOR_M), (-narrow, narrow), waist_r)),
                               a, b, em.FRAME, axis=0, r=0.002)
    lay_r, st_r, s_still, s_turn = sr.laid(), sr.stack(), sm.still(), sm.turning()
    sr_band = sr._along(cl.turned([(lay_r['house'][0] + 0.001, st_r['gear'][0], lay_r['band']),
                                   (lay_r['motor'][1] - 0.001, st_r['motor'][0], lay_r['band'])], 0.0), 'z')
    for deg, way in ((sr.TRAVEL[0], -1.0), (sr.TRAVEL[1], 1.0)):
        r0 = lay_r['rib'] + 0.5 * sr.HORN_M
        b = math.radians(deg) + way * sr.HORN_M / r0
        at = (-r0 * math.sin(b), -r0 * math.cos(b))
        sr_band += km._rod((at[0], at[1], lay_r['plate'][1] + sr.AIR_M), (at[0], at[1], lay_r['house'][0] + 0.002),
                           0.5 * sr.HORN_M, em.FRAME)
    top_b, bot_b = lay_r['board']
    r_b, (x0_h, x1_h) = st_r['board'][0], sr._hood_x()
    sr_guard = sr._along(em._drum(r_b + sr.GUARD_M, -bot_b - sr.GUARD_M, -bot_b, em.FRAME) + cl.turned(
        [(-bot_b - sr.GUARD_M, r_b + 0.0005, r_b + sr.GUARD_M), (-top_b + 0.001, r_b + 0.0005, r_b + sr.GUARD_M)], None), 'y')
    lay_n, n_still, n_nod = nl.laid(), nm.still(), nm.nodding()
    return {'elbow stator band': stator_b, 'elbow stator saddle': stator_s, 'elbow rotor clamp': rotor_c,
            'elbow cover': cv.cover(*cover, outlet=fit[4], window=boards.window(turn)),
            'elbow lid': cv.lid(cover[0], cover[2], cover[3], fit[3], fit[4]),
            'elbow foot': em._limb(top + em.SEAT_M, 1.0, arm_in, 0.0, 'torx', False)[1],
            'forearm head': em._limb(-(low + em.ROTOR_M + em.SEAT_M), -1.0, arm_out, 0.0, 'torx', False)[1],
            'crossbar foot': s_still['crossbar foot'], 'shoulder block': s_still['block'],
            'shoulder housing': s_turn['housing'], 'shoulder guard': s_turn['guard'], 'shoulder lid': s_turn['lid'],
            'abduction band': sr_band, 'abduction guard': sr_guard,
            'abduction post': sr._box((x0_h + 0.002, x1_h - 0.002), (-top_b, -sl.laid()['band'][2] + 0.001),
                                      (-0.008, 0.008), em.FRAME),
            'abduction saddle': sr._box((-0.5 * sr.WIDE_M, 0.5 * sr.WIDE_M),
                                        (-lay_r['drop'] - 0.5 * lay_r['saddle'], -lay_r['drop'] + 0.5 * lay_r['saddle']),
                                        (-lay_r['plate'][1], lay_r['plate'][1]), em.FRAME),
            'torso block': nm._crossbar_block(lay_n), 'neck band': nm._neck_clamp(lay_n),
            'spine foot': n_still['spine'], 'bay puck': n_still['bay'], 'bay clamp': n_still['bay clamp'],
            'bay guard': n_still['bay guard'], 'bay lid': n_still['bay lid'],
            'neck cheek': n_nod['cheek'], 'neck crown': n_nod['crown'], 'neck shoe': n_nod['shoe']}


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
def piece(name):
    """(tris, the way it grows off its bed): a piece as its sketch draws it, that sketch's frame, its printed inks."""
    import numpy as np
    rows = trunk() if name.startswith('trunk ') else (upper() if name.startswith('upper ') else PIECES)
    _name, sketch, parts, way, _count = next(p for p in rows if p[0] == name)
    inks = INKS.get(name, PRINTED)
    drawn = _trunk() if sketch == 'trunk' else (_upper() if sketch == 'upper' else _drawn()[sketch])
    tris = [(tri, c) for part in parts for tri, c in drawn[part] if tuple(c) in inks]
    points = np.concatenate([tri for tri, _c in tris])
    return tris, np.asarray(way, float) if way else _way(points, _tube(name)[1] if name in ON_TUBE else None)


@functools.lru_cache(maxsize=None)
def laid(name):
    """(tris, (across, deep, tall) m): a piece on its bed - the plane y = 0, the piece grown up it -, its footprint
    the least rectangle round it, its sides along x and z, a corner at their zero."""
    import numpy as np
    tris, up = piece(name)
    points = np.concatenate([tri for tri, _c in tris])
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


def plates(rows=PIECES):
    """[[(name, x, z, across, deep)]], a plate each: every piece of a leg (PIECES, as many as it takes) - or of
    `rows` -, its longer side along a row, laid the deepest first in the first row of the first plate with room for
    it - rows from the plate's front, AIR_M apart and off its edges, clear of its front left corner."""
    todo = sorted(((sorted(laid(name)[1][:2], reverse=True), name) for name, _s, _p, _w, count in rows
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


def fitted(rows=PIECES):
    """{'pieces': how many a leg - or `rows` - prints, 'plates': how many plates they take, 'over': [the pieces
    that lie past a plate's edges or its height]}."""
    got = plates(rows)
    over = [name for name, _s, _p, _w, _c in rows if laid(name)[1][2] > TALL_M]
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
