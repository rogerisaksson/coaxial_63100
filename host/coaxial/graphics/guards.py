"""The robot's guards: honeycomb foam in rims, stood off its tubes on struts - a rider's protectors.

Inside each limb is a carbon tube a bone's girth (`machine.body.skeleton`'s runs); outside it,
where a rider wears protectors, a guard: a slab of foam, honeycomb on its face and under it -
a hexagon a cell, a groove between two -, a rim round it riveted at its corners, laid where
the limb's surface is (`surfaces`: a body's beside the figure, else the robot's own shell's),
clear of the drums and the tubes by it, and held off the tube on two struts. Air shows
between a guard and
its tube and between a guard and the slats under it, and no guard closes its limb: a
shoulder's cap and one down each arm, one on a thigh's outside and one over a knee and a
shin; the trunk wears a cup a side, a sternum's and a back's over its shoulder blades, and
its hips, its seat and the small of its back none - open all the way.

    parts = guards.laid(surfaces)     # [(name, parent, offset, mesh)]: 'guard_' and its segment
                                      # surfaces {segment: (its shell's corners, the mechanism's by it)}
"""
import math

from coaxial.graphics.livery import BOLT, FOAM, FRAME, PAD, STEEL, VOID
from coaxial.graphics.panels import LIMBS
from coaxial.graphics.shapes import joined, strut
from machine.body import skeleton

#: A surface is told about its axis in rows ROW_M along it and SECTORS round it: a body's
#: corners stand 12-14 mm apart. Where the mechanism stands by it - a drum, a tube - the
#: surface is no nearer than AIR_M past that and a guard's foam: told of the body's corners and
#: the mechanism's as one, a place the body had no corner in was a tube's, and a guard sank to
#: its limb's axis there. The surface falls off a drum in a skirt, SKIRT times a place no lower
#: than its four neighbours' mean: over a step a pad's backing cut through its cells.
ROW_M, SECTORS, SKIRT, AIR_M = 0.015, 24, 3, 0.006
#: A pad's cells PITCH_M from middle to middle, a hexagon FILL of that across its flats, and
#: between its face's cells and those under it a backing no dot of which is drawn
#: (`livery.VOID`): seen through its grooves to the tube behind, a pad's dots were all lit and
#: it was a grey slab. A pad's honeycomb is fitted to its window - whole cells across it and
#: down it, a cell's flats its column's sides -: on a lattice of its own pitch, its cells a
#: twelfth of a turn off it, a pad kept half its window's cells and wore a black margin. A
#: guard's outline is a box of the power POWER, AROUND corners, and what fills it is laid in
#: RINGS rings on the surface: a fan from its middle cut over a thigh's hollows and hid its
#: cells.
PITCH_M, FILL, PROUD_M, AROUND, RINGS, POWER = 0.026, 0.8, 0.003, 32, 6, 6.0
#: A guard is a slab: its face SKIN_M over the surface, THICK_M of foam under it. The rim is
#: BAND_M wide and no more than BAND of its lesser side, HEADER_M at the head of one that bears
#: a name (`lettering`), a rivet RIVET_R round in each of its corners, CORNER of the way out to
#: them. Slots through a pad took its cells: of a thigh's a handful were left, two dark eyes
#: in it.
SKIN_M, THICK_M, BAND_M, BAND, HEADER_M, RIVET_R, CORNER = 0.002, 0.012, 0.012, 0.2, 0.03, 0.0045, 0.78
#: A guard's two struts STRUT_R round, to STRUTS of its half width either side of its middle.
STRUT_R, STRUTS = 0.004, 0.45
#: A limb's guards by the end of its segment's name: (its name, from and to down the limb,
#: shares of its length, its middle about the limb, deg from its outer side toward ahead, its
#: half width, deg, its kind: 'pad' a honeycomb pad under a header, 'cap' one with no header,
#: 'slat' a bar, 'plate' one in the rim's gunmetal - a name's plate). None closes its limb, and a leg's inside and its back wear none: a rider's
#: shoulder cup and elbow guard, a pad on a thigh's outside, a knee cup and a shin guard,
#: slats under them.
LIMB = {'upper_arm': (('cap', -0.06, 0.32, 0.0, 85.0, 'cap'), ('guard', 0.42, 0.9, 10.0, 70.0, 'pad')),
        'forearm': (('guard', 0.08, 0.66, 20.0, 75.0, 'pad'), ('slat', 0.7, 0.84, 20.0, 60.0, 'slat')),
        'thigh': (('guard', 0.14, 0.62, 40.0, 72.0, 'pad'), ('slat', 0.655, 0.735, 40.0, 62.0, 'slat'),
                  ('slat', 0.77, 0.85, 40.0, 52.0, 'slat')),
        'shank': (('knee', -0.07, 0.11, 90.0, 55.0, 'cap'), ('guard', 0.16, 0.72, 90.0, 66.0, 'pad'),
                  ('slat', 0.755, 0.835, 90.0, 52.0, 'slat'))}
#: The trunk's guards, its frame: (its name, from and to up it, m, its middle about its upright,
#: deg from its left toward ahead, its half width, deg, its kind) - a cup a side, a sternum's,
#: two slats down its belly, a back's over its shoulder blades and a plate under that - its
#: name's, under the hair, seen from behind as the page looks: a header on the back's guard lay
#: under the hair. None under its ribs behind, none on its hips: open all the way.
TRUNK = (('left_cup', 0.152, 0.262, 62.0, 26.0, 'cap'), ('right_cup', 0.152, 0.262, 118.0, 26.0, 'cap'),
         ('sternum', 0.27, 0.35, 90.0, 13.0, 'cap'), ('belly', 0.1, 0.13, 90.0, 34.0, 'slat'),
         ('belly', 0.06, 0.09, 90.0, 29.0, 'slat'), ('back', 0.215, 0.37, -90.0, 52.0, 'cap'),
         ('spine', 0.155, 0.195, -90.0, 30.0, 'plate'))
_FAN = ((0, 1, 2), (0, 2, 3), (0, 3, 5), (3, 4, 5))


def _reach(corners, x, hard):
    """`at(y, about)`: how far a shell's `corners` (its part's frame, its axis its y) stand from
    that axis at the heights `y` and the ways round `about`, rad from its side `x` (1 its
    left's +x, -1 its right's) toward ahead: its furthest in each row and sector, a cell with
    none its neighbours' round its row, a row with none its neighbours'; no nearer than the
    mechanism's corners `hard` there and AIR_M and a guard's foam past them; skirted (SKIRT)."""
    from coaxial.model.blocks import numpy as np
    low = min(float(corners[:, 1].min()), float(hard[:, 1].min(initial=np.inf)))
    ys = np.arange(low, max(float(corners[:, 1].max()), float(hard[:, 1].max(initial=-np.inf))) + ROW_M, ROW_M)

    def told(points):
        table = np.zeros((len(ys), SECTORS))
        turn = np.arctan2(points[:, 2], x * points[:, 0]) / (2.0 * math.pi) * SECTORS
        np.maximum.at(table, (np.rint((points[:, 1] - ys[0]) / ROW_M).astype(int), np.floor(turn).astype(int) % SECTORS),
                      np.hypot(points[:, 0], points[:, 2]))
        return table
    table, k = told(corners), np.arange(SECTORS)
    rows = np.flatnonzero(table.any(axis=1))
    for r in rows:
        table[r] = np.interp(k, k[table[r] > 0.0], table[r, table[r] > 0.0], period=SECTORS)
    table = np.stack([np.interp(np.arange(len(ys)), rows, table[rows, c]) for c in k], 1)
    by = told(hard)
    table = np.maximum(table, np.where(by > 0.0, by + AIR_M + THICK_M - SKIN_M, 0.0))
    for _ in range(SKIRT):
        tall = np.pad(table, ((1, 1), (0, 0)), 'edge')
        table = np.maximum(table, (np.roll(table, 1, 1) + np.roll(table, -1, 1) + tall[:-2] + tall[2:]) / 4.0)

    def at(yy, about):
        v, u = np.clip((yy - ys[0]) / ROW_M, 0.0, len(ys) - 1.001), (about / (2.0 * math.pi) * SECTORS - 0.5) % SECTORS
        i, j = v.astype(int), u.astype(int)
        low, high = (table[row, j] + (u - j) * (table[row, (j + 1) % SECTORS] - table[row, j]) for row in (i, i + 1))
        return low + (v - i) * (high - low)
    return at


def _guard(at, x, top, foot, mid, half, kind):
    """(mesh, [a strut's foot on it]): a guard on the surface `at` (`_reach`'s) from the height
    `top` down to `foot`, `half` deg either side of `mid` about the axis - a slab, a box its
    corners round: its wall, and on its face and under it a 'slat''s plate or its rim -
    HEADER_M at its head where its `kind` is 'pad', a rivet in each corner - and within that
    its pad: a backing and, on its face and under it, its cells, a hexagon each."""
    from coaxial.model.blocks import numpy as np
    am, ah, ym, yh = math.radians(mid), math.radians(half), (top + foot) / 2.0, (top - foot) / 2.0
    radius = float(at(np.array([ym]), np.array([am]))[0]) + SKIN_M
    low = SKIN_M - THICK_M
    t, k = np.arange(AROUND) * (2.0 * math.pi / AROUND), np.arange(AROUND)
    u, v = (np.sign(f(t)) * np.abs(f(t)) ** (2.0 / POWER) for f in (np.cos, np.sin))     # a box, its corners round

    def place(yy, about, off=SKIN_M):
        r = at(yy, about) + off
        return np.stack([x * r * np.cos(about), yy, r * np.sin(about)], 1)

    def hexes(cy, ca, tall, wide, off):
        """Hexagons about (`cy`, `ca`), a corner up: `tall` to it, `wide` to a corner aside, m."""
        turn = np.arange(6) * (math.pi / 3.0)
        corners = place((cy[:, None] + tall * np.cos(turn)).ravel(), (ca[:, None] + wide / radius * np.sin(turn)).ravel(), off)
        return corners, (6 * np.arange(len(cy))[:, None, None] + np.array(_FAN)[None]).reshape(-1, 3)

    def ringed(my, hy, ha, off, inner):
        """A band between two outlines of the box about (`my`, am): `hy`, `ha` and `inner`'s."""
        iy, jy, ja = inner
        corners = np.vstack([place(my + hy * v, am + ha * u, off), place(iy + jy * v, am + ja * u, off)])
        return corners, np.vstack([np.stack([k, k + AROUND, (k + 1) % AROUND], 1),
                                   np.stack([(k + 1) % AROUND, k + AROUND, (k + 1) % AROUND + AROUND], 1)])

    def filled(my, hy, ha, off):
        """The box about (`my`, am) filled, in RINGS rings on the surface."""
        s = np.arange(RINGS, 0, -1)[:, None] / RINGS
        corners = np.vstack([place((my + hy * s * v).ravel(), (am + ha * s * u).ravel(), off), place(np.array([my]), np.array([am]), off)])
        rows = [np.vstack([np.stack([k, k + AROUND, (k + 1) % AROUND], 1), np.stack([(k + 1) % AROUND, k + AROUND, (k + 1) % AROUND + AROUND], 1)])
                + i * AROUND for i in range(RINGS - 1)]
        return corners, np.vstack(rows + [np.stack([np.full(AROUND, RINGS * AROUND), k + (RINGS - 1) * AROUND,
                                                    (k + 1) % AROUND + (RINGS - 1) * AROUND], 1)])
    band = min(BAND_M, BAND * 2.0 * min(yh, ah * radius))
    head = HEADER_M if kind == 'pad' else band
    im, ih, iw = ym + (band - head) / 2.0, yh - (band + head) / 2.0, ah - band / radius
    wall = np.vstack([place(ym + yh * v, am + ah * u), place(ym + yh * v, am + ah * u, low)])
    out = [(wall, np.vstack([np.stack([k, k + AROUND, (k + 1) % AROUND], 1), np.stack([(k + 1) % AROUND, k + AROUND, (k + 1) % AROUND + AROUND], 1)]), FRAME)]
    if kind in ('slat', 'plate'):
        out += [(*filled(ym, yh, ah, off), ink) for off, ink in ((SKIN_M, PAD if kind == 'slat' else FRAME), (low, FOAM))]
        ry, ra = np.full(2, ym), am + (ah - yh / radius) * np.array([-1.0, 1.0])
    else:
        out += [(*ringed(ym, yh, ah, off, (im, ih, iw)), FRAME) for off in (SKIN_M, low)]
        out.append((*filled(im, ih, iw, (SKIN_M + low) / 2.0), VOID))
        # The pad: a honeycomb fitted to the rim's window - `across` whole cells in its middle
        # row, a row off it half a cell aside, `down` rows -, those within its round corners.
        across = max(1, int(2.0 * iw * radius / PITCH_M + 0.5))
        tall, rise = 2.0 * FILL / math.sqrt(3.0) + 1.0 - FILL, math.sqrt(3.0) / 2.0     # a cell with a groove, a row: of a pitch
        rise = rise if across > 1 else tall
        down = max(1, int((2.0 * ih / PITCH_M - tall) / rise + 1.5))
        pa, pv = 2.0 * iw * radius / across, 2.0 * ih / ((down - 1) * rise + tall)
        counts = [across - (i - down // 2) % 2 * (across > 1) for i in range(down)]
        cy = np.concatenate([np.full(n, im + (i - (down - 1) / 2.0) * rise * pv) for i, n in enumerate(counts)])
        ca = np.concatenate([am + (np.arange(n) - (n - 1) / 2.0) * pa / radius for n in counts])
        turn, ry, ra = np.arange(6) * (math.pi / 3.0), pv * FILL / math.sqrt(3.0), pa * FILL / math.sqrt(3.0)
        hy, ha = cy[:, None] + ry * np.cos(turn), ca[:, None] + ra / radius * np.sin(turn)
        keep = (np.abs((hy - im) / ih) ** POWER + np.abs((ha - am) / iw) ** POWER < 1.0).all(axis=1)
        out += [(*hexes(cy[keep], ca[keep], ry, ra, off), ink) for off, ink in ((SKIN_M, PAD), (low, FOAM))]
        ry = ym + CORNER * (yh - band / 2.0) * np.array([1.0, 1.0, -1.0, -1.0])
        ra = am + CORNER * (ah - band / 2.0 / radius) * np.array([-1.0, 1.0, -1.0, 1.0])
    out.append((*hexes(ry, ra, RIVET_R, RIVET_R, SKIN_M + PROUD_M), BOLT))
    base, cs, ts, ms = 0, [], [], []
    for corners, triangles, ink in out:
        cs.append(corners)
        ts.append(triangles + base)
        ms.append(np.full(len(corners), ink))
        base += len(corners)
    corners = np.vstack(cs)
    return ((corners, np.vstack(ts), np.zeros((len(corners), 2)), np.concatenate(ms)),
            place(np.full(2, ym), am + STRUTS * ah * np.array([-1.0, 1.0]), low))


def _strutted(feet, tubes):
    """[mesh]: a strut from each of `feet` (n, 3) to the nearest point of the `tubes` [(a, b,
    radius)], its frame."""
    from coaxial.model.blocks import numpy as np
    out = []
    for foot in feet:
        near = []
        for a, b, _r in tubes:
            a, b = np.asarray(a, float), np.asarray(b, float)
            on = a + (b - a) * float(np.clip((foot - a) @ (b - a) / ((b - a) @ (b - a)), 0.0, 1.0))
            near.append((float(np.linalg.norm(foot - on)), on))
        on = min(near, key=lambda pair: pair[0])[1]
        way = (foot - on) / np.linalg.norm(foot - on)
        if way[2] < -0.999:                    # `shapes.strut` lays none straight back
            on = on + (0.002, 0.0, 0.0)
        out.append(strut(on, foot, STRUT_R, STEEL))
    return out


def laid(surfaces):
    """[(name, parent, offset, mesh)]: the robot's guards, a part ('guard_' and its segment) a
    segment that wears any (LIMB, TRUNK), rigid on it: each guard's slab on the segment's
    surface - `surfaces` {segment: (its shell's corners, the mechanism's by it), its frame} -
    and its two struts to the segment's nearest tube (`skeleton.runs`, the trunk's frame's)."""
    made, tubes = {}, {}
    for seg, (corners, hard) in surfaces.items():
        side, _, limb = seg.partition('_')
        x = -1.0 if side == 'right' else 1.0
        if limb in LIMB:
            at, tubes[seg] = _reach(corners, x, hard), skeleton.runs(limb, x)
            made[seg] = [_guard(at, x, -a * LIMBS[limb][0], -b * LIMBS[limb][0], mid, half, kind)
                         for _name, a, b, mid, half, kind in LIMB[limb]]
        elif seg == 'torso':
            at = _reach(corners, 1.0, hard)
            tubes[seg] = [shape[1:] for frame, shape in skeleton.trunk() if frame == seg and shape[0] == 'tube'] or [
                ((0.0, 0.0, 0.0), (0.0, 0.38, 0.0), 0.0)]
            made[seg] = [_guard(at, 1.0, high, low, mid, half, kind) for _name, low, high, mid, half, kind in TRUNK]
    return [('guard_' + seg, seg, (0.0, 0.0, 0.0), joined(
        [mesh for mesh, _feet in guards] + [s for _mesh, feet in guards for s in _strutted(feet, tubes[seg])]))
        for seg, guards in made.items()]
