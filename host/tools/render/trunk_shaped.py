"""The trunk's heaviest prints shaped by their loads: its column, its tower and its fork, by `topo`.

    COAXIAL_ROBOT=gynoid_mesomorph_medium_coax python -X utf8 tools/render/trunk_shaped.py [OUT_DIR]

A print of `trunk_clamped` is the optimiser's room as drawn: its drawing filled to voxels on its bed (`_room`),
nothing added to it - material is only taken away. A print's case (`cases`) holds it where it is screwed on and
loads it with what the trunk bears at the margin, a case a load: the column held by its foot on the web, the spine's
measured swing, lateral, torsion and thrust (`knee_loads.loads`) on its cupped head; the tower held where its
saddle lies on the cage, the joint's push either way as both rods pull or push their most, its twist and a push
across on the journal; the fork held by its two eyes on the pitch pin, the rods' pull on the chest's two pins -
at rest together and against each other, bowed to the travel's end, leant and bowed back -, each time with the
moment the tube's foot then bears on the seat, the pin a hinge that bears none, and the tube's thrust and twist
on that seat. `shaped` runs a case once (.cache/topo keeps it), `drawn` gives its solid's voxels' faces in the
joint's frame - a solid that carries its loads, with nothing told to stay lost: else the print stays as drawn -,
`posed` the trunk with every such print in its place, `told` what each kept; with OUT_DIR the solids are written
as STL, on their beds, mm.
"""
import os
import sys
import types
from typing import Any

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import elbow_mount as em  # noqa: E402
from tools.render import hip_frame as hf  # noqa: E402
from tools.render import knee_loads as kl  # noqa: E402
from tools.render import knee_mount as km  # noqa: E402
from tools.render import knee_topo as kt  # noqa: E402
from tools.render import topo  # noqa: E402
from tools.render import trunk_clamped as tc  # noqa: E402
from tools.render import trunk_laid as tl  # noqa: E402

#: The prints shaped: {name: (its voxel, mm, the share of what is the optimiser's that it may keep)}; how many
#: cells a voxel's side is filled by, and how far round an axis a seat's voxels are kept, mm.
SHAPED, FINE, ROUND_MM = {'column': (3.0, 0.35), 'tower': (2.5, 0.4), 'fork': (3.0, 0.35)}, 3, 4.0


def _room(name):
    """(room [across, up, deep], turn, corner, (x, y, z)): a print's drawing filled to voxels on its bed - the
    frame's rows across, up the way it grows and deep, its low corner there, mm - and its voxels' middles in the
    joint's frame, mm."""
    import numpy as np
    from scipy import ndimage
    (link, way), h = tc.PRINTS[name], SHAPED[name][0]
    up = np.asarray(way, float)
    across = np.cross(up, (1.0, 0.0, 0.0) if abs(up[0]) < 0.9 else (0.0, 1.0, 0.0))
    turn = np.array((across / np.linalg.norm(across), up, np.cross(across / np.linalg.norm(across), up)))
    cell = h / FINE
    drawing = [(tri, c) for tri, c in tc.links()[link][name] if tuple(c) == em.FRAME]
    points = 1e3 * km._points(drawing, 1e-3 * cell / 3.0) @ turn.T
    corner = points.min(axis=0)
    at = np.floor((points - corner) / cell).astype(int) + FINE
    faces = np.zeros(FINE * (-(-(at.max(axis=0) + 1 + FINE) // FINE)), bool)
    faces[tuple(at.T)] = True
    labelled: Any = ndimage.label(~faces)
    flood = labelled[0]
    solid = flood != flood[0, 0, 0]
    n = np.array(solid.shape) // FINE
    room = solid.reshape(n[0], FINE, n[1], FINE, n[2], FINE).mean(axis=(1, 3, 5))[1:-1, 1:-1, 1:-1] >= 0.5
    middles = (corner + (np.indices(room.shape).transpose(1, 2, 3, 0) + 0.5) * h) @ turn
    return room, turn, corner, (middles[..., 0], middles[..., 1], middles[..., 2])


def _pulls(roll, pitch):
    """[(the force, N, the left rod pulls the chest's pin with at the margin, the chest's frame), (the right's)] at
    a pose, deg: each rod bearing the most it does there (`trunk_laid.able`)."""
    import numpy as np
    from machine.body import structural
    chest = tl.turn(roll, pitch)[1]
    most = structural.MARGIN * (tl.able(roll, pitch) or {'pull': 0.0})['pull']
    return [chest.T @ (pin - ball) / np.linalg.norm(pin - ball) * most for _b, pin, ball in tl.cranked(roll, pitch) or ()]


def cases():
    """{name: its case for `topo`}: the room, what must stay, what holds it and its loads - in its bed's frame."""
    import numpy as np
    from machine.body import structural
    spine = kl.loads('spine')
    hy, hz = (1e3 * v for v in tl.HUB[1:])
    cage = 1e3 * tc.kc.laid()['cage'][3] + 1.0
    pull = structural.MARGIN * max(v['pull'] for v in tl.reached().values() if v)
    (r0, r1), (p0, p1) = tl.travel()
    out, none = {}, (0.0, 0.0, 0.0)
    for name, (h, share) in SHAPED.items():
        room, turn, corner, (x, y, z) = _room(name)

        def load(patch, force, moment, about, turn=turn, corner=corner):
            return (patch, turn @ np.asarray(force, float), turn @ np.asarray(moment, float),
                    turn @ np.asarray(about, float) - corner)
        on_cage = np.hypot(y - hy, z - hz) <= cage + 2.0 * h
        if name == 'column':
            held = y <= y[room].min() + 2.0 * h
            kept, loads = held | on_cage, [
                [load(on_cage, none, (spine['swing'], 0.0, 0.0), none)], [load(on_cage, none, (0.0, 0.0, spine['lateral']), none)],
                [load(on_cage, none, (0.0, spine['torsion'], 0.0), none)], [load(on_cage, (0.0, -spine['axial'], 0.0), none, none)]]
        elif name == 'tower':
            held, bore = on_cage, np.hypot(x, y) <= 1e3 * tc.JOURNAL_R + ROUND_MM
            kept, loads = bore, [
                [load(bore, (0.0, -2.0 * pull - spine['axial'], 0.0), none, none)], [load(bore, (0.0, 2.0 * pull, 0.0), none, none)],
                [load(bore, none, (0.0, spine['torsion'], 0.0), none)], [load(bore, (spine['axial'], 0.0, 0.0), none, none)]]
        else:
            ball = 1e3 * np.array(tl.BALL)
            held = (np.hypot(y, z) <= 1e3 * tc.PIN_R + ROUND_MM) & (np.abs(x) >= 1e3 * tc.ARM_X)
            seat = y >= 1e3 * tc.BRIDGE[1] - 2.0 * h
            boss = (np.hypot(y - ball[1], z - ball[2]) <= 1e3 * tc.ROD_PIN + ROUND_MM) & (np.abs(x) > 1e3 * tc.PIN_X)
            kept, sides = held | seat | boss, (boss & (x > 0.0), boss & (x < 0.0))
            pins = (ball, ball * (-1.0, 1.0, 1.0))

            def rods(pose, ways):                      # the rods' pulls, and on the seat the moment that meets them
                pulls = [way * force for way, force in zip(ways, _pulls(*pose))]
                met = -sum(np.cross(pin, force) for pin, force in zip(pins, pulls))
                return [load(side, force, none, pin) for side, force, pin in zip(sides, pulls, pins)] + [
                    load(seat, none, met, none)]
            loads = [rods((0.0, 0.0), (1.0, 1.0)), rods((0.0, 0.0), (1.0, -1.0)), rods((0.0, p1), (1.0, 1.0)),
                     rods((r1, p0), (1.0, -1.0)),
                     [load(seat, (0.0, -spine['axial'], 0.0), (0.0, spine['torsion'], 0.0), (0.0, 1e3 * tc.BRIDGE[1], 0.0))]]
        out[name] = topo.case('trunk ' + name, room, kept, held, loads, h, share)
    return out


def shaped(name, log=None):
    """A print shaped (`topo.shaped`): run once, then from .cache/topo."""
    return topo.shaped(cases()[name], log)


def drawn(name):
    """[(tri, colour)] of a shaped print's voxels' faces, the joint's frame, m - None where the print is not
    shaped yet, or its solid does not carry its loads or lost a voxel told to stay."""
    case = cases()[name]
    if not topo.ready(case):
        return None
    _room_, turn, corner, _at = _room(name)
    got = topo.shaped(case)
    if got['margin'] < 1.0 or got['lost']:
        return None
    return [(1e-3 * (tri + corner) @ turn, em.FRAME) for tri in kt.faces(got['points'], got['cells'])]


def posed(roll=0.0, pitch=0.0):
    """`trunk_clamped.posed` with every print shaped so far drawn as its solid."""
    out = tc.posed(roll, pitch)
    chest = tl.turn(roll, pitch)[1]
    for name in SHAPED:
        tris, link = drawn(name), tc.PRINTS[name][0]
        if tris is not None:
            out[(link, name)] = hf._put(tris, chest) if link == 'chest' else tris
    return out


def told(log=None):
    """{name: {'room', 'grams': g before and after, 'margin', 'level', 'lost': kept voxels that hung, 'hung':
    voxels of the solid with none of the nine under them}} of each print shaped."""
    out = {}
    for name in SHAPED:
        got = shaped(name, log)
        layers = got['solid'].transpose(1, 0, 2)
        out[name] = {'room': round(float(got['room']), 1), 'grams': round(float(got['grams']), 1),
                     'margin': round(float(got['margin']), 2), 'level': float(got['level']), 'lost': int(got['lost']),
                     'hung': int((layers & ~topo.stood(layers)).sum())}
    return out


def main(argv=None):
    import numpy as np
    out = (sys.argv[1:] if argv is None else argv)[:1]
    for name, got in told(log=print).items():
        print('%-7s %6.1f g of %6.1f g at %.1f, margin x%.2f, %d hung, %d kept voxels lost' % (
            name, got['grams'], got['room'], got['level'], got['margin'], got['hung'], got['lost']))
        if out:
            solid = shaped(name)
            os.makedirs(out[0], exist_ok=True)
            kt.stl(os.path.join(out[0], 'trunk_%s.stl' % name), types.SimpleNamespace(p=solid['points'], t=solid['cells']),
                   np.ones(solid['cells'].shape[1], bool))
    return 0


if __name__ == '__main__':
    sys.exit(main())
