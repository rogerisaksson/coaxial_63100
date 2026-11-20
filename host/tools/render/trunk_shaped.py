"""The trunk's prints shaped by their loads: the pelvis's yoke and the drives' levers, by `topo`.

    COAXIAL_ROBOT=gynoid_mesomorph_medium_coax python -X utf8 tools/render/trunk_shaped.py [OUT_DIR]

A print of `trunk_clamped` is the optimiser's room as drawn: its drawing filled to voxels on its bed (`_room`),
nothing added to it - material is only taken away. A print's case (`cases`) holds it where it is screwed on and
loads it with what the trunk bears at the margin, a case a load. The pelvis's yoke is held by its base on the web;
the rods pull its two pins with what holds the chest's measured bow and lean (`knee_loads.loads`, the swing no
less than the build's spine's peak) through the linkage at rest, at the travel's corner - leant 35 deg, bowed back
30 - and bowed 85 deg, each rod at most its drive's own torque at the margin on its crank (`trunk_laid.pulled`),
the cross pushing back on its eyes and turning them back about its own third axis - what the rods' pulls turn
the chest about it the cross passes on, so the base bears the chest's moment alone -; and on those eyes the
spine's thrust and twist. A drive's lever is held where its band grips the output ring and the rod pulls its pin
across the arm and along it, at the cross out on the pin. `shaped` runs a case once
(.cache/topo keeps it), `drawn` gives its solid's voxels' faces in the joint's frame - a solid that carries its
loads, with nothing told to stay lost: else the print stays as drawn -, `posed` the trunk with every such print in
its place, `told` what each kept; with OUT_DIR the solids are written as STL, on their beds, mm.
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
from tools.render import trunk_rods as tr  # noqa: E402

#: The prints shaped: {name: (its voxel, mm, the share of what is the optimiser's that it may keep - the yoke's
#: at 0.35 carried x0.56 at its best cut)}; how many cells a voxel's side is filled by, and how far round an axis
#: a seat's voxels are kept, mm.
SHAPED, FINE, ROUND_MM = {'yoke': (4.0, 0.6), 'lever': (2.0, 0.6)}, 3, 4.0


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


def _loader(turn, corner):
    """A load in a print's room - its patch, force, moment and the point it acts at, the link's frame, mm - as `topo`
    takes it, in the bed's frame."""
    import numpy as np

    def load(patch, force, moment, about):
        return (patch, turn @ np.asarray(force, float), turn @ np.asarray(moment, float),
                turn @ np.asarray(about, float) - corner)
    return load


def _yoke(name, h, share):
    """The pelvis's yoke's case: held by its base's foot on the web, loaded on its pins and its eyes."""
    import numpy as np
    spine = kl.loads('spine')
    (_r0, r1), (p0, p1) = tl.travel()
    bow, lean, none = 1e-3 * spine['swing'], 1e-3 * spine['lateral'], (0.0, 0.0, 0.0)
    room, turn, corner, (x, y, z) = _room(name)
    load, ball = _loader(turn, corner), 1e3 * np.array(tl.BALL)
    held = ((y <= y[room].min() + 2.0 * h) & (np.abs(x) <= 1e3 * tc.BASE_X)          # the base's foot on the web
            & (z >= 1e3 * max(tc.BASE_Z[0], tc.WEB_Z[0])) & (z <= 1e3 * min(tc.BASE_Z[1], tc.WEB_Z[1])))
    eyes = (np.hypot(y, z) <= 1e3 * tc.PIN_R + ROUND_MM) & (np.abs(x) >= 1e3 * tc.ARM_X)
    boss = ((np.hypot(y - ball[1], z - ball[2]) <= 1e3 * tr.PIN_R[0] + ROUND_MM)
            & (np.abs(x) >= 1e3 * (tl.BALL[0] - tr.LEAN_M - tr.SEAT_M)))
    sides, pins = (boss & (x > 0.0), boss & (x < 0.0)), (ball, ball * (-1.0, 1.0, 1.0))

    def holding(pose, moment):
        pulls = tl.pulled(pose[0], pose[1], moment)
        assert pulls is not None, 'every pose of the travel is reached (trunk_laid.reached)'
        way = tl.turn(*pose)[0] @ np.array((0.0, 1.0, 0.0))         # the cross's own y: the twist it passes on
        twist = sum(np.cross(pin, force) for pin, force in zip(pins, pulls)) @ way * way
        return ([load(side, force, none, pin) for side, force, pin in zip(sides, pulls, pins)]
                + [load(eyes, -sum(pulls), -twist, none)])
    loads = [holding((0.0, 0.0), (bow, 0.0)), holding((0.0, 0.0), (0.0, lean)), holding((r1, p0), (bow, 0.0)),
             holding((r1, p0), (0.0, lean)), holding((0.0, p1), (bow, 0.0)),
             [load(eyes, (0.0, -spine['axial'], 0.0), (0.0, spine['torsion'], 0.0), none)]]
    return topo.case('trunk ' + name, room, held | eyes | boss, held, loads, h, share)


def _lever(name, h, share):
    """A drive's lever's case: held at its band's bore round the output ring, its band and its eye kept, the rod's
    most pull (`trunk_rods.worst`) on the pin's middle out at the cross, across the arm and along it."""
    import numpy as np
    room, turn, corner, (_x, y, z) = _room(name)
    lay, load = tr._laid(), _loader(turn, corner)
    at, pin = 1e3 * lay['at'], 1e3 * lay['pin']
    hub, round_pin = np.hypot(y - at[0], z - at[1]), np.hypot(y - pin[0], z - pin[1])
    out = (pin - at) / np.linalg.norm(pin - at)
    about, bore = np.array((1e3 * tl.HUB[0], pin[0], pin[1])), round_pin <= 1e3 * tr.PIN_R[1] + ROUND_MM
    loads = [[load(bore, tr.worst() * np.array(way), (0.0, 0.0, 0.0), about)]
             for way in ((0.0, out[0], out[1]), (0.0, -out[1], out[0]))]
    kept = (hub <= 1e3 * lay['rim'] + ROUND_MM) | (round_pin <= 1e3 * (tr.PIN_R[1] + tr.EYE_M))
    return topo.case('trunk ' + name, room, kept, hub <= 1e3 * lay['ring'] + ROUND_MM, loads, h, share)


def cases():
    """{name: its case for `topo`}: the room, what must stay, what holds it and its loads - in its bed's frame."""
    made = {'yoke': _yoke, 'lever': _lever}
    return {name: made[name](name, h, share) for name, (h, share) in SHAPED.items()}


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
    """`trunk_clamped.posed` with every print shaped so far drawn as its solid - a lever on either drive, turned
    where its rod holds it."""
    import math
    from tools.render import clamped as cl
    out = tc.posed(roll, pitch)
    turns = dict(zip(('cross', 'chest'), tl.turn(roll, pitch)))
    for name in SHAPED:
        tris, link = drawn(name), tc.PRINTS[name][0]
        if tris is None:
            continue
        if link == 'left crank':
            got, rest = tl.cranked(roll, pitch), math.radians(tl.CRANK[1])
            assert got is not None, 'every pose of the travel is reached (trunk_laid.reached)'
            for side, (bearing, _pin, _ball) in zip(('left', 'right'), got):
                own = tris if side == 'left' else tc._mirrored(tris)
                out[(side + ' crank', name)] = hf._put(cl.tilted(own, bearing - rest, tl.HUB[1:]), turns['chest'])
        else:
            out[(link, name)] = hf._put(tris, turns[link]) if link in turns else tris
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
