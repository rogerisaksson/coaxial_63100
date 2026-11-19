"""The robot's hoses on its page: hung in a MuJoCo world of their own, carried by its pose.

    props = hoses.layers(body, frames, t, travel, m, cam, centre, depth)     # [(dots, ink)], `lit.braille`'s

Where the build has hoses across its joints laid (`machine.world.hoses`, its `spans`), they are kept in one world a
drawing process: each frame its fittings' segments are carried from where they were to where `frames` (the robot's
segments' frames, `gynoid.Body._frames`) puts them now, in the time `t` (the robot's world's, s) went on since - so a
hose swings, sags and lags as its own weight and stiffness have it, at whatever rate the page draws - its links the
fewest that hold the robot's own world's step (`peripherals.world`), so a frame costs no more steps than the robot's.
A gap in `t` past GAP_S, or `t` going back (a seek, a robot laid anew), lays them anew where the robot stands. What a
hose pulls on the robot is left out, so nothing of the robot's own world changes. Drawn as its centre line and one
beside it either way, SHARE of its radius off, in the hoses' ink: where no more than `marks.PROUD_M` stands before
it, on bare ground too.
"""
from typing import Any

from coaxial.graphics.marks import _dots

#: The hoses' ink (the sketches' `tees.HOSE`); the gap in the page's time past which they are laid anew, s; how
#: far off its centre line a hose's side lines are drawn, a share of its radius.
INK, GAP_S, SHARE = (255, 48, 200), 0.5, 2.0 / 3.0

#: The world kept, the time it stands at, and the build's spans, each a drawing process's own.
_KEPT: dict = {}


def _np():
    import numpy
    return numpy


def layers(body, frames, t, travel, m, cam, centre, depth):
    """[(dots, ink)]: the build's hoses at the robot's world's time `t` (None: none drawn), its segments at `frames`
    drawn `travel` (x, z) off where they stand, through the view `m` about `centre` by the camera `cam`."""
    if t is None:
        return []
    from machine.body import mechatronics, peripherals
    from machine.world import hoses
    np = _np()
    if 'spans' not in _KEPT:
        _KEPT['spans'], _KEPT['step'] = hoses.spans(), float(peripherals.world()['step_s'])
        _KEPT['radius'] = {s['name']: float(mechatronics.materials()['hoses'][s['kind']][0]) for s in _KEPT['spans']}
    spans = _KEPT['spans']
    if not spans:
        return []
    if not hasattr(body, 'hosed'):
        names = [part[0] for part in body.parts]
        body.hosed = {seg: names.index(seg) for s in spans for seg in (s['from'][0], s['to'][0])}
    index: Any = body.hosed
    shift = np.array([travel[0], 0.0, travel[1]])
    poses = {seg: (frames[i][0], frames[i][1] + shift) for seg, i in index.items()}
    world: Any = _KEPT.get('world')
    if world is None or not 0.0 <= t - _KEPT['t'] <= GAP_S:
        world = _KEPT['world'] = hoses.World(spans, poses, step=_KEPT['step'])
    else:
        world.follow(poses, t - _KEPT['t'])
    _KEPT['t'] = t
    eye = np.asarray(m, float).reshape(3, 3)[2]
    first, last = [], []
    for span, line in zip(spans, world.lines()):
        pts = hoses.spline(line, 4) - shift
        along = pts[1:] - pts[:-1]
        side = np.cross(along, eye)
        side *= SHARE * _KEPT['radius'][span['name']] / np.maximum(np.linalg.norm(side, axis=1, keepdims=True), 1e-12)
        for k in (-1.0, 0.0, 1.0):
            first.append(pts[:-1] + k * side)
            last.append(pts[1:] + k * side)
    dots = _dots(np.vstack(first), np.vstack(last), m, cam, centre, depth, bare=True)
    return [(dots, INK)] if dots is not None else []
