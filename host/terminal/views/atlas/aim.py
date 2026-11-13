"""Where the camera looks so what it frames stands in its view's middle: worked out, not guessed.

A frame's target is where the view turns about and what the drawing's middle shows; in perspective a point's
offset on screen is its offset across the view over its depth, so a point in the world's middle of what is
framed falls off the drawing's middle wherever the view looks askew at it - the nearer side drawn larger, a
cap hung under a ring pulling the middle down. `centred` moves the target across the view, right and up, till
the farthest out on each side stand as far out as those on the other. `eased` takes the camera there a frame
at a time.
"""
import math

from terminal.views.atlas import scene

#: The ease's time constant, s; O's orbit, degrees a second.
EASE_S, ORBIT_DEG_S = 0.2, 9.0


def eased(state, now, framing=None):
    """The camera a frame on: its goal kept on its frame as the view turns - `framing`() (key, target, distance)
    for the view as it faces, falsy where none: the goal moved as the frame moved, its distance as the frame's
    grew, so a pan and a zoom stay -, its target and distance eased to the goal (EASE_S), O's orbit."""
    frame, was, goal = framing() if framing else None, state.get('framed'), state['goal']
    if frame and was and was[0] == frame[0]:
        goal['target'] = tuple(g + t - w for g, t, w in zip(goal['target'], frame[1], was[1]))
        goal['distance'] *= frame[2] / was[2]
    state['framed'] = frame
    then, state['then'] = state['then'], now
    dt = 0.0 if then is None else min(0.25, max(0.0, now - then))
    cam, goal, k = state['cam'], state['goal'], 1.0 - math.exp(-dt / EASE_S)
    cam['target'] = tuple(c + (g - c) * k for c, g in zip(cam['target'], goal['target']))
    cam['distance'] *= (goal['distance'] / cam['distance']) ** k
    if state['orbit'] and not state['mirror']:
        cam['yaw'] += ORBIT_DEG_S * dt


def centred(points, cam, distance, across=0.0):
    """The target that, `distance` off and turned as `cam` is (its yaw and pitch), puts `points` in the middle
    of the view - their projection's middle `across` off it where told (x / z, right on) -: their box's middle
    moved across the view, each way where the farthest out on either side stand as far from it - a move across
    the view leaves every depth as it is, and the middle of the farthest falls as the target moves that way, so
    halving finds the one place, to as near as a float tells."""
    from coaxial.model.blocks import numpy as np
    at = np.array(points, float).reshape(-1, 3)
    sight = scene.view(dict(cam, distance=distance), 1, 1)
    ahead, middle = np.array(sight['f']), (at.max(0) + at.min(0)) / 2
    eye = middle - distance * ahead
    depth, moved = (at - eye) @ ahead, []
    for axis, off in ((np.array(sight['r']), across), (np.array(sight['u']), 0.0)):
        coord, pad = (at - eye) @ axis, abs(off) * float(depth.max())   # the room the offset asks
        low, high = float(coord.min()) - pad, float(coord.max()) + pad
        mid = (low + high) / 2
        while low < mid < high:
            seen = (coord - mid) / depth
            low, high = (mid, high) if seen.max() + seen.min() > 2 * off else (low, mid)
            mid = (low + high) / 2
        moved.append(mid)
    return tuple((middle + moved[0] * np.array(sight['r']) + moved[1] * np.array(sight['u'])).tolist())


def beside(points, cam, distance, panel, half):
    """(target, distance, whether the panel stands right) that stands `points` in the middle of the room a panel
    `panel` wide leaves of a view `half` wide each way (x / z across it): the panel on the side the points,
    centred, hold fewer of; as far as `distance`, or as far again as their width needs to fit the room - each step
    out by how many times too wide they stand, the width falling as the view draws back, to as near as a float
    tells."""
    from coaxial.model.blocks import numpy as np
    at = np.array(points, float).reshape(-1, 3)

    def seen(target, d):   # each point across the view over its depth
        sight = scene.view(dict(cam, distance=d, target=target), 1, 1)
        rel = at - np.array(sight['eye'])
        return (rel @ np.array(sight['r'])) / (rel @ np.array(sight['f']))
    x = seen(centred(points, cam, distance), distance)
    side = 1.0 if (x > 0).sum() <= (x < 0).sum() else -1.0   # the panel right where the right holds no more
    while True:
        target = centred(points, cam, distance, -side * panel / 2)
        x = seen(target, distance)
        wide = (x.max() - x.min()) / (2 * half - panel)
        if wide <= 1.0 + 1e-12:
            return target, distance, side > 0
        distance *= wide
