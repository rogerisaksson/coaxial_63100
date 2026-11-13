"""Where the camera looks so what it frames stands in its view's middle: worked out, not guessed.

A frame's target is where the view turns about and what the drawing's middle shows; in perspective a point's
offset on screen is its offset across the view over its depth, so a point in the world's middle of what is
framed falls off the drawing's middle wherever the view looks askew at it - the nearer side drawn larger, a
cap hung under a ring pulling the middle down. `centred` moves the target across the view, right and up, till
the farthest out on each side stand as far out as those on the other.
"""
from terminal.views.atlas import scene


def centred(points, cam, distance):
    """The target that, `distance` off and turned as `cam` is (its yaw and pitch), puts `points` in the middle
    of the view: their box's middle moved across the view, each way where the farthest out on either side stand
    as far out - a move across the view leaves every depth as it is, and the middle of the farthest falls as the
    target moves that way, so halving finds the one place, to as near as a float tells."""
    from coaxial.model.blocks import numpy as np
    at = np.array(points, float).reshape(-1, 3)
    sight = scene.view(dict(cam, distance=distance), 1, 1)
    ahead, middle = np.array(sight['f']), (at.max(0) + at.min(0)) / 2
    eye = middle - distance * ahead
    depth, moved = (at - eye) @ ahead, []
    for axis in (np.array(sight['r']), np.array(sight['u'])):
        across = (at - eye) @ axis
        low, high = float(across.min()), float(across.max())
        mid = (low + high) / 2
        while low < mid < high:
            seen = (across - mid) / depth
            low, high = (mid, high) if seen.max() + seen.min() > 0 else (low, mid)
            mid = (low + high) / 2
        moved.append(mid)
    return tuple((middle + moved[0] * np.array(sight['r']) + moved[1] * np.array(sight['u'])).tolist())
