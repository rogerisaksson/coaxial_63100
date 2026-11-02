"""Moved from host/coaxial/graphics/stereotype.py: _box_fit - nothing in the tree named it."""
import math


def _box_fit(pts):
    """(corners, dev): the least-area box round `pts` over angles in
    2-degree steps, and the points' mean distance to its boundary over
    its shorter side - zero when every point sits on a side."""
    def at(deg):
        a = math.radians(deg)
        ca, sa = math.cos(a), math.sin(a)
        us = [p[0] * ca + p[1] * sa for p in pts]
        vs = [-p[0] * sa + p[1] * ca for p in pts]
        u0, u1, v0, v1 = min(us), max(us), min(vs), max(vs)
        return (u1 - u0) * (v1 - v0), a, u0, u1, v0, v1, us, vs

    _area, a, u0, u1, v0, v1, us, vs = min((at(d) for d in range(0, 90, 2)),
                                           key=lambda fit: fit[0])
    short = max(1e-9, min(u1 - u0, v1 - v0))
    dev = sum(min(u - u0, u1 - u, v - v0, v1 - v)
              for u, v in zip(us, vs)) / len(us) / short
    ca, sa = math.cos(a), math.sin(a)
    corners = [(u * ca - v * sa, u * sa + v * ca)
               for u, v in ((u0, v0), (u1, v0), (u1, v1), (u0, v1))]
    return corners, dev
