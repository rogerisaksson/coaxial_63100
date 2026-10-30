"""Her sneaker, size 37-38, drawn: the shoe and the toe cap as lofts along z, their gum soles.

    sneaker.feet(side)     # {side + '_foot': mesh, side + '_toes': mesh}
    sneaker.soles(side)    # [(name, parent, offset, mesh)]: the gum under each

The shoe's rings forward of the ankle, (z, half width, half height), each hung so its bottom is
the sole, flat ANKLE_H under the ankle, as the walk plants it - the collar round the ankle, the
tongue over the instep, the laces down to the ball; the toe cap's from the ball, its sole sprung
SPRING_M up at its tip `figure.TOE_M` ahead; its sole SOLE_M deep at the heel and at the ball
(the drop), gum. A 38's: 237 long, 94 wide at the ball, 76 tall at the
heel, 15 of toe spring, the sole 28 and 18. Its mass `build.HOLDS`, its grip and give
`machine.body.mjcf`, its sole's box `figure.CONTACTS`, its forefoot's bend `drives.TOE_K`. Built, the
sole is printed in TPU with air pockets and the shoe goes over it, so nothing breaks.
"""
from coaxial.graphics.lit import paint
from coaxial.graphics.shapes import loft
from machine.body.figure import ANKLE_H, BALL, HEEL, TOE_M, TOE_RY

SNEAKER, GUM, LACE, COUNTER = (236, 236, 232), (196, 150, 100), (218, 220, 226), (222, 224, 220)
SHOE = ((-0.052, 0.031, 0.041), (-0.028, 0.034, 0.034), (0.0, 0.038, 0.031),
        (0.033, 0.036, 0.032), (0.066, 0.045, 0.027), (0.095, 0.048, 0.022), (BALL, 0.047, 0.019))
LAST_X = (0.0, -0.001, -0.002, 0.004, 0.001, -0.003, -0.004)
TOE_CAP = ((0.0, 0.047, 0.019), (0.019, 0.046, 0.018), (0.035, 0.042, 0.016),
           (0.052, 0.034, 0.013), (0.063, 0.024, 0.009))
TOE_X = (-0.004, -0.005, -0.006, -0.007, -0.008)
SPRING_M, SOLE_M, SOLE_PROUD = 0.015, (0.028, 0.018), 0.002


def _spring(z):
    """How far the toe cap's sole lifts off the floor `z` m ahead of the ball."""
    return SPRING_M * max(0.0, z / TOE_M) ** 2


def _sole_m(z):
    """The sole's depth `z` m ahead of the ankle: the heel's to the ball's (SOLE_M) with lug tread."""
    t = min(1.0, max(0.0, (z + HEEL) / (BALL + HEEL)))
    lug = 0.0015 if int(round((z + HEEL) / 0.018)) % 2 == 0 else 0.0
    return SOLE_M[0] + (SOLE_M[1] - SOLE_M[0]) * t + lug


def _skew(mesh, z_pts, x_pts, sign):
    from coaxial.model.blocks import numpy as np
    c, tri, uv, mat = mesh
    c = c.copy()
    c[:, 0] += sign * np.interp(c[:, 2], z_pts, x_pts)
    return c, tri, uv, mat


def feet(side):
    """{segment: mesh}: `side`'s shoe on its foot and its toe cap on its toes."""
    white, s = paint(SNEAKER), (1.0 if side == 'left' else -1.0)
    shoe = loft([(z, rx, rv, ANKLE_H - rv) for z, rx, rv in SHOE], white,
                poles=(-HEEL, BALL + 0.006), along='z')
    toes = loft([(z, rx, rv, -(rv - TOE_RY + _spring(z))) for z, rx, rv in TOE_CAP], white,
                poles=(-0.006, TOE_M + 0.002), along='z')
    return {side + '_foot': _skew(shoe, [r[0] for r in SHOE], LAST_X, s),
            side + '_toes': _skew(toes, [r[0] for r in TOE_CAP], TOE_X, s)}


def soles(side):
    """[(name, parent, offset, mesh)]: the gum sole under the shoe and under the toe cap, SOLE_M
    deep and SOLE_PROUD wider than the white above it, with heel counter and laced eyestay."""
    gum, lace, counter, h = paint(GUM), paint(LACE), paint(COUNTER), SOLE_M[1] / 2.0
    s = 1.0 if side == 'left' else -1.0
    main_sole = _skew(loft(
        [(z, rx + SOLE_PROUD, _sole_m(z) / 2.0, ANKLE_H - _sole_m(z) / 2.0) for z, rx, _rv in SHOE],
        gum, poles=(-HEEL - SOLE_PROUD, BALL + 0.006), along='z'), [r[0] for r in SHOE], LAST_X, s)
    toe_sole = _skew(loft(
        [(z, rx + SOLE_PROUD, h, -(h - TOE_RY + _spring(z))) for z, rx, _rv in TOE_CAP],
        gum, poles=(-0.006, TOE_M + SOLE_PROUD), along='z'), [r[0] for r in TOE_CAP], TOE_X, s)
    heel_cup = _skew(loft(
        [(z, rx + 0.001, rv * 0.72, ANKLE_H - rv * 1.15) for z, rx, rv in SHOE[:3]],
        counter, poles=(-HEEL - 0.002, 0.004), along='z'), [r[0] for r in SHOE[:3]], LAST_X[:3], s)
    eyestay = _skew(loft(
        [(z, rx * 0.52, 0.006, ANKLE_H - 2.0 * rv - 0.003) for z, rx, rv in SHOE[2:6]],
        lace, poles=(0.005, 0.098), along='z'), [r[0] for r in SHOE[2:6]], LAST_X[2:6], s)
    return [(side + '_sole', side + '_foot', (0.0, 0.0, 0.0), main_sole),
            (side + '_toe_sole', side + '_toes', (0.0, 0.0, 0.0), toe_sole),
            (side + '_counter_sole', side + '_foot', (0.0, 0.0, 0.0), heel_cup),
            (side + '_lace_sole', side + '_foot', (0.0, 0.0, 0.0), eyestay)]

