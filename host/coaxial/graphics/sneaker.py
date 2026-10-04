"""Her sneaker, size 37-38, drawn: the shoe and the toe cap as lofts along z, their gum soles.

    sneaker.feet(side)     # {side + '_foot': mesh, side + '_toes': mesh}
    sneaker.soles(side)    # [(name, parent, offset, mesh)]: the gum under each

The shoe's rings forward of the ankle, (z, half width, half height), each hung so its bottom is
the sole, flat ANKLE_H under the ankle, as the walk plants it - the collar round the ankle, the
tongue over the instep, the laces down to the ball; the toe cap's from the ball, its sole sprung
SPRING_M up at its tip `figure.TOE_M` ahead; its sole SOLE_M deep at the heel and at the ball
(the drop), gum. A 38's (the user, 2026-10-04): 237 long, 94 wide at the ball, 76 tall at the
heel, 15 of toe spring, the sole 28 and 18. Its mass `build.HOLDS`, its grip and give
`machine.mjcf`, its sole's box `figure.CONTACTS`, its forefoot's bend `drives.TOE_K`. Built, the
sole is printed in TPU with air pockets and the shoe goes over it, so nothing breaks (the user,
2026-10-04).
"""
from coaxial.graphics.lit import paint
from coaxial.graphics.shapes import loft
from machine.figure import TOE_M, TOE_RY
from machine.gait import ANKLE_H, BALL, HEEL

SNEAKER, GUM = (236, 236, 232), (196, 150, 100)
SHOE = ((-0.05, 0.033, 0.038), (-0.028, 0.036, 0.036), (0.0, 0.040, 0.033),
        (0.033, 0.043, 0.031), (0.066, 0.046, 0.027), (0.095, 0.048, 0.022), (BALL, 0.047, 0.019))
TOE_CAP = ((0.0, 0.047, 0.019), (0.019, 0.046, 0.018), (0.035, 0.042, 0.016),
           (0.052, 0.034, 0.013), (0.063, 0.022, 0.009))
SPRING_M, SOLE_M, SOLE_PROUD = 0.015, (0.028, 0.018), 0.002


def _spring(z):
    """How far the toe cap's sole lifts off the floor `z` m ahead of the ball."""
    return SPRING_M * max(0.0, z / TOE_M) ** 2


def _sole_m(z):
    """The sole's depth `z` m ahead of the ankle: the heel's to the ball's (SOLE_M)."""
    t = min(1.0, max(0.0, (z + HEEL) / (BALL + HEEL)))
    return SOLE_M[0] + (SOLE_M[1] - SOLE_M[0]) * t


def feet(side):
    """{segment: mesh}: `side`'s shoe on its foot and its toe cap on its toes."""
    white = paint(SNEAKER)
    return {side + '_foot': loft([(z, rx, rv, ANKLE_H - rv) for z, rx, rv in SHOE], white,
                                 poles=(-HEEL, BALL + 0.006), along='z'),
            side + '_toes': loft([(z, rx, rv, -(rv - TOE_RY + _spring(z))) for z, rx, rv in TOE_CAP],
                                 white, poles=(-0.006, TOE_M + 0.002), along='z')}


def soles(side):
    """[(name, parent, offset, mesh)]: the gum sole under the shoe and under the toe cap, SOLE_M
    deep and SOLE_PROUD wider than the white above it."""
    gum, h = paint(GUM), SOLE_M[1] / 2.0
    return [(side + '_sole', side + '_foot', (0.0, 0.0, 0.0), loft(
        [(z, rx + SOLE_PROUD, _sole_m(z) / 2.0, ANKLE_H - _sole_m(z) / 2.0) for z, rx, _rv in SHOE],
        gum, poles=(-HEEL - SOLE_PROUD, BALL + 0.006), along='z')),
            (side + '_toe_sole', side + '_toes', (0.0, 0.0, 0.0), loft(
                [(z, rx + SOLE_PROUD, h, -(h - TOE_RY + _spring(z))) for z, rx, _rv in TOE_CAP],
                gum, poles=(-0.006, TOE_M + SOLE_PROUD), along='z'))]
