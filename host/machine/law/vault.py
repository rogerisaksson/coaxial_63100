"""Her hip over a standing leg: the arc about its ankle, or held where its heel rises for.

    risen = vault.vaults(legs, v, shod, along, propped, pressed, axial_m, hips_by, low, high, reach, duck, heeled, striding, asked, sway, kp, kd)

Arrays in, arrays out and no branch (tests/test_ortho.py); `machine.law.impedance` renders with it, her
heel's rise and the height it reaches for from `machine.law.rocker.heels`.
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine.law.poise import TINY, clip
from machine.law.stride import EDGE_RAD, UP


def vaults(legs, v, shod, along, propped, pressed, axial_m, hips_by, low, high, reach, duck, heeled, striding, asked, sway, kp, kd):
    """Each standing leg's hip's acceleration up, m/s^2 (2,). Over a flat sole, as the leg's length (`legs`: hip
    from ankle, length, rate) follows `pressed` (the axial spring's, units of `axial_m` m a unit of her centre's,
    `hips_by` of her hips') while her hip arcs over the ankle: what her going over it (`v`, hers; `shod`, the
    sole's) and her `along` ask give taken off, by how far over the ankle the hip is (`low` m at least). Its heel
    asked up (`heeled`, in full from EDGE_RAD) in a stride's lone stance (`striding`, 0 to 1: the other sole
    swinging - not the double support, where this leg unloads and its knee folds for its swing: held through it,
    her knee left the floor at 18 deg and her hip swung it at 170 N m; nor a leap's toe landing, nor a throw's), as holds the
    hip at the height the heel rises for (`heels`' `reach` over the floor, `duck` m down and its rate; `high` the
    hip over the floor) on that spring (`kp`, `kd`): vaulting about a still ankle while the heel rose, the loop
    asked her centre down 2.4 m/s^2 as the ankle came up 4, her trailing knee folded 21 -> 30 deg before each
    landing and her hip came 3.6 cm low to it; held at a leap's toe landing, its damper threw her falling centre
    back up 19 m/s^2 and she bounced onto one sole (8 of test_leap's 23). And no further than her drive is still
    asked: its amplitude as `asked` (the text's, `latent.ASKED`) over the one running (`sway`, blending to it),
    1 while the ask stands, 0 the pass it is let go - stopping from her march, her stance leg vaulted her
    0.82 -> 1.24 m/s (knee 44 -> 5 deg, heel 43) into a landing at 3.48 of her weight. Gated on her speed
    over the pace `stride.cruises` reckons instead, her march, 15 % faster than that reckoning, lost 15 % of
    its hold and the 143 N m/rad leaf's squatted to 52-89 deg."""
    push = clip(np.abs(asked) / np.maximum(np.abs(sway), TINY), 0.0, 1.0)
    la = legs[0][:, :2] @ along
    arc = ((legs[2] ** 2 - ((v - shod) ** 2).sum(axis=1) - np.maximum(la, (1.0 - propped) * la)
            - legs[1] * axial_m * hips_by * pressed) / np.maximum(legs[0][:, UP], low))
    held = kp * (reach + duck[0] * hips_by - high) + kd * (duck[1] * hips_by - hips_by * v[UP])
    return arc + striding * push * clip(heeled / EDGE_RAD, 0.0, 1.0) * (held - arc)
