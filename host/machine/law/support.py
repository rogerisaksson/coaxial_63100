"""Sparse coupling between her driven joints' kinematic innovation and her modal references.

    dq = support.couple(q, qd, ref_q, ref_v, eso_z, out)   # arrays in, array out, no branch

When an upper-body joint (her arms or her waist) meets an obstruction that holds it back from
its inner-loop reference (`ref_q`, `ref_v`), its kinematic innovation `nu` is shrunk smoothly
by `nu^3 / (nu^2 + SIGMA^2)`: her non-weight-bearing posture yields (`GAMMA`), her observer's
disturbance integral (`eso_z`) bleeds so the arm never winds up into a rigid hook, and the
sparse coupling matrix `PI` shifts `LATERAL` and `TURN` away from the deflected side so she
steps clear without stopping.
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine.law import latent
from machine.law.modal import LATERAL, N, TURN

#: Driven indices (`rounds.DRIVEN`) of the non-weight-bearing upper body: waist, left arm, right arm.
UPPER = np.array([2, 5, 6, 13, 14])
DRIVEN = 21

#: Velocity lead, s; ESO disturbance floor and scale, rad/s^2; shrinkage knee, rad; ESO bleed, 1/rad^2.
TAU_V, ESO_0, ESO_S, SIGMA, BLEED = 0.08, 12.0, 25.0, 0.38, 35.0

#: Upper-body posture admittance per driven joint (0 on weight-bearing legs and spine pitch/roll).
GAMMA = np.zeros(DRIVEN)
GAMMA[UPPER] = (0.35, 0.95, 0.85, 0.95, 0.85)

#: Sparse coupling matrix `PI` (modes, driven): left arm held back (`nu < 0`) shifts `LATERAL`
#: and `TURN` negative (to her right); right arm held back (`nu < 0`) shifts them positive (left).
PI = np.zeros((N, DRIVEN))
PI[LATERAL, UPPER] = (0.18, 0.25, 0.12, -0.25, -0.12)
PI[TURN, UPPER] = (0.45, 0.40, 0.18, -0.40, -0.18)


def couple(q, qd, ref_q, ref_v, eso_z, out):
    """`dq` (driven,) rad of upper-body posture admittance, `out[latent.REF]` shifted by `PI @ phi`
    and `eso_z` bled on obstructed non-weight-bearing joints."""
    err, s2, z0 = q - ref_q, SIGMA * SIGMA, ESO_S * ESO_S
    e2, z2 = err * err, np.maximum(eso_z * np.copysign(1.0, err) - ESO_0, 0.0) ** 2
    e4, z4, s4, sz4 = e2 * e2, z2 * z2, s2 * s2, z0 * z0
    e8, s8 = e4 * e4, s4 * s4
    phi = (z4 / (z4 + sz4)) * (e8 / (e8 + s8)) * (err + TAU_V * (qd - ref_v) * e2 / (e2 + s2))
    out[latent.REF] += PI @ phi
    eso_z /= 1.0 + BLEED * GAMMA * (phi * phi)
    return GAMMA * phi

