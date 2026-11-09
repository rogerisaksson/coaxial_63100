"""Her statics of what is sensed standing: her frame, the mass each mode moves, her rest vector.

Laid once, before her loop runs (modal's FRAME, `ortho`'s parameters).

    fr = statics.frame(y, mass, limb, trunk, arm, forearm, knee, dt)  # her statics, once (`sprung`)
    rest = statics.rest(fr)                                         # her rest vector (`latent.state`)
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine.body.figure import ANKLE_H
from machine.law import latent
from machine.law.modal import (ARM, ARM0, ARM_RAD, AXIAL, AXIAL_M, CLEAR_M, COM, DT, ELBOW, ELBOW0, FLOOR, FLOOR_AT,
                               FRAME, FREE, G, H0, HEADING, HIP, HIPS_BY, HOT, KNEE, LIMB_KD, LIST_RAD, MASS, OMEGA,
                               PITCH0, PITCH_RAD, QUAT, REACH0, REST_K, ROT, SOLE, SUPPORT, TURN_RAD, UNIT, W0, WEIGHT,
                               WIDTH, Z0, pitched)
from machine.law.poise import HALF_M
from machine.law.stride import ALONG, LEFT, ROCK_RAD, STRIDE_M, UP
from machine.moves import leap
from machine.stack.wbc import HEIGHT_KD, POSTURE_KD, TURN_KD
from machine.texts import ortho


def frame(y, mass, limb, trunk, arm, forearm, knee, dt):
    """Her statics of what is sensed standing: `mass` hers, a `limb`'s, her `trunk`'s, an
    `arm`'s and a `forearm`'s, kg; `knee` her knees' bend, rad; `dt` s from an ask to the next."""
    q = y[QUAT]
    yaw = 2.0 * np.arctan2(q[2], q[0])
    s, c = np.sin(yaw), np.cos(yaw)
    rot = np.array([[s, 0.0, c], [c, 0.0, -s], [0.0, 1.0, 0.0]])
    soles = y[SOLE].reshape(2, 3)
    legs = y[HIP].reshape(2, 3) - soles + leap.MIDDLE * rot[ALONG] - ANKLE_H * rot[UP]
    fr = np.zeros(FRAME)
    fr[ROT] = rot.ravel()
    fr[FLOOR], fr[FLOOR_AT] = soles[:, 1].mean(), soles[:, [0, 2]].mean(axis=0)
    fr[H0] = y[COM][1] - fr[FLOOR]
    fr[W0] = 0.5 * abs((soles[0] - soles[1]) @ rot[LEFT])
    fr[OMEGA], fr[WEIGHT], fr[KNEE], fr[DT] = np.sqrt(G / fr[H0]), mass * G, knee, dt
    fr[PITCH0] = pitched(y, rot)
    fr[HEADING], fr[ARM0], fr[ELBOW0] = yaw, y[ARM], y[ELBOW]
    fr[REACH0], fr[HIPS_BY] = np.linalg.norm(legs, axis=1).mean(), mass / (mass - limb)
    fr[UNIT] = (AXIAL_M, fr[W0], STRIDE_M, PITCH_RAD, HALF_M, 0.5 * CLEAR_M, fr[W0], LIST_RAD, TURN_RAD,
                ARM_RAD, ROCK_RAD, ARM_RAD, ARM_RAD)
    fr[Z0] = HOT[AXIAL] * fr[H0] / AXIAL_M - HOT[WIDTH]
    fr[MASS] = masses(mass, limb, trunk, arm, forearm)
    return fr


def masses(mass, limb, trunk, arm, forearm):
    """The mass each mode moves, kg, of hers, a limb's, her trunk's, an arm's and a forearm's."""
    return np.array([mass, mass, limb, trunk, mass, limb, limb, trunk, trunk, arm, limb, arm, forearm])


def rest(fr):
    """Her rest vector (`ortho`'s parameters): every target 0, each mode's spring and damper,
    her rhythm off at 1 Hz, her ZMP in her support polygon, her yaw momentum free, what bears her
    unsaid - her soles alone then (`observer.weighed`), nothing capped (`somatic.asked`)."""
    floor = REST_K * fr[OMEGA], fr[OMEGA] + REST_K
    p = np.zeros(ortho.P)
    for part, kd in ((latent.K, 0), (latent.D, 1)):
        p[part] = fr[MASS] * np.array([HEIGHT_KD[kd], floor[kd], LIMB_KD[kd], TURN_KD[kd], floor[kd],
                                       LIMB_KD[kd], LIMB_KD[kd], TURN_KD[kd], TURN_KD[kd],
                                       POSTURE_KD[kd], TURN_KD[kd], POSTURE_KD[kd], POSTURE_KD[kd]])
    p[latent.HZ], p[latent.ZMP], p[latent.YAW] = 1.0, SUPPORT, FREE
    p[latent.MANIFOLD] = 0.0
    return p
