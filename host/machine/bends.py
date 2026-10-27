"""How each driven joint goes a radian of her modes: her knees' bend, her arms' swing, reach and fold.

Each a vector over her driven joints (`machine.terms`), her modes' (`machine.modal`).
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine import terms
from machine.stride import LIMBS

#: Her elbow's bend, of its shoulder's reach.
ELBOW_BY = 0.4


def crouch(names):
    """How each driven joint (`names`) goes a radian of her knees' bend as she comes straight
    down: the knee by it, its hip and its ankle by half (`machine.terms`)."""
    bend = np.zeros(len(names))
    for limb in (0, 1):
        for part, term, by in (('hip', 'flexion', 0.5), ('knee', 'flexion', 1.0), ('ankle', 'dorsiflexion', 0.5)):
            kind, sign, _low, _high = terms.TERMS[(part, term)]
            bend[names.index(terms.joint(limb, kind))] = sign * by
    return bend


def swing(names):
    """How each driven joint (`names`) goes a radian of her arms' swing: her first shoulder
    forward by it and the other back, each elbow bent ELBOW_BY of its shoulder's way."""
    out = np.zeros(len(names))
    for limb in (0, 1):
        for part, by in (('shoulder', 1.0), ('elbow', ELBOW_BY)):
            kind, sign, _low, _high = terms.TERMS[(part, 'flexion')]
            out[names.index(terms.joint(limb, kind))] = sign * by * LIMBS[limb]
    return out


def reach(names):
    """How each driven joint (`names`) goes a radian of her arms' reach: both shoulders forward by it."""
    out = np.zeros(len(names))
    kind, sign, _low, _high = terms.TERMS[('shoulder', 'flexion')]
    for limb in (0, 1):
        out[names.index(terms.joint(limb, kind))] = sign
    return out


def fold(names):
    """How each driven joint (`names`) goes a radian of her arms' fold: both elbows bent by it."""
    out = np.zeros(len(names))
    kind, sign, _low, _high = terms.TERMS[('elbow', 'flexion')]
    for limb in (0, 1):
        out[names.index(terms.joint(limb, kind))] = sign
    return out
