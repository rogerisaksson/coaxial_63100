"""The robot's poise on its latent loop: its ZMP into its soles' polygon, and which soles stand.

    low, high = poise.boxes(flat, edge, tilt)       # each sole's box, along and across (2, 2)
    stance, zmp = poise.stands(zmp, xi, low, high, om, held, up, swaying, beat, bears, stood)

Arrays in, arrays out and no branch (tests/test_ortho.py); `machine.law.modal` renders with it. Its
ZMP as its springs ask it is brought into its soles' polygon - and, the polygon its support's,
to where its capture point stays in it (`maintain_zmp`). A sole stands that touches the floor
(`leap.standing`) and is asked neither up nor on: one asked up leaves once that ZMP lies in
the other; one whose swing has begun leaves once the other stands where it came down.
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine.body import gait
from machine.moves import leap
from machine.body.figure import BALL, HEEL, SOLE_HALF

#: A sole out of the polygon; a sway of nothing.
BIG, TINY = 1e3, 1e-9
#: The robot's soles' half length, its figure's.
HALF_M = (BALL + HEEL) / 2.0
#: The robot's poise's numbers, its build's (`[gait.poise]`, `machine.body.gait`).
_G = gait.table('poise')
#: Its ZMP kept this far inside a sole's edge, m, and its capture point let toward an edge no
#: faster than its room there over CAPTURE_S.
MARGIN_M, CAPTURE_S = _G['margin_m'], _G['capture_s']
EXTENT = np.array([HALF_M - MARGIN_M, SOLE_HALF - MARGIN_M])
#: A sole stands by `leap.standing`, unless it is asked up: over LIFT_EPS of the lift's
#: unit, it leaves the floor once its ZMP as its springs ask it lies in the other sole's
#: box, GATE_M past its edge at most.
LIFT_EPS, GATE_M = _G['lift_eps'], _G['gate_m']
#: A sole EARLY of its sweep ahead of it that bears is down, wherever its swing is.
EARLY = _G['early']
#: Along its way, of along and across.
ON = np.array([1.0, 0.0])


def clip(x, low, high):
    """`x` no lower than `low`, no higher than `high`."""
    return np.minimum(np.maximum(x, low), high)


def boxes(flat, edge, tilt):
    """(low, high), each sole's box (2, 2: along, across) round its middle `flat` - along,
    its edge alone as far as it stands on it (`edge`: 1 its ball, -1 its heel), pitched
    `tilt`."""
    on_edge = edge[:, None] * ON
    mid = flat + on_edge * HALF_M * np.cos(tilt)[:, None]
    return mid - EXTENT * (1.0 - np.abs(on_edge)), mid + EXTENT * (1.0 - np.abs(on_edge))


def spanned(lo, hi, prop):
    """The box `lo` to `hi` grown over what bears it off its soles as far as it does: `prop`
    its (low, high, how far)."""
    low, high, by = prop
    return lo + by * (np.minimum(lo, low) - lo), hi + by * (np.maximum(hi, high) - hi)


def into(zmp, xi, lo, hi, om, share):
    """`zmp` in the box `lo` to `hi` - `share` of the way to where its capture point `xi`
    stays in it."""
    plain = clip(zmp, lo, hi)
    viable = clip(clip(zmp, xi - (hi - xi) / (om * CAPTURE_S), xi + (xi - lo) / (om * CAPTURE_S)), lo, hi)
    return plain + share * (viable - plain)


def stands(zmp, xi, low, high, om, held, up, swaying, beat, bears, stood, prop):
    """(which soles stand, its ZMP): `zmp` as its springs ask it, `xi` its capture point, `low`
    and `high` its soles' boxes, `held` the polygon said (0 none, 1 its support's, 2 the
    dynamic footprint); `up` how far each sole is asked up, `swaying` whether it sways as
    asked; `beat` its stride's (sweep, off, flying, toes), `bears` (loads N, heights over the floor
    as it runs on under them and over the floor felt m, its speed up, what bears it off them
    N), `stood` the soles that stood the pass before;
    `prop` what bears it off
    them (`spanned`), its support with them."""
    sweep, off, flying, toes = beat
    loads, over, felt, rising, borne = bears
    # a sole stands by its height alone only over a floor felt: at a 10 deg ramp's top a swing
    # that ended in the air on the ramp's plane run on stood at 0 N, the other was let go and
    # it came down at 1.8 kN
    level = np.maximum(over, felt)
    kept, foot = clip(held, 0.0, 1.0), clip(held - 1.0, 0.0, 1.0)
    zmp = zmp + kept * (into(zmp, xi, *spanned(low.min(axis=0), high.max(axis=0), prop), om, 1.0 - foot) - zmp)
    # which soles stand. One asked up leaves the floor once that ZMP lies in the other and the
    # other bears as much of it: let go bearing 196 N, the other 43, it dropped onto it
    # at 51 N and fell (play_about's stop, its head's swing no longer under it). One
    # whose swing has begun leaves once the other stands where it came down - or stays
    # down itself, come down ahead of it before its swing was out. Where it lands meets
    # its capture point, wherever that has gone: held down till its capture point was 5
    # cm from the other, its ZMP in the other took it further off and the sole went up
    # and down on its ball. One its stride has off leaves so too once its heel is up as far as a
    # woman's as its toes leave (`toes`): kept down to its swing, at 1.08 m/s its knee went from
    # 28 to 16 deg in 57 ms, and its swing's start reversed it - its hip at its emergency clamp;
    # kept down till its toes bore nothing instead, its heel's rise pressed them and it never left
    out_up = BIG * (up > LIFT_EPS)[:, None]
    lo_b, hi_b = spanned(low + out_up, high - out_up, prop)
    within = ((zmp >= lo_b - GATE_M) & (zmp <= hi_b + GATE_M)).all(axis=1)
    landed = np.array(leap.standing(off < 0.5, loads, level, rising, borne))
    early = (sweep > EARLY) & ((loads >= leap.TOUCH_N) | (stood > 0.5))
    gone = (np.maximum(flying, off * toes) > 0.5) & (landed[::-1] | (over > leap.TOUCH_M)) & ~early
    lifting = ((up > LIFT_EPS) & ((within[::-1] & swaying) | (over > leap.TOUCH_M)) & (loads <= loads[::-1])) | gone
    stance = np.array(leap.standing(~lifting, loads, level, rising, borne), float)
    # and into the soles that are to stand - its capture point kept in them too while both
    # do, the footprint said: let go there, a drive asked under a sway its soles refused
    # leant it past its toes standing. A sole whose stride has it off is out of it once
    # the other has landed: its weight is off it before it leaves - left in it, it bore 283
    # N of its 305 as it was let go, the other's knee gave 20 deg and it sank 26 mm
    left = stance * (1.0 - off * landed[::-1])
    left = left + (1.0 - left.max()) * stance
    out_of = BIG * (1.0 - left)[:, None]
    return stance, zmp + kept * (into(zmp, xi, *spanned((low + out_of).min(axis=0), (high - out_of).max(axis=0), prop),
                                      om, foot * left.min()) - zmp)
