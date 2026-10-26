"""Her poise on her latent loop: her ZMP into her soles' polygon, and which soles stand.

    low, high = poise.boxes(flat, edge, tilt)       # each sole's box, along and across (2, 2)
    stance, zmp = poise.stands(zmp, xi, low, high, om, held, up, swaying, beat, bears, stood)

Arrays in, arrays out and no branch (tests/test_ortho.py); `machine.modal` renders with it. Her
ZMP as her springs ask it is brought into her soles' polygon - and, the polygon her support's,
to where her capture point stays in it (`maintain_zmp`). A sole stands that touches the floor
(`leap.standing`) and is asked neither up nor on: one asked up leaves once that ZMP lies in
the other; one whose swing has begun leaves once the other stands where it came down.
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine import leap
from machine.figure import BALL, HEEL, SOLE_HALF

#: A sole out of the polygon; a sway of nothing.
BIG, TINY = 1e3, 1e-9
#: Her soles' half length, her figure's.
HALF_M = (BALL + HEEL) / 2.0
#: Her ZMP kept this far inside a sole's edge, m, and her capture point let toward an edge no
#: faster than its room there over CAPTURE_S: on the ZMP's polygon alone a spring of 3 000 N/m
#: took her centre of mass a quarter of the way past her sole's middle and she fell over it; 5
#: mm in, the edge was past where the loop's own margin lets her soles press.
MARGIN_M, CAPTURE_S = 0.018, 0.2
EXTENT = np.array([HALF_M - MARGIN_M, SOLE_HALF - MARGIN_M])
#: A sole stands by `leap.standing`, unless it is asked up: over LIFT_EPS of the lift's
#: unit, it leaves the floor once her ZMP as her springs ask it lies in the other sole's
#: box, GATE_M past its edge at most. Let go by its own load, it never went: the loop keeps
#: a third of her weight on a sole her ZMP is 0.6 of the way off, a twelfth at its furthest.
LIFT_EPS, GATE_M = 0.02, 0.005
#: A sole EARLY of its sweep ahead of her that bears is down, wherever its swing is.
EARLY = 0.3
#: Along her way, of along and across.
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
    """The box `lo` to `hi` grown over what bears her off her soles as far as it does: `prop`
    its (low, high, how far)."""
    low, high, by = prop
    return lo + by * (np.minimum(lo, low) - lo), hi + by * (np.maximum(hi, high) - hi)


def into(zmp, xi, lo, hi, om, share):
    """`zmp` in the box `lo` to `hi` - `share` of the way to where her capture point `xi`
    stays in it."""
    plain = clip(zmp, lo, hi)
    viable = clip(clip(zmp, xi - (hi - xi) / (om * CAPTURE_S), xi + (xi - lo) / (om * CAPTURE_S)), lo, hi)
    return plain + share * (viable - plain)


def stands(zmp, xi, low, high, om, held, up, swaying, beat, bears, stood, prop):
    """(which soles stand, her ZMP): `zmp` as her springs ask it, `xi` her capture point, `low`
    and `high` her soles' boxes, `held` the polygon said (0 none, 1 her support's, 2 the
    dynamic footprint); `up` how far each sole is asked up, `swaying` whether she sways as
    asked; `beat` her stride's (sweep, off, flying), `bears` (loads N, heights over the floor
    as it runs on under them and over the floor felt m, her speed up, what bears her off them
    N), `stood` the soles that stood the pass before;
    `prop` what bears her off
    them (`spanned`), her support with them."""
    sweep, off, flying = beat
    loads, over, felt, rising, borne = bears
    # a sole stands by its height alone only over a floor felt: at a 10 deg ramp's top a swing
    # that ended in the air on the ramp's plane run on stood at 0 N, the other was let go and
    # she came down at 1.8 kN
    level = np.maximum(over, felt)
    kept, foot = clip(held, 0.0, 1.0), clip(held - 1.0, 0.0, 1.0)
    zmp = zmp + kept * (into(zmp, xi, *spanned(low.min(axis=0), high.max(axis=0), prop), om, 1.0 - foot) - zmp)
    # which soles stand. One asked up leaves the floor once that ZMP lies in the other and the
    # other bears as much of her: let go bearing 196 N, the other 43, she dropped onto it
    # at 51 N and fell (play_about's stop, her head's swing no longer under her). One
    # whose swing has begun leaves once the other stands where it came down - or stays
    # down itself, come down ahead of her before its swing was out. Where it lands meets
    # her capture point, wherever that has gone: held down till her capture point was 5
    # cm from the other, her ZMP in the other took it further off and the sole went up
    # and down on its ball
    out_up = BIG * (up > LIFT_EPS)[:, None]
    lo_b, hi_b = spanned(low + out_up, high - out_up, prop)
    within = ((zmp >= lo_b - GATE_M) & (zmp <= hi_b + GATE_M)).all(axis=1)
    landed = np.array(leap.standing(off < 0.5, loads, level, rising, borne))
    early = (sweep > EARLY) & ((loads >= leap.TOUCH_N) | (stood > 0.5))
    gone = (flying > 0.5) & (landed[::-1] | (over > leap.TOUCH_M)) & ~early
    lifting = ((up > LIFT_EPS) & ((within[::-1] & swaying) | (over > leap.TOUCH_M)) & (loads <= loads[::-1])) | gone
    stance = np.array(leap.standing(~lifting, loads, level, rising, borne), float)
    # and into the soles that are to stand - her capture point kept in them too while both
    # do, the footprint said: let go there, a drive asked under a sway her soles refused
    # leant her past her toes standing. A sole whose stride has it off is out of it once
    # the other has landed: her weight is off it before it leaves - left in it, it bore 283
    # N of her 305 as it was let go, the other's knee gave 20 deg and she sank 26 mm
    left = stance * (1.0 - off * landed[::-1])
    left = left + (1.0 - left.max()) * stance
    out_of = BIG * (1.0 - left)[:, None]
    return stance, zmp + kept * (into(zmp, xi, *spanned((low + out_of).min(axis=0), (high - out_of).max(axis=0), prop),
                                      om, foot * left.min()) - zmp)
