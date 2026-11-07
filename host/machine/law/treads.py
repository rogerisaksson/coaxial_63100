"""A floor told her: where each swing lands on its treads, the height under a place and between two.

    s_new, top, land, run, cell = treads.footholds(along, rise, told, s_meet, s_fore, s_now, s_com, reach, hold, gap, step)
    treads.floor_at(along, rise, told, s)            # a told floor's height under each s
    treads.floor_mean(along, rise, told, s0, s1)     # its mean height from s0 to s1

Arrays in, arrays out and no branch (tests/test_ortho.py); `machine.law.impedance` lands with it, her
frame's map (`modal.MAP_ALONG`, `MAP_RISE`) the floor told; wbc_floor.c the same in C.
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine.body.figure import TOE_M
from machine.law.poise import BIG, HALF_M, MARGIN_M, TINY, clip


def footholds(along, rise, told, s_meet, s_fore, s_now, s_com, reach, hold, gap, step):
    """Each swing's landing along a floor told her (`along`, `rise`, `told` breakpoints) moved onto
    the flat tread nearest where it will touch down (`s_fore`) that its whole sole fits within her
    reach (`s_com` +- `reach`), the tread chosen as it left kept (`hold`, -1 none), the landing
    that tread's point nearest its natural one (`s_meet`) drawn to the tread's end by how near
    that end lies within a `step` (m, her centre's travel a step); the highest floor between each sole
    (`s_now`) and its landing; the landing's height over the floor felt (`gap` the told floor's
    over it); each tread's run; the tread landed on (-1 none). Nothing told, or no tread in reach,
    the landings as they are and the floor level."""
    cells = np.arange(len(along) - 1)
    # the last cell runs on past the map's end, as its height does (`floor_at`): drawn back to its
    # band at the top landing's told end she stumbled, 37 cm short
    a, b = along[:-1], np.where(cells == told - 2, BIG, along[1:])
    on = cells < told - 1
    # the heel may hang over the tread's edge behind, the toes stay MARGIN_M short of the riser ahead;
    # the landing the band's point nearest its natural one (at the band's middle instead, a swing
    # bound 4 mm past the flat's end was pulled 21 cm back behind its own sole), drawn on to the
    # band's end as that comes within a step: landed 6-9 cm ahead of her centre where the takes'
    # feet land 14-25, the pull-up pushed her past the stance toe at 1.0 m/s (findings/stairs.md)
    flat = on & (np.abs(rise[1:] - rise[:-1]) < TINY) & (b - a > HALF_M + TOE_M + 2.0 * MARGIN_M)
    lo, hi = np.maximum(a + MARGIN_M, s_com - reach), np.minimum(b - HALF_M - TOE_M - MARGIN_M, s_com + reach)
    fits = flat & (lo <= hi)
    k = np.argmin(np.abs(clip(s_fore[:, None], lo[None, :], hi[None, :]) - s_fore[:, None]) + BIG * (1.0 - fits), axis=1)
    held = (hold >= 0) & fits[np.maximum(hold, 0).astype(int)]
    k = np.where(held, np.maximum(hold, 0).astype(int), k)
    any_ = float(fits.any())
    drawn = clip(s_meet, lo[k], hi[k])
    pull = clip(1.0 - (hi[k] - drawn) / np.maximum(step, TINY), 0.0, 1.0)
    s_new = s_meet + any_ * (drawn + (hi[k] - drawn) * pull - s_meet)
    span_lo, span_hi = np.minimum(s_now, s_new) - HALF_M, np.maximum(s_now, s_new) + HALF_M + TOE_M
    over = on[None, :] & (b[None, :] >= span_lo[:, None]) & (a[None, :] <= span_hi[:, None])
    n = max(int(told), 2)
    ends = np.maximum(np.interp(span_lo, along[:n], rise[:n]), np.interp(span_hi, along[:n], rise[:n]))
    top = any_ * (np.maximum(np.max(np.where(over, rise[:-1][None, :], -BIG), axis=1), ends) + gap) - BIG * (1.0 - any_)
    return (s_new, top, any_ * (rise[:-1][k] + gap), any_ * (b - a)[k] + BIG * (1.0 - any_),
            any_ * k - (1.0 - any_))


def floor_at(along, rise, told, s):
    """A told floor's height under each `s` (m along it): the tread's it lies on, its ends' past them,
    0 with nothing told."""
    n = max(int(told), 2)
    x = clip(s, along[0], along[n - 1])[:, None]
    on = (np.arange(len(along) - 1) < told - 1)[None, :] & (along[None, :-1] <= x) & (x < along[None, 1:])
    return (told > 0) * (np.sum(np.where(on, rise[None, :-1], 0.0), axis=1) + (1.0 - on.any(axis=1)) * rise[n - 1])


def floor_mean(along, rise, told, s0, s1):
    """A told floor's mean height from `s0` to `s1` (m along it): its cells' integral, its ends'
    heights past them; 0 with nothing told."""
    n = max(int(told), 2)
    on = np.arange(len(along) - 1) < told - 1
    s = np.array([s0, s1])
    x = clip(s, along[0], along[n - 1])[:, None]
    area = (rise[0] * np.minimum(s - along[0], 0.0) + rise[n - 1] * np.maximum(s - along[n - 1], 0.0)
            + np.sum(on[None, :] * rise[None, :-1] * (clip(x, along[None, :-1], along[None, 1:]) - along[None, :-1]), axis=1))
    return (told > 0) * (area[1] - area[0]) / (s1 - s0)
