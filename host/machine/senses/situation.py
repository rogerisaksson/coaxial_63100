"""The robot's situation read from its sensors each pass, and the text that fits it laid over its law.

    table = situation.text()                   # the robot's text's (defsituation ..) forms, lisp/humanoid/situations.lisp
    r = situation.readings(y, fr, a, grip, touched, weight)   # (R,), READINGS' order
    into = situation.read(table, r)            # (S,) how far it is in each, 0 to 1
    situation.told(table, into)                # the name of the one it is deepest in, '' in none
    situation.gripped(out, held, stance, table['caps'], into, table['floors'], table['period'], eased, dt)

    (defsituation slippery (grip below 0.25) (blend_duration 500ms)
      (latent_targets (mode_axial_compression 0.35) ..) (cpg_config (amplitude mode_sagittal_drive 0.20) ..))

A situation is bounds on READINGS and what the robot's law takes inside them, whatever it does: the
robot's rhythm's amplitudes no more than its caps, blended in over FIFTH of each bound, and its modes no
lower than its latent targets, eased in over its blend (`gripped`). A situation names no world, no scene
and no body: a robot placed anywhere reads the same.
"""
import math
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine import libm
from machine.law import latent, modal, stride
from machine.texts import ortho
from machine.senses import touch

#: What the robot reads of where it is: the floor's grip, the least either sole has felt of it
#: (`machine.senses.grip`: a sole that slid knows it while the other still stands); the floor's rise a
#: metre along the robot's way under its standing soles (`modal.under`); the share of its
#: weight borne off its soles (`modal.PROP`); its pelvis's up along the world's, 1 upright and
#: 0 lying; the push on its above its soles, of its weight; how high an edge met the
#: robot's swings, m (`modal.RAISE`).
READINGS = ('grip', 'slope', 'borne', 'upright', 'pushed', 'raised')
#: A bound is entered over this share of it from its edge in: grip below 0.25 from 0.25 to 0.20,
#: the robot's grip's envelope (`grip.outside`).
FIFTH = 0.2


def text(text=None, here=None):
    """{names (S,), lo (S, R), hi (S, R), caps (S, N), grips (S,), floors (S, N), period} of each
    (defsituation NAME (READING below|above X) .. (blend_duration T) (latent_targets (MODE N) ..)
    (cpg_config (amplitude MODE N) ..)) in `text` (or the robot's text and its includes, `ortho._FILE`): its
    bounds and its caps on the robot's rhythm's amplitudes, NaN where unsaid; its bound on the robot's grip, 0
    with none; its floors under the robot's modes, -inf where unsaid; the longest blend a floor is said
    with, s (0 with none)."""
    here = here or os.path.dirname(ortho._FILE)
    if text is None:
        with open(ortho._FILE, encoding='utf-8') as f:
            text = f.read()
    names, lo, hi, caps, floors, blends = [], [], [], [], [], []
    for form in ortho._forms(text, here):
        if not (isinstance(form, list) and form and form[0] == 'defsituation'):
            continue
        bounds = [c for c in form[2:] if isinstance(c, list) and c and c[0] in READINGS]
        if len(form) < 3 or not isinstance(form[1], str) or not bounds:
            raise SyntaxError('%s: (defsituation NAME (READING below|above X) .. (cpg_config ..)); a reading is %s'
                              % (ortho.said(form)[:60], ', '.join(READINGS)))
        low, high = np.full(len(READINGS), np.nan), np.full(len(READINGS), np.nan)
        for c in bounds:
            if len(c) != 3 or c[1] not in ('below', 'above') or not isinstance(c[2], float) or not c[2] or not math.isfinite(c[2]):
                raise SyntaxError('%s: (%s below|above X), X a number and not 0' % (ortho.said(c), c[0]))
            (high if c[1] == 'below' else low)[READINGS.index(c[0])] = c[2]
        r = ortho._row([c for c in form[2:] if not any(c is b for b in bounds)], ortho._PAYLOAD)
        names.append(form[1])
        lo.append(low)
        hi.append(high)
        caps.append(r[ortho._HEAD + ortho.AMP:ortho._HEAD + ortho.AMP + ortho.N])
        floors.append(r[ortho._HEAD + ortho.Z:ortho._HEAD + ortho.Z + ortho.N])
        blends.append(1e-3 * r[ortho._BLEND] * float(not np.isnan(floors[-1]).all()))
    lo, hi = np.array(lo, float).reshape(-1, len(READINGS)), np.array(hi, float).reshape(-1, len(READINGS))
    floors = np.array(floors, float).reshape(-1, ortho.N)
    return {'names': tuple(names), 'lo': lo, 'hi': hi, 'caps': np.array(caps, float).reshape(-1, ortho.N),
            'grips': np.nan_to_num(hi[:, READINGS.index('grip')]),
            'floors': np.where(np.isnan(floors), -np.inf, floors), 'period': max(blends, default=0.0)}


def readings(y, fr, a, grip, touched, weight):
    """(R,) READINGS of what the robot senses: `y` (`modal.SENSED`), `fr` its statics, `a` the loop's
    ask, `grip` each sole's as felt (2,), `touched` the edge's touch a segment (SKIN, 3), `weight`
    its own, N."""
    _w, x, _j, k = y[modal.QUAT][:4]
    return np.array([np.min(grip), modal.under(y, fr, a[modal.STANCE])[1], y[modal.PROP], 1.0 - 2.0 * (x * x + k * k),
                     np.linalg.norm(touched[touch.ABOVE], axis=1).sum() / weight, a[modal.RAISE]])


def read(table, r):
    """(S,) how far the robot is in each situation of `table`, 0 to 1: over its bounds, how far `r`
    (`readings`) is inside each, FIFTH of the bound from its edge in, multiplied."""
    lo, hi = table['lo'], table['hi']
    over = np.where(np.isnan(lo), 1.0, np.clip((r - lo) / (FIFTH * np.abs(lo)), 0.0, 1.0))
    under = np.where(np.isnan(hi), 1.0, np.clip((hi - r) / (FIFTH * np.abs(hi)), 0.0, 1.0))
    return (over * under).prod(axis=1)


def told(table, into, at=0.5):
    """The name of the situation the robot is deepest in, `at` or more of the way; '' in none."""
    if not len(into) or into.max() < at:
        return ''
    return table['names'][int(np.argmax(into))]


def gripped(out, held, stance, caps, into, floors, period, eased, dt):
    """`out` in the situations the robot is in (`into` (S,), `read`): each mode's sway and its sine scaled
    down to each's caps (S, N; `text`) as far as the robot is in it; the robot's drive's further to its going
    no faster than sqrt(2 STRIDE_M grip g) on its standing soles' grip (`held`) - a stride it can stop within:
    0.77 m/s on ice (0.1), 1.9 at its law's own 0.6. Asked 0.69 on ice at 0.02 the robot ran on and fell at
    3.0-3.4 s. And each mode no lower than the floors (S, N) the robot is in, as far as it is in them,
    whatever its row says - a told row's crouch the brake's and the stand's rows set upright again on the
    ice -: their lift `eased` (2, N: now and its rate, a pass `dt` s on), a critically damped spring whose
    period is `period` (the longest blend a floor is said with), 99 % there at its end."""
    grip_now = ((stance + 1e-9) / (stance.sum() + 2e-9)) @ held
    sway = np.abs(out[latent.SWAY])
    under = into[:, None]
    capped = np.where(np.isnan(caps), 1.0, np.minimum(1.0, np.nan_to_num(caps) / np.maximum(sway, 1e-9)))
    k = np.min(1.0 - under + under * capped, axis=0, initial=1.0)
    turns = out[latent.TURNS]
    cruise = 2.0 / math.pi * stride.STRIDE_M * (sway[modal.DRIVE] * k[modal.DRIVE]) * libm.sin(stride.swings(sway[modal.DRIVE] * k[modal.DRIVE], turns, 2.0 / math.pi * stride.STRIDE_M * (sway[modal.DRIVE] * k[modal.DRIVE]) * turns)) * turns
    k[modal.DRIVE] *= np.minimum(1.0, np.sqrt(2.0 * stride.STRIDE_M * grip_now * modal.G) / np.maximum(cruise, 1e-9))
    sine = out[latent.SWAY] * libm.sin(out[latent.PSI])
    out[latent.REF] += (k - 1.0) * sine
    out[latent.RATE] += (k - 1.0) * out[latent.SWAY] * turns * libm.cos(out[latent.PSI])
    out[latent.ACC] -= (k - 1.0) * sine * turns * turns
    out[latent.SWAY] *= k
    want = np.max(under * np.maximum(floors - out[latent.REF], 0.0), axis=0, initial=0.0)
    om = 2.0 * math.pi / np.maximum(period, dt)
    acc = om * om * (want - eased[0]) - 2.0 * om * eased[1]
    eased[1] += acc * dt
    eased[0] += eased[1] * dt
    out[latent.REF] += eased[0]
    out[latent.RATE] += eased[1]
    out[latent.ACC] += acc
