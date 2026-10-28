"""Her situation read from her sensors each pass, and the text that fits it laid over her law.

    table = situation.text()                   # her text's (defsituation ..) forms, lisp/humanoid/situations.lisp
    r = situation.readings(y, fr, a, grip, touched, weight)   # (R,), READINGS' order
    into = situation.read(table, r)            # (S,) how far she is in each, 0 to 1
    situation.told(table, into)                # the name of the one she is deepest in, '' in none

    (defsituation slippery (grip below 0.25)
      (cpg_config (amplitude mode_sagittal_drive 0.20) ..))

A situation is bounds on READINGS and what her law takes inside them, whatever she does: her
rhythm's amplitudes no more than its caps (`law.gripped`), blended in over FIFTH of each bound.
It names no world, no scene and no body: a robot placed anywhere reads the same.
"""
import math
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine import modal, ortho, touch

#: What she reads of where she is: the floor's grip, the least either sole has felt of it
#: (`machine.grip`: a sole that slid knows it while the other still stands); the floor's rise a
#: metre along her way under her standing soles (`modal.under`); the share of her weight borne
#: off her soles (`modal.PROP`); her pelvis's up along the world's, 1 upright and 0 lying; the
#: push on her above her soles, of her weight; how high an edge met her swings, m (`modal.RAISE`).
READINGS = ('grip', 'slope', 'borne', 'upright', 'pushed', 'raised')
#: A bound is entered over this share of it from its edge in: grip below 0.25 from 0.25 to 0.20,
#: her grip's envelope (`grip.outside`).
FIFTH = 0.2


def text(text=None, here=None):
    """{names (S,), lo (S, R), hi (S, R), caps (S, N), grips (S,)} of each (defsituation NAME
    (READING below|above X) .. (cpg_config (amplitude MODE N) ..)) in `text` (or her text and its
    includes, `ortho._FILE`): its bounds and its caps on her rhythm's amplitudes, NaN where unsaid;
    its bound on her grip, 0 with none."""
    here = here or os.path.dirname(ortho._FILE)
    if text is None:
        with open(ortho._FILE, encoding='utf-8') as f:
            text = f.read()
    names, lo, hi, caps = [], [], [], []
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
    lo, hi = np.array(lo, float).reshape(-1, len(READINGS)), np.array(hi, float).reshape(-1, len(READINGS))
    return {'names': tuple(names), 'lo': lo, 'hi': hi, 'caps': np.array(caps, float).reshape(-1, ortho.N),
            'grips': np.nan_to_num(hi[:, READINGS.index('grip')])}


def readings(y, fr, a, grip, touched, weight):
    """(R,) READINGS of what she senses: `y` (`modal.SENSED`), `fr` her statics, `a` the loop's
    ask, `grip` each sole's as felt (2,), `touched` the edge's touch a segment (SKIN, 3), `weight`
    hers, N."""
    _w, x, _j, k = y[modal.QUAT][:4]
    return np.array([np.min(grip), modal.under(y, fr, a[modal.STANCE])[1], y[modal.PROP], 1.0 - 2.0 * (x * x + k * k),
                     np.linalg.norm(touched[touch.ABOVE], axis=1).sum() / weight, a[modal.RAISE]])


def read(table, r):
    """(S,) how far she is in each situation of `table`, 0 to 1: over its bounds, how far `r`
    (`readings`) is inside each, FIFTH of the bound from its edge in, multiplied."""
    lo, hi = table['lo'], table['hi']
    over = np.where(np.isnan(lo), 1.0, np.clip((r - lo) / (FIFTH * np.abs(lo)), 0.0, 1.0))
    under = np.where(np.isnan(hi), 1.0, np.clip((hi - r) / (FIFTH * np.abs(hi)), 0.0, 1.0))
    return (over * under).prod(axis=1)


def told(table, into, at=0.5):
    """The name of the situation she is deepest in, `at` or more of the way; '' in none."""
    if not len(into) or into.max() < at:
        return ''
    return table['names'][int(np.argmax(into))]
