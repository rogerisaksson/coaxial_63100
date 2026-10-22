"""A transition on her loop: a patch's lisp read as forms, a morph as rows, when it may begin.

    forms = patch.forms(text)                     # lists, vectors as tuples, keywords, numbers
    wire, span = patch.morph(dry, wet, bands, clock)  # contiguous curve rows on the bar grid
    at = patch.crossing(st, now)                  # her lateral voice's next zero crossing, s
    span = patch.window(st)                       # its double support, s, to wait in for the lock
    ok = patch.locked(y, fr, a)                   # both soles stand, the ZMP asked in their boxes

A patch's lisp is evaluated and voiced on her row by `machine.dsp`. A morph is two bars of rows a
sixteenth apart (GRID), each saying every moved slot at its time - the dry plus the wet's change
on its band's curve over the band's window - and the clock on its own, contiguous curve samples
(`ortho.CURVE`) her loop draws as one Catmull-Rom; its times run on the clock as it slews. A bar
is a stride, her clock's turn. Measured on her (findings/sequencer.md): the dry and the wet at
once, by a linear crossfade at any phase of the stride or by the morph locked to the crossing,
stand alike; the rock voice stiffened for the heel's transient felled her alone.
"""
import math
import re

from machine import modal, ortho, stride
from machine.curves import CURVES

#: The morph's grid, sixteenths of a bar.
GRID = 16
#: A morph's rows' sequence id, after the takes' (300) and a modified row (301).
ROW_ID = 302

_TOKEN = re.compile(r'\s*(?:(;[^\n]*)|([()\[\]])|(->)|("(?:[^"\\]|\\.)*")|([^\s()\[\];"]+))')


def _tokens(text):
    pos = 0
    while pos < len(text):
        m = _TOKEN.match(text, pos)
        if not m or m.end() == pos:
            if text[pos:].strip():
                raise SyntaxError('unreadable at %r' % text[pos:pos + 30])
            return
        pos = m.end()
        comment, paren, arrow, string, atom = m.groups()
        if not comment:
            yield paren or arrow or string or atom


def _atom(tok):
    if tok.startswith('"'):
        return tok[1:-1]
    if tok in ('true', 'false', 'nil'):
        return {'true': True, 'false': False, 'nil': None}[tok]
    try:
        return float(tok)
    except ValueError:
        return tok


def _put(into, item):
    """`item` onto the open form `into`, as (quote item) where a quote stands before it."""
    into[-1:] = [['quote', item]] if into[-1:] == ["'"] else into[-1:] + [item]


def forms(text):
    """The text's forms: lists, vectors as tuples, keywords and symbols as strings, numbers, a
    quoted form (quote form)."""
    stack = [[]]
    for tok in _tokens(text):
        if tok in ('(', '['):
            stack.append([] if tok == '(' else ['[]'])
        elif tok in (')', ']'):
            if len(stack) == 1:
                raise SyntaxError('a %s closes nothing' % tok)
            done = stack.pop()
            _put(stack[-1], tuple(done[1:]) if done[:1] == ['[]'] else done)
        else:
            _put(stack[-1], _atom(tok))
    if len(stack) != 1:
        raise SyntaxError('%d forms left open' % (len(stack) - 1))
    return stack[0]


def morph(dry, wet, bands, clock, bars=2.0):
    """The transition's rows (`ortho.ROW`, contiguous curve samples a sixteenth of a bar apart) and
    their span, s: each sample every slot at the dry plus the wet's change on its band's curve, the
    clock's Hz on its own, the times as the clock slews (a sixteenth over its mean Hz)."""
    import numpy as np
    at = np.arange(int(bars * GRID) + 1) / GRID
    share = lambda c, b0, b1, b: CURVES[c](min(max((b - b0) / (b1 - b0), 0.0), 1.0))
    curve, c0, c1, hz0, hz1 = clock
    hz = np.array([hz0 + (hz1 - hz0) * share(curve, c0, c1, b) for b in at])
    ms = np.round(1e3 * np.concatenate([[0.0], np.cumsum(np.diff(at) / (0.5 * (hz[1:] + hz[:-1])))])).astype(int)
    gaps = np.diff(ms)
    rows = []
    for k, b in enumerate(at):
        p = [math.nan] * ortho.P
        p[ortho.GAIN], p[ortho.HZ] = 1.0, float(hz[k])
        for slot, (c, b0, b1) in bands.items():
            p[slot] = dry[slot] + (wet[slot] - dry[slot]) * share(c, b0, b1, b)
        rows.append(ortho.ROW.pack(ROW_ID, int(ms[k]), int(gaps[min(k, len(gaps) - 1)]),
                                   ortho.CURVE * (k + 1 < len(at)), *p))
    return b''.join(rows), float(ms[-1]) * 1e-3


def crossing(st, now):
    """The time of her lateral voice's next zero crossing, s: her clock's angle and the voice's
    phase carried at her rhythm's rate (`latent` THETA, PHASE, HZ)."""
    from machine import latent
    hz = float(st[latent.NOW][latent.HZ])
    psi = float(st[latent.THETA] + st[latent.NOW][latent.PHASE][modal.LATERAL])
    return now + ((-psi) % math.pi) / (math.tau * hz)


def window(st):
    """The double support past each crossing, s: pi - 2 stride.PHI_W of her clock's turn (her lateral
    voice 108 deg past her drive's, pi - PHI_W; measured her soles both stand 0 to 0.6 rad past it)."""
    from machine import latent
    return (math.pi - 2.0 * stride.PHI_W) / (math.tau * float(st[latent.NOW][latent.HZ]))


def locked(y, fr, a):
    """Whether a transition may begin now: both soles stand and the ZMP her law asks (her centre of
    mass less its asked acceleration over omega squared) lies in their boxes (`poise.boxes`: a sole
    less poise.MARGIN_M round it). Her capture point runs ahead of them walking, never in them."""
    from machine import poise
    rot = fr[modal.ROT].reshape(3, 3)
    c = rot @ y[modal.COM]
    zmp = c[:2] - (rot @ a[modal.COM_ACC])[:2] / float(fr[modal.OMEGA]) ** 2
    lo, hi = poise.boxes((y[modal.SOLE].reshape(2, 3) @ rot.T)[:, :2], a[modal.EDGE], y[modal.TILT])
    return bool(a[modal.STANCE].min() > 0.5 and (zmp >= lo.min(axis=0)).all() and (zmp <= hi.max(axis=0)).all())
