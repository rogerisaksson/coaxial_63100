"""The rows of her table that wait or wake her, held a pass on her law's arrays, in C bit for bit.

    waits.held_back(g, dt)     # rows that halt her rhythm wait while her capture point steps her
    waits.risen_back(g, dt)    # rows that raise her wait while something bears her off her soles
    waits.landed_back(g, dt)   # rows that lower her wait while nothing bears her
    waits.borne_back(g, dt)    # rows a contact releases wait till a seat or a bed bears her
    waits.woken_rows(tab, st, t, dt)   # the rows due wake her rhythm where it has died down

`g` is `sprung.laid`'s; the C is `tools.cores.wait` on `g['waits']` (`motion.laid`); `held_rows`,
`risen_rows`, `landed_rows` and `woken_rows` are its python, the reference.
"""
import math
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine import impedance, latent, leap, modal, poise, stride


def held_back(g, dt):
    """Rows that halt her rhythm - a stand, a hop's load - wait a pass while her capture point
    steps her or a swing is under way (`modal.NEED`, `modal.SWINGS`), every row after them, her
    labels and a hop's end with them: stepped through a hop's load, she left the floor rolling
    and came down at 60 deg. Held longer than two of her rhythm's swings (`latent.TURNS`) they
    come: the swing waited for is not coming - the brake's sway kept one commanded on two
    standing soles for ever, the stand that would end it never came, a sit asked held with it,
    and told to stop on ice she shuffled on 38 cm at 0.045 m/s. Each hold counts its own: a
    hop said 2.1 s after a stop, still stepping, loaded on one sole at once on the stand's. Her
    rhythm at rest, no swing comes: sitting, her centre of mass behind her heels, a seat asked
    was held for ever."""
    t = g['t']
    shift, first, g['_held_s'] = g['waits'].held(g['tab'], t, dt, g.get('_held_s', 0.0))
    g['labels'] = [(w + shift * (w >= first), n) for w, n in g['labels']]
    g['_jump_until'] = g.get('_jump_until', 0.0) + shift * (g.get('_jump_until', 0.0) > t)


def held_rows(tab, a, out, t, dt, held_s):
    """(shift, first, held_s) of `held_back` on her table `tab` (its times shifted in place), modal's
    `a`, the latent `out`, at `t`, `dt` on, her hold `held_s` s so far. In C bit for bit (`g['waits']`,
    `laid`)."""
    stepping = float((a[modal.NEED] > 0.0) | (a[modal.SWINGS].max() > 0.0))
    at = tab[:, latent.AT]
    halts = (tab[:, latent.SAID][:, latent.GAIN] > 0.5) & (tab[:, latent.VALS][:, latent.GAIN] < 0.05)
    due = halts & (at <= t + latent.EDGE_S) & (at > t - dt + latent.EDGE_S)
    active = stepping * float(np.any(due))
    held = active * held_s
    turns = float(out[latent.TURNS])
    within = float((turns > stride.TINY) & (held <= 4.0 * stride.PHI_W / max(turns, stride.TINY)))
    shift = dt * active * within
    first = float(np.min(at, where=due, initial=math.inf)) - latent.EDGE_S
    at += shift * (at >= first)
    return shift, first, held + active * dt


def risen_back(g, dt):
    """Rows that raise her - her height asked up, her centre of mass told over her soles - while
    something bears her off her soles wait a pass, every row after them with them, while her
    capture point is off her soles: thrust up off a bed on the clock, her centre of mass 14 cm
    behind her soles' middle, she rose, her trunk came up and she stepped back as it let her go."""
    shift, first, g['_prop_until'] = g['waits'].risen(g['tab'], g['t'], dt, g.get('_prop_until', -1.0))
    g['labels'] = [(w + shift * (w >= first), n) for w, n in g['labels']]


def risen_rows(tab, y, fr, a, st, t, dt, prop_until):
    """(shift, first, prop_until) of `risen_back` on her table `tab` (its times shifted in place),
    modal's `y`, `fr` and `a`, her latent `st`, at `t`, `dt` on, `prop_until` when a prop last
    bore her. In C bit for bit (`g['waits']`, `laid`)."""
    has_prop = float(y[modal.PROP] > 0.0)
    until = has_prop * (t + 0.06) + (1.0 - has_prop) * prop_until
    propped = float((y[modal.PROP] > 0.0) | (t <= until))
    rot = fr[modal.ROT].reshape(3, 3)
    c, v = rot @ y[modal.COM], rot @ y[modal.VCOM]
    xi = c[:2] + v[:2] / math.sqrt(modal.G / max(float(c[2] - fr[modal.FLOOR]), 0.3))
    lo, hi = poise.boxes((y[modal.SOLE].reshape(2, 3) @ rot.T)[:, :2], a[modal.EDGE], y[modal.TILT])
    outside = float(np.any((xi < lo.min(axis=0)) | (xi > hi.max(axis=0))))
    at, axial, sag = tab[:, latent.AT], modal.AXIAL, modal.SAGITTAL
    vals, said, to = tab[:, latent.VALS][:, latent.Z], tab[:, latent.SAID][:, latent.Z], st[latent.TO][latent.Z]
    # a row raises her over her soles with its centre of mass told from her heels' edge on (the
    # sagittal's -1); told behind them her legs straighten lying, and would wait for nothing
    over = np.where(said[:, sag] > 0.5, vals[:, sag], to[sag]) >= -1.0
    rises = (said[:, axial] > 0.5) & (vals[:, axial] < min(float(to[axial]), 2.0) - 1e-6) & over
    due = rises & (at <= t + latent.EDGE_S) & (at > t - dt + latent.EDGE_S)
    shift = dt * propped * outside * float(np.any(due))
    first = float(np.min(at, where=due, initial=math.inf)) - latent.EDGE_S
    at += shift * (at >= first)
    return shift, first, until


def landed_back(g, dt):
    """Rows that lower her - her height asked down - wait a pass while nothing bears her, soles or
    prop, every row after them and a hop's end with them: a jump's absorption on the clock met
    her soles 0.1 s early, her legs long on the told stand, and she bounced off them to a
    second fall."""
    shift, first = g['waits'].landed(g['tab'], g['t'], dt)
    g['labels'] = [(w + shift * (w >= first), n) for w, n in g['labels']]
    g['_jump_until'] = g.get('_jump_until', 0.0) + shift * (g.get('_jump_until', 0.0) > g['t'])


def landed_rows(tab, y, st, t, dt):
    """(shift, first) of `landed_back` on her table `tab` (its times shifted in place), modal's `y`,
    her latent `st`, at `t`, `dt` on. In C bit for bit (`g['waits']`, `laid`)."""
    airborne = float((y[modal.LOAD].max() < leap.TOUCH_N) & (y[modal.PROP] <= 0.0))
    at = tab[:, latent.AT]
    vals, said, to = tab[:, latent.VALS][:, latent.Z], tab[:, latent.SAID][:, latent.Z], st[latent.TO][latent.Z]
    lowers = (said[:, modal.AXIAL] > 0.5) & (vals[:, modal.AXIAL] > to[modal.AXIAL] + 1e-6)
    due = lowers & (at <= t + latent.EDGE_S) & (at > t - dt + latent.EDGE_S)
    shift = dt * airborne * float(np.any(due))
    first = float(np.min(at, where=due, initial=math.inf)) - latent.EDGE_S
    at += shift * (at >= first)
    return shift, first


def borne_back(g, dt):
    """Rows a contact releases (`ortho.CONTACT`: a seat's or a bed's cross-fade) wait a pass, every
    row after them with them, while the furniture bears less of her than her support counts it from
    (`impedance.PROP_ON`): laid from her descent's start they meet the seat where it is. Rows her
    soles' share releases (`ortho.SOLES`: a rise's cross-fade onto them) wait while the furniture
    bears more of her than the rest: crossed on the clock off a seat bearing half of her, her ZMP
    engaged asked a step and she fell. Rows her hands release (`ortho.PALMED`: a hitch's press) wait
    till each bears (`modal.PALMS`): pressed on the clock, the first hand down took her and the other
    hung 5 cm over the seat."""
    shift, first = g['waits'].borne(g['tab'], g['t'], dt)
    g['labels'] = [(w + shift * (w >= first), n) for w, n in g['labels']]


def latched_back(g, dt):
    """A row that latches her (`ortho.LATCH`: a wake's) sets her vector where she is read at its
    time (`modal.read`), going nowhere: her gains come back on no error - woken with her trunk 0.7
    rad back of its last word, its spring threw her off the chair at twice her weight."""
    g['waits'].latched(g['tab'], g['t'], dt)


def latched_rows(tab, y, fr, a, st, t, dt):
    """`latched_back` on her latent `st` (in place), her table `tab`, modal's `y`, `fr` and `a`, at
    `t`, `dt` on. In C bit for bit (`g['waits']`)."""
    at = tab[:, latent.AT]
    due = (tab[:, latent.LATCH] > 0.5) & (at <= t + latent.EDGE_S) & (at > t - dt + latent.EDGE_S)
    latch, z = float(np.any(due)), modal.read(y, fr, a[modal.STANCE])[0]
    z[[modal.LATERAL, modal.DRIVE, modal.LIFT, modal.WIDTH, modal.REACH, modal.ELBOWS]] *= 1.0 - a[modal.BORNE]
    for part in (latent.NOW, latent.FROM, latent.TO):
        st[part][latent.Z] += latch * (z - st[part][latent.Z])
    for part in (latent.V0, latent.V1):
        st[part][latent.Z] *= 1.0 - latch


def borne_rows(tab, y, t, dt):
    """(shift, first) of `borne_back` on her table `tab` (its times shifted in place), modal's `y`,
    at `t`, `dt` on. In C bit for bit (`g['waits']`)."""
    prop, palm, contact, at = float(y[modal.PROP]), float(y[modal.PALMS].min()), tab[:, latent.CONTACT], tab[:, latent.AT]
    due = (contact != 0.0) & (at <= t + latent.EDGE_S) & (at > t - dt + latent.EDGE_S)
    short = (((contact > 0.5) & (contact < 1.5) & (prop < impedance.PROP_ON)) | ((contact > 1.5) & (palm <= 0.0))
             | ((contact < 0.0) & (prop > 1.0 + contact)))
    shift = dt * float(np.any(due & short))
    first = float(np.min(at, where=due, initial=math.inf)) - latent.EDGE_S
    at += shift * (at >= first)
    return shift, first


def woken_rows(tab, st, t, dt):
    """The rows of `tab` due at `t` that ask a rhythm - their gain times their amplitude, its sway -
    wake hers in `st` where its gain has died down: its angle let go, its phases theirs. Woken by a
    row laid whole alone, the sway's 108 deg lead came in with its gain, its lateral mode 22 % fast,
    and its first extremes left a sole 2.0 mm up on 0.6 N where 1.8 on 2.9; by its amplitude alone,
    a stride leap's throw (gain 0) let her angle go and the run's leap fell. In C bit for bit
    (`g['waits']`)."""
    hit = ((tab[:, latent.AT] <= t + latent.EDGE_S) & (tab[:, latent.AT] > t - dt + latent.EDGE_S)).astype(float)
    vals = tab[:, latent.VALS]
    w_row = (vals[:, latent.GAIN] * vals[:, latent.AMP].max(axis=1) > 0.05).astype(float)
    wake = min(1.0, max(0.0, (0.04 - float(st[latent.NOW][latent.GAIN])) / 0.02)) * min(1.0, float(hit @ w_row))
    st[latent.THETA] *= 1.0 - wake
    w_ph = wake * np.minimum(hit @ tab[:, latent.SAID][:, latent.PHASE], 1.0)
    st[latent.NOW][latent.PHASE] += w_ph * (hit @ tab[:, latent.VALS][:, latent.PHASE] - st[latent.NOW][latent.PHASE])
