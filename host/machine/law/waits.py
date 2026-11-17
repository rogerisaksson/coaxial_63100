"""The robot's table's rows that wait or wake it, held a pass on its law's arrays, in C bit for bit.

    waits.played_back(g, dt)   # rows not yet due wait while a move of its own plays (`joints.MOVES`)
    waits.backs(g, dt)         # held, risen, landed, sunk, borne and latched rows, a law pass's, in one call
    waits.woken_rows(tab, st, t, dt)   # the rows due wake its rhythm where it has died down

`g` is `sprung.laid`'s; the C is `tools.cores.wait` on `g['waits']` (`motion.laid`); `held_rows`,
`risen_rows`, `landed_rows`, `sunk_rows`, `borne_rows`, `latched_rows`, `played_rows` and `woken_rows`
are its python, the reference.
"""
import math
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine.law import impedance, latent, modal, poise, stride
from machine.moves import leap


def held_rows(tab, a, out, t, dt, held_s):
    """(shift, first, held_s) on the robot's table `tab` (its times shifted in place), modal's
    `a`, the latent `out`, at `t`, `dt` on, the robot's hold `held_s` s so far: rows that halt the
    robot's rhythm wait while its capture point steps it or a swing is under way. In C bit for bit
    (`g['waits']`, `laid`)."""
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


def risen_rows(tab, y, fr, a, st, t, dt, prop_until):
    """(shift, first, prop_until) on the robot's table `tab` (its times shifted in place),
    modal's `y`, `fr` and `a`, the robot's latent `st`, at `t`, `dt` on, `prop_until` when a prop last
    bore it: rows that raise the robot while something bears it off its soles wait while its capture
    point is off its soles. In C bit for bit (`g['waits']`, `laid`)."""
    # what bears the robot off its soles, its hands' share with it (`borne.latched`): risen off a
    # chair's seat on the robot's hands alone, its capture point off its soles, it was thrust
    # off one sole at 500 N
    bears = float(y[modal.PROP] + y[modal.PALMS].sum())
    has_prop = float(bears > 0.0)
    until = has_prop * (t + 0.06) + (1.0 - has_prop) * prop_until
    propped = float((bears > 0.0) | (t <= until))
    rot = fr[modal.ROT].reshape(3, 3)
    c, v = rot @ y[modal.COM], rot @ y[modal.VCOM]
    xi = c[:2] + v[:2] / math.sqrt(modal.G / max(float(c[2] - fr[modal.FLOOR]), 0.3))
    # over the soles that stand (`modal.STANCE`): a sole being placed, in the air, holds the robot's capture point up
    # nowhere
    off = poise.BIG * (1.0 - a[modal.STANCE])[:, None]
    lo, hi = poise.boxes((y[modal.SOLE].reshape(2, 3) @ rot.T)[:, :2], a[modal.EDGE], y[modal.TILT])
    # along the robot's way over its standing soles' middle - their boxes meaned by how each stands -, across
    # over either: over the rearmost heel, a sole slid back 3-10 cm let it thrust off a bed's end with the
    # robot's capture point 7.5-10 cm behind its soles' middle and it fell back onto it (7 that rose:
    # -0.042 to +0.047 m)
    stood = a[modal.STANCE]
    total = max(float(stood[0] + stood[1]), poise.TINY)
    w0, w1 = float(stood[0]) / total, float(stood[1]) / total
    outside = float((xi[0] < w0 * lo[0, 0] + w1 * lo[1, 0]) | (xi[0] > w0 * hi[0, 0] + w1 * hi[1, 0])
                    | (xi[1] < (lo[:, 1] + off[:, 0]).min()) | (xi[1] > (hi[:, 1] - off[:, 0]).max()))
    at, axial, sag = tab[:, latent.AT], modal.AXIAL, modal.SAGITTAL
    vals, said, to = tab[:, latent.VALS][:, latent.Z], tab[:, latent.SAID][:, latent.Z], st[latent.TO][latent.Z]
    zmp = np.where(tab[:, latent.SAID][:, latent.ZMP] > 0.5, tab[:, latent.VALS][:, latent.ZMP],
                   st[latent.TO][latent.ZMP]) > 0.5
    # a row raises the robot onto its soles - its ZMP in its support
    # (`acts.SUPPORT`) - with its centre of mass told from the robot's heels' edge on (the sagittal's -1); ZMP
    # let go, or behind them, is a recline or lie
    over = (np.where(said[:, sag] > 0.5, vals[:, sag], to[sag]) >= -1.0) & zmp
    rises = (said[:, axial] > 0.5) & (vals[:, axial] < min(float(to[axial]), 2.0) - 1e-6) & over
    due = rises & (at <= t + latent.EDGE_S) & (at > t - dt + latent.EDGE_S)
    shift = dt * propped * outside * float(np.any(due))
    first = float(np.min(at, where=due, initial=math.inf)) - latent.EDGE_S
    at += shift * (at >= first)
    return shift, first, until


def landed_rows(tab, y, st, t, dt):
    """(shift, first) on the robot's table `tab` (its times shifted in place), modal's `y`,
    the robot's latent `st`, at `t`, `dt` on: rows that lower the robot wait while nothing bears it.
    In C bit for bit (`g['waits']`, `laid`)."""
    airborne = float((y[modal.LOAD].max() < leap.TOUCH_N) & (y[modal.PROP] <= 0.0))
    at = tab[:, latent.AT]
    vals, said, to = tab[:, latent.VALS][:, latent.Z], tab[:, latent.SAID][:, latent.Z], st[latent.TO][latent.Z]
    lowers = (said[:, modal.AXIAL] > 0.5) & (vals[:, modal.AXIAL] > to[modal.AXIAL] + 1e-6)
    due = lowers & (at <= t + latent.EDGE_S) & (at > t - dt + latent.EDGE_S)
    shift = dt * airborne * float(np.any(due))
    first = float(np.min(at, where=due, initial=math.inf)) - latent.EDGE_S
    at += shift * (at >= first)
    return shift, first


def sunk_rows(tab, y, fr, t, dt):
    """(shift, first) on the robot's table `tab` (its times shifted in place), modal's `y` and `fr`,
    at `t`, `dt` on: rows that raise the robot out of a dip (`ortho.RISE`) wait while its centre of mass
    still sinks. In C bit for bit (`g['waits']`, `laid`)."""
    sinking = float(fr[modal.ROT].reshape(3, 3)[2] @ y[modal.VCOM] < 0.0)
    at = tab[:, latent.AT]
    due = (tab[:, latent.RISES] > 0.5) & (at <= t + latent.EDGE_S) & (at > t - dt + latent.EDGE_S)
    shift = dt * sinking * float(np.any(due))
    first = float(np.min(at, where=due, initial=math.inf)) - latent.EDGE_S
    at += shift * (at >= first)
    return shift, first


def backs(g, dt):
    """`held_rows`, `risen_rows`, `landed_rows`, `sunk_rows`, `borne_rows` and `latched_rows` in that order, the
    robot's law pass's: their C in one call (`g['waits'].backs`), its labels and its jump's times shifted after as
    each shifts them - nothing of them is read between -, a label through the five shifts in turn."""
    t = g['t']
    shifts, g['_held_s'], g['_prop_until'] = g['waits'].backs(g['tab'], t, dt, g.get('_held_s', 0.0),
                                                               g.get('_prop_until', -1.0))
    labels = []
    for w, n in g['labels']:
        for shift, first in shifts:
            w = w + shift * (w >= first)
        labels.append((w, n))
    g['labels'] = labels
    held, _risen, landed, sunk, _borne = (shift for shift, _first in shifts)
    g['_jump_until'] = g.get('_jump_until', 0.0) + held * (g.get('_jump_until', 0.0) > t)
    g['_jump_until'] = g['_jump_until'] + landed * (g['_jump_until'] > t)
    g['_jump_throw'] = g.get('_jump_throw', 0.0) + sunk * (g.get('_jump_throw', 0.0) > t)
    g['_jump_until'] = g['_jump_until'] + sunk * (g['_jump_until'] > t)


def played_back(g, dt):
    """The robot's text's rows not yet due wait a pass, every one, while a move of its own plays
    (`joints.MOVES`, what its senses phase, not the clock): the bed's rise laid its plant, lean and thrust
    on the clock under the move that scoots the robot to the edge, and at the move's end its law asked the
    robot's standing at rest and it slid off the edge."""
    shift, first = g['waits'].played(g['tab'], g['t'], dt)
    g['labels'] = [(w + shift * (w > first), n) for w, n in g['labels']]


def latched_rows(tab, y, fr, a, st, t, dt):
    """A latch (`ortho.LATCH`: a wake's) on the robot's latent `st` (in place), its table `tab`, modal's `y`,
    `fr` and `a`, at `t`, `dt` on. In C bit for bit (`g['waits']`)."""
    at = tab[:, latent.AT]
    due = (tab[:, latent.LATCH] > 0.5) & (at <= t + latent.EDGE_S) & (at > t - dt + latent.EDGE_S)
    latch, z = float(np.any(due)), modal.read(y, fr, a[modal.STANCE])[0]
    z[[modal.LATERAL, modal.DRIVE, modal.LIFT, modal.WIDTH, modal.REACH, modal.ELBOWS]] *= 1.0 - a[modal.BORNE]
    for part in (latent.NOW, latent.FROM, latent.TO):
        st[part][latent.Z] += latch * (z - st[part][latent.Z])
    for part in (latent.V0, latent.V1):
        st[part][latent.Z] *= 1.0 - latch


def borne_rows(tab, y, fr, t, dt):
    """(shift, first) of contact, plantar and palmar waits on the robot's table `tab` (its times shifted in place), modal's `y`
    and `fr`, at `t`, `dt` on. In C bit for bit (`g['waits']`)."""
    prop, palm, contact, at = float(y[modal.PROP]), float(y[modal.PALMS].sum()), tab[:, latent.CONTACT], tab[:, latent.AT]
    soles = (float(y[modal.LOAD][0]) + float(y[modal.LOAD][1])) / float(fr[modal.WEIGHT])
    due = (contact != 0.0) & (at <= t + latent.EDGE_S) & (at > t - dt + latent.EDGE_S)
    short = (((contact > 0.5) & (contact < 1.5) & (prop < impedance.PROP_ON)) | ((contact > 1.5) & (palm <= 0.0))
             | ((contact < 0.0) & (soles < -contact)))
    shift = dt * float(np.any(due & short))
    first = float(np.min(at, where=due, initial=math.inf)) - latent.EDGE_S
    at += shift * (at >= first)
    return shift, first


def played_rows(tab, t, dt):
    """(shift, first) of `played_back` on the robot's table `tab` (its times shifted in place) at `t`, `dt` on:
    the rows after `t` (`latent.EDGE_S` on) later by `dt`. In C bit for bit (`g['waits']`)."""
    at, first = tab[:, latent.AT], t + latent.EDGE_S
    at += dt * (at > first)
    return dt, first


def woken_rows(tab, st, t, dt):
    """The rows of `tab` due at `t` that ask a rhythm - their gain times their amplitude, its sway -
    wake the robot's own in `st` where its gain has died down: its angle let go to the one laid for it
    (`latent.THETA0`, kept till a row comes), its phases theirs. Woken by a
    row laid whole alone, the sway's 108 deg lead came in with its gain, its lateral mode 22 % fast,
    and its first extremes left a sole 2.0 mm up on 0.6 N where 1.8 on 2.9; by its amplitude alone,
    a stride leap's throw (gain 0) let the robot's angle go and the run's leap fell. In C bit for bit
    (`g['waits']`)."""
    hit = ((tab[:, latent.AT] <= t + latent.EDGE_S) & (tab[:, latent.AT] > t - dt + latent.EDGE_S)).astype(float)
    vals = tab[:, latent.VALS]
    w_row = (vals[:, latent.GAIN] * vals[:, latent.AMP].max(axis=1) > 0.05).astype(float)
    wake = min(1.0, max(0.0, (0.04 - float(st[latent.NOW][latent.GAIN])) / 0.02)) * min(1.0, float(hit @ w_row))
    st[latent.THETA] = st[latent.THETA] * (1.0 - wake) + wake * st[latent.THETA0]
    st[latent.THETA0] *= 1.0 - min(1.0, float(hit.sum()))
    w_ph = wake * np.minimum(hit @ tab[:, latent.SAID][:, latent.PHASE], 1.0)
    st[latent.NOW][latent.PHASE] += w_ph * (hit @ tab[:, latent.VALS][:, latent.PHASE] - st[latent.NOW][latent.PHASE])
