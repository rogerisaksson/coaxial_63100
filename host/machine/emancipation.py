"""A limb released on purpose from what bears her: its gate and its cascade.

intentional_limb_emancipation.lisp on her loop.

    gamma = emancipation.step(g, s, told, weighed, dt)     # a pass: g['emancipated'] her legs' release, 0 to 1
    eta, why = emancipation.gate(s, weighed, weight, leg)  # the physics' leave to release, and its three parts

A leg is released as far as her text asks it (its soles told nothing, `iota`) and her physics allows it (`eta`): her
centre still within the Froude number her text locks out (`release-gate`'s max-allowable-froude, her leg her length),
what bears her besides her soles told no less than its minimum sum, and her capture point inside the polygon of the
contacts that bear her besides her soles by the text's least margin, of her weight. Released, its cascade's windows
(`unloading-cascade`): her observer lets the sole go as an anchor first, then her step sheds its load, then the leg
is free once its sole bears under the text's floor of her weight.
"""
import math
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine import borne, kernel, maneuver, modal, ortho

#: The gate, the cascade and the open chain as her text lays them (`xform-intentional-limb-emancipation`).
GATE = kernel.node('xform-intentional-limb-emancipation', 'release-gate')
CASCADE = kernel.node('xform-intentional-limb-emancipation', 'unloading-cascade')
OPEN = kernel.node('xform-intentional-limb-emancipation', 'open-chain')['open-chain-impedance']
#: Her Froude number's lockout, the least share of her weight her other supports are told, the least margin, N m.
FROUDE, ANCHORS, MARGIN_NM = GATE['max-allowable-froude'], GATE['min-complement-weight-sum'], GATE['min-residual-cone-margin']
#: The cascade's slew, s, its observer and load-shedding windows in its release, and the floor of her weight under
#: which a shed sole is free.
SLEW_S = 1e-3 * CASCADE['slew-duration-ms']
UNANCHOR, SHED = CASCADE['observer-unanchor-window'], CASCADE['qp-load-shed-window']
FREED = CASCADE['kinematic-release-fz-floor']
#: Her leg, m: what her Froude number is over.
LEG_M = maneuver._manifest()[1]['leg']
#: The comparator's release share of her weight (`observer.RELEASE`): a sole told less than it is told nothing.
TOLD_NOTHING = borne.PROP_RELEASE
PLANTAR = ortho.MANIFOLDS.index('plantar')


def smooth(x):
    """The smooth step of `x` over [0, 1]."""
    u = np.clip(x, 0.0, 1.0)
    return u * u * (3.0 - 2.0 * u)


def window(gamma, span):
    """How far `gamma` is through its cascade window `span` (lo, hi), 0 to 1."""
    return np.clip((gamma - span[0]) / (span[1] - span[0]), 0.0, 1.0)


def margin(point, pts):
    """m `point` (2,) lies inside the convex hull of `pts` (n, 2), its nearest edge's distance; outside, less than
    0 by its distance to the hull; with under three points, its distance to them, less than 0."""
    pts = np.asarray(pts, float).reshape(-1, 2)
    if len(pts) < 3:
        return -float(np.min(np.linalg.norm(pts - point, axis=1))) if len(pts) else -math.inf
    hull = _hull(pts)
    if len(hull) < 3:
        return -float(np.min(np.linalg.norm(pts - point, axis=1)))
    edges = np.roll(hull, -1, axis=0) - hull
    normal = np.stack([edges[:, 1], -edges[:, 0]], axis=1) / np.linalg.norm(edges, axis=1)[:, None]
    inward = normal * np.sign(np.sum(normal * (hull.mean(axis=0) - hull), axis=1))[:, None]
    signed = np.sum(inward * (point - hull), axis=1)
    if (signed >= 0.0).all():
        return float(signed.min())
    t = np.clip(np.sum((point - hull) * edges, axis=1) / np.sum(edges * edges, axis=1), 0.0, 1.0)
    return -float(np.min(np.linalg.norm(hull + t[:, None] * edges - point, axis=1)))


def _hull(pts):
    """The convex hull of `pts` (n, 2), counterclockwise (Andrew's monotone chain)."""
    p = sorted(map(tuple, np.round(pts, 9)))
    cross = lambda o, a, b: (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for q in p:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], q) <= 0.0:
            lower.pop()
        lower.append(q)
    for q in reversed(p):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], q) <= 0.0:
            upper.pop()
        upper.append(q)
    return np.array(lower[:-1] + upper[:-1])


def gate(s, weighed, weight, floor):
    """(eta, (froude's, anchors', margin's), margin N m): the physics' leave to release her legs - her centre's
    Froude number under the lockout, what bears her besides her soles weighed at its least sum or over, her capture
    point inside the contacts that bear her besides her soles (`s['contacts']`, a let-go foot not among them) by the
    least margin of her weight - `floor` the floor's height, m."""
    v, c = np.asarray(s['vcom'], float), np.asarray(s['com'], float)
    froude = (v[0] * v[0] + v[2] * v[2]) / (modal.G * LEG_M)
    by_froude = math.exp(-(froude / FROUDE) ** 2)
    others = float(np.sum(weighed)) - float(weighed[PLANTAR])
    by_anchors = float(smooth((others - ANCHORS) / (1.0 - ANCHORS)))
    om = math.sqrt(modal.G / max(float(c[1]) - floor, 1e-3))
    xi = np.array([c[0] + v[0] / om, c[2] + v[2] / om])
    pts = [np.array([c_['p'][0], c_['p'][2]]) for c_ in s.get('contacts', ()) if c_.get('patch', 0) < borne.LET]
    by_margin_nm = weight * margin(xi, pts) if pts else -math.inf
    by_margin = float(smooth((by_margin_nm - MARGIN_NM) / MARGIN_NM))
    return by_froude * by_anchors * by_margin, (by_froude, by_anchors, by_margin), by_margin_nm


def intent(told):
    """How far her text asks her legs released, 0 to 1: her soles told under the comparator's release share of
    what it tells bears her - told nothing - while it tells something else bears her."""
    total = float(np.sum(told))
    off = (total - float(told[PLANTAR])) / max(total, 1e-12)
    return float(np.clip(1.0 - (float(told[PLANTAR]) / max(total, 1e-12)) / TOLD_NOTHING, 0.0, 1.0)) * float(off > 0.0)


def step(g, s, told, weighed, dt):
    """Her legs' release `dt` s on (g['emancipated'], 0 to 1 each), slewed over the cascade's SLEW_S toward her
    text's intent times her physics' leave; `g['emancipation']` the parts as they were this pass."""
    weight, floor = float(g['fr'][modal.WEIGHT]), float(g['fr'][modal.FLOOR])
    iota = intent(told)
    eta, parts, margin_nm = gate(s, weighed, weight, floor)
    was = g.get('emancipated', np.zeros(2))
    gamma = was + np.clip(iota * eta - was, -dt / SLEW_S, dt / SLEW_S)
    g['emancipated'] = gamma
    g['emancipation'] = {'iota': iota, 'eta': eta, 'parts': parts, 'margin_nm': margin_nm}
    return gamma
