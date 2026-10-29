"""Each floor point's cost among her things, whatever they are (smooth_interactive_wayfinder.lisp).

    nodes = obstacles.laws()                                     # the wayfinder's texts threaded: {node: params}
    d = obstacles.apart(points, things)                          # (N, M) m from each floor point to each footprint
    cost = obstacles.costed(points, things, floor, seen, slot, face)   # (N,) a metre's cost through each, inf blocked
    obstacles.LAYERS[':name'] = (phase, layer)                   # an extension's layer, its name on the text's :layers

A thing is a box as her registry holds it (kind, centre, half sizes, turn, slot, clear, swell): a lump of
her stereo's voxels turned by its footprint's fit (`voxels.FITS`), or the stand-in's prop. A point is
blocked within a thing's footprint grown by her reach across her way in the bands it meets
(`footprint.met`), half a cell and its swell - its blind spot's and its wake's; a free point costs 1 a
metre and each layer's. A layer `layer(field, node)` moves {points, things, reach, ahead, margin, kept, d,
cost, kinds, slots, centres, slot, face, seen}: a thing's first (phase 0), then a point's (phase 1), each in the
text's order - `:class-margin` a kind's margin on its reach (recognition's, the one layer reading a kind),
`:affordance-aperture` the thing she goes to opened before its face, `:clearance` w exp(-m / decay) at m
past the nearest grown footprint she keeps off, `:unknown` w where her eyes have seen nothing.
"""
import functools
import math
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine import affordance, footprint, kernel, voxels

#: The wayfinder's texts in order: her base, then the specializers threaded after it - what she goes to docked as
#: furniture: her errands go to a seat, a bed and the door, the door no slot of hers (`machine.dock`).
STAGES = ('xform-smooth-interactive-stereo-navigator', 'specialize-nav-zero-distance-furniture-dock')
#: Her reach across her way: her walk's directions within a direction's step of her sides; her strides' lies
#: along it. One radius for all (0.215 m) let her hand meet a chair's back.
ACROSS = [k for k in range(footprint.K)
          if abs(math.cos(2.0 * math.pi * k / footprint.K)) <= math.sin(2.0 * math.pi / footprint.K) + 1e-9]


@functools.cache
def laws(stages=STAGES):
    """{node: params} of the wayfinder's texts `stages` threaded over her nominal gait in order."""
    from machine import dsp
    got = kernel.read()
    g = got['nominal-biped-gait']
    for stage in stages:
        g = dsp.value([stage, g], got)
    return g


def boxes(things):
    """`things` that are boxes: a kind, a centre, half sizes and a turn at the least."""
    return [p for p in things if len(p) >= 4 and len(p[1]) >= 3 and np.size(p[2]) >= 3 and np.size(p[3]) == 9]


def apart(points, things):
    """(N, M) m from each floor point (N, 2) x, z to each of `things`' footprints, 0 within one."""
    prints = [affordance.footprint(t) for t in things]
    c = np.array([p[0] for p in prints], float).reshape(-1, 2)
    axes = np.array([p[1] for p in prints], float).reshape(-1, 2, 2)
    half = np.array([p[2] for p in prints], float).reshape(-1, 2)
    out = np.maximum(np.abs(np.einsum('mij,nmj->nmi', axes, np.asarray(points, float)[:, None, :] - c[None])) - half[None], 0.0)
    return np.hypot(out[..., 0], out[..., 1])


def _class_margin(field, node):
    """Each thing's reach widened by its kind's margin (`:class-margins-mm`)."""
    margins = dict(node['class-margins-mm'] or {})
    field['margin'] = field['margin'] + 1e-3 * np.array([margins.get(k, 0.0) for k in field['kinds']], float)


def _clearance(field, node):
    """w exp(-m / decay) a metre at m past the nearest grown footprint of those she keeps off: her way
    kept off them where the room leaves her the width."""
    m = np.where(field['kept'][None, :], field['d'] - field['margin'], math.inf).min(axis=1, initial=math.inf)
    field['cost'] = field['cost'] + node['clearance-weight'] * np.exp(-np.maximum(m, 0.0) / (1e-3 * node['clearance-decay-mm']))


def _unknown(field, node):
    """w a metre where her eyes have seen nothing."""
    field['cost'] = field['cost'] + node['unknown-weight'] * (1.0 - field['seen'])


def _affordance_aperture(field, node):
    """The thing she goes to (`slot`) and the things standing within its footprint - a seat's legs -
    opened before its `face`, its outward normal: their margins at each point the blend by s = ((1 +
    cos t) / 2)^gamma, t the point's bearing from its centre off `face`, of her reach across her way (s
    0: behind it, round its corners) and her reach ahead with the text's standoff (s 1: before its face,
    walked in on head on) - `carve-anisotropic-affordance-docking-funnel`. Her swinging foot reaches
    0.32 m ahead, 0.24 across: low, it opens nothing; from her hips up her reach ahead is the less."""
    funnel = laws()['docking-funnel']
    goes = (field['slots'] == field['slot']) & (field['slot'] >= 0) & (field['face'] is not None) & (funnel['selector'] != ':none')
    if not goes.any():
        return
    target = field['things'][int(np.argmax(goes))]
    hit = goes | (apart(field['centres'], [target])[:, 0] <= 0.0)
    off = field['points'] - np.array([target[1][0], target[1][2]], float)
    cos = off @ np.asarray(field['face'], float) / np.maximum(np.hypot(off[:, 0], off[:, 1]), 1e-9)
    s = (0.5 * (1.0 + cos)) ** funnel['cone-exponent']
    margin = np.broadcast_to(field['margin'], (len(field['points']), len(hit))).copy()
    ahead = margin[0, hit] - field['reach'][hit] + field['ahead'][hit] + funnel['touch-standoff-m']
    margin[:, hit] = (1.0 - s[:, None]) * margin[:, hit] + s[:, None] * ahead[None, :]
    field['margin'] = margin


#: The text's layers by name (`:layers`): (its phase - 0 a thing's reach, 1 a point's cost -, the layer); an
#: extension's its own entry.
LAYERS = {':class-margin': (0, _class_margin), ':affordance-aperture': (1, _affordance_aperture),
          ':clearance': (1, _clearance), ':unknown': (1, _unknown)}


def costed(points, things, floor, seen, slot=-1, face=None):
    """(N,) a metre's cost walked through each floor point (N, 2) among `things` over `floor`: inf
    within a thing's grown footprint, else 1 and each of the text's layers (`laws()['costmap']`), a
    thing's before a point's; `seen` (N,) 1 where her eyes have seen, `slot` the thing she goes to,
    `face` (x, z) the outward normal of its face she docks at, None for none."""
    node = laws()['costmap']
    things = boxes(things)
    meets, toward = footprint.met(things, floor, footprint.reach('walk'))
    things = [p for p, m in zip(things, meets) if m]
    reach = toward[meets][:, ACROSS].max(axis=1, initial=0.0)
    swell = np.array([float(p[6]) if len(p) > 6 else 0.0 for p in things], float)
    points = np.asarray(points, float).reshape(-1, 2)
    field = {'points': points, 'things': things, 'reach': reach, 'ahead': toward[meets][:, 0],
             'margin': (reach + 0.5 * voxels.CELL_M + swell)[None, :], 'kept': np.ones(len(things), bool),
             'd': apart(points, things), 'cost': np.ones(len(points)), 'kinds': [p[0] for p in things],
             'slots': np.array([int(p[4]) if len(p) > 4 else -1 for p in things], np.int64),
             'centres': np.array([(p[1][0], p[1][2]) for p in things], float).reshape(-1, 2), 'slot': slot,
             'face': face, 'seen': np.asarray(seen, float)}
    for name in sorted(node['layers'], key=lambda n: LAYERS[n][0]):
        LAYERS[name][1](field, node)
    return np.where((field['d'] <= field['margin']).any(axis=1), np.inf, field['cost'])
