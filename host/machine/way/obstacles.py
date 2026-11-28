"""Each floor point's cost among the robot's things, of any kind (smooth_interactive_wayfinder.lisp).

    nodes = obstacles.laws()                                     # the wayfinder's texts threaded: {node: params}
    d = obstacles.apart(points, things)                          # (N, M) m from each floor point to each footprint
    cost = obstacles.costed(points, things, floor, seen, slot, face)   # (N,) a metre's cost through each, inf blocked
    obstacles.LAYERS[':name'] = (phase, layer)                   # an extension's layer, its name on the text's :layers

A thing is a box as the robot's registry holds it (kind, centre, half sizes, turn, slot, clear, swell): a lump
of the robot's stereo's voxels turned by its footprint's fit (`voxels.FITS`), or the stand-in's prop. A point is
blocked within a thing's footprint grown by the robot's reach across its way in the bands it meets
(`footprint.met`), half a cell and its swell - its blind spot's and its wake's; a free point costs 1 a metre and
each layer's. A layer `layer(field, node)` moves {points, things, reach, ahead, margin, kept, d, cost, kinds,
slots, centres, slot, face, seen}: a thing's first (phase 0), then a point's (phase 1), each in the text's
order - `:class-margin` a kind's margin on its reach (recognition's, the one layer reading a kind),
`:affordance-aperture` the thing the robot goes to opened before its face, `:clearance` w exp(-m / decay) at m
past the nearest grown footprint the robot keeps off, `:unknown` w where its eyes have seen nothing.
"""
import functools
import math
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine import libm
from machine.way import affordance
from machine.body import footprint
from machine.texts import kernel
from machine.eyes import voxels

#: The wayfinder's texts in order: the robot's base, then the specializers threaded after it - what the robot
#: goes to docked as furniture: its errands go to a seat, a bed and the door, the door no slot of the
#: robot's own (`machine.way.dock`).
STAGES = ('xform-smooth-interactive-stereo-navigator', 'specialize-nav-zero-distance-furniture-dock')
#: The robot's reach across its way: its walk's directions within a direction's step of the
#: robot's sides; its strides' lies along it. One radius for all (0.215 m) let the robot's hand meet a
#: chair's back.
ACROSS = [k for k in range(footprint.K)
          if abs(libm.cos(2.0 * math.pi * k / footprint.K)) <= libm.sin(2.0 * math.pi / footprint.K) + 1e-9]


@functools.cache
def laws(stages=STAGES):
    """{node: params} of the wayfinder's texts `stages` threaded over the robot's nominal gait in order."""
    from machine.texts import dsp
    got = kernel.read()
    g = got['nominal-biped-gait']
    for stage in stages:
        g = dsp.value([stage, g], got)
    return g


def boxes(things):
    """`things` that are boxes: a kind, a centre, half sizes and a turn at the least."""
    return [p for p in things if len(p) >= 4 and len(p[1]) >= 3 and np.size(p[2]) >= 3 and np.size(p[3]) == 9]


def apart(points, things):
    """(N, M) m from each floor point (N, 2) x, z to each of `things`' footprints, 0 within one: each point's offset
    turned into a footprint's axes numpy's einsum's sum written out - its two products added into a zero, the same
    bits (`apart_twin` the einsum) -, its loop 11 ms of a plan's 33."""
    (_lo, _hi, c, axes, half), _meetings = footprint._laid_things(things)
    from tools.cores import way
    got = way.apart(points, c, axes, half)
    if got is not None:
        return got
    c = c.reshape(-1, 2)
    axes = axes.reshape(-1, 2, 2)
    half = half.reshape(-1, 2)
    off = np.asarray(points, float)[:, None, :] - c[None]
    turned = (0.0 + axes[None, :, :, 0] * off[:, :, None, 0]) + axes[None, :, :, 1] * off[:, :, None, 1]
    out = np.maximum(np.abs(turned) - half[None], 0.0)
    return libm.hypot(out[..., 0], out[..., 1])


def apart_twin(points, things):
    """`apart` in numpy's einsum, as it was (test_wayfinder holds the two to the bit)."""
    prints = [affordance.footprint(t) for t in things]
    c = np.array([p[0] for p in prints], float).reshape(-1, 2)
    axes = np.array([p[1] for p in prints], float).reshape(-1, 2, 2)
    half = np.array([p[2] for p in prints], float).reshape(-1, 2)
    out = np.maximum(np.abs(np.einsum('mij,nmj->nmi', axes, np.asarray(points, float)[:, None, :] - c[None])) - half[None], 0.0)
    return libm.hypot(out[..., 0], out[..., 1])


def _class_margin(field, node):
    """Each thing's reach widened by its kind's margin (`:class-margins-mm`)."""
    margins = dict(node['class-margins-mm'] or {})
    field['margin'] = field['margin'] + 1e-3 * np.array([margins.get(k, 0.0) for k in field['kinds']], float)


def _clearance(field, node):
    """w exp(-m / decay) a metre at m past the nearest grown footprint of those the robot keeps off: its way
    kept off them where the room leaves it the width."""
    m = np.where(field['kept'][None, :], field['d'] - field['margin'], math.inf).min(axis=1, initial=math.inf)
    field['cost'] = field['cost'] + node['clearance-weight'] * libm.exp(-np.maximum(m, 0.0) / (1e-3 * node['clearance-decay-mm']))


def _unknown(field, node):
    """w a metre where the robot's eyes have seen nothing."""
    field['cost'] = field['cost'] + node['unknown-weight'] * (1.0 - field['seen'])


def _affordance_aperture(field, node):
    """The thing the robot goes to (`slot`) and the things standing within its footprint - a seat's legs -
    opened before its `face`, its outward normal: their margins at each point the blend by s = ((1 +
    cos t) / 2)^gamma, t the point's bearing from its centre off `face`, of the robot's reach across the
    robot's way (s 0: behind it, round its corners) and the robot's reach ahead with the text's standoff (s 1:
    before its face, walked in on head on) - `carve-anisotropic-affordance-docking-funnel`. The robot's
    swinging foot reaches 0.32 m ahead, 0.24 across: low, it opens nothing; from the robot's hips up its
    dock-approach reach ahead (`footprint.reach('dock')`, braked to `route.CORNER_SPEED` and stopped) is the less."""
    funnel = laws()['docking-funnel']
    goes = (field['slots'] == field['slot']) & (field['slot'] >= 0) & (field['face'] is not None) & (funnel['selector'] != ':none')
    if not goes.any():
        return
    k = int(np.argmax(goes))
    target = field['things'][k]
    hit = goes | (apart(field['centres'], [target])[:, 0] <= 0.0)
    off = field['points'] - np.array([target[1][0], target[1][2]], float)
    cos = off @ np.asarray(field['face'], float) / np.maximum(libm.hypot(off[:, 0], off[:, 1]), 1e-9)
    s = libm.power(0.5 * (1.0 + cos), funnel['cone-exponent'])
    margin = np.broadcast_to(field['margin'], (len(field['points']), len(hit))).copy()
    ahead = np.minimum(margin[0, hit], margin[0, hit] - field['reach'][hit] + field['ahead'][k] + funnel['touch-standoff-m'])
    margin[:, hit] = (1.0 - s[:, None]) * margin[:, hit] + s[:, None] * ahead[None, :]
    field['margin'] = margin


#: The text's layers by name (`:layers`): (its phase - 0 a thing's reach, 1 a point's cost -, the layer); an
#: extension's its own entry.
LAYERS = {':class-margin': (0, _class_margin), ':affordance-aperture': (1, _affordance_aperture),
          ':clearance': (1, _clearance), ':unknown': (1, _unknown)}
_BUILTIN = dict(LAYERS)
_APART = apart
_PREPARED: list = []


def _prepared(things, floor):
    """Cached `(m_things, reach, ahead, base_margin, kinds, slots, centres, fp_c, fp_axes, fp_half)` for `(things, floor)`."""
    for entry in _PREPARED:
        if entry[0] is things and entry[1] == floor:
            return entry[2]
    b_things = boxes(things)
    meets, toward = footprint.met(b_things, floor, footprint.reach('walk'))
    _d_meets, docked = footprint.met(b_things, floor, footprint.reach('dock'))
    m_things = [p for p, m in zip(b_things, meets) if m]
    (_lo, _hi, fp_c, fp_axes, fp_half), _meetings = footprint._laid_things(m_things)
    reach = toward[meets][:, ACROSS].max(axis=1, initial=0.0)
    ahead = docked[meets][:, 0]
    swell = np.array([float(p[6]) if len(p) > 6 else 0.0 for p in m_things], float)
    base_margin = np.ascontiguousarray(reach + 0.5 * voxels.CELL_M + swell, dtype=np.float64)
    kinds = [p[0] for p in m_things]
    slots = np.array([int(p[4]) if len(p) > 4 else -1 for p in m_things], np.int64)
    centres = np.ascontiguousarray([(p[1][0], p[1][2]) for p in m_things], dtype=np.float64).reshape(-1, 2) \
        if m_things else np.empty((0, 2), dtype=np.float64)
    got = (m_things, reach, ahead, base_margin, kinds, slots, centres, fp_c, fp_axes, fp_half)
    if len(_PREPARED) >= 8:
        _PREPARED.pop(0)
    _PREPARED.append((things, floor, got))
    return got


def costed(points, things, floor, seen, slot=-1, face=None):
    """(N,) a metre's cost walked through each floor point (N, 2) among `things` over `floor`: inf
    within a thing's grown footprint, else 1 and each of the text's layers (`laws()['costmap']`), a
    thing's before a point's; `seen` (N,) 1 where the robot's eyes have seen, `slot` the thing it goes
    to, `face` (x, z) the outward normal of its face the robot docks at, None for none."""
    node = laws()['costmap']
    m_things, reach, ahead, base_margin, kinds, slots, centres, fp_c, fp_axes, fp_half = _prepared(things, float(floor))
    ordered = sorted(node['layers'], key=lambda n: LAYERS[n][0])
    if apart is _APART and all(LAYERS.get(n) == _BUILTIN.get(n) for n in ordered):
        p1 = [n for n in ordered if LAYERS[n][0] == 1]
        if (p1.count(':clearance') <= 1 and p1.count(':unknown') <= 1
                and p1.count(':affordance-aperture') <= 1
                and (':affordance-aperture' not in p1 or ':clearance' not in p1
                     or p1.index(':affordance-aperture') < p1.index(':clearance'))):
            margin0 = base_margin
            if ':class-margin' in ordered:
                margins = dict(node['class-margins-mm'] or {})
                cm = 1e-3 * np.array([margins.get(k, 0.0) for k in kinds], float)
                for n in ordered:
                    if n == ':class-margin':
                        margin0 = margin0 + cm
            aperture = None
            if ':affordance-aperture' in p1:
                funnel = laws()['docking-funnel']
                goes = (slots == slot) & (slot >= 0) & (face is not None) & (funnel['selector'] != ':none')
                if goes.any():
                    k = int(np.argmax(goes))
                    target = m_things[k]
                    hit = goes | (apart(centres, [target])[:, 0] <= 0.0)
                    ahead_h = np.minimum(margin0, margin0 - reach + ahead[k] + funnel['touch-standoff-m'])
                    f_arr = np.asarray(face, float)
                    aperture = (float(target[1][0]), float(target[1][2]), float(f_arr[0]), float(f_arr[1]),
                                float(funnel['cone-exponent']), np.ascontiguousarray(hit, dtype=np.uint8),
                                np.ascontiguousarray(ahead_h, dtype=np.float64))
            has_clear = ':clearance' in p1
            has_unk = ':unknown' in p1
            clear_first = not (has_clear and has_unk and p1.index(':unknown') < p1.index(':clearance'))
            from tools.cores import way
            got = way.costed(
                points, fp_c, fp_axes, fp_half, margin0, aperture,
                (has_clear, node['clearance-weight'] if has_clear else 0.0,
                 1e-3 * node['clearance-decay-mm'] if has_clear else 1.0),
                (has_unk, node['unknown-weight'] if has_unk else 0.0), seen, clear_first,
            )
            if got is not None:
                return got
    points = np.asarray(points, float).reshape(-1, 2)
    field = {'points': points, 'things': m_things, 'reach': reach, 'ahead': ahead,
             'margin': base_margin[None, :], 'kept': np.ones(len(m_things), bool),
             'd': apart(points, m_things), 'cost': np.ones(len(points)), 'kinds': kinds,
             'slots': slots, 'centres': centres, 'slot': slot, 'face': face, 'seen': np.asarray(seen, float)}
    for name in ordered:
        LAYERS[name][1](field, node)
    return np.where((field['d'] <= field['margin']).any(axis=1), np.inf, field['cost'])
