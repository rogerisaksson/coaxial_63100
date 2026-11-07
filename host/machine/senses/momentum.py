"""Her momentum observer laid in C once on her law's body: its points, pushes, stops and capsules.

    obs = momentum.laid(body)       # `observer.laid`: the observer at rest, its outputs viewed

What `machine.senses.observer` splits her external force over, laid in wbc_gmo.c's struct at her law's laying: each
of her registry's points on its link with its pushes' edges, her joints' stops, her capsules and the pairs of them
MuJoCo collides, each manifold's shares; her forefeet compiled from their numpy (`tools.cores.bodied`).
"""
import math
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine.texts import kernel
from machine.body.figure import JOINTS
from machine.stack.wbc import G
from tools.cores.model import rotation


def _link(seg):
    """A segment's link in her chain (wbc_model.h): its last joint's, the pelvis the root."""
    from machine.body.figure import SEGMENTS
    joints = next(j for name, _p, j, *_r in SEGMENTS if name == seg)
    return joints[-1][0] if joints else 'root'


def _capsules(m):
    """(her capsules, each (segment, radius, its axis's ends in its frame), the pairs of them MuJoCo collides) of
    her model `m`'s colliding geoms: a sphere a capsule of no length, a box one along its longest side as thick as
    its thinnest; a pair on two bodies not welded, not parent and child, not excluded, their bits meeting."""
    from machine.body.figure import SEGMENTS
    segs, kinds = {s[0] for s in SEGMENTS}, m.geom_type
    geoms = [g for g in range(m.ngeom) if m.body(m.geom_bodyid[g]).name in segs and kinds[g] in _AXES]
    out = []
    for g in geoms:
        R, size = rotation(m.geom_quat[g]), m.geom_size[g]
        r, k, half = _AXES[kinds[g]](size)
        out.append((m.body(m.geom_bodyid[g]).name, float(r), *(tuple(m.geom_pos[g] + s * half * R[:, k]) for s in (-1, 1))))
    weld = m.body_weldid
    up = lambda b: weld[m.body_parentid[weld[b]]]
    excluded = {int(s) for s in m.exclude_signature}
    meet = lambda a, b: (m.geom_contype[a] & m.geom_conaffinity[b]) | (m.geom_contype[b] & m.geom_conaffinity[a])
    pairs = [(i, j) for i, a in enumerate(geoms) for j, b in enumerate(geoms) if i < j
             and weld[m.geom_bodyid[a]] != weld[m.geom_bodyid[b]]
             and weld[m.geom_bodyid[a]] != up(m.geom_bodyid[b]) and weld[m.geom_bodyid[b]] != up(m.geom_bodyid[a])
             and (min(m.geom_bodyid[a], m.geom_bodyid[b]) << 16) + max(m.geom_bodyid[a], m.geom_bodyid[b]) not in excluded
             and meet(a, b)]
    return out, pairs


#: Each colliding geom's kind (mjtGeom): its capsule's (radius, axis, half its axis's length) of its sizes.
_AXES = {2: lambda s: (s[0], 2, 0.0), 3: lambda s: (s[0], 2, s[1]), 5: lambda s: (s[0], 2, s[1]),
         6: lambda s: (min(s), int(np.argmax(s)), max(s) - min(s))}


def laid(body):
    """The observer at rest on her law's `body` (`wbc.Body`), in C (wbc_gmo.c): its points, shares, her
    joints' stops, her capsules and her drives' friction laid, its sums empty; its outputs viewed (`r`,
    `f`, `normal`); her forefeet compiled from their numpy (`tools.cores.bodied`)."""
    import ctypes
    from machine.senses import observer as ob
    from tools.cores import bodied, gmo, wbc as cores
    lib, names = cores.library(), cores.Core().names
    o = gmo.made(horizon=gmo.horizon())
    o.slip, o.eps = ob.SLIP_RAD_S, ob.FLOOR
    o.weight, o.upright, o.reach, o.still = body.mass * G, ob.UPRIGHT_COS, ob.REACH_M, ob.STILL_M_S
    o.alpha = kernel.said('configure-momentum-observer', 'observer-bandwidth-hz') * 2.0 * math.pi
    o.loss[:] = body.m.dof_frictionloss[:body.n].tolist()
    driven = [k for k, j in enumerate(JOINTS) if body.driven[k]]
    o.driven_joint[:] = driven
    o.driven_at[:] = [5 + names.index(JOINTS[k]) for k in driven]
    points, edges, manifolds, rows = ob.POINTS, ob.EDGES, ob.MANIFOLDS, ob.ROWS
    cols = [sum(len(e) for e in edges[:p]) for p in range(len(points))]
    o.pushes = cols[-1] + len(edges[-1])
    if o.pushes > gmo.GMO_PUSH:
        raise ValueError('her points push %d ways, wbc_gmo.h at most %d' % (o.pushes, gmo.GMO_PUSH))
    for p, (shares, seg, point, normal) in enumerate(points):
        o.link[p], o.edges[p], o.col[p] = names.index(_link(seg)), len(edges[p]), cols[p]
        o.world[p], o.soled[p] = int(shares[0][0] in ob.UPRIGHT), int(shares[0][0] == 'plantar-bilateral')
        o.local[p][:] = list(point)
        for e, along in enumerate(edges[p]):
            o.edge[p][e][:] = list(along)
    for h, name in enumerate(names[1:]):
        j = body.m.joint(name).id
        o.stopped[h] = int(body.m.jnt_limited[j])
        o.stop_lo[h], o.stop_hi[h] = (float(v) for v in body.m.jnt_range[j])
    capsules, pairs = _capsules(body.m)
    o.capsules, o.pairs = len(capsules), len(pairs)
    for c, (seg, r, a, b) in enumerate(capsules):
        o.capsule_link[c], o.capsule_r[c] = names.index(_link(seg)), r
        o.capsule_a[c][:], o.capsule_b[c][:] = list(a), list(b)
    for k, pair in enumerate(pairs):
        o.pair[k][:] = list(pair)
    shared = [(p, manifolds.index(m), names.index(_link(frame)), axis)
              for p, (shares, *_r) in enumerate(points) for m, frame, axis in shares]
    for k, (p, m, link, axis) in enumerate(shared):
        o.share_point[k], o.share_manifold[k], o.share_link[k] = p, m, link
        o.share_axis[k][:] = np.asarray(axis, float).tolist()
    view = lambda field: np.ctypeslib.as_array(getattr(o, field))
    row = np.full(len(manifolds), -1)
    row[rows] = np.arange(len(rows))
    row = row[[manifolds.index(sh[0][0]) for sh, *_r in points]]
    col = lambda seg: cols[next(p for p, (_sh, at, _p, _n) in enumerate(points) if at == seg)]
    base, q, u = np.zeros(12), np.zeros(body.n - 6), np.zeros(body.n)
    return {'c': o, 'at': ctypes.byref(o), 'felt': lib.wbh_gmo_felt, 'step': lib.wbh_gmo_step,
            'r': view('r'), 'f': view('f'), 'normal': view('force'), 'points': view('at'), 'cols': np.array(cols),
            'pushes': o.pushes, 'gated': view('gated'), 'slide': view('slide'),
            'latch_of': np.array([[any(m == manifolds[k] for m, *_r in sh) for k in rows] for sh, *_r in points], float),
            'soled': np.array([sh[0][0] == 'plantar-bilateral' for sh, *_r in points]),
            'skin': np.array([seg not in ob.SOLED for _sh, seg, *_r in points]) & (row >= 0), 'row': row,
            'feet': [(col(side + '_foot'), col(side + '_toes')) for side in ('left', 'right')],
            'forefoot': bodied.on_args(ob.forefoot, 'forefoot', (0,)), 'base': base, 'q': q, 'u': u,
            'ins': tuple(cores.pointer(a) for a in (base, q, u)), 'soles': np.zeros(2),
            'latched': np.zeros(len(rows)), 'below': np.zeros(len(rows)), 'since': np.zeros(len(rows)),
            'gate': np.zeros(len(rows))}
