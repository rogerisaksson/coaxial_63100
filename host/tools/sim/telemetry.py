"""The autotuner's telemetry: a root run on her, the text's six classes sampled as residuals.

    out = telemetry.ran('sit_chair', {(path, start, end): value})   # {r, kind, measures, form, ..}
    r = telemetry.residuals(measures, tune.grammar(), bounds)       # the text's order, each dimensionless

A run (RUNS) is a gait on her walk (`suite.played`, 20 s), a stunt in the first scenario that says
it (`scenarios.ran`), a schedule to its first refusal (`suite.schedule`), on the suite's texts as
the run splices them (`suite_said`). From SETTLE_S, or the stunt's saying, her world is sampled each
pass over whatever bears her (`sampler`); each class a residual in its dimension (CHANNELS).
"""
import contextlib
import math
import os
import tempfile
import time

from coaxial.model.blocks import numpy as np  # behind the OpenBLAS cap

from tools.sim import look, looks, strides, suite

#: Each class's measure of hers, how it is a residual, and its bound: None the class's invariant
#: (the manifold's), a name the manifold's plantar boundary's or her weight (N: J/m over it her cost
#: of transport), a number her form's landing.
CHANNELS = {'com-wrench-balance': ('pivot', 'short', None), 'normal-force-floor': ('load', 'short', None),
            'tangential-shear-slip': ('slip', 'over', 'support-polygon-x-mm'),
            'spatial-sdf-clearance': ('gap', 'short', None),
            'contact-transient-ringdown': ('peak', 'past', looks.LANDS),
            'actuator-thermal-passivity': ('energy', 'over', 'weight')}
#: A measure as a residual, its bound's share: its mean shortfall under the bound, how far its
#: most is past it, its size.
HOW = {'short': lambda x, b: float(np.mean(np.maximum(0.0, b - np.atleast_1d(x)))) / b,
       'past': lambda x, b: max(0.0, float(np.max(x)) - b) / b, 'over': lambda x, b: float(x) / b}


def margin(p, points):
    """How far point `p` lies inside the hull of `points` (`sight.convex`): negative outside; the
    hull of one or two points a point or a segment, everything off it outside."""
    from machine import sight
    hull = sight.convex(points)
    if len(hull) < 3:
        a, b = hull[0], hull[-1]
        ab = (b[0] - a[0], b[1] - a[1])
        t = max(0.0, min(1.0, ((p[0] - a[0]) * ab[0] + (p[1] - a[1]) * ab[1]) / max(ab[0] ** 2 + ab[1] ** 2, 1e-300)))
        return -math.hypot(p[0] - a[0] - t * ab[0], p[1] - a[1] - t * ab[1])
    return min(((p[1] - a[1]) * (b[0] - a[0]) - (p[0] - a[0]) * (b[1] - a[1])) / math.hypot(b[0] - a[0], b[1] - a[1])
               for a, b in zip(hull, hull[1:] + hull[:1]))


def _touching(world, hers, force, v6):
    """(the world's force on her, N, (x, y, z); each contact's (x, z); their mean height; the slip
    speed, m/s, weighed by the normal forces) of her contacts with whatever bears her now."""
    m, d, mj = world.model, world.data, world._mj
    f_on, points, ys, slip, normal = np.zeros(3), [], [], 0.0, 0.0
    for i in range(d.ncon):
        c = d.contact[i]
        a, b = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
        if hers[a] == hers[b]:
            continue
        mine = c.geom1 if hers[a] else c.geom2
        mj.mj_contactForce(m, d, i, force)
        frame = np.asarray(c.frame).reshape(3, 3)
        f_on += (frame.T @ force[:3]) * (1.0 if mine == c.geom2 else -1.0)
        points.append((float(c.pos[0]), float(c.pos[2])))
        ys.append(float(c.pos[1]))
        mj.mj_objectVelocity(m, d, mj.mjtObj.mjOBJ_GEOM, mine, v6, 0)
        v = v6[3:] + np.cross(v6[:3], np.asarray(c.pos) - d.geom_xpos[mine])
        slip += abs(float(force[0])) * float(np.linalg.norm(v - float(v @ frame[0]) * frame[0]))
        normal += abs(float(force[0]))
    return f_on, points, (sum(ys) / len(ys) if ys else 0.0), slip / normal if normal else 0.0


def sampler(rec, since):
    """A pass's sampling into `rec` from `since` s on: the centroidal pivot's margin in her contacts'
    hull, mm (where her centre of mass's line along the world's force on her meets their height),
    her load over her weight, least and most, her contacts' slip, mm, her feet's gap, mm, every
    ROW_S, her drives' joules and her centre of mass's way, m."""
    from machine import edge, modal

    def laid(s):
        g = s['g']
        world, t = g['world'], g['t']
        if t < since:
            return
        m, d = world.model, world.data
        c = d.subtree_com[world.pelvis]
        if not rec['hers']:
            rec.update(hers=[int(edge._rooted(m, b, world.pelvis)) for b in range(m.nbody)],
                       weight=float(g['fr'][modal.WEIGHT]), t=t, com=c.tolist())
        f_on, points, floor_y, slip = _touching(world, rec['hers'], rec['force'], rec['v6'])
        dt, load = t - rec['t'], float(f_on[1]) / rec['weight']
        rec.update(load=min(rec['load'], load), peak=max(rec['peak'], load), slip=rec['slip'] + 1e3 * slip * dt,
                   joules=rec['joules'] + world.drawn() * dt, way=rec['way'] + float(np.linalg.norm(c - rec['com'])))
        if points and f_on[1] > 0.0:
            k = (float(c[1]) - floor_y) / float(f_on[1])
            rec['pivot'].append(1e3 * margin((float(c[0]) - k * f_on[0], float(c[2]) - k * f_on[2]), points))
        if t - rec['gap_t'] >= suite.ROW_S - 1e-9:
            rec.update(gap=min(rec['gap'], 1e3 * world.gap(look.LEFT_FOOT, look.RIGHT_FOOT)), gap_t=t)
        rec.update(t=t, com=c.tolist())
    return laid


def residuals(measures, gr, bounds):
    """The text's six residuals of her `measures`, in its order, each in its dimension (CHANNELS),
    `bounds` the manifold's invariants and plantar boundary and her weight."""
    out = []
    for name, r in gr.items():
        measure, how, bound = CHANNELS[name]
        out.append(HOW[how](measures[measure], bounds[r.invariant] if bound is None else bounds.get(bound, bound)))
    return np.array(out)


@contextlib.contextmanager
def suite_said(said):
    """The suite's files copied with `said` ({(path, start, end): value}) at those spans and
    `dsp.SUITE` pointed at the copies while it lasts: every reader of the suite in this process -
    her gym's stunts too (`maneuver`, read once a process) - reads what the run measures."""
    from machine import dsp, tune
    lits = [tune.Literal(p, a, z, None, None, None, None, v) for (p, a, z), v in said.items()]
    texts, kept = tune.spliced(tune.texts(), lits, [lit.value for lit in lits]), list(dsp.SUITE)
    with tempfile.TemporaryDirectory() as where:
        dsp.SUITE[:] = [os.path.join(where, os.path.basename(path)) for path in kept]
        for path, copy in zip(kept, dsp.SUITE):
            with open(copy, 'w', encoding='utf-8', newline='') as f:
                f.write(texts[path])
        try:
            yield
        finally:
            dsp.SUITE[:] = kept


def _gait(root, got, rec):
    """A gait of the suite on her walk, sampled from SETTLE_S: (rows, fell s or None, its scene, {})."""
    from machine import voicing
    row, told = voicing.voiced(got[root], got['nominal-biped-manifold'], suite._her())
    scene = suite.scene_of(got[root])
    rows, fell = suite.played(row, told, scene, laid=sampler(rec, suite.SETTLE_S))
    return rows, fell, scene, {}


def _stunt(root, got, rec):
    """A stunt she plays in the first scenario that says it, sampled from its saying: (no rows, fell
    s or None, its scene, {met: its goal by the world's truth})."""
    from machine.errors import MachineError
    from tools.sim import scenarios
    said = [(name, sc, at) for name, sc in scenarios.SCENARIOS.items() for at, c in sc[1] if c.get('say') == root]
    if not said:
        raise MachineError('%s: no scenario says it' % root)
    name, sc, at = said[0]
    out = scenarios.ran(name, laid=sampler(rec, at))
    return [], out['fell_at'], sc[0], {'met': out['met']}


def _schedule(root, got, rec):
    """A schedule to its first refusal on her walk, sampled from SETTLE_S: (rows, fell, 'flat',
    {plan})."""
    plan, rows, fell = suite.schedule(root, sampler(rec, suite.SETTLE_S))
    return rows, fell, 'flat', {'plan': [list(p) for p in plan]}


#: A root's run by its kind (`tune.kind`).
RUNS = {'gait': _gait, 'stunt': _stunt, 'schedule': _schedule}


def ran(root, said):
    """{r, kind, measures, form, broken, fell, scene, wall s, ..} of `root` run on her (RUNS), the
    suite's texts with `said` ({(path, start, end): value}) at those spans (`suite_said`)."""
    from machine import dsp, tune
    from machine.errors import MachineError
    rec = {'hers': None, 'force': np.zeros(6), 'v6': np.zeros(6), 'load': math.inf, 'peak': 0.0, 'slip': 0.0,
           'pivot': [], 'joules': 0.0, 'way': 0.0, 'gap': math.inf, 'gap_t': -math.inf}
    began = time.time()
    with suite_said(said):
        got = dsp.read(dsp.SUITE)
        kind = tune.kind(root, got)
        rows, fell, scene, extra = RUNS[kind](root, got, rec)
    if rec['hers'] is None:
        raise MachineError('%s: nothing sampled, she fell at %s s, before its window' % (root, fell))
    form = strides.measured(rows) if kind == 'gait' else {}
    measures = {'pivot': rec['pivot'], 'load': rec['load'], 'slip': rec['slip'], 'gap': rec['gap'], 'peak': rec['peak'],
                'energy': rec['joules'] / rec['way']}
    bounds = dict(got['nominal-biped-manifold']['plantar-boundary'], weight=rec['weight'], **got['physical-invariants'])
    pivot = np.array(rec['pivot'])
    return dict(extra, r=residuals(measures, tune.grammar(), bounds).tolist(), kind=kind, fell=fell, scene=scene,
                measures=dict(measures, pivot=[float(np.median(pivot)), float(np.mean(pivot < bounds['min-zmp-margin-mm']))]),
                form={name: form.get(name) for name, _least, _most in looks.FORM},
                broken=[name for name, _v, _b in looks.broken(form)] if form else [], wall=round(time.time() - began, 1))
