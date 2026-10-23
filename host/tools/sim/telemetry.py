"""The autotuner's telemetry: a root run on her, the text's six classes sampled as residuals.

    out = telemetry.ran('sit_chair', {(path, start, end): value})   # {r, kind, measures, form, ..}
    r = telemetry.residuals(measures, tune.grammar(), bounds)       # the text's order, each dimensionless

A run (RUNS) is a gait on her walk (`suite.played`, 20 s), her walk or a behaviour of her gym said
on it, a stunt in the first scenario that says it (`scenarios.ran`), a schedule to its first
refusal (`suite.schedule`): of her texts with the run's literals spliced, passed to the session it
lays (`worker.session`). From SETTLE_S, or the stunt's saying, her world is sampled each pass over
whatever bears her (`sampler`); each class a residual in its dimension (CHANNELS, BOUNDS).
"""
import math
import time

from coaxial.model.blocks import numpy as np  # behind the OpenBLAS cap

from tools.sim import look, looks, strides, suite

#: Each class's measure of hers, how it is a residual, and the bound it is over: None the class's
#: invariant (the manifold's), else a name of BOUNDS'.
CHANNELS = {'com-wrench-balance': ('pivot', 'short', None), 'normal-force-floor': ('load', 'short', None),
            'tangential-shear-slip': ('cone', 'over', 'friction-cone-edge'),
            'spatial-sdf-clearance': ('gap', 'short', None),
            'contact-transient-ringdown': ('peak', 'past', 'landing-load'),
            'actuator-thermal-passivity': ('bleed', 'over', 'tank-capacity-joules')}
#: A measure as a residual, its bound's share: its mean shortfall under the bound, how far its
#: most is past it, its size.
HOW = {'short': lambda x, b: float(np.mean(np.maximum(0.0, b - np.atleast_1d(x)))) / b,
       'past': lambda x, b: max(0.0, float(np.max(x)) - b) / b, 'over': lambda x, b: float(x) / b}


def bounds(got):
    """{name: value} a residual is bounded by: the suite's invariants and plantar boundary, the
    friction cone's edge (|F_t| = mu F_n), her form's landing (`looks.LANDS`), her master bus's tank
    (`supervisor.insert`)."""
    from machine import supervisor
    return dict(got['nominal-biped-manifold']['plantar-boundary'], **got['physical-invariants'],
                **{'friction-cone-edge': 1.0, 'landing-load': looks.LANDS,
                   'tank-capacity-joules': supervisor.insert()['energy-tank']['tank-capacity-joules']})


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
    speed, m/s, and the friction cone's use, |F_t| over mu F_n, each weighed by the normal forces) of
    her contacts with whatever bears her now."""
    m, d, mj = world.model, world.data, world._mj
    f_on, points, ys, slip, shear, grip, borne = np.zeros(3), [], [], 0.0, 0.0, 0.0, 0.0
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
        normal = abs(float(force[0]))
        slip += normal * float(np.linalg.norm(v - float(v @ frame[0]) * frame[0]))
        shear += math.hypot(float(force[1]), float(force[2]))
        grip += float(c.friction[0]) * normal
        borne += normal
    return f_on, points, (sum(ys) / len(ys) if ys else 0.0), slip / borne if borne else 0.0, shear / grip if grip else 0.0


def sampler(rec, since):
    """A pass's sampling into `rec` from `since` s on: the centroidal pivot's margin in her contacts'
    hull, mm (where her centre of mass's line along the world's force on her meets their height),
    her load over her weight, least and most, her contacts' slip, mm, the friction cone's use, her
    feet's gap, mm, every ROW_S, her master bus's tank's bleed, J (what its band power drew from
    it), her drives' joules and her centre of mass's way, m."""
    from machine import edge, modal

    def laid(s):
        g = s['g']
        world, t = g['world'], g['t']
        if t < since:
            return
        m, d = world.model, world.data
        c, tank = d.subtree_com[world.pelvis], float(g['bus'][1].energy)
        if not rec['hers']:
            rec.update(hers=[int(edge._rooted(m, b, world.pelvis)) for b in range(m.nbody)],
                       weight=float(g['fr'][modal.WEIGHT]), t=t, com=c.tolist(), tank=tank)
        f_on, points, floor_y, slip, cone = _touching(world, rec['hers'], rec['force'], rec['v6'])
        dt, load = t - rec['t'], float(f_on[1]) / rec['weight']
        borne = float(f_on[1] > 0.0)
        rec.update(load=min(rec['load'], load), peak=max(rec['peak'], load), slip=rec['slip'] + 1e3 * slip * dt,
                   cone=rec['cone'] + borne * cone, borne=rec['borne'] + borne, bleed=rec['bleed'] + max(0.0, rec['tank'] - tank),
                   joules=rec['joules'] + world.drawn() * dt, way=rec['way'] + float(np.linalg.norm(c - rec['com'])))
        if points and f_on[1] > 0.0:
            k = (float(c[1]) - floor_y) / float(f_on[1])
            rec['pivot'].append(1e3 * margin((float(c[0]) - k * f_on[0], float(c[2]) - k * f_on[2]), points))
        if t - rec['gap_t'] >= suite.ROW_S - 1e-9:
            rec.update(gap=min(rec['gap'], 1e3 * world.gap(look.LEFT_FOOT, look.RIGHT_FOOT)), gap_t=t)
        rec.update(t=t, com=c.tolist(), tank=tank)
    return laid


def residuals(measures, gr, bounds):
    """The text's six residuals of her `measures`, in its order, each in its dimension (CHANNELS)."""
    out = []
    for name, r in gr.items():
        measure, how, bound = CHANNELS[name]
        out.append(HOW[how](measures[measure], bounds[r.invariant if bound is None else bound]))
    return np.array(out)


def _gait(root, got, gym, texts, rec):
    """A gait of the suite on her walk, sampled from SETTLE_S: (rows, fell s or None, its scene, her
    form's condition)."""
    from machine import voicing
    row, told = voicing.voiced(got[root], got['nominal-biped-manifold'], suite._her(), gym)
    scene = suite.scene_of(got[root])
    rows, fell = suite.played(row, told, scene, laid=sampler(rec, suite.SETTLE_S), texts=texts)
    return rows, fell, scene, {'formed': scene == 'flat'}


def _behaviour(root, got, gym, texts, rec):
    """Her walk (`tune.WHOLE`) told her walk's twist, or a behaviour of her gym said, on a flat floor,
    sampled from SETTLE_S: her walk's form its condition."""
    from machine import tune
    from tools.sim import scenarios
    told = {'twist': scenarios.WALK} if root in tune.WHOLE else {'twist': (0.0, 0.0, 0.0), 'say': root}
    rows, fell = suite.played({}, told, 'flat', laid=sampler(rec, suite.SETTLE_S), texts=texts)
    return rows, fell, 'flat', {'formed': root in tune.WHOLE}


def _stunt(root, got, gym, texts, rec):
    """A stunt she plays in the first scenario that says it, sampled from its saying: (no rows, fell
    s or None, its scene, its goal met by the world's truth)."""
    from machine.errors import MachineError
    from tools.sim import scenarios
    said = [(name, sc, at) for name, sc in scenarios.SCENARIOS.items() for at, c in sc[1] if c.get('say') == root]
    if not said:
        raise MachineError('%s: no scenario says it' % root)
    name, sc, at = said[0]
    out = scenarios.ran(name, laid=sampler(rec, at), texts=texts)
    return [], out['fell_at'], sc[0], {'met': out['met'], 'formed': False}


def _schedule(root, got, gym, texts, rec):
    """A schedule to its first refusal on her walk, sampled from SETTLE_S: (rows, fell, 'flat',
    {plan})."""
    plan, rows, fell = suite.schedule(root, sampler(rec, suite.SETTLE_S), texts)
    return rows, fell, 'flat', {'plan': [list(p) for p in plan], 'formed': False}


#: A root's run by its kind (`tune.kind`).
RUNS = {'gait': _gait, 'behaviour': _behaviour, 'stunt': _stunt, 'schedule': _schedule}


def ran(root, said):
    """{r, kind, measures, form, broken, fell, scene, wall s, ..} of `root` run on her (RUNS), her
    texts with `said` ({(path, start, end): value}) at those spans."""
    from machine import tune
    from machine.errors import MachineError
    lits = [tune.Literal(p, a, z, None, None, None, None, v) for (p, a, z), v in said.items()]
    texts = tune.spliced(tune.texts(), lits, [lit.value for lit in lits])
    got, gym = tune.evaluated(texts)
    rec = {'hers': None, 'force': np.zeros(6), 'v6': np.zeros(6), 'load': math.inf, 'peak': 0.0, 'slip': 0.0,
           'cone': 0.0, 'borne': 0.0, 'bleed': 0.0, 'pivot': [], 'joules': 0.0, 'way': 0.0, 'gap': math.inf,
           'gap_t': -math.inf}
    began, kind = time.time(), tune.kind(root, got, gym)
    rows, fell, scene, extra = RUNS[kind](root, got, gym, texts, rec)
    if rec['hers'] is None:
        raise MachineError('%s: nothing sampled, she fell at %s s, before its window' % (root, fell))
    form = strides.measured(rows) if extra['formed'] else {}
    measures = {'pivot': rec['pivot'], 'load': rec['load'], 'cone': rec['cone'] / max(rec['borne'], 1.0),
                'gap': rec['gap'], 'peak': rec['peak'], 'bleed': rec['bleed'], 'slip': rec['slip'],
                'energy': rec['joules'] / rec['way']}
    limits, pivot = bounds(got), np.array(rec['pivot'])
    return dict(extra, r=residuals(measures, tune.grammar(), limits).tolist(), kind=kind, fell=fell, scene=scene,
                measures=dict(measures, pivot=[float(np.median(pivot)), float(np.mean(pivot < limits['min-zmp-margin-mm']))]),
                form={name: form.get(name) for name, _least, _most in looks.FORM},
                broken=[name for name, _v, _b in looks.broken(form)] if form else [], wall=round(time.time() - began, 1))
