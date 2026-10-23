"""The autotuner's telemetry: a root run on her, the text's six classes sampled as residuals.

    out = telemetry.ran('sit_chair', {(path, start, end): value})   # {r, kind, measures, form, ..}
    r = telemetry.residuals(measures, tune.grammar(), bounds)       # the text's order, each dimensionless

A run (RUNS) is a gait on her walk (`suite.played`, 20 s), her walk or a behaviour of her gym said
on it, a stunt in the first scenario that says it (`scenarios.ran`), a schedule to its first
refusal (`suite.schedule`): of her texts with the run's literals spliced, passed to the session it
lays (`worker.session`). From SETTLE_S, or the stunt's saying, her world is copied each pass
(`tap.raw`), cooked and measured after (`tap.cooked`, `tap.measured`): her runtime tuner's
measures, one definition; each class a residual in its dimension (CHANNELS, `bounds`).
"""
import time

from coaxial.model.blocks import numpy as np  # behind the OpenBLAS cap

from tools.sim import looks, strides, suite

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


def sampler(rec, since):
    """Each pass from `since` s on, her raw pass (`tap.raw`) into `rec`, her weight beside it."""
    from machine import modal, tap

    def laid(s):
        if s['g']['t'] >= since:
            rec['weight'] = float(s['g']['fr'][modal.WEIGHT])
            rec['raws'].append(tap.raw(s, rec['state']))
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
    from machine import tap, tune
    from machine.errors import MachineError
    texts = tune.given(said)
    got, gym = tune.evaluated(texts)
    rec = {'raws': [], 'state': tap.fresh(), 'weight': None}
    began, kind = time.time(), tune.kind(root, got, gym)
    rows, fell, scene, extra = RUNS[kind](root, got, gym, texts, rec)
    if not rec['raws']:
        raise MachineError('%s: nothing sampled, she fell at %s s, before its window' % (root, fell))
    return dict(scored(got, tap.measured(tap.cooked(rec['raws']), rec['weight']), rows, extra), kind=kind, fell=fell,
                scene=scene, wall=round(time.time() - began, 1))


def scored(got, measures, rows, extra):
    """A run's {r, measures, form, broken, ..extra} of its `measures` (`tap.measured`) and her look's
    `rows` - her form measured where `extra` says it is formed -, bounded by the suite `got`'s."""
    from machine import tune
    form = strides.measured(rows) if extra['formed'] else {}
    limits, pivot = bounds(got), measures['pivot']
    return dict(extra, r=residuals(measures, tune.grammar(), limits).tolist(),
                measures=dict(measures, pivot=[float(np.median(pivot)), float(np.mean(pivot < limits['min-zmp-margin-mm']))]),
                form={name: form.get(name) for name, _least, _most in looks.FORM},
                broken=[name for name, _v, _b in looks.broken(form)] if form else [])


def rows(seen):
    """Her looks (`tap.look`'s, n) as the page's rows (`humanoid_keys.row`), as `scenarios._row` lays them."""
    from machine import tap
    from machine.figure import JOINTS
    from terminal.views.humanoid_keys import HEADER, row
    out, n = [], len(tap.LOOK)
    for x in seen:
        angles = dict(zip(JOINTS, (float(v) for v in x[n:n + len(JOINTS)])))
        now = {'t': float(x[0]), 'stage': '', 'speed': float(x[1]), 'phase': 0.0, 'loads': (float(x[2]), float(x[3])),
               'where': tuple(float(v) for v in x[4:7]), 'turn': tuple(float(v) for v in x[7:11]), 'angles': angles,
               'set': angles, 'watts': 0.0}
        out.append(dict(zip(HEADER, row(now, 0.0))))
    return out
