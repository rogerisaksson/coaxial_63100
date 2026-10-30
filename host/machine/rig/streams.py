"""Her subsystems' streams, each on its own clock: what a gate (`machine.rig.gate`) records of her.

STREAMS, a row a stream in her pass's order - the order one instant's records keep -: its name;
`by`, what makes it ('module:function'), its words; `due(s, c)`, the key and time of its newest
sample on its subsystem's own clock (`clock`: a pass's start for what her edge read and her law
made of it, its end for the world it left), None where the pass made none; `laid(s)`, a vector's
width and fields laid once from her code's own layouts (`introspect.layout`, `introspect.counted`),
None a record's; `take(s, lay, row)`, its sample - a vector's into its ring's row, a record's
returned, its arrays copied -; `period(s)`, s, None where it comes as it comes. The commands she
hears and her analyzers' events are handed in as they happen (`gate.Scope`), never polled.
"""
import ast
import collections
import importlib
import inspect
import math
import re

Stream = collections.namedtuple('Stream', 'name by due laid take period')


def clock(s, last):
    """This pass's clock in her session `s`, `last` the last pass's (`seen`) or None: {'fresh': none
    to compare with - a new epoch, the first pass seen, passes unseen between -, 't0': its start,
    't1': its end, 'felt': her edge read, 'passed': her law's pass ran (her drives told), 'law': her
    law ran, 'moved': her world stepped}. A pass that raised is not recorded."""
    g = s['g']
    t1, n = float(g['t']), int(g['n'])
    fresh = last is None or last[0] != s['epoch'] or last[3] + 1 != s['passes']
    t0, n0 = (t1, n) if last is None or fresh else (last[1], last[2])
    passed = n != n0
    return {'fresh': fresh, 't0': t0, 't1': t1, 'felt': t1 != t0, 'passed': passed,
            'law': passed and (n - 1) % int(g['every']) == 0, 'moved': t1 != t0}


def seen(s):
    """This pass as the next one's clock compares with it: (epoch, its end, her law's passes, her passes)."""
    return s['epoch'], float(s['g']['t']), int(s['g']['n']), int(s['passes'])


def said(by):
    """The first sentence of the words of `by` ('module:qualified name'), its whitespace one space."""
    module, name = by.split(':')
    obj = importlib.import_module(module)
    for part in name.split('.'):
        obj = getattr(obj, part)
    return re.split(r'\.\s', ' '.join((inspect.getdoc(obj) or '').split()), maxsplit=1)[0]


def _at(key, end=False):
    """Due where this pass's clock says `key`: at the pass's start, or its `end`."""
    return lambda s, c: (c['t1'],) * 2 if end and c[key] else (c['t0'],) * 2 if c[key] else None


def _things(s, c):
    at = float(s['g']['things_at'])
    return (at, at) if math.isfinite(at) else None


def _wear(s, c):
    return id(s['wear_told']), c['t1']


def _pushed(s, c):
    return None


def _pass(s):
    from machine.process import worker
    return worker.DT


def _law(s):
    return _pass(s) * int(s['g']['every'])


def _refresh(s):
    """Her things' period: the first law pass `schema.REFRESH_S` after the last (`sprung.passed`)."""
    from machine.law import schema
    return _law(s) * math.ceil(schema.REFRESH_S / _law(s))


def _fields(name, size, end=None, base=0, prefix=''):
    """{field: {'at', 'n'[, 'of', 'each']}} of module `name`'s layout at `base` of a stream's vector,
    named with `prefix`: 'of' the names its entries are, 'each' their components (`introspect.counted`)."""
    from machine.process import introspect
    lay, of = introspect.layout(name, size, end), introspect.counted(name, size, end)
    return {prefix + k: dict({'at': base + sp.start, 'n': sp.stop - sp.start}, **of.get(k, {})) for k, sp in lay.items()}


def _edge(fn):
    """A read of her edge's array (`edge.felt`, `edge.read`): the layout's fields its definition in
    her edge's source names - not whatever stands in its place at the time -, packed."""
    def laid(s):
        import numpy as np
        from machine.senses import edge
        tree = ast.parse(inspect.getsource(edge))
        read = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == fn)
        named = {n.id for n in ast.walk(read) if isinstance(n, ast.Name)}
        whole = _fields('machine.senses.edge', 'SENSED')
        kept = [(k, f) for k, f in whole.items() if k in named]
        at = np.cumsum([0] + [f['n'] for _k, f in kept]).tolist()
        return {'n': at[-1], 'idx': np.concatenate([np.arange(f['at'], f['at'] + f['n']) for _k, f in kept]),
                'fields': {k: dict(f, at=a) for (k, f), a in zip(kept, at)}}
    return laid


def _sensed(s, lay, row):
    import numpy as np
    np.take(s['g']['sense'], lay['idx'], out=row)


def _latent(s):
    """Her latent state - each row of it by the row's layout - and her latent out after it."""
    from machine.law import latent
    rows = _fields('machine.law.latent', 'P', 'NOW')
    fields = {}
    for k, f in _fields('machine.law.latent', 'STATE').items():
        fields.update({k + '.' + r: dict(v, at=f['at'] + v['at']) for r, v in rows.items()} if f['n'] == latent.P else {k: f})
    fields.update(_fields('machine.law.latent', 'OUT', base=latent.STATE, prefix='out.'))
    return {'n': latent.STATE + latent.OUT, 'split': latent.STATE, 'fields': fields}


def _latent_take(s, lay, row):
    row[:lay['split']], row[lay['split']:] = s['g']['st'], s['g']['out']


def _impedance(s):
    from machine.law import modal
    return {'n': modal.ASK, 'fields': _fields('machine.law.modal', 'ASK')}


def _impedance_take(s, lay, row):
    row[:] = s['g']['a']


def _copied(d):
    """`d` with its arrays copied: this pass's, as the next leaves it."""
    import numpy as np
    return {k: v.copy() if isinstance(v, np.ndarray) else v for k, v in d.items()}


def _wrench_take(s, lay, row):
    return {'ask': _copied(s['g'].get('ask') or {}), 'got': _copied(s['g'].get('got') or {})}


def _drives(s):
    """Each of her drives' setpoint, feed and clamp, by her driven joints' names."""
    from machine.body.figure import JOINTS
    g = s['g']
    keys, n = list(g['cmd']), len(g['joints'])
    return {'n': n * len(keys), 'keys': keys, 'names': {'driven': [JOINTS[i] for i in g['joints']]},
            'fields': {k: {'at': i * n, 'n': n, 'of': 'driven', 'each': 1} for i, k in enumerate(keys)}}


def _drives_take(s, lay, row):
    cmd, n = s['g']['cmd'], len(s['g']['joints'])
    for i, k in enumerate(lay['keys']):
        row[i * n:(i + 1) * n] = cmd[k]


def _world(s):
    """The world's joints as the pass left them, its model's names: their places, then their speeds."""
    m = s['g']['world'].model
    q, v = list(m.jnt_qposadr) + [m.nq], list(m.jnt_dofadr) + [m.nv]
    names = [m.joint(i).name for i in range(m.njnt)]
    fields = {'qpos.' + nm: {'at': int(q[i]), 'n': int(q[i + 1] - q[i])} for i, nm in enumerate(names)}
    fields.update({'qvel.' + nm: {'at': int(m.nq + v[i]), 'n': int(v[i + 1] - v[i])} for i, nm in enumerate(names)})
    return {'n': int(m.nq + m.nv), 'nq': int(m.nq), 'fields': fields}


def _world_take(s, lay, row):
    d = s['g']['world'].data
    row[:lay['nq']], row[lay['nq']:] = d.qpos, d.qvel


STREAMS = (
    Stream('heard', 'machine.process.worker:apply', _pushed, None, None, None),
    Stream('felt', 'machine.senses.edge:felt', _at('felt'), _edge('felt'), _sensed, _pass),
    Stream('read', 'machine.senses.edge:read', _at('law'), _edge('read'), _sensed, _law),
    Stream('things', 'machine.eyes.registry:boxes', _things, None, lambda s, lay, row: list(s['g']['things']), _refresh),
    Stream('latent', 'machine.law.latent:step', _at('law'), _latent, _latent_take, _law),
    Stream('impedance', 'machine.law.impedance:ask', _at('law'), _impedance, _impedance_take, _law),
    Stream('wrench', 'machine.stack.wrench:step', _at('law'), None, _wrench_take, _law),
    Stream('drives', 'machine.law.motion:drives', _at('passed'), _drives, _drives_take, _pass),
    Stream('world', 'machine.world.physics:World.advance', _at('moved', end=True), _world, _world_take, _pass),
    Stream('wear', 'machine.identify.wear:told', _wear, None, lambda s, lay, row: s['wear_told'], None),
    Stream('events', 'machine.process.watch:Watch.step', _pushed, None, None, None),
)
NAMES = tuple(r.name for r in STREAMS)
