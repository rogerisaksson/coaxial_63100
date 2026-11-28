"""Bit-exact MuJoCo, controller, and worker session checkpointing for deterministic branching.

    snap = checkpoint.capture(s)
    checkpoint.restore(s, snap)
    snap, ok = checkpoint.advance_until(s, predicate, max_s, commands=())

All C-pointer-bound NumPy arrays and ctypes structs in `world` and `g` are restored in place so
live C delegates (`beat`, `tick`, `drives`, `observer`, `bus`, `played`) remain bound and subsequent
simulation ticks match an uninterrupted trajectory bit for bit.
"""
import copy
import ctypes
from typing import Any

from machine.errors import MachineError
from machine.world import loose, steps

#: Static or explicitly-handled keys on `g`; compiled C/NumPy hooks join via `tools.cores.motion.HOOKS`.
_BASE_STATIC_G = frozenset({
    'world', 'body', 'beat', 'planner', 'vision', 'keep', 'gym', 'bodily',
    'schema_at', 'schema_body', '_skin_ids', '_tips', '_cv_sh', '_acv_sh', '_derate', 'grip_at',
    'joints', 'legs', 'leg_joints', 'limbs', 'cervical', 'waist_idx', 'head_idx', 'situations',
    'rng', 'q_lo', 'q_hi', 'clamp', 'wep_lim', 'free_kp', 'rate_hz', 'every', 'texts', 'suite',
    'bend', 'arms', 'reaches', 'folds', 'distal', 'distal_side', '_stand_entry', '_brake_wire',
    'drives', 'bus', 'tick', 'observer', 'heat', 'registry', 'bank', 'played', 'settled', '_prefetch', '_w_up',
})
#: Derived caches on `g` (holding raw C mask addresses or identity-keyed memos) rebuilt on restore.
_DERIVED_G = frozenset({'_falls_by', '_falls_down', '_falls_parts', '_falls_of', '_said_of', '_told_with'})
#: Mutable fields inside `s['wear']` restored in place (leaving static `body`/`lay`/`D` intact).
_WEAR_MUTABLE = ('z', 'P', 'hours', 'serviced_at', 'residual', 'read', 'boxes', 'box_n', 'zero', 'torso')
#: Immutable scalar atom types safe to share by reference across snapshots.
_IMMUTABLE = (type(None), bool, int, float, str, bytes, frozenset, slice, range, complex)
_STATIC_CACHE = None

#: Slot array buffers inside each `g['bank']['moves'][k]` restored in place to keep `Played` valid.
_SLOT_ARRAYS = ('pose', 'cmd', 'state', 'feed', 'target', 'limit')
_SLOT_SCALARS = ('when', 'at', 'last', 'enter_t0')


def _static_g():
    """Return full set of static, derived, and explicitly-handled keys on `g`."""
    global _STATIC_CACHE
    if _STATIC_CACHE is None:
        from tools.cores import motion as c_motion
        _STATIC_CACHE = frozenset(c_motion.HOOKS) | _BASE_STATIC_G | _DERIVED_G
    return _STATIC_CACHE


def _copy_val(val: Any, path: str = 'val') -> Any:
    """Clone mutable values strictly; reject unknown opaque stateful objects instead of leaking them."""
    import numpy as np
    if isinstance(val, np.ndarray):
        return val.copy()
    if isinstance(val, np.generic) or isinstance(val, _IMMUTABLE) or callable(val):
        return val
    if isinstance(val, dict):
        return {k: _copy_val(v, '%s.%s' % (path, k)) for k, v in val.items()}
    if isinstance(val, set):
        return {_copy_val(v, '%s{}' % path) for v in val}
    if isinstance(val, list):
        copied = [_copy_val(v, '%s[%d]' % (path, i)) for i, v in enumerate(val)]
        arrays = getattr(val, 'arrays', None)
        if arrays is not None:
            out: Any = type(val)(copied)
            out.arrays = tuple(a.copy() for a in arrays)
            return out
        return copied
    if isinstance(val, tuple):
        items = tuple(_copy_val(v, '%s[%d]' % (path, i)) for i, v in enumerate(val))
        return type(val)(*items) if hasattr(val, '_fields') else items
    raise TypeError('checkpoint cannot snapshot mutable %s at %s' % (type(val).__name__, path))


def _restore_inplace_dict(dst, saved):
    """Restore NumPy arrays in `dst` in place when shapes match, otherwise copy new values."""
    import numpy as np
    for k in list(dst):
        if k not in saved:
            del dst[k]
    for k, src in saved.items():
        if k in dst and isinstance(dst[k], np.ndarray) and isinstance(src, np.ndarray):
            if dst[k].shape == src.shape and dst[k].dtype == src.dtype:
                np.copyto(dst[k], src)
            else:
                dst[k] = src.copy()
        elif k in dst and isinstance(dst[k], dict) and isinstance(src, dict):
            _restore_inplace_dict(dst[k], src)
        elif k in dst and isinstance(dst[k], set) and isinstance(src, set):
            dst[k].clear()
            dst[k].update(_copy_val(src, str(k)))
        elif k in dst and isinstance(dst[k], list) and isinstance(src, list) and getattr(src, 'arrays', None) is None:
            dst[k][:] = _copy_val(src, str(k))
        else:
            dst[k] = _copy_val(src, str(k))


def _capture_world(world):
    """Capture complete MuJoCo `MjData` and mutable `World` state."""
    mj, m, d = world._mj, world.model, world.data
    steps.behind(world)
    snap_d = mj.MjData(m)
    mj.mj_copyData(snap_d, m, d)
    box = world.boxes
    stuck = world.stuck
    link_map = world.link
    furn_state = [[bool(a), bool(b), float(c), None if p is None else p.copy()]
                  for a, b, c, p in world.furnished[7]]
    return {
        'mj_data': snap_d,
        'dof_damping': m.dof_damping[world.vadr].copy(),
        'dof_armature': m.dof_armature[world.vadr].copy(),
        'dof_frictionloss': m.dof_frictionloss[world.vadr].copy(),
        'tendon_range': m.tendon_range[world.lace_at].copy(),
        'geom_contype': m.geom_contype.copy(),
        'geom_conaffinity': m.geom_conaffinity.copy(),
        'arrays': {name: getattr(world, name).copy() for name in (
            'gains', 'limit', 'target', 'was', 'rate', 'feed', 'motor',
            'geared', 'shorted', 'cut', 'testing', 'push_n')},
        'scalars': {
            'at': world.at, 'stamp': world.stamp, 'work': world.work, 'brake': world.brake,
            'heat': world.heat, 'effort': world.effort, 'borne': list(world.borne),
            'borne_at': world.borne_at, 'pending': dict(world.pending), 'pending_at': world.pending_at,
            'push_until': world.push_until, 'pushed': world.pushed, 'laced': world.laced,
            'hung': _copy_val(world.hung), 'glitch_at': world.glitch_at, 'glitch_until': world.glitch_until,
            'switched': getattr(world, 'switched', None), 'lamp': getattr(world, 'lamp', None),
        },
        'passing': (float(world.passing.effort), float(world.passing.work),
                    float(world.passing.brake), float(world.passing.heat)),
        'boxes': {name: getattr(box, name).copy() for name in (
            'delta', 'w', 'zero', 'q_was', 'q_step', 'went', 'hat', 'p_int', 'drive', 'asked', 'rate', 'j', 'c')},
        'link': {k: _copy_val(v) for k, v in link_map.items()},
        'stuck_n': int(stuck[0].n),
        'stuck_arrays': {i: stuck[i].copy() for i in (1, 2, 3, 4, 7, 10)},
        'furnished_state': furn_state,
    }


def _restore_world(world, snap):
    """Restore MuJoCo and `World` state bit for bit while preserving bound C array addresses."""
    import numpy as np
    mj, m, d = world._mj, world.model, world.data
    steps.behind(world)
    mj.mj_copyData(d, m, snap['mj_data'])
    m.dof_damping[world.vadr] = snap['dof_damping']
    m.dof_armature[world.vadr] = snap['dof_armature']
    m.dof_frictionloss[world.vadr] = snap['dof_frictionloss']
    m.tendon_range[world.lace_at] = snap['tendon_range']
    m.geom_contype[:] = snap['geom_contype']
    m.geom_conaffinity[:] = snap['geom_conaffinity']
    for name, arr in snap['arrays'].items():
        np.copyto(getattr(world, name), arr)
    sc = snap['scalars']
    for name in ('at', 'stamp', 'work', 'brake', 'heat', 'effort', 'borne_at',
                 'pending_at', 'push_until', 'pushed', 'laced', 'glitch_at', 'glitch_until'):
        setattr(world, name, sc[name])
    world.borne = list(sc['borne'])
    world.pending = dict(sc['pending'])
    world.hung = _copy_val(sc['hung'])
    if sc['switched'] is not None:
        world.switched = sc['switched']
    if sc['lamp'] is not None:
        world.lamp = sc['lamp']
    p = world.passing
    p.effort, p.work, p.brake, p.heat = snap['passing']
    box = world.boxes
    for name, arr in snap['boxes'].items():
        np.copyto(getattr(box, name), arr)
    _restore_inplace_dict(world.link, snap['link'])
    stuck = world.stuck
    stuck[0].n = snap['stuck_n']
    for idx, arr in snap['stuck_arrays'].items():
        np.copyto(stuck[idx], arr)
    for k, row in enumerate(snap['furnished_state']):
        world.furnished[7][k][:] = [row[0], row[1], row[2], None if row[3] is None else row[3].copy()]
    loose._held(world.furnished)


def _capture_bank(g):
    """Capture `g['bank']` move slot bindings, active engrams/cores, hooks, and `g['played']`."""
    from tools.cores import motion as c_motion
    bank = g.get('bank')
    played = g.get('played')
    played_kind, played_val = None, None
    if isinstance(played, dict):
        matched = next((k for k, sl in (bank or {}).get('moves', {}).items() if sl is played), None)
        if matched is not None:
            played_kind, played_val = 'slot', matched
        else:
            played_kind, played_val = 'dict', _copy_val(played, 'played')
    slots = {}
    if bank is not None:
        for k, sl in bank['moves'].items():
            slots[k] = {
                'ref': sl,
                'arrays': {a: sl[a].copy() for a in _SLOT_ARRAYS},
                'scalars': {a: sl[a] for a in _SLOT_SCALARS},
                'gains': None if sl['gains'] is None else sl['gains'].copy(),
            }
    return {
        'slots': slots,
        'handover': _copy_val(bank['handover'], 'bank.handover') if bank is not None else None,
        'programs': _copy_val(bank.get('programs', {}), 'bank.programs') if bank is not None else {},
        'active': dict(bank.get('active', {})) if bank is not None else {},
        'stock': dict(bank.get('stock', {})) if bank is not None else {},
        'active_cores': dict(bank.get('active_cores', {})) if bank is not None else {},
        'stock_cores': dict(bank.get('stock_cores', {})) if bank is not None else {},
        'hooks': {h: g[h] for h in c_motion.HOOKS if h in g},
        'played': (played_kind, played_val),
    }


def _restore_bank(g, snap):
    """Restore `g['bank']` slot bindings, active engram/core metadata, hooks, and `g['played']`."""
    import numpy as np
    from machine.senses import edge
    for h, fn in snap.get('hooks', {}).items():
        g[h] = fn
    bank = g.get('bank')
    if bank is not None:
        bank['moves'].clear()
        for k, saved in snap['slots'].items():
            sl = saved['ref']
            bank['moves'][k] = sl
            for a, arr in saved['arrays'].items():
                np.copyto(sl[a], arr)
            for a, val in saved['scalars'].items():
                sl[a] = val
            if saved['gains'] is None:
                sl['gains'] = None
            elif sl['gains'] is not None and sl['gains'].shape == saved['gains'].shape:
                np.copyto(sl['gains'], saved['gains'])
            else:
                sl['gains'] = saved['gains'].copy()
        bank['handover'] = _copy_val(snap['handover'], 'bank.handover')
        if 'programs' in bank or snap['programs']:
            bank['programs'] = _copy_val(snap['programs'], 'bank.programs')
        if 'active' in bank or snap['active']:
            bank['active'] = dict(snap['active'])
        if 'stock' in bank or snap['stock']:
            bank['stock'] = dict(snap['stock'])
        if 'active_cores' in bank or snap.get('active_cores'):
            bank['active_cores'] = dict(snap.get('active_cores', {}))
        if 'stock_cores' in bank or snap.get('stock_cores'):
            bank['stock_cores'] = dict(snap.get('stock_cores', {}))
    pk, pv = snap['played']
    if pk == 'slot' and bank is not None:
        g['played'] = bank['moves'][pv]
    elif pk == 'dict':
        restored = _copy_val(pv, 'played')
        if restored.get('drive') is not None and 'joints' in restored:
            restored['drive'] = edge.bound(
                g['world'], restored['joints'], target=restored['target'],
                feed=restored['feed'], kp=restored['kp'], limit=restored['limit'])
        g['played'] = restored
    else:
        g['played'] = None


def _chain_struct(g):
    """Return persistent C `wbc_step_t` struct for closed-chain QP solver `g['solves'][1]`, if present."""
    solves = g.get('solves')
    if isinstance(solves, tuple) and len(solves) > 1:
        args = getattr(solves[1], 'args', ())
        if args and hasattr(args[0], 'chain_nlet'):
            return args[0]
    return None


def _capture_chain(g):
    """Capture closed-chain QP complementarity and active-set warm-start fields on `g['solves'][1]`."""
    k = _chain_struct(g)
    if k is None:
        return None
    return (
        ctypes.string_at(ctypes.addressof(k.chain_held), ctypes.sizeof(k.chain_held)),
        ctypes.string_at(ctypes.addressof(k.chain_nheld), ctypes.sizeof(k.chain_nheld)),
        ctypes.string_at(ctypes.addressof(k.chain_let), ctypes.sizeof(k.chain_let)),
        int(k.chain_nlet),
    )


def _restore_chain(g, saved):
    """Restore closed-chain QP complementarity and active-set fields bit for bit in place."""
    k = _chain_struct(g)
    if k is None or saved is None:
        return
    held_b, nheld_b, let_b, nlet = saved
    ctypes.memmove(ctypes.addressof(k.chain_held), held_b, len(held_b))
    ctypes.memmove(ctypes.addressof(k.chain_nheld), nheld_b, len(nheld_b))
    ctypes.memmove(ctypes.addressof(k.chain_let), let_b, len(let_b))
    k.chain_nlet = nlet


def _capture_body(body):
    """Capture wear-mutable link masses, inertias, drive parameters, and load on `body`."""
    m = body.m
    return {
        'body_mass': m.body_mass.copy(), 'body_ipos': m.body_ipos.copy(),
        'body_inertia': m.body_inertia.copy(), 'body_iquat': m.body_iquat.copy(),
        'dof_armature': m.dof_armature.copy(), 'dof_damping': m.dof_damping.copy(),
        'dof_frictionloss': m.dof_frictionloss.copy(), 'mass': float(body.mass),
        'load': None if getattr(body, 'load', None) is None else body.load.copy(),
    }


def _restore_body(body, saved):
    """Restore wear-mutable model parameters on `body`, recomputing constants only if modified."""
    import numpy as np
    m = body.m
    changed = False
    for field in ('body_mass', 'body_ipos', 'body_inertia', 'body_iquat',
                  'dof_armature', 'dof_damping', 'dof_frictionloss'):
        dst = getattr(m, field)
        if not np.array_equal(dst, saved[field]):
            np.copyto(dst, saved[field])
            changed = True
    if changed:
        body.mj.mj_setConst(m, body.d)
    body.mass = saved['mass']
    if saved['load'] is None:
        if hasattr(body, 'load'):
            body.load = None
    elif getattr(body, 'load', None) is not None and body.load.shape == saved['load'].shape:
        np.copyto(body.load, saved['load'])
    else:
        body.load = saved['load'].copy()


def _capture_g(g):
    """Capture mutable `g` arrays, C structs (`observer`, `bus`, `heat`, `tick`, `solves`), and scalars."""
    import numpy as np
    static_g = _static_g()
    obs = g['observer']
    obs_c = obs['c']
    obs_bytes = ctypes.string_at(ctypes.addressof(obs_c), ctypes.sizeof(obs_c))
    obs_arrays = {k: obs[k].copy() for k in ('slide', 'soles', 'latched', 'below', 'since', 'gate', 'base', 'q', 'u')}
    bus_k, bus_s, bus_arrs = g['bus']
    _tk_at, _tk_fn, tick_s, tick_vec = g['tick']
    heat = g['heat']
    heat_arrays = {k: getattr(heat, k).copy() for k in (
        't', 'sq', 'ticks', 'krpm', 'limb_rad_s', 'derate', 'gates', 'trips', 'shorted',
        'spent', 'worst', 'die', 'cap', 'cap_at', '_peak')}
    reg = g['registry']
    drives_kept = {k: v.copy() for k, v in g['drives'][1].items() if isinstance(v, np.ndarray)}
    vis = g['vision']
    dynamic = {k: _copy_val(v, 'g.%s' % k) for k, v in g.items() if k not in static_g}
    return {
        'obs_bytes': obs_bytes,
        'obs_arrays': obs_arrays,
        'obs_gated_of': list(obs['gated_of']),
        'bus_state': (int(bus_s.n), float(bus_s.energy), float(bus_s.power), int(bus_s.primed)),
        'bus_arrays': {k: v.copy() for k, v in bus_arrs.items()},
        'tick_state': (float(tick_s.surety), float(tick_s.left), tick_vec.copy()),
        'heat_arrays': heat_arrays,
        'heat_scalars': (float(heat._h.at), bool(heat.envelope)),
        'reg_rows': {k: v.copy() for k, v in reg['rows'].items()},
        'reg_kinds': list(reg['kinds']),
        'reg_classed': copy.deepcopy(reg['classed']),
        'drives_kept': drives_kept,
        'chain': _capture_chain(g),
        'body': _capture_body(g['body']),
        'vision': (bool(vis.pending), _copy_val(vis._things, 'vision._things')),
        'gym': list(g['gym']),
        'bank': _capture_bank(g),
        'dynamic': dynamic,
        '_dynamic_inplace': [k for k, v in g.items()
                             if k not in static_g and isinstance(v, np.ndarray)],
    }


def _restore_g(g, snap):
    """Restore mutable `g` state bit for bit without reallocating C-pinned buffers."""
    import numpy as np
    static_g = _static_g()
    obs = g['observer']
    obs_c = obs['c']
    ctypes.memmove(ctypes.addressof(obs_c), snap['obs_bytes'], len(snap['obs_bytes']))
    for k, arr in snap['obs_arrays'].items():
        np.copyto(obs[k], arr)
    obs['gated_of'][:] = snap['obs_gated_of']
    _bus_k, bus_s, bus_arrs = g['bus']
    bus_s.n, bus_s.energy, bus_s.power, bus_s.primed = snap['bus_state']
    _restore_inplace_dict(bus_arrs, snap['bus_arrays'])
    _tk_at, _tk_fn, tick_s, tick_vec = g['tick']
    tick_s.surety, tick_s.left, tvec = snap['tick_state']
    np.copyto(tick_vec, tvec)
    heat = g['heat']
    for k, arr in snap['heat_arrays'].items():
        np.copyto(getattr(heat, k), arr)
    heat._h.at, heat.envelope = snap['heat_scalars']
    reg = g['registry']
    for k, arr in snap['reg_rows'].items():
        np.copyto(reg['rows'][k], arr)
    reg['kinds'][:] = snap['reg_kinds']
    reg['classed'] = copy.deepcopy(snap['reg_classed'])
    _restore_inplace_dict(g['drives'][1], snap['drives_kept'])
    _restore_chain(g, snap['chain'])
    _restore_body(g['body'], snap['body'])
    vis = g['vision']
    vis.pending, vis._things = snap['vision'][0], _copy_val(snap['vision'][1], 'vision._things')
    vis._boxed.clear()
    g['gym'] = list(snap['gym'])
    _restore_bank(g, snap['bank'])
    for k in _DERIVED_G:
        g.pop(k, None)
    saved = snap['dynamic']
    inplace = set(snap['_dynamic_inplace'])
    for k in list(g):
        if k not in static_g and k not in saved:
            del g[k]
    for k, src in saved.items():
        if (k in inplace and k in g and isinstance(g[k], np.ndarray)
                and isinstance(src, np.ndarray) and g[k].shape == src.shape and g[k].dtype == src.dtype):
            np.copyto(g[k], src)
        elif k in g and isinstance(g[k], dict) and isinstance(src, dict):
            _restore_inplace_dict(g[k], src)
        elif k in g and isinstance(g[k], set) and isinstance(src, set):
            g[k].clear()
            g[k].update(_copy_val(src, 'g.%s' % k))
        elif k in g and isinstance(g[k], list) and isinstance(src, list) and getattr(src, 'arrays', None) is None:
            g[k][:] = _copy_val(src, 'g.%s' % k)
        else:
            g[k] = _copy_val(src, 'g.%s' % k)


def capture(s, allow_apart=False):
    """Capture a bit-exact in-memory snapshot of worker session `s`."""
    g = s['g']
    worn = s['worn']
    if not allow_apart and (getattr(g.get('vision'), 'apart', False) or g.get('keep') is not None
                            or getattr(worn, 'apart', False)):
        raise MachineError('checkpoint requires in-process vision, no camera subprocess, and in-process wear')
    worn_apart = bool(getattr(worn, 'apart', False))
    if not worn_apart and worn.due_at is not None and worn.got is None and worn.parts is not None:
        while True:
            try:
                next(worn.parts)
            except StopIteration as end:
                worn.got = end.value
                break
    return {
        'world': _capture_world(g['world']),
        'g': _capture_g(g),
        's': {
            'passes': s['passes'],
            'fallen_at': s['fallen_at'],
            'shutdown': s['shutdown'],
            'refused': s['refused'],
            'epoch': s['epoch'],
            'gym': list(s['gym']),
            'says': list(s['says']),
            'torque': s['torque'].copy(),
            'power': s['power'].copy(),
            'watts': s['watts'],
            'ratio': s['ratio'],
            'headroom': s['headroom'],
            'wear_told': copy.deepcopy(s['wear_told']),
            'wear': {k: _copy_val(s['wear'][k], 'wear.%s' % k) for k in _WEAR_MUTABLE if k in s['wear']},
            'worn_ring': None if worn_apart else (worn.ring.rows.copy(), int(worn.ring.start), int(worn.ring.n)),
            'worn_state': (None, None) if worn_apart else (_copy_val(worn.got, 'worn.got'), worn.due_at),
        },
    }


def restore(s, snap):
    """Restore worker session `s` bit for bit to snapshot `snap`."""
    import numpy as np
    _restore_world(s['g']['world'], snap['world'])
    _restore_g(s['g'], snap['g'])
    ss = snap['s']
    for k in ('passes', 'fallen_at', 'shutdown', 'refused', 'epoch', 'watts', 'ratio', 'headroom'):
        s[k] = ss[k]
    s['gym'] = list(ss['gym'])
    s['says'] = list(ss['says'])
    np.copyto(s['torque'], ss['torque'])
    np.copyto(s['power'], ss['power'])
    s['wear_told'] = copy.deepcopy(ss['wear_told'])
    for k, src in ss['wear'].items():
        if k in s['wear'] and isinstance(s['wear'][k], np.ndarray) and isinstance(src, np.ndarray):
            np.copyto(s['wear'][k], src)
        else:
            s['wear'][k] = _copy_val(src, 'wear.%s' % k)
    worn = s['worn']
    if ss['worn_ring'] is None or getattr(worn, 'apart', False):
        worn.emptied()
        worn.laid(s['wear'])
    else:
        rows, start, n = ss['worn_ring']
        np.copyto(worn.ring.rows, rows)
        worn.ring.start, worn.ring.n = start, n
        worn.w = s['wear']
        worn.got, worn.due_at = _copy_val(ss['worn_state'][0], 'worn.got'), ss['worn_state'][1]
        worn.parts = iter(()) if worn.due_at is not None else None
    if getattr(s['g'].get('vision'), 'apart', False):
        s['g']['vision'].pending = False
    pf = s['g'].get('_prefetch')
    if pf is not None:
        pf.rebuild(s['g'], eager=False)


def advance_until(s, predicate, max_s, commands=()):
    """Step session `s` with scheduled `(at_s, cmd)` commands until `predicate(s)` holds, then capture."""
    from machine.process import worker
    pending = sorted((float(at), dict(cmd)) for at, cmd in commands)
    idx = 0
    steps_max = int(round(float(max_s) / worker.DT))
    t0 = float(s['g']['t'])
    for _ in range(steps_max):
        t_now = float(s['g']['t'])
        while idx < len(pending) and t_now + 1e-9 >= t0 + pending[idx][0]:
            worker.apply(s, pending[idx][1])
            idx += 1
        if predicate(s):
            return capture(s), True
        worker.stepped(s)
        if s['fallen_at'] is not None:
            return capture(s), False
    return capture(s), bool(predicate(s))
