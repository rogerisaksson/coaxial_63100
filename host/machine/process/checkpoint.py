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

from machine.world import loose, steps

#: Static or runtime-service keys in `g` whose identity is invariant across deterministic stepping.
_STATIC_G = frozenset({
    'world', 'body', 'beat', 'solves', 'sensed', 'planner', 'vision', 'keep', 'gym', 'waits',
    'surety_field', 'touch_step', 'twist_step', 'latent', 'couple', 'borne', 'em_supports',
    'leaned', 'hull', 'upright', 'bodily', 'estimated', 'felt', 'weighed', 'clear', 'gap',
    'rooms', 'outside', 'sloped', 'readings', 'read', 'modes', 'pitched', 'capped', 'told_on',
    'settling', 'bounds', 'gripped', 'sured', 'released', 'poses', 'ringed', 'ringed_ahead',
    'seek', 'kept', 'shield', 'impedance', 'braced', 'inekf_points', 'inekf_step', 'inekf_glue',
    'schema_at', 'schema_body', '_skin_ids', '_tips', '_cv_sh', '_acv_sh', '_derate', 'grip_at',
    'joints', 'legs', 'leg_joints', 'limbs', 'cervical', 'waist_idx', 'head_idx', 'situations',
    'rng', 'q_lo', 'q_hi', 'clamp', 'wep_lim', 'free_kp', 'rate_hz', 'every', 'texts', 'suite',
    'bend', 'arms', 'reaches', 'folds', 'distal', 'distal_side', '_stand_entry', '_brake_wire',
    'drives', 'bus', 'tick', 'observer', 'heat', 'registry', 'bank', 'played',
})

#: Slot array buffers inside each `g['bank']['moves'][k]` restored in place to keep `Played` valid.
_SLOT_ARRAYS = ('pose', 'cmd', 'state', 'feed', 'target', 'limit')
_SLOT_SCALARS = ('when', 'at', 'last', 'enter_t0')


def _copy_val(val):
    """Clone mutable values while leaving callable or opaque C handles untouched."""
    import numpy as np
    if isinstance(val, np.ndarray):
        return val.copy()
    if isinstance(val, dict):
        return {k: _copy_val(v) for k, v in val.items()}
    if isinstance(val, list):
        return [_copy_val(v) for v in val]
    if isinstance(val, tuple):
        return tuple(_copy_val(v) for v in val)
    return val


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
        elif k in dst and isinstance(dst[k], list) and isinstance(src, list):
            dst[k][:] = _copy_val(src)
        else:
            dst[k] = _copy_val(src)


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
    """Capture `g['bank']` move slots, handover state, custom programs, and `g['played']`."""
    bank = g.get('bank')
    played = g.get('played')
    played_kind, played_val = None, None
    if isinstance(played, dict):
        matched = next((k for k, sl in (bank or {}).get('moves', {}).items() if sl is played), None)
        if matched is not None:
            played_kind, played_val = 'slot', matched
        else:
            played_kind, played_val = 'dict', _copy_val(played)
    slots = {}
    if bank is not None:
        for k, sl in bank['moves'].items():
            slots[k] = {
                'arrays': {a: sl[a].copy() for a in _SLOT_ARRAYS},
                'scalars': {a: sl[a] for a in _SLOT_SCALARS},
                'gains': None if sl['gains'] is None else sl['gains'].copy(),
            }
    return {
        'slots': slots,
        'handover': _copy_val(bank['handover']) if bank is not None else None,
        'programs': _copy_val(bank.get('programs', {})) if bank is not None else {},
        'played': (played_kind, played_val),
    }


def _restore_bank(g, snap):
    """Restore `g['bank']` move slots in place and re-link `g['played']`."""
    import numpy as np
    bank = g.get('bank')
    if bank is not None:
        for k, saved in snap['slots'].items():
            sl = bank['moves'].get(k)
            if sl is None:
                continue
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
        bank['handover'] = _copy_val(snap['handover'])
        if 'programs' in bank or snap['programs']:
            bank['programs'] = _copy_val(snap['programs'])
    pk, pv = snap['played']
    if pk == 'slot' and bank is not None:
        g['played'] = bank['moves'][pv]
    elif pk == 'dict':
        g['played'] = _copy_val(pv)
    else:
        g['played'] = None


def _capture_g(g):
    """Capture mutable `g` arrays, C structs (`observer`, `bus`, `heat`, `tick`), and scalars."""
    import numpy as np
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
    dynamic = {k: _copy_val(v) for k, v in g.items() if k not in _STATIC_G}
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
        'bank': _capture_bank(g),
        'dynamic': dynamic,
        '_dynamic_inplace': [k for k, v in g.items()
                             if k not in _STATIC_G and isinstance(v, np.ndarray)],
    }


def _restore_g(g, snap):
    """Restore mutable `g` state bit for bit without reallocating C-pinned buffers."""
    import numpy as np
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
    _restore_bank(g, snap['bank'])
    saved = snap['dynamic']
    inplace = set(snap['_dynamic_inplace'])
    for k in list(g):
        if k not in _STATIC_G and k not in saved:
            del g[k]
    for k, src in saved.items():
        if (k in inplace and k in g and isinstance(g[k], np.ndarray)
                and isinstance(src, np.ndarray) and g[k].shape == src.shape and g[k].dtype == src.dtype):
            np.copyto(g[k], src)
        elif k in g and isinstance(g[k], dict) and isinstance(src, dict):
            _restore_inplace_dict(g[k], src)
        else:
            g[k] = _copy_val(src)


def capture(s):
    """Capture a bit-exact in-memory snapshot of worker session `s`."""
    worn = s['worn']
    if worn.due_at is not None and worn.got is None and worn.parts is not None:
        while True:
            try:
                next(worn.parts)
            except StopIteration as end:
                worn.got = end.value
                break
    return {
        'world': _capture_world(s['g']['world']),
        'g': _capture_g(s['g']),
        's': {
            'passes': s['passes'],
            'fallen_at': s['fallen_at'],
            'shutdown': s['shutdown'],
            'refused': s['refused'],
            'epoch': s['epoch'],
            'torque': s['torque'].copy(),
            'power': s['power'].copy(),
            'watts': s['watts'],
            'ratio': s['ratio'],
            'headroom': s['headroom'],
            'wear_told': copy.deepcopy(s['wear_told']),
            'wear': _copy_val(s['wear']),
            'worn_ring': (worn.ring.rows.copy(), int(worn.ring.start), int(worn.ring.n)),
            'worn_state': (_copy_val(worn.got), worn.due_at),
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
    np.copyto(s['torque'], ss['torque'])
    np.copyto(s['power'], ss['power'])
    s['wear_told'] = copy.deepcopy(ss['wear_told'])
    _restore_inplace_dict(s['wear'], ss['wear'])
    worn = s['worn']
    rows, start, n = ss['worn_ring']
    np.copyto(worn.ring.rows, rows)
    worn.ring.start, worn.ring.n = start, n
    worn.w = s['wear']
    worn.got, worn.due_at = _copy_val(ss['worn_state'][0]), ss['worn_state'][1]
    worn.parts = iter(()) if worn.due_at is not None else None


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
