"""Hot-swappable motor engrams: runtime sensorimotor cores, C move compilation, and stunt bundles.

Motor engrams (`engrams`) encapsulate self-contained closed-loop programs -- compiled C sensed move
tables (`tools.cores.sensed`), latent mode wires (`ortho.ROW`), and beat-paced joint programs -- that
swap live into a running robot session with C0 pose continuity and lossless unload back to stock.
"""
import json
import math
from typing import Any

from machine.law import gait
from machine.moves import sensed
from machine.process.engram_catalog import CATALOG, STUNT_BUNDLES, SUITES, Engram
from machine.texts import ortho, takes

_CACHE = {}
_CUSTOM = {}

_MODE_INDEX = {
    'axial': 0, 'lateral': 1, 'drive': 2, 'pitch': 3, 'sagittal': 4, 'lift': 5,
    'width': 6, 'list': 7, 'turn': 8, 'arms': 9, 'rock': 10, 'reach': 11, 'elbows': 12,
}


def _mode_id(key):
    """Resolve mode name or integer index to a valid mode column index in `ortho.MODES`."""
    if isinstance(key, int):
        return key
    s = str(key).strip().lower()
    if s.isdigit():
        return int(s)
    if s in _MODE_INDEX:
        return _MODE_INDEX[s]
    return [str(m) for m in ortho.MODES].index(s)


def _sensed_tables(spec):
    """Build modified `machine.moves.sensed.Tables` from a stock defmove and structured overrides."""
    import numpy as np
    if 'tables' in spec:
        return spec['tables']
    base = sensed.tables(spec['base'])
    goal = base.goal.copy()
    rates = base.rates.copy()
    how = base.how.copy()
    scale = float(spec.get('rate_scale', 1.0))
    if scale != 1.0:
        active = how != sensed.HELD
        rates[active] = rates[active] * scale
    for phase_name, pscale in spec.get('phase_rate_scales', {}).items():
        idx = base.names.index(phase_name)
        active = how[idx] != sensed.HELD
        rates[idx, active] = rates[idx, active] * float(pscale)
    for phase_name, pgoals in spec.get('phase_goals', {}).items():
        idx = base.names.index(phase_name)
        for part, entry in pgoals.items():
            pidx = sensed.PARTS.index(part)
            deg, rate = float(entry[0]), float(entry[1])
            mode = float(entry[2]) if len(entry) > 2 else float(how[idx, pidx] or sensed.RATE)
            goal[idx, pidx], rates[idx, pidx], how[idx, pidx] = deg, rate, mode
    num_map = dict(zip(sensed.NUMBERS, base.numbers))
    num_map.update({k: float(v) for k, v in spec.get('constants', {}).items()})
    numbers = tuple(num_map[k] for k in sensed.NUMBERS)
    holds = bool(spec.get('holds', base.holds))
    return sensed.Tables(base.names, base.over.copy(), base.next.copy(), base.again.copy(),
                         base.counts.copy(), np.ascontiguousarray(goal, float),
                         np.ascontiguousarray(rates, float), np.ascontiguousarray(how, float),
                         base.done, numbers, holds)


def _scale_wire(raw_wire, tempo_scale=1.0, voice_scales=None, voice_offsets=None):
    """Time-scale and optionally adjust voiced targets on packed `ortho.ROW` bytes."""
    v_scales = {_mode_id(k): float(v) for k, v in (voice_scales or {}).items()}
    v_offsets = {_mode_id(k): float(v) for k, v in (voice_offsets or {}).items()}
    if abs(float(tempo_scale) - 1.0) < 1e-12 and not v_scales and not v_offsets:
        return raw_wire
    rows = []
    for row in ortho.rows(raw_wire):
        seq = int(row[0])
        at_ms = int(round(row[1] * float(tempo_scale)))
        ms = max(1, int(round(row[2] * float(tempo_scale))))
        flags = int(row[3])
        params = list(row[4:])
        for midx, sc in v_scales.items():
            val = params[ortho.Z + midx]
            if not math.isnan(val):
                params[ortho.Z + midx] = val * sc
        for midx, off in v_offsets.items():
            val = params[ortho.Z + midx]
            if not math.isnan(val):
                params[ortho.Z + midx] = val + off
        rows.append(ortho.ROW.pack(seq, at_ms, ms, flags, *params))
    return b''.join(rows)


def _take_wire(take_key, spec):
    """Build packed `ortho.ROW` bytes from mocap take `take_key` with voice and tempo controls."""
    voices = tuple(_mode_id(v) for v in spec['voices']) if 'voices' in spec else takes.VOICES.get(take_key, takes.STREAMED)
    t, z = takes.beats(take_key, voices, float(spec.get('since_s', 0.0)))
    z = z.copy()
    amp_scale = float(spec.get('amp_scale', 1.0))
    v_scales = {_mode_id(k): float(v) for k, v in spec.get('voice_scales', {}).items()}
    v_offsets = {_mode_id(k): float(v) for k, v in spec.get('voice_offsets', {}).items()}
    if len(z) > 0:
        z0 = z[0].copy()
        for i in voices:
            sc = amp_scale * v_scales.get(i, 1.0)
            off = v_offsets.get(i, 0.0)
            z[:, i] = z0[i] + sc * (z[:, i] - z0[i]) + off
    tempo = float(spec.get('tempo_scale', 1.0))
    enter_s = float(spec.get('enter_s', takes.ENTER_S))
    enter_ms = max(20, int(round(1e3 * enter_s * tempo)))
    gaps = [enter_ms] + [max(10, int(round(1e3 * (t[k] - t[k - 1]) * tempo))) for k in range(1, len(t))]
    rows, at = [], 0
    for k in range(len(t)):
        p = [math.nan] * ortho.P
        for i in voices:
            p[ortho.Z + i] = float(z[k, i])
        rows.append(ortho.ROW.pack(takes.TAKE_ID, at, gaps[k], ortho.CURVE, *p))
        at += gaps[k]
    if spec.get('rest_tail', True):
        rows.append(ortho.ROW.pack(takes.TAKE_ID, at, ortho.REST_MS, ortho.REST, *([math.nan] * ortho.P)))
    return b''.join(rows)


def _pattern_wire(spec):
    """Build a modified gait `ortho.ROW` from `spec['pattern']` and optional modifier overrides."""
    mods = takes.amounts(spec['pattern'])
    for k, sc in spec.get('amount_scales', {}).items():
        if k in mods:
            mods[k] = float(mods[k]) * float(sc)
    for k, delta in spec.get('modifiers', {}).items():
        mods[k] = float(mods.get(k, 0.0)) + float(delta)
    raw = gait.modified(float(spec.get('blend_s', takes.ENTER_S)), **mods)
    return _scale_wire(raw, spec.get('tempo_scale', 1.0), spec.get('voice_scales'), spec.get('voice_offsets'))


def _compile_wire(engram):
    """Compile a `'wire'` engram into `(slot, wire_bytes_or_none, names)`."""
    spec = engram.spec
    if 'take' in spec:
        wire = _take_wire(spec['take'], spec)
        names = tuple(spec.get('names', (engram.slot,)))
    elif 'pattern' in spec:
        wire = _pattern_wire(spec)
        names = tuple(spec.get('names', (engram.slot,)))
    elif 'base_wire' in spec:
        wire = _scale_wire(spec['base_wire'], spec.get('tempo_scale', 1.0),
                           spec.get('voice_scales'), spec.get('voice_offsets'))
        names = tuple(spec.get('names', (engram.slot,)))
    elif 'base_entry' in spec:
        wire = None
        names = tuple(spec.get('names', ()))
    else:
        wire = spec['wire']
        names = tuple(spec.get('names', (engram.slot,)))
    return (engram.slot, wire, names)


def compile_engram(engram):
    """Compile `engram` into its cached runtime payload (C `CustomMove`, `ortho` wire entry, or beat tuple)."""
    cached = _CACHE.get(engram.name)
    if cached is not None:
        return cached
    spec = engram.spec
    if engram.kind == 'sensed':
        from tools.cores import sensed as c_sensed
        payload = c_sensed.from_tables(engram.name, _sensed_tables(spec))
    elif engram.kind == 'wire':
        payload = _compile_wire(engram)
    elif engram.kind == 'program':
        payload = tuple((str(b), float(sec), dict(goals)) for b, sec, goals in spec['beats'])
    else:
        raise ValueError('unknown engram kind %r' % (engram.kind,))
    if engram.kind != 'wire' or 'base_entry' not in spec:
        _CACHE[engram.name] = payload
    return payload


def register(items):
    """Register one or more runtime `Engram` objects (bypassing repo catalog mutation)."""
    seq = [items] if isinstance(items, Engram) else items
    for eng in seq:
        _CACHE.pop(eng.name, None)
        _CUSTOM[eng.name] = eng
    return _CUSTOM


def load_specs(source):
    """Load `Engram` records from a JSON file path or sequence of spec dicts and register them."""
    if isinstance(source, str):
        with open(source, encoding='utf-8') as f:
            raw = json.load(f)
    else:
        raw = source
    rows = raw.get('engrams', raw) if isinstance(raw, dict) else raw
    loaded = {}
    for row in rows:
        eng = row if isinstance(row, Engram) else Engram(
            row['name'], row['slot'], row['kind'], row['task'],
            row.get('stunt_ref'), row.get('description', ''), row['spec'])
        _CACHE.pop(eng.name, None)
        _CUSTOM[eng.name] = eng
        loaded[eng.name] = eng
    return loaded


def get(engram):
    """Resolve an `Engram` instance or registered name from `CATALOG` or runtime custom registry."""
    if isinstance(engram, Engram):
        return engram
    if engram in _CUSTOM:
        return _CUSTOM[engram]
    return CATALOG[engram]


def variant(base_name, name, rng):
    """Synthesize a bounded parametric `Engram` mutation from `base_name` using `rng`."""
    base = get(base_name)
    spec: dict[str, Any] = {k: (dict(v) if isinstance(v, dict) else v) for k, v in base.spec.items()}
    if base.kind == 'sensed':
        tb = sensed.tables(spec['base'])
        spec['rate_scale'] = round(float(spec.get('rate_scale', 1.0)) * rng.uniform(0.82, 1.18), 3)
        prs = dict(spec.get('phase_rate_scales', {}))
        active_phases = [n for n in tb.names if n != 'done']
        if active_phases:
            ph = rng.choice(active_phases)
            prs[ph] = round(float(prs.get(ph, 1.0)) * rng.uniform(0.78, 1.18), 3)
            spec['phase_rate_scales'] = prs
        num_map = dict(zip(sensed.NUMBERS, tb.numbers))
        consts = dict(spec.get('constants', {}))
        if num_map.get('pull-in', 0.0) > 0.0:
            consts['pull-in'] = round(max(0.14, float(consts.get('pull-in', num_map['pull-in'])) + rng.uniform(-0.02, 0.03)), 3)
        if consts:
            spec['constants'] = consts
    elif base.kind == 'wire':
        spec['tempo_scale'] = round(float(spec.get('tempo_scale', 1.0)) * rng.uniform(0.84, 1.24), 3)
        if 'take' in spec:
            spec['amp_scale'] = round(float(spec.get('amp_scale', 1.0)) * rng.uniform(0.86, 1.08), 3)
    return Engram(name, base.slot, base.kind, base.task, base.stunt_ref,
                  'Parametric variant of %s' % base.name, spec)


def _ensure_bank(g):
    """Ensure `g['bank']` has initialized engram tracking containers."""
    from machine.process import bank
    if g.get('bank') is None:
        bank.laid(g)
    bk = g['bank']
    bk.setdefault('engrams', {})
    bk.setdefault('stock', {})
    bk.setdefault('active', {})
    bk.setdefault('programs', {})
    return bk


def _build_custom_slot(g, world, slot_name, move_obj, keep=''):
    """Pre-allocate persistent drive/pass buffers and C `wbh_sil_played` handle for custom C move."""
    import numpy as np
    from machine.body import figure
    from machine.process import joints
    from machine.senses import edge
    kept = {j for grp in str(keep).split() for j in joints.KEPT[grp]}
    mine = [i for i, j in enumerate(figure.JOINTS) if j not in kept]
    j_idx = np.array([i for i in g['joints'] if i in mine], np.int32)
    kp = np.asarray(joints.KP)[j_idx]
    kd = np.asarray(joints.KD)[j_idx]
    feed = np.zeros(len(j_idx))
    target = np.zeros(len(j_idx))
    limit = np.zeros(len(j_idx))
    pose = np.degrees(g['sense'][edge.Q]).copy()
    cmd = pose.copy()
    state = move_obj.laid(pose, g['t'])
    slot = {
        'name': slot_name, 'keep': str(keep), 'move': move_obj, 'when': None, 'at': None,
        'pose': pose, 'cmd': cmd, 'state': state, 'last': g['t'], 'gains': None,
        'mine': mine, 'joints': j_idx, 'kp': kp, 'kd': kd, 'feed': feed,
        'target': target, 'limit': limit,
        'sensed': getattr(move_obj, 'SENSED', joints.SENSED),
        'holds': getattr(move_obj, 'HOLDS', True),
        'warm': True, 'enter_t0': None,
    }
    slot['drive'] = edge.bound(world, j_idx, target=target, feed=feed, kp=kp, limit=limit)
    c_played = joints._played()
    slot['_played_fn'] = c_played.laid
    slot['pass'] = c_played.laid(g, world, slot)
    return slot


def _resolve_wire_entry(g, engram):
    """Build `(slot, wire, names)` for a wire engram, resolving `base_entry` from `g['gym']` if needed."""
    if 'base_entry' not in engram.spec:
        return _compile_wire(engram)
    base_name = engram.spec['base_entry']
    base_entry = next((e for e in g['gym'] if e[0] == base_name), None)
    if base_entry is None:
        raise KeyError('wire engram %s: missing gym base entry %r' % (engram.name, base_name))
    wire = _scale_wire(base_entry[1], engram.spec.get('tempo_scale', 1.0),
                       engram.spec.get('voice_scales'), engram.spec.get('voice_offsets'))
    names = tuple(engram.spec.get('names', base_entry[2]))
    return (engram.slot, wire, names)


def _part_rates(move_obj, phase_idx, cmd):
    """Signed instantaneous slew rates (deg/s) for active non-HELD joints at `phase_idx`."""
    import numpy as np
    from machine.moves import anatomy as an, sensed as py_sensed
    tb = getattr(move_obj, 'tables', None)
    if tb is None or not (0 <= phase_idx < len(tb.names)):
        return None
    x = cmd[py_sensed.JOINT]
    how = tb.how[phase_idx, py_sensed.PART]
    goal = an.aimed(how, tb.goal[phase_idx, py_sensed.PART], x)
    rates = tb.rates[phase_idx, py_sensed.PART]
    active = (how == an.RATE) | (how == an.LEAST)
    diff = goal - x
    return np.where(active & (np.abs(diff) > tb.numbers[-1]), np.sign(diff) * rates, 0.0)


def load(g, engram, keep=''):
    """Pre-warm `engram` in `g['bank']['engrams']` without activating or replacing any running slot."""
    engram = get(engram)
    bk = _ensure_bank(g)
    key = (engram.name, str(keep))
    cached = bk['engrams'].get(key)
    if cached is not None and cached[0] == engram:
        return cached[1]
    if engram.kind == 'sensed':
        mv = compile_engram(engram)
        built = _build_custom_slot(g, g['world'], engram.slot, mv, keep=keep)
    elif engram.kind == 'wire':
        from machine.process import bank
        entry = _resolve_wire_entry(g, engram)
        if entry[1] is not None:
            bank._slots_table(g, entry[1], False)
            bank._slots_table(g, entry[1], True)
        built = entry
    elif engram.kind == 'program':
        built = compile_engram(engram)
    else:
        raise ValueError('unsupported engram kind %r' % (engram.kind,))
    bk['engrams'][key] = (engram, built)
    return built


def _handover_live_slot(g, prev_slot, new_slot):
    """Perform bumpless C0 handover with phase preservation and genuine discontinuity telemetry."""
    import numpy as np
    from machine.moves import sensed as py_sensed
    from machine.senses import edge
    start = np.asarray(prev_slot['cmd'], float).copy()
    prev_state = None if prev_slot.get('state') is None else np.asarray(prev_slot['state'], float).copy()
    prev_tb = getattr(prev_slot.get('move'), 'tables', None)
    new_tb = getattr(new_slot.get('move'), 'tables', None)
    new_slot['pose'][:] = start
    new_slot['cmd'][:] = start
    if np.array_equal(prev_slot['joints'], new_slot['joints']):
        new_slot['target'][:] = prev_slot['target']
        new_slot['feed'][:] = prev_slot['feed']
        new_slot['limit'][:] = prev_slot['limit']
    else:
        new_slot['target'][:] = np.radians(start[new_slot['joints']])
        new_slot['feed'][:] = 0.0
        new_slot['limit'][:] = g['clamp'][new_slot['joints']]
    new_slot['gains'] = None if prev_slot.get('gains') is None else prev_slot['gains'].copy()
    new_slot['enter_t0'] = prev_slot.get('enter_t0')
    new_slot['when'] = prev_slot.get('when')
    new_slot['at'] = prev_slot.get('at')
    new_slot['last'] = prev_slot.get('last', g['t'])
    new_slot['state'][:] = new_slot['move'].laid(start, g['t'])
    phase_before = phase_after = None
    phase_preserved = False
    if prev_state is not None and len(prev_state) > py_sensed.WAY and len(new_slot['state']) > py_sensed.WAY:
        new_slot['state'][py_sensed.WAY] = prev_state[py_sensed.WAY]
        new_slot['state'][py_sensed.COUNT] = prev_state[py_sensed.COUNT]
        if prev_tb is not None and new_tb is not None:
            p_idx = int(prev_state[py_sensed.PHASE])
            if 0 <= p_idx < len(prev_tb.names):
                phase_before = prev_tb.names[p_idx]
                if phase_before in new_tb.names:
                    n_idx = new_tb.names.index(phase_before)
                    phase_preserved = True
                elif p_idx >= prev_tb.done:
                    n_idx = new_tb.done
                else:
                    n_idx = min(new_tb.done - 1, int(round(p_idx * new_tb.done / max(1, prev_tb.done))))
                new_slot['state'][py_sensed.PHASE] = float(n_idx)
                phase_after = new_tb.names[n_idx]
    g['played'] = new_slot
    cmd_jump = float(np.max(np.abs(new_slot['cmd'] - start)))
    track_err = float(np.max(np.abs(new_slot['cmd'] - np.degrees(g['sense'][edge.Q]))))
    rate_jump = 0.0
    if prev_state is not None and prev_tb is not None and new_tb is not None:
        r0 = _part_rates(prev_slot['move'], int(prev_state[py_sensed.PHASE]), start)
        r1 = _part_rates(new_slot['move'], int(new_slot['state'][py_sensed.PHASE]), start)
        if r0 is not None and r1 is not None:
            rate_jump = float(np.max(np.abs(r1 - r0)))
    return {
        'pose_jump_deg': cmd_jump, 'tracking_err_deg': track_err, 'rate_jump_deg_s': rate_jump,
        'phase_before': phase_before, 'phase_after': phase_after, 'phase_preserved': phase_preserved,
    }


def swap(g, engram, keep='', s=None):
    """Hot-swap `engram` (instance or registered name) into `g` (and `s` if given) with C0 continuity."""
    engram = get(engram)
    bk = _ensure_bank(g)
    payload = load(g, engram, keep=keep)
    swapped_live = False
    hv = {'pose_jump_deg': 0.0, 'tracking_err_deg': 0.0, 'rate_jump_deg_s': 0.0,
          'phase_before': None, 'phase_after': None, 'phase_preserved': False}
    if engram.kind == 'sensed':
        skey = ('sensed', engram.slot, str(keep))
        if skey not in bk['stock'] and (engram.slot, str(keep)) in bk['moves']:
            bk['stock'][skey] = bk['moves'][(engram.slot, str(keep))]
        prev = g.get('played')
        bk['moves'][(engram.slot, str(keep))] = payload
        if isinstance(prev, dict) and prev.get('name') == engram.slot and prev.get('keep', '') == str(keep):
            swapped_live = True
            hv = _handover_live_slot(g, prev, payload)
    elif engram.kind == 'wire':
        skey = ('wire', engram.slot, '')
        gym = list(g['gym'])
        idx = next((i for i, e in enumerate(gym) if e[0] == engram.slot), None)
        if skey not in bk['stock']:
            bk['stock'][skey] = (idx is not None, gym[idx] if idx is not None else None)
        if idx is not None:
            gym[idx] = payload
        else:
            gym.append(payload)
        g['gym'] = gym
        if s is not None:
            s['gym'] = list(gym)
            s['says'] = [s['gym_name']] + [entry[0] for entry in s['gym']]
    elif engram.kind == 'program':
        skey = ('program', engram.slot, '')
        if skey not in bk['stock']:
            bk['stock'][skey] = bk['programs'].get(engram.slot)
        bk['programs'][engram.slot] = payload
    bk['active'][(engram.slot, str(keep))] = engram.name
    return {'engram': engram.name, 'slot': engram.slot, 'swapped_live': swapped_live, **hv}


def unload(g, slot, keep='', s=None):
    """Restore stock definition for `slot` in `g` (and `s` if provided) with live C0 handover."""
    bk = _ensure_bank(g)
    keep = str(keep)
    for kind in ('sensed', 'wire', 'program'):
        skey = (kind, slot, keep)
        if skey not in bk['stock']:
            continue
        orig = bk['stock'].pop(skey)
        if kind == 'sensed':
            prev = g.get('played')
            bk['moves'][(slot, keep)] = orig
            if isinstance(prev, dict) and prev.get('name') == slot and prev.get('keep', '') == keep:
                _handover_live_slot(g, prev, orig)
        elif kind == 'wire':
            had, entry = orig
            gym = list(g['gym'])
            idx = next((i for i, e in enumerate(gym) if e[0] == slot), None)
            if had and idx is not None:
                gym[idx] = entry
            elif not had and idx is not None:
                gym.pop(idx)
            g['gym'] = gym
            if s is not None:
                s['gym'] = list(gym)
                s['says'] = [s['gym_name']] + [e[0] for e in s['gym']]
        elif kind == 'program':
            if orig is None:
                bk['programs'].pop(slot, None)
            else:
                bk['programs'][slot] = orig
    bk['active'].pop((slot, keep), None)


def unload_all(g, s=None):
    """Unload all active custom engrams in `g` back to their captured stock definitions."""
    bk = g.get('bank')
    if not bk:
        return
    for slot, keep in list(bk.get('active', ())):
        unload(g, slot, keep=keep, s=s)


def active(g):
    """Return active `{slot: engram_name}` mapping on `g`."""
    bk = g.get('bank') or {}
    return {slot: name for (slot, _keep), name in bk.get('active', {}).items()}


def catalog(task=None):
    """Return `Engram` records filtered to `task` (or all when `task` is None)."""
    merged = {**CATALOG, **_CUSTOM}
    if task is None:
        return merged
    return {k: v for k, v in merged.items() if v.task == task}


def suites():
    """Return prepared combinatorial engram candidate lists by task and slot."""
    return {task: {slot: tuple(items) for slot, items in slots.items()} for task, slots in SUITES.items()}


def stunt_bundle(stunt_key):
    """Return default verified engram bundle for `stunt_key`, or empty tuple when none."""
    return tuple(STUNT_BUNDLES.get(stunt_key, ()))
