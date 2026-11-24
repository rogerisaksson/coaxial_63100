"""Hot-swappable motor engrams: runtime sensorimotor cores, C move compilation, and stunt catalogs.

Motor engrams (`engrams`) encapsulate self-contained closed-loop programs -- compiled C sensed move
tables (`tools.cores.sensed`), latent mode wires (`ortho.ROW`), and beat-paced joint programs -- that
swap live into a running robot session with C0 pose continuity and lossless unload back to stock.
"""
import collections
import math

from machine.moves import anatomy, sensed
from machine.texts import ortho, takes

#: Runtime hot-swappable sensorimotor module bound to a controller slot (`sensed`, `wire`, or `program`).
Engram = collections.namedtuple('Engram', 'name slot kind task stunt_ref description spec')

_CACHE = {}


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


def _scale_wire(raw_wire, tempo_scale=1.0):
    """Time-scale packed `ortho.ROW` bytes by `tempo_scale`."""
    if abs(float(tempo_scale) - 1.0) < 1e-12:
        return raw_wire
    rows = []
    for row in ortho.rows(raw_wire):
        seq, at_ms, ms, flags = int(row[0]), int(round(row[1] * tempo_scale)), max(1, int(round(row[2] * tempo_scale))), int(row[3])
        rows.append(ortho.ROW.pack(seq, at_ms, ms, flags, *row[4:]))
    return b''.join(rows)


def _take_wire(take_key, tempo_scale=1.0, amp_scale=1.0, enter_s=takes.ENTER_S, rest_tail=True):
    """Build packed `ortho.ROW` wire bytes from mocap take `take_key` with optional tempo and amplitude scaling."""
    voices = takes.VOICES.get(take_key, takes.STREAMED)
    t, z = takes.beats(take_key, voices, 0.0)
    z = z.copy()
    if abs(float(amp_scale) - 1.0) > 1e-12 and len(z) > 0:
        z0 = z[0].copy()
        for i in voices:
            z[:, i] = z0[i] + float(amp_scale) * (z[:, i] - z0[i])
    enter_ms = max(20, int(round(1e3 * float(enter_s) * float(tempo_scale))))
    gaps = [enter_ms] + [max(10, int(round(1e3 * (t[k] - t[k - 1]) * float(tempo_scale)))) for k in range(1, len(t))]
    rows, at = [], 0
    for k in range(len(t)):
        p = [math.nan] * ortho.P
        for i in voices:
            p[ortho.Z + i] = float(z[k, i])
        rows.append(ortho.ROW.pack(takes.TAKE_ID, at, gaps[k], ortho.CURVE, *p))
        at += gaps[k]
    if rest_tail:
        rows.append(ortho.ROW.pack(takes.TAKE_ID, at, ortho.REST_MS, ortho.REST, *([math.nan] * ortho.P)))
    return b''.join(rows)


def _compile_wire(engram):
    """Compile a `'wire'` engram into `(slot, wire_bytes_or_none, names)`."""
    spec = engram.spec
    if 'take' in spec:
        wire = _take_wire(spec['take'], spec.get('tempo_scale', 1.0), spec.get('amp_scale', 1.0),
                          spec.get('enter_s', takes.ENTER_S), spec.get('rest_tail', True))
        names = tuple(spec.get('names', (engram.slot,)))
    elif 'pattern' in spec:
        wire = _scale_wire(takes.pattern(spec['pattern'], blend_s=spec.get('blend_s', takes.ENTER_S)),
                           spec.get('tempo_scale', 1.0))
        names = tuple(spec.get('names', (engram.slot,)))
    elif 'base_wire' in spec:
        wire = _scale_wire(spec['base_wire'], spec.get('tempo_scale', 1.0))
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
    wire = _scale_wire(base_entry[1], engram.spec.get('tempo_scale', 1.0))
    names = tuple(engram.spec.get('names', base_entry[2]))
    return (engram.slot, wire, names)


def load(g, engram, keep=''):
    """Pre-warm `engram` in `g['bank']['engrams']` without activating or replacing any running slot."""
    engram = CATALOG[engram] if isinstance(engram, str) else engram
    bk = _ensure_bank(g)
    key = (engram.name, str(keep))
    if key in bk['engrams']:
        return bk['engrams'][key]
    if engram.kind == 'sensed':
        mv = compile_engram(engram)
        built = _build_custom_slot(g, g['world'], engram.slot, mv, keep=keep)
    elif engram.kind == 'wire':
        from machine.process import bank
        entry = _resolve_wire_entry(g, engram)
        bank._slots_table(g, entry[1], False)
        bank._slots_table(g, entry[1], True)
        built = entry
    elif engram.kind == 'program':
        built = compile_engram(engram)
    else:
        raise ValueError('unsupported engram kind %r' % (engram.kind,))
    bk['engrams'][key] = built
    return built


def _handover_live_slot(g, prev_slot, new_slot):
    """Perform bumpless C0 pose handover when hot-swapping an active move mid-execution."""
    import numpy as np
    from machine.moves import sensed as py_sensed
    pose_jump = float(np.max(np.abs(new_slot['cmd'] - prev_slot['cmd'])))
    start = np.asarray(prev_slot['cmd'], float).copy()
    new_slot['pose'][:] = start
    new_slot['cmd'][:] = start
    new_slot['gains'] = None if prev_slot.get('gains') is None else prev_slot['gains'].copy()
    new_slot['enter_t0'] = prev_slot.get('enter_t0')
    new_slot['when'] = prev_slot.get('when')
    new_slot['at'] = g['t'] if prev_slot.get('at') is not None else None
    new_slot['last'] = g['t']
    new_slot['state'][:] = new_slot['move'].laid(start, g['t'])
    if prev_slot.get('state') is not None and len(prev_slot['state']) > py_sensed.WAY and len(new_slot['state']) > py_sensed.WAY:
        new_slot['state'][py_sensed.WAY] = prev_slot['state'][py_sensed.WAY]
    g['played'] = new_slot
    return float(np.max(np.abs(new_slot['cmd'] - start))) if pose_jump >= 0.0 else 0.0


def swap(g, engram, keep='', s=None):
    """Hot-swap `engram` (instance or `CATALOG` name) into `g` (and session `s` if given) with C0 continuity."""
    engram = CATALOG[engram] if isinstance(engram, str) else engram
    bk = _ensure_bank(g)
    payload = load(g, engram, keep=keep)
    swapped_live = False
    pose_jump = 0.0
    if engram.kind == 'sensed':
        skey = ('sensed', engram.slot, str(keep))
        if skey not in bk['stock'] and (engram.slot, str(keep)) in bk['moves']:
            bk['stock'][skey] = bk['moves'][(engram.slot, str(keep))]
        prev = g.get('played')
        bk['moves'][(engram.slot, str(keep))] = payload
        if isinstance(prev, dict) and prev.get('name') == engram.slot:
            swapped_live = True
            pose_jump = _handover_live_slot(g, prev, payload)
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
    return {'engram': engram.name, 'slot': engram.slot, 'swapped_live': swapped_live, 'pose_jump_deg': pose_jump}


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
            if isinstance(prev, dict) and prev.get('name') == slot:
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


#: Prepared catalog of motor engrams modeled on natural human mocap takes in `stunts/`.
CATALOG = {
    'chair_mount_stock': Engram(
        'chair_mount_stock', 'chair_mount', 'sensed', 'chair', 'stand_to_sit',
        'Stock chair-mount sensed move table',
        {'base': 'chair-mount'},
    ),
    'chair_mount_calm': Engram(
        'chair_mount_calm', 'chair_mount', 'sensed', 'chair', 'stand_to_sit',
        'Slower vault and lap transition aligned with Stand To Sit cadence',
        {'base': 'chair-mount', 'rate_scale': 0.82, 'constants': {'pull-in': 0.20}},
    ),
    'chair_mount_deep_pull': Engram(
        'chair_mount_deep_pull', 'chair_mount', 'sensed', 'chair', 'stand_to_sit',
        'Deeper posterior pull-in before arm opening for secure seat placement',
        {'base': 'chair-mount', 'rate_scale': 0.90, 'constants': {'pull-in': 0.22}},
    ),
    'chair_mount_soft_arms': Engram(
        'chair_mount_soft_arms', 'chair_mount', 'sensed', 'chair', 'stand_to_sit',
        'Reduced upper-limb slew during open and lap phases to cut hand jerk',
        {'base': 'chair-mount', 'phase_rate_scales': {'open': 0.72, 'lap': 0.75},
         'constants': {'pull-in': 0.20}},
    ),
    'chair_mount_brisk': Engram(
        'chair_mount_brisk', 'chair_mount', 'sensed', 'chair', 'stand_to_sit',
        'Faster arm vault and pull-in rates for rapid docking',
        {'base': 'chair-mount', 'rate_scale': 1.15},
    ),
    'chair_dismount_stock': Engram(
        'chair_dismount_stock', 'chair_dismount', 'sensed', 'chair', 'sit_to_stand',
        'Stock chair-dismount sensed move table',
        {'base': 'chair-dismount'},
    ),
    'chair_dismount_forward': Engram(
        'chair_dismount_forward', 'chair_dismount', 'sensed', 'chair', 'sit_to_stand',
        'Enhanced forward trunk lean and dorsiflexion aligned with Sit To Stand',
        {'base': 'chair-dismount', 'rate_scale': 0.95,
         'phase_goals': {'lean': {'hip': (-120.0, 72.0, anatomy.RATE),
                                  'ankle': (-14.0, 44.0, anatomy.RATE)}}},
    ),
    'chair_dismount_calm': Engram(
        'chair_dismount_calm', 'chair_dismount', 'sensed', 'chair', 'sit_to_stand',
        'Reduced dismount slew to limit chair pitch recoil during weight transfer',
        {'base': 'chair-dismount', 'rate_scale': 0.82},
    ),
    'chair_dismount_brisk': Engram(
        'chair_dismount_brisk', 'chair_dismount', 'sensed', 'chair', 'sit_to_stand',
        'Brisk foot plant and press rates matching faster sit-to-stand transfer',
        {'base': 'chair-dismount', 'rate_scale': 1.15},
    ),
    'bed_entry_stock': Engram(
        'bed_entry_stock', 'bed_entry', 'sensed', 'bed', 'lying_down_from_sitting',
        'Stock bed-entry sensed move table',
        {'base': 'bed-entry'},
    ),
    'bed_entry_gentle': Engram(
        'bed_entry_gentle', 'bed_entry', 'sensed', 'bed', 'lying_down_from_sitting',
        'Smoother recline and leg fold matching Lying Down from Sitting',
        {'base': 'bed-entry', 'rate_scale': 0.85, 'phase_rate_scales': {'lie': 0.78, 'fold': 0.85}},
    ),
    'bed_entry_deep': Engram(
        'bed_entry_deep', 'bed_entry', 'sensed', 'bed', 'lying_down_from_sitting',
        'Deeper initial bed pull-in before supine transition',
        {'base': 'bed-entry', 'rate_scale': 0.92, 'constants': {'pull-in': 0.33}},
    ),
    'bed_entry_compact': Engram(
        'bed_entry_compact', 'bed_entry', 'sensed', 'bed', 'lying_down_from_sitting',
        'Slightly brisker bridging cadence for compact bed clearance',
        {'base': 'bed-entry', 'rate_scale': 1.05, 'constants': {'near': 0.04}},
    ),
    'crouch_down_mocap': Engram(
        'crouch_down_mocap', 'crouch_down', 'wire', 'crouch', 'crouching_down',
        'Mocap-derived crouching-down latent curve from Crouching Down take',
        {'take': 'crouching_down', 'tempo_scale': 1.0, 'amp_scale': 1.0, 'rest_tail': False},
    ),
    'crouch_down_calm': Engram(
        'crouch_down_calm', 'crouch_down', 'wire', 'crouch', 'crouching_down',
        'Time-stretched, damped crouch descent from Crouching Down take',
        {'take': 'crouching_down', 'tempo_scale': 1.25, 'amp_scale': 0.92, 'rest_tail': False},
    ),
    'crouch_down_brisk': Engram(
        'crouch_down_brisk', 'crouch_down', 'wire', 'crouch', 'crouching_down',
        'Faster crouch descent from Crouching Down take',
        {'take': 'crouching_down', 'tempo_scale': 0.85, 'amp_scale': 0.95, 'rest_tail': False},
    ),
    'crouch_up_mocap': Engram(
        'crouch_up_mocap', 'crouch_up', 'wire', 'crouch', 'crouch_up',
        'Mocap-derived crouch recovery curve from Crouch Up take',
        {'take': 'crouch_up', 'tempo_scale': 1.0, 'amp_scale': 1.0, 'rest_tail': True},
    ),
    'crouch_up_calm': Engram(
        'crouch_up_calm', 'crouch_up', 'wire', 'crouch', 'crouch_up',
        'Calm crouch recovery curve returning cleanly to rest',
        {'take': 'crouch_up', 'tempo_scale': 1.25, 'amp_scale': 1.0, 'rest_tail': True},
    ),
    'kneel_down_mocap': Engram(
        'kneel_down_mocap', 'kneel_down', 'wire', 'kneel', 'kneeling_down',
        'Mocap-derived kneeling descent from Kneeling Down take',
        {'take': 'kneeling_down', 'tempo_scale': 1.0, 'amp_scale': 0.90, 'rest_tail': False},
    ),
    'kneel_down_calm': Engram(
        'kneel_down_calm', 'kneel_down', 'wire', 'kneel', 'kneeling_down',
        'Calm kneeling descent scaled from Kneeling Down take',
        {'take': 'kneeling_down', 'tempo_scale': 1.25, 'amp_scale': 0.85, 'rest_tail': False},
    ),
    'turn_left_stock': Engram(
        'turn_left_stock', 'turn_left', 'wire', 'turn', 'left_turn',
        'Stock turn_left orthokinematic table',
        {'base_entry': 'turn_left', 'tempo_scale': 1.0},
    ),
    'turn_left_calm': Engram(
        'turn_left_calm', 'turn_left', 'wire', 'turn', 'left_turn',
        'Extended blend turn_left matching softer Left Turn yaw onset',
        {'base_entry': 'turn_left', 'tempo_scale': 1.25},
    ),
    'turn_left_brisk': Engram(
        'turn_left_brisk', 'turn_left', 'wire', 'turn', 'left_turn',
        'Compressed blend turn_left for snappier Left Turn pivot',
        {'base_entry': 'turn_left', 'tempo_scale': 0.82},
    ),
    'walk_stock': Engram(
        'walk_stock', 'walk', 'wire', 'gait', 'walking_arc_left',
        'Stock walk gait row',
        {'base_entry': 'walk', 'tempo_scale': 1.0},
    ),
    'walk_arc_pattern': Engram(
        'walk_arc_pattern', 'walk', 'wire', 'gait', 'walking_arc_left',
        'Walk gait modified by Walking Arc Left spectral CPG modifiers',
        {'pattern': 'walking_arc_left', 'tempo_scale': 1.0},
    ),
}

#: Prepared candidate engram lists grouped by controller slot for combinatorial Monte Carlo sweeps.
SUITES = {
    'chair': {
        'mount': ('chair_mount_stock', 'chair_mount_calm', 'chair_mount_deep_pull',
                  'chair_mount_soft_arms', 'chair_mount_brisk'),
        'dismount': ('chair_dismount_stock', 'chair_dismount_forward', 'chair_dismount_calm',
                     'chair_dismount_brisk'),
    },
    'bed': {
        'entry': ('bed_entry_stock', 'bed_entry_gentle', 'bed_entry_deep', 'bed_entry_compact'),
    },
    'crouch': {
        'down': ('crouch_down_mocap', 'crouch_down_calm', 'crouch_down_brisk'),
        'up': ('crouch_up_mocap', 'crouch_up_calm'),
    },
    'kneel': {
        'down': ('kneel_down_mocap', 'kneel_down_calm'),
    },
    'turn': {
        'turn': ('turn_left_stock', 'turn_left_calm', 'turn_left_brisk'),
    },
}


def catalog(task=None):
    """Return `Engram` records filtered to `task` (or all when `task` is None)."""
    if task is None:
        return dict(CATALOG)
    return {k: v for k, v in CATALOG.items() if v.task == task}


def suites():
    """Return prepared combinatorial engram candidate lists by task and slot."""
    return {task: {slot: tuple(items) for slot, items in slots.items()} for task, slots in SUITES.items()}
