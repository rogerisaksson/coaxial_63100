"""The robot's pre-warmed move bank and bumpless law/move handover (`joints`, `sprung`, `se2`).

All joint moves, C tables, bound drives and decoded gait tables are allocated once at session init (`laid`)
and retained across resets (`reset`), so activating or hot-swapping between modes requires zero cold imports
or C struct rebuilds and transitions without step discontinuities in damping, ESO state or reference pose.
"""
import importlib
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

from machine.body import figure
from machine.law import latent
from machine.senses import edge

#: Smooth gain crossfade duration between WBC law and joint moves, s.
HANDOVER_S = 0.08


def _slots_table(g, wire, reset):
    """Build and return a cached normalized latent table template (starting at t=0) for `wire`."""
    import numpy as np
    tables = g['bank']['tables']
    key = (wire, bool(reset))
    base = tables.get(key)
    if base is None:
        base = latent.table(wire, 0.0, g['rest'])
        if reset and len(base):
            base[0, latent.VALS] = np.where(base[0, latent.SAID] > 0.5, base[0, latent.VALS], g['rest'])
            base[0, latent.SAID] = 1.0
        tables[key] = base
    return base


def table(g, wire, at, reset):
    """Return a time-shifted copy of the cached latent table for `wire` at `at` s."""
    import numpy as np
    if g.get('bank') is None:
        out = latent.table(wire, at, g['rest'])
        if reset and len(out):
            out[0, latent.VALS] = np.where(out[0, latent.SAID] > 0.5, out[0, latent.VALS], g['rest'])
            out[0, latent.SAID] = 1.0
        return out
    out = _slots_table(g, wire, reset).copy()
    out[:, latent.AT] += at
    return out


def _build_slot(g, world, name, keep=''):
    """Pre-import, compile and bind all persistent buffers and C move tables for `(name, keep)`."""
    import numpy as np
    from machine.process import joints
    kept = {j for grp in str(keep).split() for j in joints.KEPT[grp]}
    mine = [i for i, j in enumerate(figure.JOINTS) if j not in kept]
    mod = importlib.import_module(joints.MOVES[name])
    mv = mod.move(name.replace('_', '-')) if hasattr(mod, 'move') else mod
    j_idx = np.array([i for i in g['joints'] if i in mine], np.int32)
    kp = np.asarray(joints.KP)[j_idx]
    kd = np.asarray(joints.KD)[j_idx]
    feed = np.zeros(len(j_idx))
    target = np.zeros(len(j_idx))
    limit = np.zeros(len(j_idx))
    pose = np.degrees(g['sense'][edge.Q]).copy()
    cmd = pose.copy()
    state = mv.laid(pose, g['t'])
    slot = {'name': name, 'keep': str(keep), 'move': mv, 'when': None, 'at': None,
            'pose': pose, 'cmd': cmd, 'state': state, 'last': g['t'], 'gains': None,
            'mine': mine, 'joints': j_idx, 'kp': kp, 'kd': kd, 'feed': feed,
            'target': target, 'limit': limit,
            'sensed': getattr(mv, 'SENSED', joints.SENSED), 'holds': getattr(mv, 'HOLDS', True),
            'warm': True, 'enter_t0': None}
    slot['drive'] = edge.bound(world, j_idx, target=target, feed=feed, kp=kp, limit=limit)
    c_played = joints._played()
    slot['_played_fn'] = c_played.laid
    slot['pass'] = c_played.laid(g, world, slot)
    return slot


def laid(g):
    """Pre-warm bodily kernel, all `joints.MOVES` slots and all decoded `gym` wire tables on `g`."""
    from machine.process import joints
    from tools.cores import bodied
    if g.get('bodily') is None:
        g['bodily'] = bodied.on_args(joints.bodily, 'bodily', (0, 1, 2, 3, 4, 5))
    g['bank'] = {'moves': {}, 'tables': {}, 'handover': None}
    world = g['world']
    for name in joints.MOVES:
        g['bank']['moves'][(name, '')] = _build_slot(g, world, name, '')
    for _lbl, wire, _names in g.get('gym', ()):
        _slots_table(g, wire, False)
        _slots_table(g, wire, True)


def reset(g):
    """Reset active handover state and sync pre-warmed move slots cleanly after `relaid`."""
    import numpy as np
    bank = g.get('bank')
    if bank is None:
        return
    bank['handover'] = None
    pose_deg = np.degrees(g['sense'][edge.Q])
    for slot in bank['moves'].values():
        slot['pose'][:] = pose_deg
        slot['cmd'][:] = pose_deg
        slot['at'] = None
        slot['gains'] = None
        slot['when'] = None
        slot['enter_t0'] = None


def activate(g, command):
    """Activate pre-warmed move `command['joints']` onto `g['played']` with C0 pose continuity."""
    import numpy as np
    bank = g.get('bank')
    if bank is None:
        laid(g)
        bank = g['bank']
    name, keep = command['joints'], str(command.get('keep', ''))
    key = (name, keep)
    slot = bank['moves'].get(key)
    if slot is None:
        slot = bank['moves'][key] = _build_slot(g, g['world'], name, keep)
    prev = g.get('played')
    start = np.asarray(prev['cmd'], float).copy() if prev is not None and 'cmd' in prev else np.degrees(g['sense'][edge.Q]).copy()
    saved_gains = prev.get('gains') if prev is not None else None
    slot['when'] = command.get('when')
    slot['at'] = None
    slot['last'] = g['t']
    slot['pose'][:] = start
    slot['cmd'][:] = start
    slot['gains'] = saved_gains
    slot['enter_t0'] = None
    bank['handover'] = None
    g['played'] = slot


def begin_move(g, world, p):
    """Start `p` cleanly from its live or hot-swapped pose without destroying pre-warmed buffers."""
    import numpy as np
    from machine.process import joints
    p['at'] = p['last'] = p['enter_t0'] = g['t']
    if p['gains'] is None:
        p['gains'] = world.gains.copy()
        p['pose'][:] = np.degrees(g['sense'][edge.Q])
    p['state'][:] = p['move'].laid(p['pose'], g['t'])
    p['cmd'][:] = p['pose']
    c_played = joints._played()
    if c_played.laid is not p.get('_played_fn'):
        p['_played_fn'] = c_played.laid
        p['pass'] = c_played.laid(g, world, p)


def ramp_entry_gains(g, world, p):
    """Smoothly crossfade derivative damping toward move KD over `HANDOVER_S` on move entry."""
    gains = p.get('gains')
    t0 = p.get('enter_t0')
    if gains is None or t0 is None:
        return
    a = min(1.0, max(0.0, (g['t'] - t0) / HANDOVER_S))
    j = p['joints']
    world.gains[j, 1] = gains[j, 1] + a * (p['kd'] - gains[j, 1])


def release(g):
    """Hand authority back from active program `g['played']` to the WBC law without velocity/ESO jerk."""
    import numpy as np
    p = g.get('played') or {}
    world = g['world']
    kept = p.get('gains')
    j = g['joints']
    q_now = g['sense'][edge.Q][j]
    qd_now = g['sense'][edge.QD][j]
    if 'ref_q' in g and 'eso' in g:
        g['ref_q'][:] = q_now
        g['ref_v'][:] = qd_now
        g['eso'][0][:] = q_now
        g['eso'][1][:] = qd_now
        g['eso'][2][:] = 0.0
    if p.get('holds') and 'y' in g and 'fr' in g:
        from machine import libm
        from machine.law import modal
        w, x, jj, k = (float(v) for v in g['y'][modal.QUAT][:4])
        if 1.0 - 2.0 * (x * x + k * k) > 0.5:
            yaw = libm.arctan2(2.0 * (w * jj + x * k), 1.0 - 2.0 * (x * x + jj * jj))
            sh, ch = libm.sin(yaw), libm.cos(yaw)
            g['heading'] = g['target_heading'] = yaw
            g['has_heading'] = 0.0
            g['fr'][modal.HEADING], g['fr'][modal.PLANTED] = yaw, 0.0
            g['fr'][modal.ROT] = np.array([sh, 0.0, ch, ch, 0.0, -sh, 0.0, 1.0, 0.0])
    if kept is not None:
        world.gains[:] = kept
    bank = g.get('bank')
    if bank is not None:
        bank['handover'] = None
    g['played'], g['programs'] = None, {}


def blend_exit(g, world):
    """Step active exit gain crossfade after a move returns control to the WBC law."""
    bank = g.get('bank')
    if bank is None or bank['handover'] is None or g.get('played') is not None:
        return
    hv = bank['handover']
    a = min(1.0, max(0.0, (g['t'] - hv['t0']) / HANDOVER_S))
    world.gains[:, 1] = hv['kd_from'] + a * (hv['kd_to'] - hv['kd_from'])
    if a >= 1.0:
        bank['handover'] = None
