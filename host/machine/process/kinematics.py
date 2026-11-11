"""Her measured modes, way-frame kinematics and drive commands named for introspection.

    modes = kinematics.modes(y, fr, a, out)       # z, zd, ref, rate, acc, err, rate_err, kp, kd, sway, asked
    way = kinematics.way(y, fr, a)                # CoM, capture point, soles, legs and turns in her way's frame
    drv = kinematics.drives(s)                    # driven joint commands, clamp shares, knee and ankle angles

Read beside `introspect.snapshot` (`snap['modes']`, `snap['kinematics']`, `snap['drives']`) and reconstructed
from `gate.Scope`'s vector streams in `tools/sim/recording.py` without touching her control loop.
"""
import math
from typing import Any


def _by_mode(vec: Any) -> dict[str, float]:
    """`vec` keyed by `ortho.MODES`."""
    from machine.texts import ortho
    return {m: float(vec[i]) for i, m in enumerate(ortho.MODES)}


def modes(y: Any, fr: Any, a: Any, out: Any) -> dict[str, Any]:
    """Her modes as she is (`modal.reading`) beside what `latent.step` asked of them (`out`), by mode name."""
    import numpy as np
    from machine.law import latent, modal
    y_a, fr_a, a_a, out_a = (np.asarray(v, float).ravel() for v in (y, fr, a, out))
    z, zd, _legs, _turn, _spun = modal.reading(y_a, fr_a, a_a[modal.STANCE])
    ref, rate, acc = out_a[latent.REF], out_a[latent.RATE], out_a[latent.ACC]
    err, rate_err = ref - z, rate - zd
    worst = int(np.argmax(np.abs(err)))
    from machine.texts import ortho
    return {'z': _by_mode(z), 'zd': _by_mode(zd), 'ref': _by_mode(ref), 'rate': _by_mode(rate),
            'acc': _by_mode(acc), 'err': _by_mode(err), 'rate_err': _by_mode(rate_err),
            'kp': _by_mode(out_a[latent.KS]), 'kd': _by_mode(out_a[latent.DS]),
            'sway': _by_mode(out_a[latent.SWAY]), 'psi': _by_mode(out_a[latent.PSI]),
            'asked': _by_mode(out_a[latent.ASKED]),
            'worst_mode': ortho.MODES[worst], 'worst_err': float(err[worst])}


def _sole_entry(i: int, place: Any, vsole: Any, y: Any, fr: Any, a: Any, floors: Any, c_way: Any) -> dict[str, Any]:
    """One sole's spatial state in her way's frame (`i`: 0 left, 1 right)."""
    from machine.law import modal
    meet = a[modal.MEET].reshape(2, 2)[i] - c_way[:2]
    aim = a[modal.AIM].reshape(2, 2)[i] - c_way[:2]
    return {'place_way_m': place[i].tolist(), 'vel_way_m_s': vsole[i].tolist(),
            'clearance_m': float(y[modal.LOWEST][i] - floors[i]), 'load_n': float(y[modal.LOAD][i]),
            'grip': float(y[modal.GRIP][i]), 'stance': float(a[modal.STANCE][i]),
            'off': float(a[modal.SWINGS][i]), 'strode': float(a[modal.STRODE][i]),
            'tilt_deg': math.degrees(float(y[modal.TILT][i])),
            'tipped_deg': math.degrees(float(a[modal.TIPPED][i])),
            'heeled_deg': math.degrees(float(a[modal.HEELED][i])),
            'edge': float(a[modal.EDGE][i]), 'planted_deg': math.degrees(float(fr[modal.PLANTED][i])),
            'meet_way_m': meet.tolist(), 'aim_way_m': aim.tolist()}


def way(y: Any, fr: Any, a: Any) -> dict[str, Any]:
    """Her CoM, capture point, angular momentum, pelvis/trunk turns, soles and legs in her way's frame."""
    import numpy as np
    from machine.law import modal
    from machine.process import introspect
    y_a, fr_a, a_a = (np.asarray(v, float).ravel() for v in (y, fr, a))
    rot = fr_a[modal.ROT].reshape(3, 3)
    stance = a_a[modal.STANCE]
    _z, _zd, legs, turn, spun = modal.reading(y_a, fr_a, stance)
    floors, rise = modal.under(y_a, fr_a, stance)
    c_way = y_a[modal.COM] @ rot.T
    v_way = y_a[modal.VCOM] @ rot.T
    om = max(float(fr_a[modal.OMEGA]), 1e-9)
    place = (y_a[modal.SOLE].reshape(2, 3) - y_a[modal.COM]) @ rot.T
    vsole = y_a[modal.VSOLE].reshape(2, 3) @ rot.T
    hips_up = (y_a[modal.HIP].reshape(2, 3) @ rot.T)[:, 2] - floors
    sides = ('left', 'right')
    soles = {sides[i]: _sole_entry(i, place, vsole, y_a, fr_a, a_a, floors, c_way) for i in range(2)}
    leg_d = {sides[i]: {'hip_from_ankle_way_m': legs[0][i].tolist(), 'length_m': float(legs[1][i]),
                        'length_rate_m_s': float(legs[2][i]), 'hip_over_floor_m': float(hips_up[i]),
                        'dip_m': float(a_a[modal.DIP][i]),
                        'prefold_deg': math.degrees(float(a_a[modal.PREFOLD][i]))} for i in range(2)}
    sensed_f = introspect.layout('machine.law.modal', 'SENSED')
    frame_f = introspect.layout('machine.law.modal', 'FRAME', 'MAP_K')
    return {'heading_rad': float(fr_a[modal.HEADING]), 'omega': float(fr_a[modal.OMEGA]),
            'floor_m': float(fr_a[modal.FLOOR]), 'floor_rise': float(rise), 'going_m_s': float(a_a[modal.GOING]),
            'com_m': y_a[modal.COM].tolist(), 'com_above_floor_m': float(y_a[modal.COM][1] - fr_a[modal.FLOOR]),
            'v_way_m_s': v_way.tolist(), 'capture_way_m': (v_way[:2] / om).tolist(),
            'momentum_way': (rot @ y_a[modal.MOMENT]).tolist(),
            'pelvis_deg': [math.degrees(float(x)) for x in turn[:, 0]],
            'trunk_deg': [math.degrees(float(x)) for x in turn[:, 1]],
            'pelvis_deg_s': [math.degrees(float(x)) for x in spun[:, 0]],
            'trunk_deg_s': [math.degrees(float(x)) for x in spun[:, 1]],
            'soles': soles, 'legs': leg_d,
            'sensed': introspect._named(y_a, sensed_f, introspect.counted('machine.law.modal', 'SENSED')),
            'frame': introspect._named(fr_a, frame_f, introspect.counted('machine.law.modal', 'FRAME', 'MAP_K'))}


def drives(s: dict[str, Any]) -> dict[str, Any]:
    """Her driven joints' commands (`g['cmd']`), clamp shares, and bilateral knee and ankle angles (deg)."""
    import numpy as np
    from machine.body.figure import JOINTS
    g = s['g']
    names = [JOINTS[i] for i in g['joints']]
    cmd = {k: dict(zip(names, np.asarray(v, float).ravel().tolist())) for k, v in (g.get('cmd') or {}).items()}
    tau = np.asarray((g.get('cmd') or {}).get('tau', ()), float).ravel()
    clamp = np.asarray(getattr(g.get('body'), 'clamp', ()), float).ravel()
    share = (dict(zip(names, (np.abs(tau) / np.maximum(clamp, 1e-9)).tolist()))
             if len(tau) == len(names) and len(clamp) == len(names) else {})
    d, m = g['world'].data, g['world'].model
    qdeg = lambda jn: math.degrees(float(d.qpos[int(m.jnt_qposadr[m.joint(jn).id])]))
    return {'joints': names, 'cmd': cmd, 'clamp_share': share,
            'knees_deg': [qdeg('left_knee'), qdeg('right_knee')],
            'ankles_deg': [qdeg('left_ankle'), qdeg('right_ankle')]}


def unpack_frame(raw: Any, fields: dict[str, Any] | None = None) -> Any:
    """Expand a packed `'frame'` stream sample (`streams._frame`) into a full `modal.FRAME` vector."""
    import numpy as np
    from machine.law import modal
    arr = np.asarray(raw, float).ravel()
    if len(arr) >= modal.FRAME or not fields:
        return arr
    full = np.zeros(modal.FRAME)
    for fname, meta in fields.items():
        sp = getattr(modal, fname, None)
        if isinstance(sp, slice):
            full[sp] = arr[meta['at']:meta['at'] + meta['n']]
        elif isinstance(sp, int):
            full[sp] = arr[meta['at']]
    return full


def named(s: dict[str, Any]) -> dict[str, Any]:
    """`{'modes', 'kinematics', 'drives'}` for a live session `s` (`introspect._layers`)."""
    g = s['g']
    return {'modes': modes(g['y'], g['fr'], g['a'], g['out']),
            'kinematics': way(g['y'], g['fr'], g['a']),
            'drives': drives(s)}
