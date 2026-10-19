"""Her law's pass: what her sensors read, through her latent loop, to what her drives are told.

The world is never seen: her array is `machine.edge`'s.

    law.control(g, dt)       # a pass of the loop, every `g['every']`: her ask solved, her legs' stiffness
    law.drives(g, dt)        # every pass: each drive's setpoint, feed and clamp, `g['cmd']`

`g` is `sprung.laid`'s; its `sense` the edge's array, its `est` her estimate (`machine.estimate`).
"""
import ctypes
import functools
import math
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine import borne, edge, estimate, flight, grip, latent, modal, poise, se2, situation, stride, touch, wbc


def seen(g, dt):
    """`wbc.sense`'s view of her as her sensors and her estimate have her, `dt` s on: its C bit for
    bit once built (`g['sensed']`, `laid`)."""
    body, limbs = g['body'], g['limbs']
    qpos, qvel = estimate.step(g['est'], body, g['sense'], dt)
    s = g['sensed'](body, qpos, qvel)
    s['hips'], s['arms'] = body.d.xpos[limbs['hips']].copy(), s['q'][limbs['shoulders']]
    s['prop'], s['contacts'] = borne.read(body, s, g['sense'], float(g['fr'][modal.WEIGHT]))
    return s


def held_back(g, dt):
    """Rows that halt her rhythm - a stand, a hop's load - wait a pass while her capture point
    steps her or a swing is under way (`modal.NEED`, `modal.SWINGS`), every row after them, her
    labels and a hop's end with them: stepped through a hop's load, she left the floor rolling
    and came down at 60 deg. Held longer than two of her rhythm's swings (`latent.TURNS`) they
    come: the swing waited for is not coming - the brake's sway kept one commanded on two
    standing soles for ever, the stand that would end it never came, a sit asked held with it,
    and told to stop on ice she shuffled on 38 cm at 0.045 m/s. Each hold counts its own: a
    hop said 2.1 s after a stop, still stepping, loaded on one sole at once on the stand's. Her
    rhythm at rest, no swing comes: sitting, her centre of mass behind her heels, a seat asked
    was held for ever."""
    tab, t, a = g['tab'], g['t'], g['a']
    if not len(tab) or (a[modal.NEED] <= 0.0 and a[modal.SWINGS].max() <= 0.0):
        g['_held_since'] = None
        return
    at = tab[:, latent.AT]
    halts = (tab[:, latent.SAID][:, latent.GAIN] > 0.5) & (tab[:, latent.VALS][:, latent.GAIN] < 0.05)
    due = halts & (at <= t + latent.EDGE_S) & (at > t - dt + latent.EDGE_S)
    if not due.any():
        g['_held_since'] = None
        return
    since = g.get('_held_since')
    if since is None:
        since = g['_held_since'] = t
    turns = float(g['out'][latent.TURNS])
    if turns <= stride.TINY or t - since > 4.0 * stride.PHI_W / turns:
        return
    first = float(at[due].min()) - latent.EDGE_S
    at[at >= first] += dt
    g['labels'] = [(w + dt if w >= first else w, n) for w, n in g['labels']]
    if g.get('_jump_until', 0.0) > t:
        g['_jump_until'] += dt


def gripped(out, held, stance, caps, into):
    """`out` in the situations she is in (`into` (S,), `situation.read`): each mode's sway and its
    sine scaled down to each's caps (S, N; `situation.text`) as far as she is in it; her drive's
    further to her going no faster than sqrt(2 STRIDE_M grip g) on her standing soles' grip
    (`held`) - a stride she can stop within: 0.77 m/s on ice (0.1), 1.9 at her law's own 0.6.
    Asked 0.69 on ice at 0.02 she ran on and fell at 3.0-3.4 s."""
    grip_now = float(((stance + 1e-9) / (stance.sum() + 2e-9)) @ held)
    sway = np.abs(out[latent.SWAY])
    under = into[:, None]
    capped = np.where(np.isnan(caps), 1.0, np.minimum(1.0, np.nan_to_num(caps) / np.maximum(sway, 1e-9)))
    k = np.min(1.0 - under + under * capped, axis=0, initial=1.0)
    turns = float(out[latent.TURNS])
    cruise = 2.0 / math.pi * stride.STRIDE_M * float(sway[modal.DRIVE] * k[modal.DRIVE]) * math.sin(stride.PHI_W) * turns
    k[modal.DRIVE] *= min(1.0, math.sqrt(2.0 * stride.STRIDE_M * grip_now * modal.G) / max(cruise, 1e-9))
    sine = out[latent.SWAY] * np.sin(out[latent.PSI])
    out[latent.REF] += (k - 1.0) * sine
    out[latent.RATE] += (k - 1.0) * out[latent.SWAY] * turns * np.cos(out[latent.PSI])
    out[latent.ACC] -= (k - 1.0) * sine * turns * turns
    out[latent.SWAY] *= k


def risen_back(g, dt):
    """Rows that raise her - her height asked up - while something bears her off her soles wait a
    pass, every row after them with them, while her capture point is off her soles: thrust up off
    a bed on the clock, her centre of mass 14 cm behind her soles' middle, she rose, her trunk
    came up and she stepped back as it let her go."""
    tab, t, y, fr = g['tab'], g['t'], g['y'], g['fr']
    if y[modal.PROP] > 0.0:
        g['_prop_until'] = t + (0.06 if g.get('active_entry') in ('rise_bed', 'rise_floor') else 0.0)
    if not len(tab) or (y[modal.PROP] <= 0.0 and t > g.get('_prop_until', -1.0)):
        return
    rot = fr[modal.ROT].reshape(3, 3)
    c, v = rot @ y[modal.COM], rot @ y[modal.VCOM]
    xi = c[:2] + v[:2] / math.sqrt(modal.G / max(float(c[2] - fr[modal.FLOOR]), 0.3))
    lo, hi = poise.boxes((y[modal.SOLE].reshape(2, 3) @ rot.T)[:, :2], g['a'][modal.EDGE], y[modal.TILT])
    if (xi >= lo.min(axis=0)).all() and (xi <= hi.max(axis=0)).all():
        return
    at, axial = tab[:, latent.AT], modal.AXIAL
    rises = ((tab[:, latent.SAID][:, latent.Z][:, axial] > 0.5)
             & (tab[:, latent.VALS][:, latent.Z][:, axial] < g['st'][latent.TO][latent.Z][axial] - 1e-6))
    due = rises & (at <= t + latent.EDGE_S) & (at > t - dt + latent.EDGE_S)
    if not due.any():
        return
    first = float(at[due].min()) - latent.EDGE_S
    at[at >= first] += dt
    g['labels'] = [(w + dt if w >= first else w, n) for w, n in g['labels']]


def _step_latent(g, dt):
    """Her latent state stepped, `latent.step` in C bit for bit (`g['latent']`, `laid`)."""
    return g['latent'](g['st'], g['tab'], g['t'], dt, g['out'])


def _step_modal(g, s, y, y_in, out):
    """Her upper body's yield and the loop's ask: `support.couple` and `impedance.ask` in C bit
    for bit (`g['couple']`, `g['impedance']`, `laid`)."""
    qd = np.ascontiguousarray(y_in[edge.QD][g['joints']], dtype=np.float64)
    dq = g['couple'](np.ascontiguousarray(s['q'], dtype=np.float64), qd, g['ref_q'], g['ref_v'], g['eso'][2], out)
    return dq, g['impedance'](out, y, g['fr'], g['a'])


def control(g, dt):
    """A pass of her latent loop on what she senses: `g['got']` solved, `g['leg_kp']` her legs'
    stiffness by their stance."""
    body, ev, y_in = g['body'], g['every'], g['sense']
    g['t_law'] = g['t']
    s = g['s'] = seen(g, dt * ev)
    g['grip'] = grip.felt(g['grip'], g['grip_seen'], body.d.xmat[g['grip_at']].reshape(2, 2, 3, 3),
                          y_in[edge.TOUCH].reshape(-1, 3),
                          g['est']['slip'], float(g['fr'][modal.WEIGHT]), dt * ev, g['grip_prior'],
                          (y_in[edge.LOAD] >= flight.TOUCH_N) & ~np.array(g.get('bore', (True, True))))
    g['loads'] = [float(v) for v in y_in[edge.LOAD]]
    y = modal.sensed(s, g['loads'], g['y'])
    y[modal.GRIP] = g['held'] = grip.outside(g['grip'], g['situations']['grips'])
    g['into'] = situation.read(g['situations'], situation.readings(
        y, g['fr'], g['a'], g['grip'], y_in[edge.TOUCH].reshape(-1, 3), float(g['fr'][modal.WEIGHT])))
    thd, cur_doing = g.get('target_heading'), se2.doing(g)
    if thd is not None:
        g['heading'] += max(-1.6 * dt * ev, min(1.6 * dt * ev, se2.wrap_angle(thd - g['heading'])))
    sh, ch = math.sin(g['heading']), math.cos(g['heading'])
    g['fr'][modal.HEADING], g['fr'][modal.ROT] = g['heading'], np.array([[sh, 0.0, ch], [ch, 0.0, -sh], [0.0, 1.0, 0.0]]).ravel()
    if g.get('_sync_cpg_at') is not None and g['t'] + 1e-9 >= g['_sync_cpg_at']:
        g['_sync_cpg_at'], g['st'][latent.THETA] = None, 0.0
        fut = g['tab'][(g['tab'][:, latent.AT] <= g['t'] + 1e-9) & (g['tab'][:, latent.AT] > g['t'] - dt * ev + 1e-9)]
        if len(fut):
            ph = fut[0, latent.SAID][latent.PHASE] > 0.5
            g['st'][latent.NOW][latent.PHASE] = np.where(ph, fut[0, latent.VALS][latent.PHASE], g['st'][latent.NOW][latent.PHASE])
    held_back(g, dt * ev)
    risen_back(g, dt * ev)
    out = _step_latent(g, dt * ev)
    cw = -1.0 if (math.sin(float(out[latent.PSI][modal.LATERAL]) - float(out[latent.PSI][modal.DRIVE])) * (1.0 if float(out[latent.SWAY][modal.LATERAL]) >= 0.0 else -1.0) < -0.5) else 1.0
    if g.get('_last_way') is not None and cw != g['_last_way'] and (g['st'][latent.NOW][latent.GAIN] > 0.05 or abs(float(out[latent.SWAY][modal.DRIVE])) > 0.02):
        g['st'][latent.THETA] = (float(g['st'][latent.THETA]) + math.pi) % math.tau
        out = _step_latent(g, 0.0)
    g['_last_way'] = cw
    se2.step_physical_twist(g, dt * ev, out)
    gripped(out, g['held'], g['a'][modal.STANCE], g['situations']['caps'], g['into'])
    cur_doing = se2.doing(g)
    touch.check_contacts(g, out, cur_doing)
    ae, tw = g.get('active_entry'), g.get('twist', (0.0, 0.0, 0.0))
    backing = ((cur_doing in se2._BACK_GAITS or ae in se2._BACK_GAITS or tw[0] < -0.05)
               and cur_doing not in ('rest', '#201', 'brake') and cur_doing not in se2.STUNT_PHASES and g.get('_jump_until', 0.0) <= g['t'])
    tl = ((-1.0 if (cur_doing == 'turn_back_right' or ae == 'turn_back_right' or tw[2] < -0.05) else 1.0) if backing
          else (max(-0.35, min(0.35, 0.35 * tw[2] / se2.YAW_RATE)) if abs(tw[2]) > 0.03 and cur_doing not in ('rest', '#201') else 0.0))
    g['_look_yaw'] += min(1.0, (4.5 if backing else 5.0) * dt * ev) * (tl - g.get('_look_yaw', 0.0))
    for idx, k in ((g.get('waist_idx'), 0.38), (g.get('head_idx'), 0.72)):
        if idx is not None:
            g['posture'][idx] = k * g['_look_yaw'] if cur_doing not in se2.STUNT_PHASES else 0.0
    dq, imp = _step_modal(g, s, y, y_in, out)
    if backing and abs(g['_look_yaw']) > 1e-3 and cur_doing not in se2.STUNT_PHASES and g.get('waist_idx') is not None:
        imp[modal.HEADED][1] = g['posture'][g['waist_idx']]
    clear = 0.115 * bool((g.get('_twist_active') or g.get('_twist_stop_until', 0.0) > g['t'])
                         and cur_doing not in se2.STUNT_PHASES and g.get('_jump_until', 0.0) <= g['t'])
    ask = g['ask'] = modal.given(imp, y, g['fr'], g['posture'] + dq, g['bend'], g['arms'], body.knees, clear)
    ask['mu'] = g['held']
    s_solve, cv = s, list(g.get('cervical', ()))
    if (cur_doing in ('load', 'throw', 'land') or ae == 'hop') and ae not in ('hop_up_chair', 'hop_down_chair', 'controlled_fall'):
        cv = cv + list(g['limbs']['shoulders'])
    no_stride = (((g['a'][modal.STRIDES] < 0.5 and float(g['st'][latent.TO][latent.GAIN]) < 0.5
                   and not (len(g['tab']) and (g['tab'][:, latent.VALS][:, latent.GAIN] >= 0.5).any()))
                  or g.get('_jump_until', 0.0) > g['t']) and ae not in ('sit_chair', 'rise_chair', 'hop_up_chair', 'hop_down_chair'))
    if len(cv) and no_stride:
        acv, M = body.act[cv], s['M'].copy()
        M[:6, acv] = M[acv, :6] = 0.0
        s_solve = dict(s, M=M)
    wb = bool(g.get('_leap') or g.get('active_entry') in se2.WASD_GAITS or (g['a'][modal.LEAD] > 0.0).any() or g['a'][modal.OUTSIDE] > 0.0 or cur_doing in se2.STUNT_PHASES)
    der = np.where((g['hold'] > 0.0) | wb, g['wep_lim'] / np.maximum(1.0, body.wep), 1.0)
    g['got'] = g['solve'](body, s_solve, dict(ask, wep=g['left'] > 0.0, derate=der))
    if len(cv) and no_stride:
        cv_p = cv + ([1] if cur_doing == 'land' and g.get('_land_touch_t') is not None and not g.get('_leap') else [])
        acv = body.act[cv_p]
        qdd_cv = np.clip(3.2 * wbc.POSTURE_KD[0] * (ask['posture'][cv_p] - s['q'][cv_p]) - 1.8 * wbc.POSTURE_KD[1] * s['v'][acv], -130.0, 130.0)
        g['got']['qacc'][acv], g['got']['over'][cv_p] = qdd_cv, False
        g['got']['tau'][cv_p] = s['M'][acv, :][:, acv] @ qdd_cv + s['h'][acv]
    g['jeff'] = np.diag(s['M'])[body.act]
    g['leg_kp'] = np.concatenate([g['free_kp'][leg] * (1.0 + (se2.STANDING_KP - 1.0) * float(ask['stance'][k]))
                                  for k, leg in enumerate(g['legs'])])


def laid(g):
    """Her drive-side step laid in C (`wbc_pass.c`, `tools.cores.wbc.Drives`): the struct
    pointing at `g`'s arrays once - the observers, the setpoints and their rates, the holds, the
    knees' spring, the feed and the clamps her drives are told (`g['cmd']`) - and what it reads
    by, kept beside it. Her latent step, her upper body's coupling and her ask in C, her python's
    bit for bit (`tools.cores.law`); her sensing compiled from its numpy (`tools.cores.bodied`)."""
    from tools.cores import bodied, law as c_law, wbc as cores
    g['sensed'] = bodied.on_body(wbc.sense, 'sense')
    g['impedance'] = functools.partial(c_law.asked, *c_law.laid())
    laid_ = c_law.latent_laid()
    g['latent'], g['couple'] = functools.partial(c_law.stepped, laid_), functools.partial(c_law.coupled, laid_)
    body, joints = g['body'], g['joints']
    n = len(joints)
    kept = {'joints': np.ascontiguousarray(joints, dtype=np.int32),
            'act': np.ascontiguousarray(body.act, dtype=np.int32), 'sb': np.zeros(n), 'feed': np.zeros(n),
            'limit': np.zeros(n), 'clamp_j': np.ascontiguousarray(g['clamp'][joints], dtype=np.float64)}
    p = cores.Drives(n=n, q_at=edge.Q.start, qd_at=edge.QD.start, qm_at=edge.QM.start, knee_nm=se2.KNEE_NM,
                     knee_soft=se2.KNEE_SOFT, leak_s=se2.LEAK_S, wep_hold_s=se2.WEP_HOLD_S)
    p.knee[0], p.knee[1] = body.knees
    cv = list(g.get('cervical', ()))
    p.cervical[0], p.cervical[1] = (cv[0], cv[1]) if len(cv) >= 2 else (-1, -1)
    for name, arr in (('joints', kept['joints']), ('act', kept['act'])):
        setattr(p, name, cores.pointer(arr, np.int32))
    for name, arr in (('y', g['sense']), ('clamp', body.clamp), ('wep', body.wep), ('wep_lim', g['wep_lim']),
                      ('clamp_j', kept['clamp_j']), ('q_lo', g['q_lo']), ('q_hi', g['q_hi']), ('eso', g['eso']),
                      ('ref_q', g['ref_q']), ('ref_v', g['ref_v']), ('hold', g['hold']), ('sb', kept['sb']),
                      ('feed', kept['feed']), ('limit', kept['limit'])):
        setattr(p, name, cores.pointer(arr))
    g['cmd'] = {'target': g['ref_q'], 'feed': kept['feed'], 'limit': kept['limit']}
    g['drives'] = (p, kept, cores.library().wbh_pass_drives)


def drives(g, dt):
    """Each driven joint's setpoint (`g['ref_q']`), feed and clamp (`g['cmd']`) this pass, its
    observer stepped on its motor's encoder: the step in C (`laid`), told here what of her it
    reads whole - her law's yield as she tilts, a hop, which WEP limit is hers."""
    y, got, p, _kept, step = g['sense'], g['got'], *g['drives']
    held, wo, ct = float(g['hold'].any()), g['wo'], edge.tilt(y)
    ys = max(0.0, min(1.0, (85.0 - ct) / 20.0)) if ct > 65.0 else 1.0
    doing = se2.doing(g)
    in_hop = (g.get('_jump_until', 0.0) > g['t'] or doing in ('load', 'throw', 'land')) and not g.get('_leap')
    wb = bool(g.get('_leap') or g.get('active_entry') in se2.WASD_GAITS or (g['a'][modal.LEAD] > 0.0).any()
              or g['a'][modal.OUTSIDE] > 0.0 or doing in se2.STUNT_PHASES)
    qacc, tau, jeff = (np.ascontiguousarray(a, dtype=np.float64) for a in (got['qacc'], got['tau'], g['jeff']))
    over = np.ascontiguousarray(got['over'], dtype=np.bool_)
    p.qacc, p.tau, p.over, p.jeff = qacc.ctypes.data, tau.ctypes.data, over.ctypes.data, jeff.ctypes.data
    step(ctypes.byref(p), dt, wo, wo ** 3, ys, int(in_hop), int(wb))
    g['left'] = min(se2.WEP_S, max(0.0, g['left'] + dt * (se2.WEP_FILL * (1.0 - held) - held)))
