"""Her law's pass: what her sensors read, through her latent loop, to what her drives are told.

The world is never seen: her array is `machine.edge`'s.

    law.control(g, dt)       # a pass of the loop, every `g['every']`: her ask solved, her legs' stiffness
    law.drives(g, dt)        # every pass: each drive's setpoint, feed and clamp, `g['cmd']`

`g` is `sprung.laid`'s; its `sense` the edge's array, its `est` her estimate (`machine.estimate`).
"""
import math
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine import edge, estimate, flight, impedance, latent, modal, se2, support, wbc


def seen(g, dt):
    """`wbc.sense`'s view of her as her sensors and her estimate have her, `dt` s on."""
    body, limbs = g['body'], g['limbs']
    qpos, qvel = estimate.step(g['est'], body, g['sense'], dt)
    s = wbc.sense(body, qpos, qvel)
    s['hips'], s['arms'] = body.d.xpos[limbs['hips']].copy(), s['q'][limbs['shoulders']]
    return s


def control(g, dt):
    """A pass of her latent loop on what she senses: `g['got']` solved, `g['leg_kp']` her legs'
    stiffness by their stance."""
    from machine import sprung
    body, ev, y_in = g['body'], g['every'], g['sense']
    s = g['s'] = seen(g, dt * ev)
    g['loads'] = [float(v) for v in y_in[edge.LOAD]]
    y = modal.sensed(s, g['loads'], g['y'])
    pyaw, thd, cur_doing = edge.yaw(y_in), g.get('target_heading'), sprung.doing(g)
    if thd is not None:
        g['heading'] += max(-1.6 * dt * ev, min(1.6 * dt * ev, se2.wrap_angle(thd - g['heading'])))
    elif abs(g.get('twist', (0.0, 0.0, 0.0))[2]) < 0.02:
        g['heading'] += (0.02 if cur_doing == 'brake' else 0.005) * ev * se2.wrap_angle(pyaw - g['heading'])
    sh, ch = math.sin(g['heading']), math.cos(g['heading'])
    g['fr'][modal.HEADING], g['fr'][modal.ROT] = g['heading'], np.array([[sh, 0.0, ch], [ch, 0.0, -sh], [0.0, 1.0, 0.0]]).ravel()
    if g.get('_sync_cpg_at') is not None and g['t'] + 1e-9 >= g['_sync_cpg_at']:
        g['_sync_cpg_at'], g['st'][latent.THETA] = None, 0.0
        fut = g['tab'][(g['tab'][:, latent.AT] <= g['t'] + 1e-9) & (g['tab'][:, latent.AT] > g['t'] - dt * ev + 1e-9)]
        if len(fut):
            ph = fut[0, latent.SAID][latent.PHASE] > 0.5
            g['st'][latent.NOW][latent.PHASE] = np.where(ph, fut[0, latent.VALS][latent.PHASE], g['st'][latent.NOW][latent.PHASE])
    out = latent.step(g['st'], g['tab'], g['t'], dt * ev, g['out'])
    cw = -1.0 if (math.sin(float(out[latent.PSI][modal.LATERAL]) - float(out[latent.PSI][modal.DRIVE])) * (1.0 if float(out[latent.SWAY][modal.LATERAL]) >= 0.0 else -1.0) < -0.5) else 1.0
    if g.get('_last_way') is not None and cw != g['_last_way'] and (g['st'][latent.NOW][latent.GAIN] > 0.05 or abs(float(out[latent.SWAY][modal.DRIVE])) > 0.02):
        g['st'][latent.THETA] = (float(g['st'][latent.THETA]) + math.pi) % math.tau
        out = latent.step(g['st'], g['tab'], g['t'], 0.0, g['out'])
    g['_last_way'] = cw
    se2.step_physical_twist(g, dt * ev, out)
    cur_doing = sprung.doing(g)
    flight.check_contacts(g, out, cur_doing)
    if g['t'] < max(g['stumble_until'], g.get('parry_until', 0.0)):
        se2.boost_stumble(g, out)
    ae, tw = g.get('active_entry'), g.get('twist', (0.0, 0.0, 0.0))
    backing = ((cur_doing in sprung._BACK_GAITS or ae in sprung._BACK_GAITS or tw[0] < -0.05)
               and cur_doing not in ('rest', '#201', 'brake') and cur_doing not in sprung.STUNT_PHASES and g.get('_jump_until', 0.0) <= g['t'])
    tl = ((-1.0 if (cur_doing == 'turn_back_right' or ae == 'turn_back_right' or tw[2] < -0.05) else 1.0) if backing
          else (max(-0.35, min(0.35, 0.35 * tw[2] / se2.YAW_RATE)) if abs(tw[2]) > 0.03 and cur_doing not in ('rest', '#201') else 0.0))
    g['_look_yaw'] += min(1.0, (4.5 if backing else 5.0) * dt * ev) * (tl - g.get('_look_yaw', 0.0))
    for idx, k in ((g.get('waist_idx'), 0.38), (g.get('head_idx'), 0.72)):
        if idx is not None:
            g['posture'][idx] = k * g['_look_yaw'] if cur_doing not in sprung.STUNT_PHASES else 0.0
    imp = impedance.ask(out, y, g['fr'], g['a'])
    if backing and abs(g['_look_yaw']) > 1e-3 and cur_doing not in sprung.STUNT_PHASES and g.get('waist_idx') is not None:
        imp[modal.HEADED][1] = g['posture'][g['waist_idx']]
    ask = g['ask'] = modal.given(imp, y, g['fr'], g['posture'], g['bend'], g['arms'], body.knees)
    support.shape_hop_and_support(g, ask, out, cur_doing)
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
    wb = bool(g.get('_twist_active') or g.get('_leap') or cur_doing in sprung.STUNT_PHASES)
    der = np.where((g['hold'] > 0.0) | wb, g['wep_lim'] / np.maximum(1.0, body.wep), 1.0)
    g['got'] = g['solve'](body, s_solve, dict(ask, wep=g['left'] > 0.0, derate=der))
    if len(cv) and no_stride:
        cv_p = cv + ([1] if cur_doing == 'land' and g.get('_land_touch_t') is not None and not g.get('_leap') else [])
        acv = body.act[cv_p]
        qdd_cv = np.clip(3.2 * wbc.POSTURE_KD[0] * (ask['posture'][cv_p] - s['q'][cv_p]) - 1.8 * wbc.POSTURE_KD[1] * s['v'][acv], -130.0, 130.0)
        g['got']['qacc'][acv], g['got']['over'][cv_p] = qdd_cv, False
        g['got']['tau'][cv_p] = s['M'][acv, :][:, acv] @ qdd_cv + s['h'][acv]
    g['jeff'] = np.diag(s['M'])[body.act]
    g['leg_kp'] = np.concatenate([g['free_kp'][leg] * (1.0 + (sprung.STANDING_KP - 1.0) * float(ask['stance'][k]))
                                  for k, leg in enumerate(g['legs'])])


def drives(g, dt):
    """Each driven joint's setpoint (`g['ref_q']`), feed and clamp (`g['cmd']`) this pass, its
    observer stepped on its motor's encoder."""
    from machine import sprung
    body, y, joints = g['body'], g['sense'], g['joints']
    got, held, eso, wo, ct = g['got'], float(g['hold'].any()), g['eso'], g['wo'], edge.tilt(y)
    ys, sb = (max(0.0, min(1.0, (85.0 - ct) / 20.0)) if ct > 65.0 else 1.0), np.zeros(len(body.act))
    in_hop = (g.get('_jump_until', 0.0) > g['t'] or sprung.doing(g) in ('load', 'throw', 'land')) and not g.get('_leap')
    qpos_act, qvel_act = y[edge.Q][joints], y[edge.QD][joints]
    qk, qvk = qpos_act[list(body.knees)], qvel_act[list(body.knees)]
    sb[list(body.knees)] = (sprung.KNEE_NM * np.clip(1.0 - qk / sprung.KNEE_SOFT, 0.0, 3.0) + 2400.0 * np.maximum(0.0, (0.0 if in_hop else 0.02) - qk)
                            - (0.0 if in_hop else 35.0) * np.clip((0.20 - qk) / 0.15, 0.0, 1.5) * np.minimum(0.0, qvk)) * ys
    qa = got['qacc'][body.act] * ys
    if not in_hop:
        qa = np.clip(qa, -250.0, 250.0)
    q_m = y[edge.QM][joints]
    e = q_m - eso[0]
    eso += dt * np.array([eso[1] + 3.0 * wo * e, qa + sb / g['jeff'] + eso[2] + 3.0 * wo * wo * e, wo ** 3 * e])
    if not in_hop:
        eso[0], z_lim = np.clip(eso[0], q_m - 0.08, q_m + 0.08), 0.35 * body.clamp / np.maximum(0.05, g['jeff'])
        eso[2] = np.clip(eso[2], -z_lim, z_lim)
    if ys < 1.0:
        eso[1:] *= ys
    g['hold'] = np.where(got['over'], sprung.WEP_HOLD_S, np.maximum(0.0, g['hold'] - dt))
    g['left'] = min(sprung.WEP_S, max(0.0, g['left'] + dt * (sprung.WEP_FILL * (1.0 - held) - held)))
    wb = bool(g.get('_twist_active') or g.get('_leap') or sprung.doing(g) in sprung.STUNT_PHASES)
    leak, lo, hi = dt / sprung.LEAK_S, g['q_lo'], g['q_hi']
    g['ref_v'] += qa * dt + leak * (qvel_act - g['ref_v'])
    if not in_hop:
        g['ref_v'] = np.clip(g['ref_v'], -18.0, 18.0)
    g['ref_q'] += g['ref_v'] * dt + leak * (qpos_act - g['ref_q'])
    if in_hop:
        g['ref_q'][list(body.knees)] = np.maximum(0.0, g['ref_q'][list(body.knees)])
        cv = g.get('cervical', ())
        if len(cv) >= 2:
            g['ref_q'][cv[0]], g['ref_q'][cv[1]] = np.clip(g['ref_q'][cv[0]], -0.35, 0.45), np.clip(g['ref_q'][cv[1]], -0.85, 0.85)
    else:
        g['ref_v'] = np.where(((g['ref_q'] <= lo) & (g['ref_v'] < 0.0)) | ((g['ref_q'] >= hi) & (g['ref_v'] > 0.0)), 0.0, g['ref_v'])
        g['ref_q'] = np.clip(g['ref_q'], lo, hi)
    if ys < 1.0:
        g['ref_v'], g['ref_q'] = g['ref_v'] * ys, qpos_act + (g['ref_q'] - qpos_act) * ys
    g['cmd'] = {'target': g['ref_q'], 'feed': got['tau'] * ys + sb - g['jeff'] * eso[2],
                'limit': np.where(g['hold'] > 0.0, g['wep_lim'] if wb else body.wep, g['clamp'][joints])}
