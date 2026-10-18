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

from machine import borne, edge, estimate, flight, grip, impedance, latent, modal, poise, se2, situation, stride, support, touch, wbc


def seen(g, dt):
    """`wbc.sense`'s view of her as her sensors and her estimate have her, `dt` s on."""
    body, limbs = g['body'], g['limbs']
    qpos, qvel = estimate.step(g['est'], body, g['sense'], dt)
    s = wbc.sense(body, qpos, qvel)
    s['hips'], s['arms'] = body.d.xpos[limbs['hips']].copy(), s['q'][limbs['shoulders']]
    s['prop'], s['contacts'] = borne.read(body, s, g['sense'], float(g['fr'][modal.WEIGHT]))
    return s


def held_back(g, dt):
    """Rows that halt her rhythm - a stand, a hop's load - wait a pass while her capture point
    steps her or a swing is under way (`modal.NEED`, `modal.SWINGS`), every row after them, her
    labels and a hop's end with them: stepped through a hop's load, she left the floor rolling
    and came down at 60 deg."""
    tab, t, a = g['tab'], g['t'], g['a']
    if not len(tab) or (a[modal.NEED] <= 0.0 and a[modal.SWINGS].max() <= 0.0):
        return
    at = tab[:, latent.AT]
    halts = (tab[:, latent.SAID][:, latent.GAIN] > 0.5) & (tab[:, latent.VALS][:, latent.GAIN] < 0.05)
    due = halts & (at <= t + latent.EDGE_S) & (at > t - dt + latent.EDGE_S)
    if not due.any():
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
    if not len(tab) or y[modal.PROP] <= 0.0:
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


def control(g, dt):
    """A pass of her latent loop on what she senses: `g['got']` solved, `g['leg_kp']` her legs'
    stiffness by their stance."""
    from machine import sprung
    body, ev, y_in = g['body'], g['every'], g['sense']
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
    thd, cur_doing = g.get('target_heading'), sprung.doing(g)
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
    out = latent.step(g['st'], g['tab'], g['t'], dt * ev, g['out'])
    cw = -1.0 if (math.sin(float(out[latent.PSI][modal.LATERAL]) - float(out[latent.PSI][modal.DRIVE])) * (1.0 if float(out[latent.SWAY][modal.LATERAL]) >= 0.0 else -1.0) < -0.5) else 1.0
    if g.get('_last_way') is not None and cw != g['_last_way'] and (g['st'][latent.NOW][latent.GAIN] > 0.05 or abs(float(out[latent.SWAY][modal.DRIVE])) > 0.02):
        g['st'][latent.THETA] = (float(g['st'][latent.THETA]) + math.pi) % math.tau
        out = latent.step(g['st'], g['tab'], g['t'], 0.0, g['out'])
    g['_last_way'] = cw
    se2.step_physical_twist(g, dt * ev, out)
    gripped(out, g['held'], g['a'][modal.STANCE], g['situations']['caps'], g['into'])
    cur_doing = sprung.doing(g)
    touch.check_contacts(g, out, cur_doing)
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
    ask['mu'] = g['held']
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
    wb = bool(g.get('_leap') or g.get('active_entry') in se2.WASD_GAITS or (g['a'][modal.LEAD] > 0.0).any() or g['a'][modal.OUTSIDE] > 0.0 or cur_doing in sprung.STUNT_PHASES)
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
    wb = bool(g.get('_leap') or g.get('active_entry') in se2.WASD_GAITS or (g['a'][modal.LEAD] > 0.0).any() or g['a'][modal.OUTSIDE] > 0.0 or sprung.doing(g) in sprung.STUNT_PHASES)
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
