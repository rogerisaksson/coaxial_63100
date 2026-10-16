"""Flight on the whole-body stack: which soles stand, told from this pass alone.

    ask = flight.step(ask, loads, s, hips)        # between a law's ask and `wbc.step`

The stack lays a force on every sole it is asked to stand on; a floor that is not there gives
none. A sole the ask means to stand stands as it bears TOUCH_N; one that bears nothing stands
lying within TOUCH_M of the floor beside one that bears, or with her centre of mass at rest. One
that does not stand is asked to its landing: under her capture point as she comes down, as far
off it as it is off the others, drawn in to what its leg reaches (`reach.land`), its hip
LAND_DROP_M under the leg's length over it, falling as she falls. With no sole standing no turn
is asked, the stack lays no force, and her centre of mass falls: `wbc.step` and the loop in C
need nothing more. A limb is an index, as `reach.LIMBS`; nothing is kept from one pass to the
next.
"""
import math

from machine import reach
from machine.figure import ANKLE_H, BALL, HEEL

G = 9.81

#: A sole bears at TOUCH_N, lies on the floor within TOUCH_M; she rests under REST_M_S.
#: Near alone, the stack pressed a sole 1 mm over the floor onto it as she left: 648 N on it,
#: 23 deg over. All or none while she rises flickered on one sole: 21 deg over three hops.
TOUCH_N, TOUCH_M, REST_M_S = 15.0, 0.002, 0.1

#: A sole in the air waits for the floor with its hip LAND_DROP_M under the leg's length over
#: it - the knee 25 deg -, no straighter than LAND_SHORT_M short of that length where drawn.
LAND_DROP_M, LAND_SHORT_M = 0.018, 0.004

#: The sole's middle ahead of its ankle, m.
MIDDLE = (BALL - HEEL) / 2.0


def standing(meant, loads, over, rising):
    """Which soles stand, a bool a sole: `meant` the ask's stance, `loads` the soles' N, `over`
    each sole's height over the floor, `rising` her centre of mass's speed up."""
    rests = (max(loads) >= TOUCH_N) | (abs(rising) <= REST_M_S)
    return tuple(bool(m) & bool((n >= TOUCH_N) | ((h <= TOUCH_M) & rests))
                 for m, n, h in zip(meant, loads, over))


def landing(limb, hip, heading, yaw, com, vcom, off):
    """(sole's middle, speed, acceleration, quaternion), the stack's swing ask for a sole in the
    air: under her capture point as she lands - from her hip the same whenever that is -, `off`
    it (x, z), drawn in to the leg's reach from `hip`."""
    drop = reach.THIGH + reach.SHANK - LAND_DROP_M
    omega = math.sqrt(G / max(0.3, com[1] - (hip[1] - drop - ANKLE_H)))
    c, s = math.cos(yaw), math.sin(yaw)
    want = ((com[0] + vcom[0] / omega + off[0] - MIDDLE * s, hip[1] - drop,
             com[2] + vcom[2] / omega + off[1] - MIDDLE * c), yaw)
    (ankle, yaw), _m = reach.land(limb, hip, heading, want, LAND_SHORT_M)
    c, s = math.cos(yaw), math.sin(yaw)
    return ((ankle[0] + MIDDLE * s, ankle[1] - ANKLE_H, ankle[2] + MIDDLE * c),
            (vcom[0], vcom[1], vcom[2]), (0.0, -G, 0.0),
            (math.cos(0.5 * yaw), 0.0, math.sin(0.5 * yaw), 0.0))


def step(ask, loads, s, hips, floor=0.0):
    """`ask` as the stack is to be asked it: the soles it means to stand that do not touch asked
    to their landing; with none standing, no turn. `s` is `wbc.sense`'s, `hips` each limb's hip
    joint, world, `floor` the floor's height under her."""
    meant = ask['stance']
    feet = s['feet']
    over = [float(f['pts'][:, 1].min()) - floor for f in feet]
    stance = standing(meant, loads, over, float(s['vcom'][1]))
    count = sum(bool(m) for m in meant)
    mid = [sum(float(f['sole'][i]) * bool(m) for f, m in zip(feet, meant)) / max(1, count) for i in (0, 2)]
    w, x, y, z = (float(v) for v in s['turns'][0]['quat'])
    heading = math.atan2(2.0 * (x * z + w * y), 1.0 - 2.0 * (x * x + y * y))
    swing = dict(ask.get('swing', {}))
    swing.update({k: landing(k, hips[k], heading, math.atan2(feet[k]['R'][0][2], feet[k]['R'][2][2]),
                             s['com'], s['vcom'], (float(feet[k]['sole'][0]) - mid[0],
                                                   float(feet[k]['sole'][2]) - mid[1]))
                  for k in (0, 1) if meant[k] and not stance[k]})
    # with no sole standing there is nothing to turn her against: asked, even as they were,
    # the stack spun her legs to hold her trunk still - the knees out at 3 rad/s with no
    # torque in them, on their stops as she landed
    return dict(ask, stance=stance, swing=swing, turns=ask['turns'][:2 * any(stance)])


def _hop_horiz(is_leap, g, ask, y, modal, mid, s_al, s_lt, c_al, c_lt, py, fl, ay, a_al_d, a_lt_d, p_p, p_r, w_p, w_r, sh, ch):
    if not is_leap:
        return 16.0 * (mid[0] - float(y[modal.COM][0])) - 8.0 * float(y[modal.VCOM][0]), 16.0 * (mid[2] - float(y[modal.COM][2])) - 8.0 * float(y[modal.VCOM][2])
    st_idx = [k for k in (0, 1) if ask['stance'][k]] or [0, 1]
    al_lo, al_hi = min(float(s_al[k]) for k in st_idx) - 0.045, max(float(s_al[k]) for k in st_idx) + 0.055
    lt_lo, lt_hi = min(float(s_lt[k]) for k in st_idx) - 0.040, max(float(s_lt[k]) for k in st_idx) + 0.040
    om2 = max(4.0, (ay + G) / max(0.35, py - fl))
    z_al = max(al_lo, min(al_hi, c_al - a_al_d / om2))
    z_lt = max(lt_lo, min(lt_hi, c_lt - a_lt_d / om2))
    aal, alt = om2 * (c_al - z_al), om2 * (c_lt - z_lt)
    return aal * sh + alt * ch, aal * ch - alt * sh


def shape_hop(g, ask, out, cur_doing, tx, ty, mods):
    """Anatomical two-footed jump (jämfotahopp) and running leap (språng framåt) shaping."""
    import numpy as np
    latent, modal, wrench = mods
    est, y, fl = g['est'], g['y'], float(g['fr'][modal.FLOOR])
    py, vy, y0 = float(est['pos'][1]), float(est['vel'][1]), g['rest_y'] + fl - g.get('floor0', 0.0)
    soles, hd = y[modal.SOLE].reshape(2, 3), float(g['fr'][modal.HEADING])
    mid, sh, ch = 0.5 * (soles[0] + soles[1]), math.sin(hd), math.cos(hd)
    vxp, vyp = float(y[modal.VCOM][0]) * sh + float(y[modal.VCOM][2]) * ch, float(y[modal.VCOM][0]) * ch - float(y[modal.VCOM][2]) * sh
    c_al, c_lt = float(y[modal.COM][0]) * sh + float(y[modal.COM][2]) * ch, float(y[modal.COM][0]) * ch - float(y[modal.COM][2]) * sh
    s_al, s_lt = soles[:, 0] * sh + soles[:, 2] * ch, soles[:, 0] * ch - soles[:, 2] * sh
    mid_lt = 0.5 * float(s_lt[0] + s_lt[1])
    turn, spun = modal._turns(y, g['fr'][modal.ROT].reshape(3, 3))
    p_p, p_r, w_p, w_r = float(turn[0, 0] - g['fr'][modal.PITCH0][0]), float(turn[1, 0]), float(spun[0, 0]), float(spun[1, 0])
    is_leap = bool(g.get('_leap') or abs(tx) >= 0.10 or abs(ty) >= 0.10)
    vf = max(0.42, min(0.85, 0.92 * vxp)) if (is_leap or tx >= 0.05) else (min(-0.30, 0.75 * tx) if tx <= -0.05 else 0.0)
    vl, (sh_l, sh_r) = (0.75 * ty if abs(ty) >= 0.05 else 0.0), g['limbs']['shoulders']
    el_l, el_r, sp_i = g.setdefault('_hop_idx', (wrench._DRIVEN.index('left_elbow'), wrench._DRIVEN.index('right_elbow'), wrench._DRIVEN.index('spine')))
    q_now, v_now = g['s']['q'], g['s']['v'][g['body'].act]
    arm_s = 0.25 if g.get('_upper_hit') else 1.0

    def _upper(pose, sh_d, el_d, sp_d):
        pose[sh_l], pose[sh_r] = sh_d, sh_d
        pose[el_l], pose[el_r] = el_d, el_d
        pose[sp_i] = sp_d

    def _horiz(ay, a_al_d, a_lt_d):
        return _hop_horiz(is_leap, g, ask, y, modal, mid, s_al, s_lt, c_al, c_lt, py, fl, ay, a_al_d, a_lt_d, p_p, p_r, w_p, w_r, sh, ch)

    k_p = float(y[modal.MOMENT][0]) * ch - float(y[modal.MOMENT][2]) * sh

    def _turns(sp_d, kp_t=95.0, k_fb=0.0):
        aim_p = g['fr'][modal.PITCH0] + np.array([-0.20 * sp_d - k_fb * k_p, 0.80 * sp_d - k_fb * k_p])
        want_p = turn[0] + (kp_t * (aim_p - turn[0]) + 22.0 * spun[0]) / 400.0
        want_r = turn[1] + (-80.0 * turn[1] + 22.0 * spun[1]) / 400.0
        return tuple(modal._turned(np.array([hd, hd]), want_p, want_r))

    if is_leap and (cur_doing in ('load', 'throw') or not g.get('_leap_synced')):
        ask['settle'][:] = 0.0
        ask['edge'], ask['strides'] = (0.0, 0.0), 0.0
    lead_k = 0 if float(s_al[0]) >= float(s_al[1]) else 1
    split = is_leap and abs(float(s_al[0] - s_al[1])) > 0.14 and float(g['loads'][lead_k]) > 25.0
    if cur_doing == 'load':
        g['_land_touch_t'], g['_was_air'], g['_hop_until'] = None, False, 0.0
        dt_sw = max(0.0, g['t'] - g.get('_leap_t0', g['t']))
        u_cur = max(0.0, (y0 - py) / 0.210)
        u0 = max(0.30, min(0.65, (y0 - float(g.get('_leap_y0', py))) / 0.210)) if is_leap else 0.0
        u_ld = min(1.0, max(0.0, (u0 + (0.78 - u0) * min(1.0, dt_sw / 0.045)) if is_leap
                            else max(float(out[latent.REF][modal.AXIAL]) / 0.32, (y0 - py) / 0.18)))
        pose = g['posture'] + g['bend'] * (1.15 * min(u_ld, u_cur + 0.18) if is_leap else 1.40 * u_ld)
        st_l = ((lead_k == 0, lead_k == 1) if split
                else tuple(float(g['loads'][k]) > 10.0 or float(y[modal.LOWEST][k]) < 0.015 for k in (0, 1))) if is_leap else (True, True)
        ask['stance'], ask['swing'], ask['fold'] = (st_l if any(st_l) else (True, True)), {}, {}
        if is_leap:
            lead_w = max(0.12, min(0.30, 0.32 * vxp))
            for k, leg in enumerate(wrench.LEGS):
                if not ask['stance'][k]:
                    lag = max(0.0, float(c_al - s_al[k]))
                    q_tk = g['posture'][leg] + g['bend'][leg] * 0.54
                    q_tk[2] -= lead_w + 0.85 * lag
                    q_tk[3] += 1.60 * lag
                    q_tk[4] += lead_w
                    pose[leg] = q_now[leg] + (450.0 * np.clip(q_tk - q_now[leg], -0.28, 0.28) - 18.0 * v_now[leg]) / 100.0
        ay = max(-7.5 if is_leap else -9.8, min(0.5 if is_leap else 16.0, 95.0 * (y0 - (0.210 if is_leap else 0.240) * u_ld - py) - (14.0 if is_leap else 16.0) * vy))
        ask['com_acc'][1] = ay
        ask['com_acc'][0], ask['com_acc'][2] = _horiz(ay, 5.0 * (vf - vxp), 14.0 * (mid_lt - c_lt) + 6.0 * (vl - vyp))
        sp_d = (0.12 if is_leap else 0.34) * u_ld
        _upper(pose, -0.80 * arm_s * u_ld, float(g['posture'][el_l]) + 0.35 * arm_s * u_ld, sp_d)
        ask['posture'] = pose
        ask['turns'] = _turns(sp_d, 112.0, 0.0)
        ask['kdot'] = -14.0 * y[modal.MOMENT]
        return
    st_both = (tuple(float(g['loads'][k]) > 12.0 for k in (0, 1)) if is_leap else (True, True))
    if not any(st_both):
        st_both = (True, True)
    st_t = (((lead_k == 0 and float(g['loads'][0]) > 15.0 and float(q_now[wrench.LEGS[0][3]]) > 0.28,
              lead_k == 1 and float(g['loads'][1]) > 15.0 and float(q_now[wrench.LEGS[1][3]]) > 0.28) if split
             else tuple(float(g['loads'][k]) > 15.0 and float(q_now[wrench.LEGS[k][3]]) > 0.28 for k in (0, 1)))
            if is_leap else (True, True))
    if is_leap and ((not any(st_t) and vy > 0.38) or vy > 0.95):
        g['_was_air'] = True
    cmd_hop, v_cut = g.get('_jump_until', 0.0) > g['t'] or g['every'] > 1, (0.95 if is_leap else 2.185)
    if (not g.get('_was_air') and max(g['loads']) > 10.0 and any(st_t) and py < y0 - (0.004 if is_leap else 0.014)
            and (not cmd_hop or vy < v_cut)):
        g['_land_touch_t'] = None
        ask['stance'], ask['swing'], ask['fold'] = st_t, {}, {}
        ay = 38.0 if is_leap else (41.8 if cmd_hop else 35.0)
        ask['com_acc'][1] = ay
        ask['com_acc'][0], ask['com_acc'][2] = _horiz(ay, 5.0 * (vf - vxp), 14.0 * (mid_lt - c_lt) + 6.0 * (vl - vyp))
        pose = g['posture'].copy()
        if is_leap:
            lead_w = max(0.12, min(0.32, 0.34 * vxp))
            for k, leg in enumerate(wrench.LEGS):
                if ask['stance'][k]:
                    b_st = max(0.26, float(q_now[leg[3]] - g['posture'][leg[3]]) - 0.08)
                    pose[leg] = g['posture'][leg] + g['bend'][leg] * b_st
                else:
                    lag = max(0.0, float(c_al - s_al[k]))
                    q_tk = g['posture'][leg] + g['bend'][leg] * 0.54
                    q_tk[2] -= lead_w + 0.85 * lag
                    q_tk[3] += 1.60 * lag
                    q_tk[4] += lead_w
                    pose[leg] = q_now[leg] + (480.0 * np.clip(q_tk - q_now[leg], -0.32, 0.32) - 18.0 * v_now[leg]) / 100.0
        elif vy > 1.25:
            for leg in wrench.LEGS:
                pose[leg[4]] = 0.38 * min(1.0, (vy - 1.25) / 0.50)
        _upper(pose, 1.38 * arm_s, float(g['posture'][el_l]) + 0.20 * arm_s, 0.0)
        ask['posture'] = pose
        ask['turns'] = _turns(0.0, 35.0 if not is_leap else 125.0, 0.045 if is_leap else 0.0)
        ask['kdot'] = -16.0 * y[modal.MOMENT]
        return
    _shape_hop_air_land(g, ask, y, (latent, modal, wrench), is_leap, cmd_hop, v_cut, py, vy, y0, vxp, vyp, vf, vl,
                        c_al, c_lt, s_al, mid_lt, q_now, v_now, arm_s, el_l, st_both, _upper, _horiz, _turns)


def _shape_hop_air_land(g, ask, y, mods, is_leap, cmd_hop, v_cut, py, vy, y0, vxp, vyp, vf, vl,
                        c_al, c_lt, s_al, mid_lt, q_now, v_now, arm_s, el_l, st_both, _upper, _horiz, _turns):
    import numpy as np
    latent, modal, wrench = mods
    if max(g['loads']) <= 12.0 or (g.get('_land_touch_t') is None and is_leap and g.get('_was_air') and vy > 0.15) or (not g.get('_was_air') and cmd_hop and vy >= v_cut - 0.02):
        g['_land_touch_t'] = None
        u_down = min(1.0, max(0.0, ((0.20 if is_leap else 0.78) - vy) / (0.95 if is_leap else 1.25)))
        b_air = ((0.48 - 0.26 * u_down) if is_leap
                 else (0.12 + 0.86 * min(1.0, max(0.0, (1.05 - vy) / 0.35)) if vy >= -0.05
                       else (0.20 if vy > -1.00 else 0.28)))
        q_tuck = g['posture'] + g['bend'] * b_air
        if is_leap:
            lead_w = max(0.12, min(0.36, 0.38 * vxp))
            for k, leg in enumerate(wrench.LEGS):
                lag = max(0.0, float(c_al - s_al[k]))
                q_tuck[leg[2]] -= lead_w + 0.75 * lag
                q_tuck[leg[3]] += 1.50 * lag
                q_tuck[leg[4]] += lead_w + 0.25 * lag
        sh_air = arm_s * (1.38 * (1.0 - u_down) + (0.35 if is_leap else -0.45) * u_down)
        if (py > y0 - 0.01 or vy < 0.5) and (not is_leap or vy > 0.25 or py > y0):
            g['_was_air'] = True
        ask['stance'], ask['swing'], ask['fold'], ask['turns'] = (False, False), {}, {}, ()
        pose, legs_idx = q_tuck.copy(), wrench.LEGS[0] + wrench.LEGS[1]
        if is_leap:
            pose[legs_idx] = q_now[legs_idx] + (480.0 * np.clip(q_tuck[legs_idx] - q_now[legs_idx], -0.35, 0.35) - 18.0 * v_now[legs_idx]) / 100.0
        else:
            pose[legs_idx] = q_now[legs_idx] + (650.0 * (q_tuck[legs_idx] - q_now[legs_idx]) - 25.0 * v_now[legs_idx]) / 100.0
        if not is_leap:
            u_to = min(1.0, max(0.0, (vy - 0.70) / 0.25))
            u_pre = math.sin(min(1.0, max(0.0, (-0.02 - vy) / 1.08)) * math.pi)
            for leg in wrench.LEGS:
                a_i = leg[4]
                ank_d = float(q_tuck[a_i]) * (1.0 - max(u_to, u_pre)) + 0.46 * u_to + 0.56 * u_pre
                pose[a_i] = q_now[a_i] + 5.8 * (ank_d - q_now[a_i]) - 0.22 * v_now[a_i]
        _upper(pose, sh_air, float(g['posture'][el_l]) + 0.18 * arm_s, 0.0)
        ask['posture'] = pose
        return
    if max(g['loads']) > 12.0:
        t_touch = g.get('_land_touch_t')
        if t_touch is None:
            t_touch = g['_land_touch_t'] = g['t']
        dt_l = max(0.0, g['t'] - t_touch)
        if is_leap and dt_l > 0.10:
            if not g.get('_leap_synced'):
                g['_leap_synced'], g['_jump_until'] = True, g['t']
                l0, l1 = float(g['loads'][0]), float(g['loads'][1])
                g['st'][latent.THETA] = (-1.06 if l1 < l0 else 2.08) if abs(l0 - l1) > 50.0 else (-1.06 if float(s_al[1]) < float(s_al[0]) else 2.08)
                is_jog = g.get('active_entry') == 'jog'
                g['a'][modal.GOING] = min(0.55 if is_jog else 0.45, max(0.38, vxp))
                for part in (latent.FROM, latent.NOW, latent.TO):
                    g['st'][part][latent.GAIN], g['st'][part][latent.HZ], g['st'][part][latent.ZMP] = 1.0, (1.12 if is_jog else 0.96), 2.0
                    g['st'][part][latent.Z][modal.AXIAL] = 0.075 if is_jog else 0.065
                    g['st'][part][latent.K][modal.AXIAL], g['st'][part][latent.D][modal.AXIAL] = (3200.0 if is_jog else 3000.0), 600.0
                    g['st'][part][latent.K][modal.LATERAL], g['st'][part][latent.D][modal.LATERAL] = 3000.0, 300.0
                    g['st'][part][latent.K][modal.SAGITTAL], g['st'][part][latent.D][modal.SAGITTAL] = 0.0, 260.0
                    g['st'][part][latent.K][modal.ROCK], g['st'][part][latent.D][modal.ROCK] = 2400.0, 240.0
                    g['st'][part][latent.AMP][modal.DRIVE] = (0.68 if is_jog else 0.605) if part == latent.TO else (0.42 if is_jog else 0.38)
                    g['st'][part][latent.AMP][modal.LATERAL] = 0.28
                    g['st'][part][latent.AMP][modal.LIFT], g['st'][part][latent.AMP][modal.ROCK] = 1.05, 0.62
                    g['st'][part][latent.AMP][modal.LIST], g['st'][part][latent.AMP][modal.TURN] = 0.35, 0.40
                    g['st'][part][latent.AMP][modal.ARMS] = 0.88
                    g['st'][part][latent.PHASE][modal.LATERAL] = g['st'][part][latent.PHASE][modal.LIST] = 1.885
                g['st'][latent.T0], g['st'][latent.BLEND] = g['t'], 0.38
                g['tab'] = g['tab'][:0]
                g['labels'].append((g['t'], 'run' if is_jog else (g.get('active_entry') or '#204')))
            return
        if is_leap:
            ask['stance'], ask['swing'], ask['fold'] = st_both, {}, {}
        u_abs = math.sin(min(1.0, dt_l / 0.28) * math.pi) if dt_l < 0.28 else 0.0
        ay = max(float(ask['com_acc'][1]), 36.0 * (y0 - (0.08 if is_leap else 0.105) * u_abs - py) - 10.0 * vy)
        ask['com_acc'][1] = ay
        if is_leap or not g.get('_twist_active'):
            ask['com_acc'][0], ask['com_acc'][2] = _horiz(ay, 7.0 * (vf - vxp), 14.0 * (mid_lt - c_lt) + 6.0 * (vl - vyp))
        pose = ask['posture'] + (0.0 if is_leap else 0.26 * u_abs) * g['bend']
        if is_leap:
            lead_w = max(0.12, min(0.32, 0.34 * vxp))
            for k, leg in enumerate(wrench.LEGS):
                if not ask['stance'][k]:
                    lag = max(0.0, float(c_al - s_al[k]))
                    q_tk = g['posture'][leg] + g['bend'][leg] * 0.22
                    q_tk[2] -= lead_w + 0.65 * lag
                    q_tk[3] += 1.20 * lag
                    q_tk[4] += lead_w
                    pose[leg] = q_now[leg] + (420.0 * (q_tk - q_now[leg]) - 18.0 * v_now[leg]) / 100.0
        sp_d = (0.10 if is_leap else 0.24) * u_abs
        sh_lnd = arm_s * ((0.45 if is_leap else 0.56) * u_abs)
        _upper(pose, sh_lnd, 0.0, sp_d)
        ask['posture'] = pose
        ask['turns'] = _turns(0.0, 95.0)
        ask['kdot'] = -12.0 * y[modal.MOMENT]


def stride_hop(g, by, t0, spd, back_now, fwd_gaits, back_gaits, said_fn, walk_on_wire):
    import numpy as np
    from machine import latent, modal
    prev = (g.get('active_entry') if g.get('active_entry') in (fwd_gaits + back_gaits)
            else ('#201' if not g.get('_twist_active') else ('#205' if back_now else '#204')))
    leap = prev != '#201' or spd >= 0.15
    y, hd = g['y'], float(g['heading'])
    sh, ch = math.sin(hd), math.cos(hd)
    tr = 0 if float(y[modal.SOLE][0]) * sh + float(y[modal.SOLE][2]) * ch < float(y[modal.SOLE][3]) * sh + float(y[modal.SOLE][5]) * ch else 1
    g['_leap'], g['_leap_t0'], g['_leap_synced'], g['_leap_tr'], g['_leap_y0'] = leap, t0, False, tr, float(g['est']['pos'][1])
    g['_leap_p0'] = y[modal.SOLE].reshape(2, 3)[tr].copy()
    if leap:
        rows = np.frombuffer(by['hop'][1], latent.WIRE)[:3].copy()
        rows['at'], rows['blend'] = (0, 45, 220), (40, 0, 0)
        rows['p'][:, latent.GAIN] = 0.0
        rows['p'][2, latent.Z.start + modal.AXIAL], rows['p'][2, latent.K.start + modal.AXIAL], rows['p'][2, latent.D.start + modal.AXIAL] = 0.14, 3000.0, 600.0
        said_fn(g, 'hop', rows.tobytes(), by['hop'][2][:3], at=t0, reset=False)
        t_res = t0 + 0.82
    else:
        said_fn(g, *by['hop'], at=t0, reset=True)
        t_res = t0 + 1.45
    g['_jump_until'], g['active_entry'] = t_res, prev
    if prev == '#204':
        said_fn(g, prev, walk_on_wire, (prev,), at=t_res + 0.01)
    elif prev == 'jog' and 'jog' in by:
        rj = np.frombuffer(by['jog'][1], latent.WIRE)[:1].copy()
        r1 = np.frombuffer(by['jog'][1], latent.WIRE)[1]
        rj['p'][0, ~np.isnan(r1['p'])] = r1['p'][~np.isnan(r1['p'])]
        rj['at'], rj['blend'] = (0,), (160,)
        said_fn(g, 'jog', rj.tobytes(), ('run',), at=t_res + 0.01)
    elif prev in by:
        said_fn(g, *by[prev], at=t_res + 0.01, reset=(prev == '#201'))
