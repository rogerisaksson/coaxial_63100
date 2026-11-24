#!/usr/bin/env python3
"""World-truth chair sitting invariant test: deep seat, hands on thighs, zero foot load, no spasms."""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen
from gynoid_kit import Report

#: Hold window after joint playback finishes, s; max rise time, s.
HOLD_S, RISE_S = 5.0, 12.0
#: Anti-cheat bounds enforced across the closed-loop seated hold.
BOUNDS = {
    'pel_front_min_m': 0.18,
    'pel_lat_max_m': 0.06,
    'pel_drift_max_m': 0.025,
    'soles_max_n': 1.0,
    'seat_up_min_frac': 0.90,
    'hands_chair_max_n': 1.0,
    'hand_thigh_frac': 0.90,
    'hand_gap_max_m': 0.008,
    'hand_dy_min_m': 0.03,
    'hand_t_lo': 0.18,
    'hand_t_hi': 0.78,
    'chair_tilt_max_deg': 5.0,
    'chair_drift_max_m': 0.08,
    'trunk_pitch_max_deg': 12.0,
    'pelvis_roll_max_deg': 5.0,
    'qvel_hold_max': 1.5,
    'scale_min': 0.50,
    'rise_chair_tilt_max_deg': 8.0,
    'rise_soles_min_frac': 0.85,
}


def judge_seated(metrics):
    """Return `{'ok': bool, 'reasons': [str]}` for a completed seated trial dict."""
    reasons = []
    if metrics.get('fallen_at') is not None or metrics.get('epoch', 0) != 0:
        reasons.append('fell_at=%s epoch=%s' % (metrics.get('fallen_at'), metrics.get('epoch')))
    if not metrics.get('hold_reached', False):
        reasons.append('never reached closed-loop seated hold')
    if metrics.get('played_during_hold', True):
        reasons.append('joint program played during closed-loop hold')
    if metrics.get('pel_front_min_m', -1.0) < BOUNDS['pel_front_min_m']:
        reasons.append('pel_front_min_m=%.3f < %.3f' % (
            metrics.get('pel_front_min_m', -1.0), BOUNDS['pel_front_min_m']))
    if metrics.get('pel_lat_max_m', 9.0) > BOUNDS['pel_lat_max_m']:
        reasons.append('pel_lat_max_m=%.3f > %.3f' % (
            metrics.get('pel_lat_max_m', 9.0), BOUNDS['pel_lat_max_m']))
    if metrics.get('pel_drift_max_m', 9.0) > BOUNDS['pel_drift_max_m']:
        reasons.append('pel_drift_max_m=%.3f > %.3f' % (
            metrics.get('pel_drift_max_m', 9.0), BOUNDS['pel_drift_max_m']))
    if metrics.get('soles_max_n', 999.0) >= BOUNDS['soles_max_n']:
        reasons.append('soles_max_n=%.2f >= %.2f' % (
            metrics.get('soles_max_n', 999.0), BOUNDS['soles_max_n']))
    if metrics.get('seat_up_min_frac', 0.0) < BOUNDS['seat_up_min_frac']:
        reasons.append('seat_up_min_frac=%.3f < %.3f' % (
            metrics.get('seat_up_min_frac', 0.0), BOUNDS['seat_up_min_frac']))
    if metrics.get('hands_chair_max_n', 999.0) >= BOUNDS['hands_chair_max_n']:
        reasons.append('hands_chair_max_n=%.2f >= %.2f' % (
            metrics.get('hands_chair_max_n', 999.0), BOUNDS['hands_chair_max_n']))
    for side in ('left', 'right'):
        frac = metrics.get(side + '_contact_frac', 0.0)
        gap = metrics.get(side + '_gap_max_m', 9.0)
        dy = metrics.get(side + '_dy_min_m', -9.0)
        t_lo = metrics.get(side + '_t_min', -1.0)
        t_hi = metrics.get(side + '_t_max', 2.0)
        if frac < BOUNDS['hand_thigh_frac'] and gap > BOUNDS['hand_gap_max_m']:
            reasons.append('%s hand off thigh (frac=%.2f, gap=%.4f)' % (side, frac, gap))
        if dy < BOUNDS['hand_dy_min_m']:
            reasons.append('%s hand dy=%.3f < %.3f' % (side, dy, BOUNDS['hand_dy_min_m']))
        if t_lo < BOUNDS['hand_t_lo'] or t_hi > BOUNDS['hand_t_hi']:
            reasons.append('%s hand t=[%.2f, %.2f] outside [%.2f, %.2f]' % (
                side, t_lo, t_hi, BOUNDS['hand_t_lo'], BOUNDS['hand_t_hi']))
    if metrics.get('chair_tilt_max_deg', 90.0) >= BOUNDS['chair_tilt_max_deg']:
        reasons.append('chair_tilt_max_deg=%.2f >= %.2f' % (
            metrics.get('chair_tilt_max_deg', 90.0), BOUNDS['chair_tilt_max_deg']))
    if metrics.get('chair_drift_max_m', 9.0) >= BOUNDS['chair_drift_max_m']:
        reasons.append('chair_drift_max_m=%.3f >= %.3f' % (
            metrics.get('chair_drift_max_m', 9.0), BOUNDS['chair_drift_max_m']))
    if metrics.get('trunk_pitch_max_deg', 90.0) >= BOUNDS['trunk_pitch_max_deg']:
        reasons.append('trunk_pitch_max_deg=%.2f >= %.2f' % (
            metrics.get('trunk_pitch_max_deg', 90.0), BOUNDS['trunk_pitch_max_deg']))
    if metrics.get('pelvis_roll_max_deg', 90.0) >= BOUNDS['pelvis_roll_max_deg']:
        reasons.append('pelvis_roll_max_deg=%.2f >= %.2f' % (
            metrics.get('pelvis_roll_max_deg', 90.0), BOUNDS['pelvis_roll_max_deg']))
    if metrics.get('qvel_hold_max', 99.0) >= BOUNDS['qvel_hold_max']:
        reasons.append('qvel_hold_max=%.2f >= %.2f' % (
            metrics.get('qvel_hold_max', 99.0), BOUNDS['qvel_hold_max']))
    if metrics.get('scale_min', 0.0) <= BOUNDS['scale_min']:
        reasons.append('scale_min=%.3f <= %.3f' % (
            metrics.get('scale_min', 0.0), BOUNDS['scale_min']))
    if not metrics.get('rose_ok', False):
        reasons.append('rise_chair failed (stood=%s soles=%.2f tilt=%.2f)' % (
            metrics.get('rise_stood'), metrics.get('rise_soles_frac', 0.0),
            metrics.get('rise_chair_tilt_deg', 90.0)))
    return {'ok': not reasons, 'reasons': reasons}


def _hand_thigh_stats(m, d, figure):
    """Per-side contact presence, capsule surface gap, dy above femur, and longitudinal femur fraction."""
    import numpy as np
    out = {}
    for side in ('left', 'right'):
        hb = m.body(side + '_hand').id
        fb = m.body(side + '_fingers').id
        tb = m.body(side + '_thigh').id
        sb = m.body(side + '_shank').id
        hit = any((m.geom_bodyid[c.geom1] in (hb, fb) and m.geom_bodyid[c.geom2] == tb)
                  or (m.geom_bodyid[c.geom2] in (hb, fb) and m.geom_bodyid[c.geom1] == tb)
                  for c in d.contact[:d.ncon])
        hp, tp, kp = d.xpos[hb], d.xpos[tb], d.xpos[sb]
        u = kp - tp
        t_al = float(np.clip((hp - tp) @ u / max(float(u @ u), 1e-12), 0.0, 1.0))
        cl = tp + t_al * u
        dy = float(hp[1] - cl[1])
        # Shortest capsule-to-capsule gap between hand/fingers and thigh.
        min_gap = 1e9
        h_caps, t_caps = [], []
        sx = 1.0 if side == 'left' else -1.0
        for seg, b in ((side + '_hand', hb), (side + '_fingers', fb)):
            p0, rmat = d.xpos[b], d.xmat[b].reshape(3, 3)
            base = seg.split('_', 1)[1]
            for sname, rad, a, bpt in figure.BODY:
                if sname == base:
                    h_caps.append((rad, p0 + rmat @ np.array((sx * a[0], a[1], a[2])),
                                   p0 + rmat @ np.array((sx * bpt[0], bpt[1], bpt[2]))))
        p_t, r_t = d.xpos[tb], d.xmat[tb].reshape(3, 3)
        for sname, rad, a, bpt in figure.BODY:
            if sname == 'thigh':
                t_caps.append((rad, p_t + r_t @ np.array((sx * a[0], a[1], a[2])),
                               p_t + r_t @ np.array((sx * bpt[0], bpt[1], bpt[2]))))
        ts = np.linspace(0.0, 1.0, 9)
        for r1, a1, b1 in h_caps:
            p1 = a1[None] + ts[:, None] * (b1 - a1)[None]
            for r2, a2, b2 in t_caps:
                p2 = a2[None] + ts[:, None] * (b2 - a2)[None]
                dist = float(np.linalg.norm(p1[:, None, :] - p2[None, :, :], axis=2).min())
                min_gap = min(min_gap, dist - (r1 + r2))
        out[side] = (hit, min_gap, dy, t_al)
    return out


def _world_forces(w, m, d, seat_body_id, feet_ids, seat_parts, hand_ids):
    """Return `(soles_world_n, seat_up_n, hands_chair_n)` directly from MuJoCo contacts."""
    import numpy as np
    f6 = np.zeros(6)
    soles_n = seat_up = hands_ch = 0.0
    for i in range(d.ncon):
        c = d.contact[i]
        b1, b2 = int(m.geom_bodyid[c.geom1]), int(m.geom_bodyid[c.geom2])
        if b1 in feet_ids and b2 not in seat_parts and b2 not in feet_ids and b2 not in hand_ids:
            w._mj.mj_contactForce(m, d, i, f6)
            soles_n += abs(float(f6[0]))
        elif b2 in feet_ids and b1 not in seat_parts and b1 not in feet_ids and b1 not in hand_ids:
            w._mj.mj_contactForce(m, d, i, f6)
            soles_n += abs(float(f6[0]))
        if (b1 in seat_parts and b2 == seat_body_id) or (b2 in seat_parts and b1 == seat_body_id):
            w._mj.mj_contactForce(m, d, i, f6)
            seat_up += abs(float(f6[0]) * float(c.frame[1]))
        if (b1 in hand_ids and b2 == seat_body_id) or (b2 in hand_ids and b1 == seat_body_id):
            w._mj.mj_contactForce(m, d, i, f6)
            hands_ch += abs(float(f6[0]))
    return soles_n, seat_up, hands_ch


def _run_trial(scene, room_on, at, geom_name, wobble=0.0):
    """Execute a complete sit -> 5.0 s closed-loop WBC hold -> rise trial and gather world metrics."""
    import numpy as np
    from machine.body import figure
    from machine.law import modal
    from machine.process import sprung, worker
    from machine.world import physics

    prev_room, prev_sight = physics.ROOM, physics.SIGHT
    physics.ROOM, physics.SIGHT = float(room_on), 0.0
    try:
        s = worker.session()
        if not room_on:
            worker.apply(s, {'scenario': scene})
        g = s['g']
        w = g['world']
        m, d = w.model, w.data
        if wobble:
            q = d.qpos[3:7].copy()
            cw, sw = math.cos(wobble / 2.0), math.sin(wobble / 2.0)
            d.qpos[3:7] = (q[0] * cw - q[2] * sw, q[1] * cw + q[3] * sw, q[2] * cw + q[0] * sw, q[3] * cw - q[1] * sw)
            w._mj.mj_forward(m, d)
        for _ in range(500):
            worker.stepped(s)
        gid = m.geom(geom_name).id
        bid = int(m.geom_bodyid[gid])
        ch0 = d.xpos[bid].copy()
        feet_ids = {m.body(n).id for n in ('left_foot', 'left_toes', 'right_foot', 'right_toes')}
        seat_parts = {m.body(n).id for n in ('pelvis', 'left_thigh', 'right_thigh')}
        hand_ids = {m.body(n).id for n in ('left_hand', 'left_fingers', 'right_hand', 'right_fingers')}
        torso_id = m.body('torso').id
        pelvis_id = m.body('pelvis').id
        wt = float(g['fr'][modal.WEIGHT])

        worker.apply(s, {'say': 'sit_chair', **({'at': at} if at else {})})
        t_hold, max_sit_t = None, (35.0 if room_on else 10.0) + HOLD_S
        ch_tilt_max, ch_drift_max = 0.0, 0.0
        played_hold, hold_samples = False, []
        qvel_hold, scale_min = 0.0, 1.0
        pel_hold = []
        side_hist = {'left': [], 'right': []}
        soles_max = 0.0
        seat_up_min = math.inf
        hands_ch_max = 0.0
        trunk_max = pel_roll_max = 0.0

        while g['t'] < max_sit_t and s['fallen_at'] is None and s['epoch'] == 0:
            worker.stepped(s)
            sR = d.geom_xmat[gid].reshape(3, 3)
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(sR[1, 1])))))
            ch_tilt_max = max(ch_tilt_max, tilt)
            ch_drift_max = max(ch_drift_max, float(np.linalg.norm(d.xpos[bid] - ch0)))
            if t_hold is None and sprung.doing(g) == 'seated' and g.get('played') is None:
                t_hold = g['t']
            if t_hold is not None:
                if g['t'] > t_hold + HOLD_S:
                    break
                if g.get('played') is not None:
                    played_hold = True
                if g['n'] % 10 == 0:
                    sc = d.geom_xpos[gid]
                    sh = m.geom_size[gid]
                    pel = sR.T @ (d.xpos[pelvis_id] - sc)
                    pel_hold.append(pel.copy())
                    soles_n, seat_up, hands_ch = _world_forces(w, m, d, bid, feet_ids, seat_parts, hand_ids)
                    soles_max = max(soles_max, soles_n)
                    seat_up_min = min(seat_up_min, seat_up / wt)
                    hands_ch_max = max(hands_ch_max, hands_ch)
                    ht = _hand_thigh_stats(m, d, figure)
                    for side in ('left', 'right'):
                        side_hist[side].append(ht[side])
                    t_up = d.xmat[torso_id].reshape(3, 3)[:, 1]
                    p_mat = d.xmat[pelvis_id].reshape(3, 3)
                    trunk_max = max(trunk_max, math.degrees(math.acos(max(-1.0, min(1.0, float(t_up[1]))))))
                    pel_roll_max = max(pel_roll_max, abs(math.degrees(math.asin(max(-1.0, min(1.0, float(p_mat[1, 0])))))))
                    if g['t'] >= t_hold + 0.5:
                        qvel_hold = max(qvel_hold, float(np.abs(d.qvel[6:]).max()))
                    if 'got' in g:
                        scale_min = min(scale_min, float(g['got'].get('scale', 1.0)))
                    hold_samples.append(float(pel[2] + sh[2]))

        rose_stood = False
        rise_soles, rise_tilt = 0.0, 90.0
        if s['fallen_at'] is None and t_hold is not None:
            worker.apply(s, {'say': 'rise_chair'})
            t_r0 = g['t']
            while g['t'] < t_r0 + RISE_S and s['fallen_at'] is None and s['epoch'] == 0:
                worker.stepped(s)
                sR = d.geom_xmat[gid].reshape(3, 3)
                rise_tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(sR[1, 1])))))
                if sprung.doing(g) in ('stand', 'rest') and g.get('played') is None:
                    rose_stood = True
                    if g['t'] >= t_r0 + 1.0 and sum(g['loads']) / wt >= BOUNDS['rise_soles_min_frac']:
                        break
            rise_soles = sum(g['loads']) / wt
    finally:
        physics.ROOM, physics.SIGHT = prev_room, prev_sight

    pel_arr = np.array(pel_hold) if pel_hold else np.zeros((1, 3))
    pel_drift = float(np.linalg.norm(pel_arr.max(axis=0) - pel_arr.min(axis=0))) if len(pel_hold) > 1 else 9.0
    metrics = {
        'fallen_at': s['fallen_at'],
        'epoch': s['epoch'],
        'hold_reached': t_hold is not None and len(hold_samples) >= 200,
        'played_during_hold': played_hold,
        'pel_front_min_m': min(hold_samples) if hold_samples else -1.0,
        'pel_lat_max_m': float(np.abs(pel_arr[:, 0]).max()) if pel_hold else 9.0,
        'pel_drift_max_m': pel_drift,
        'soles_max_n': soles_max,
        'seat_up_min_frac': seat_up_min if hold_samples else 0.0,
        'hands_chair_max_n': hands_ch_max,
        'chair_tilt_max_deg': ch_tilt_max,
        'chair_drift_max_m': ch_drift_max,
        'trunk_pitch_max_deg': trunk_max,
        'pelvis_roll_max_deg': pel_roll_max,
        'qvel_hold_max': qvel_hold,
        'scale_min': scale_min,
        'rose_ok': (rose_stood and rise_soles >= BOUNDS['rise_soles_min_frac']
                    and rise_tilt < BOUNDS['rise_chair_tilt_max_deg'] and s['fallen_at'] is None),
        'rise_stood': rose_stood,
        'rise_soles_frac': rise_soles,
        'rise_chair_tilt_deg': rise_tilt,
    }
    for side in ('left', 'right'):
        rows = side_hist[side]
        metrics[side + '_contact_frac'] = sum(1 for r in rows if r[0]) / max(len(rows), 1)
        metrics[side + '_gap_max_m'] = max((r[1] for r in rows), default=9.0)
        metrics[side + '_dy_min_m'] = min((r[2] for r in rows), default=-9.0)
        metrics[side + '_t_min'] = min((r[3] for r in rows), default=-1.0)
        metrics[side + '_t_max'] = max((r[3] for r in rows), default=2.0)
    return metrics


def test_anti_cheat_judge_rejects_every_shortcut(report):
    """Anti-cheat evaluator `judge_seated` accepts valid deep sitting and rejects every shortcut."""
    valid = {
        'fallen_at': None, 'epoch': 0, 'hold_reached': True, 'played_during_hold': False,
        'pel_front_min_m': 0.24, 'pel_lat_max_m': 0.01, 'pel_drift_max_m': 0.004,
        'soles_max_n': 0.0, 'seat_up_min_frac': 0.98, 'hands_chair_max_n': 0.0,
        'left_contact_frac': 1.0, 'left_gap_max_m': 0.0, 'left_dy_min_m': 0.06, 'left_t_min': 0.35, 'left_t_max': 0.55,
        'right_contact_frac': 1.0, 'right_gap_max_m': 0.0, 'right_dy_min_m': 0.06, 'right_t_min': 0.35, 'right_t_max': 0.55,
        'chair_tilt_max_deg': 0.5, 'chair_drift_max_m': 0.01, 'trunk_pitch_max_deg': 4.0, 'pelvis_roll_max_deg': 0.5,
        'qvel_hold_max': 0.2, 'scale_min': 0.95, 'rose_ok': True, 'rise_stood': True,
        'rise_soles_frac': 0.99, 'rise_chair_tilt_deg': 1.0,
    }
    cheats = (
        ('edge_glue', {'pel_front_min_m': 0.10}),
        ('foot_load', {'soles_max_n': 15.0}),
        ('hands_on_chair', {'hands_chair_max_n': 20.0}),
        ('hands_off_thigh', {'left_contact_frac': 0.0, 'left_gap_max_m': 0.05}),
        ('playback_active', {'played_during_hold': True}),
        ('chair_tipped', {'chair_tilt_max_deg': 10.0}),
        ('joint_spasm', {'qvel_hold_max': 5.0}),
        ('low_seat_load', {'seat_up_min_frac': 0.50}),
        ('rise_failed', {'rose_ok': False}),
    )
    ok_valid = judge_seated(valid)['ok']
    rejected = [name for name, patch in cheats if not judge_seated({**valid, **patch})['ok']]
    report.check('judge accepts valid deep sit and rejects all %d shortcuts' % len(cheats),
                 ok_valid and len(rejected) == len(cheats),
                 'ok_valid=%s rejected=%s' % (ok_valid, rejected))


def test_chair_deep_sit_hands_on_thighs_zero_foot_load(report):
    """Fixed `chair_behind` across three wobbles sits deep, holds 5 s closed-loop with hands on thighs, and rises."""
    for wobble in (0.0, 2e-4, -2e-4):
        got = _run_trial('chair_behind', False, None, 'chair_seat', wobble=wobble)
        res = judge_seated(got)
        report.check('chair_behind wobble=%+.4f passes strict seated judge' % wobble,
                     res['ok'], '; '.join(res['reasons']) or 'ok')


def test_loose_lodge_chair_deep_sit_no_tipping(report):
    """Free 5 kg wooden chair in the lodge is sat on deeply, held 5 s with zero tipping and zero foot weight, and risen from."""
    from machine.world import room
    got = _run_trial('lodge', True, room.CHAIR_AT, 'seat', wobble=0.0)
    res = judge_seated(got)
    report.check('free 5 kg lodge_chair passes strict seated judge without tipping',
                 res['ok'], '; '.join(res['reasons']) or 'ok')


ROSTER = (test_anti_cheat_judge_rejects_every_shortcut,
          test_chair_deep_sit_hands_on_thighs_zero_foot_load,
          test_loose_lodge_chair_deep_sit_no_tipping)


def main(argv=None):
    """Run all chair seated tests or those named on the CLI."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
