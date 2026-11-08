#!/usr/bin/env python3
"""Her hip through each handover: its height, her landing knee, her trailing leg's heel and length.

Her walk on the flat (the player's `w12 -3`), measured from 6 to 12 s at every landing - a sole's
load crossing LANDED of her weight -: her hips' height a stance before, 0.10 s before and then; her
centre's climb and the lift her loop asked; the trailing leg's load, its hip-to-ankle length, its
knee, its heel's pitch and the pitch asked of it, its torques; the landing knee, the least it bends
to after, the sole's fall and how far ahead it is; the load's peak; her hip's passes over its clamp.

    python tools/sim/handover.py                          # her walk as built, here: a JSON row
    python tools/sim/handover.py --specs '' k=12000 heelrate=12,flatk=800   # variants, a relay job each
    python tools/sim/handover.py --specs '' sea=143,track=12 --ensemble    # each over 3 x 3 starts: median, worst

A spec is `key=value` pairs: `ax` the walk's axial target (init_walk's and WALK_ON's), `rock` a
factor on the walk stream's foot rock, `drive` its drive, `k` the stride spring's stiffness (`c` its
damping, 0.2 k unsaid), `heel` deg, `heelrate` rad/s, `flatk`, `fold`, `dip`, `ls`, `rise`, `edge` rad the stride's; `mhz`,
`mdrive`, `mturn`, `max` (its axial target) the march's (the jog behaviour's two phases); `sea` her ankle's series
spring, N m/rad, `track` its rotor's torque loop (`gearbox.TRACK`), `wfold` the stack's knee fold weight
(`wrench.W_FOLD`). `--play 'W12 -3'` marches her. `--ensemble` runs each spec over ENSEMBLE's grid of the walk's
drive and axial target - the gait is on an edge where one run measures its phase's luck, not its margin - and
prints a summary a spec: how many stood, the hip clamp passes' median and worst, the landing knee's and the
plan's load error's medians. A row also carries the double support's load planning against the sensed loads:
`plan_err_land`/`plan_err_trail` the mean |planned - real| share over the 150 ms from a landing (the landing
and the trailing sole), `plan_zero_ms` when the plan zeroes the trailing sole, `trail_off_ms` when its load leaves.
"""
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.sim.handovers import ENSEMBLE, ensemble, planned

#: A sole has landed at this share of her weight; the window measured, s; a stance before, s.
LANDED, FROM_S, TO_S, STANCE_S = 0.2, 6.0, 12.0, 0.45
GAIT_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'lisp', 'humanoid', 'gait_base.lisp')
POSTURES_PATH = os.path.join(os.path.dirname(GAIT_PATH), 'postures_base.lisp')


def literal(src, key, start=0):
    """(where the number after `key` begins and ends in `src`, from `start`, and its value)."""
    j = src.find(key, start) + len(key)
    z = j
    while src[z] in '0123456789.-':
        z += 1
    return j, z, float(src[j:z])


def arranged(spec):
    """The texts and the stride's numbers as `spec` has them: the texts edited (`tune.given`), or None."""
    from machine.law import rocker, se2, stride, vault
    from machine.texts import ortho, tune
    texts = tune.texts()
    GAIT, POSTURES = (next(k for k in texts if os.path.normcase(os.path.abspath(k)) == os.path.normcase(p)) for p in (GAIT_PATH, POSTURES_PATH))
    gait, postures = texts[GAIT], texts[POSTURES]
    edits = {}
    if 'ax' in spec:
        j, z, _ = literal(se2.WALK_ON, '(mode_axial_compression ')
        se2.WALK_ON = se2.WALK_ON[:j] + spec['ax'] + se2.WALK_ON[z:]
        se2._WALK_ON_WIRE = ortho.compiled(se2.WALK_ON)[0]['']
        j, z, _ = literal(postures, '(mode_axial_compression ', postures.find('(defbehavior init_walk'))
        edits[(POSTURES, j, z)] = float(spec['ax'])
    walk = gait.find('(sequence_id 204)')
    if 'rock' in spec:
        j, z, was = literal(gait, '(amplitude mode_foot_rock ', walk)
        edits[(GAIT, j, z)] = round(was * float(spec['rock']), 3)
    if 'drive' in spec:
        j, z, _ = literal(gait, '(amplitude mode_sagittal_drive ', walk)
        edits[(GAIT, j, z)] = float(spec['drive'])
    jog = postures.find('(defbehavior jog')
    for key, name in (('mhz', '(frequency '), ('mdrive', '(amplitude mode_sagittal_drive '), ('mturn', '(amplitude mode_pelvis_turn '),
                      ('max', '(mode_axial_compression ')):
        if key in spec:
            at = jog
            for _phase in range(2):
                j, z, _ = literal(postures, name, at)
                edits[(POSTURES, j, z)] = float(spec[key])
                at = z
    if 'k' in spec:
        j, z, _ = literal(gait, '(stiffness mode_axial_compression ')
        edits[(GAIT, j, z)] = float(spec['k'])
        j, z, _ = literal(gait, '(damping mode_axial_compression ')
        edits[(GAIT, j, z)] = float(spec.get('c', 0.2 * float(spec['k'])))
    # a stride constant is bound by name where rocker and vault import it: set in each (the C table takes stride's)
    for key, name, scale in (('heel', 'HEEL_RAD', math.pi / 180.0), ('heelrate', 'HEEL_RAD_S', 1.0), ('flatk', 'FLAT_K', 1.0),
                             ('fold', 'FOLD_RAD', 1.0), ('dip', 'DIP_M', 1.0), ('ls', 'LAND_SHARE', 1.0), ('rise', 'RISE_U', 1.0),
                             ('edge', 'EDGE_RAD', 1.0)):
        if key in spec:
            for mod in (stride, rocker, vault):
                if hasattr(mod, name):
                    setattr(mod, name, scale * float(spec[key]))
    if 'swing' in spec:
        stride.SWING = float(spec['swing'])
        stride.PHI_W = math.pi * stride.SWING
    if 'swing_fast' in spec:
        stride.SWING_FAST = float(spec['swing_fast'])
    if 'pace_fast' in spec:
        stride.PACE_FAST_M_S = float(spec['pace_fast'])
    if 'rund' in spec:
        from machine.law import rhythm
        rhythm.R['run_drive'] = float(spec['rund'])
    if 'sea' in spec:
        from machine.body import drives
        drives.SERIES['ankle'] = float(spec['sea'])
    if 'wind' in spec:
        from machine.body import drives
        drives.AHEAD_WIND = float(spec['wind'])
    if 'track' in spec:
        from machine.body import gearbox
        gearbox.TRACK = float(spec['track'])
    if 'wfold' in spec:
        from machine.stack import wrench
        wrench.W_FOLD = float(spec['wfold'])
    return tune.given(edits) if edits else None


def ran(spec, trace=False, play='w12 -3', entries=None, window=(FROM_S, TO_S), at=None, right=False):
    """Her walk under `spec`, measured: a row (dict); `trace`, the passes round one landing in it - round the second `at` instead, given."""
    import numpy as np
    from machine.law import modal
    from tools.sim import scenarios
    from tools.sim.player import played
    texts = arranged(spec)
    if entries:
        from tools.sim import ortho as gym
        plan, secs = [(at, {'say': label}) for at, label, _wire, _names in gym.planned(entries.split())], window[1] + 0.5
    else:
        plan, secs = played(play)
    scenarios.SCENARIOS['handover'] = ('flat', plan, secs, 'stands', None)
    from_s, to_s = window
    log = {k: [] for k in ('t', 'ld', 'knee', 'tilt', 'heeled', 'hip', 'along', 'tau_h', 'tau_a', 'tau_k', 'vs', 'vcom', 'acc', 'leg', 'st', 'bend', 'pre', 'ank', 'tau_hs', 'begun', 'lead', 'off', 'pitch', 'qp', 'sw', 'low', 'tilt_g', 'leaf', 'leaf_k', 'hips', 'dip', 'zmp', 'vx', 'ahead')}
    seen = {'ix': None, 'pos': {}, 'clamp': {}}

    def laid(s):
        g = s['g']
        m, d, t = g['world'].model, g['world'].data, g['t']
        if seen['ix'] is None:
            names = [m.joint(int(m.dof_jntid[i])).name for i in g['body'].act]
            seen['ix'] = {k: [names.index('left_' + k), names.index('right_' + k)] for k in ('hip', 'knee', 'ankle')}
            seen['kq'] = [int(m.jnt_qposadr[m.joint(n).id]) for n in ('left_knee', 'right_knee')]
            seen['aq'] = [int(m.jnt_qposadr[m.joint(n).id]) for n in ('left_ankle', 'right_ankle')]
            clamp = np.asarray(g['body'].clamp, float).ravel()
            if len(clamp) == len(names):
                seen['clamp'] = {k: round(float(clamp[names.index('left_' + k)]), 1) for k in ('hip', 'knee', 'ankle', 'ankle_roll')}
        for mark in (from_s, to_s):
            if mark not in seen['pos'] and t >= mark:
                seen['pos'][mark] = (float(d.qpos[0]), float(d.qpos[2]))
        if not (from_s <= t <= to_s) or g['n'] % g['every'] != 1:
            return
        fr, a, y, ix = g['fr'], g['a'], g['y'], seen['ix']
        c, soles = np.asarray(y[modal.COM]), np.asarray(y[modal.SOLE]).reshape(2, 3)
        rot, hips, tilt = np.asarray(fr[modal.ROT]).reshape(3, 3), np.asarray(y[modal.HIP]).reshape(2, 3), np.asarray(y[modal.TILT])
        spoke = np.stack([np.sin(tilt), 0.0 * tilt, np.cos(tilt)], axis=1)
        inward = np.stack([np.cos(tilt), 0.0 * tilt, -np.sin(tilt)], axis=1)
        leg = (hips - soles) @ rot.T - modal.ANKLE_H * spoke + 0.031 * inward
        tau = g['got']['tau']
        log['t'].append(t)
        log['ld'].append([float(x) / float(fr[modal.WEIGHT]) for x in g['loads']])
        log['knee'].append([math.degrees(float(d.qpos[q])) for q in seen['kq']])
        log['tilt'].append(list(np.degrees(tilt)))
        log['heeled'].append(list(np.degrees(np.asarray(a[modal.HEELED]))))
        log['hip'].append(float(hips[:, 1].mean() - fr[modal.FLOOR]))
        log['hips'].append(list(hips[:, 1] - fr[modal.FLOOR]))
        log['dip'].append(list(a[modal.DIP]))
        # the ask's ZMP along her way against each sole: -acc_along / omega^2 from her centre, less the sole's place
        om2 = modal.G / max(float(c[1]) - float(fr[modal.FLOOR]), 0.3)
        zmp_rel = -float(np.asarray(a[modal.COM_ACC]) @ rot[0]) / om2
        log['zmp'].append([zmp_rel - float(((soles - c) @ rot.T)[b, 0]) for b in (0, 1)] + [float(np.asarray(a[modal.COM_ACC]) @ rot[0])])
        log['along'].append(list(((soles - c) @ rot.T)[:, 0]))
        log['leg'].append(list(np.linalg.norm(leg, axis=1)))
        log['vs'].append(list(np.asarray(y[modal.VSOLE]).reshape(2, 3)[:, 1]))
        log['vcom'].append(float(y[modal.VCOM][1]))
        log['vx'].append(float(np.asarray(y[modal.VCOM]) @ rot[0]))
        from machine.process import sprung
        log['tilt_g'].append(float(sprung.tilt(g)))
        log['acc'].append(float(a[modal.COM_ACC][1]))
        log['tau_h'].append([abs(float(tau[i])) for i in ix['hip']])
        log['tau_a'].append([float(tau[i]) for i in ix['ankle']])
        log['tau_k'].append([float(tau[i]) for i in ix['knee']])
        log['st'].append(list(a[modal.STANCE]))
        log['tau_hs'].append([float(tau[i]) for i in ix['hip']])
        log['begun'].append(list(a[modal.BEGUN]))
        log['lead'].append(list(a[modal.LEAD]))
        log['off'].append(list(a[modal.SWINGS]))
        hq = np.asarray(y[modal.HQUAT])
        hup = np.array([2.0 * (hq[1] * hq[2] - hq[0] * hq[3]), 1.0 - 2.0 * (hq[1] * hq[1] + hq[3] * hq[3]), 2.0 * (hq[2] * hq[3] + hq[0] * hq[1])])
        log['pitch'].append([math.degrees(float(modal.pitched(y, rot)[0])), math.degrees(float(a[modal.WANT][0])),
                             math.degrees(math.atan2(float(hup @ rot[0]), float(hup[1]))), math.degrees(float(modal._turns(y, rot)[0][1, 0]))])
        log['sw'].append([float(a[modal.SOLE_ACC][3 * b + 1]) for b in (0, 1)] + [float(a[modal.RAISE])] + [math.degrees(float(a[modal.TIPPED][b])) for b in (0, 1)]
                         + list(np.asarray(a[modal.SOLE_ACC]).reshape(2, 3) @ rot[0]))
        log['low'].append(list(np.asarray(y[modal.LOWEST]) - fr[modal.FLOOR]))
        log['qp'].append([float(x) / float(fr[modal.WEIGHT]) for x in np.asarray(g['got']['bears'], float).ravel()[:2]]
                         + [float(np.linalg.norm(np.asarray(g['got'].get('residual', 0.0), float))), float(np.max(np.asarray(g['got']['over'], float)))])
        log['bend'].append(math.degrees(float(a[modal.BEND])))
        log['pre'].append(list(np.degrees(np.asarray(a[modal.PREFOLD]))))
        log['ank'].append([math.degrees(float(d.qpos[q])) for q in seen['aq']])
        boxes = getattr(g['world'], 'boxes', None)
        if boxes is not None and seen.get('bx') is None:
            from machine.body.figure import JOINTS
            seen['bx'] = [JOINTS.index('left_ankle'), JOINTS.index('right_ankle')]
        log['leaf'].append([math.degrees(float(boxes.delta[i])) for i in seen['bx']] if boxes is not None else [0.0, 0.0])
        # the board's wound-ahead ask on each ankle's rotor (gearbox.Boxes.ahead) and the motor torque it is read from
        log['ahead'].append(([float(boxes.ahead(np, g['world'].motor)[i]) for i in seen['bx']] + [float(g['world'].motor[i]) for i in seen['bx']]
                             + [float(boxes.drive[i]) for i in seen['bx']]
                             + [float(g['world'].gains[seen['bx'][0], 0]), float(g['world'].target[seen['bx'][0]] - (float(d.qpos[g['world'].qadr[seen['bx'][0]]]) + float(boxes.delta[seen['bx'][0]]))),
                                float(g['world'].feed[seen['bx'][0]])]) if boxes is not None else [0.0] * 9)
        log['leaf_k'].append(float(boxes.k[seen['bx'][0]]) if boxes is not None else 0.0)

    out = scenarios.ran('handover', 0.0, laid=laid, **({'texts': texts} if texts is not None else {}))
    row: dict = dict(spec=','.join('%s=%s' % kv for kv in spec.items()), stood=out['stood'], fell=out['fell_at'], samples=len(log['t']))
    if len(log['t']) < 10:
        return row
    return _summed(log, seen, row, trace, from_s, to_s, at, right)


def _summed(log, seen, row, trace, from_s, to_s, at_s=None, right=False):
    """`ran`'s row of its `log`: the landings' medians, the hip's passes and, `trace`, the passes round one landing."""
    import numpy as np
    A = {k: np.array(v) for k, v in log.items()}
    pos = seen['pos']
    if from_s in pos and to_s in pos:
        row['v'] = round(math.hypot(pos[to_s][0] - pos[from_s][0], pos[to_s][1] - pos[from_s][1]) / (to_s - from_s), 3)
    dt = float(np.median(np.diff(A['t'])))
    back, b10, b5, fwd = (int(round(s / dt)) for s in (STANCE_S, 0.10, 0.05, 0.15))
    events, at = [], []
    for b in (0, 1):
        o = 1 - b
        for i in range(b10 + 1, len(A['t'])):
            if A['ld'][i - 1, b] < LANDED <= A['ld'][i, b]:
                lo, hi = max(0, i - back), min(len(A['t']), i + fwd)
                at.append((i, b))
                events.append(dict(
                    hip_mid=A['hip'][lo:i].max(), hip_10=A['hip'][i - b10], hip_land=A['hip'][i], vcom_10=A['vcom'][i - b10], vcom_land=A['vcom'][i],
                    acc_10=A['acc'][i - b10], acc_5=A['acc'][i - b5], ld_10=A['ld'][i - b10, o], ld_5=A['ld'][i - b5, o], ld_land=A['ld'][i, o],
                    leg_mid=A['leg'][lo:i, o].max(), leg_10=A['leg'][i - b10, o], leg_land=A['leg'][i, o], knee_10=A['knee'][i - b10, o],
                    trail_knee=A['knee'][i, o], tauk_10=A['tau_k'][i - b10, o], tauk_5=A['tau_k'][i - b5, o], taua_5=A['tau_a'][i - b5, o],
                    tauh_5=A['tau_h'][i - b5, o], land_knee=A['knee'][i, b], knee_min=A['knee'][i:hi, b].min(), land_vs=A['vs'][i, b],
                    ahead=A['along'][i, b], behind=A['along'][i, o], trail_tilt=A['tilt'][i, o], trail_heeled=A['heeled'][i, o],
                    peak_ld=A['ld'][i:hi, b].max()))
    row.update(events=len(events), hip_passes=int((A['tau_h'] > 108.0).sum()), hip_p99=round(float(np.percentile(A['tau_h'], 99)), 1),
               ank_p99=round(float(np.percentile(np.abs(A['tau_a']), 99)), 1), knee_p99=round(float(np.percentile(np.abs(A['tau_k']), 99)), 1))
    if events:
        row.update({k: round(float(np.median([e[k] for e in events])), 3) for k in events[0]})
    row['clamp'] = seen['clamp']
    row.update(planned(A, at, dt, fwd, b10, LANDED))
    lands = sorted(A['t'][i] for i, _b in at)
    passes = {}
    for i, b in zip(*np.nonzero(A['tau_h'] > 108.0)):
        since = min([A['t'][i] - t for t in lands if t <= A['t'][i]] or [-1.0])
        kind = '%s %s %s' % ('swing' if A['st'][i, b] < 0.5 else ('heel' if A['heeled'][i, b] > 0.03 else 'flat'),
                             '+' if A['tau_hs'][i, b] > 0 else '-', 'early' if since < 0.1 else ('mid' if since < 0.3 else 'late'))
        passes[kind] = passes.get(kind, 0) + 1
    row['passes'] = passes
    row['pelvis_pitch_pp'] = round(float(np.percentile(A['pitch'][:, 0], 99) - np.percentile(A['pitch'][:, 0], 1)), 1)
    row['head_pitch_pp'] = round(float(np.percentile(A['pitch'][:, 2], 99) - np.percentile(A['pitch'][:, 2], 1)), 1)
    row['pelvis_roll_pp'] = round(float(np.percentile(A['pitch'][:, 3], 99) - np.percentile(A['pitch'][:, 3], 1)), 1)
    row['pitch_max_at'] = [round(float(A['t'][i]), 2), round(float(A['pitch'][i, 0]), 1)] if (i := int(np.argmax(np.abs(A['pitch'][:, 0])))) >= 0 else None
    row['head_max_at'] = [round(float(A['t'][i]), 2), round(float(A['pitch'][i, 2]), 1)] if (i := int(np.argmax(np.abs(A['pitch'][:, 2])))) >= 0 else None
    row['tilt_g_max_at'] = [round(float(A['t'][i]), 2), round(float(A['tilt_g'][i]), 1)] if (i := int(np.argmax(A['tilt_g']))) >= 0 else None
    row['land_tilt_max'] = round(float(max(abs(A['pitch'][i, 0]) for i, _b in at)), 1) if at else None
    offs = [(i, b) for b in (0, 1) for i in range(1, len(A['t'])) if A['st'][i - 1, b] > 0.5 >= A['st'][i, b]]
    if offs:
        row['rel_knee'] = round(float(np.median([A['knee'][i, b] for i, b in offs])), 1)
        row['rel_ld'] = round(float(np.median([A['ld'][i, b] for i, b in offs])), 2)
    leaf = np.abs(A['leaf'])
    row['leaf_deg_p99'] = round(float(np.percentile(leaf, 99)), 1)
    row['ahead_p99'], row['motor_p99'], row['drive_p99'] = (round(float(np.percentile(np.abs(A['ahead'][:, :2]), 99)), 4),
                                                            round(float(np.percentile(np.abs(A['ahead'][:, 2:4]), 99)), 1),
                                                            round(float(np.percentile(np.abs(A['ahead'][:, 4:6]), 99)), 1))
    row['ankle_kp'], row['pd_err_p99'], row['feed_p99'] = (round(float(A['ahead'][-1, 6]), 1), round(float(np.percentile(np.abs(A['ahead'][:, 7]), 99)), 4),
                                                          round(float(np.percentile(np.abs(A['ahead'][:, 8]), 99)), 1))
    row['leaf_J_p99'] = round(float(0.5 * A['leaf_k'].max() * np.radians(np.percentile(leaf, 99)) ** 2), 1)
    if offs and lands:
        row['toe_tilt'] = round(float(np.median([A['tilt'][i, b] for i, b in offs])), 1)
        row['double_s'] = round(float(np.median([min([A['t'][i] - t for t in lands if t <= A['t'][i]] or [-1.0]) for i, _b in offs])), 3)
        row['begun'] = round(float(np.median([max(A['begun'][min(i + 3, len(A['t']) - 1), b], A['begun'][i, b]) for i, b in offs])), 3)
    row['lead_p99'] = round(float(np.percentile(np.abs(A['lead']), 99)), 3)
    row['period'] = round(float(np.median(np.diff(lands))), 3) if len(lands) > 2 else None
    starts = sorted(A['t'][i] for b in (0, 1) for i in range(1, len(A['t'])) if A['off'][i - 1, b] < 0.5 <= A['off'][i, b])
    row['rhythm_s'] = round(float(np.median(np.diff(starts))), 3) if len(starts) > 2 else None
    # the trace round a clean landing: the sole off the floor through half a STANCE_S before (no bounce, no flight)
    clean = [(i, b) for i, b in at if (A['ld'][max(0, i - back // 2):i, b] < LANDED).all()]
    if trace and at_s is not None:
        clean = [(min(int(np.searchsorted(A['t'], at_s)), len(A['t']) - 1), 1 - int(right))]
    if trace and clean:
        i, b = clean[min(6, len(clean) - 1)]
        o = 1 - b
        row['trace'] = ['t ld_o ld_b qp_o qp_b st_o off_o knee_o knee_b tilt_o tipped_o hip acc pre_o tauk_o taua_o tauh_o tauh_b sole_o_up sacc_up sacc_along along_o heeled_o hip_o hip_b dip_o dip_b zmp_o zmp_b acc_along vx']
        for j in range(max(0, i - 8 * int(round(0.03 / dt))), min(len(A['t']), i + 3 * int(round(0.03 / dt)) + 1), max(1, int(round(0.01 / dt)))):
            row['trace'].append(' '.join('%.3g' % x for x in (
                A['t'][j] - A['t'][i], A['ld'][j, o], A['ld'][j, b], A['qp'][j, o], A['qp'][j, b], A['st'][j, o], A['off'][j, o],
                A['knee'][j, o], A['knee'][j, b], A['tilt'][j, o], A['sw'][j, 3 + o], A['hip'][j], A['acc'][j], A['pre'][j, o],
                A['tau_k'][j, o], A['tau_a'][j, o], A['tau_hs'][j, o], A['tau_hs'][j, b], A['vs'][j, o], A['sw'][j, o], A['sw'][j, 5 + o], A['along'][j, o],
                A['heeled'][j, o], A['hips'][j, o], A['hips'][j, b], A['dip'][j, o], A['dip'][j, b], A['zmp'][j, o], A['zmp'][j, b], A['zmp'][j, 2], A['vx'][j])))
    return row


def parsed(text):
    """A spec of `text`'s `key=value` pairs, comma-separated."""
    return dict(kv.split('=', 1) for kv in text.split(',') if kv)


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--spec', default=None, help='one run here, its row as JSON')
    parser.add_argument('--specs', nargs='*', default=None, help="variants, a relay job each ('' her walk as built)")
    parser.add_argument('--trace', action='store_true', help='the passes round one landing, a line each')
    parser.add_argument('--trace-at', type=float, default=None, help='the trace round this second instead of a landing (sole o her left)')
    parser.add_argument('--trace-right', action='store_true', help='with --trace-at: sole o her right')
    parser.add_argument('--play', default='w12 -3', help="the player's moves (tools/sim/player.py), her walk unsaid")
    parser.add_argument('--entries', default=None, help="the gym's entries instead, 'stand@0.5 sway@2 step@5 walk@9' (tools/sim/ortho.planned)")
    parser.add_argument('--window', default='%g,%g' % (FROM_S, TO_S), help='the seconds measured, from,to')
    parser.add_argument('--ensemble', action='store_true', help="each spec over ENSEMBLE's starts (drive x axial), a summary a spec")
    args = parser.parse_args(argv)
    if args.specs is None:
        row = ran(parsed(args.spec or ''), args.trace, args.play, args.entries, tuple(float(x) for x in args.window.split(',')), args.trace_at, args.trace_right)
        for line in row.pop('trace', []):
            print(line)
        print(json.dumps(row))
        return 0
    from tools.dev import focus
    here = [sys.executable, '-X', 'utf8', os.path.abspath(__file__)]
    starts = [',drive=%g,ax=%g' % (d, ax) for d in ENSEMBLE[0] for ax in ENSEMBLE[1]] if args.ensemble else ['']
    runs = [(spec, spec + start) for spec in args.specs for start in starts]
    jobs = [focus.Job(run or 'base', here + ['--spec', run, '--play', args.play, '--window', args.window]
                      + (['--entries', args.entries] if args.entries else []) + (['--trace'] if args.trace else [])
                      + (['--trace-at', str(args.trace_at)] if args.trace_at is not None else []) + (['--trace-right'] if args.trace_right else []), 1.5, 900.0)
            for _spec, run in runs]
    rows = {}
    for job, text, _code, seconds in focus.relay(jobs):
        lines = [line for line in text.splitlines() if line.startswith('{')]
        rows[job.name] = (lines[-1] if lines else 'NO ROW: ' + ' | '.join(text.splitlines()[-3:])[-400:]) + ' (%.0f s)' % seconds
        if args.trace:
            traced = [line for line in text.splitlines() if line[:1] in 't-0123456789']
            rows[job.name] = os.linesep.join([job.name + ':'] + traced + [rows[job.name]])
    if args.ensemble:
        for spec in args.specs:
            texts = [rows[run or 'base'] for s, run in runs if s == spec]
            print(ensemble(spec or 'base', [json.loads(t[:t.rfind('}') + 1]) for t in texts if t.startswith('{')]))
        return 0
    for spec in args.specs:
        print(rows[spec or 'base'])
    return 0


if __name__ == '__main__':
    sys.exit(main())
