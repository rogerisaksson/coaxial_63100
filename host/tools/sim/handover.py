#!/usr/bin/env python3
"""Her hip through each handover: its height, her landing knee, her trailing leg's heel and length.

Her walk on the flat (the player's `w12 -3`), measured from 6 to 12 s at every landing - a sole's
load crossing LANDED of her weight -: her hips' height a stance before, 0.10 s before and then; her
centre's climb and the lift her loop asked; the trailing leg's load, its hip-to-ankle length, its
knee, its heel's pitch and the pitch asked of it, its torques; the landing knee, the least it bends
to after, the sole's fall and how far ahead it is; the load's peak; her hip's passes over its clamp.

    python tools/sim/handover.py                          # her walk as built, here: a JSON row
    python tools/sim/handover.py --specs '' k=12000 heelrate=12,flatk=800   # variants, a relay job each

A spec is `key=value` pairs: `ax` the walk's axial target (init_walk's and WALK_ON's), `rock` a
factor on the walk stream's foot rock, `drive` its drive, `k` the stride spring's stiffness (`c` its
damping, 0.2 k unsaid), `heel` deg, `heelrate` rad/s, `flatk`, `fold`, `dip`, `ls`, `rise` the stride's; `mhz`,
`mdrive`, `mturn`, `max` (its axial target) the march's (the jog behaviour's two phases); `sea` her ankle's series
spring, N m/rad. `--play 'W12 -3'` marches her.
"""
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

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
    from machine.law import se2, stride
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
    for key, name, scale in (('heel', 'HEEL_RAD', math.pi / 180.0), ('heelrate', 'HEEL_RAD_S', 1.0), ('flatk', 'FLAT_K', 1.0),
                             ('fold', 'FOLD_RAD', 1.0), ('dip', 'DIP_M', 1.0), ('ls', 'LAND_SHARE', 1.0), ('rise', 'RISE_U', 1.0)):
        if key in spec:
            setattr(stride, name, scale * float(spec[key]))
    if 'swing' in spec:
        stride.SWING = float(spec['swing'])
        stride.PHI_W = math.pi * stride.SWING
    if 'sea' in spec:
        from machine.body import drives
        drives.SERIES['ankle'] = float(spec['sea'])
    return tune.given(edits) if edits else None


def ran(spec, trace=False, play='w12 -3', entries=None, window=(FROM_S, TO_S)):
    """Her walk under `spec`, measured: a row (dict); `trace`, the passes round one landing in it."""
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
    log = {k: [] for k in ('t', 'ld', 'knee', 'tilt', 'heeled', 'hip', 'along', 'tau_h', 'tau_a', 'tau_k', 'vs', 'vcom', 'acc', 'leg', 'st', 'bend', 'pre', 'ank', 'tau_hs', 'begun', 'lead', 'off', 'pitch', 'qp', 'sw', 'low', 'tilt_g')}
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
        log['along'].append(list(((soles - c) @ rot.T)[:, 0]))
        log['leg'].append(list(np.linalg.norm(leg, axis=1)))
        log['vs'].append(list(np.asarray(y[modal.VSOLE]).reshape(2, 3)[:, 1]))
        log['vcom'].append(float(y[modal.VCOM][1]))
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

    out = scenarios.ran('handover', 0.0, laid=laid, **({'texts': texts} if texts is not None else {}))
    row: dict = dict(spec=','.join('%s=%s' % kv for kv in spec.items()), stood=out['stood'], fell=out['fell_at'], samples=len(log['t']))
    if len(log['t']) < 10:
        return row
    return _summed(log, seen, row, trace, from_s, to_s)


def _summed(log, seen, row, trace, from_s, to_s):
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
    if offs and lands:
        row['toe_tilt'] = round(float(np.median([A['tilt'][i, b] for i, b in offs])), 1)
        row['double_s'] = round(float(np.median([min([A['t'][i] - t for t in lands if t <= A['t'][i]] or [-1.0]) for i, _b in offs])), 3)
        row['begun'] = round(float(np.median([max(A['begun'][min(i + 3, len(A['t']) - 1), b], A['begun'][i, b]) for i, b in offs])), 3)
    row['lead_p99'] = round(float(np.percentile(np.abs(A['lead']), 99)), 3)
    row['period'] = round(float(np.median(np.diff(lands))), 3) if len(lands) > 2 else None
    starts = sorted(A['t'][i] for b in (0, 1) for i in range(1, len(A['t'])) if A['off'][i - 1, b] < 0.5 <= A['off'][i, b])
    row['rhythm_s'] = round(float(np.median(np.diff(starts))), 3) if len(starts) > 2 else None
    if trace and len(at) > 6:
        i, b = at[6]
        o = 1 - b
        row['trace'] = ['t ld_o ld_b qp_o qp_b st_o off_o knee_o knee_b tilt_o tipped_o hip acc pre_o tauk_o taua_o tauh_o tauh_b sole_o_up sacc_up sacc_along along_o']
        for j in range(i - 3 * int(round(0.03 / dt)), i + 5 * int(round(0.03 / dt)) + 1, max(1, int(round(0.01 / dt)))):
            row['trace'].append(' '.join('%.3g' % x for x in (
                A['t'][j] - A['t'][i], A['ld'][j, o], A['ld'][j, b], A['qp'][j, o], A['qp'][j, b], A['st'][j, o], A['off'][j, o],
                A['knee'][j, o], A['knee'][j, b], A['tilt'][j, o], A['sw'][j, 3 + o], A['hip'][j], A['acc'][j], A['pre'][j, o],
                A['tau_k'][j, o], A['tau_a'][j, o], A['tau_hs'][j, o], A['tau_hs'][j, b], A['vs'][j, o], A['sw'][j, o], A['sw'][j, 5 + o], A['along'][j, o])))
    return row


def parsed(text):
    """A spec of `text`'s `key=value` pairs, comma-separated."""
    return dict(kv.split('=', 1) for kv in text.split(',') if kv)


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--spec', default=None, help='one run here, its row as JSON')
    parser.add_argument('--specs', nargs='*', default=None, help="variants, a relay job each ('' her walk as built)")
    parser.add_argument('--trace', action='store_true', help='the passes round one landing, a line each')
    parser.add_argument('--play', default='w12 -3', help="the player's moves (tools/sim/player.py), her walk unsaid")
    parser.add_argument('--entries', default=None, help="the gym's entries instead, 'stand@0.5 sway@2 step@5 walk@9' (tools/sim/ortho.planned)")
    parser.add_argument('--window', default='%g,%g' % (FROM_S, TO_S), help='the seconds measured, from,to')
    args = parser.parse_args(argv)
    if args.specs is None:
        row = ran(parsed(args.spec or ''), args.trace, args.play, args.entries, tuple(float(x) for x in args.window.split(',')))
        for line in row.pop('trace', []):
            print(line)
        print(json.dumps(row))
        return 0
    from tools.dev import focus
    here = [sys.executable, '-X', 'utf8', os.path.abspath(__file__)]
    jobs = [focus.Job(spec or 'base', here + ['--spec', spec, '--play', args.play, '--window', args.window]
                      + (['--entries', args.entries] if args.entries else []) + (['--trace'] if args.trace else []), 1.5, 900.0)
            for spec in args.specs]
    rows = {}
    for job, text, _code, seconds in focus.relay(jobs):
        lines = [line for line in text.splitlines() if line.startswith('{')]
        rows[job.name] = (lines[-1] if lines else 'NO ROW: ' + ' | '.join(text.splitlines()[-3:])[-400:]) + ' (%.0f s)' % seconds
        if args.trace:
            traced = [line for line in text.splitlines() if line[:1] in 't-0123456789']
            rows[job.name] = os.linesep.join([job.name + ':'] + traced + [rows[job.name]])
    for spec in args.specs:
        print(rows[spec or 'base'])
    return 0


if __name__ == '__main__':
    sys.exit(main())
