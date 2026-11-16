#!/usr/bin/env python3
"""The robot's frictions read back on test_sysid's world off the figure; a drive's misread by its rows.

    python tools/sim/frictions.py                                   # the stand's gains, each drive's f read back
    python tools/sim/frictions.py --gains hip_yaw=523,18.2 --gains hip_yaw=355,12   # a kind's gains, a run each
    python tools/sim/frictions.py --drive right_hip_yaw             # its misread a band of its speed, a stage
    python tools/sim/frictions.py --drive right_hip_yaw --hems      # its misread, the hems integrated or the world's
    python tools/sim/frictions.py --against C:/sources/coaxial_ab   # each run on another tree too, beside

test_sysid's own case (`tests/test_sysid.py`, test_a_world_off_the_figure_is_read_back: the world off the
figure at seed 7, the campaign at seed 4, a row every EVERY steps), the stand's servo gains or a kind's
given: each drive's Coulomb friction read back, the worst four; the corrections' misfit on the fresh
campaign beside the build's and the world's own. With a drive, its friction's misread - the base
parameters' least squares on what the world's own model leaves (`sysid.identify`), each row's share
additive - laid by the drive's speed in its STICK bands (1-2, 2-4, 4-8, 8- of it) and by the campaign's
stage (`campaign.STAGES`), its own rows' share apart, and each band's mean residual along its sliding
(the world's friction under its model's where negative). A run a job on the relay (`tools.dev.focus.relay`).
"""
import argparse
import json
import os
import sys

HOST = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, HOST)

#: A drive's speed bands, in its STICK band's multiples.
BANDS = (1.0, 2.0, 4.0, 8.0)


def misread(lay, base, Y, tau, true, drive):
    """{total, own, bands, stages, along} of `drive`'s friction misread, relative to its own: what the world's
    model `true` (full theta) leaves of `tau`, each row's share of the least squares on the base parameters."""
    import numpy as np
    from machine.identify import campaign, sysid
    ind, k, i = base['ind'], base['names'].index(drive + '.f'), lay['drives'].index(drive)
    Yi, theta = Y[:, :, ind], sysid.projected(base, true)
    left = tau - Yi @ theta
    G = np.einsum('ndp,ndq->pq', Yi, Yi)
    row = np.linalg.solve(G, np.eye(len(ind))[k])
    inflation = float(row[k] * G[k, k])
    partner = {n: float(-row[j] * np.sqrt(G[j, j] / G[k, k]) / row[k]) for j, n in enumerate(base['names'])}
    partners = sorted(partner.items(), key=lambda kv: -abs(kv[1]) if kv[0] != drive + '.f' else 0.0)[:3]
    each = np.einsum('ndp,p,nd->nd', Yi, row, left)
    share, own = each.sum(axis=1) / theta[k], each[:, i] / theta[k]
    speed = np.abs(Y[:, i, lay['names'].index(drive + '.b')]) / lay['stick'][i]
    sliding = np.sign(Y[:, i, lay['names'].index(drive + '.f')])
    edges = np.linspace(0, len(Y), len(campaign.STAGES) + 1).round().astype(int)
    bands, along = [], []
    for lo, hi in zip(BANDS, BANDS[1:] + (np.inf,)):
        at = (speed >= lo) & (speed < hi)
        bands.append(round(100 * float(share[at].sum()), 2))
        along.append(round(float((left[at, i] * sliding[at]).mean()) if at.any() else 0.0, 4))
    return {'total': round(100 * float(share.sum()), 2), 'own': round(100 * float(own.sum()), 2), 'bands': bands,
            'stages': [round(100 * float(share[a:b].sum()), 2) for a, b in zip(edges[:-1], edges[1:])],
            'along': along, 'rows': int((speed > 0.0).sum()), 'inflation': round(inflation, 1),
            'partners': [(n, round(p, 2)) for n, p in partners],
            'left': round(float(np.sqrt(np.mean(left[speed > 0.0, i] ** 2))), 4)}


def played(world, ref, joints):
    """`campaign.excite`'s (ys, taus, touched) of `ref`, and (N + 1, 2, joints) the world's own `joints`' angles
    and speeds a step, read beside its edge arrays."""
    import numpy as np
    from machine.senses import edge
    from machine.world import physics
    m, d = world.model, world.data
    qa, va = ([int(adr[m.joint(j).id]) for j in joints] for adr in (m.jnt_qposadr, m.jnt_dofadr))
    y, driven, now = edge.laid(world), np.flatnonzero(world.driven), [world.at]
    world.clock = lambda: now[0]
    ys, taus, own, touched = [], [], [], 0
    for k in range(len(ref)):
        ys.append(edge.felt(world, y).copy())
        own.append((d.qpos[qa].copy(), d.qvel[va].copy()))
        touched += int(world.robot_own[m.geom_bodyid[d.contact.geom[:d.ncon]]].any())
        if k == len(ref) - 1:
            break
        world.target[driven] = ref[k + 1, driven]
        now[0] = world.at + physics.STEP_S
        world.advance()
        taus.append(world.motor.copy())
    world.clock = lambda: world.data.time
    return np.array(ys), np.array(taus), touched, np.array(own)


def hems(c, world, drive):
    """{laid: misread %} of `drive`'s friction, the campaign's rows laid with its hems integrated (the test's),
    integrated without the strikes' mask, and the world's own hems."""
    import numpy as np
    import test_sysid as case
    from machine.body import mjcf
    from machine.identify import campaign, identified, sysid
    from machine.stack import wbc
    from machine.world import physics
    lay, body, m = c['lay'], c['body'], world.model
    world.reset(campaign.pose(), where=(0.0, mjcf.STAND_M, 0.0))
    (qadr, vadr, *_), _free = sysid.done(campaign.hemming(lay, body, np.zeros((1, 1)), physics.STEP_S, case.EVERY))
    joints = [body.m.joint(int(np.flatnonzero(body.m.jnt_dofadr == v)[0])).name for v in vadr]
    ys, taus, _touched, own = played(world, campaign.setpoints(world, case.SECONDS, np.random.default_rng(4)), joints)
    at = np.arange(0, len(ys) - 1, case.EVERY)
    integrated, _free = sysid.done(campaign.hemming(lay, body, ys, physics.STEP_S, case.EVERY))
    twin = wbc.Body(identified=False)
    for k in lay['hems']:
        b = int(lay['bodies'][k])
        w = m.body(body.m.body(b).name).id
        twin.m.body_mass[b], twin.m.body_ipos[b], twin.m.body_inertia[b] = m.body_mass[w], m.body_ipos[w], m.body_inertia[w]
    twin.mj.mj_setConst(twin.m, twin.d)
    theirs, _free = sysid.done(campaign.hemming(lay, twin, ys, physics.STEP_S, case.EVERY))
    true = (qadr, vadr, own[at, 0], own[at, 1], (own[at + 1, 1] - own[at, 1]) / physics.STEP_S)
    Y0, tau0 = sysid.rows(lay, body, ys, taus, physics.STEP_S, case.EVERY)
    read = {}
    for name, delta in (('whole', identified._fit(lay, body, Y0, tau0, identified.RIDGE)),
                        ('shared', identified.correction(lay, body, Y0, tau0))):
        delta = dict(zip(lay['names'], delta)) if not isinstance(delta, dict) else delta
        twin = wbc.Body(identified=False)
        for k in lay['hems']:
            b = int(lay['bodies'][k])
            link = body.m.body(b).name
            mass = float(body.m.body_mass[b]) + delta.get(link + '.m', 0.0)
            h = float(body.m.body_mass[b]) * body.m.body_ipos[b] + np.array([delta.get('%s.h%s' % (link, a), 0.0)
                                                                            for a in 'xyz'])
            twin.m.body_mass[b], twin.m.body_ipos[b] = mass, h / mass
        twin.mj.mj_setConst(twin.m, twin.d)
        read[name], _free = sysid.done(campaign.hemming(lay, twin, ys, physics.STEP_S, case.EVERY))
    out = {}
    still = (qadr, vadr) + (np.zeros_like(true[2]),) * 3
    for name, extra in (('integrated', None), ('unmasked', integrated), ('on the world\'s hems', theirs),
                        ('on the hems read whole', read['whole']), ('read shared', read['shared']),
                        ('world\'s', true), ('held still', still)):
        Y, tau = sysid.rows(lay, body, ys, taus, physics.STEP_S, case.EVERY, extra)
        base = sysid.base(lay, Y)
        got, theta = sysid.identify(base, Y, tau), sysid.projected(base, sysid.truth(lay, m))
        err = (got['theta'] - theta) / np.maximum(np.abs(theta), 1e-9)
        worst = max((abs(float(e)), n[:-2]) for e, n in zip(err, got['names']) if n.endswith('.f'))
        i = lay['drives'].index(drive)
        left = (tau - Y @ sysid.truth(lay, m))[:, i][np.abs(Y[:, i]).sum(axis=1) > 0.0]
        out[name] = '%+.2f%% (worst %s %.1f%%; its rows %.3f N m rms on the world\'s model)' % (
            100 * float(err[got['names'].index(drive + '.f')]), worst[1], 100 * worst[0],
            float(np.sqrt(np.mean(left ** 2))))
    out['with the hems\' sensitivities'] = sensitive(lay, body, ys, taus, integrated, m, drive)
    lo, hi = body.m.jnt_range[[int(np.flatnonzero(body.m.jnt_dofadr == v)[0]) for v in vadr]].T
    near = (true[2] < lo + 0.1 * (hi - lo)) | (true[2] > hi - 0.1 * (hi - lo))
    near_int = (integrated[2] < lo + 0.1 * (hi - lo)) | (integrated[2] > hi - 0.1 * (hi - lo))
    _own, free = sysid.done(campaign.hemming(lay, body, ys, physics.STEP_S, case.EVERY))
    decay = []
    for k in lay['hems']:
        b = int(lay['bodies'][k])
        jx = int(body.m.body_jntadr[b])
        mass, L, I0 = float(body.m.body_mass[b]), -float(body.m.body_ipos[b, 1]), float(body.m.body_inertia[b, 0])
        J, ks, c = I0 + mass * L * L, float(body.m.jnt_stiffness[jx]), float(body.m.dof_damping[body.m.jnt_dofadr[jx]])
        decay.append(2.0 * J / c)
    after = ~free.copy()
    for h, tau_s in enumerate(decay):
        hit = np.flatnonzero(~free[:, h])
        for s in hit:
            after[s:s + int(round(tau_s / (case.EVERY * physics.STEP_S))) + 1, h] = True
    Yu, tu = sysid.rows(lay, body, ys, taus, physics.STEP_S, case.EVERY, integrated)
    for name, bad in (('hits', ~free), ('near stops', near_int.reshape(len(at), -1, 2).any(axis=2)),
                      ('near the world\'s stops', near.reshape(len(at), -1, 2).any(axis=2)),
                      ('hits and their decay', after)):
        keep = (bad.astype(int) @ lay['hem_limb']) == 0
        Y, tau = Yu * keep[:, :, None], tu * keep
        base = sysid.base(lay, Y)
        got, theta = sysid.identify(base, Y, tau), sysid.projected(base, sysid.truth(lay, m))
        err = (got['theta'] - theta) / np.maximum(np.abs(theta), 1e-9)
        worst = max((abs(float(e)), n[:-2]) for e, n in zip(err, got['names']) if n.endswith('.f'))
        own_err = (100 * float(err[got['names'].index(drive + '.f')]) if drive + '.f' in got['names']
                   else float('nan'))
        out['masked by ' + name] = '%+.2f%% (worst %s %.1f%%; %d of %d rows kept on %s)' % (
            own_err, worst[1], 100 * worst[0],
            int((np.abs(Y[:, lay['drives'].index(drive)]).sum(axis=1) > 0).sum()),
            int((np.abs(Yu[:, lay['drives'].index(drive)]).sum(axis=1) > 0).sum()), drive)
    out['decay s'] = ' '.join('%.2f' % d for d in decay)
    out['hem angles'] = ', '.join('%s rms %.3f of %.3f rad, near a stop %.1f%%' % (
        body.m.joint(int(np.flatnonzero(body.m.jnt_dofadr == v)[0])).name, float(np.sqrt(np.mean((integrated[2][:, j]
        - true[2][:, j]) ** 2))), float(np.sqrt(np.mean(true[2][:, j] ** 2))), 100 * float(near[:, j].mean()))
        for j, v in enumerate(vadr))
    return out


def sensitive(lay, body, ys, taus, integrated, m, drive, step=0.05):
    """`drive`'s friction misread, the rows unmasked beside each hem's sensitivity columns: the torque the
    build's model gives as its pendulum's J or its m L moves `step` of itself, the other held."""
    import time
    import numpy as np
    import test_sysid as case
    from machine.identify import campaign, sysid
    from machine.stack import wbc
    from machine.world import physics
    t0 = time.perf_counter()
    Y0, tau = sysid.rows(lay, body, ys, taus, physics.STEP_S, case.EVERY, integrated)
    t_rows = time.perf_counter() - t0
    nominal, cols = sysid.truth(lay, body.m), []
    for k in lay['hems']:
        b = int(lay['bodies'][k])
        mass, L, I0 = float(body.m.body_mass[b]), -float(body.m.body_ipos[b, 1]), body.m.body_inertia[b].copy()
        for moved in ('J', 'mL'):
            twin = wbc.Body(identified=False)
            if moved == 'J':
                twin.m.body_inertia[b] = I0 + step * (I0[0] + mass * L * L)
            else:
                twin.m.body_ipos[b, 1] = -(1.0 + step) * L
                twin.m.body_inertia[b] = I0 + mass * L * L * (1.0 - (1.0 + step) ** 2)
            twin.mj.mj_setConst(twin.m, twin.d)
            moved_hems, _free = sysid.done(campaign.hemming(lay, twin, ys, physics.STEP_S, case.EVERY))
            Yk, _tau = sysid.rows(lay, body, ys, taus, physics.STEP_S, case.EVERY, moved_hems)
            cols.append((Yk - Y0) @ nominal / step)
    base = sysid.base(lay, Y0)
    Ya = np.concatenate([Y0[:, :, base['ind']], np.stack(cols, axis=2) * (np.abs(Y0).sum(axis=2) > 0.0)[:, :, None]],
                        axis=2).reshape(-1, len(base['ind']) + len(cols))
    got = np.linalg.lstsq(Ya, tau.reshape(-1), rcond=None)[0]
    theta = sysid.projected(base, sysid.truth(lay, m))
    err = (got[:len(theta)] - theta) / np.maximum(np.abs(theta), 1e-9)
    worst = max((abs(float(e)), n[:-2]) for e, n in zip(err, base['names']) if n.endswith('.f'))
    return '%+.2f%% (worst %s %.1f%%; rows %.1f s; moves %s)' % (
        100 * float(err[base['names'].index(drive + '.f')]), worst[1], 100 * worst[0], t_rows,
        ' '.join('%+.3f' % g for g in got[len(theta):]))


def judged(gains, drive, laid_hems=False):
    """{worst, halfway, misread} of test_sysid's off-world case, `gains` {kind: (kp, kd)} on the world's drives;
    `laid_hems` {hems} alone (`hems`)."""
    import numpy as np
    import test_sysid as case
    from machine.body import drives
    from machine.body.figure import JOINTS
    from machine.identify import identified, sysid
    from machine.stack import wbc
    from machine.world import physics
    body = wbc.Body(identified=False)
    c = {'body': body, 'lay': sysid.laid(body)}
    lay, world = c['lay'], case._off(physics.World(), c, 7)
    for i, joint in enumerate(JOINTS):
        if drives.kind(joint) in gains:
            world.gains[i] = gains[drives.kind(joint)]
    if laid_hems:
        return {'hems': hems(c, world, drive)}
    m = world.model
    Y, tau, touched, world = case._campaign(c, 4, world)
    base = sysid.base(lay, Y)
    got = sysid.identify(base, Y, tau)
    true = sysid.projected(base, sysid.truth(lay, m))
    err = np.abs(got['theta'] - true) / np.maximum(np.abs(true), 1e-9)
    f = sorted(((round(100 * float(err[k]), 1), n[:-2]) for k, n in enumerate(got['names']) if n.endswith('.f')),
               reverse=True)
    fresh = wbc.Body(identified=False)
    identified.correct(fresh, identified.correction(lay, body, Y, tau))
    Y3, tau3, _touched, world = case._campaign(c, 5, world, case.SHORT)
    rms = [round(float(sysid.misfit(Y3, tau3, sysid.truth(lay, model)).mean()), 4) for model in (body.m, fresh.m, m)]
    out = {'touched': int(touched), 'worst': f[:4], 'halfway': rms}
    if drive:
        out['misread'] = misread(lay, base, Y, tau, sysid.truth(lay, m), drive)
        out['misread']['read'] = round(100 * float((got['theta'] - true)[got['names'].index(drive + '.f')]
                                                   / true[got['names'].index(drive + '.f')]), 2)
    return out


def line(name, r):
    """A run's lines: its frictions, its corrections' misfit, its drive's misread; or its hems' three."""
    if 'hems' in r:
        return '%-30s misread with the hems %s' % (name, '; '.join('%s %s' % kv for kv in r['hems'].items()))
    b, c_, w = r['halfway']
    text = '%-30s f worst %s; misfit build %.3f corrected %.3f world %.3f (half %.3f)%s' % (
        name, ', '.join('%s %.1f%%' % (n, e) for e, n in r['worst']), b, c_, w, 0.5 * (b + w),
        '' if not r['touched'] else ' TOUCHED %d' % r['touched'])
    if 'misread' in r:
        d = r['misread']
        text += ('\n%-30s misread %+.2f%% (read %+.2f%%), own rows %+.2f%%, %d rows; bands %s %%; stages %s %%;'
                 ' along %s N m' % ('', d['total'], d['read'], d['own'], d['rows'], d['bands'], d['stages'], d['along']))
        text += '\n%-30s inflation %.1f, partners %s, its rows\' residual %.4f N m rms' % (
            '', d['inflation'], ', '.join('%s %+.2f' % tuple(p) for p in d['partners']), d['left'])
    return text


def main(argv=None):
    p = argparse.ArgumentParser(description=(__doc__ or '').split('\n\n')[0])
    p.add_argument('--gains', action='append', default=[], help='KIND=KP,KD on the world\'s drives, a run each')
    p.add_argument('--drive', help='this drive\'s friction misread by its rows')
    p.add_argument('--hems', action='store_true', help='with --drive: its misread, the rows laid with the hems '
                                                       'integrated, unmasked, and the world\'s own')
    p.add_argument('--against', help='another tree (its root), each run there too')
    p.add_argument('--jobs', type=int, default=6, help='runs side by side (`tools.dev.focus.relay`)')
    p.add_argument('--one', nargs=4, help=argparse.SUPPRESS)
    args = p.parse_args(argv)
    if args.one:
        root, told, drive, laid_hems = args.one
        sys.path[:0] = [os.path.join(root, 'host'), os.path.join(root, 'host', 'tests')]
        os.environ['PYTHONPATH'] = os.path.join(root, 'host')
        gains = {k: tuple(v) for k, v in json.loads(told).items()}
        print('FRICTIONS ' + json.dumps(judged(gains, drive if drive != '-' else None, laid_hems == '1')))
        return 0
    from tools.dev import focus
    trees = [os.path.dirname(HOST)] + ([os.path.abspath(args.against)] if args.against else [])
    runs = [{}] + [{g.split('=')[0]: [float(v) for v in g.split('=')[1].split(',')]} for g in args.gains]
    work = [focus.Job('%s %s' % (os.path.basename(tree), ' '.join('%s=%s' % (k, ','.join('%g' % x for x in v))
                                                                  for k, v in run.items()) or 'stand\'s'),
                      [sys.executable, '-X', 'utf8', os.path.abspath(__file__), '--one', tree, json.dumps(run),
                       args.drive or '-', '1' if args.hems and args.drive else '0'], timeout=900.0)
            for run in runs for tree in trees]
    for job, text, _code, _s in focus.relay(work, batons=args.jobs):
        got = next((ln[10:] for ln in (text or '').splitlines() if ln.startswith('FRICTIONS ')), None)
        print(line(job.name, json.loads(got)) if got else '%-30s no run: %s' % (
            job.name, (text or 'out of time')[-300:].replace('\n', ' | ')), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
