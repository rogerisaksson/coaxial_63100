"""A bench row traced through her chain's QP: joints, contacts, the way chosen, the laws met or not.

The python twin stands in for wbc_chain.c as `chain_bounded.py` runs it (a variant, a turned start); each line counts
the passes whose QP met its laws and those it could not (the least squares alone, her torques shrunk).

    python -X utf8 tools/sim/chain_trace.py bed_leg --variant twin --wobble 8 --since 5 --until 11
"""
import argparse
import math
import sys

#: The joints a line shows: her left hip, left knee and spine.
SHOWN = ('left_hip', 'left_knee', 'spine')


def bodily(g, m, d):
    """Her body this pass, a line each: her hips' roll, her trunk off plumb and as asked, her joints against her
    posture; her let-go feet; her step's accelerations against her world's; what bears her as felt and as her step
    counted on it; her step's torques against what her actuators applied."""
    import numpy as np
    from machine.body import drives
    from machine.body.figure import JOINTS
    from machine.law import se2
    from machine.senses.borne import LET
    from machine.stack import chain
    # her hips' line off level (+ her right hip up: on her left side) and her trunk off plumb, deg (`stunts`'s)
    hips = d.xpos[m.body('right_thigh').id] - d.xpos[m.body('left_thigh').id]
    trunk = d.xpos[m.body('head').id] - d.xpos[m.body('pelvis').id]
    roll = math.degrees(math.asin(max(-1.0, min(1.0, float(hips[1]) / max(float(np.linalg.norm(hips)), 1e-9)))))
    plumb = math.degrees(math.acos(max(-1.0, min(1.0, float(trunk[1]) / max(float(np.linalg.norm(trunk)), 1e-9)))))
    # her joints' angle and her posture's target for each, deg (her driven joints' order, `wbc.Body`'s: no passive
    # spring)
    s, ask, got = g.get('s', {}), g.get('ask', {}), g.get('got', {})
    q, want = s.get('q'), ask.get('posture')
    driven = [j for j in JOINTS if drives.passive(j) is None]
    joints = ' '.join('%s %+4.0f>%+4.0f' % (j, math.degrees(float(q[driven.index(j)])), math.degrees(float(want[driven.index(j)])))
                      for j in SHOWN) if q is not None and want is not None else ''
    # her trunk's turn as her chain is asked it (`chain.tasks`' second turn), off plumb, deg
    turns = ask.get('turns', ())
    asked = (math.degrees(math.acos(max(-1.0, min(1.0, float(chain.ways(np.asarray(turns[1], float))[1][1])))))
             if len(turns) > 1 else math.nan)
    print('       hips roll %+4.0f trunk off plumb %3.0f asked %3.0f %s' % (roll, plumb, asked, joints), flush=True)
    # her let-go feet (`borne.LET`'s patches), a side each: the cap her release lays on them (`g['let_cap']`,
    # `cascade.released`), how far each leg is released (`g['emancipated']`), what they feel and what her step put on
    # them (`got['borne']`, her step's contacts in `s['contacts']`' order), N up
    z = np.asarray(got.get('borne', np.zeros((0, 3))), float).reshape(-1, 3)
    felt_up, step_up, felt_by, step_by, give_by = [0.0, 0.0], [0.0, 0.0], {}, {}, {}
    for i, c in enumerate(s.get('contacts', ())):
        planned = float(z[i][1]) if i < len(z) else math.nan
        if int(c.get('patch', 0)) >= LET:
            felt_up[int(c['patch']) // LET - 1] += float(c['f'][1])
            step_up[int(c['patch']) // LET - 1] += planned
        k = int(c.get('body', -1))
        nm = m.body(k).name if k >= 0 else 'sole%d' % (-1 - k)
        felt_by[nm], step_by[nm] = felt_by.get(nm, 0.0) + float(c['f'][1]), step_by.get(nm, 0.0) + planned
        give_by[nm] = max(give_by.get(nm, 0.0), float(c.get('give', 0.0)))
    cap, gamma = np.asarray(g.get('let_cap', (math.inf, math.inf)), float), np.asarray(g.get('emancipated', (0.0, 0.0)), float)
    print('       let-go feet cap %5.0f %5.0f released %.2f %.2f felt %4.0f %4.0f step %4.0f %4.0f %s' % (
        cap[0], cap[1], gamma[0], gamma[1], felt_up[0], felt_up[1], step_up[0], step_up[1],
        'program' if g.get('played') is not None else 'law'), flush=True)
    # her step's accelerations (`got['qacc']`) against her world's (MuJoCo's last), deg/s^2
    qacc, tau, act = got.get('qacc'), got.get('tau'), getattr(g.get('body'), 'act', None)
    if qacc is not None and act is not None:
        print('       acc step>world ' + ' '.join('%s %+6.0f>%+6.0f' % (
            j, math.degrees(float(qacc[act[driven.index(j)]])), math.degrees(float(d.qacc[act[driven.index(j)]])))
            for j in SHOWN), flush=True)
    # what bears her by body: what she feels against what her step counted on, N up, the most felt first; the give her
    # skin reads of it, m/s^2 a N (`borne.read`'s)
    print('       up felt>step ' + ' '.join('%s %.0f>%.0f (%.2g)' % (nm, felt_by[nm], step_by[nm], give_by[nm])
                                          for nm in sorted(felt_by, key=lambda n: -abs(felt_by[n]))[:5]), flush=True)
    # her drives: the torque her step asked (`got['tau']`) against what her world's actuators applied and its passive
    # forces (her boards' damping, `qfrc_passive`), N m, and how limp she rests (`se2.limp`)
    if tau is not None and act is not None:
        print('       tau step>applied ' + ' '.join('%s %+5.1f>%+5.1f pas %+5.1f' % (
            j, float(tau[driven.index(j)]), float(d.qfrc_actuator[act[driven.index(j)]]),
            float(d.qfrc_passive[act[driven.index(j)]])) for j in SHOWN) + ' limp %.2f' % se2.limp(g), flush=True)


def kinematics(snap, g, m, d, regime):
    """Her kinematics this pass through her introspection's snapshot (`introspect.Tap`): her pelvis's speed along
    its way and up against the twist told (m/s), her arms' joints - angle, speed and its peak since the last
    snapshot (deg, deg/s; `body`) -, and for each what her step meant to do against what her world did (deg/s^2,
    `wrench.qacc` against the world's), her base's the same (m/s^2); the hands' slip on what bears them (m/s)."""
    import numpy as np
    from machine.stack import chain
    body, got = snap['body'], g.get('got', {})
    way = chain.ways(g['s']['turns'][0]['quat'])[2] if g.get('s') else np.array([0.0, 0.0, 1.0])
    vel, acc = np.asarray(d.qvel[:3], float), np.asarray(d.qacc[:3], float)
    plan = np.asarray(got['qacc'], float)[:3] if 'qacc' in got else np.full(3, math.nan)
    print('       kin    pelvis along %+.3f up %+.3f m/s (told %+.2f) | base along plan %+5.2f world %+5.2f, up plan %+5.2f world %+5.2f m/s^2 | let %d' % (
        float(vel @ way), float(vel[1]), float(g.get('ask', {}).get('twist', (0.0,))[0]), float(plan @ way), float(acc @ way),
        float(plan[1]), float(acc[1]), len(getattr(g['body'], 'chain_let', ()))), flush=True)
    qadr, vadr = g['world'].qadr, g['world'].vadr
    from machine.body import drives
    from machine.senses.borne import CARPAL
    from machine.stack.shrunk import box
    b = g['body']
    driven = [n for n in body['angles'] if drives.passive(n) is None]
    # the rows level 1 held: the laws' rows a contact (2 held or 3 free - `holds`, the step's -, and 8 of its pyramid
    # where its tangents are unknown; `chain_qp.level1`), then her torques' bounds (nd each way): which drives sit on
    # a bound
    contacts, twist = g.get('s', {}).get('contacts', ()), float(g.get('ask', {}).get('twist', (0.0,))[0])
    holds = got.get('holds') or [True] * len(contacts)
    laws = sum((2 if holds[i] else 3) + 8 * (not (int(c['body']) >= 0 and int(c.get('patch', 0)) < CARPAL and twist != 0.0))
               for i, c in enumerate(contacts))
    held1, nd = (got.get('held') or ((), ()))[0], len(b.act)
    bound = sorted((i - laws) % nd for i in held1 if laws <= i < laws + 2 * nd)
    lo_hi = box(b, g['ask'])[1:] if g.get('ask') else (None, None)
    print('       kin    level 1 held %d rows, torque bounds on %s' % (len(held1), ' '.join(driven[i] for i in bound) or 'none'), flush=True)
    for j, name in enumerate(body['angles']):
        if not any(part in name for part in ('shoulder', 'elbow', 'wrist')):
            continue
        plan = math.degrees(float(np.asarray(got['qacc'])[vadr[j]])) if 'qacc' in got else math.nan
        # its torques: the step's ask, the world's actuator, its clamp and its mesh's friction (the band a change
        # under it never leaves), N m
        i = driven.index(name) if name in driven else -1
        asked = float(np.asarray(got['tau'])[i]) if i >= 0 and 'tau' in got else math.nan
        print('       kin    %-20s %+6.0f deg %+6.0f deg/s (peak %5.0f) | plan %+7.0f world %+7.0f deg/s^2 | tau %+5.1f>%+5.1f in [%+4.0f, %+4.0f] mesh %.1f' % (
            name, body['angles'][name], body['speeds'][name], body['peaks'][name], plan, math.degrees(float(d.qacc[vadr[j]])),
            asked, float(d.qfrc_actuator[vadr[j]]), float(lo_hi[0][i]) if i >= 0 and lo_hi[0] is not None else math.nan,
            float(lo_hi[1][i]) if i >= 0 and lo_hi[1] is not None else math.nan, drives.mesh(name)), flush=True)
    for c in g.get('s', {}).get('contacts', ()):
        if c.get('carpal'):
            slip = np.asarray(c['J'], float) @ np.asarray(g['s']['v'], float)
            n = np.asarray(c['n'], float)
            print('       kin    %-20s slip %.3f m/s along, %+.3f out' % (
                m.body(int(c['body'])).name, float(np.linalg.norm(slip - (slip @ n) * n)), float(slip @ n)), flush=True)


def replayed(g, regime, flags):
    """Her last closed-chain pass replayed on its own inputs (`g['_chain_in']`, as her step was called): each body
    as she feels it bear her and as her QP gives it, N up, its laws met or not; the replay's torques against the
    pass's; then the QP with each of its asks let go - her posture, her turns, both - whose rows ask what of her
    contacts (the most felt three)."""
    import numpy as np
    from machine.stack import chain
    b, sc, ask = g['_chain_in']
    m, load, warm = g['world'].model, np.array(b.load, copy=True), getattr(b, 'chain_warm', ((), ()))
    let = set(getattr(b, 'chain_let', ()))
    try:
        got = chain.step(b, sc, ask, regime, *flags)
    finally:
        b.load, b.chain_warm, b.chain_let = load, warm, set(let)
    contacts = sc.get('contacts', ())
    names = [m.body(int(c['body'])).name if int(c.get('body', -1)) >= 0 else 'sole%d' % (-1 - int(c['body'])) for c in contacts]
    felt = {n: sum(float(c['f'][1]) for c, n2 in zip(contacts, names) if n2 == n) for n in set(names)}
    # each point's speed along the force it feels, m/s (+ out of what bears it), her bodies' most
    Q = chain.frames(np.concatenate([np.asarray(c['n'], float) for c in contacts])) if len(contacts) else np.zeros((0, 3, 3))
    vn = [float(Q[i, 0] @ (np.asarray(c['J'], float) @ sc['v'])) for i, c in enumerate(contacts)]
    for nm in sorted(felt, key=lambda n: -felt[n])[:6]:
        idx = [i for i, n in enumerate(names) if n == nm]
        print('       replay %-16s felt %5.0f step %5.0f v_n %+.2f' % (
            nm, felt[nm], sum(float(got['borne'][i][1]) for i in idx), max((vn[i] for i in idx), key=abs)), flush=True)
    # each contact in its own frame: what it feels and what the QP plans, normal and shear, N; held by its patch's felt
    # (regime.hold) or free
    by = {}
    for c in contacts:
        by[c.get('patch', 0)] = by.get(c.get('patch', 0), 0.0) + float(np.linalg.norm(c['f']))
    order = sorted(range(len(contacts)), key=lambda i: -float(np.linalg.norm(contacts[i]['f'])))[:8]
    way = chain.ways(sc['turns'][0]['quat'])[2] if sc.get('turns') else np.array([0.0, 0.0, 1.0])
    holds = got.get('holds') or [True] * len(contacts)
    for i in order:
        fw, zw = np.asarray(contacts[i]['f'], float), np.asarray(got['borne'][i], float)
        fl, zl = Q[i] @ fw, Q[i] @ zw
        # its normal acceleration as the step means to move it (+ out of its support): a force on a contact leaving
        # its support is one the world never delivers; `way` each force's part along her sled's way, world
        an = float(Q[i, 0] @ (np.asarray(contacts[i]['J'], float) @ np.asarray(got['qacc'], float) + np.asarray(contacts[i]['dJv'], float)))
        print('       frame  %-16s %s felt n %5.0f t %4.0f  step n %5.0f t %4.0f  a_n %+6.1f v_n %+.2f  way felt %+4.0f step %+4.0f' % (
            names[i], 'held' if holds[i] else 'free', fl[0], np.hypot(fl[1], fl[2]), zl[0], np.hypot(zl[1], zl[2]), an, vn[i],
            float(fw @ way), float(zw @ way)), flush=True)
    print("       replay's laws %s, residual %.2g N, torques off the pass's %.2g N m" % (
        'met' if got.get('met') else 'NOT met', got['residual'],
        float(np.abs(np.asarray(got['tau']) - np.asarray(g['got']['tau'])).max())), flush=True)
    top = sorted(felt, key=lambda n: -felt[n])[:3]
    for what, kw in (('no posture', {'posture': 0.0}), ('no turns', {'turn': 0.0}), ('neither', {'posture': 0.0, 'turn': 0.0})):
        try:
            gone = chain.step(b, sc, ask, regime._replace(**kw), *flags)
        finally:
            b.load, b.chain_warm, b.chain_let = load, warm, set(let)
        print('       replay %-10s ' % what + ' '.join('%s %.0f' % (nm, sum(float(gone['borne'][i][1]) for i, n in enumerate(names)
                                                                      if n == nm)) for nm in top), flush=True)


def main(argv=None):
    import numpy as np
    from machine.process import introspect, sprung
    from machine.stack import chain
    from tools.cores import step as glue
    from tools.sim import chain_bounded
    p = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    p.add_argument('name', help='a bench row (tools/sim/scenarios.py)')
    p.add_argument('--variant', default='twin', choices=sorted(chain_bounded.VARIANTS))
    p.add_argument('--wobble', type=float, default=0.0, help='the start laid turned, mrad')
    p.add_argument('--since', type=float, default=0.0, help='s')
    p.add_argument('--until', type=float, default=math.inf, help='s')
    p.add_argument('--every', type=float, default=0.25, help='s between lines')
    p.add_argument('--rounds', action='store_true', help="each line's closed-chain pass replayed, its asks let go in turn")
    p.add_argument('--kin', action='store_true', help="her kinematics through her introspection's snapshot (`kinematics`)")
    args = p.parse_args(argv)
    regime, flags = glue.regime(), chain_bounded.VARIANTS[args.variant]
    seen, last, tap, kept = {'calls': 0, 'unmet': 0}, [-math.inf], introspect.Tap(), [None]
    from tools.sim import scenarios
    real = scenarios.ran

    def ran(name, wobble=0.0, laid=None, texts=None):
        def traced(s):
            if laid is not None:
                laid(s)
            g = s['g']
            if args.rounds and g.get('_twin') and not g.get('_captured'):
                # her closed chains' step called through a catch of its inputs, the replay's (`replayed`)
                def captured(b, sc, ask, solve=g['solves'][1], g=g):
                    g['_chain_in'] = (b, sc, ask)
                    return solve(b, sc, ask)
                g['solves'], g['_captured'] = (g['solves'][0], captured), True
            t = g['t']
            ask = g.get('ask', {})
            if ask:
                # every pass the way chosen otherwise than were she lying as her support's latch has her (reclined
                # alone), and what bears her off her soles (`borne`'s prop, of her weight; `somatic.asked`'s propped)
                latch = dict(ask, propped=True)
                if chain.chained(latch) != chain.chained(ask):
                    print('%6.3f way %s, latched %s: swing %s stance %s let_go %s reclined %.1f prop %.2f palms %.2f' % (
                        t, 'chain' if chain.chained(ask) else 'upright', 'chain' if chain.chained(latch) else 'upright',
                        sorted(ask.get('swing', {})), ask.get('stance'), ask.get('let_go'),
                        float(ask.get('reclined', 0.0)), float(g.get('s', {}).get('prop', (math.nan,))[0]),
                        float(np.sum(g.get('s', {}).get('palms', (math.nan,))))), flush=True)
            if ask and chain.chained(ask) and g.get('got'):
                # the passes since the last line, and those whose QP could not meet its laws (the python twin says)
                seen['calls'] += 1
                seen['unmet'] += int(not g['got'].get('met', True))
            snap = tap.passed(s)
            kept[0] = snap if snap is not None else kept[0]
            if t < args.since or t > args.until or t - last[0] < args.every:
                return
            last[0] = t
            m, d = g['world'].model, g['world'].data
            if kept[0] is not None and args.kin:
                kinematics(kept[0], g, m, d, regime)
            v = np.abs(d.qvel[6:]) * 180.0 / math.pi
            j = int(np.argmax(v))
            bodies = {}
            for c in g.get('s', {}).get('contacts', ()):
                k = int(c.get('body', -1))
                nm = m.body(k).name if k >= 0 else 'sole%d' % (-1 - k)
                bodies[nm] = bodies.get(nm, 0.0) + float(np.linalg.norm(c['f']))
            top = ' '.join('%s:%.0f' % kv for kv in sorted(bodies.items(), key=lambda kv: -kv[1])[:3])
            bodily(g, m, d)
            ask = g.get('ask', {})
            if args.rounds and g.get('_chain_in') is not None and ask and chain.chained(ask):
                replayed(g, regime, flags)
            # the way chosen (chain.chained): the closed chains' step, or the upright's, and what chose it
            print('       way %s: swing %s stance %s let_go %s reclined %.1f prop %.2f' % (
                'chain' if chain.chained(ask) else 'upright', sorted(ask.get('swing', {})), ask.get('stance'),
                ask.get('let_go'), float(ask.get('reclined', 0.0)), float(g.get('s', {}).get('prop', (math.nan,))[0])),
                flush=True)
            print('%6.2f %-24s pel %+.2f %+.2f %+.2f fast %-14s %4.0f | %-40s | scale %.2f | passes %d unmet %d' % (
                t, str(sprung.doing(g))[:24], d.qpos[0], d.qpos[1], d.qpos[2], m.joint(m.dof_jntid[6 + j]).name,
                v[j], top, float(g.get('got', {}).get('scale', math.nan)), seen['calls'], seen['unmet']), flush=True)
            seen.update(calls=0, unmet=0)
        return real(name, wobble=wobble, laid=traced, texts=texts)
    scenarios.ran = ran
    r = chain_bounded.ran(args.name, args.variant, 1e-3 * args.wobble)
    print(chain_bounded.line(args.name, args.variant, args.wobble, r), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
