"""A bench row traced through her chain's plan: joints, contacts, the way chosen, the bounds.

The python twin stands in for wbc_chain.c as `chain_bounded.py` runs it (a variant, a turned start); `chain_plan.within`
is watched where it is called: each pass's bounded solves, how many found their bounds over the unbounded plan, and
how many could not meet them (a held set unsolvable or its rounds out: the plan as unbounded).

    python -X utf8 tools/sim/chain_trace.py bed_leg --variant bounded --wobble 8 --since 5 --until 11
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


def replayed(g, regime, flags):
    """Her last closed-chain pass replayed on its own inputs (`g['_chain_in']`, as her step was called), her plan
    after 1 .. `regime.rounds` rounds (`chain_plan.planned`): each body as she feels it bear her, as her plan holds it
    round by round and as her step's Gauss-Seidel gives it, N up; the replay's torques against the pass's."""
    import numpy as np
    from machine.stack import chain
    b, sc, ask = g['_chain_in']
    m, real, load, plans, got = g['world'].model, chain.planned, np.array(b.load, copy=True), [], {}
    try:
        for k in range(1, regime.rounds + 1):
            def planned(*a, **kw):
                plans.append(real(*a, **kw))
                return plans[-1]
            chain.planned = planned
            got = chain.step(b, sc, ask, regime._replace(rounds=k), *flags)
    finally:
        chain.planned, b.load = real, load
    contacts = sc.get('contacts', ())
    names = [m.body(int(c['body'])).name if int(c.get('body', -1)) >= 0 else 'sole%d' % (-1 - int(c['body'])) for c in contacts]
    felt = {n: sum(float(c['f'][1]) for c, n2 in zip(contacts, names) if n2 == n) for n in set(names)}
    # each point's speed along the force it feels, m/s (+ out of what bears it), her bodies' most
    Q = chain.frames(np.concatenate([np.asarray(c['f'], float) for c in contacts])) if len(contacts) else np.zeros((0, 3, 3))
    vn = [float(Q[i, 0] @ (np.asarray(c['J'], float) @ sc['v'])) for i, c in enumerate(contacts)]
    for nm in sorted(felt, key=lambda n: -felt[n])[:6]:
        idx = [i for i, n in enumerate(names) if n == nm]
        rounds = ' '.join('%.0f' % sum(float(p[2][3 * i + 1]) for i in idx) for p in plans)
        print('       replay %-16s felt %5.0f plan by round %-18s step %5.0f v_n %+.2f' % (
            nm, felt[nm], rounds, sum(float(got['borne'][i][1]) for i in idx), max((vn[i] for i in idx), key=abs)), flush=True)
    print("       replay's torques off the pass's %.2g N m" % float(np.abs(np.asarray(got['tau']) - np.asarray(g['got']['tau'])).max()),
          flush=True)
    # her plan's first round (all held) with each of its asks let go: her posture, her turns, both - whose rows ask
    # what of her contacts, N up (the most felt three)
    top = sorted(felt, key=lambda n: -felt[n])[:3]
    for what, kw in (('no posture', {'posture': 0.0}), ('no turns', {'turn': 0.0}), ('neither', {'posture': 0.0, 'turn': 0.0})):
        plans.clear()
        try:
            chain.planned = planned
            chain.step(b, sc, ask, regime._replace(rounds=1, **kw), *flags)
        finally:
            chain.planned, b.load = real, load
        print('       replay round 1 %-10s ' % what + ' '.join('%s %.0f' % (nm, sum(float(plans[-1][2][3 * i + 1]) for i, n in enumerate(names)
                                                                              if n == nm)) for nm in top), flush=True)


def main(argv=None):
    import numpy as np
    from machine.process import sprung
    from machine.stack import chain, chain_plan
    from tools.cores import step as glue
    from machine.stack.qp import TOL
    from tools.sim import chain_bounded
    p = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    p.add_argument('name', help='a bench row (tools/sim/scenarios.py)')
    p.add_argument('--variant', default='bounded', choices=sorted(chain_bounded.VARIANTS))
    p.add_argument('--wobble', type=float, default=0.0, help='the start laid turned, mrad')
    p.add_argument('--since', type=float, default=0.0, help='s')
    p.add_argument('--until', type=float, default=math.inf, help='s')
    p.add_argument('--every', type=float, default=0.25, help='s between lines')
    p.add_argument('--rounds', action='store_true', help="each line's closed-chain pass replayed round by round")
    args = p.parse_args(argv)
    regime, flags = glue.regime(), chain_bounded.VARIANTS[args.variant]
    seen, last, unbound = {'calls': 0, 'over': 0, 'unmet': 0, 'worst': 0.0}, [-math.inf], chain_plan.within

    def within(H, g, A, b, x, S, s, w):
        out = unbound(H, g, A, b, x, S, s, w)
        seen['calls'] += 1
        if len(b) and float(np.max(A @ x - b)) > TOL:
            seen['over'] += 1
            miss = float(np.max(A @ out - b))
            seen['worst'] = max(seen['worst'], miss)
            seen['unmet'] += int(miss > TOL)
        return out
    chain_plan.within = within
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
            if t < args.since or t > args.until or t - last[0] < args.every:
                return
            last[0] = t
            m, d = g['world'].model, g['world'].data
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
            print('%6.2f %-24s pel %+.2f %+.2f %+.2f fast %-14s %4.0f | %-40s | scale %.2f | bounds %d over %d unmet %d '
                  'worst %.0f' % (t, str(sprung.doing(g))[:24], d.qpos[0], d.qpos[1], d.qpos[2], m.joint(m.dof_jntid[6 + j]).name,
                                  v[j], top, float(g.get('got', {}).get('scale', math.nan)), seen['calls'], seen['over'],
                                  seen['unmet'], seen['worst']), flush=True)
            seen.update(calls=0, over=0, unmet=0, worst=0.0)
        return real(name, wobble=wobble, laid=traced, texts=texts)
    scenarios.ran = ran
    r = chain_bounded.ran(args.name, args.variant, 1e-3 * args.wobble)
    print(chain_bounded.line(args.name, args.variant, args.wobble, r), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
