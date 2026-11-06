"""A bench row traced through her chain's plan: joints, contacts, the way chosen, the bounds.

The python twin stands in for wbc_chain.c as `chain_bounded.py` runs it (a variant, a turned start); `chain_plan.within`
is watched where it is called: each pass's bounded solves, how many found their bounds over the unbounded plan, and
how many could not meet them (a held set unsolvable or its rounds out: the plan as unbounded).

    python -X utf8 tools/sim/chain_trace.py bed_leg --variant bounded --wobble 8 --since 5 --until 11
"""
import argparse
import math
import sys


def main(argv=None):
    import numpy as np
    from machine.process import sprung
    from machine.stack import chain, chain_plan
    from machine.stack.qp import TOL
    from tools.sim import chain_bounded
    p = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    p.add_argument('name', help='a bench row (tools/sim/scenarios.py)')
    p.add_argument('--variant', default='bounded', choices=sorted(chain_bounded.VARIANTS))
    p.add_argument('--wobble', type=float, default=0.0, help='the start laid turned, mrad')
    p.add_argument('--since', type=float, default=0.0, help='s')
    p.add_argument('--until', type=float, default=math.inf, help='s')
    p.add_argument('--every', type=float, default=0.25, help='s between lines')
    args = p.parse_args(argv)
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
            t = g['t']
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
            ask = g.get('ask', {})
            # the way chosen (chain.chained): the closed chains' step, or the upright's, and what chose it
            print('       way %s: swing %s stance %s let_go %s reclined %.1f' % (
                'chain' if chain.chained(ask) else 'upright', sorted(ask.get('swing', {})), ask.get('stance'),
                ask.get('let_go'), float(ask.get('reclined', 0.0))), flush=True)
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
