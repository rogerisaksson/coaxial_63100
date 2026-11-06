"""Bench rows with her chain's plan bounded by her drives' clamps or not, over turned starts.

The python twin stands in for wbc_chain.c in every run, so a row's lines differ by the variant alone: `twin` her
plan as it is, `bounded` held within her drives' clamps (`chain.step(bounded=True)`), `held` one past its box not
let slide (`released=False`), `bounded-held` both. One start is not a measure: her leg over the bed's edge is
chaotic (the twin and the C apart by last bits rolled 61.5 deg against 6.2), so each runs over turned starts.

    python -X utf8 tools/sim/chain_bounded.py bed_rise bed_leg --variants twin bounded-held --wobbles 0 4 -4 8 -8
"""
import argparse
import statistics
import sys

#: Each variant's (bounded, released, stopped, compliant): `stopped` her joints' stops on her plan (`chain.stop_rows`),
#: `compliant` what she holds giving (`chain.gives_of`): her skin's, `twin`'s - `rigid` none, as it was; `compliant`
#: all at the foam's `regime.soft`; `material` as her world has what bears it.
VARIANTS = {'twin': (False, True, False, 'sensed'), 'rigid': (False, True, False, False),
            'bounded': (True, True, False, 'sensed'), 'held': (False, False, False, 'sensed'),
            'bounded-held': (True, False, False, 'sensed'), 'stopped': (False, True, True, 'sensed'),
            'bounded-stopped': (True, True, True, 'sensed'), 'compliant': (False, True, False, True),
            'material': (False, True, False, 'material')}


def ran(name, variant, wobble):
    """`scenarios.ran` of row `name`, laid turned `wobble` rad, the python twin's closed-chain step as `variant`; its
    `match` how far her step's model of her is from her world over its closed-chain passes: `support` what bears her
    as felt against what her step counted on, summed over her contacts, of her weight (mean), `acc` her step's
    accelerations against her world's, deg/s^2 (each pass's mean over her driven joints, their median)."""
    import math
    import numpy as np
    from machine.law import modal
    from machine.stack import chain
    from tools.cores import step as glue
    from tools.sim import scenarios
    regime, flags, support, acc = glue.regime(), VARIANTS[variant], [], []

    def laid(s):
        g = s['g']
        if not g.get('_twin'):
            g['_twin'] = True
            g['solves'] = (g['solves'][0], lambda b, sc, ask: chain.step(
                b, sc, dict(ask, _tc=g.get('_tc')) if flags[3] == 'material' else ask, regime, *flags))
        if flags[3] == 'material':
            # what each body of hers touches, its time constant (solref's), read off her world for `chain.gives_of`
            wm, wd, tc = g['world'].model, g['world'].data, {}
            for j in range(wd.ncon):
                con = wd.contact[j]
                for mine, other in ((con.geom1, con.geom2), (con.geom2, con.geom1)):
                    nm = wm.body(int(wm.geom_bodyid[mine])).name
                    tc[nm] = max(tc.get(nm, 0.0), float(wm.geom_solref[other][0]))
            g['_tc'] = tc
        if g['n'] % g['every'] != 1 or not g.get('ask') or 'borne' not in g.get('got', {}) or not chain.chained(g['ask']):
            return
        contacts, z = g['s'].get('contacts', ()), np.asarray(g['got']['borne'], float).reshape(-1, 3)
        if len(contacts) and len(z) >= len(contacts):
            felt = np.array([float(c['f'][1]) for c in contacts])
            support.append(float(np.abs(felt - z[:len(contacts), 1]).sum()) / float(g['fr'][modal.WEIGHT]))
        act = g['body'].act
        acc.append(math.degrees(float(np.abs(np.asarray(g['got']['qacc'])[act] - g['world'].data.qacc[act]).mean())))
    out = scenarios.ran(name, wobble=wobble, laid=laid)
    out['match'] = {'support': float(np.mean(support)) if support else math.nan,
                    'acc': float(np.median(acc)) if acc else math.nan, 'passes': len(acc)}
    return out


def line(name, variant, mrad, r):
    """One run's line: stood or fell, met, what bore her how long, her pelvis's and head's jerk, her step's model off
    her world (`ran`'s `match`)."""
    bore = ' '.join('%s %.1f' % kv for kv in sorted(r.get('bore', {}).items()))
    match = r.get('match', {})
    return '%-12s %-17s %+4.0f mrad stood %-5s fell %-6s met %-5s bore %-22s jerk %5.0f %5.0f off %.2f %4.0f' % (
        name, variant, mrad, r['stood'], '-' if r['fell_at'] is None else '%.2f' % r['fell_at'], r['met'], bore,
        r['jerk']['pelvis'], r['jerk']['head'], match.get('support', float('nan')), match.get('acc', float('nan')))


def summed(name, variant, runs):
    """A variant's runs of a row summed: met, fell, the floor's and the jerk's medians, its model off her world's."""
    floor = [r.get('bore', {}).get('floor', 0.0) for r in runs]
    off = [r.get('match', {}).get('support', float('nan')) for r in runs]
    acc = [r.get('match', {}).get('acc', float('nan')) for r in runs]
    return ('%-12s %-17s met %d of %d, fell %d, floor median %.1f s, jerk pelvis median %.0f, support off median %.2f, '
            'acc off median %.0f' % (name, variant, sum(bool(r['met']) for r in runs), len(runs),
                                     sum(r['fell_at'] is not None for r in runs), statistics.median(floor),
                                     statistics.median(r['jerk']['pelvis'] for r in runs), statistics.median(off),
                                     statistics.median(acc)))


def relayed(names, variants, wobbles):
    """{(name, variant): [runs]}, each run its own process on the relay's batons (`focus.relay`), a line as it ends."""
    import json
    from tools.dev import focus
    jobs = [focus.Job('%s %s %g' % (n, v, w), [sys.executable, '-X', 'utf8', __file__, n, '--variants', v, '--wobbles', '%g' % w,
                                               '--json'], timeout=2400.0) for n in names for v in variants for w in wobbles]
    got = {}
    for job, text, _code, _seconds in focus.relay(jobs):
        n, v, w = job.name.split()
        last = next((ln for ln in reversed((text or '').splitlines()) if ln.startswith('{')), None)
        if last is None:
            print('%s: %s' % (job.name, (text or '').strip()[-300:]), flush=True)
            continue
        r = json.loads(last)
        got.setdefault((n, v), []).append(r)
        print(line(n, v, float(w), r), flush=True)
    return got


def main(argv=None):
    import json
    p = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    p.add_argument('names', nargs='+', help='bench rows (tools/sim/scenarios.py)')
    p.add_argument('--variants', nargs='+', default=['twin', 'bounded-held'], choices=sorted(VARIANTS))
    p.add_argument('--wobbles', nargs='+', type=float, default=[0.0], help='starts laid turned, mrad')
    p.add_argument('--relay', action='store_true', help='each run its own process on the relay')
    p.add_argument('--json', action='store_true', help="each run's line as JSON")
    args = p.parse_args(argv)
    if args.relay:
        got = relayed(args.names, args.variants, args.wobbles)
        for (name, variant), runs in sorted(got.items()):
            print(summed(name, variant, runs), flush=True)
        return 0
    for name in args.names:
        for variant in args.variants:
            runs = [ran(name, variant, 1e-3 * mrad) for mrad in args.wobbles]
            for mrad, r in zip(args.wobbles, runs):
                print(json.dumps(r, default=float) if args.json else line(name, variant, mrad, r), flush=True)
            if not args.json:
                print(summed(name, variant, runs), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
