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

#: Each variant's (bounded, released).
VARIANTS = {'twin': (False, True), 'bounded': (True, True), 'held': (False, False), 'bounded-held': (True, False)}


def ran(name, variant, wobble):
    """`scenarios.ran` of row `name`, laid turned `wobble` rad, the python twin's closed-chain step as `variant`."""
    from machine.stack import chain
    from tools.cores import step as glue
    from tools.sim import scenarios
    regime, (bounded, released) = glue.regime(), VARIANTS[variant]

    def laid(s):
        g = s['g']
        if not g.get('_twin'):
            g['_twin'] = True
            g['solves'] = (g['solves'][0], lambda b, sc, ask: chain.step(b, sc, ask, regime, bounded, released))
    return scenarios.ran(name, wobble=wobble, laid=laid)


def line(name, variant, mrad, r):
    """One run's line: stood or fell, met, what bore her how long, her pelvis's and head's jerk."""
    bore = ' '.join('%s %.1f' % kv for kv in sorted(r.get('bore', {}).items()))
    return '%-12s %-13s %+4.0f mrad stood %-5s fell %-6s met %-5s bore %-22s jerk %5.0f %5.0f' % (
        name, variant, mrad, r['stood'], '-' if r['fell_at'] is None else '%.2f' % r['fell_at'], r['met'], bore,
        r['jerk']['pelvis'], r['jerk']['head'])


def main(argv=None):
    p = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    p.add_argument('names', nargs='+', help='bench rows (tools/sim/scenarios.py)')
    p.add_argument('--variants', nargs='+', default=['twin', 'bounded-held'], choices=sorted(VARIANTS))
    p.add_argument('--wobbles', nargs='+', type=float, default=[0.0], help='starts laid turned, mrad')
    args = p.parse_args(argv)
    for name in args.names:
        for variant in args.variants:
            runs = [ran(name, variant, 1e-3 * mrad) for mrad in args.wobbles]
            for mrad, r in zip(args.wobbles, runs):
                print(line(name, variant, mrad, r), flush=True)
            floor = [r.get('bore', {}).get('floor', 0.0) for r in runs]
            print('%-12s %-13s met %d of %d, fell %d, floor median %.1f s, jerk pelvis median %.0f' % (
                name, variant, sum(bool(r['met']) for r in runs), len(runs), sum(r['fell_at'] is not None for r in runs),
                statistics.median(floor), statistics.median(r['jerk']['pelvis'] for r in runs)), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
