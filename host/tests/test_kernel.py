"""The dual-domain kernel on her loop (docs/todo/protocol.md): her momentum observer against her skin."""
import sys

from tools.dev.focus import chosen
from gynoid_kit import Report


def _engage():
    """The text's contact comparator's engaging share of her weight (`latch-hysteretic-contact`)."""
    from machine import dsp, kernel
    got = kernel.read()
    return dsp.value(['latch-hysteretic-contact', 'nominal-biped-gait'], got)['contact-comparators']['schmitt-high-engage-fz']


def _observed(scene, since, until):
    """(observed, felt) each law pass of `scene` from `since` to `until` s, (n, MANIFOLDS); her weight, N."""
    import numpy as np
    from tools.sim import observed, scenarios
    rows, last, weight = [], [None], [0.0]

    def laid(s):
        g = s['g']
        weight[0] = g['body'].mass * 9.81
        if s['fallen_at'] is None and g.get('t_law') != last[0] and since <= g['t'] <= until:
            last[0] = g['t_law']
            rows.append((g['observer']['normal'].copy(), observed.felt(g['world'], g['sense'])))

    scenarios.ran(scene, laid=laid)
    return np.array([r[0] for r in rows]), np.array([r[1] for r in rows]), weight[0]


def test_her_patches_observed_walking(report):
    """Walking past her first stride, the observer reads her soles' push within the text's engaging
    share of her weight at its 95th percentile, and nowhere her skin feels nothing past it."""
    import numpy as np
    from machine import observer
    from tools.sim import observed
    got, felt, weight = _observed('play_walk', 1.5, 6.0)
    share, sole = _engage() * weight, observer.MANIFOLDS.index('plantar-bilateral')
    ghost = max(float(got[felt[:, k] < observed.FELT_N, k].max(initial=0.0)) for k in range(len(observer.MANIFOLDS)))
    p95 = float(np.percentile(np.abs(got[:, sole] - felt[:, sole]), 95))
    report.check('her soles within %.0f N at the 95th percentile (%.1f), nothing unfelt past it (%.1f), %d passes'
                 % (share, p95, ghost, len(got)), p95 < share and ghost < share and len(got) > 1000, '')


def test_her_patches_observed_seated(report):
    """Sat on the chair: her soles and the seat each within the text's engaging share of her weight of
    what her skin feels, at the 95th percentile."""
    import numpy as np
    from machine import observer
    got, felt, weight = _observed('chair', 4.0, 7.5)
    share = _engage() * weight
    worst = {m: float(np.percentile(np.abs(got[:, k] - felt[:, k]), 95)) for k, m in enumerate(observer.MANIFOLDS)
             if m in ('plantar-bilateral', 'ischial-bilateral')}
    report.check('her soles and the seat within %.0f N of her skin at the 95th percentile: %s' % (share, worst),
                 all(v < share for v in worst.values()) and len(got) > 1000, '')


ROSTER = (test_her_patches_observed_walking, test_her_patches_observed_seated)


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
