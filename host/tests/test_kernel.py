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


def test_her_chest_yaw_leaks_nothing_standing(report):
    """Standing, her trunk's turn asked 0.3 rad about up through her whole-body step (wbc_wrench.c):
    her chest turns, her standing soles and her centre of mass move under 1e-4 of its acceleration -
    the text's projector, `nullspace-leakage-guard` 0."""
    import copy
    import math
    import numpy as np
    from machine import worker
    from tools.cores import wbc as cores
    s = worker.session()
    g = s['g']
    while g['t'] < 2.0:
        worker.stepped(s)
    b, ask, core = g['body'], copy.deepcopy(g['ask']), cores.shared()
    qpos, qvel, n = b.d.qpos, b.d.qvel, b.n
    R = cores.rotation(qpos[3:7])
    base, u = np.concatenate([R.ravel(), qpos[:3]]), np.concatenate([qvel[3:6], R.T @ qvel[:3], qvel[6:n]])
    q = qpos[7:7 + n - 6].copy()
    w, x, y, z = ask['turns'][1]
    c, sn = math.cos(0.15), math.sin(0.15)
    asked = copy.deepcopy(ask)
    asked['turns'] = (ask['turns'][0], np.array([c * w - sn * y, c * x + sn * z, c * y + sn * w, c * z - sn * x]))
    was, now = core.wrench(base, q, u, ask), core.wrench(base, q, u, asked)
    core.pose(base, q)
    core.mass()
    core.bias(u)
    du = now['udot'] - was['udot']
    chest = float(np.abs(du[8]))
    moved = max([float(np.abs(core.com()[1] @ du).max())] + [float(np.abs(core.jacobian(link, b.sole) @ du).max())
                                                              for link in (23, 32)])
    report.check('her chest %.1f rad/s^2; her soles and centre of mass %.2g' % (chest, moved),
                 chest > 1.0 and moved < 1e-4 * chest, '')


def test_what_bears_her_weighs_as_felt(report):
    """Her rows' told weights on her comparator (`observer.weighed`): her soles alone as told to the
    bit; a told seat unfelt weighs nothing, its share her soles'; felt past the engaging share, as
    told after 1 / UP s; let go, nothing within 1 / DOWN s of its release's dwell."""
    import numpy as np
    from machine import observer
    obs = {k: np.zeros(len(observer.ROWS)) for k in ('latched', 'below', 'since', 'gate')}
    obs['normal'] = np.zeros(len(observer.MANIFOLDS))
    weight, dt, seat = 300.0, 0.003, int(observer.ROWS[1])
    soles = observer.weighed(obs, np.array([1.0, 0.0, 0.0, 0.0]), weight, dt)
    told = np.array([0.15, 0.85, 0.0, 0.0])
    unfelt = observer.weighed(obs, told, weight, dt)
    obs['normal'][seat] = 0.5 * weight
    felt = [observer.weighed(obs, told, weight, dt) for _k in range(int(1.0 / observer.UP / dt) + 1)][-1]
    obs['normal'][seat] = 0.0
    gone = [observer.weighed(obs, told, weight, dt)
            for _k in range(int((observer.DWELL_S + 1.0 / observer.DOWN) / dt) + 2)][-1]
    report.check('soles as told %s; unfelt %s; felt %s; let go %s' % (soles, unfelt, felt.round(3), gone.round(3)),
                 soles.tolist() == [1.0, 0.0, 0.0, 0.0] and unfelt.tolist() == [1.0, 0.0, 0.0, 0.0]
                 and np.allclose(felt, told) and np.allclose(gone, [1.0, 0.0, 0.0, 0.0]), '')


ROSTER = (test_her_patches_observed_walking, test_her_patches_observed_seated, test_her_chest_yaw_leaks_nothing_standing,
          test_what_bears_her_weighs_as_felt)


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
