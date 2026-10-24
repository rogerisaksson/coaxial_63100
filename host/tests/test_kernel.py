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


def test_the_barrier_holds_her_capture_point(report):
    """`barrier.barred` over a 0.2 m square: her capture point 0.05 m in of an edge moving out at
    0.5 m/s, an ask outward cut to the edge's h' + gamma h = 0; past the edge none further out; at
    rest 0 the ask as given."""
    import numpy as np
    from machine import barrier
    corners = np.array([[x, 0.0, z] for x in (-0.1, 0.1) for z in (-0.1, 0.1)])
    om, v = 3.3, np.array([0.5, 0.0, 0.0])
    com = np.array([0.1 - 0.05 - v[0] / om, 0.9, 0.0])
    got = barrier.barred(np.array([5.0, 0.0, 0.0]), com, v, om, corners, 1.0)
    h = 0.05 - barrier.MARGIN_M
    held = -(v[0] + got[0] / om) + barrier.GAMMA * h
    past = barrier.barred(np.array([5.0, 0.0, 0.0]), com + np.array([0.1, 0.0, 0.0]), v, om, corners, 1.0)
    free = barrier.barred(np.array([5.0, 0.0, 0.0]), com, v, om, corners, 0.0)
    report.check("inside h' + gamma h %.2g; past it xi' %.2g; at rest 0 %s" % (held, v[0] + past[0] / om, free),
                 abs(held) < 1e-9 and abs(v[0] + past[0] / om) < 1e-9 and free[0] == 5.0, '')


def test_a_pulled_chair_weighs_nothing(report):
    """The chair pulled 1 m back over 0.3 s from under her sit while her pelvis descends over it
    (1.5 s): what bears her takes no seat till her rise is said (8 s), and she stands."""
    import numpy as np
    from machine import latent
    from tools.sim import scenarios
    seat, x0 = [0.0], [None]

    def laid(s):
        g, w = s['g'], s['g']['world']
        k = w.model.body_mocapid[w.model.body('chair').id]
        x0[0] = w.data.mocap_pos[k].copy() if x0[0] is None else x0[0]
        u = min(1.0, max(0.0, (g['t'] - 1.5) / 0.3))
        w.data.mocap_pos[k] = x0[0] - np.array([0.0, 0.0, u * u * (3.0 - 2.0 * u)])
        seat[0] = max(seat[0], float(g['out'][latent.MANIFOLDED][1]) * (g['t'] < 8.0))

    out = scenarios.ran('chair', laid=laid)
    report.check('the seat taken at most %.3f, stood %s' % (seat[0], out['stood']), seat[0] == 0.0 and out['stood'], '')


def test_her_link_under_its_ceiling(report):
    """Her DC link (`machine.dclink`) through a hop down off a chair: its landing's braking bled on her
    boards' d axes past the text's onset, the link under its clamp; at rest her packs' volts."""
    from machine import dclink, drives
    from tools.sim import scenarios
    seen = {}
    out = scenarios.ran('hop_down', laid=lambda s: seen.update(link=s['g']['world'].link))
    link = seen['link']
    report.check('peak %.2f V of %.1f, %.1f J bled, stood %s' % (link['peak'], dclink.CLAMP_V, link['bled'], out['stood']),
                 dclink.ONSET_V < link['peak'] < dclink.CLAMP_V and link['bled'] > 0.0 and out['stood']
                 and dclink.laid()['v'] == drives.PACK_V, '')


def test_the_field_seeks_a_seat_not_found(report):
    """`field.step` on a table whose seat's contact row waits: her rows' height landed, it seeks down
    at the text's 0.30 m/s through 43 ms; still descending, nothing; the seat bearing her, it stops
    and holds; her rows leaning on none, it lets go."""
    import numpy as np
    from machine import field, impedance, latent, modal
    tab = np.zeros((1, latent.TABLE))
    tab[0, latent.CONTACT] = 1.0
    y, out, dt = np.zeros(modal.SENSED), np.zeros(latent.OUT), 0.003
    f, moving = field.laid(), field.laid()
    for k in range(200):
        tab[0, latent.AT] = k * dt
        field.step(f, tab, y, 0.85, 0.0, k * dt, dt, out.copy())
        field.step(moving, tab, y, 0.85, 1.0, k * dt, dt, out.copy())
    sought = f['offset'][0] * field._MODES['mode_axial_compression'][0]
    y[modal.PROP] = impedance.PROP_ON
    held = [field.step(f, tab, y, 0.85, 0.0, 0.6 + k * dt, dt, out.copy())['offset'][0] for k in range(100)]
    gone = [field.step(f, tab, y, 0.0, 0.0, 0.9 + k * dt, dt, out.copy())['offset'][0] for k in range(100)]
    report.check('sought %.3f m in 0.6 s (0.30 m/s less its lag), none descending, held %.3g, let go %.4f'
                 % (sought, held[-1] - held[-10], gone[-1]),
                 abs(sought - 0.30 * (0.6 - 1.0 / field.RATE)) < 0.01 and moving['offset'][0] == 0.0
                 and abs(held[-1] - held[-10]) < 1e-3 * held[0] and gone[-1] < 1e-3 * held[0], '')


def test_the_invariant_ekf_drifts_less(report):
    """Her walk (`tools/sim/drifted.py`): the right-invariant EKF's drift along the floor and in height
    and its velocity's error each under her estimate's."""
    from tools.sim import drifted
    line = drifted.one('play_walk')
    e, i = line['estimate'], line['inekf']
    report.check('walked %.2f m: floor %.3f / %.3f, height %+.3f / %+.3f, speed %.3f / %.3f m/s' % (
        line['went'], i['floor'], e['floor'], i['height'], e['height'], i['speed'], e['speed']),
        i['floor'] < e['floor'] and abs(i['height']) < abs(e['height']) and i['speed'] < e['speed'], '')


def test_the_lowering_lays_the_texts_abi(report):
    """The room's schedule lowered (`machine.lowering`): C lays every section where Python does, the
    text's offsets held by wbc_soa.h; each bank's fields as its gait says them in SI over her
    nominal's, a field past its bounds clamped; each node's biquad the text's, seeded on her value."""
    import numpy as np
    from machine import dsp, kernel, lowering as L, rcu
    lib, got = rcu._core(), kernel.read()
    at = np.zeros(len(L.SECTIONS) + 1, np.int64)
    lib.wbh_soa_offsets(at.ctypes.data)
    report.check("C lays %d sections where Python does, %d bytes, the text's fields at its offsets" % (len(L.SECTIONS), L.SIZE),
                 at.tolist() == [a for _n, a, _c in L.SECTIONS] + [L.SIZE]
                 and [a for k, a, *_r in L.SECTIONS[:len(L.FIELDS)]] == [a for k, a, *_r in L.FIELDS], str(at.tolist()))
    banks = [b for _seg, b in L.scheduled(got['compile-complete-humanoid-soa']) if b is not None]
    room, stairs = banks[0], banks[-1]
    fast = L.lowered(dsp.value(['->', 'nominal-biped-gait', ['quantize-grid', ':bpm', 600.0]], got))
    report.check('four banks; the room 110 BPM as 1.833 Hz, its sole 32 mm, its track her nominal 103 mm where '
                 'her world says it, shins 32 mm; the stairs 1.5 Hz, 15 /s and 16 mm, thorax 0.06 rad; 600 BPM at 4 Hz',
                 len(banks) == 4 and abs(L.field(room, 'bpm') - 110.0 / 60.0) < 1e-6
                 and abs(L.field(room, 'sole-stack-z-mm') - 0.032) < 1e-7 and abs(L.field(room, 'track-width-mm') - 0.103) < 1e-7
                 and abs(L.field(room, 'min-shin-clearance-mm') - 0.032) < 1e-7
                 and abs(L.field(stairs, 'bpm') - 1.5) < 1e-6 and L.field(stairs, 'barrier-gamma') == 15.0
                 and abs(L.field(stairs, 'min-capture-margin-mm') - 0.016) < 1e-7
                 and abs(L.field(stairs, 'dc-offset-rad', 'c7-t12-thoracic') - 0.06) < 1e-7
                 and abs(L.field(stairs, 'dc-offset-rad', 'l4-l5-lumbar') + 0.14) < 1e-7
                 and L.field(fast, 'bpm') == 4.0 and L.field(room, 'weight') == 1.0
                 and abs(L.field(room, 'r0') - 0.048) < 1e-7, '')
    node, x0 = 'c7-t12-thoracic', 0.3
    b = L.coefficients(':lowpass', 8.0, L.Q)
    k = L.NODES.index(node)
    bank = L.lowered(got['nominal-biped-gait'], {node: x0})
    held = [L.biquad(bank, node, x0) for _ in range(1000)]
    spread = max(abs(y - held[0]) for y in held)
    report.check("the thorax's 8 Hz low-pass the text's at %g Hz; seeded on %.1f its first output %.6f (its float32 "
                 "DC gain), its rest within %.1e over a second" % (L.RATE, x0, held[0], spread),
                 [float(bank[L.AT[c] + k]) for c in L.BIQUAD[:5]] == [float(np.float32(v)) for v in b]
                 and abs(held[0] - x0) < 1e-4 * x0 and spread < 1e-5 * x0,
                 '%.9f %.9f' % (min(held), max(held)))


def test_a_bank_swaps_with_no_step(report):
    """A bank published over her link (`machine.rcu`) and taken by her loop's C side: its biquad in C
    bit for bit Python's; her thorax's filter settled on one bank, the next lowered on her value goes
    on with no step where an unseeded one drops."""
    import numpy as np
    from machine import kernel, lowering as L, rcu
    got, node = kernel.read(), 'c7-t12-thoracic'
    rng = np.random.default_rng(7)
    x = (0.3 + 0.05 * rng.standard_normal(500)).astype(np.float32)
    a = L.lowered(got['nominal-biped-gait'], {node: float(x[0])})
    link = rcu.Link.made(((4, 2),), 1, L.SIZE // 8)
    her = rcu.Link.found(link.name)
    sent = link.published(L.words(a))
    taken = her.taken()
    y = np.zeros(len(x), np.float32)
    if taken is not None:
        rcu._core().wbh_soa_biquad(taken[1].ctypes.data, L.NODES.index(node), x.ctypes.data, y.ctypes.data, len(x))
    her.closed()
    link.closed()
    mine = a.copy()
    ours = np.array([L.biquad(mine, node, v) for v in x], np.float32)
    report.check("the bank over the link bit for bit, its C biquad Python's over %d passes" % len(x),
                 sent and taken is not None and L.words(a).tobytes() == taken[1].tobytes() and y.tobytes() == ours.tobytes(),
                 '%g' % float(np.abs(y - ours).max()))
    run = L.lowered(got['nominal-biped-gait'], {node: 0.3})
    last = [L.biquad(run, node, 0.3) for _ in range(2000)][-1]
    seeded = L.lowered(got['nominal-biped-gait'], {node: 0.3})
    bare = L.lowered(got['nominal-biped-gait'])
    steps = (abs(L.biquad(seeded, node, 0.3) - last), abs(L.biquad(bare, node, 0.3) - last))
    report.check('swapped settled at 0.3: seeded, a step of %.1e; unseeded %.3f' % steps,
                 steps[0] < 1e-6 and steps[1] > 0.29, '')


ROSTER = (test_the_lowering_lays_the_texts_abi, test_a_bank_swaps_with_no_step, test_the_invariant_ekf_drifts_less, test_the_field_seeks_a_seat_not_found, test_her_link_under_its_ceiling, test_the_barrier_holds_her_capture_point, test_a_pulled_chair_weighs_nothing, test_her_patches_observed_walking, test_her_patches_observed_seated, test_her_chest_yaw_leaks_nothing_standing,
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
