"""The dual-domain kernel on her loop (docs/todo/protocol.md): her momentum observer against her skin."""
import sys

from tools.dev.focus import chosen
from gynoid_kit import Report, near


def _engage():
    """The text's contact comparator's engaging share of her weight (`latch-hysteretic-contact`)."""
    from machine.texts import dsp, kernel
    got = kernel.read()
    return dsp.value(['latch-hysteretic-contact', 'nominal-biped-gait'], got)['contact-comparators']['schmitt-high-engage-fz']


def _observed(scene, since, until):
    """(observed, felt) each law pass of `scene` from `since` to `until` s, (n, MANIFOLDS); her weight, N."""
    import numpy as np
    from machine.senses import observer
    from tools.sim import scenarios
    rows, last, weight = [], [None], [0.0]

    def laid(s):
        g = s['g']
        weight[0] = g['body'].mass * 9.81
        if s['fallen_at'] is None and g.get('t_law') != last[0] and since <= g['t'] <= until:
            last[0] = g['t_law']
            rows.append((g['observer']['normal'].copy(), observer.skin(g['world'], g['sense'])))

    scenarios.ran(scene, laid=laid)
    return np.array([r[0] for r in rows]), np.array([r[1] for r in rows]), weight[0]


def test_her_patches_observed_walking(report):
    """Walking past her first stride, the observer reads her soles' push within the text's engaging
    share of her weight at its 95th percentile, and nowhere her skin feels nothing past it."""
    import numpy as np
    from machine.senses import observer
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
    from machine.senses import observer
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
    from machine.process import worker
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
    bit, and unsaid (`observer.said`: shares of what is told, her soles alone where nothing is); a told
    seat unfelt weighs nothing, its share her soles'; felt past the engaging share, as told after 1 /
    UP s; let go, nothing within 1 / DOWN s of its release's dwell."""
    import numpy as np
    from machine.senses import observer
    obs = {k: np.zeros(len(observer.ROWS)) for k in ('latched', 'below', 'since', 'gate')}
    obs['normal'] = np.zeros(len(observer.MANIFOLDS))
    weight, dt, seat = 300.0, 0.003, int(observer.ROWS[1])
    sole, n = observer.SOLES, len(observer.SOLES)
    soles = observer.weighed(obs, sole, weight, dt)
    unsaid = observer.weighed(obs, np.zeros(n), weight, dt)
    told = 0.15 * np.eye(n)[0] + 0.85 * np.eye(n)[1]
    unfelt = observer.weighed(obs, told, weight, dt)
    obs['normal'][seat] = 0.5 * weight
    felt = [observer.weighed(obs, told, weight, dt) for _k in range(int(1.0 / observer.UP / dt) + 1)][-1]
    obs['normal'][seat] = 0.0
    gone = [observer.weighed(obs, told, weight, dt)
            for _k in range(int((observer.DWELL_S + 1.0 / observer.DOWN) / dt) + 2)][-1]
    said = observer.said(2.0 * told)
    report.check('soles as told %s, unsaid %s; unfelt %s; felt %s; let go %s; said %s of twice told' % (
        soles, unsaid, unfelt, felt.round(3), gone.round(3), said),
                 soles.tolist() == sole.tolist() and unsaid.tolist() == sole.tolist()
                 and unfelt.tolist() == sole.tolist() and np.allclose(said, told)
                 and observer.said(np.zeros(n)).tolist() == sole.tolist()
                 and np.allclose(felt, told) and np.allclose(gone, sole), '')


def test_the_barrier_holds_her_capture_point(report):
    """`barrier.barred` over a 0.2 m square: her capture point 0.05 m in of an edge moving out at
    0.5 m/s, an ask outward cut to the edge's h' + gamma h = 0; past the edge none further out; at
    rest 0 the ask as given."""
    import numpy as np
    from machine.law import barrier
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
    from machine.law import latent
    from tools.sim import scenarios
    seat, x0 = [0.0], [None]

    def laid(s):
        g, w = s['g'], s['g']['world']
        k = w.model.body_mocapid[w.model.body('chair').id]
        x0[0] = w.data.mocap_pos[k].copy() if x0[0] is None else x0[0]
        u = min(1.0, max(0.0, (g['t'] - 1.5) / 0.3))
        w.data.mocap_pos[k] = x0[0] - np.array([0.0, 0.0, u * u * (3.0 - 2.0 * u)])
        seat[0] = max(seat[0], float(g['weighed_w'][1]) * (g['t'] < 8.0))

    out = scenarios.ran('chair', laid=laid)
    report.check('the seat taken at most %.3f, stood %s' % (seat[0], out['stood']), seat[0] == 0.0 and out['stood'], '')


def test_her_link_under_its_ceiling(report):
    """Her DC link (`machine.body.dclink`) through a hop down off a chair: its landing's braking bled on her
    boards' d axes past the text's onset, the link under its clamp; at rest her packs' volts."""
    from machine.body import dclink, drives
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
    from machine.senses import field
    from machine.law import impedance, latent, modal
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


def test_the_kernel_reads_whole(report):
    """The dual-domain kernel's texts (lisp/humanoid/kernel/) evaluated, each op its mechanism's
    node: the room's schedule; the room's walk with its bus, bands, body schema and shield; the
    stool's shove gated on its funnel, each funnel's executor run from her nominal, the sit's blind
    settle at the observer's 35 Hz, weights slewed 4.5 up and 45 down, the barrier at 14 /s and 18
    mm, its slide; the stairs; the lowering's own functions evaluated as written."""
    import math
    from machine.texts import dsp
    got = dsp.read(dsp.KERNEL)
    plan = dsp.schedule(got['compile-complete-humanoid-soa'])
    report.check('the room schedule: 8 bars of the room, the stool, 4 s, the bed, 8 s, the stairs',
                 [p[0] for p in plan] == [0.0, 'oneshot', 'dwell', 'oneshot', 'dwell', 'oneshot']
                 and plan[0][1] == 8.0 and (plan[2][1], plan[4][1]) == (4.0, 8.0), str([p[:2] for p in plan]))
    room = plan[0][2]
    omega = 2.0 * 3.14159265 * 11.0
    report.check('the room: its bus 48 / 56.5 / 60 V, bands cut at 260 and 980 mm, the shield at 11 Hz, 1.05, '
                 'the cocoon 14 mm, shins 32 mm clear, the margin probed at 15-38 Hz',
                 (room['dc-bus']['nominal-voltage-v'], room['dc-bus']['bleed-threshold-v'], room['dc-bus']['max-voltage-v'])
                 == (48.0, 56.5, 60.0)
                 and [s.args['z-window-mm'] for s in room['sdf-bands']['slabs']] == [(0.0, 260.0), (260.0, 980.0), (980.0, 1920.0)]
                 and near(room['ecbf-shield']['position-gain-kp'], omega * omega)
                 and near(room['ecbf-shield']['velocity-gain-kd'], 2.0 * 1.05 * omega)
                 and room['braking-cocoon']['base-margin-m'] == 0.014
                 and room['foothold']['crural-capsule-fk-guard']['min-shin-clearance-m'] == 0.032
                 and room['phase-margin']['probe-frequencies-hz'] == (15.0, 22.0, 30.0, 38.0), str(room['ecbf-shield']))
    schema = got['anatomical-capsule-schema']
    report.check('the body schema: 18 capsules and 4 payload slots, LSE at 25, 13 pairs; eight manifolds, the soles 0.80 mu',
                 len(schema['capsules']) == schema['memory-layout']['num-body-capsules'] == 18.0
                 and schema['memory-layout']['num-payload-slots'] == 4.0 and len(schema['self-collision-pairs']) == 13
                 and len(got['anatomical-manifold-registry']) == 8
                 and got['anatomical-manifold-registry']['plantar-bilateral']['default-mu'] == 0.80
                 and got['spatial-object-soa-registry']['memory-layout']['max-tracked-objects'] == 256.0, str(schema['memory-layout']))
    stool = plan[1][1]
    report.check('the stool: the shove gated on its funnel, 18 N on the chair, its payload spliced',
                 stool['_gates'][0][:2] == ('when-funnel-active', ':somatic-object-coupled')
                 and {'somatic-shove', 'payload-capsule'} <= set(stool['_gates'][0][2])
                 and stool['somatic-shove']['unilateral-contact-maintenance']['min-normal-bias-n'] == 18.0, str(stool['_gates']))
    runs = {(t['from'], t['to']): dsp.run(t['execute'], got['nominal-biped-gait'])
            for t in stool['funnel-executors']['transitions']}
    sit = runs[(':bipedal-upright', ':ischial-seated')]
    report.check("each executor runs; the sit's observer 35 Hz, weights 4.5 / 45 per s, barrier 14 /s 18 mm, "
                 "slid at 0.16 m/s under 0.55 of her, her leg 890 mm",
                 len(runs) == 4 and sit['momentum-observer']['filter-cutoff-hz'] == 35.0
                 and sit['manifold-weights']['asymmetric-slew'] == {'attack-rate': 4.5, 'release-rate': 45.0}
                 and (sit['dcm-barrier']['barrier-decay-rate'], sit['dcm-barrier']['safety-margin-mm']) == (14.0, 18.0)
                 and sit['coulomb-slide']['tangent-velocity-attractor'][0] == -0.16
                 and sit['coulomb-slide']['normal-load-governor']['cap-fz-ratio'] == 0.55
                 and sit['reachability']['nominal-max-length-mm'] == 890.0, str(sit['manifold-weights']))
    stairs = plan[5][1]
    brace = stairs['adjoint-cones']['opportunistic-egress-brace']
    report.check("the stairs at 90 BPM, down 175 mm 38 mm clear of the nosing; the egress's brace on what her "
                 "registry finds within 680 mm",
                 stairs['oscillator']['bpm'] == 90.0 and stairs['step-energy']['elevation-delta-mm'] == -175.0
                 and stairs['step-energy']['nosing-radial-guard-mm'] == 38.0
                 and isinstance(brace['mu-static'], dsp.Law) and brace['mu-static'].name == 'slot-estimated-mu', str(brace))
    b0, b1, b2, a1, a2 = dsp.value(['precompute-tustin-biquad', ':lowpass', 42.0, 0.70710678], got)
    s1, s2 = dsp.value(['seed-bumpless-biquad-states', b0, b1, b2, a1, a2, 0.3], got)
    report.check("the lowering's biquad as written: unity at DC, seeded on 0.3 its first output 0.3",
                 near(b0 + b1 + b2, 1.0 + a1 + a2) and math.isclose(b0 * 0.3 + s1, 0.3, rel_tol=1e-6)
                 and got['*abi-field-descriptors*'][0] == (':bpm', 0.0, 0.016666668, 0.0, 4.0), '%r' % ((b0, b1, b2, a1, a2),))


def _both(c, step, outs, got):
    """Her observer's `step` (wbc_gmo.c) on `c` scoped and on its copy whole (`whole`), in turns first, timed:
    `got` counts the law passes, those whose `outs` were apart, her working set's sizes and each one's seconds."""
    import ctypes
    import time
    from tools.cores.gmo import Gmo

    def stepped(at, base, q, u, dt):
        whole = Gmo.from_buffer_copy(c)
        whole.whole = 1
        runs = [(ctypes.byref(whole), 'whole'), (at, 'scoped')][::1 - 2 * (got['passes'] % 2)]
        for arg, key in runs:
            t = time.perf_counter()
            step(arg, base, q, u, dt)
            got[key] += time.perf_counter() - t
        got['apart'] += any(bytes(getattr(whole, k)) != bytes(getattr(c, k)) for k in outs) or whole.rounds != c.rounds
        got['passes'] += 1
        got['lives'] += c.lives
        got['listed'] += c.listed
        got['touches'] += sum(t >= 0 for t in c.touched)
    return stepped


def test_her_observer_scoped_as_whole(report):
    """Her observer's working set (working_envelope_scoper.lisp) at every law pass of her walk and her crouch -
    hinges on their stops, capsules touching -: its step scoped and its step whole from the same state the same
    bits; scoped, about half her unknowns split and a fraction of her pairs searched, within the text's envelope."""
    from machine.texts import kernel
    from tools.cores.gmo import GMO_F
    from tools.sim import scenarios
    outs = ('r', 'f', 'force', 'at', 'touched', 'past', 'prior', 'speed', 'expected')
    envelope = kernel.said('evaluate-operating-point-horizon', 'manifold-preload-distance-mm') * 1e-3
    for scene, touching in (('play_walk', False), ('crouching_down', True)):
        got, held = dict.fromkeys(('passes', 'apart', 'lives', 'listed', 'touches', 'whole', 'scoped'), 0), {}

        def laid(s, got=got, held=held):
            obs = s['g']['observer']
            if held.get('obs') is not obs:
                held['obs'], held['c'] = obs, obs['c']
                obs['step'] = _both(obs['c'], obs['step'], outs, got)

        scenarios.ran(scene, laid=laid)
        n, c = max(got['passes'], 1), held['c']
        report.check('%s, %d law passes scoped as whole bit for bit: %.0f of %d unknowns, %.0f of %d pairs within '
                     '%.2f m, %d touches; %.0f against %.0f us' % (
                         scene, got['passes'], got['lives'] / n, GMO_F, got['listed'] / n, c.pairs, c.horizon,
                         got['touches'], 1e6 * got['scoped'] / n, 1e6 * got['whole'] / n),
                     got['apart'] == 0 and got['passes'] > 600 and c.horizon == envelope > 0.0
                     and got['lives'] / n < 0.6 * GMO_F and got['listed'] / n < 0.4 * c.pairs
                     and (got['touches'] > 0) == touching, '%d passes apart' % got['apart'])


ROSTER = (test_the_invariant_ekf_drifts_less, test_the_field_seeks_a_seat_not_found, test_her_link_under_its_ceiling, test_the_barrier_holds_her_capture_point, test_a_pulled_chair_weighs_nothing, test_her_patches_observed_walking, test_her_patches_observed_seated, test_her_chest_yaw_leaks_nothing_standing,
          test_what_bears_her_weighs_as_felt, test_the_kernel_reads_whole, test_her_observer_scoped_as_whole)


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
