"""The kernel's mechanisms apart from her scenes (docs/todo/protocol.md): its bank, shield, box, probe.

Its lowering and bank, her body schema and its shield, her drives' back-EMF box, the phase-margin
probe, the wayfinder."""
import sys

from tools.dev.focus import chosen
from gynoid_kit import Report


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


def _brute(caps, pairs, steps=20001):
    """Each pair's axes' least distance by a fine walk along the first, the second's nearest point to
    each step in closed form."""
    import numpy as np
    out = []
    for i, j in pairs:
        s = np.linspace(0.0, 1.0, steps)[:, None]
        p = caps['p0'][i] + s * (caps['p1'][i] - caps['p0'][i])
        q0, d2 = caps['p0'][j], caps['p1'][j] - caps['p0'][j]
        t = np.clip(((p - q0) @ d2) / max(float(d2 @ d2), 1e-300), 0.0, 1.0)[:, None]
        out.append(float(np.sqrt(((p - q0 - t * d2) ** 2).sum(1)).min()))
    return np.array(out)


def test_the_body_schema_lays_her_capsules(report):
    """The text's 18 capsules on her skin (`machine.schema`): her segments, the barriers' floors;
    two capsules' nearest axes as a fine walk finds them, parallel, offset and a sphere among them;
    the log-sum-exp under the least by at most log(n) / alpha."""
    import numpy as np
    from machine import capsules, schema
    rng = np.random.default_rng(3)
    n = 64
    caps = {'p0': rng.normal(size=(n, 3)), 'p1': rng.normal(size=(n, 3)), 'r0': np.zeros(n), 'r1': np.zeros(n)}
    caps['p1'][:4] = caps['p0'][:4] + [1.0, 0.0, 0.0]
    caps['p0'][4:8] = caps['p0'][:4] + [0.0, 0.5, 0.0]
    caps['p1'][4:8] = caps['p0'][4:8] + [2.0, 0.0, 0.0]
    caps['p1'][8:12] = caps['p0'][8:12]
    pairs = np.array([(k, k + 4) for k in range(4)] + [(8 + k, 30 + k) for k in range(4)]
                     + [(2 * k, 2 * k + 1) for k in range(8, 32)])
    d = capsules.nearest(caps, pairs)['d']
    off = float(np.abs(d - _brute(caps, pairs)).max())
    soft = capsules.smooth(d)
    named = dict(zip(capsules.NAMES, capsules.SEGMENT))
    floors = {tuple(capsules.NAMES[k] for k in pair): f for pair, f in zip(capsules.PAIRS, capsules.FLOOR)}
    report.check('%d capsules on her segments, the feet 7.5 mm apart at least, shanks 12, forearm and hips 15, '
                 'thighs touching; %d pairs as a fine walk finds them within %.1e m; the soft least %.3f under %.3f'
                 % (len(capsules.NAMES), len(pairs), off, soft, float(d.min())),
                 len(capsules.NAMES) == 18 and named['foot-shell-l'] == 'left_foot' and named['thoracic-ribcage'] == 'torso'
                 and floors[('foot-shell-l', 'foot-shell-r')] == 0.0075
                 and floors[('shank-crural-l', 'shank-crural-r')] == 0.012
                 and floors[('forearm-ulnar-l', 'pelvic-girdle')] == 0.015
                 and floors[('thigh-femoral-l', 'thigh-femoral-r')] == 0.0
                 and off < 1e-6 and d.min() - np.log(len(d)) / capsules.ALPHA <= soft <= d.min(), '')


def test_the_shield_holds_a_closing_pair(report):
    """Her standing, her left foot closing on her right at 4 m/s: the shield changes her posture's
    ask on the pair's barrier's evading joints alone (her hips' rolls, the swing's circumduction),
    each within its drive's peak, so that each pair's h'' + k_d h' + k_p h >= 0 holds or what it
    cannot is spilled - least in her mass's metric there (its change her mass's inverse through the
    held rows, their multipliers not negative); her left forearm closing on her hips, a pair whose
    barrier evades by her shoulder's abduction she has not, at 1.5 m/s spilled whole; at rest nothing."""
    import numpy as np
    from machine import bemf, capsules, edge, schema, worker
    from machine.wbc import POSTURE_KD
    s = worker.session()
    while s['g']['t'] < 1.0:
        worker.stepped(s)
    g = s['g']
    body, st, ask, at, d = g['body'], g['s'], g['ask'], g['schema_at'], g['schema_d']
    volts = float(g['sense'][edge.LINK][0])
    rest, info = schema.shielded(body, d, st, ask, volts, at)
    pairs = [tuple(p) for p in capsules.PAIRS.tolist()]

    def closing(left, right, speed):
        k = pairs.index((capsules.NAMES.index(left), capsules.NAMES.index(right)))
        near = capsules.nearest(capsules.laid(capsules.posed(body, d, st), at), capsules.PAIRS)
        J, _rate = capsules.rows(body, d, near, capsules.PAIRS, np.array([k]), at, st['v'])
        v = st['v'] - speed * J[0] / float(J[0] @ J[0])
        moved = dict(st, v=v)
        return k, near['d'][k], v, moved, schema.shielded(body, d, moved, ask, volts, at)
    k, gap, v, moved, (posture, held) = closing('foot-shell-l', 'foot-shell-r', 4.0)
    act, rows = body.act, held['which']
    free = np.flatnonzero(capsules.EVADES[rows].any(0))
    Jh, rate = capsules.rows(body, d, capsules.nearest(capsules.laid(capsules.posed(body, d, moved), at), capsules.PAIRS),
                           capsules.PAIRS, rows, at, v)
    want = POSTURE_KD[0] * (posture - st['q']) - POSTURE_KD[1] * v[act]
    sides = Jh[:, act] @ want + schema.KD * rate + schema.KP * held['h'] + held['spill']
    change = POSTURE_KD[0] * (posture - np.asarray(ask['posture']))
    Mf = st['M'][np.ix_(act[free], act[free])]
    top = np.asarray(bemf.PEAK)[free] / np.diag(Mf)
    on = [list(rows).index(r) for r in held['held']]
    inner = np.abs(change[free]) < top * (1.0 - 1e-9)
    lam = np.linalg.lstsq(Jh[on][:, act[free]][:, inner].T, (Mf @ change[free])[inner], rcond=None)[0]
    fit = float(np.abs(Jh[on][:, act[free]][:, inner].T @ lam - (Mf @ change[free])[inner]).max())
    arm, _gap, _v, _moved, (arm_posture, arm_held) = closing('forearm-ulnar-l', 'pelvic-girdle', 1.5)
    report.check("at rest nothing held; her feet %.1f mm apart closing at 4 m/s: %d held on %s, each row met to "
                 "%.1e, the change her mass's inverse through them (%.1e), multipliers %s; her forearm on her hips "
                 "spilled %.0f m/s^2, her posture as asked"
                 % (1e3 * float(gap), len(held['held']), ' '.join(bemf.DRIVEN[i] for i in free), float(sides.min()), fit,
                    np.round(lam, 4), float(np.max(arm_held['spill'], initial=0.0))),
                 not info['held'] and np.array_equal(rest, np.asarray(ask['posture'], float))
                 and k in held['held'] and float(sides.min()) > -1e-6 and not change[np.setdiff1d(np.arange(len(act)), free)].any()
                 and bool((np.abs(change[free]) <= top * (1.0 + 1e-9)).all())
                 and fit < 1e-6 * max(1.0, float(np.abs(Mf @ change[free]).max())) and bool((lam >= -1e-9).all())
                 and arm in arm_held['held'] and float(np.max(arm_held['spill'])) > 0.0
                 and np.array_equal(arm_posture, np.asarray(ask['posture'], float)), '')


def test_her_drives_back_emf_bounds(report):
    """Her drives' torque bounds under their back-EMF (`machine.bemf`): still, each its peak or its
    winding's volts' torque both ways; motoring falls with speed to 0 at V m / E, braking keeps its
    peak. Her law's three steps (`wrench.step`, wbc_step.c, the board's wbc_wrench.c) under a box
    that binds: her ask not shrunk for it, each drive clipped inside it, the same torques within the
    twins' own 5e-3 N m."""
    import math
    import numpy as np
    from machine import bemf, drives, physics, figure, wbc, wrench
    from tools.cores import step as c_step
    from tools.cores.wbc import shared, wrench_step
    volts = drives.PACK_V
    k, e, peak = (np.asarray(x, float) for x in (bemf.K, bemf.E, bemf.PEAK))
    still = bemf.bounds(np.zeros(len(k)), volts, peak)
    top = bemf.REACH * volts / e
    fast = bemf.bounds(top, volts, peak)
    report.check('still each drive its peak or its volts\' torque both ways; at V m / E (%.1f-%.1f rad/s) motoring 0, '
                 'braking its peak' % (top.min(), top.max()),
                 np.allclose(still[1], np.minimum(peak, k * bemf.REACH * volts)) and np.allclose(still[0], -still[1])
                 and np.allclose(fast[1], 0.0, atol=1e-9) and np.allclose(fast[0], -peak)
                 and math.isclose(bemf.REACH, 1.0 / math.sqrt(3.0), rel_tol=1e-6), '')
    b, world = wbc.Body(), physics.World()
    world.reset(figure.stand())
    rng = np.random.default_rng(11)
    qvel = np.zeros(len(world.data.qvel))
    qvel[:b.n] = 0.3 * rng.normal(size=b.n)
    sense = wbc.sense(b, world.data.qpos, qvel)
    ask = {'stance': (True, True), 'com_acc': np.array([0.4, 1.5, -0.3]), 'posture': sense['q'] + 0.2 * rng.normal(size=len(sense['q'])),
           'turns': (sense['turns'][0]['quat'], sense['turns'][1]['quat']), 'swing': {}, 'fold': {}, 'wep': False}
    plain = wrench.step(b, sense, ask)
    free = plain['tau']
    hi = np.where(free > 0.0, 0.5 * free, np.inf)
    lo = np.where(free < 0.0, 0.5 * free, -np.inf)
    boxed = dict(ask, lo=lo, hi=hi)
    ours = wrench.step(b, sense, boxed)
    solve = c_step.wrench_laid(b)
    theirs, board = solve(b, wbc.sense(b, world.data.qpos, qvel), boxed), wrench_step(b, sense, boxed, shared())
    apart = max(float(np.abs(ours['tau'] - theirs['tau']).max()), float(np.abs(ours['tau'] - board['tau']).max()))
    report.check('a box half her torques each way: her ask carried %.3f as without it, each drive clipped to the box, '
                 'its C twins %.1e N m apart' % (ours['scale'], apart),
                 ours['scale'] == plain['scale'] and np.array_equal(ours['tau'], np.clip(plain['tau'], lo, hi))
                 and bool((ours['tau'] != plain['tau']).any())
                 and apart < 5e-3, '')


def test_the_lock_in_reads_a_known_loop(report):
    """The insert's phase-margin probe (`machine.margin`) on a known loop at 1 kHz - a drive's PD on
    an inertia, its torque three passes late, the probe's sine in its torque, each frequency in its
    turn: its gain and phase at each as the loop's own L(e^jwT); the text's rollback where a margin falls short, to
    its floors, none where all hold."""
    import cmath
    import math
    from machine import margin
    inertia, kp, kd, dt, late = 0.02, 40.0, 0.8, 1e-3, 3
    probe, x, v, t, past = margin.laid(), 0.0, 0.0, 0.0, [0.0] * late
    while t < 4 * margin.DWELL_S + 0.01:
        d = margin.injected(probe, t, 0.2)
        c = past.pop(0)
        u = c + d
        v += u / inertia * dt
        x += v * dt
        past.append(-(kp * x + kd * v))
        margin.step(probe, u, c, t, dt)
        t += dt
    got = margin.read(probe)

    def own(f):
        z = cmath.exp(1j * 2.0 * math.pi * f * dt)
        plant = dt * dt * z / (inertia * (z - 1.0) ** 2)
        return (kp + kd * (z - 1.0) / (dt * z)) * plant * z ** (1 - late)
    off = [(abs(r['gain_db'] - 20.0 * math.log10(abs(own(r['hz'])))),
            abs((r['phase_deg'] - math.degrees(cmath.phase(own(r['hz']))) + 180.0) % 360.0 - 180.0)) for r in got]
    short = [{'gain_db': 0.0, 'margin_deg': 19.0}, {'gain_db': -10.0, 'margin_deg': 5.0}]
    report.check('at %s Hz its gain within %.2f dB and phase within %.2f deg of the loop\'s own; a 19 deg margin at '
                 'crossover rolls K_d and gamma to 0.5, 5 deg to their floors, held margins none'
                 % ('/'.join('%g' % f for f in margin.HZ), max(o[0] for o in off), max(o[1] for o in off)),
                 max(o[0] for o in off) < 0.25 and max(o[1] for o in off) < 2.5
                 and margin.derated(short) == (0.5, 0.5)
                 and margin.derated([{'gain_db': 0.0, 'margin_deg': 5.0}]) == (margin.KD_FLOOR, margin.GAMMA_FLOOR)
                 and margin.derated([{'gain_db': 0.0, 'margin_deg': 60.0}, {'gain_db': -10.0, 'margin_deg': 1.0}]) == (1.0, 1.0),
                 str(got))


ROSTER = (test_the_lowering_lays_the_texts_abi,
          test_a_bank_swaps_with_no_step,
          test_the_body_schema_lays_her_capsules,
          test_the_shield_holds_a_closing_pair,
          test_her_drives_back_emf_bounds,
          test_the_lock_in_reads_a_known_loop)


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
