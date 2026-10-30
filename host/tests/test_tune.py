"""The autotuner on her texts: its text read, her texts given, every literal walked, its reach, its step.

`machine.texts.tune`, `tools/sim/tune.py`, `tools/sim/telemetry.py`, `machine.process.rcu`, `machine.process.tap`: her texts
given play as her files, every literal is walked where it stands and reaches her where a voice reads
it, the step is the text's, a run's residuals and the verdicts on a step; at runtime a link between
her loop and a tuner apart, her walk linked as unlinked, her raw passes cooked as MuJoCo reads
them, her walk measured at runtime with her loop apart, a table laid at her barrier blending C2."""
import math
import sys

from tools.dev.focus import chosen
from gynoid_kit import Report


def _got():
    from machine.texts import dsp
    return dsp.read(dsp.SUITE)


def _her():
    from machine.process import worker
    return worker.session()['g']['fr']


def _near(a, b):
    return math.isclose(a, b, rel_tol=1e-9, abs_tol=1e-9)


def test_the_autotuner_reads_as_written(report):
    """The autotuner's text: its six classes in order, its keys and invariants the suite's names,
    each primitive done here, its step's numbers its own."""
    from machine.texts import tune
    gr = tune.grammar()
    report.check("six classes in its order, their invariants the manifold's",
                 [(r.name, r.invariant) for r in gr.values()] == [
                     ('com-wrench-balance', 'min-zmp-margin-mm'), ('normal-force-floor', 'min-normal-force-ratio'),
                     ('tangential-shear-slip', None), ('spatial-sdf-clearance', 'min-coronal-clearance-mm'),
                     ('contact-transient-ringdown', None), ('actuator-thermal-passivity', None)]
                 and all(r.invariant in _got()['physical-invariants'] for r in gr.values() if r.invariant),
                 str([(r.name, r.invariant) for r in gr.values()]))
    report.check("its keys the suite's: a duck's depth its modulation, a cutoff its low-pass's",
                 'modulation-depth-mm' in gr['normal-force-floor'].keys
                 and 'lpf-cutoff-hz' in gr['actuator-thermal-passivity'].keys, str(gr['normal-force-floor'].keys))
    k = tune.knobs()
    said = [k[('compute-adjoint-ast-deltas', 'damping-lambda')], k[('compute-adjoint-ast-deltas', 'trust-region-scale')],
            k[('universal-autotune-daemon', 'tol')], k[('sync-mutated-packages-to-disk', 'float-precision')],
            k[('introspect-active-ast', 'max-nodes')]]
    report.check('each primitive done here; lambda, trust, tol, precision, table its own',
                 tune.primitives() == list(tune.PRIMITIVES) and said == [0.08, 0.15, 1e-3, 4.0, 512.0],
                 '%s %s' % (tune.primitives(), said))


def test_her_texts_given_play_as_her_files(report):
    """Her texts read and given (`tune.texts`): the suite, her gym's entries and a stunt's entry the
    same as her files'; one not among them refused by name."""
    from machine.texts import dsp, maneuver, ortho, tune
    from machine.process import sprung
    tx = tune.texts()
    got, gym = tune.evaluated(tx)
    report.check('%d texts: the suite and her gym as from her files' % len(tx),
                 got == _got() and gym == sprung.entries()
                 and maneuver.entry('sit_chair', None, got, tx) == maneuver.entry('sit_chair'), '')
    short = {k: v for k, v in tx.items() if not k.endswith('gait_base.lisp')}
    try:
        ortho.compiled(texts=short)
        said = ''
    except SyntaxError as error:
        said = str(error)
    report.check('a text not given refused by name', 'gait_base.lisp' in said, said)


def test_a_literal_is_walked_where_it_stands(report):
    """Each keyword's number, each op's and each of her gym's clauses' where it stands: written back
    in full the suite and her gym read the same; a value at the text's precision reads back; her gym's
    stiffness the suite's compliance, its value her walk's row's axial spring."""
    from machine.texts import dsp, ortho, tune, voicing
    from machine.law import modal
    from machine.process import sprung
    tx = tune.texts()
    lits = [lit for path, text in tx.items() for lit in tune.literals(path, text)]
    report.check('%d literals, each span its value' % len(lits),
                 lits and all(float(tx[lit.path][lit.start:lit.end]) == lit.value for lit in lits), '')
    back = tune.spliced(tx, lits, [lit.value for lit in lits])
    report.check('every literal written back: the suite and her gym read the same',
                 dsp.read(dsp.SUITE, back) == _got() and sprung.entries(texts=back) == sprung.entries(), '')
    spring = next(lit for lit in lits if lit.symbol == 'stand' and lit.node == 'impedance stiffness mode_axial_compression')
    report.check("her stand's axial stiffness is the suite's compliance, in her row's axial spring",
                 spring.key == 'compliance' and spring.slot == ortho.K + modal.AXIAL
                 and float(voicing.walk(_her(), sprung.entries())[spring.slot]) == spring.value, str(spring))
    op = next(lit for lit in lits if lit.symbol == 'surface-granular-dissipative' and lit.key == 'stride-length-mm')
    moved = tune.spliced(tx, [op], [0.71234567], 4)
    report.check("an op's number said of its keyword, written at four places",
                 op.op == 'scale' and op.value == 0.82 and '(:scale 0.7123)' in moved[op.path]
                 and dsp.read(dsp.SUITE, moved)['surface-granular-dissipative'] != _got()['surface-granular-dissipative'],
                 str(op))


def test_a_literal_reaches_her_where_a_voice_reads_it(report):
    """The runway reaches 17 literals: its stride, its trunk's lean and its duck's attack move her
    voices by their rules, nothing voiceless does, a 0 the manifold says nothing larger of has no
    scale. The chair's sit: its hand-over's and its descent's times move her phases."""
    from machine.law import modal
    from machine.texts import ortho, tune, voicing
    tx, fr, gr = tune.texts(), _her(), tune.grammar()
    table = tune.walked('patch-sneaker-runway', gr, tx)
    cols, refused = tune.reach('patch-sneaker-runway', table, tx, fr)
    live = {table[i][0].key: col for i, col in cols.items()}
    twist = voicing.voiced(_got()['patch-sneaker-runway'], _got()['nominal-biped-manifold'], fr)[1]['twist'][0]
    h = 0.15 * 640.0
    want = {'stride-length-mm': {('twist', 0): -twist * 640.0 / (640.0 ** 2 - h * h)},
            'dc-offset-rad': {('slot', ortho.Z + modal.PITCH): -1.0 / fr[modal.UNIT][modal.PITCH]},
            'attack-ms': {('slot', ortho.DUCK.start + 1): 1e-3}}
    report.check('17 walked: stride, lean and attack move her by their rules, the rest inert',
                 len(table) == 17 and set(live) == set(want)
                 and all(set(live[k]) == set(v) and all(_near(live[k][j], x) for j, x in v.items()) for k, v in want.items()),
                 '%d %s' % (len(table), live))
    report.check('a circumduction of 0 has no scale', [table[i][0].key for i in refused] == ['swing-circumduction-mm'],
                 str(refused))
    table = tune.walked('sit_chair', gr, tx)
    cols, refused = tune.reach('sit_chair', table, tx, fr)
    report.check("the sit: its hand-over's and descent's ms move her phases' times",
                 sorted(table[i][0].key for i in cols) == ['descent-duration-ms', 'transfer-time-ms']
                 and all(v == 1.0 for col in cols.values() for v in col.values()), str(cols))
    table = tune.walked('walk', gr, tx)
    cols, refused = tune.reach('walk', table, tx, fr)
    report.check("her walk: her stand's axial spring and damper and her step's lift move her row, her lean's 0s no scale",
                 sorted(table[i][0].key for i in cols) == ['compliance', 'damping-ratio', 'swing-clearance-z-mm']
                 and sorted(table[i][0].key for i in refused) == ['dc-offset-rad', 'dc-offset-rad'], '%s %s' % (cols, refused))


def test_the_step_is_damped_least_squares_in_its_trust_region(report):
    """The step solves (J^T J + lambda I) x = -J^T r in the literals' scales, each move within the
    text's trust region of its scale, a held measure not growing."""
    import numpy as np
    from machine.texts import tune
    J, scales = np.array([[2.0, 0.0], [0.0, 0.5]]), np.array([1.0, 4.0])
    small = tune.step(J, np.array([0.01, -0.02]), scales)
    a = np.diag(J) * scales
    want = -a * np.array([0.01, -0.02]) / (a * a + 0.08) * scales
    big = tune.step(J, np.array([10.0, 0.0]), scales)
    report.check('a small residual: the damped solve; a large one: held to 0.15 of its scale',
                 np.allclose(small, want) and math.isclose(abs(big[0]), 0.15 * scales[0]) and abs(big[1]) < 1e-12,
                 '%s %s %s' % (small, want, big))
    free = tune.step(np.eye(2), np.array([-0.02, 0.01]), np.ones(2))
    held = tune.step(np.eye(2), np.array([-0.02, 0.01]), np.ones(2), [[1.0, 1.0]])
    report.check('a measure growing with both literals held: free it grows, held they move against each other',
                 free[0] + free[1] > 0.0 and np.allclose(held, [0.015 / 1.08, -0.015 / 1.08]), '%s %s' % (free, held))


def test_a_run_measured_and_a_step_judged(report):
    """Her contacts' hull and a point's margin in it; the six residuals of known measures in their
    dimensions; a step not kept where she falls, its goal is lost, her form moves out past its
    floor or the norm falls no further than its floor."""
    import numpy as np
    from machine.process import tap
    from machine.texts import tune
    from tools.sim import looks, telemetry, tune as tuner
    square = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.5, 0.5)]
    margin = tap.margin
    report.check("a square's centre 0.5 inside, a point 0.25 past its side outside, a segment's off point outside",
                 math.isclose(margin((0.5, 0.5), square), 0.5) and math.isclose(margin((1.25, 0.5), square), -0.25)
                 and math.isclose(margin((0.5, 1.0), [(0.0, 0.0), (1.0, 0.0)]), -1.0),
                 '%s %s' % (margin((0.5, 0.5), square), margin((1.25, 0.5), square)))
    bounds = telemetry.bounds(_got())
    r = telemetry.residuals({'pivot': [10.0, 20.0], 'load': 0.7, 'cone': 0.5, 'gap': 3.25, 'peak': 1.5,
                             'bleed': 0.5 * bounds['tank-capacity-joules']}, tune.grammar(), bounds)
    report.check('a pivot 5 mm short half the time, no load short, half the cone used, the feet half their gap '
                 'short, a peak 0.2 past a landing, half the tank bled',
                 np.allclose(r, [2.5 / 15.0, 0.0, 0.5, 0.5, 0.25 / 1.25, 0.5]), str(r))
    form = {name: None for name, _least, _most in looks.FORM}
    base = {'r': [1.0, 0, 0, 0, 0, 0], 'fell': None, 'formed': True, 'met': True, 'floor': 0.1,
            'form': dict(form, **{'landing load': 1.3}), 'off_floor': {name: 0.01 for name in form}}
    said = [tuner.judged(base, dict(base, **change)) for change in (
        {'fell': 3.2}, {'met': False}, {'form': dict(form, **{'landing load': 1.4}), 'r': [0.5, 0, 0, 0, 0, 0]},
        {'r': [0.95, 0, 0, 0, 0, 0]}, {'r': [0.5, 0, 0, 0, 0, 0]})]
    report.check('a fall, a lost goal, a landing worse, a gain under its floor not kept; a gain past it kept',
                 [str(s).split(':')[0].split(',')[0] for s in said] == [
                     'she fell at 3.20 s', 'its goal no longer met', 'her form worse', '|r| 1.0000 -> 0.9500', 'None'],
                 str(said))


def test_a_link_between_her_loop_and_a_tuner(report):
    """Two sides of one block of shared memory, her loop's in C: a ring's samples drained in order, a
    full one dropping and counting; a table published, a second refused till her loop took the first;
    every section on a 64-byte line where both sides lay it; the block locked in memory here."""
    import sys
    import numpy as np
    from machine.process import rcu
    a = rcu.Link.made(((13, 4), (5, 3)), 5, 3)
    b = rcu.Link.found(a.name)
    lib, words = rcu._core(), lambda x: (x.ctypes.data - a.base) // rcu.WORD
    laid = [[lib.wbh_rcu_at(a.base, what, k) for what in range(3)] for k in range(2)]
    report.check("C lays the rings' lines, their slots and the banks where rcu does; locked on Windows",
                 laid == [[words(a.r[k]), words(a.rings[k]), words(a.banks[k])] for k in range(2)]
                 and (b.locked or sys.platform != 'win32'), '%s %s' % (laid, b.locked))
    tapped = [b.tap(np.arange(13.0) + k) for k in range(5)]
    dropped, drained = b.word('dropped', 0), a.drained()
    first, second = a.published(np.ones((5, 2))), a.published(np.full((5, 2), 2.0))
    took = b.taken()
    again, lines = b.taken(), [x.ctypes.data % rcu.LINE for x in a.rings + [a.banks, a.banks[0, 1], a.banks[1]]]
    b.closed()
    a.closed()
    report.check('four of five samples kept, the fifth dropped and counted, drained in order; one table taken, '
                 'a second refused before it; every section on a line',
                 tapped == [True] * 4 + [False] and dropped == 1 and drained[:, 0].tolist() == [0.0, 1.0, 2.0, 3.0]
                 and first and not second and took is not None and took[0] == 1
                 and took[1].tolist() == np.ones((5, 2)).tolist() and again is None
                 and not any(lines), '%s %d %s %s' % (tapped, dropped, drained[:, 0], lines))


def _walked(seconds, link=None):
    """Her session walking `seconds` s, linked to `link` where given: (the session, each pass's time)."""
    import time
    from machine.process import worker
    from tools.sim import scenarios
    s = worker.session()
    if link is not None:
        worker.apply(s, {'link': link.name})
    worker.apply(s, {'twist': scenarios.WALK})
    times = []
    while s['g']['t'] < seconds:
        began = time.perf_counter()
        worker.stepped(s)
        times.append(time.perf_counter() - began)
    return s, times


def test_her_loop_linked_passes_as_unlinked(report):
    """Her walk with a tuner linked and nothing laid passes as without one, bit for bit: the hook reads
    her world and writes the link alone."""
    import numpy as np
    from machine.process import tap
    plain, plain_s = _walked(3.0)
    link = tap.made(4.0)
    linked, linked_s = _walked(3.0, link)
    same = np.array_equal(plain['g']['world'].data.qpos, linked['g']['world'].data.qpos)
    tapped = link.word('head', 0)
    for found in linked['links']:
        found.closed()
    link.closed()
    report.check('her pose after 3 s the same bit for bit, %d passes tapped; a pass %.2f ms linked, %.2f ms not'
                 % (tapped, 1e3 * float(np.median(linked_s)), 1e3 * float(np.median(plain_s))),
                 same and tapped == len(linked_s), '%s %d' % (same, tapped))


def test_her_raw_passes_cook_as_mujoco_reads_them(report):
    """Her raw passes (C) cooked: the world's force on her, her slip and the cone's use as MuJoCo's
    own reads of each contact give them - `mj_contactForce`, her point's velocity off `cvel`."""
    import importlib
    import numpy as np
    from machine.senses import edge
    from machine.process import tap, worker
    mujoco = importlib.import_module('mujoco')
    s, _times = _walked(1.0)
    world = s['g']['world']
    m, d = world.model, world.data
    state, raws, want = tap.fresh(), [], []
    hers = np.array([edge._rooted(m, b, world.pelvis) for b in range(m.nbody)])[m.geom_bodyid]
    while s['g']['t'] < 1.3:
        worker.stepped(s)
        raws.append(tap.raw(s, state))
        f_on, slip, cone, borne, grip = np.zeros(3), 0.0, 0.0, 0.0, 0.0
        for i in range(d.ncon):
            c, f6, v6 = d.contact[i], np.zeros(6), np.zeros(6)
            one = hers[c.geom1]
            if one == hers[c.geom2] or c.efc_address < 0:
                continue
            mujoco.mj_contactForce(m, d, i, f6)
            frame, body = c.frame.reshape(3, 3), m.geom_bodyid[c.geom1 if one else c.geom2]
            mujoco.mju_transformSpatial(v6, d.cvel[body], 0, c.pos, d.subtree_com[m.body_rootid[body]], np.eye(3).ravel())
            f_on += (-1.0 if one else 1.0) * frame.T @ f6[:3]
            slip += abs(f6[0]) * float(np.linalg.norm(v6[3:] - (v6[3:] @ frame[0]) * frame[0]))
            cone, borne, grip = cone + math.hypot(f6[1], f6[2]), borne + abs(f6[0]), grip + c.friction[0] * abs(f6[0])
        want.append((*f_on, slip / borne, cone / grip))
    got, want = tap.cooked(raws), np.array(want)
    off = np.abs(got[:, [tap.S[k] for k in ('fx', 'fy', 'fz', 'slip', 'cone')]] - want).max(0) / np.abs(want).max(0)
    report.check('%d passes: force, slip and cone use off MuJoCo\'s by %.1e, %.1e, %.1e of their most'
                 % (len(raws), off[:3].max(), off[3], off[4]), bool(off.max() < 1e-12), str(off))


def test_her_walk_measured_at_runtime(report):
    """Her loop in a process of its own, linked: her walk's row compiled from her texts as they are
    is her loop's, and a window of it published once she walks is measured standing, its six
    residuals finite, her form measured."""
    import multiprocessing
    import numpy as np
    from machine.law import modal
    from machine.process import tap
    from machine.texts import tune, voicing
    from tools.sim import runtime, suite
    tx, fr = tune.texts(), suite._her()
    c = {'gr': tune.grammar(), 'texts': tx, 'fr': fr, 'got': tune.evaluated(tx)[0], 'weight': float(fr[modal.WEIGHT]),
         'seq': 0, 'slots': [11, 22, 38], 'link': tap.made(runtime.RING_S)}
    walked = voicing.walk(fr, tune.evaluated(tx)[1])
    c['her'] = multiprocessing.get_context('spawn').Process(target=runtime.loop, args=(c['link'].name,), daemon=True)
    c['her'].start()
    try:
        out = runtime.runs(c)([(runtime.ROOT, {})])[0]
    finally:
        c['link'].said('halt', 1)
        c['her'].join(10.0)
        c['link'].closed()
    report.check('her walk as her loop holds it; a window standing, |r| %.3f, her form measured in %d of %d'
                 % (float(np.linalg.norm(out.get('r', [math.inf]))), sum(v is not None for v in out.get('form', {}).values()),
                    len(out.get('form', {}))),
                 runtime.row(c, {}) == {k: float(walked[k]) for k in c['slots']} and out.get('fell', 0) is None
                 and bool(np.all(np.isfinite(out['r']))) and any(v is not None for v in out['form'].values()), str(out)[:300])


def test_a_table_is_laid_at_her_barrier_and_blends_c2(report):
    """Her walk linked, a table of her lean published walking: laid at a pass with both soles locked
    and her last blend done; her lean's target leaves and meets its new value minimum-jerk, its rate
    with no step - a linear blend's step the control."""
    import numpy as np
    from machine.law import gait, latent, modal
    from machine.texts import ortho, patch
    from machine.process import tap, worker
    slot, link = ortho.Z + modal.PITCH, tap.made(30.0)
    s, _times = _walked(6.0, link)
    g, said, flags = s['g'], [], []
    for k, (delta, shaped) in enumerate(((0.1, True), (-0.1, False))):
        table = tap.table({slot: float(g['st'][latent.NOW][slot]) + delta}, gait.BLEND_S, gait.ROW_ID)
        table[3, 0] = table[3, 0] if shaped else 0
        link.published(table)
        while link.word('laid') == k:
            worker.stepped(s)
            said.append((g['t'], float(g['st'][latent.NOW][slot])))
        flags.append((g['t'], patch.locked(g['y'], g['fr'], g['a']),
                      g['t'] >= float(g['st'][latent.T0] + g['st'][latent.BLEND])))
        while g['t'] < flags[-1][0] + gait.BLEND_S + 0.5:
            worker.stepped(s)
            said.append((g['t'], float(g['st'][latent.NOW][slot])))
    for found in s['links']:
        found.closed()
    link.closed()
    t, v = np.array(said).T
    steps = []
    for at, _locked, _done in flags:
        k, j = int(np.searchsorted(t, at)), int(np.searchsorted(t, at + gait.BLEND_S + 0.1))
        rate = np.abs(np.diff(v[k - 3:k + 15:3]))
        steps.append(float(rate.max() / (abs(v[j] - v[k - 3]) * 3e-3 / gait.BLEND_S)))
    report.check('laid with both soles locked, her last blend done; its rate leaving %.4f of its mean '
                 'minimum-jerk, %.2f linear' % tuple(steps),
                 all(locked and done for _t, locked, done in flags) and steps[0] < steps[1], str(flags))


ROSTER = (test_the_autotuner_reads_as_written, test_her_texts_given_play_as_her_files,
          test_a_literal_is_walked_where_it_stands, test_a_literal_reaches_her_where_a_voice_reads_it,
          test_the_step_is_damped_least_squares_in_its_trust_region,
          test_a_run_measured_and_a_step_judged, test_a_link_between_her_loop_and_a_tuner,
          test_her_loop_linked_passes_as_unlinked, test_her_raw_passes_cook_as_mujoco_reads_them,
          test_her_walk_measured_at_runtime, test_a_table_is_laid_at_her_barrier_and_blends_c2)


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
