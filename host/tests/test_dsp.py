"""The ortho-DSP suite on her loop (`machine.dsp`, `machine.voicing`): its modules read and evaluated
as written, a patch voiced on her walk by its changes, what she cannot do refused by name, the
runway's dialect read alike, a take converted to a modifier she reads (`tools/sim/identify.py`)."""
import math
import sys

from tools.dev.focus import chosen
from gynoid_kit import Report


def _got():
    from machine import dsp
    return dsp.read(dsp.SUITE)


def _her():
    from machine import worker
    return worker.session()['g']['fr']


def _near(a, b):
    return math.isclose(a, b, rel_tol=1e-9, abs_tol=1e-9)


def test_the_suite_reads_as_written(report):
    """Its patches' params as their stages compute them, in their order: the catwalk's track 0.44
    of the sneaker's foot and its bow clear of it, a clamp on a sidechain none had its bound;
    the ice's track clamped above the foot, its roll clamped under; the boot's track clamped,
    then the pack's added; the stairs' riser the step's; the schedule's nine segments."""
    from machine import dsp
    got = _got()
    run, ice, snow, down = (got['patch-' + k] for k in ('sneaker-runway', 'stealth-crouch-on-ice',
                                                        'loaded-boot-snow-ascent', 'barefoot-stair-descent'))
    side = run['com-dynamics']['sidechain']
    report.check('the runway: track 0.44 of 96.5 mm, bow its gap and 7.5, 110 BPM x 1.03636, duty 0.58',
                 _near(run['tarsal-coronal']['track-width-mm'], 0.44 * 96.5)
                 and _near(run['tarsal-coronal']['swing-circumduction-mm'], 96.5 - 0.44 * 96.5 + 7.5)
                 and _near(run['oscillator']['bpm'], 110.0 * 1.03636) and _near(run['oscillator']['duty-factor-l'], 0.58),
                 str(run['tarsal-coronal']))
    report.check('its sidechain: attack at its clamp 105 ms, release 260 ms, depth -26 mm (pitch 0.035 under 0.3)',
                 (side['attack-ms'], side['release-ms'], side['modulation-depth-mm']) == (105.0, 260.0, -26.0), str(side))
    report.check('the crouch on ice: 920 - 165 - 45 mm high, track 96.5 + 18 mm, roll 0.055, mu 0.08 unsplayed',
                 _near(ice['com-dynamics']['base-height-z-mm'], 710.0) and _near(ice['tarsal-coronal']['track-width-mm'], 114.5)
                 and _near(ice['iliac-coronal']['lfo-amplitude-rad'], 0.055) and _near(ice['wrench-cone']['mu-static'], 0.08),
                 str(ice['com-dynamics']))
    report.check('the snow ascent: duty 0.62 + 0.08/3, track 118.5 + 35/3, stride and clearance as its stages scale them',
                 _near(snow['oscillator']['duty-factor-l'], 0.62 + 0.08 * 25.0 / 75.0)
                 and _near(snow['tarsal-coronal']['track-width-mm'], 118.5 + 35.0 * 25.0 / 75.0)
                 and _near(snow['tarsal-sagittal']['stride-length-mm'], 640.0 * (1.0 - 0.8 * 0.21) * 0.82)
                 and _near(snow['tarsal-sagittal']['swing-clearance-z-mm'], 38.0 + 18.0 + 1.35 * 75.0),
                 str(snow['tarsal-sagittal']))
    report.check('the stair descent: a -175 mm step, 207 mm clearance, 102 BPM',
                 (down['tarsal-sagittal']['step-elevation-z-mm'], down['tarsal-sagittal']['swing-clearance-z-mm'],
                  down['oscillator']['bpm']) == (-175.0, 207.0, 102.0), str(down['tarsal-sagittal']))
    plan = dsp.schedule(got['compile-omnidirectional-suite'])
    report.check('its schedule: 9 segments over 32 bars, its morphs of 2 bars each',
                 len(plan) == 9 and plan[-1][1] == 32.0
                 and [b1 - b0 for b0, b1, x in plan if isinstance(x, dsp.Morph)] == [2.0] * 4,
                 str([(b0, b1, type(x).__name__) for b0, b1, x in plan]))


def test_the_maneuvers_read_as_written(report):
    """A patch of a gait threads its stages and transitions in order, a maneuver's acts in theirs:
    the chair's pivot with its stagger and overlap, its envelope at 450 + 18 mm, the contact that
    releases the cross-fade, the wavefront 26 dB down; the bed's -inf, high-Z; the lifecycle's
    oneshots and dwell, its dock braking the runway's clock to 0, its re-ignition shifting 0.45 of
    the runway's track."""
    from machine import dsp
    got = _got()

    def patched(name):
        g = dsp.value([name, 'nominal-biped-manifold'], got)
        assert isinstance(g, dict), g
        return g

    chair = patched('patch-confident-chair-sit')
    pivot = chair['_trace'][1][1]
    (env, at), release = chair['_events'][:2], chair['_events'][2][1]
    report.check('the chair: gaze, pivot, hand-over, release; the pivot a bar, 95 mm staggered, 0.785 rad overlapped',
                 [t[0] for t in chair['_trace']][:4] == ['lock-horizon-gaze', 'pivot-single-anchor-arc',
                                                         'handover-staggered-vertical-chair', 'release-impedance-wavefront']
                 and pivot.bars == 1.0 and pivot.target is None
                 and pivot.acts[0][1]['terminal-footprint']['sagittal-stagger-mm'] == 95.0
                 and pivot.acts[1][1]['fire-below-rad'] == 0.785, str(pivot.acts))
    report.check('its envelope at 468 mm, the trunk 0.14; the seat at 0.15 g releases 240 ms of cross-fade; -26 dB',
                 env[0] == 'trigger-one-shot-envelope' and env[1]['apex-deceleration-height-mm'] == 468.0
                 and env[1]['joint-targets'][0] == (':tibiofemoral', 1.54)
                 and env[1]['axial-counterweight']['dc-offset-rad'] == 0.14
                 and at[0] == 'on-contact-threshold' and at[1]['trigger-fz-ratio'] == 0.15
                 and at[2][0][0] == 'crossfade-support-manifold' and at[2][0][1]['transfer-time-ms'] == 240.0
                 and release['stiffness-atten-db'] == -26.0, str(chair['_events']))
    bed = patched('patch-confident-bed-roll')['_events'][-1][1]
    report.check('the bed: -inf dB, high-Z', bed['stiffness-atten-db'] == -math.inf
                 and bed['inverter-gate-mode'] == ':high-z-passive-bypass', str(bed))
    plan = dsp.schedule(got['compile-full-lifecycle-soa'])
    dock = next(m for name, m in plan[1][1]['_trace'] if name == 'dock-terminal-stance')
    wake = next(m for name, m in plan[3][1]['_trace'] if name == 'ignite-cyclic-clock')
    report.check('the lifecycle: 6 bars, a oneshot, 10 s, a oneshot, 8 bars; 114 BPM braked to 0, duty to 1; '
                 'the walk re-ignited, 0.45 of its 42.5 mm track shifted',
                 [p[0] for p in plan] == [0.0, 'oneshot', 'dwell', 'oneshot', 0.0] and plan[2][1] == 10.0
                 and _near(dock.clock['bpm-ramp'][0], 110.0 * 1.03636) and dock.clock['bpm-ramp'][1] == 0.0
                 and dock.clock['duty-ramp'] == (0.58, 1.0) and wake.target is plan[0][2]
                 and _near(wake.acts[0][1]['coronal-com-shift-mm'], 0.45 * 0.44 * 96.5), str(wake.acts))


def test_a_patch_moves_her_walk_by_its_changes(report):
    """The nominal manifold voiced on itself moves no slot of her walk and tells her walk's speed;
    the runway moves each voiced slot by its change off the nominal, relative to her walk."""
    from machine import dsp, modal, ortho, se2, voicing
    got, fr = _got(), _her()
    dry = got['nominal-biped-manifold']
    row, told = voicing.voiced(dry, dry, fr)
    report.check('the nominal is her walk: no slot moved, her walk speed told', not row and told['twist'] == (
        se2.WALK_SPEED, 0.0, 0.0), '%s %s' % (row, told))
    walk = voicing.walk(fr)
    row, told = voicing.voiced(got['patch-sneaker-runway'], dry, fr)
    want = {ortho.HZ: walk[ortho.HZ] * 110.0 * 1.03636 / 110.0, ortho.Z + modal.WIDTH: 0.44 * 96.5 / 103.0 - 1.0,
            ortho.AMP + modal.LIST: walk[ortho.AMP + modal.LIST] * 1.92,
            ortho.Z + modal.PITCH: (0.20 - 0.04) / fr[modal.UNIT][modal.PITCH],
            ortho.AMP + modal.ARMS: walk[ortho.AMP + modal.ARMS] * 10.0 ** (-1.5 / 20.0),
            ortho.PHASE + modal.ARMS: walk[ortho.PHASE + modal.ARMS] - 0.25,
            ortho.DUCK.start: -0.026, ortho.DUCK.start + 1: 0.105, ortho.DUCK.start + 2: 0.26}
    report.check('the runway: its clock, track, roll, lean, arms and duck on her voices, nothing else',
                 set(row) == set(want) and all(_near(row[k], v) for k, v in want.items()),
                 str({k: (round(row.get(k, math.nan), 4), round(v, 4)) for k, v in want.items()}))
    report.check('its twist her walk speed by its tempo, its grip its cone', _near(told['twist'][0], se2.WALK_SPEED * 1.03636)
                 and told['grip'] == 0.85, str(told))


def test_what_she_cannot_do_is_refused_by_name(report):
    """A gallop by its phase, a sprint by its flight, a carioca by its crossing, a param in no
    table by its name; a sidestep's way and a retro's are told her as a twist."""
    from machine import dsp, voicing
    from machine.errors import MachineError
    got, fr = _got(), _her()
    dry = got['nominal-biped-manifold']

    def refusal(g):
        try:
            voicing.voiced(g, dry, fr)
        except MachineError as error:
            return str(error)
        return ''
    weave = dsp.value(['->', 'nominal-biped-manifold', ['topology-coronal-sidestep', ':carioca-weave', True]], got)
    odd = dsp.value(['->', 'nominal-biped-manifold', ['modulate-node', ':glenohumeral', ':wrist-flick-hz', 3.0]], got)
    found = {k: refusal(g) for k, g in (('gallop', got['patch-asymmetric-gallop-trail']), ('sprint', got['patch-ballistic-sprint']),
                                       ('carioca', weave), ('wrist', odd))}
    report.check('refused by name: the gallop its phase, the sprint its flight, the carioca its crossing, a wrist flick unknown',
                 'inter-limb-phase-rad' in found['gallop'] and 'flight' in found['sprint']
                 and 'crossover-weave' in found['carioca'] and 'glenohumeral.wrist-flick-hz' in found['wrist'], str(found))
    side = dsp.value(['->', 'nominal-biped-manifold', ['topology-coronal-sidestep']], got)
    back = dsp.value(['->', 'nominal-biped-manifold', ['topology-retro-locomotion']], got)
    told = [voicing.voiced(g, dry, fr)[1]['twist'] for g in (side, back)]
    report.check('a sidestep told her leftward, a retro walk backward', told[0][1] > 100.0 * abs(told[0][0])
                 and told[1][0] < 0.0, str(told))


def test_the_insert_is_every_patch_last(report):
    """The anti-spasm insert: its five stages in order, its nine nodes; every param of it named on
    her loop (`machine.supervisor`); every patch of the suite and the clean runtime stream ending
    in it, a number said otherwise refused by name; her bus laid of it - each drive's slew and jerk
    its own (its peak through its current loop, over its rotor's inertia), a buffer before each stop
    of its boundary nodes' joints."""
    import copy
    from machine import dsp, supervisor, voicing
    from machine.errors import MachineError
    from machine.figure import JOINTS
    got, fr = _got(), _her()
    g = dsp.value(['->', 'nominal-biped-manifold', ['master-anti-spasm-insert']], got)
    assert isinstance(g, dict), g
    stages = ['latch-hysteretic-contact', 'regularize-kinematic-singularities', 'bridge-backlash-zero-crossing',
              'suppress-derivative-chatter', 'enforce-passivity-energy-tank', 'master-anti-spasm-insert']
    report.check('its stages in order, a node an op', [t[0] for t in g['_trace']] == stages
                 and all(at in g for at in dsp.GUARDS.values()), str([t[0] for t in g['_trace']]))
    said = {k for k in voicing._flat(g) if k[0] in dsp.GUARDS.values()}
    unnamed = sorted('%s.%s' % k for k in said if not supervisor.named(*k))
    report.check('every param of it named on her loop: %d' % len(said), not unnamed, str(unnamed))
    def last(name):
        v = got[name]
        p = v if isinstance(v, dict) else dsp.value([name] + ['nominal-biped-manifold'] * len(v.params), got)
        assert isinstance(p, dict), p
        trace = [step for step, _info in p['_trace']]
        return trace[-2] if trace[-1] == name else trace[-1]
    ends = {n: last(n) for n in got if n.startswith(('patch-', 'clean-'))}
    report.check('every patch ends in it: %d' % len(ends), set(ends.values()) == {'master-anti-spasm-insert'}, str(ends))
    stream = got['clean-runtime-stream']
    supervisor.checked(stream)
    bad = copy.deepcopy(stream)
    bad['zero-crossing']['co-contraction-bias-nm'] = 2.4
    try:
        supervisor.checked(bad)
        refusal = ''
    except MachineError as error:
        refusal = str(error)
    report.check('its text hers; another preload refused by name', 'co-contraction-bias-nm' in refusal, refusal)
    row, _told = voicing.voiced(stream, got['nominal-biped-manifold'], fr)
    run, _told = voicing.voiced(got['patch-sneaker-runway'], got['nominal-biped-manifold'], fr)
    report.check('the stream voices her walk as the runway, its clock its own', set(row) <= set(run), str(sorted(row)))
    from coaxial.model import inverter, sensorless
    from machine import drives, wbc
    names = [j for j in JOINTS if drives.passive(j) is None]
    _k, _s, laid = supervisor.bus(names, 1000.0)
    loop = sensorless.current_loop(0.0, 0.0, inverter.FSW, 0.0, drives.PACK_V)['bw_hz']
    report.check("her bus: each slew its peak x 2 pi x its board's loop, %.0f Hz; its jerk that over its rotor's inertia" % loop,
                 set(drives.LOOP_HZ.values()) == {loop}
                 and all(_near(v, drives.peak(n) * 2.0 * math.pi * loop) for n, v in zip(names, laid['slew']))
                 and all(_near(j, v / drives.armature(n)) for n, v, j in zip(names, laid['slew'], laid['jerk'])),
                 str([(n, round(v / 1e3)) for n, v in zip(names, laid['slew'])]))
    body = wbc.Body()
    rows, (width, stiff, damp) = body.buffers
    ends = sorted((names[i], side) for i, _stop, side in rows)
    report.check('a buffer %.2f rad before each stop of %d joints, critically damped' % (width, len(ends) // 2),
                 ends == sorted((names[i], side) for i in supervisor.joints_of(
                     supervisor.insert()['joint-boundary']['nodes'], names) if i in body.stops for side in (1.0, -1.0))
                 and _near(damp * damp, 4.0 * stiff), str(ends))


def test_the_runway_reads_in_its_own_dialect(report):
    """machine/runway.lisp through the same reader: its dry plays her walk, its wet narrows her
    track by its own ratio, its morph lays a curve row every sixteenth of two bars."""
    import os
    from machine import dsp, modal, ortho, patch, voicing
    got = dsp.read([os.path.join(os.path.dirname(dsp.__file__), 'runway.lisp')])
    m, fr = got['morph-dry-to-catwalk'], _her()
    dry_row, _told = voicing.voiced(m.source, m.source, fr)
    wet_row, _told = voicing.voiced(m.target, m.source, fr)
    wire, span = voicing.morphed(m, m.source, fr)
    rows = [ortho.ROW.unpack_from(wire, k) for k in range(0, len(wire), ortho.ROW.size)]
    report.check('its dry moves nothing, its wet her width by 42 over 115 mm', not dry_row and _near(
        wet_row[ortho.Z + modal.WIDTH], 42.0 / 115.0 - 1.0), str(wet_row))
    report.check('its morph: 33 rows, every one but the last a curve sample, %.2f s' % span,
                 len(rows) == 2 * patch.GRID + 1 and all(r[3] == ortho.CURVE for r in rows[:-1]) and rows[-1][3] == 0,
                 '%d rows' % len(rows))


def test_a_take_converts_to_a_modifier_she_reads(report):
    """Her walk as approved against itself: the converter emits the nominal, read back it moves
    her walk's slots no further than its text's four places and tells her no duck. Its peak
    finder and Welch's estimate read a known signal as scipy's do."""
    import os
    import numpy as np
    from machine import dsp, ortho, voicing
    from tools.sim import identify
    from tools.sim.fbx import take
    peaks = identify.peaks([0.0, 1.0, 0.0, 2.0, 0.0, 1.5, 0.0], 1, 0.5)
    spaced = identify.peaks([0.0, 1.0, 0.0, 2.0, 0.0, 1.5, 0.0], 3, 0.5)
    f, power = identify.welch(np.sin(math.tau * 2.0 * np.arange(1000) / 100.0), 100.0, 256)
    report.check('peaks at 1, 3, 5; 3 apart the highest alone; a 2 Hz sine peaks at 2 Hz',
                 list(peaks) == [1, 3, 5] and list(spaced) == [3] and abs(f[int(np.argmax(power))] - 2.0) < f[1],
                 '%s %s %.2f Hz' % (list(peaks), list(spaced), f[int(np.argmax(power))]))
    got = identify.identified(identify.soa(*take(identify.REFERENCE)))
    text = identify.emitted('take-herself', got, got)
    path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'build', 'patches')
    os.makedirs(path, exist_ok=True)
    path = os.path.join(path, 'take-herself.lisp')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(text)
    suite = dsp.read(dsp.SUITE + [path])
    g = dsp.value(['->', 'nominal-biped-manifold', ['take-herself']], suite)
    fr = _her()
    row, told = voicing.voiced(g, suite['nominal-biped-manifold'], fr)
    walk = voicing.walk(fr)
    report.check('her walk converted against itself: her slots within 1e-4, no duck, ahead',
                 all(abs(v - walk[k]) <= 1e-4 * max(1.0, abs(walk[k])) for k, v in row.items())
                 and ortho.DUCK.start not in row and told['twist'][1] == 0.0, '%s %s' % (row, told))


ROSTER = (test_the_suite_reads_as_written, test_the_maneuvers_read_as_written, test_a_patch_moves_her_walk_by_its_changes,
          test_what_she_cannot_do_is_refused_by_name, test_the_insert_is_every_patch_last,
          test_the_runway_reads_in_its_own_dialect, test_a_take_converts_to_a_modifier_she_reads)


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
