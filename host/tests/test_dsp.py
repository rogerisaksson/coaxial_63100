"""The ortho-DSP suite on her loop (`machine.dsp`): its six modules read and evaluated as written,
a patch voiced on her walk by its changes, what she cannot do refused by name, the runway's
dialect read alike, a take converted to a modifier she reads (`tools/sim/identify.py`)."""
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


def test_a_patch_moves_her_walk_by_its_changes(report):
    """The nominal manifold voiced on itself moves no slot of her walk and tells her walk's speed;
    the runway moves each voiced slot by its change off the nominal, relative to her walk."""
    from machine import dsp, modal, ortho, se2
    got, fr = _got(), _her()
    dry = got['nominal-biped-manifold']
    row, told = dsp.voiced(dry, dry, fr)
    report.check('the nominal is her walk: no slot moved, her walk speed told', not row and told['twist'] == (
        se2.WALK_SPEED, 0.0, 0.0), '%s %s' % (row, told))
    walk = dsp.walk(fr)
    row, told = dsp.voiced(got['patch-sneaker-runway'], dry, fr)
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
    from machine import dsp
    from machine.errors import MachineError
    got, fr = _got(), _her()
    dry = got['nominal-biped-manifold']

    def refusal(g):
        try:
            dsp.voiced(g, dry, fr)
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
    told = [dsp.voiced(g, dry, fr)[1]['twist'] for g in (side, back)]
    report.check('a sidestep told her leftward, a retro walk backward', told[0][1] > 100.0 * abs(told[0][0])
                 and told[1][0] < 0.0, str(told))


def test_the_runway_reads_in_its_own_dialect(report):
    """machine/runway.lisp through the same reader: its dry plays her walk, its wet narrows her
    track by its own ratio, its morph lays a curve row every sixteenth of two bars."""
    import os
    from machine import dsp, modal, ortho, patch
    got = dsp.read([os.path.join(os.path.dirname(dsp.__file__), 'runway.lisp')])
    m, fr = got['morph-dry-to-catwalk'], _her()
    dry_row, _told = dsp.voiced(m.source, m.source, fr)
    wet_row, _told = dsp.voiced(m.target, m.source, fr)
    wire, span = dsp.morphed(m, m.source, fr)
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
    from machine import dsp, ortho
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
    row, told = dsp.voiced(g, suite['nominal-biped-manifold'], fr)
    walk = dsp.walk(fr)
    report.check('her walk converted against itself: her slots within 1e-4, no duck, ahead',
                 all(abs(v - walk[k]) <= 1e-4 * max(1.0, abs(walk[k])) for k, v in row.items())
                 and ortho.DUCK.start not in row and told['twist'][1] == 0.0, '%s %s' % (row, told))


ROSTER = (test_the_suite_reads_as_written, test_a_patch_moves_her_walk_by_its_changes,
          test_what_she_cannot_do_is_refused_by_name, test_the_runway_reads_in_its_own_dialect,
          test_a_take_converts_to_a_modifier_she_reads)


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
