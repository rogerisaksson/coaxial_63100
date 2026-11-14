"""Orthokinematics on her: her latent state stepped, rendered on her, on the loop (its texts:
test_ortho_texts.py)."""
import math
import os
import sys
import time

from tools.dev.focus import chosen
from gynoid_kit import Report

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _text(name):
    with open(os.path.join(HERE, 'lisp', 'humanoid', name + '.lisp'), encoding='utf-8') as f:
        return f.read()


def test_her_latent_state_blends_and_turns(report):
    """A row laid over her rest blends in, steps CPG sine references, and returns to rest on REST."""
    import numpy as np
    from machine.law import latent
    from machine.texts import ortho
    rest = np.zeros(ortho.P)
    rest[latent.K], rest[latent.HZ] = 100.0, 1.0
    tables, _ = ortho.compiled(
        '(defbehavior b (phase p (duration 1s) (blend_duration 100ms) (latent_targets (mode_axial_compression 0.5))'
        ' (impedance (damping mode_lateral_shift 7)) (cpg_config (enable true) (frequency 2Hz)'
        ' (amplitude mode_lateral_shift 0.2) (phase_offset mode_lateral_shift mode_sagittal_drive 90deg))))')
    st, out, dt = latent.state(rest), np.zeros(latent.OUT), 0.001
    tab = latent.table(tables['b'], 0.5, rest)
    seen = {}
    for tick in range(2000):
        now = tick * dt
        latent.step(st, tab, now, dt, out)
        seen[tick] = (float(out[latent.REF][0]), float(out[latent.REF][1]), float(out[latent.DS][1]),
                      float(out[latent.KS][0]), float(st[latent.THETA]), float(out[latent.SWAY][1]))
    report.check('half way through its blend, half way there; at its end, there',
                 abs(seen[549][0] - 0.25) < 0.01 and abs(seen[600][0] - 0.5) < 1e-9 and seen[499][0] == 0.0,
                 'axial %.3f at 50 ms, %.3f at 100' % (seen[549][0], seen[600][0]))
    report.check('what it says is laid, what it leaves stays', abs(seen[700][2] - 7.0) < 1e-9
                 and seen[700][3] == 100.0, 'damping %.1f, stiffness %.1f' % (seen[700][2], seen[700][3]))
    theta, sway = seen[900][4], seen[900][5]
    report.check('her rhythm: the sway its amplitude times the sine of her angle a quarter turn on',
                 abs(sway - 0.2) < 1e-6 and abs(seen[900][1] - 0.2 * math.sin(theta + math.pi / 2)) < 1e-6,
                 'sway %.3f, reference %+.4f' % (sway, seen[900][1]))
    report.check('her rest again after its last phase', abs(seen[1999][0]) < 1e-9 and seen[1999][2] == 0.0
                 and seen[1999][5] == 0.0, 'axial %.3f, sway %.3f' % (seen[1999][0], seen[1999][5]))


def _went(said, text=None, to=2.0):
    from tools.sim import ortho as run
    return run.went(run.planned(said, text), to)


def test_she_crouches_as_asked(report):
    """`crouch`: axial compression asked 0.6 within 0.05, standing, and back to rest."""
    got = _went(['crouch'])
    asked, _far, ended, rest = got['modes']['axial_compression']
    report.check('crouched 0.6 she is within 0.05 of it, and stands', got['stood'] and abs(ended - 0.6) < 0.05,
                 '%.2f of %.2f; tilt %.1f deg' % (ended, asked or 0.6, got['tilt']))
    report.check('at rest after, within 0.05 of where she stood', abs(rest) < 0.05, '%+.3f' % rest)


def test_her_weight_goes_over_one_sole(report):
    """`weight_shift`: lateral shift asked 1.0 within 0.1, the other sole bearing under a fifth."""
    got = _went(['weight_shift'])
    _asked, far, ended, _rest = got['modes']['lateral_shift']
    light, heavy = sorted(got['loads'])
    report.check('shifted 1.0 she is within 0.1 of it, never 0.2 past, and stands',
                 got['stood'] and abs(ended - 1.0) < 0.1 and far < 1.2, '%.2f as it ends, %.2f at its furthest'
                 % (ended, far))
    report.check('her other sole bears under a fifth of her', light < 0.2 * (light + heavy),
                 '%d N of %d' % (light, light + heavy))


def test_she_bows_as_asked(report):
    """`bow`: pitch attitude asked 0.5 within 0.05, standing, upright again at rest."""
    got = _went(['bow'])
    _asked, _far, ended, rest = got['modes']['pitch_attitude']
    report.check('bowed 0.5 she is within 0.05 of it, and stands', got['stood'] and abs(ended - 0.5) < 0.05,
                 '%.2f' % ended)
    report.check('upright again at rest, within 0.05', abs(rest) < 0.05, '%+.3f' % rest)


def test_she_stands_on_one_sole(report):
    """`one_sole`: weight over one sole and the other lifted off the floor."""
    got = _went(['one_sole'])
    _asked, _far, lifted, _rest = got['modes']['limb_lift']
    report.check('her other sole is off the floor, 0.8 of the clearance and more, bearing nothing',
                 lifted < -0.8 and min(got['loads']) < 1.0, 'lift %+.2f, loads %s' % (lifted, got['loads']))
    report.check('she stands on the one, and on both again after', got['stood'] and min(got['rest']) > 20.0,
                 'tilt %.1f deg, at rest %s N' % (got['tilt'], got['rest']))


def test_she_hops_and_stands(report):
    """`hop`: loaded, thrown, landed on soft knees: >= 0.15 s off the floor, standing."""
    import numpy as np
    from machine.law import latent, modal, support
    got = _went(['hop'], to=5.0)
    report.check('she leaves the floor 0.15 s and more, 0.9 m/s up and more',
                 got['air'] >= 0.15 and got['up'] >= 0.9, '%.2f s, %.2f m/s' % (got['air'], got['up']))
    report.check('and stands after, never 15 deg over, both soles bearing',
                 got['stood'] and got['tilt'] < 15.0 and min(got['rest']) > 20.0,
                 'tilt %.1f deg, %s N' % (got['tilt'], got['rest']))
    q, qd, ref_q, ref_v, eso_z = (np.zeros(support.DRIVEN) for _ in range(5))
    out = np.zeros(latent.OUT)
    q[5], eso_z[5] = -0.6, -50.0
    dq = support.couple(q, qd, ref_q, ref_v, eso_z, out)
    report.check('left arm held back: posture yields, ESO bleeds, lateral and turn shift right',
                 dq[5] < -0.35 and abs(eso_z[5]) < 10.0 and out[latent.REF][modal.LATERAL] < -0.08
                 and out[latent.REF][modal.TURN] < -0.14,
                 'dq %.2f, eso %.1f, lat %+.3f, turn %+.3f' % (
                     dq[5], eso_z[5], out[latent.REF][modal.LATERAL], out[latent.REF][modal.TURN]))


def test_she_walks_on_her_own_payloads(report):
    """Payloads stand to walk: 5 m and more in 20 s, >= 0.5 m/s by the end, upright."""
    got = _went(['stand@0.5', 'sway@2', 'step@5', 'walk@9'], to=9.5)
    report.check('she walks 5 m and more, at 0.5 m/s and more by the end',
                 got['stood'] and got['went'] >= 5.0 and got['speed'] >= 0.5,
                 '%.2f m, %.2f m/s, %.1f s' % (got['went'], got['speed'], got['t']))
    report.check('a sole lands every half turn of her rhythm: 24 and more; her pelvis within 12 deg of upright',
                 got['landings'] >= 24 and got['tilt'] < 12.0, '%d landings, tilt %.1f deg' % (got['landings'], got['tilt']))


def test_her_walk_is_none_of_the_gaits_it_is_not_to_be(report):
    """Her walk over 20 s against normal.BAND and looks.FORM."""
    from tools.sim import looks
    from tools.sim import ortho as run
    got = run.looked(run.planned(['stand@0.5', 'sway@2', 'step@5', 'walk@9']), 20.0)
    band = got.get('band', {})
    report.check('she walks 0.5 m/s and more, under 450 J/m, standing',
                 got['stood'] and got.get('speed', 0.0) >= 0.5 and got.get('energy', 1e9) < 450.0,
                 '%.2f m/s, %.0f J/m' % (got.get('speed', 0.0), got.get('energy', 0.0)))
    report.check("off a woman's band under 3.2, by name none of the gaits",
                 got.get('off', 9.0) < 3.2 and not got.get('named'),
                 '%.2f: %s%s' % (got.get('off', 9.0), ', '.join('%s %.2f' % w for w in got.get('words', ())),
                                 ''.join(' - ' + n for n in got.get('named', ()))))
    report.check('her heel 20 deg and more up as her toes leave, her toes 5 up as she lands',
                 band.get('heel at lift', 0.0) >= 20.0 and band.get('toes up at landing', 0.0) >= 5.0,
                 'heel %.1f, toes %.1f deg' % (band.get('heel at lift', 0.0), band.get('toes up at landing', 0.0)))
    report.check('her pelvis rolls 4 deg and turns 5 deg and more, her arms swing 20, her soles 0.1 to 0.2 legs apart',
                 band.get('pelvis roll', 0.0) >= 4.0 and band.get('pelvis turn', 0.0) >= 5.0
                 and band.get('arm', 0.0) >= 20.0 and 0.1 <= band.get('feet apart', 0.0) <= 0.2,
                 'roll %.1f, turn %.1f, arm %.1f deg, %.2f legs' % (
                     band.get('pelvis roll', 0.0), band.get('pelvis turn', 0.0), band.get('arm', 0.0),
                     band.get('feet apart', 0.0)))
    nod = got.get('measures', {}).get('head nod', 99.0)
    report.check('her head nods %g deg and under a stride' % looks.NODS_DEG, nod <= 40.0, '%.1f deg' % nod)


def test_the_first_texts_as_given(report):
    """lisp/humanoid/examples.lisp: 101, 102 and 103 walk her - her height as 102 asks it over her whole
    strides, its swing within them a woman's pelvis's bob -; hop_asymmetric leaves the floor."""
    from tools.sim import normal, ortho as run
    text = _text('examples')
    said = ['#101@1', '#102@3', '#103@6']
    got = _went(said, text, to=5.8)
    report.check('101, 102 and 103 as given: she walks 1.5 m and more in 9 s, within 8 deg of upright',
                 got['stood'] and got['went'] >= 1.5 and got['tilt'] < 8.0,
                 'tilt %.1f deg, %.2f m, %d landings' % (got['tilt'], got['went'], got['landings']))
    held = got['strode'].get('axial_compression', math.nan)
    report.check('her height as 102 asks it, 0.25 compressed within 0.08 over her whole strides',
                 got['strides'] >= 2 and abs(held - 0.25) < 0.08, '%.3f over %d strides; asked, furthest, ended, '
                 'at the end %s' % (held, got['strides'], got['modes']['axial_compression']))
    bob = run.looked(run.planned(said, text), 5.8).get('band', {}).get('pelvis bob', math.nan)
    _name, _unit, low, high = next(b for b in normal.BAND if b[0] == 'pelvis bob')
    report.check("and its swing a woman's: her pelvis bobs %g to %g legs a stride" % (low, high),
                 low <= bob <= high, '%.3f legs' % bob)
    hop = _went(['hop_asymmetric'], text, to=0.3)
    far = hop['modes']['axial_compression'][1]
    report.check('the hop as given loads to 0.75 and past it, goes up 1 m/s and more, 0.1 s off the floor',
                 far >= 0.75 and hop['up'] >= 1.0 and hop['air'] >= 0.1,
                 'compressed %.2f, %.2f m/s, %.2f s' % (far, hop['up'], hop['air']))


def test_the_gym_runs_her_in_a_process(report):
    """`sprung.Running`, the page's body: her state said with her modes and what the digits
    say; an entry said, she does it; the gym's other text taken; a text the compiler refuses
    and a command the loop does not know said with her state."""
    from machine.process import sprung
    from terminal.views import gym
    body = sprung.Running(walks=False)
    try:
        first, _ = _until(body, lambda state: True)
        says = first.get('says', ()) if first else ()
        report.check('her state comes with her modes and what the digits say',
                     first is not None and len(first['modes']) == len(gym.SHORT) and says[:1] == ('gynoid_mesomorph_medium',)
                     and 'crouch' in says,
                     '%s' % (says,))
        body.send(say=says.index('crouch') - 1 if 'crouch' in says else 0)
        state, took = _until(body, lambda state: state['modes'][0][2] >= 0.5, 30.0)
        deep = state['modes'][0][2] if state else 0.0
        report.check('the crouch said, she is down past 0.5', deep >= 0.5, '%.2f in %.1f s of hers' % (deep, took))
        body.send(gym='examples')
        state, _ = _until(body, lambda state: state.get('says', ())[:1] == ('examples',), 10.0)
        other = state.get('says', ()) if state else ()
        report.check("the gym's other text taken: the first texts", other[:2] == ('examples', '#101'), '%s' % (other,))
        body.send(text='(stream_inject (latent_targets (mode_wing_flap 1)))')
        state, _ = _until(body, lambda state: bool(state.get('refused')), 10.0)
        refused = state.get('refused') if state else None
        report.check('a text the compiler refuses is said with her state, and she stands on',
                     bool(refused) and 'mode_wing_flap' in refused, '%s' % (refused or '')[:60])
        body.send(pace=0.5)
        state, _ = _until(body, lambda state: 'pace' in (state.get('refused') or ''), 10.0)
        refused = state.get('refused') if state else None
        report.check('a command the loop does not know is refused by name: nothing else runs her',
                     bool(refused) and 'pace' in refused, '%s' % (refused or '')[:70])
    finally:
        body.close()


def _until(body, done, hers=math.inf):
    """Her `body`'s state once `done`, and the s it took - while she lives, `hers` s of her time, not the wall's."""
    state, start = None, None
    while body.alive():
        state = body.latest() or state
        if state is not None:
            start = start if start and start[0] == state['epoch'] else (state['epoch'], state['t'])
            if done(state) or state['t'] - start[1] >= hers:
                return state, state['t'] - start[1]
        time.sleep(0.05)
    return state, 0.0


def test_the_pages_keys_on_the_loop(report):
    """Her page's keys (`humanoid_keys.PAD`): a digit says its entry, Shift+P shoves her along the
    view, > takes the other text, C crouches her; LATENT and SAY are her state's (`gym`)."""
    import types
    from terminal.views import gym, humanoid_keys
    from views_kit import typed
    sent = []
    body = types.SimpleNamespace(send=lambda **command: sent.append(command), close=lambda: None)
    typed(humanoid_keys.PAD, '13P>c', {'body': body, 'yaw': 180.0})
    report.check('1 and 3 say the first and third, Shift+P 80 N along the view, > the other text, C a crouch',
                 sent[:2] == [{'say': 0}, {'say': 2}] and abs(sent[2]['push'][2] - 80.0) < 1e-9
                 and sent[3] == {'gym': 'examples'} and sent[4] == {'say': 'crouch'}, '%s' % sent)
    modes = [(name, 0.1, 0.2, 3000.0, 60.0) for name in gym.SHORT]
    boxes = gym.boxes({'modes': modes, 'says': ('gynoid_mesomorph_medium', 'stand', 'crouch')})
    report.check('LATENT and SAY of her state; none before it', len(boxes) == 2 and gym.boxes(None) == []
                 and gym.status({'fallen': False, 'doing': 'crouch'}) == 'LATENT  crouch')
    report.check('a text refused is said in her status',
                 gym.status({'fallen': False, 'doing': 'rest', 'refused': 'a ( never closed'}).startswith('REFUSED'))


ROSTER = [test_her_latent_state_blends_and_turns,
          test_she_crouches_as_asked, test_her_weight_goes_over_one_sole, test_she_bows_as_asked,
          test_she_stands_on_one_sole, test_she_hops_and_stands, test_she_walks_on_her_own_payloads,
          test_her_walk_is_none_of_the_gaits_it_is_not_to_be, test_the_first_texts_as_given,
          test_the_gym_runs_her_in_a_process, test_the_pages_keys_on_the_loop]


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
