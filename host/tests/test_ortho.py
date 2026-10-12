"""Orthokinematics: a text compiled to rows, her latent state stepped, rendered on her, on the loop.

`machine.ortho` compiles; `machine.latent` steps without a branch; `machine.impedance` renders -
her modes read by `machine.modal`, her soles by `machine.poise`, her stride by `machine.stride` -; the
page's gym runs it (`machine.sprung`): crouched, her weight over one sole, bowed, on one sole,
each as its text asks and standing; walking on her own payloads, and how that walk looks; the
first texts as given; the gym's worker and keys."""
import ast
import math
import os
import sys
import time

from tools.dev.focus import chosen
from gynoid_kit import Report

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _text(name):
    with open(os.path.join(HERE, 'machine', name + '.lisp'), encoding='utf-8') as f:
        return f.read()


def _same(got, want):
    return len(got) == len(want) and all(
        (math.isnan(a) and math.isnan(b)) or abs(a - b) < 1e-6 * max(1.0, abs(b)) for a, b in zip(got, want))


def test_a_text_compiles_to_rows(report):
    """The first texts as given: a payload a row of ROW.size bytes - its id, its blend, its
    parameters, NaN where it says nothing -, a behaviour its phases' rows each at its ms and a
    last row back to her rest; MAX_SYS_K a number."""
    from machine import ortho
    nan, n = math.nan, ortho.N
    tables, phases = ortho.compiled(_text('examples'))
    stream = ortho.rows(tables[''])
    report.check('four payloads, %d bytes a row' % ortho.ROW.size,
                 len(tables['']) == 4 * ortho.ROW.size and [r[0] for r in stream] == [101, 102, 103, 104],
                 '%d bytes, ids %s' % (len(tables['']), [r[0] for r in stream]))
    p = stream[1][4:]
    k, d, amp, phase = (p[at:at + n] for at in (ortho.K, ortho.D, ortho.AMP, ortho.PHASE))
    report.check('102: blended 800 ms; axial 0.25; lateral 1 200 N/m and 15 N s/m; her rhythm on at 1.1 Hz, '
                 'the drive 0.4, the sway 0.35 a quarter turn ahead; the rest unsaid',
                 stream[1][1:4] == (0, 800, 0) and _same(p[:n], (0.25,) + (nan,) * (n - 1))
                 and _same(k, (nan, 1200.0) + (nan,) * (n - 2)) and _same(d, (nan, 15.0) + (nan,) * (n - 2))
                 and _same(amp, (nan, 0.35, 0.4) + (nan,) * (n - 3))
                 and _same(phase, (nan, math.pi / 2) + (nan,) * (n - 2))
                 and _same(p[ortho.GAIN:], (1.0, 1.1, nan, nan)), '%s' % (stream[1][:4],))
    pre, go, rest = ortho.rows(tables['hop_asymmetric'])
    report.check('the hop: pre_load at 0 ms, propulsion at 400, her rest at 550 blended 200',
                 phases['hop_asymmetric'] == ('pre_load', 'propulsion')
                 and (pre[1], go[1], rest[1:4]) == (0, 400, (550, ortho.REST_MS, ortho.REST)),
                 '%s %s %s' % (pre[:4], go[:4], rest[:4]))
    report.check('pre_load: compressed 0.8, shifted 0.5, upright; 4 500 N/m, 25 N s/m; her rhythm off',
                 _same(pre[4:4 + n], (0.8, 0.5, nan, 0.0) + (nan,) * (n - 4))
                 and pre[4 + ortho.K] == 4500.0 and pre[4 + ortho.D] == 25.0 and pre[4 + ortho.GAIN] == 0.0)
    report.check('propulsion: extended -1.0 at MAX_SYS_K undamped, her ZMP in her support polygon',
                 go[4] == -1.0 and go[4 + ortho.K] == ortho.SYMBOLS['MAX_SYS_K'] and go[4 + ortho.D] == 0.0
                 and go[4 + ortho.ZMP] == 1.0, 'stiffness %g' % go[4 + ortho.K])
    hers, named = ortho.compiled(_text('moves'))
    report.check('machine/moves.lisp compiles: her walk and her behaviours',
                 len(hers['']) == 5 * ortho.ROW.size
                 and set(named) >= {'crouch', 'weight_shift', 'bow', 'hop', 'one_sole'},
                 ', '.join(named))


def test_what_is_no_text_is_refused(report):
    """A mode she has not, a target past 1, a stiffness under 0, a phase with no duration, a
    time with no unit, a clause unknown, a behaviour said twice, a form left open or of no
    kind: each refused in words."""
    from machine import ortho
    phase = '(defbehavior b (phase p (duration 1s) %s))'
    texts = (phase % '(latent_targets (mode_wing_flap 0.5))',
             phase % '(latent_targets (mode_axial_compression 1.5))',
             phase % '(impedance (stiffness mode_axial_compression -1))',
             '(defbehavior b (phase p (latent_targets (mode_lateral_shift 0.5))))',
             phase % '(blend_duration 400)',
             '(stream_inject (cpg_config (frequency 1.1)))',
             phase % '(motor left_knee 30)',
             phase % '(cpg_override (dance))',
             phase % '(task_constraints (maintain_altitude 3))',
             '(stream_inject (duration 1s))',
             phase % '' + phase % '',
             '(defbehavior b (phase p (duration 1s))',
             '(behavior b)')
    refused = []
    for text in texts:
        try:
            ortho.compiled(text)
        except SyntaxError as error:
            refused.append(str(error))
    report.check('%d texts are refused, each saying why' % len(texts),
                 len(refused) == len(texts) and all(len(r) > 12 for r in refused),
                 '%d refused: %s' % (len(refused), ' | '.join(r[:32] for r in refused)))


def test_the_core_and_the_renderer_branch_nowhere(report):
    """machine/latent.py, impedance.py, stride.py and poise.py: no if, no conditional expression,
    no while, no try, no and/or, no class, no lambda, no loop, no dict. machine/modal.py: none of
    the first seven, its dicts the stack's ask alone."""
    branches = (ast.If, ast.IfExp, ast.While, ast.Match, ast.Try, ast.BoolOp, ast.ClassDef, ast.Lambda, ast.With)
    found = {}
    pure = ('latent', 'impedance', 'stride', 'poise')
    for name in pure + ('modal',):
        with open(os.path.join(HERE, 'machine', name + '.py'), encoding='utf-8') as f:
            tree = ast.parse(f.read())
        found[name] = (['%s:%d' % (type(node).__name__, getattr(node, 'lineno', 0)) for node in ast.walk(tree)
                        if isinstance(node, branches) or (isinstance(node, ast.comprehension) and node.ifs)],
                       [node for node in ast.walk(tree) if isinstance(node, (ast.For, ast.Dict, ast.DictComp))],
                       tree)
    for name in pure:
        report.check('%s.py: no branch, no loop, no dict' % name, not found[name][0] and not found[name][1],
                     ', '.join(found[name][0]) or 'none')
    given = next(node for node in found['modal'][2].body
                 if isinstance(node, ast.FunctionDef) and node.name == 'given')
    dicts = [node.lineno for node in found['modal'][1] if isinstance(node, (ast.Dict, ast.DictComp))]
    report.check("modal.py: no branch; its dicts the stack's ask alone",
                 not found['modal'][0] and all(given.lineno <= at <= (given.end_lineno or 0) for at in dicts),
                 ', '.join(found['modal'][0]) or 'none; dicts at %s' % dicts)


def test_her_latent_state_blends_and_turns(report):
    """A row laid over her rest: what it says goes there in a straight line over its blend,
    what it leaves stays; her rhythm's reference is its amplitude's sine at her angle and the
    mode's phase; a row flagged REST is her rest again."""
    import numpy as np
    from machine import latent, ortho
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
    """`crouch`: her axial compression asked 0.6: there within 0.05 as it ends, standing, and at
    rest within 0.05 of where she stood."""
    got = _went(['crouch'])
    asked, _far, ended, rest = got['modes']['axial_compression']
    report.check('crouched 0.6 she is within 0.05 of it, and stands', got['stood'] and abs(ended - 0.6) < 0.05,
                 '%.2f of %.2f; tilt %.1f deg' % (ended, asked or 0.6, got['tilt']))
    report.check('at rest after, within 0.05 of where she stood', abs(rest) < 0.05, '%+.3f' % rest)


def test_her_weight_goes_over_one_sole(report):
    """`weight_shift`: her lateral shift asked 1.0: over that sole within 0.1, never 0.2 past
    it, the other bearing under a fifth of her, standing."""
    got = _went(['weight_shift'])
    _asked, far, ended, _rest = got['modes']['lateral_shift']
    light, heavy = sorted(got['loads'])
    report.check('shifted 1.0 she is within 0.1 of it, never 0.2 past, and stands',
                 got['stood'] and abs(ended - 1.0) < 0.1 and far < 1.2, '%.2f as it ends, %.2f at its furthest'
                 % (ended, far))
    report.check('her other sole bears under a fifth of her', light < 0.2 * (light + heavy),
                 '%d N of %d' % (light, light + heavy))


def test_she_bows_as_asked(report):
    """`bow`: her pitch attitude asked 0.5: there within 0.05, standing, upright again at rest."""
    got = _went(['bow'])
    _asked, _far, ended, rest = got['modes']['pitch_attitude']
    report.check('bowed 0.5 she is within 0.05 of it, and stands', got['stood'] and abs(ended - 0.5) < 0.05,
                 '%.2f' % ended)
    report.check('upright again at rest, within 0.05', abs(rest) < 0.05, '%+.3f' % rest)


def test_she_stands_on_one_sole(report):
    """`one_sole`: her weight asked over one sole and the other off the floor, in one row: the
    other leaves once her ZMP is in the first's, bears nothing, and she stands."""
    got = _went(['one_sole'])
    _asked, _far, lifted, _rest = got['modes']['limb_lift']
    report.check('her other sole is off the floor, 0.8 of the clearance and more, bearing nothing',
                 lifted < -0.8 and min(got['loads']) < 1.0, 'lift %+.2f, loads %s' % (lifted, got['loads']))
    report.check('she stands on the one, and on both again after', got['stood'] and min(got['rest']) > 20.0,
                 'tilt %.1f deg, at rest %s N' % (got['tilt'], got['rest']))


def test_she_hops_and_stands(report):
    """`hop`: loaded, thrown by a spring that lets go under her standing height with her trunk
    free, landed on soft knees: 0.15 s and more off the floor, never 15 deg over, standing."""
    got = _went(['hop'], to=2.5)
    report.check('she leaves the floor 0.15 s and more, 0.9 m/s up and more',
                 got['air'] >= 0.15 and got['up'] >= 0.9, '%.2f s, %.2f m/s' % (got['air'], got['up']))
    report.check('and stands after, never 15 deg over, both soles bearing',
                 got['stood'] and got['tilt'] < 15.0 and min(got['rest']) > 20.0,
                 'tilt %.1f deg, %s N' % (got['tilt'], got['rest']))


def test_she_walks_on_her_own_payloads(report):
    """machine/moves.lisp's 201 to 204 - stand, sway, step, walk on -: 5 m and more in 20 s, at
    0.5 m/s and more by the end, a sole landing every half turn of her rhythm, her pelvis
    never 8 deg off upright."""
    got = _went(['#201@0.5', '#202@2', '#203@5', '#204@9'], to=9.5)
    report.check('she walks 5 m and more, at 0.5 m/s and more by the end',
                 got['stood'] and got['went'] >= 5.0 and got['speed'] >= 0.5,
                 '%.2f m, %.2f m/s, %.1f s' % (got['went'], got['speed'], got['t']))
    report.check('a sole lands every half turn of her rhythm: 24 and more; her pelvis within 8 deg of upright',
                 got['landings'] >= 24 and got['tilt'] < 8.0, '%d landings, tilt %.1f deg' % (got['landings'], got['tilt']))


def test_her_walk_is_none_of_the_gaits_it_is_not_to_be(report):
    """Her walk as tools/sim/look.py measures one, 20 s of it: off a woman's band
    (`normal.BAND`) under 3.2 - 7.7 on a level centre of mass and flat soles, on stilts,
    Groucho's and a shuffle by name -, none of those names; her heel up as her toes leave, her
    toes up as she lands, her pelvis listing and turning, her arms swinging, her soles a
    woman's width apart. Her knee lands bent yet and her step is short (docs/TODO.md)."""
    from tools.sim import ortho as run
    got = run.looked(run.planned(['#201@0.5', '#202@2', '#203@5', '#204@9']), 20.0)
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
    report.check('her pelvis rolls and turns 5 deg and more, her arms swing 20, her soles 0.1 to 0.2 legs apart',
                 band.get('pelvis roll', 0.0) >= 5.0 and band.get('pelvis turn', 0.0) >= 5.0
                 and band.get('arm', 0.0) >= 20.0 and 0.1 <= band.get('feet apart', 0.0) <= 0.2,
                 'roll %.1f, turn %.1f, arm %.1f deg, %.2f legs' % (
                     band.get('pelvis roll', 0.0), band.get('pelvis turn', 0.0), band.get('arm', 0.0),
                     band.get('feet apart', 0.0)))


def test_the_first_texts_as_given(report):
    """machine/examples.lisp as it was written: 101 stands her relaxed; 102 begins the gait,
    0.25 compressed, and 103 keeps it: she walks - its sway is asked past what her soles'
    width gives at 1.1 Hz, and where a sole lands meets her capture point wherever that goes -;
    the hop loads to 0.75 and leaves the floor. Its landing is not held (docs/TODO.md,
    orthokinematics)."""
    text = _text('examples')
    got = _went(['#101@1', '#102@3', '#103@6'], text, to=5.8)
    report.check('101, 102 and 103 as given: she walks 1.5 m and more in 9 s, within 6 deg of upright',
                 got['stood'] and got['went'] >= 1.5 and got['tilt'] < 6.0,
                 'tilt %.1f deg, %.2f m, %d landings' % (got['tilt'], got['went'], got['landings']))
    report.check('her height as 102 asks it, 0.25 compressed within 0.08',
                 abs(got['modes']['axial_compression'][3] - 0.25) < 0.08, '%s' % (
                     got['modes']['axial_compression'],))
    hop = _went(['hop_asymmetric'], text, to=0.3)
    far = hop['modes']['axial_compression'][1]
    report.check('the hop as given loads to 0.75 and past it, goes up 1 m/s and more, 0.1 s off the floor',
                 far >= 0.75 and hop['up'] >= 1.0 and hop['air'] >= 0.1,
                 'compressed %.2f, %.2f m/s, %.2f s' % (far, hop['up'], hop['air']))


def test_the_gym_runs_her_in_a_process(report):
    """`sprung.Running`, the page's body: her state said with her modes and what the digits
    say; an entry said, she does it; the gym's other text taken; a text the compiler refuses
    and a command the loop does not know said with her state."""
    from machine import sprung
    from terminal.views import gym
    body = sprung.Running(walks=False)
    try:
        began, first = time.monotonic(), None
        while first is None and time.monotonic() - began < 30.0:
            first = body.latest()
            time.sleep(0.05)
        says = first.get('says', ()) if first else ()
        report.check('her state comes with her modes and what the digits say',
                     first is not None and len(first['modes']) == len(gym.SHORT) and says[:1] == ('moves',)
                     and 'crouch' in says,
                     '%s' % (says,))
        body.send(say=says.index('crouch') - 1 if 'crouch' in says else 0)
        deep, began = 0.0, time.monotonic()
        while deep < 0.5 and time.monotonic() - began < 30.0:
            state = body.latest()
            deep = max(deep, state['modes'][0][2] if state else 0.0)
            time.sleep(0.05)
        report.check('the crouch said, she is down past 0.5', deep >= 0.5, '%.2f in %.1f s' % (
            deep, time.monotonic() - began))
        body.send(gym='examples')
        began, other = time.monotonic(), ()
        while other[:1] != ('examples',) and time.monotonic() - began < 10.0:
            state = body.latest()
            other = state.get('says', ()) if state else ()
            time.sleep(0.05)
        report.check("the gym's other text taken: the first texts", other[:2] == ('examples', '#101'), '%s' % (other,))
        body.send(text='(stream_inject (latent_targets (mode_wing_flap 1)))')
        began, refused = time.monotonic(), None
        while not refused and time.monotonic() - began < 10.0:
            state = body.latest()
            refused = state.get('refused') if state else None
            time.sleep(0.05)
        report.check('a text the compiler refuses is said with her state, and she stands on',
                     bool(refused) and 'mode_wing_flap' in refused, '%s' % (refused or '')[:60])
        body.send(pace=0.5)
        began, refused = time.monotonic(), None
        while not (refused and 'pace' in refused) and time.monotonic() - began < 10.0:
            state = body.latest()
            refused = state.get('refused') if state else None
            time.sleep(0.05)
        report.check('a command the loop does not know is refused by name: nothing else runs her',
                     bool(refused) and 'pace' in refused, '%s' % (refused or '')[:70])
    finally:
        body.close()


def test_the_pages_keys_on_the_loop(report):
    """`terminal.views.gym`: a digit says its entry, W walks her from rest, P shoves her, > takes
    the other text; LATENT and SAY are her state's."""
    import types
    from terminal.views import gym
    sent = []
    body = types.SimpleNamespace(send=lambda **command: sent.append(command), close=lambda: None)
    keys = gym.table({})
    state: dict = {'body': body, 'side': 1.0}
    for key in '13p>w':
        keys[key](state)
    report.check('1 and 3 say the first and third, P a shove across, > the other text, W her walk',
                 sent[:2] == [{'say': 0}, {'say': 2}] and sent[2]['push'][0] != 0.0 and sent[2]['push'][2] == 0.0
                 and sent[3] == {'gym': 'examples'} and sent[4] == {'walk': True}, '%s' % sent)
    modes = [(name, 0.1, 0.2, 3000.0, 60.0) for name in gym.SHORT]
    boxes = gym.boxes({'modes': modes, 'says': ('moves', '#201', 'crouch')})
    report.check('LATENT and SAY of her state; none before it', len(boxes) == 2 and gym.boxes(None) == []
                 and gym.status({'fallen': False, 'doing': 'crouch'}) == 'LATENT  crouch')
    report.check('a text refused is said in her status',
                 gym.status({'fallen': False, 'doing': 'rest', 'refused': 'a ( never closed'}).startswith('REFUSED'))


ROSTER = [test_a_text_compiles_to_rows, test_what_is_no_text_is_refused,
          test_the_core_and_the_renderer_branch_nowhere, test_her_latent_state_blends_and_turns,
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
