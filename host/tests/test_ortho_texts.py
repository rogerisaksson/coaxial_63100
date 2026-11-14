"""Orthokinematics' texts: compiled to rows, what is no text refused, and her law's code branchless."""
import ast
import math
import os
import sys

from tools.dev.focus import chosen
from gynoid_kit import Report, same

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _lisp(name):
    """lisp/humanoid/`name`.lisp's text."""
    from machine.body import mechatronics
    with open(os.path.join(mechatronics.HUMANOID, name + '.lisp'), encoding='utf-8') as f:
        return f.read()


def test_a_text_compiles_to_rows(report):
    """The first texts as given: a payload a row of ROW.size bytes, a behaviour its phases' rows."""
    from machine.texts import ortho
    nan, n = math.nan, ortho.N
    tables, phases = ortho.compiled(_lisp('examples'))
    stream = ortho.rows(tables[''])
    report.check('four payloads, %d bytes a row' % ortho.ROW.size,
                 len(tables['']) == 4 * ortho.ROW.size and [r[0] for r in stream] == [101, 102, 103, 104],
                 '%d bytes, ids %s' % (len(tables['']), [r[0] for r in stream]))
    p = stream[1][4:]
    k, d, amp, phase = (p[at:at + n] for at in (ortho.K, ortho.D, ortho.AMP, ortho.PHASE))
    report.check('102: blended 800 ms; axial 0.25; lateral 1 200 N/m and 15 N s/m; her rhythm on at 1.1 Hz',
                 stream[1][1:4] == (0, 800, 0) and same(p[:n], (0.25,) + (nan,) * (n - 1))
                 and same(k, (nan, 1200.0) + (nan,) * (n - 2)) and same(d, (nan, 15.0) + (nan,) * (n - 2))
                 and same(amp, (nan, 0.35, 0.4) + (nan,) * (n - 3))
                 and same(phase, (nan, math.pi / 2) + (nan,) * (n - 2))
                 and same(p[ortho.GAIN:ortho.YAW + 1], (1.0, 1.1, nan, nan))
                 and same(p[ortho.DUCK], (nan,) * 3), '%s' % (stream[1][:4],))
    pre, go, rest = ortho.rows(tables['hop_asymmetric'])
    report.check('the hop: pre_load at 0 ms, propulsion at 400, her rest at 550 blended 200',
                 phases['hop_asymmetric'] == ('pre_load', 'propulsion')
                 and (pre[1], go[1], rest[1:4]) == (0, 400, (550, ortho.REST_MS, ortho.REST)),
                 '%s %s %s' % (pre[:4], go[:4], rest[:4]))
    report.check('pre_load: compressed 0.8, shifted 0.5, upright; 4 500 N/m, 25 N s/m; her rhythm off',
                 same(pre[4:4 + n], (0.8, 0.5, nan, 0.0) + (nan,) * (n - 4))
                 and pre[4 + ortho.K] == 4500.0 and pre[4 + ortho.D] == 25.0 and pre[4 + ortho.GAIN] == 0.0)
    report.check('propulsion: extended -1.0 at MAX_SYS_K undamped, her ZMP in her support polygon',
                 go[4] == -1.0 and go[4 + ortho.K] == ortho.SYMBOLS['MAX_SYS_K'] and go[4 + ortho.D] == 0.0
                 and go[4 + ortho.ZMP] == 1.0, 'stiffness %g' % go[4 + ortho.K])
    hers, named = ortho.compiled(_lisp('gynoid_mesomorph_medium'))
    base_t, base_p = ortho.compiled(_lisp('gait_base'))
    post_t, post_p = ortho.compiled(_lisp('postures_base'))
    stunt_t, stunt_p = ortho.compiled(_lisp('motion'))
    report.check('lisp/humanoid/gynoid_mesomorph_medium.lisp compiles: her walk and her behaviours',
                 len(hers['']) == 5 * ortho.ROW.size and ortho.compiled() == (hers, named)
                 and base_t[''] == hers[''] and set(base_p) < set(post_p) < set(stunt_p) < set(named)
                 and ortho.rows(post_t['hop'])[1][4 + ortho.K] == 20000.0
                 and ortho.rows(hers['hop'])[1][4 + ortho.K] == 21800.0
                 and set(named) >= {'crouch', 'weight_shift', 'bow', 'hop', 'one_sole',
                                    'strafe_right', 'turn_right', 'circle_walk', 'hop_up_chair'},
                 ', '.join(named))
    sl, sr = ortho.rows(hers['strafe_left'])[0], ortho.rows(hers['strafe_right'])[0]
    oo_t, oo_p = ortho.compiled(
        '(deftrait t1 (latent_targets (mode_axial_compression 0.2) (mode_lateral_shift 0.5))'
        ' (cpg_override (halt_oscillators)))'
        '(deftrait t_clear (extends t1) (latent_targets (mode_lateral_shift none)))'
        '(defbehavior b1 (extends t1) (phase p1 (duration 300ms) (latent_targets (mode_axial_compression 0.6)))'
        ' (phase p2 (use p1) (duration 500ms) (latent_targets (mode_axial_compression 0.3))))'
        '(defbehavior b2 (extends b1) (phase p2 (latent_targets (mode_axial_compression 0.4) (mode_lateral_shift unsaid))))'
        '(defbehavior b_reord (extends b1) (phase p2 (duration 200ms)) (phase p1 (duration 400ms)))'
        '(defbehavior b_mro (extends t_clear b1))')
    b2_r0, b2_r1, _b2_rest = ortho.rows(oo_t['b2'])
    br_r0, br_r1, _br_rest = ortho.rows(oo_t['b_reord'])
    bm_r0, _bm_r1, _bm_rest = ortho.rows(oo_t['b_mro'])
    report.check('inheritance: derive, (deftrait ..), (use ..), MRO, phase reordering and none/unsaid clearing',
                 sl[:4] == sr[:4] and abs(sl[4 + 1] - 0.08) < 1e-6 and abs(sr[4 + 1] + 0.08) < 1e-6
                 and same(sl[4 + 2:], sr[4 + 2:]) and oo_p['b2'] == ('p1', 'p2')
                 and (b2_r0[1], b2_r1[1]) == (0, 300)
                 and abs(b2_r0[4] - 0.6) < 1e-6 and abs(b2_r1[4] - 0.4) < 1e-6
                 and abs(b2_r0[5] - 0.5) < 1e-6 and math.isnan(b2_r1[5])
                 and b2_r0[4 + ortho.GAIN] == 0.0 and b2_r1[4 + ortho.GAIN] == 0.0
                 and oo_p['b_reord'] == ('p2', 'p1') and (br_r0[1], br_r1[1]) == (0, 200)
                 and abs(br_r0[4] - 0.3) < 1e-6 and abs(br_r1[4] - 0.6) < 1e-6
                 and abs(bm_r0[4] - 0.6) < 1e-6)


def test_what_is_no_text_is_refused(report):
    """Invalid modes, targets, stiffnesses, missing durations/units, and malformed OO/module forms are refused."""
    from machine.texts import ortho
    phase = '(defbehavior b (phase p (duration 1s) %s))'
    texts = (phase % '(latent_targets (mode_wing_flap 0.5))',
             phase % '(latent_targets (mode_axial_compression 6.5))',
             phase % '(impedance (stiffness mode_axial_compression -1))',
             '(defbehavior b (phase p (latent_targets (mode_lateral_shift 0.5))))',
             phase % '(blend_duration 400)',
             '(stream_inject (cpg_config (frequency 1.1)))',
             phase % '(motor left_knee 30)',
             phase % '(cpg_override (dance))',
             phase % '(task_constraints (maintain_altitude 3))',
             '(stream_inject (duration 1s))',
             phase % '' + phase % '',
             '(derive b2 unknown_base (phase p (duration 1s)))',
             '(defbehavior b (extends unknown_base) (phase p (duration 1s)))',
             '(defbehavior b (extends) (phase p (duration 1s)))',
             phase % '(use unknown_trait)',
             '(deftrait t (duration 1s))(deftrait t (duration 1s))',
             '(deftrait t (duration 1s))(stream_inject (use t))',
             '(defbehavior b1 (phase p1 (duration 1s)) (phase p2 (duration 1s)))'
             '(defbehavior b2 (extends b1) (phase typo (duration 1s)))',
             '(defbehavior b (phase p (duration 1s)) (phase p (duration 1s)))',
             '(include "no_such_module.lisp")',
             '(include)',
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
    """latent.py, impedance.py, swing.py, stride.py, treads.py, poise.py, fours.py, groundwork.py, substrate.py, crawl.py, bedside.py, modal.py, touch.py, motion.py and grip.py: branchless AST verification."""
    branches = (ast.If, ast.IfExp, ast.While, ast.Match, ast.Try, ast.BoolOp, ast.ClassDef, ast.Lambda, ast.With)
    found = {}
    pure = ('law/latent', 'law/impedance', 'law/swing', 'law/stride', 'law/treads', 'law/vault', 'law/rocker', 'law/poise', 'law/support', 'moves/fours',
            'world/groundwork', 'world/substrate', 'moves/crawl', 'moves/bedside')
    inner = ('senses/touch', 'law/motion', 'senses/grip')
    for name in pure + ('law/modal',) + inner:
        with open(os.path.join(HERE, 'machine', *name.split('/')) + '.py', encoding='utf-8') as f:
            tree = ast.parse(f.read())
        found[name] = (['%s:%d' % (type(node).__name__, getattr(node, 'lineno', 0)) for node in ast.walk(tree)
                        if isinstance(node, branches) or (isinstance(node, ast.comprehension) and node.ifs)],
                       [node for node in ast.walk(tree) if isinstance(node, (ast.For, ast.Dict, ast.DictComp))],
                       tree)
    for name in pure:
        report.check('%s.py: no branch, no loop, no dict' % name, not found[name][0] and not found[name][1],
                     ', '.join(found[name][0]) or 'none')
    for name in inner:
        report.check('%s.py: no branch' % name, not found[name][0], ', '.join(found[name][0]) or 'none')
    modal = found['law/modal']
    given = next(node for node in modal[2].body if isinstance(node, ast.FunctionDef) and node.name == 'given')
    dicts = [node.lineno for node in modal[1] if isinstance(node, (ast.Dict, ast.DictComp))]
    report.check("modal.py: no branch; its dicts the stack's ask alone",
                 not modal[0] and all(given.lineno <= at <= (given.end_lineno or 0) for at in dicts),
                 ', '.join(modal[0]) or 'none; dicts at %s' % dicts)


ROSTER = [test_a_text_compiles_to_rows, test_what_is_no_text_is_refused, test_the_core_and_the_renderer_branch_nowhere]


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
