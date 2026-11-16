#!/usr/bin/env python3
"""The static meta-compiler held to itself: one source a number, the same answer twice, similarity.

The world's gravity is stated in `world.toml` alone; a build compiled anew is the build its cache
holds; an override moves the one number it names; a figure scaled keeps its angles and shares and
moves its lengths no further than its scale; a joint dressed in nothing is bare; no compiled
number is not a number; the values the specs state only fall.
The rig held to the library: test_metacompiler_rig. The crafts' spec: test_metacompiler_crafts. The
meta-compiler's cache keys and the generated files' texts: test_mirrors; the specs' own words:
test_gynoid."""
import ast
import json
import math
import os
import re
import subprocess
import sys
import tomllib

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev import token_budget
from tools.dev.focus import chosen
from structure_kit import HOST, REPO, Report, sources

#: A gait number's unit by its name's end: a name ending in none of them is an angle or a share.
UNITS = ('_m', '_m_s', '_s', '_hz', '_rad_s', '_1_s', '_1_s2', '_m_s2', '_n')
#: A scaled build's gait as one line of JSON, asked of a process of its own (ROBOT is a process's).
ASKED = ('import json; from machine.body import compile, figure; g = compile.gait(); '
         'print(json.dumps({"scale": figure.SCALE, "leg_m": g["leg_m"], '
         '"tables": {k: g[k] for k in ("twist", "stride")}}))')


def _plain(value):
    """A compiled value as JSON's text, its tuples lists and its keys in order: equal texts, equal bits."""
    return json.dumps(value, sort_keys=True)


def _numbers(value, path=''):
    """(path, number) for every number a compiled value holds."""
    if isinstance(value, dict):
        for k, v in value.items():
            yield from _numbers(v, '%s.%s' % (path, k))
    elif isinstance(value, (list, tuple)):
        for i, v in enumerate(value):
            yield from _numbers(v, '%s[%d]' % (path, i))
    elif isinstance(value, (int, float)) and not isinstance(value, bool):
        yield path, float(value)


def test_the_worlds_gravity_is_stated_once(r):
    """No module of the host's packages or scripts holds the world's gravity as a number of its
    own: `world.toml` states it; the body's compiler, the simulated accelerometer and the
    emulator's worlds read it there, the air's density with it; the C cores of the robot's law
    take the constant `tools/cores/model.py` lays in wbc_model.h."""
    from coaxial.simulated import sensors
    from machine.body import compile as body, mechatronics
    from tools.emu import world
    env = mechatronics.environment(None)
    g = float(env['g_m_s2'])
    stated = sorted({'%s:%d' % (path.replace(os.sep, '/'), node.lineno)
                     for path, _text, tree in sources(beside=False) for node in ast.walk(tree)
                     if isinstance(node, ast.Constant) and isinstance(node.value, (int, float))
                     and not isinstance(node.value, bool) and abs(float(node.value)) == g})
    r.check('no module states the world\'s gravity as a number of its own', not stated, stated)
    left_out = dict(world.BODY)
    r.check('the compiler, the simulated accelerometer and the emulator\'s worlds read the world\'s',
            body.G_M_S2 == g and sensors.G == g and sensors.SimulatedImu.VECTORS[0][4] == (0.0, 0.0, g)
            and sensors.SimulatedImu.ONE_G == int(g * (1 << 8))
            and left_out['gravity'] == g and left_out['rho'] == float(env['air_kg_m3']), left_out)
    number, hand = re.compile(r'(?<![\w.])%s(?![\w.])' % re.escape(repr(g))), []
    for folder in ('src', 'inc'):
        for name in sorted(os.listdir(os.path.join(REPO, 'wbc', folder))):
            with open(os.path.join(REPO, 'wbc', folder, name), encoding='utf-8') as f:
                first, rest = f.readline(), f.read()
            if token_budget.GENERATED not in first and number.search(first + rest):
                hand.append(name)
    with open(os.path.join(REPO, 'wbc', 'inc', 'wbc_model.h'), encoding='utf-8') as f:
        laid = re.search(r'#define WBC_GRAVITY (\S+)', f.read())
    r.check('no core of the robot\'s law written by hand states it: the generated model\'s constant is the world\'s',
            not hand and laid is not None and float(laid.group(1)) == g, hand)


def test_a_build_compiled_anew_is_its_cache(r):
    """Both compilers run again from their specs give what `.cache/compiled` holds, bit for bit."""
    from machine.aircraft import compile as craft
    from machine.body import compile as body
    st = body._subtrees()
    fresh = {'subtrees': st, 'servos': body._servos(st), 'gait': body._gait(st)}
    kept = {'subtrees': body.subtrees(), 'servos': body.servos(), 'gait': body.gait()}
    off = [k for k in fresh if _plain(fresh[k]) != _plain(kept[k])]
    r.check('its body\'s subtrees, servos and gait compiled anew are the cached', not off, off)
    again, held = craft._build_aircraft(), craft._compiled()
    off = [k for k in again if _plain(again[k]) != _plain(held[k])]
    r.check('every craft\'s blocks and parts compiled anew are the cached', not off and set(again) == set(held), off)


def test_no_compiled_number_is_not_a_number(r):
    """Every number of its subtrees, servos, gait and of the crafts' blocks and parts is finite."""
    from machine.aircraft import compile as craft
    from machine.body import compile as body
    whole = {'subtrees': body.subtrees(), 'servos': body.servos(), 'gait': body.gait(), 'craft': craft._compiled()}
    numbers = list(_numbers(whole))
    wild = [p for p, v in numbers if not math.isfinite(v)]
    r.check('%d compiled numbers, every one finite' % len(numbers), len(numbers) > 500 and not wild, wild[:8])
    servos = body.servos()
    weak = [k for k, (peak, kp, kd, j) in servos.items() if not (peak > 0.0 and kp > 0.0 and kd > 0.0 and j >= 0.0)]
    r.check('every servo has a peak, a stiffness and a damping above nothing', not weak, weak)


def test_an_override_moves_the_number_it_names(r):
    """`compile.gait` and `compile.servos` merge a build's stated table over the derived: the one
    key moves, its table's other keys and the other tables stay, a peak not stated is the drive's."""
    from machine.body import compile as body, drives
    base = body.gait()
    told = body.gait({'twist': {'walk_m_s': 2.0 * base['twist']['walk_m_s']}})
    moved = [(t, k) for t in base if isinstance(base[t], dict) for k in base[t] if told[t][k] != base[t][k]]
    r.check('a gait override moves its one key', moved == [('twist', 'walk_m_s')]
            and _plain(body.gait()) == _plain(base), moved)
    plain = body.servos()
    over = body.servos({'knee': (float('nan'), 1.0, 2.0, 3.0)})
    r.check('a servo override keeps the others, its unstated peak the drive\'s own',
            over['knee'] == (drives.peak('left_knee'), 1.0, 2.0, 3.0)
            and all(over[k] == plain[k] for k in plain if k != 'knee'), over['knee'])


def _gait_of(build):
    """A build's scale, leg and its twist and stride tables, compiled in a process of its own."""
    env = dict(os.environ, COAXIAL_ROBOT=build, PYTHONPATH=HOST)
    out = subprocess.run([sys.executable, '-X', 'utf8', '-c', ASKED], env=env, cwd=HOST,
                         capture_output=True, text=True, check=True)
    return json.loads(out.stdout.strip().splitlines()[-1])


def test_a_scaled_figure_follows_similarity(r):
    """Every build that states only a scale of its own: its leg is the stock's by that scale, a
    length of its gait lies between the stock's and the stock's scaled, an angle or a share stays."""
    from machine.body import mechatronics
    scaled = sorted(name for name in mechatronics.robots()
                    if float(mechatronics.robot(name)['figure']['scale']) != 1.0)
    stock = _gait_of(mechatronics.STOCK)
    r.check('a build scales the stock figure', bool(scaled) and stock['scale'] == 1.0, scaled)
    for name in scaled:
        its = _gait_of(name)
        s = its['scale']
        lo, hi = min(1.0, s) - 1e-9, max(1.0, s) + 1e-9
        long, turned = [], []
        for table, rows in its['tables'].items():
            for key, value in rows.items():
                was = stock['tables'][table][key]
                if key.endswith('_m') and not lo <= value / was <= hi:
                    long.append((key, round(value / was, 4)))
                if not key.endswith(UNITS) and value != was:
                    turned.append((key, value, was))
        r.check('%s: its leg is the stock\'s by %.2f' % (name, s),
                math.isclose(its['leg_m'], s * stock['leg_m'], rel_tol=1e-12), its['leg_m'])
        r.check('%s: its gait\'s lengths move no further than its scale' % name, not long, long)
        r.check('%s: its gait\'s angles and shares are the stock\'s' % name, not turned, turned)


def test_a_joint_dressed_in_nothing_is_bare(r):
    """A build's `[cloth]` names the joints it dresses: the resolver lays bare (`mechatronics.BARE`,
    the identity over bare air) under the rest, after those named and in the stacks' order, and
    the stock build states no joint bare."""
    from machine.body import drives, mechatronics
    with open(os.path.join(mechatronics.ROBOTS, mechatronics.STOCK + '.toml'), 'rb') as f:
        stated = tomllib.load(f)['cloth']
    build = mechatronics.robot(mechatronics.STOCK)
    bare = [kind for kind in build['stacks'] if kind not in stated]
    r.check('the stock build states no joint bare and dresses only joints it has',
            bool(bare) and mechatronics.BARE not in stated.values() and set(stated) <= set(build['stacks']), stated)
    r.check('every joint has its cloth: the named as stated, then the bare in the stacks\' order',
            list(build['cloth']) == list(stated) + bare and set(build['cloth']) == set(build['stacks'])
            and all(build['cloth'][kind] == mechatronics.BARE for kind in bare)
            and all(build['cloth'][kind] == value for kind, value in stated.items()), list(build['cloth']))
    r.check('the robot\'s drives read the table as resolved', drives.CLOTH_AIR == mechatronics.robot()['cloth'],
            drives.CLOTH_AIR)


def test_the_specs_inputs_only_fall(r):
    """Each spec file states as many values as its ceiling (`tools/dev/stated.py`): a value more is an input the
    static solver was to derive, a value fewer lowers the ceiling with it - the solver's inputs only fall."""
    from tools.dev import stated
    counts = {path: total for path, (total, _tables) in stated.counted().items()}
    r.check('every spec file has its ceiling, and no ceiling a file that is gone',
            set(counts) == set(stated.CEILING), sorted(set(counts) ^ set(stated.CEILING)))
    over = {path: (n, stated.CEILING[path]) for path, n in counts.items() if n > stated.CEILING.get(path, n)}
    r.check('no spec states more values than its ceiling: %d in all' % sum(counts.values()), not over, over)
    slack = {path: (n, stated.CEILING[path]) for path, n in counts.items() if n < stated.CEILING.get(path, n)}
    r.check("a ceiling is its spec's count: where a spec states fewer, the ceiling falls with it", not slack, slack)


ROSTER = (test_the_worlds_gravity_is_stated_once, test_a_build_compiled_anew_is_its_cache,
          test_no_compiled_number_is_not_a_number, test_an_override_moves_the_number_it_names,
          test_a_scaled_figure_follows_similarity, test_a_joint_dressed_in_nothing_is_bare,
          test_the_specs_inputs_only_fall)


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed, %d skipped'
          % (report.passed, report.failed, getattr(report, 'skipped', 0)))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
