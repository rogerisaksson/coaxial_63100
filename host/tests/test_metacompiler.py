#!/usr/bin/env python3
"""The static meta-compiler held to itself: one source a number, the same answer twice, similarity.

The world's gravity is stated in `world.toml` alone; a build compiled anew is the build its cache
holds; an override moves the one number it names; a figure scaled keeps its angles and shares and
moves its lengths no further than its scale; a joint dressed in nothing is bare; no compiled
number is not a number; the values the specs state only fall; a build's drives are stated once.
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


def test_a_builds_drives_are_stated_once(r):
    """A build's `[drives]` lays every kind's stack: one gearbox and one inverter, a motor a group of kinds; a
    variant's own lie on its base's, the kinds it names alone moved; a pack a mount."""
    from machine.body import dclink, mechatronics
    one = mechatronics._stacks({}, {'gearbox': 'B', 'inverter': 'I', 'motors': {'M': ['a', 'b'], 'N': ['c']}})
    two = mechatronics._stacks(one, {'motors': {'P': ['b']}, 'gearboxes': {'A': ['b', 'c']}})
    r.check('a motor a group of kinds, one gearbox and one inverter on each, in the kinds\' order',
            list(one.items()) == [('a', ['M', 'B', 'I']), ('b', ['M', 'B', 'I']), ('c', ['N', 'B', 'I'])], one)
    r.check('a variant moves the kinds it names alone, each in its place, its base as it was',
            list(two.items()) == [('a', ['M', 'B', 'I']), ('b', ['P', 'A', 'I']), ('c', ['N', 'A', 'I'])]
            and one['b'] == ['M', 'B', 'I'], two)
    raw = {}
    for name in mechatronics.robots():
        with open(os.path.join(mechatronics.ROBOTS, name + '.toml'), 'rb') as f:
            raw[name] = tomllib.load(f)
    r.check('no build states a stack a kind, and none its packs\' count',
            not [n for n, spec in raw.items() if 'stacks' in spec or 'packs' in spec.get('pack', {})], sorted(raw))
    own, laid = raw[mechatronics.STOCK]['drives'], mechatronics.robot(mechatronics.STOCK)['stacks']
    r.check('the robot\'s own build lays the kinds its motors name, each on the one gearbox and inverter the build states',
            list(laid) == [kind for kinds in own['motors'].values() for kind in kinds]
            and all(stack[1:] == [own['gearbox'], own['inverter']] for stack in laid.values()), laid)
    pack = mechatronics.robot()['pack']
    r.check('a pack a mount: the DC link counts its packs by them',
            dclink.PACK_OHM == pack['series'] * pack['cell_ohm'] / pack['parallel'] / len(pack['mounts'])
            and dclink.CHARGE_A == pack['cell_charge_a'] * pack['parallel'] * len(pack['mounts']), pack['mounts'])


def test_a_joints_loop_is_solved_of_what_it_turns(r):
    """Beside the servo table the compiler lays by formulas a joint stands the solved one, one rule
    for every driven joint (`machine.body.looped`): of what its drive turns (`carried.table`: its
    limb swinging, the body it carries where a sole hangs on it, gravity's stiffness there) and the
    control design's targets (`control.toml`), by `machine.solver.loops` - the stiffer of the
    swing's and the stance's needs under its ceilings. The comparison lays a row a loop, the
    robot's and every craft's (`tools/cores/designed.py`); the robot's law runs on none of it."""
    from machine.body import carried, compile as body, drives, looped, mechatronics
    from tools.cores import designed
    seen, st, ctl = carried.table(body.G_M_S2), body.subtrees(), mechatronics.control()
    bears = sorted(kind for kind, rows in seen.items() if any(held for _swing, held, _tip in rows))
    carries = [seen[kind][0][1] for kind in ('hip', 'knee', 'ankle')]
    r.check("the joints that carry the robot are its legs': standing, the further down, the more of it a joint carries",
            bears == ['ankle', 'ankle_roll', 'hip', 'hip_roll', 'hip_yaw', 'knee'] and carries == sorted(carries)
            and all(len(rows) == len(carried.postures()) and all(row[0] > 0.0 for row in rows) for rows in seen.values()),
            bears)
    tip = seen['ankle'][0][2] / (st['weight'] * st['h0'])
    r.check("gravity tips the robot over its ankle with nearly its weight times its centre's height", 0.8 < tip < 1.0, tip)
    zeta, fall = float(ctl['damping_ratio']), float(ctl['fall_factor']) * st['omega']
    swung = float(ctl['cascade_factor']) * fall
    taken = math.radians(float(ctl['delay_phase_deg'])) * drives.BUS_HZ / looped.DEAD_PERIODS
    solved, gains = looped.table(), looped.gains()
    off, soft, held = [], [], []
    for kind, (rotor, limb, most, loop, needs, ceilings) in solved.items():
        kp, kd = gains[kind]
        on = limb if loop.need == 'swing' else most
        if (kp != float(round(min(loop.kp, ceilings[loop.ceiling])))
                or abs(kd - min(2.0 * zeta * math.sqrt(kp * math.sqrt(rotor * on)), rotor * taken)) > 0.05):
            off.append(kind)
        if not math.isclose(math.sqrt(needs['swing'] / limb), swung, rel_tol=1e-12) or not rotor < limb <= most:
            soft.append(kind)
        if loop.need == 'stance' and (kind not in bears or needs['stance'] <= max(row[2] for row in seen[kind])):
            held.append(kind)
    r.check("every driven joint's stiffness is its loop's under its least ceiling, its damping the target's on the "
            "mean of its rotor and what set its stiffness, no more than its rotor takes", set(seen) <= set(solved) and not off, off)
    r.check("every limb's loop is asked the cascade's factor over the stance loop's, itself the fall's by its factor",
            not soft and math.isclose(fall, float(ctl['fall_factor']) * math.sqrt(body.G_M_S2 / st['h0'])), soft)
    r.check('a loop set by its stance is a leg\'s, and asks more than gravity alone tips it with', not held, held)
    unmet = sorted(kind for kind, row in solved.items() if not row[3].feasible)
    r.check("the robot's own build can be what its loops ask: no need lies over a ceiling", not unmet, unmet)
    laid, today = designed.rows(), body.servos()
    own = {loop: (kp, kd, solved_kp, solved_kd) for machine, loop, kp, kd, solved_kp, solved_kd, *_ in laid
           if machine == mechatronics.ROBOT}
    crafts = sorted({machine for machine, *_ in laid} - {mechatronics.ROBOT})
    r.check('the comparison lays a row a loop: %d of the robot\'s, today\'s table beside the solved, and %d crafts\' axes'
            % (len(own), len(laid) - len(own)),
            set(own) == set(solved) and len(crafts) >= 4 and all(len(row) == len(designed.COLUMNS) for row in laid)
            and all(own[kind] == (today[kind][1], today[kind][2]) + gains[kind] for kind in own), crafts)


ROSTER = (test_the_worlds_gravity_is_stated_once, test_a_build_compiled_anew_is_its_cache,
          test_no_compiled_number_is_not_a_number, test_an_override_moves_the_number_it_names,
          test_a_scaled_figure_follows_similarity, test_a_joint_dressed_in_nothing_is_bare,
          test_the_specs_inputs_only_fall, test_a_builds_drives_are_stated_once,
          test_a_joints_loop_is_solved_of_what_it_turns)


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
