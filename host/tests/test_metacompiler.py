#!/usr/bin/env python3
"""The static meta-compiler held to itself: one source a number, the same answer twice, similarity.

Gravity in `world.toml` alone; a build compiled anew equals its cache; an override moves its one key;
a scaled figure keeps angles/shares and bounds lengths by scale; an undressed joint is bare; every
compiled number finite; stated values only fall; drives stated once. Companion suites:
test_metacompiler_rig, test_metacompiler_crafts, test_mirrors, test_gynoid."""
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
    """Gravity is stated in `world.toml` alone; body compiler, simulated IMU, emulator worlds and
    `wbc_model.h` read it there."""
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
    """`compile.gait` and `compile.servos` merge overrides cleanly, keeping unstated peaks."""
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
    """Scaled builds scale leg length, bound gait lengths within `[1, s]`, and keep angles/shares."""
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
    """`[cloth]` resolves omitted joints to `mechatronics.BARE` in stacks order; stock states none bare."""
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
    """Each spec file states at most its `stated.CEILING` count, and lowers the ceiling when fewer."""
    from tools.dev import stated
    counts = {path: total for path, (total, _tables) in stated.counted().items()}
    r.check('every spec file has its ceiling, and no ceiling a file that is gone',
            set(counts) == set(stated.CEILING), sorted(set(counts) ^ set(stated.CEILING)))
    over = {path: (n, stated.CEILING[path]) for path, n in counts.items() if n > stated.CEILING.get(path, n)}
    r.check('no spec states more values than its ceiling: %d in all' % sum(counts.values()), not over, over)
    slack = {path: (n, stated.CEILING[path]) for path, n in counts.items() if n < stated.CEILING.get(path, n)}
    r.check("a ceiling is its spec's count: where a spec states fewer, the ceiling falls with it", not slack, slack)


def test_a_builds_drives_are_stated_once(r):
    """`[drives]` lays stacks from motor groups and base gearbox/inverter; packs follow mount count."""
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
    """`compile.servos` is solved by `looped` from `carried.table` and `control.toml` via
    `machine.solver.loops`, matching `tools/cores/designed.py`."""
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
    solved, gains, today = looped.table(st), looped.gains(st), body.servos()
    off, soft, held = [], [], []
    for kind, (rotor, limb, most, loop, needs, ceilings) in solved.items():
        kp, kd = gains[kind]
        off += [kind] * (kind in seen and today[kind][1:3] != (kp, kd))
        on = limb if loop.need == 'swing' else most
        if (kp != float(round(min(loop.kp, ceilings[loop.ceiling])))
                or abs(kd - min(2.0 * zeta * math.sqrt(kp * math.sqrt(rotor * on)), rotor * taken)) > 0.05):
            off.append(kind)
        if not math.isclose(math.sqrt(needs['swing'] / limb), swung, rel_tol=1e-12) or not rotor < limb <= most:
            soft.append(kind)
        if loop.need == 'stance' and (kind not in bears or needs['stance'] <= max(row[2] for row in seen[kind])):
            held.append(kind)
    r.check("every driven joint's servo is its loop's: its stiffness under its least ceiling, its damping the target's "
            "on the mean of its rotor and what set its stiffness, no more than its rotor takes",
            set(seen) <= set(solved) and not off, off)
    r.check("every limb's loop is asked the cascade's factor over the stance loop's, itself the fall's by its factor",
            not soft and math.isclose(fall, float(ctl['fall_factor']) * math.sqrt(body.G_M_S2 / st['h0'])), soft)
    r.check('a loop set by its stance is a leg\'s, and asks more than gravity alone tips it with', not held, held)
    unmet = sorted(kind for kind, row in solved.items() if not row[3].feasible)
    if mechatronics.ROBOT != mechatronics.STOCK:
        probe = subprocess.run([sys.executable, '-X', 'utf8', '-c',
                                'import json; from machine.body import compile, looped; '
                                'print(json.dumps(sorted(k for k, r in looped.table(compile.subtrees()).items() '
                                'if not r[3].feasible)))'],
                               env=dict(os.environ, COAXIAL_ROBOT=mechatronics.STOCK, PYTHONPATH=HOST),
                               cwd=HOST, capture_output=True, text=True, check=True)
        stock_unmet = json.loads(probe.stdout.strip().splitlines()[-1])
    else:
        stock_unmet = unmet
    r.check("the robot's own build can be what its loops ask: no need lies over a ceiling",
            not stock_unmet and set(unmet) <= {'ankle_roll'}, (stock_unmet, unmet))
    laid = designed.rows()
    own = {loop: (kp, kd, solved_kp, solved_kd) for machine, loop, kp, kd, solved_kp, solved_kd, *_ in laid
           if machine == mechatronics.ROBOT}
    crafts = sorted({machine for machine, *_ in laid} - {mechatronics.ROBOT})
    r.check('the comparison lays a row a loop: %d of the robot\'s, its servo table the solved, and %d crafts\' axes'
            % (len(own), len(laid) - len(own)),
            set(own) == set(solved) and len(crafts) >= 4 and all(len(row) == len(designed.COLUMNS) for row in laid)
            and all(own[kind] == (today[kind][1], today[kind][2]) + gains[kind] for kind in own), crafts)


def test_every_build_compiles_cleanly_across_humanoids(r):
    """Every humanoid build (`mechatronics.robots`) compiles finite tables, positive split armatures
    on sprung joints, and clean drive mounts (`fit.outputs`, `held.held_by`)."""
    from machine.body import mechatronics
    script = (
        'import json, math\n'
        'from machine.body import compile, drives, figure, looped, reach, servos, terms\n'
        'from tools.sim import fit, held\n'
        'bad = []\n'
        'for j in figure.JOINTS:\n'
        '    if drives.passive(j): continue\n'
        '    k = drives.kind(j)\n'
        '    pk, kp, kd, jf = servos.SERVO[k]\n'
        '    ja = drives.armature(j)\n'
        '    if not (pk > 0 and kp > 0 and kd > 0 and 0.0 < jf < ja):\n'
        '        bad.append((j, pk, kp, kd, jf, ja))\n'
        'wrong = [(j, w) for j, _what, w in fit.outputs() if w]\n'
        'loose = [j for j, by in held.held_by().items() if not by]\n'
        'knee_hi = terms.find("knee", "flexion")[3]\n'
        'print(json.dumps({"bad": bad, "wrong": wrong, "loose": loose,\n'
        '                  "knee_ok": knee_hi == drives.TRAVEL["knee"][1],\n'
        '                  "reach_n": len(reach.FORE)}))\n'
    )
    faults = {}
    for build in mechatronics.robots():
        out = subprocess.run([sys.executable, '-X', 'utf8', '-c', script],
                             env=dict(os.environ, COAXIAL_ROBOT=build, PYTHONPATH=HOST),
                             cwd=HOST, capture_output=True, text=True, check=True)
        got = json.loads(out.stdout.strip().splitlines()[-1])
        if got['bad'] or got['wrong'] or got['loose'] or not got['knee_ok'] or got['reach_n'] < 50:
            faults[build] = got
    r.check('every humanoid build compiles positive split armatures, clean drive mounts and matching reach/terms',
            not faults, faults)


def test_trunk_pair_mounts_boards_and_transitive_sources(r):
    """Mutually exclusive `[mounts]`/`[boards]` modes replace cleanly, broken `stack_on` raises,
    `body_sources` tracks solver modules, and a cardan trunk pair (`waist` held) compiles cleanly."""
    from machine.body import mechatronics
    mm = mechatronics._merged({'mounts': {'spine_roll': {'on': 'pelvis', 'z_m': 0.005}}},
                              {'mounts': {'spine_roll': {'x_m': -0.04, 'face': '-x'}}})
    mb = mechatronics._merged({'boards': {'ankle': {'stack_on': 'knee', 'gap_m': 0.001}}},
                              {'boards': {'ankle': {'on': 'shank', 'at': [0.0, -0.1, 0.0], 'face': 'x'}}})
    srcs = {os.path.basename(p) for p in mechatronics.body_sources()}
    r.check('inheritance drops superseded mount/board keys and hashes transitive solver modules',
            mm['mounts']['spine_roll'] == {'on': 'pelvis', 'x_m': -0.04, 'face': '-x'}
            and mb['boards']['ankle'] == {'on': 'shank', 'at': [0.0, -0.1, 0.0], 'face': 'x'}
            and {'structural.py', 'linkage.py', 'skeleton.py', 'guards.py', 'loops.py', 'inertia.py'} <= srcs,
            (mm, mb))
    rod = lambda s: {'joint': 'spine', 'up': -0.086, 'ahead': 0.004, 'crank_m': 0.078, 'crank_deg': 134.0,
                     'ball_m': 0.085, 'ball_deg': 266.0, 'crank_x': s * 0.091, 'ball_x': s * 0.083,
                     'bend': [s * 0.087, -0.04, -0.04], 'stroke': [-30.0, 85.0]}
    over_json = json.dumps({
        'linkage': {'planar': {'spine_roll': False}, 'pairs': {'spine': 'spine_roll'}, 'rolled': {'spine': 35.0},
                    'rods': {'spine': rod(1), 'spine_roll': rod(-1)}},
        'mounts': {'spine': {'on': 'pelvis', 'x_m': 0.04, 'dy_m': 0.0, 'face': 'x'},
                   'spine_roll': {'x_m': -0.04, 'dy_m': 0.0, 'face': '-x'}},
        'boards': {'spine': False}, 'stops': {'spine': False, 'waist': False}, 'passive': {'waist': {'way': 2.0}},
        'leaf': {'joint': ['ankle', 'ankle_roll', 'spine', 'spine_roll'],
                 'width_m': [0.016, 0.016, 0.02, 0.02], 'thick_m': [0.002615, 0.002615, 0.003, 0.003]},
    })
    code = '\n'.join((
        'import copy, json, sys; from machine.body import mechatronics; from machine.errors import MachineError',
        'b0 = copy.deepcopy(mechatronics.robot("gynoid_mesomorph_medium_coax"))',
        'ov = json.loads(sys.argv[1]); ov["stacks"] = {**b0["stacks"], "spine": ["MN6007", "B45", "coaxial_63100"],'
        ' "spine_roll": ["MN6007", "B45", "coaxial_63100"]}',
        'bt = mechatronics._merged(b0, ov); mechatronics.robot = lambda name=None: bt',
        'from machine.body import compile as body, drives, peripherals, skeleton',
        'from coaxial.graphics import mechanism; from tools.sim import bom, fit',
        'st = body._subtrees(); sv, gt = body._servos(st), body._gait(st); bad = False',
        'drives._BUILD["boards"] = {"head": {"stack_on": "neck", "gap_m": 0.002}}',
        'try: drives._boards()',
        'except MachineError: bad = True',
        'drives._BUILD["boards"] = bt["boards"]; bs = [n for n, *_ in skeleton.bodies()]',
        'print(json.dumps([drives.JOINTS["spine"][1][1], drives.series("spine") > 0, gt["steer"]["waist_share"],',
        ' sv["waist"][1] > 0, "spine_rod" in bs and "spine_roll_rod" in bs and "left_spine_rod" not in bs,',
        ' [(j, w) for j, _, w in fit.outputs() if w], dict(bom.parts()).get("rod 12 mm, bent", 0),',
        ' len(mechanism.parts()) > 20, peripherals._identify()["campaign"]["free_rad"] > 0, bad]))',
    ))
    out = subprocess.run([sys.executable, '-X', 'utf8', '-c', code, over_json],
                         env=dict(os.environ, PYTHONPATH=HOST), cwd=HOST, capture_output=True, text=True, check=True)
    got = json.loads(out.stdout.strip().splitlines()[-1])
    r.check('cardan trunk pair, passive waist and board stack_on guard compile cleanly across the stack',
            got == [0.034, True, 0.0, True, True, [], 6, True, True, True], got)


ROSTER = (test_the_worlds_gravity_is_stated_once, test_a_build_compiled_anew_is_its_cache,
          test_no_compiled_number_is_not_a_number, test_an_override_moves_the_number_it_names,
          test_a_scaled_figure_follows_similarity, test_a_joint_dressed_in_nothing_is_bare,
          test_the_specs_inputs_only_fall, test_a_builds_drives_are_stated_once,
          test_a_joints_loop_is_solved_of_what_it_turns,
          test_every_build_compiles_cleanly_across_humanoids,
          test_trunk_pair_mounts_boards_and_transitive_sources)


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
