#!/usr/bin/env python3
"""The static meta-compiler held to itself: one source a number, the same answer twice, similarity.

What a sheet states is read from the parts library and stated nowhere else, the world's gravity
in `world.toml` alone; a motor the library and the rig's profiles both describe is one motor; a
build compiled anew is the build its cache holds; an override moves the one number it names; a
figure scaled keeps its angles and shares and moves its lengths no further than its scale; a craft
loaded heavier keeps its radii of gyration; no compiled number is not a number.
The meta-compiler's cache keys and the generated files' texts: test_mirrors; the specs' own words: test_gynoid."""
import ast
import glob
import json
import math
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen
from structure_kit import HOST, Report, sources

#: The power stage's names its model reads from the library, each the library's key or its solver.
STAGE = {'FSW': 'pwm_hz', 'RDS_ON': 'rds_ohm', 'RTH_JC': 'rth_jc', 'T_J_MAX': 'tj_max_c', 'SHUNT': 'shunt_ohm'}
#: A library motor's keys beside its rig profile's: the stand-in model's and the drive's record's.
PROFILED = (('ohm', 'r', 'motor_r'), ('lq_h', 'lq', 'motor_lq'), ('flux_vs', 'lambda', 'motor_lambda'),
            ('pole_pairs', 'pole_pairs', 'motor_pole_pairs'))
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


def _scalars(block) -> dict[str, float]:
    """A block's numbers by their names, what it nests left out."""
    return {k: float(v) for k, v in block.items() if isinstance(v, (int, float)) and not isinstance(v, bool)}


def _literals(path, names):
    """Those of `names` a module's top level binds to a bare number, a tuple's own too."""
    with open(path, encoding='utf-8') as f:
        tree = ast.parse(f.read())
    bare = []
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        target, value = node.targets[0], node.value
        pairs = (zip(target.elts, value.elts)
                 if isinstance(target, ast.Tuple) and isinstance(value, ast.Tuple) else [(target, value)])
        bare += [t.id for t, v in pairs if isinstance(t, ast.Name) and t.id in names
                 and isinstance(v, ast.Constant) and isinstance(v.value, (int, float))]
    return bare


def test_the_power_stage_is_the_librarys(r):
    """`coaxial.model.inverter` and `thermal` state no number the library's inverter states: each
    is read there, and the modulator's share of the link is the solver's (`mechatronics.v_frac`)."""
    from coaxial.devices import scaling
    from coaxial.model import inverter, thermal
    from machine.body import mechatronics
    inv = mechatronics.library()['inverters']['coaxial_63100']
    bare = _literals(os.path.join(HOST, 'coaxial', 'model', 'inverter.py'), set(STAGE) | {'V_FRAC'})
    r.check('the inverter model binds none of the library\'s numbers to a literal', not bare, bare)
    apart = [n for n, key in STAGE.items() if getattr(inverter, n) != float(inv[key])]
    r.check('its switching rate, resistances and junction limit are the library\'s',
            not apart and inverter.V_FRAC == mechatronics.v_frac(inv) and inverter.TS == 1.0 / float(inv['pwm_hz']),
            apart)
    r.check('the board\'s phase shunt as its scaling states it is the library\'s',
            scaling.PHASE_ONBOARD.r_shunt == scaling.ShuntParams().r_shunt == float(inv['shunt_ohm']),
            scaling.PHASE_ONBOARD.r_shunt)
    switch, laminate, stator, winding = (float(c) for c in inv['ceiling_c'])
    ceilings = thermal.CEILING_C
    r.check('the thermal model\'s ceilings and its die\'s resistance are the library\'s',
            thermal.CEILING_DEFAULT_C == switch
            and all(ceilings[n] == laminate for n in thermal.LAMINATE)
            and all(ceilings[n] in (stator, winding) for n in thermal.MOTOR)
            and all(ceilings[n] == float(inv['tj_max_c']) and thermal.RTH_DIE[n] == float(inv['rth_jc'])
                    for n in thermal.DRIVERS), ceilings)


def test_the_worlds_gravity_is_stated_once(r):
    """No module of the host's packages or scripts holds the world's gravity as a number of its
    own: `world.toml` states it; the body's compiler, the simulated accelerometer and the
    emulator's worlds read it there, the air's density with it."""
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


def test_a_motor_and_its_profile_are_one(r):
    """A motor the library and the rig both describe (`coaxial/profiles/<its name>.json`, its drive's
    record and its stand-in model) is one motor: its resistance, its q inductance, its flux and its
    pole pairs the same in both, its bearing drag the model's; the crafts' motor's rotor is the can
    the library's mass and diameter give; every profile's share of the link is the solver's."""
    from coaxial.devices.drive import PROFILES
    from machine.aircraft import compile as craft
    from machine.body import mechatronics
    lib, shared = mechatronics.library(), mechatronics.crafts()['shared']
    share = mechatronics.v_frac(lib['inverters'][shared['inverter']])
    both, apart, shares, rotor = [], [], [], None
    for path in sorted(glob.glob(os.path.join(PROFILES, '*.json'))):
        name = os.path.splitext(os.path.basename(path))[0]
        with open(path, encoding='utf-8') as f:
            profile = json.load(f)
        shares += [name] * (float(profile['drive']['drv_v_frac']) != share)
        motor = lib['motors'].get(name)
        if motor is None:
            continue
        both.append(name)
        apart += [(name, key) for key, model, drive in PROFILED
                  if not float(motor[key]) == float(profile['model'][model]) == float(profile['drive'][drive])]
        apart += [(name, 'viscous_nms')] * (float(motor['viscous_nms']) != float(profile['model']['b']))
        rotor = float(profile['model']['j']) if name == shared['motor'] else rotor
    r.check('a motor the library and the rig both describe is one: %s' % ', '.join(both), bool(both) and not apart, apart)
    r.check('the crafts\' motor\'s rotor is the can its mass and diameter give', rotor == craft.ROTOR_CAN_J,
            (rotor, craft.ROTOR_CAN_J))
    r.check('every profile\'s share of the link is the solver\'s', not shares, shares)


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


def test_a_heavier_craft_keeps_its_radii(r):
    """A craft's heavy variant states a mass alone: its body's inertias grow by the mass's ratio,
    its geometry stays, and the hover thrust AeroLisp starts from is the quadcopter's weight."""
    from machine.aircraft import compile as craft
    from machine.body import mechatronics
    blocks, crafts = craft.blocks(), mechatronics.crafts()
    pairs = [(k[:-len('_heavy')], k) for k in blocks if k.endswith('_heavy') and k[:-len('_heavy')] in blocks]
    off = []
    for light, heavy in pairs:
        a, b = _scalars(blocks[light][1]), _scalars(blocks[heavy][1])
        ratio = b['mass'] / a['mass']
        off += [(heavy, k) for k in a if k != 'mass' and not math.isclose(
            b[k], a[k] * (ratio if k.startswith('inertia_') else 1.0), rel_tol=1e-9)]
    r.check('%d heavy bodies: inertias by the mass\'s ratio, geometry kept' % len(pairs),
            len(pairs) >= 4 and not off, off)
    hover, _speed = craft.defaults()
    weight = float(crafts['quadcopter']['mass_kg']) * craft.GRAVITY_M_S2
    r.check('the base hover thrust is the quadcopter\'s weight', math.isclose(hover, weight, rel_tol=1e-12),
            (hover, weight))
    env = _scalars(blocks['standard_arena_envelope'][1])
    r.check('the arena\'s heights stand in their order',
            0.0 < env['floor_height'] < env['hover_height'] < env['high_height'] < env['top_height'], env)


ROSTER = (test_the_power_stage_is_the_librarys, test_the_worlds_gravity_is_stated_once,
          test_a_motor_and_its_profile_are_one, test_a_build_compiled_anew_is_its_cache,
          test_no_compiled_number_is_not_a_number, test_an_override_moves_the_number_it_names,
          test_a_scaled_figure_follows_similarity, test_a_heavier_craft_keeps_its_radii)


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
