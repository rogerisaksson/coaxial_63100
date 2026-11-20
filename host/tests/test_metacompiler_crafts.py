#!/usr/bin/env python3
"""The crafts' spec held to its compilers: a craft states its own alone, a wing's numbers its own.

`crafts.toml` as it is written and as `mechatronics.crafts` resolves it; the aerobat's lift slope
of its wing's aspect ratio and its drag due to lift of the airframe's own span efficiency, a wing a
vehicle's text resizes with them; the floor's friction read for friction alone; a craft on its
heavy frame with its radii of gyration kept; the quadcopter's hover solved once for its blocks and
its parts, as built and resized; every flying text's pitch and roll solved of its own blocks beside
today's (`machine.aircraft.attitude`).
The solver's own laws, one source a number and a build anew its cache: test_metacompiler."""
import math
import os
import sys
import tomllib

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen
from structure_kit import HOST, Report, sources

#: The crafts' spec's compilers, where a key it states is read: its polars, its blocks and its resolver.
CRAFT_SOLVERS = ('machine/aircraft/polars.py', 'machine/aircraft/compile.py', 'machine/body/mechatronics.py')


def _scalars(block) -> dict[str, float]:
    """A block's numbers by their names, what it nests left out."""
    return {k: float(v) for k, v in block.items() if isinstance(v, (int, float)) and not isinstance(v, bool)}


def test_a_craft_states_its_own_alone(r):
    """`crafts.toml` as it is written: every key a craft states is read by its compilers, none
    states again what it has of `[shared]` or of the craft it extends, and a craft resolved is
    `[shared]` with its own laid on it, a variant its craft with its own."""
    from machine.body import mechatronics
    with open(os.path.join(HOST, 'machine', 'aircraft', 'crafts.toml'), 'rb') as f:
        raw = tomllib.load(f)
    read = ''
    for name in CRAFT_SOLVERS:
        with open(os.path.join(HOST, *name.split('/')), encoding='utf-8') as f:
            read += f.read()
    stated = set()
    for cfg in raw.values():
        for key, value in cfg.items():
            stated |= {key, *(value if isinstance(value, dict) else ())}
    unread = sorted(key for key in stated if "'%s'" % key not in read)
    r.check('%d keys the crafts state, each read by a compiler' % len(stated), len(stated) > 60 and not unread, unread)
    resolved, shared = mechatronics.crafts(), raw['shared']
    again, apart = [], []
    for name, cfg in raw.items():
        base = resolved[cfg['extends']] if 'extends' in cfg else shared
        own = {key: value for key, value in cfg.items() if key != 'extends'}
        again += [(name, key) for key, value in own.items() if name != 'shared' and base.get(key) == value]
        apart += [(name, key) for key in {*base, *own}
                  if name != 'shared' and resolved[name].get(key) != own.get(key, base.get(key))]
    r.check('no craft states again what it has of the shared or of its craft', not again, again)
    r.check("a craft resolved is the shared with its own laid on it, a variant its craft's",
            not apart and not any('extends' in cfg for cfg in resolved.values()), apart)


def test_a_wings_lift_slope_is_its_aspect_ratios(r):
    """The aerobat's lift a radian of incidence is not stated: `polars.lift_slope` gives it of its
    wing's span and chord (Helmbold) - a slender wing's pi/2 an aspect ratio at the one end, thin
    airfoil theory's 2 pi at the other, rising between."""
    from machine.aircraft import compile as craft, polars
    from machine.body import mechatronics
    wing = mechatronics.crafts()['aerobat']
    slope = polars.lift_slope(wing['wing_span_m'], wing['wing_chord_m'])
    laid = _scalars(craft.blocks()['unlimited_3d_monoplane'][1])['cl_alpha']
    r.check("the aerobat states no lift slope: its block's is its wing's aspect ratio's",
            'cl_alpha' not in wing and laid == slope, (laid, slope))
    slopes = [polars.lift_slope(aspect, 1.0) for aspect in (1e-6, 1.0, 2.0, 5.0, 10.0, 1e3, 1e9)]
    r.check("a wing's lift slope rises with its aspect ratio, pi/2 of it slender, toward 2 pi a radian",
            all(a < b for a, b in zip(slopes, slopes[1:])) and slopes[-1] <= math.tau
            and math.isclose(slopes[0] / 1e-6, 0.5 * math.pi, rel_tol=1e-6)
            and math.isclose(slopes[-1], math.tau, rel_tol=1e-6), slopes)


def test_the_floors_friction_is_friction_alone(r):
    """`world.toml`'s kinetic share is the floor's under a sole, read for friction alone: the robot's
    gait compiler names it for the friction the robot's law counts on, its world's contacts
    (`stiction`) for the friction a sliding contact bears. A wing's drag due to lift takes the
    airframe's own span efficiency, and a wing a vehicle's text resizes takes its lift slope and
    that drag of its new aspect ratio."""
    from machine.aircraft import aeroblocks, aerolisp, compile as craft, polars
    from machine.body import compile as body, mechatronics
    env, wing = mechatronics.environment(None), mechatronics.crafts()['aerobat']
    readers = sorted(path.replace(os.sep, '/') for path, text, _tree in sources(beside=False)
                     if 'kinetic_share' in text)
    r.check("the kinetic share of the floor's friction is named for friction alone: the robot's gait's, its contacts'",
            readers == ['machine/body/compile.py', 'machine/world/stiction.py'], readers)
    counted = body.gait()['loop']['mu']
    r.check("the friction the robot's law counts on is the kinetic share of the floor's",
            counted == round(float(env['kinetic_share']) * float(env['mu_dry']), 2), counted)
    blocks = craft.blocks()
    base = _scalars(blocks['unlimited_3d_monoplane'][1])
    aspect = float(wing['wing_span_m']) / float(wing['wing_chord_m'])
    r.check("the aerobat's drag due to lift is of the airframe's own span efficiency",
            base['span_efficiency'] == float(wing['span_efficiency'])
            and base['cd_induced'] == round(1.0 / (math.pi * base['span_efficiency'] * aspect), 3), base['cd_induced'])
    wide = aeroblocks.kv_block([['use', 'unlimited_3d_monoplane'], ['wing_span', 2.0 * base['wing_span']]],
                               aeroblocks.AERO_BLOCKS['aero_surfaces'], aerolisp.AeroLispCompiler(), None, None,
                               blocks, 'aero_surfaces')
    r.check('a wing a text resizes takes its lift slope and its drag due to lift of its new aspect ratio',
            wide['cl_alpha'] == polars.lift_slope(wide['wing_span'], wide['wing_chord'])
            and wide['cd_induced'] == round(1.0 / (math.pi * base['span_efficiency'] * 2.0 * aspect), 3)
            and wide['span_efficiency'] == base['span_efficiency'], (wide['cl_alpha'], wide['cd_induced']))


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
    r.check("%d heavy bodies: inertias by the mass's ratio, geometry kept" % len(pairs),
            len(pairs) >= 4 and not off, off)
    hover, _speed = craft.defaults()
    weight = float(crafts['quadcopter']['mass_kg']) * craft.GRAVITY_M_S2
    r.check("the base hover thrust is the quadcopter's weight", math.isclose(hover, weight, rel_tol=1e-12),
            (hover, weight))
    env = _scalars(blocks['standard_arena_envelope'][1])
    r.check("the arena's heights stand in their order",
            0.0 < env['floor_height'] < env['hover_height'] < env['high_height'] < env['top_height'], env)


def test_a_quads_hover_is_solved_once(r):
    """The quadcopter's hover - its rotors' speed, the time they take to it, the stiffness that time
    leaves - is solved in one place (`compile._hover`) for its blocks and its parts alike: every
    governor's idle, the rotors' spin time, the attitude's stiffness and the parts' slew and force
    gains go by one speed, as built and at twice its mass; the rotor's can and torque the module
    names are the parts' own."""
    from machine.aircraft import compile as craft
    from machine.body import mechatronics
    with open(os.path.join(HOST, *CRAFT_SOLVERS[1].split('/')), encoding='utf-8') as f:
        text = f.read()
    r.check("the compiler asks the quad's thrust polar and the rotor's ring once each",
            text.count('polars.kt(') == 1 and text.count("'rotor_ring_share'") == 1,
            (text.count('polars.kt('), text.count("'rotor_ring_share'")))
    stock = mechatronics.crafts()
    twice = dict(stock, quadcopter=dict(stock['quadcopter'], mass_kg=2.0 * float(stock['quadcopter']['mass_kg'])))
    speeds = []
    for name, spec in (('as built', stock), ('at twice its mass', twice)):
        built, quad = craft._build_aircraft(spec), spec['quadcopter']
        blocks, pull = built['blocks'], built['pull']
        weight = float(quad['mass_kg']) * craft.GRAVITY_M_S2
        hover = math.sqrt(weight / (float(quad['rotors']) * blocks['quad_x_16x14'][1]['k_thrust']))
        spin = blocks['quad_direct_governor'][1]['spin_time']
        stiff = blocks['quad_attitude_impedance'][1]['stiffness']['attitude_pitch']
        idles = {block['idle_speed'] for kind, block in blocks.values() if kind == 'governor'}
        speeds.append(hover)
        r.check('%s: its hover thrust is its weight, every governor idles at its hover speed over root six' % name,
                built['defaults'][0] == weight and idles == {float(round(hover / math.sqrt(6.0)))}, idles)
        r.check("%s: the spin time, the attitude's stiffness and the parts' gains go by that one speed" % name,
                spin == round(math.sqrt(0.5) * math.pi * (pull['can_j'] + pull['prop_j']) * hover
                              / craft.ROTOR_MAX_NM, 2)
                and stiff == float(round(2.0 * pull['v_frac'] / (spin * spin), -1))
                and pull['w_slew_rad_s'] == float(round(math.sqrt(2.0) * hover, -2))
                and built['aerokin']['force_a'] == float(round(0.67 * stiff, -2)), (spin, stiff))
    r.check('twice the mass hovers the root of two faster',
            math.isclose(speeds[1], math.sqrt(2.0) * speeds[0], rel_tol=1e-12), speeds)
    pull = craft.part('pull')
    r.check("the rotor's can and torque the module names are the parts' own",
            craft.ROTOR_CAN_J == pull['can_j']
            and craft.ROTOR_MAX_NM == 1.5 * pull['pairs'] * pull['lam'] * pull['i_max'],
            (craft.ROTOR_CAN_J, craft.ROTOR_MAX_NM))


def test_a_crafts_attitude_is_solved_of_its_own(r):
    """Every text that flies has its tilt solved of its own blocks (`machine.aircraft.attitude`): one loop on
    its pitch and its roll, at the least of its ceilings with every need beneath; the quadcopter's, the
    helicopter's and the ornithopter's are the stiffness their blocks hold; a craft's inertia moves its own
    loop alone; nothing flies on the solved loops yet."""
    import copy
    import re
    from machine import aircraft
    from machine.aircraft import attitude
    from machine.body import mechatronics
    rows = attitude.rows()
    texts = [name for kind in aircraft.CRAFTS for name in aircraft.variants(kind)]
    r.check('every text that flies has a pitch and a roll row: %d rows of %d texts' % (len(rows), len(texts)),
            [row[:2] for row in rows] == [(name, loop) for name in texts for loop in ('pitch', 'roll')], rows[:2])
    once = {row[:2]: row for row in rows}
    odd = [key for key, row in once.items() if row[4].kp != min(row[6].values()) or row[6][row[4].ceiling] != row[4].kp
           or row[5][row[4].need] != max(row[5].values()) or row[4].feasible != (max(row[5].values()) <= row[4].kp)
           or set(row[5]) != {'motion', 'cascade'} or not {'dead time', 'pitch authority', 'roll authority'} <= set(row[6])
           or row[4] != once[(key[0], 'pitch')][4]]
    r.check('a tilt is one loop on both axes, at the least of its ceilings, each need and ceiling by its name', not odd, odd)
    stuck = [key for key, row in once.items() if not row[4].feasible]
    r.check('every text can follow its card: its motion and the loop it serves lie under its tilt', not stuck, stuck)
    held = {name: (once[(name, 'pitch')][4].kp, once[(name, 'pitch')][2], once[(name, 'roll')][2])
            for name in (aircraft.CRAFTS[kind] for kind in ('quad', 'helicopter', 'ornithopter'))}
    off = {name: got for name, got in held.items() if abs(got[0] / got[1] - 1.0) > 0.01 or abs(got[0] / got[2] - 1.0) > 0.01}
    r.check("the quadcopter's, the helicopter's and the ornithopter's tilt are the stiffness their blocks hold, within 1 %",
            not off and 'rotor slew' in once[(aircraft.CRAFTS['quad'], 'pitch')][6], held)
    crafts = copy.deepcopy(mechatronics.crafts())
    crafts['helicopter']['gyration_m'] = [math.sqrt(2.0) * g for g in crafts['helicopter']['gyration_m']]
    twice = {row[:2]: row for row in attitude.rows(crafts)}
    halved = [key for key in once if key[0].startswith('helicopter') and abs(twice[key][4].kp / once[key][4].kp - 0.5) > 1e-3]
    moved = [key for key in once if not key[0].startswith('helicopter') and twice[key] != once[key]]
    r.check("twice the helicopter's inertia halves its own tilt's stiffness and moves no other craft's loop",
            not halved and not moved, (halved, moved))
    light_inv = copy.deepcopy(mechatronics.crafts())
    light_inv['shared']['inverter'] = 'coaxial_63050'
    at_50a = {row[:2]: row for row in attitude.rows(light_inv)}
    slew_100 = once[(aircraft.CRAFTS['quad'], 'pitch')][6]['rotor slew']
    slew_50 = at_50a[(aircraft.CRAFTS['quad'], 'pitch')][6]['rotor slew']
    slew_15 = once[('quadcopter_15in', 'pitch')][6]['rotor slew']
    r.check('a craft\'s rotor slew follows its own inverter\'s amps and its own propeller\'s inertia',
            slew_50 < 0.35 * slew_100 and slew_15 > slew_100, (slew_50, slew_100, slew_15))
    readers = sorted(path.replace(os.sep, '/') for path, text, _tree in sources(beside=False)
                     if re.search(r'aircraft\.attitude|from machine\.aircraft import[^\n]*\battitude\b', text))
    flown = [path for path in readers if path.startswith(('machine/', 'coaxial/', 'terminal/'))]
    r.check('no law, rig or page reads the solved loops: they stand beside the blocks', not flown, flown)


ROSTER = (test_a_craft_states_its_own_alone, test_a_wings_lift_slope_is_its_aspect_ratios,
          test_the_floors_friction_is_friction_alone, test_a_heavier_craft_keeps_its_radii,
          test_a_quads_hover_is_solved_once, test_a_crafts_attitude_is_solved_of_its_own)


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
