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
    derived = {'gyration_m', 'ct', 'cp', 'prop_kgm2', 'tail_ct', 'ct_stroke', 'cp_stroke', 'cl_glide',
               'cm_elev', 'cl_ail', 'cn_rud', 'inflow_j0', 'inflow_floor', 'beat_pulse',
               'span_efficiency', 'propwash_share', 'disc_half_m'}
    c_srcs = {os.path.basename(p) for p in mechatronics.craft_sources()}
    r.check('%d keys the crafts state, each read by a compiler and none a solved polar or gyration' % len(stated),
            len(stated) >= 50 and not unread and derived.isdisjoint(stated)
            and {'aero.py', 'drivetrain.py', 'inertia.py', 'contacts.py', 'loops.py'} <= c_srcs,
            (unread, stated & derived))
    props = mechatronics.library()['propellers']
    missing_props = [(n, k, c[k]) for n, c in mechatronics.crafts().items()
                     for k in ('propeller', 'main_rotor', 'tail_rotor') if k in c and c[k] not in props]
    r.check('every craft propeller and rotor names an entry in library/propellers.toml',
            len(props) >= 5 and not missing_props, missing_props)
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
    r.check("the aerobat's drag due to lift and slipstream immersion are solved of its wing, tail and propeller",
            'span_efficiency' not in wing and 'propwash_share' not in wing
            and base['span_efficiency'] == polars.span_efficiency(wing)
            and base['propwash_share'] == polars.propwash(wing, 2.0 * base['prop_radius'])
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
    """The quadcopter's hover - its rotors' speed and spin time - is solved in one place (`compile._hover`)
    from `polars.aero_blocks`' single thrust polar, feeding every governor's idle, the solved attitude
    impedance (`aircraft.solved`) and the parts' slew and force gains, as built and at twice its mass."""
    from machine.aircraft import compile as craft
    from machine.body import mechatronics
    from machine.solver import loops
    with open(os.path.join(HOST, *CRAFT_SOLVERS[1].split('/')), encoding='utf-8') as f:
        text = f.read()
    r.check("the compiler duplicates neither the quad's thrust polar nor the rotor's ring",
            'polars.kt(' not in text and text.count("'rotor_ring_share'") == 1,
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
        q_imp = blocks['quad_attitude_impedance'][1]
        stiff, damp = q_imp['stiffness']['attitude_pitch'], q_imp['damping']['attitude_pitch']
        idles = {block['idle_speed'] for kind, block in blocks.values() if kind == 'governor'}
        speeds.append(hover)
        r.check('%s: its hover thrust is its weight, every governor idles at its hover speed over root six' % name,
                built['defaults'][0] == weight and idles == {float(round(hover / math.sqrt(6.0)))}, idles)
        r.check("%s: the spin time, the solved attitude and Bessel rate gains and the parts' gains go by that speed" % name,
                spin == round(math.sqrt(0.5) * math.pi * (pull['can_j'] + pull['prop_j']) * hover
                              / craft.ROTOR_MAX_NM, 2)
                and q_imp['stiffness']['pitch_rate'] == round(loops.bessel_rate(damp), 1)
                and pull['w_slew_rad_s'] == float(round(math.sqrt(2.0) * hover, -2))
                and built['aerokin']['force_a'] == float(round((2.0 / 3.0) * stiff, -2)), (spin, stiff, damp))
    r.check('twice the mass hovers the root of two faster',
            math.isclose(speeds[1], math.sqrt(2.0) * speeds[0], rel_tol=1e-12), speeds)
    pull = craft.part('pull')
    r.check("the rotor's can and torque the module names are the parts' own",
            craft.ROTOR_CAN_J == pull['can_j']
            and craft.ROTOR_MAX_NM == 1.5 * pull['pairs'] * pull['lam'] * pull['i_max'],
            (craft.ROTOR_CAN_J, craft.ROTOR_MAX_NM))


def test_a_crafts_attitude_is_solved_of_its_own(r):
    """Every text that flies has its pitch, roll and yaw solved of its own blocks (`aircraft.solved`,
    `aircraft.attitude`) and flies on those solved loops (`aircraft.compile`, `aircraft.aeroblocks`):
    tilt at the least of its ceilings, yaw as a rate loop on multirotors and at its stiffest need on 3D
    crafts; a vehicle's mass, inertia, inverter or inline overrides move its own dynamic configuration."""
    import copy
    import types
    from machine import aircraft
    from machine.aircraft import aerobat, aerolisp, attitude, se3mpc
    from machine.body import compile as body_compile, mechatronics, peripherals
    from tools.cores import designed
    rows = attitude.rows()
    texts = [name for kind in aircraft.CRAFTS for name in aircraft.variants(kind)]
    r.check('every text that flies has pitch, roll and yaw rows: %d rows of %d texts' % (len(rows), len(texts)),
            [row[:2] for row in rows] == [(name, loop) for name in texts for loop in ('pitch', 'roll', 'yaw')], rows[:3])
    once = {row[:2]: row for row in rows}
    live_miss = [key for key, row in once.items()
                 if (row[2], row[3]) != (round(row[4].kp, 1), round(row[4].kd, 1))]
    r.check('every flying text and variant flies on its statically solved 3-axis loops, matching designed.py',
            not live_miss and all((rw[2], rw[3]) == (rw[4], rw[5]) for rw in designed.rows()), live_miss)
    odd = []
    for (name, axis), row in once.items():
        lp, needs, ceilings = row[4], row[5], row[6]
        is_quad = name.startswith('quadcopter')
        if axis in ('pitch', 'roll'):
            bad = (lp.kp != min(ceilings.values()) or ceilings[lp.ceiling] != lp.kp
                   or lp != once[(name, 'pitch')][4] or not {'dead time', 'pitch authority', 'roll authority'} <= set(ceilings))
        elif is_quad:
            bad = (lp.kp != 0.0 or lp.kd != min(ceilings.values()) or ceilings[lp.ceiling] != lp.kd
                   or not {'dead time', 'yaw authority', 'rotor slew'} <= set(ceilings))
        else:
            bad = (lp.kp != max(needs.values()) or needs[lp.need] != lp.kp
                   or set(ceilings) != {'dead time', 'yaw authority'})
        if bad or needs[lp.need] != max(needs.values()) or lp.feasible != (max(needs.values()) <= min(ceilings.values())):
            odd.append((name, axis))
    r.check('tilt stands at its least ceiling, multirotor yaw rate at its ceiling, 3D yaw at its stiffest need',
            not odd, odd)
    stuck = [key for key, row in once.items() if not row[4].feasible]
    r.check('every text is feasible on all three axes', not stuck and 'rotor slew' in once[('quadcopter', 'pitch')][6],
            stuck)
    from machine.aircraft import compile as craft_compile
    crafts = copy.deepcopy(mechatronics.crafts())
    d_blocks = craft_compile.blocks()
    b_heli = _scalars(d_blocks['heli_pod_boom_body'][1])
    m_heli = float(crafts['helicopter']['mass_kg'])
    crafts['helicopter']['gyration_m'] = [math.sqrt(2.0 * b_heli[k] / m_heli)
                                          for k in ('inertia_xx', 'inertia_yy', 'inertia_zz')]
    twice = {row[:2]: row for row in attitude.rows(crafts)}
    halved = [key for key in once if key[0].startswith('helicopter') and key[1] != 'yaw'
              and abs(twice[key][4].kp / once[key][4].kp - 0.5) > 1e-3]
    yaw_ceil = [key for key in once if key[0].startswith('helicopter') and key[1] == 'yaw'
                and abs(twice[key][6]['yaw authority'] / once[key][6]['yaw authority'] - 0.5) > 1e-3]
    moved = [key for key in once if not key[0].startswith('helicopter') and twice[key] != once[key]]
    r.check("twice the helicopter's inertia halves its tilt stiffness and yaw ceiling and moves no other craft",
            not halved and not yaw_ceil and not moved, (halved, yaw_ceil, moved))
    geom = copy.deepcopy(mechatronics.crafts())
    p16 = mechatronics.library()['propellers'][geom['quadcopter']['propeller']]
    geom['helicopter']['tail_boom_m'] = 1.5 * float(geom['helicopter']['tail_boom_m'])
    geom['quadcopter']['pitch_m'] = 1.2 * float(p16['pitch_m'])
    g_blocks = craft_compile._build_aircraft(geom)['blocks']
    b0_quad = _scalars(d_blocks['quad_x_16x14'][1])
    bg_heli = _scalars(g_blocks['heli_pod_boom_body'][1])
    bg_quad = _scalars(g_blocks['quad_x_16x14'][1])
    r.check('lengthening tail boom grows solved pitch/yaw inertia and steeper propeller pitch grows BEMT polars',
            bg_heli['inertia_xx'] > b_heli['inertia_xx'] and bg_heli['inertia_yy'] > b_heli['inertia_yy']
            and bg_quad['k_thrust'] > b0_quad['k_thrust'] and bg_quad['k_drag'] > b0_quad['k_drag'],
            (bg_heli['inertia_xx'], b_heli['inertia_xx'], bg_quad['k_thrust'], b0_quad['k_thrust']))
    from machine.aircraft import quad as quad_mod
    mj_craft = craft_compile.part('mujoco')
    rb_quad = _scalars(d_blocks['quad_x_body'][1])
    r.check('quad and airframe MuJoCo models consume statically compiled inertia, contact solref and geometry',
            quad_mod.INERTIA == (rb_quad['inertia_xx'], rb_quad['inertia_yy'], rb_quad['inertia_zz'])
            and quad_mod.SKID_SOLREF_S == mj_craft['skid_solref_s']
            and 'ornithopter' in mj_craft, mj_craft['skid_solref_s'])
    light_inv = copy.deepcopy(mechatronics.crafts())
    light_inv['shared']['inverter'] = 'coaxial_63050'
    at_50a = {row[:2]: row for row in attitude.rows(light_inv)}
    slew_100 = once[(aircraft.CRAFTS['quad'], 'pitch')][6]['rotor slew']
    slew_50 = at_50a[(aircraft.CRAFTS['quad'], 'pitch')][6]['rotor slew']
    slew_15 = once[('quadcopter_15in', 'pitch')][6]['rotor slew']
    r.check('a craft\'s rotor slew and live gains follow its inverter\'s amps and its propeller\'s inertia',
            slew_50 < 0.35 * slew_100 and slew_15 > slew_100
            and at_50a[('quadcopter', 'pitch')][2] == round(at_50a[('quadcopter', 'pitch')][4].kp, 1),
            (slew_50, slew_100, slew_15))
    custom = aerolisp.compile_manifest(
        '(include "aerodynamics.lisp") (include "dynamics.lisp") (include "kinematics.lisp") '
        '(defvehicle heli_custom (use standard_airframe) (use swashplate_550_3d) (use tail_110_antitorque) '
        '(use swashplate_tail_load) (rigid_body (use heli_pod_boom_body) (mass 4.0kg)) '
        '(use finned_aluminium_fan_sink) (use heli_3d_governor) '
        '(impedance (use heli_3d_impedance) (stiffness attitude_yaw 99.0)) (use eso_3d_observers) '
        '(use standard_arena_envelope) (use vertical_climb_brake) (use orbit_circle) '
        '(use heli_tictoc_oscillator) (use hold_tolerances))')
    c_imp = custom['dynamics']['impedance']['stiffness']
    r.check('defvehicle re-solves impedance on inline mass resize while keeping explicit mode overrides',
            abs(c_imp['attitude_pitch'] - 0.5 * once[('helicopter', 'pitch')][2]) < 0.2
            and c_imp['attitude_yaw'] == 99.0, c_imp)
    heavy_plane = types.SimpleNamespace(**{**{k: getattr(aerobat, k) for k in dir(aerobat) if not k.startswith('__')},
                                           'MASS_KG': 2.0 * aerobat.MASS_KG})
    k0, k2 = se3mpc.constants(aerobat), se3mpc.constants(heavy_plane)
    ctl = mechatronics.control()
    r.check('SE(3) MPPI floor penalty scales with craft mass and ESO bandwidths follow control.toml observer_factor',
            k2['kf'] == 2.0 * k0['kf'] and abs(k2['cf'] - 2.0 * k0['cf']) <= 1.0
            and peripherals.bus()['eso_wo'] == round(body_compile._BUS_HZ / float(ctl['observer_factor']), 1),
            (k0['kf'], k2['kf'], k0['cf'], k2['cf']))


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
