#!/usr/bin/env python3
"""The crafts' spec held to its compilers: a craft states its own alone, a wing's numbers its own.

`crafts.toml` as it is written and as `mechatronics.crafts` resolves it; the aerobat's lift slope
of its wing's aspect ratio and its drag due to lift of the airframe's own span efficiency, a wing a
vehicle's text resizes with them; the floor's friction read for friction alone; a craft on its
heavy frame with its radii of gyration kept.
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
    """`world.toml`'s kinetic share is the floor's under a sole: the robot's gait compiler alone
    names it, for the friction the robot's law counts on. A wing's drag due to lift takes the
    airframe's own span efficiency, and a wing a vehicle's text resizes takes its lift slope and
    that drag of its new aspect ratio."""
    from machine.aircraft import aeroblocks, aerolisp, compile as craft, polars
    from machine.body import compile as body, mechatronics
    env, wing = mechatronics.environment(None), mechatronics.crafts()['aerobat']
    readers = sorted(path.replace(os.sep, '/') for path, text, _tree in sources(beside=False)
                     if 'kinetic_share' in text)
    r.check("the kinetic share of the floor's friction is named by the robot's gait compiler alone",
            readers == ['machine/body/compile.py'], readers)
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


ROSTER = (test_a_craft_states_its_own_alone, test_a_wings_lift_slope_is_its_aspect_ratios,
          test_the_floors_friction_is_friction_alone, test_a_heavier_craft_keeps_its_radii)


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
