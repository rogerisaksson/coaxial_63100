"""A craft's tilt solved of its own blocks: as stiff as its frame allows, its card's motion beneath.

    for machine, loop, kp, kd, solved, needs, ceilings in attitude.rows():    # today's beside the solved

Every text that flies (`machine.aircraft.variants`) is compiled to its own blocks by kind
(`aerolisp.compile_manifest`) and its tilt is solved by the kinds it has - no craft is named here: rotors
on arms turn a frame by what each can shed or add of its hover's thrust, a main rotor by its cyclic,
flapping wings and a monoplane's surfaces by their moments at the governed speed. A tilt is one thing,
the thrust's direction: one stiffness about its pitch and its roll, bound by the weaker of the two - all
the drive that axis has at the excursion that may ask it (`excursion_deg`) -, by the phase the loop's
dead time may take and, for rotors, by the rate their moment turns at; the loop stands at the least of
these (`loops.stiffest`). What it must follow lies beneath: the motion its card asks, its oscillator's
frequency with `motion_factor` over it, and the loop along the floor it serves with `cascade_factor` over
that - feasible where both do. The targets are the library's (`mechatronics.control`); a craft's law
takes its gains per inertia, so the loop is solved on an inertia of one, its stiffness in 1/s^2 and its
damping in 1/s. The rotors' slew is the quad's own rotor's (`pull.ROTOR`), the one the crafts' parts
hold. The heading's loop has no stated need yet and no row. Nothing flies on the solved loops: they
stand beside today's blocks until the comparison has been read.
"""
import math
import os

from machine import aircraft
from machine.aircraft import aerolisp, compile as craft, pull, quad
from machine.body import mechatronics
from machine.solver import loops

#: A loop's dead time in its law's steps: one of transport and half a one of hold.
DEAD_STEPS = 1.5
#: A tilt's axes, each with its inertia's key in a rigid body's block (y is up: the heading's).
AXES = (('pitch', 'inertia_xx'), ('roll', 'inertia_zz'))


def _drive(aero, dynamics, pull_cfg, ref_radius):
    """{loop: (the moment all a vehicle's drive gives about that axis, N m, the rate that moment turns at,
    N m/s - 0 where nothing lags it)}, by the kinds of blocks the vehicle has."""
    body, gov = dynamics['rigid_body'], dynamics['governor']
    if 'rotors' in aero:
        ro = aero['rotors']
        sr = ro['disc_radius'] / ref_radius
        prop_j = pull_cfg['prop_j'] * sr * sr * sr * sr * sr
        rotor = pull.Rotor(pull_cfg['r'], pull_cfg['lq'], pull_cfg['lam'], pull_cfg['pairs'],
                           pull_cfg['can_j'] + prop_j, pull_cfg['b'], ro['k_drag'])
        hover = math.sqrt(body['mass'] * body['gravity'] / (ro['rotors'] * ro['k_thrust']))
        spare = ro['k_thrust'] * min(hover ** 2 - gov['idle_speed'] ** 2,
                                     pull.held(pull_cfg['i_max'], rotor) ** 2 - hover ** 2)
        turn = 2.0 * ro['k_thrust'] * hover * pull.slew(
            pull_cfg['i_wep'], hover, rotor, pull_cfg['i_max'], pull_cfg['w_slew_rad_s'], gov['idle_speed'])
        arms = [sum(abs(at[k]) for at in quad.layout(int(ro['rotors']))) * ro['arm_length'] for k in (1, 0)]
        return {loop: (spare * arm, turn * arm) for (loop, _key), arm in zip(AXES, arms)}
    if 'main_rotor' in aero:
        cyclic = aero['main_rotor']['k_cyclic'] * gov['main_speed'] ** 2
        return {'pitch': (cyclic, 0.0), 'roll': (cyclic, 0.0)}
    wings = aero['flapping_wings'] if 'flapping_wings' in aero else aero['aero_surfaces']
    return {'pitch': (wings['k_pitch'] * gov['main_speed'] ** 2, 0.0),
            'roll': (wings['k_roll'] * gov['main_speed'] ** 2, 0.0)}


def _loops(name, manifest, targets, pull_cfg, ref_radius):
    """A vehicle's rows (`rows`) of its compiled text: its one tilt, on its pitch and on its roll."""
    aero, dynamics, kinematics = manifest['aerodynamics'], manifest['dynamics'], manifest['kinematics']
    body, gov, today = dynamics['rigid_body'], dynamics['governor'], dynamics['impedance']
    zeta = float(targets['damping_ratio'])
    motion = math.tau * kinematics['oscillator']['tictoc_frequency']
    along = math.sqrt(kinematics['horizontal']['spot_kp'])
    needs = {'motion': loops.stiffness(1.0, float(targets['motion_factor']) * motion),
             'cascade': loops.stiffness(1.0, float(targets['cascade_factor']) * along)}
    left = loops.margin(zeta) - math.radians(float(targets['delay_phase_deg']))
    ceilings = {'dead time': loops.stiffness(1.0, loops.natural(DEAD_STEPS * gov['step_time'], zeta, left))}
    drive, excursion = _drive(aero, dynamics, pull_cfg, ref_radius), math.radians(float(targets['excursion_deg']))
    for loop, key in AXES:
        moment, rate = drive[loop]
        ceilings[loop + ' authority'] = moment / body[key] / excursion
        if rate:
            ceilings['rotor slew'] = min(ceilings.get('rotor slew', math.inf), loops.stiffness(1.0, rate / moment))
    tilt = loops.stiffest(1.0, zeta, needs, ceilings)
    return [(name, loop, today['stiffness']['attitude_' + loop], today['damping']['attitude_' + loop],
             tilt, needs, ceilings) for loop, _key in AXES]


def rows(crafts=None):
    """[(machine, loop, today's stiffness 1/s^2, today's damping 1/s, the solved `loops.Loop`, its needs,
    its ceilings)] for every text that flies: its tilt's loop on its pitch and on its roll, beside what
    each holds today; `crafts` a resolved crafts table compiled in the stock one's place
    (`compile.blocks`)."""
    targets, out = mechatronics.control(), []
    built = craft._compiled() if crafts is None else craft._build_aircraft(crafts)
    blocks = None if crafts is None else {
        k: (kind, {sk: dict(sv) if isinstance(sv, dict) else sv for sk, sv in val.items()})
        for k, (kind, val) in built['blocks'].items()}
    pull_cfg, ref_radius = built['pull'], built['blocks']['quad_x_16x14'][1]['disc_radius']
    for kind in aircraft.CRAFTS:
        for name in aircraft.variants(kind):
            with open(os.path.join(aircraft.LISP, name + '.lisp'), encoding='utf-8') as f:
                text = f.read()
            manifest = aerolisp.compile_manifest(text, blocks=blocks)
            out += _loops(name, manifest, targets, pull_cfg, ref_radius)
    return out
