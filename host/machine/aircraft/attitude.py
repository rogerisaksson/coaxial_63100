"""A craft's attitude loops solved of its own blocks: its card's need under its frame's ceilings.

    for machine, loop, kp, kd, solved, needs, ceilings in attitude.rows():    # today's beside the solved

Every text that flies (`machine.aircraft.variants`) is compiled to its own blocks by kind
(`aerolisp.compile_manifest`) and its pitch and roll are solved by the kinds it has - no craft is named
here: rotors on arms turn a frame by what each can shed or add of its hover's thrust, a main rotor by its
cyclic, flapping wings and a monoplane's surfaces by their moments at the governed speed. A loop needs the
motion its card asks, its oscillator's frequency with `motion_factor` over it, and the loop along the
floor it serves with `cascade_factor` over that; its ceilings are the phase its dead time may take, the
acceleration all its drive gives over the excursion that may ask it and, for rotors, the rate their moment
turns at. The targets are the library's (`mechatronics.control`), the solver `machine.solver.loops`; a
craft's law takes its gains per inertia, so each loop is solved on an inertia of one, its stiffness in
1/s^2 and its damping in 1/s. The rotors' slew is the quad's own rotor's (`pull.ROTOR`), the one the
crafts' parts hold. The heading's loop has no stated need yet and no row. Nothing flies on the solved
loops: they stand beside today's blocks until the comparison has been read.
"""
import math
import os

from machine import aircraft
from machine.aircraft import aerolisp, compile as craft, pull, quad
from machine.body import mechatronics
from machine.solver import loops

#: A loop's dead time in its law's steps: one of transport and half a one of hold.
DEAD_STEPS = 1.5
#: The loops solved, each with its inertia's key in a rigid body's block (y is up: the heading's).
AXES = (('pitch', 'inertia_xx'), ('roll', 'inertia_zz'))


def _drive(aero, dynamics):
    """{loop: (the moment all a vehicle's drive gives about that axis, N m, the rate that moment turns at,
    N m/s - 0 where nothing lags it)}, by the kinds of blocks the vehicle has."""
    body, gov = dynamics['rigid_body'], dynamics['governor']
    if 'rotors' in aero:
        ro = aero['rotors']
        hover = math.sqrt(body['mass'] * body['gravity'] / (ro['rotors'] * ro['k_thrust']))
        spare = ro['k_thrust'] * min(hover ** 2 - gov['idle_speed'] ** 2, pull.held(pull.I_MAX) ** 2 - hover ** 2)
        turn = 2.0 * ro['k_thrust'] * hover * pull.slew(pull.I_WEP, hover)
        arms = [sum(abs(at[k]) for at in quad.layout(int(ro['rotors']))) * ro['arm_length'] for k in (1, 0)]
        return {loop: (spare * arm, turn * arm) for (loop, _key), arm in zip(AXES, arms)}
    if 'main_rotor' in aero:
        cyclic = aero['main_rotor']['k_cyclic'] * gov['main_speed'] ** 2
        return {'pitch': (cyclic, 0.0), 'roll': (cyclic, 0.0)}
    wings = aero['flapping_wings'] if 'flapping_wings' in aero else aero['aero_surfaces']
    return {'pitch': (wings['k_pitch'] * gov['main_speed'] ** 2, 0.0),
            'roll': (wings['k_roll'] * gov['main_speed'] ** 2, 0.0)}


def _loops(name, manifest, targets):
    """A vehicle's rows (`rows`) of its compiled text."""
    aero, dynamics, kinematics = manifest['aerodynamics'], manifest['dynamics'], manifest['kinematics']
    body, gov, today = dynamics['rigid_body'], dynamics['governor'], dynamics['impedance']
    zeta = float(targets['damping_ratio'])
    motion = math.tau * kinematics['oscillator']['tictoc_frequency']
    along = math.sqrt(kinematics['horizontal']['spot_kp'])
    needs = {'motion': loops.stiffness(1.0, float(targets['motion_factor']) * motion),
             'cascade': loops.stiffness(1.0, float(targets['cascade_factor']) * along)}
    left = math.atan(2.0 * zeta * loops.crossover(zeta)) - math.radians(float(targets['delay_phase_deg']))
    quick = loops.stiffness(1.0, loops.natural(DEAD_STEPS * gov['step_time'], zeta, left))
    drive, out = _drive(aero, dynamics), []
    for loop, key in AXES:
        moment, rate = drive[loop]
        ceilings = {'dead time': quick,
                    'authority': moment / body[key] / math.radians(float(targets['excursion_deg']))}
        if rate:
            ceilings['rotor slew'] = loops.stiffness(1.0, rate / moment)
        out.append((name, loop, today['stiffness']['attitude_' + loop], today['damping']['attitude_' + loop],
                    loops.design(1.0, zeta, needs, ceilings), needs, ceilings))
    return out


def rows(crafts=None):
    """[(machine, loop, today's stiffness 1/s^2, today's damping 1/s, the solved `loops.Loop`, its needs,
    its ceilings)] for every text that flies, its pitch and its roll; `crafts` a resolved crafts table
    compiled in the stock one's place (`compile.blocks`)."""
    targets, out = mechatronics.control(), []
    for kind in aircraft.CRAFTS:
        for name in aircraft.variants(kind):
            with open(os.path.join(aircraft.LISP, name + '.lisp'), encoding='utf-8') as f:
                text = f.read()
            manifest = aerolisp.compile_manifest(text, blocks=None if crafts is None else craft.blocks(crafts))
            out += _loops(name, manifest, targets)
    return out
