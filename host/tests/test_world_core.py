#!/usr/bin/env python3
"""The world core (world/): the loads a machine's motors turn and its body, against closed forms."""
import ctypes
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.cores.build import build, find_cc  # noqa: E402
from tools.emu.world import INCLUDES, SOURCES  # noqa: E402

from test_modbus_core import Report  # noqa: E402

JOINT, ROTOR, WHEEL = 1, 2, 3
GROUND, LIFT, VEHICLE = 0, 1, 2
G = 9.81
TS = 20e-6
#: A motor the size of the board's: 7 pole pairs, 20 uH, 5 mV.s, a rotor of 2e-5 kg m^2.
MOTOR = (0.05, 20e-6, 25e-6, 5e-3, 7.0, 2e-5, 0.0, 24.0, 0.0)


def library():
    lib = ctypes.CDLL(build(find_cc(), SOURCES, INCLUDES, 'world')[0])
    f, i, d = ctypes.c_float, ctypes.c_int, ctypes.c_double
    lib.emu_world_reset.argtypes = [i]
    lib.emu_world_body.argtypes = [i] + [f] * 9
    lib.emu_world_load.argtypes = [i, i] + [f] * 9
    lib.emu_plant_attach.argtypes = [i] + [f] * 9
    lib.emu_plant_step.argtypes = [i, f, f, f, i, f, ctypes.POINTER(f)]
    lib.emu_plant_state.argtypes = [i, ctypes.POINTER(f)]
    lib.emu_world_state.argtypes = [ctypes.POINTER(f)]
    lib.emu_world_motor.argtypes = [i, f, f]
    lib.emu_world_advance.argtypes = [d]
    lib.emu_heat_reset.argtypes = [i, f]
    lib.emu_heat_step.argtypes = [i, f, ctypes.POINTER(f), ctypes.POINTER(f)]
    return lib


def coast(lib, unit, seconds):
    """`seconds` of periods with the gates off; the load's output each period."""
    out, state = (ctypes.c_float * 4)(), (ctypes.c_float * 3)()
    trace = []
    for _ in range(int(round(seconds / TS))):
        lib.emu_plant_step(unit, 0.5, 0.5, 0.5, 0, TS, out)
        lib.emu_plant_state(unit, state)
        trace.append(state[2])
    return trace


def test_a_joint_swings_as_a_pendulum(report, lib):
    """A link hanging from a geared joint, let go at 10 degrees: the period is the pendulum's,
    the rotor's inertia reflected through the gear."""
    gear, inertia, mass, arm = 50.0, 0.02, 1.0, 0.2
    lib.emu_world_reset(1)
    lib.emu_world_body(GROUND, 0.0, G, 0, 0, 0, 0, 0, 0, 0)
    lib.emu_world_load(0, JOINT, gear, inertia, mass, arm, 0.0, 0, 0, 0, math.radians(10))
    lib.emu_plant_attach(0, *MOTOR)
    trace = coast(lib, 0, 3.0)
    crossings = [k for k in range(1, len(trace)) if trace[k - 1] > 0 >= trace[k]]
    period = (crossings[-1] - crossings[0]) * TS / (len(crossings) - 1) if len(crossings) > 1 else 0
    j = inertia + MOTOR[5] * gear * gear
    expect = 2 * math.pi * math.sqrt(j / (mass * G * arm))
    report.check('a joint swings at the pendulum\'s period, the rotor reflected',
                 abs(period / expect - 1) < 0.02, '%.4f s, the closed form %.4f s' % (period, expect))


def test_a_vehicle_rolls_back_down_its_slope(report, lib):
    """Two wheels, no friction, the gates off on 5 %: it rolls back at g sin(slope), the rotors'
    inertia added through the gear."""
    gear, radius, mass, slope = 10.0, 0.3, 100.0, math.atan(0.05)
    lib.emu_world_reset(2)
    lib.emu_world_body(VEHICLE, mass, G, slope, 0, 0, 1.2, 0, 0, 0)
    for unit in (0, 1):
        lib.emu_world_load(unit, WHEEL, gear, 0.0, 0, 0, 0.0, 0, 0, radius, 0)
        lib.emu_plant_attach(unit, *MOTOR)
    out, state = (ctypes.c_float * 4)(), (ctypes.c_float * 3)()
    for _ in range(int(1.0 / TS)):
        for unit in (0, 1):
            lib.emu_plant_step(unit, 0.5, 0.5, 0.5, 0, TS, out)
    lib.emu_plant_state(0, state)
    reflected = 2 * MOTOR[5] * gear * gear / (radius * radius)
    expect = -G * math.sin(slope) * mass / (mass + reflected)
    report.check('a vehicle rolls back down its slope at g sin(slope)',
                 abs(state[2] / expect - 1) < 0.02, '%.4f m/s after 1 s, the closed form %.4f'
                 % (state[2], expect))


def test_a_lift_climbs_on_its_thrust(report, lib):
    """Four rotors at a speed whose thrust is twice the weight: it climbs at g; below the
    weight it stays on the ground."""
    k_thrust, mass = 2e-5, 2.0
    lib.emu_world_reset(4)
    lib.emu_world_body(LIFT, mass, G, 0, 0, 0, 0, 0, 0, 0)
    for unit in range(4):
        lib.emu_world_load(unit, ROTOR, 1.0, 1e-4, 0, 0, 0.0, 0, k_thrust, 0, 0)
    body = (ctypes.c_float * 5)()
    omega = math.sqrt(2 * mass * G / (4 * k_thrust))
    for unit in range(4):
        lib.emu_world_motor(unit, 0.0, omega * 0.5)
    lib.emu_world_advance(1.0)
    lib.emu_world_state(body)
    grounded = body[0]
    for unit in range(4):
        lib.emu_world_motor(unit, 0.0, omega)
    lib.emu_world_advance(2.0)
    lib.emu_world_state(body)
    report.check('a lift holds on the ground below its weight, climbs at g on twice it',
                 grounded == 0.0 and abs(body[0] / (0.5 * G) - 1) < 0.01,
                 'ground %.3f m; %.3f m after 1 s, the closed form %.3f' % (grounded, body[0], 0.5 * G))


def test_the_heat_reads_as_its_observer_models_it(report, lib):
    """A board's heat, thermal.c's network: from the room, each die over its node by its
    watts through R_th,JC at once - 0.666 W through 40.5 K/W, 0.13 W through 3.8 with AFE_ON."""
    load, seen = (ctypes.c_float * 10)(), (ctypes.c_float * 3)()
    load[0] = 1.0
    lib.emu_heat_reset(0, 25.0)
    lib.emu_heat_step(0, 0.1, load, seen)
    ntc, mcu, a1335 = seen
    report.check('the NTC starts at the room, the dies over it by their junctions',
                 abs(ntc - 25.0) < 0.01 and abs(mcu - ntc - 0.666 * 40.5) < 0.1
                 and abs(a1335 - ntc - 0.13 * 3.8) < 0.1,
                 'NTC %.2f, MCU %.2f, A1335 %.2f C' % (ntc, mcu, a1335))
    for _ in range(1200):
        lib.emu_heat_step(0, 0.1, load, seen)
    report.check("two minutes on, the MCU's package has risen over its patch",
                 seen[1] - seen[0] > 0.666 * 40.5 + 10.0,
                 'MCU %.2f over the NTC' % (seen[1] - seen[0]))


def main():
    report = Report()
    if find_cc() is None:
        print('no C compiler: the world core cannot be built here')
        print('\n0 passed, 0 failed')
        return 0
    lib = library()
    for test in (test_a_joint_swings_as_a_pendulum, test_a_vehicle_rolls_back_down_its_slope,
                 test_a_lift_climbs_on_its_thrust, test_the_heat_reads_as_its_observer_models_it):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report, lib)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
