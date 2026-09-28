#!/usr/bin/env python3
"""The world core (world/): the loads a machine's motors turn and its body, against closed forms;
a board's heat and its STO chain against their circuits."""
import ctypes
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial.model.inverter import GATE_UVLO_V  # noqa: E402
from coaxial.simulated.values import DCBUS_V  # noqa: E402
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
    lib.emu_sto_reset.argtypes = [i]
    lib.emu_sto_step.argtypes = [i, f, ctypes.POINTER(f), i, ctypes.POINTER(f),
                                 ctypes.POINTER(f)]
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


#: The chain's circuit (electronic_simulations/sto, the STO and RS485 sheets): U4's VIT+ and
#: VIT-, Q10A's gate releasing FAULTOUT through U11, U11's 3.3 V into R34's 1.5k past its 16
#: ohm, U9's 15.5 V through 2 ohm into the drivers' 50, their UVLO; the link the bench's.
VIT_HI, VIT_LO, RELEASE_V = 3.1, 3.0, 1.55
FAULT_HIGH = 3.3 * 1500.0 / 1516.0
VGATE = 15.5 * 50.0 / 52.0
UVLO, LINK = GATE_UVLO_V, DCBUS_V
#: PA10's toggle, s: main()'s 200 kHz of edges.
EDGE_S = 5e-6


class Chain:
    """One board's chain stepped as the emulator steps it: PA10 toggled between the calls."""

    def __init__(self, lib, unit=0):
        self.lib, self.unit, self.pin = lib, unit, False
        self.given = (ctypes.c_float * 6)(1.5, 5000.0, 0.0, LINK, 1.0, 0.0)
        self.pins = (ctypes.c_float * 5)()
        lib.emu_sto_reset(unit)

    def run(self, seconds, volts=1.5, hz=5000.0, link=LINK, rail5=True, pump=True):
        """`seconds` on these inputs; (t, Cinj, Clevel, FAULTOUT, +15V7) at each step's end."""
        self.given[0], self.given[1], self.given[3] = volts, hz, link
        self.given[4] = 1.0 if rail5 else 0.0
        rows = []
        for k in range(int(round(seconds / EDGE_S))):
            self.given[5] = 1.0 if self.pin else 0.0
            self.lib.emu_sto_step(self.unit, EDGE_S, self.given, 0, None, self.pins)
            if pump:
                self.pin = not self.pin
            p = self.pins
            rows.append(((k + 1) * EDGE_S, p[0], p[1], p[3], p[4]))
        return rows


def first(rows, test):
    """The first time `test` holds of a row's (Cinj, Clevel, FAULTOUT, +15V7), s, or None."""
    return next((row[0] for row in rows if test(*row[1:])), None)


def test_the_pilot_releases_the_sto_chain(report, lib):
    """From rest, the master's pilot at 1.5 V and 5 kHz, the keepalive at 200 kHz: Cinj over
    U4's VIT+ and under the pump's 5 V, Clevel over Q10A's release and under D10's clamp at
    RESET (0.8 Cinj and a diode), FAULTOUT U11's, +15V7 U9's on the drivers' draw."""
    rows = Chain(lib).run(0.03)
    _, cinj, clevel, fault, vgate = rows[-1]
    report.check('the chain settles released: Cinj, Clevel, FAULTOUT, +15V7 where the parts put them',
                 VIT_HI <= cinj < 5.0 and RELEASE_V < clevel < 0.8 * cinj + 0.4
                 and abs(fault - FAULT_HIGH) < 0.01 and abs(vgate / VGATE - 1.0) < 0.01,
                 'Cinj %.3f, Clevel %.3f, FAULTOUT %.3f, +15V7 %.2f V' % (cinj, clevel, fault, vgate))
    released = first(rows, lambda c, l, f, g: f > FAULT_HIGH / 2)
    over = first(rows, lambda c, l, f, g: c >= VIT_HI)
    report.check("FAULTOUT rises U4's CT (216 us) after Cinj passes VIT+",
                 released is not None and over is not None and 0.15e-3 < released - over < 0.4e-3,
                 'Cinj over VIT+ at %s ms, FAULTOUT high at %s ms'
                 % tuple('%.2f' % (t * 1e3) if t else '-' for t in (over, released)))


def test_each_loss_trips_the_sto_chain(report, lib):
    """Released, then each input lost: the pilot (Cinj down C102's R96 || R43, 2.87 ms, to
    VIT-), +5 (U16B's output takes Cinj through D13), the keepalive (Clevel down R98 C106,
    0.185 ms), the link (PGD); +15V7 under UVLO C9 R28 ln(14.9 / 7.5) = 1.03 ms on."""
    for label, lost, within in (('the pilot', lambda c: c.run(0.01, volts=0.0), 2e-3),
                                ('+5', lambda c: c.run(0.01, rail5=False), 0.3e-3),
                                ('the keepalive', lambda c: c.run(0.01, pump=False), 0.3e-3),
                                ('the link', lambda c: c.run(0.01, link=15.0), 20e-6)):
        chain = Chain(lib)
        chain.run(0.03)
        rows = lost(chain)
        low = first(rows, lambda c, l, f, g: f < FAULT_HIGH / 2)
        unsupplied = first(rows, lambda c, l, f, g: g < UVLO)
        report.check('%s lost: FAULTOUT low within %.2f ms, +15V7 under UVLO within 1.2 ms of it'
                     % (label, within * 1e3),
                     low is not None and low <= within and unsupplied is not None
                     and unsupplied - low <= 1.2e-3,
                     'FAULTOUT low at %s ms, +15V7 under %.1f V at %s ms'
                     % ('%.3f' % (low * 1e3) if low else '-', UVLO,
                        '%.3f' % (unsupplied * 1e3) if unsupplied else '-'))


def test_the_pilot_window(report, lib):
    """What releases the chain on a clean bus: a pilot of 1.5 V from 3 to 10 kHz; 0.5 V under
    U16C's zero cross at 49.5 mV, 20 kHz past the 1 to 10 kHz band-pass."""
    seen = []
    for volts, hz, releases in ((1.5, 3000.0, True), (1.5, 10000.0, True), (0.5, 5000.0, False),
                                (1.5, 20000.0, False)):
        fault = Chain(lib).run(0.03, volts=volts, hz=hz)[-1][3]
        seen.append((volts, hz, fault, (fault > FAULT_HIGH / 2) == releases))
    report.check('the chain releases on the pilot the band-pass and the zero cross pass',
                 all(ok for *_, ok in seen),
                 ', '.join('%.1f V %.0f kHz -> %.2f V' % (v, hz / 1e3, f) for v, hz, f, _ in seen))


def test_the_sto_chain_holds_settled(report, lib):
    """Settled on a steady pilot and keepalive, the chain holds its outputs and skips its
    substeps; the pilot lost ends the hold and trips it as fast as ever."""
    chain = Chain(lib)
    chain.run(0.03)
    steps = int(0.02 / EDGE_S)
    t0 = time.perf_counter()
    chain.run(0.02)
    held = time.perf_counter() - t0
    rows = chain.run(0.01, volts=0.0)
    low = first(rows, lambda c, l, f, g: f < FAULT_HIGH / 2)
    report.check('held, a step costs the call alone; the pilot lost trips it within 2 ms',
                 held / steps < 20e-6 and low is not None and low <= 2e-3,
                 '%.1f us a step held, FAULTOUT low %s ms after the pilot went'
                 % (held / steps * 1e6, '%.3f' % (low * 1e3) if low else '-'))
    # Idle: AFE_ON off, main() in WFI toggling PA10 a SysTick apart - down, and held down.
    chain.given[4] = 0.0
    t0 = time.perf_counter()
    for _ in range(1000):
        chain.given[5] = 1.0 if chain.pin else 0.0
        lib.emu_sto_step(chain.unit, 1e-3, chain.given, 0, None, chain.pins)
        chain.pin = not chain.pin
    idle = time.perf_counter() - t0
    report.check('idle with AFE_ON off, down and held: a simulated second in under 50 ms',
                 idle < 0.05 and chain.pins[3] < FAULT_HIGH / 2,
                 '%.1f ms, FAULTOUT %.2f V' % (idle * 1e3, chain.pins[3]))


def test_a_library_outlives_only_its_process(report, _lib):
    """The emulator's sweep (tools.emu.world): another process's world library stays while that
    process runs, an ended one's goes."""
    import subprocess
    from tools.cores.build import OUT
    from tools.emu import world
    ended = subprocess.Popen([sys.executable, '-c', 'pass'])
    ended.wait()
    running = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)'])
    ext = '.dll' if os.name == 'nt' else '.so'
    paths = [os.path.join(OUT, 'world_emu_%d%s' % (p.pid, ext)) for p in (running, ended)]
    for path in paths:
        open(path, 'wb').close()
    world.sweep()
    kept = [os.path.exists(path) for path in paths]
    running.kill()
    running.wait()
    world.sweep()
    report.check("a running process's library kept, an ended one's removed, then the other's",
                 kept == [True, False] and not os.path.exists(paths[0]), kept)


def main():
    report = Report()
    if find_cc() is None:
        print('no C compiler: the world core cannot be built here')
        print('\n0 passed, 0 failed')
        return 0
    lib = library()
    for test in (test_a_joint_swings_as_a_pendulum, test_a_vehicle_rolls_back_down_its_slope,
                 test_a_lift_climbs_on_its_thrust, test_the_heat_reads_as_its_observer_models_it,
                 test_the_pilot_releases_the_sto_chain, test_each_loss_trips_the_sto_chain,
                 test_the_pilot_window, test_the_sto_chain_holds_settled,
                 test_a_library_outlives_only_its_process):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report, lib)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
