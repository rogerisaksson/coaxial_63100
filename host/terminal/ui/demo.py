"""The motor turned for a page to show itself, on a board that is not a real one.

The stand-in's through its model; the emulated MCU's plant through the board's own sensing
(board/emu, source adc, AFE_ON its converter's reference) - with the stage down its phases read
three offsets and their noise, as a bench does. AFE_ON up on both: the STO chain's pilot detector
runs off it, and the stage's supply off the chain. The drive holds a current vector turning at
DEMO_HZ electrical (one revolution in ~7 s) and runs it from 0 to DEMO_AMPS and back over DEMO_S.
Paced for the watcher, not the board: on the wall's clock, the vector's rate times the link's
time scale - an emulated board at 50 times real time turned one revolution in six minutes, where
a bench turns it in seven seconds. On a real board nothing here touches the stage.
"""
import math
import time
from contextlib import suppress

from coaxial.comm.hostclock import clock_of
from coaxial.errors import RigError
from terminal.ui.screen import demo

DEMO_HZ = 0.14
DEMO_AMPS = 30.0
DEMO_S = 45.0

#: The demo's setpoint written at most this often, s.
STEP_S = 0.1

#: How fast the held vector's speed ramps to its target, rad/s^2: the firmware's hold climbs at
#: `accel` and stands still at none - the stand-in's turns at once.
DEMO_ACCEL = 200.0


def _scale(rig):
    """Wall seconds a second of the board's: 1 but on an emulated board."""
    return getattr(getattr(rig.board, 'transport', None), 'time_scale', 1.0) or 1.0


def turn_motor(rig, origin, amps=None):
    """The per-frame step that runs the motor up and down for the watcher, or holds `amps` of
    vector - or None on a real board."""
    if not demo(origin):
        return None
    # First: the board refuses AFE_ON under an armed stage. And it is +5 for the STO chain's
    # pilot detector: without it the stage has no supply.
    rig.board.afe.on()
    rig.board.gate_drivers.configure(bypass_break=True)
    rig.board.gate_drivers.on()
    drive = rig.drive
    drive.configure(source='adc' if origin.real else 'model')
    # The stand-in's record clamps the current at 5 A; the meters are 100 A wide.
    drive.configure(drv_i_max=max(DEMO_AMPS, amps or 0.0))
    drive.write(id_ref=0.0, iq_ref=0.0, theta=0.0, accel=DEMO_ACCEL,
                omega_target=2.0 * math.pi * DEMO_HZ * _scale(rig))
    drive.hold()
    began = time.monotonic()
    wrote = {'at': 0.0}

    def step(_now=None):
        # A ramp over DEMO_S: a write every STEP_S, not every frame - a feed's every sample held
        # an emulated board's link.
        now = time.monotonic()
        if now - wrote['at'] < STEP_S:
            return
        wrote['at'] = now
        phase = (now - began) / DEMO_S
        held = DEMO_AMPS * 0.5 * (1.0 - math.cos(2.0 * math.pi * phase)) if amps is None else amps
        drive.write(id_ref=held, omega_target=2.0 * math.pi * DEMO_HZ * _scale(rig))
    return step


#: The shaft's sweep for a page that shows the angle: turns each way, the wall seconds a
#: there-and-back takes, and the current the vector is held at. Measured on the stand-in's
#: plant (2026-09-28): held at 5 A the shaft tracks the commanded angle to 0.6 degrees, and at
#: 30 A it pulls out and runs away at every rate tried, 0.02 to 0.25 rev/s.
SWEEP_TURNS, SWEEP_S, SWEEP_AMPS = 1.0, 63.0, 5.0


def sweep_motor(rig, origin, turns=SWEEP_TURNS, over=SWEEP_S):
    """The per-frame step that sweeps the shaft `turns` turns forward and the same back, round
    again - or None on a real board.

    A raised cosine, so it stands still at both ends. The vector turns at a rate, each aimed at
    where the cosine will be a step ahead of the host's own sum of what it asked, so it neither
    drifts nor is set down a step at a time: set down, each step kicked the held rotor, which
    rang +-0.9 degrees at 8 Hz and bobbed on the page - 832 turnings in 65 s (2026-09-28). Held
    on a free-running vector it hunted 3 degrees at 1.2 Hz (2026-09-27). Paced for the watcher,
    as `turn_motor`: the wall's clock, the rate times the link's time scale.
    """
    if not demo(origin):
        return None
    # First: the board refuses AFE_ON under an armed stage. And it is +5 for the STO chain's
    # pilot detector: without it the stage has no supply.
    rig.board.afe.on()
    rig.board.gate_drivers.configure(bypass_break=True)
    rig.board.gate_drivers.on()
    drive = rig.drive
    drive.configure(source='adc' if origin.real else 'model')
    drive.configure(drv_i_max=max(SWEEP_AMPS, 10.0))
    poles = int(drive.params()['motor_pole_pairs'] or 1)
    scale = _scale(rig)
    drive.write(id_ref=SWEEP_AMPS, iq_ref=0.0, theta=0.0, accel=DEMO_ACCEL, omega_target=0.0)
    drive.hold()
    began = time.monotonic()
    asked = {'at': began, 'angle': 0.0, 'rate': 0.0}

    def step(_now=None):
        # A write every STEP_S, not every frame - a feed's every sample held an emulated
        # board's link.
        now = time.monotonic()
        if now - asked['at'] < STEP_S:
            return
        asked['angle'] += asked['rate'] * (now - asked['at'])
        asked['at'] = now
        ahead = now - began + STEP_S
        target = 2.0 * math.pi * poles * turns * 0.5 * (
            1.0 - math.cos(2.0 * math.pi * (ahead % over) / over))
        asked['rate'] = (target - asked['angle']) / STEP_S
        drive.write(id_ref=SWEEP_AMPS, omega_target=asked['rate'] * scale)
    return step


def stop_motor(rig):
    """The drive off, the stage down, the break and the converters given back after a demo: the
    next page on the same board finds it still, and an emulated one idles again. What it did,
    for the closing list."""
    with suppress(RigError):
        rig.drive.off()
        rig.gates.off()
        rig.board.gate_drivers.configure(sync=False)
        return [('demo motor', 'drive off, stage down, break and converters back')]
    return [('demo motor', 'could not be stopped')]


def cycle_motor(rig, origin, on_s, off_s, amps=None):
    """turn_motor for `on_s` of the board's seconds, stopped for `off_s`, round again: the
    per-frame step, or None on a real board. Stopped, the drive's sync lets the meter go - the
    MCU's die reads again."""
    if not demo(origin):
        return None
    clock = clock_of(rig)
    began = clock.now()
    held: dict = {'step': None}

    def step(_now=None):
        on = (clock.now() - began) % (on_s + off_s) < on_s
        if on and held['step'] is None:
            held['step'] = turn_motor(rig, origin, amps)
        elif not on and held['step'] is not None:
            stop_motor(rig)
            held['step'] = None
        if held['step'] is not None:
            held['step']()
    return step
