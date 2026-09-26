"""The motor turned for a page to show itself, on a board that is not a real one.

The stand-in's through its model; the emulated MCU's plant through the board's own sensing
(board/emu, source adc, AFE_ON its converter's reference) - with the stage down its phases read
three offsets and their noise, as a bench does. The drive holds a current vector turning at
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

#: How fast the held vector's speed ramps to its target, rad/s^2: the firmware's hold climbs at
#: `accel` and stands still at none - the stand-in's turns at once.
DEMO_ACCEL = 200.0


def _scale(rig):
    """Wall seconds a second of the board's: 1 but on an emulated board."""
    return getattr(getattr(rig.board, 'transport', None), 'time_scale', 1.0) or 1.0


def turn_motor(rig, origin):
    """The per-frame step that runs the motor up and down for the watcher - or None on a real
    board."""
    if not demo(origin):
        return None
    if origin.real:
        # First: the board refuses AFE_ON under an armed stage.
        rig.board.afe.on()
    rig.board.gate_drivers.configure(bypass_break=True)
    rig.board.gate_drivers.on()
    drive = rig.drive
    drive.configure(source='adc' if origin.real else 'model')
    # The stand-in's record clamps the current at 5 A; the meters are 100 A wide.
    drive.configure(drv_i_max=DEMO_AMPS)
    drive.write(id_ref=0.0, iq_ref=0.0, theta=0.0, accel=DEMO_ACCEL,
                omega_target=2.0 * math.pi * DEMO_HZ * _scale(rig))
    drive.hold()
    began = time.monotonic()

    def step(_now=None):
        phase = (time.monotonic() - began) / DEMO_S
        drive.write(id_ref=DEMO_AMPS * 0.5 * (1.0 - math.cos(2.0 * math.pi * phase)),
                    omega_target=2.0 * math.pi * DEMO_HZ * _scale(rig))
    return step


def stop_motor(rig):
    """The drive off, the stage down and the converters given back after a demo: the next page
    on the same board finds it still, and an emulated one idles again. What it did, for the
    closing list."""
    with suppress(RigError):
        rig.drive.off()
        rig.board.gate_drivers.off()
        rig.board.gate_drivers.configure(sync=False)
        return [('demo motor', 'drive off, stage down, converters back')]
    return [('demo motor', 'could not be stopped')]


def cycle_motor(rig, origin, on_s, off_s):
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
            held['step'] = turn_motor(rig, origin)
        elif not on and held['step'] is not None:
            stop_motor(rig)
            held['step'] = None
        if held['step'] is not None:
            held['step']()
    return step
