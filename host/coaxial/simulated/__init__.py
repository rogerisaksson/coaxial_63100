"""A board that was never plugged in, for exercising the rest of this
codebase - the REPL, the spinner, a tool call - without touching a serial
port at all.

This is not a protocol simulator. Nothing here builds a Modbus frame or
answers one; it is a duck-typed stand-in for `coaxial_mcp.session.Session`
and `coaxial.board.Board`, shaped exactly like the real ones so every tool
in `coaxial_mcp/tools.py` works against it unmodified. Every value it
returns is invented, and every touchpoint says so - `firmware` and `build`
in the version record read literally "simulated", not a plausible-looking
number, so `board_info` alone is enough to tell the two apart. This is the
same principle as invariant 9's AFE-off banner: a label a reader cannot
mistake for the real thing, not a refusal that would just push someone into
guessing instead.

Values otherwise follow the same shapes real hardware was measured to
produce this session: the seven-channel table, mid-scale codes for every
channel with the front end off, small drift on top of a nominal reading
with it on. None of it is a claim about calibration - see `coaxial.scaling`
for the one thing that already is.
"""

from .values import (CHANNELS, DCBUS_V, DRIFT, GUST, GUST_CHANCE, NOMINAL,
                     PITCH_TURNS, RIPPLE, ROLL_TURNS, SWEEP_HZ, SWING,
                     _tumble)
from .link import (BROADCAST_REFUSAL, DEFAULT_BUS, SIMULATED_BUSES,
                   SimulatedLink, bus_nodes)
from .system import (DIGITAL, PARTS, RESERVED, SUBSYSTEMS, SimulatedGpio,
                     SimulatedSystem, UNITS)
from .analog import SimulatedAfe, SimulatedAnalog, SimulatedCalibration
from .sensors import SimulatedAngle, SimulatedImu
from .power import SimulatedGateDrivers, SimulatedPower, SimulatedThermal
from .daq import SimulatedCapture, SimulatedClock, SimulatedDaq
from .drive import SimulatedDrive
from .board import SimulatedBoard, SimulatedSession
