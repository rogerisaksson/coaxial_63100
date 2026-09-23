"""A board that was never plugged in, for exercising the rest of this codebase
- the REPL, the spinner, a tool call - without touching a serial port at
all.
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
