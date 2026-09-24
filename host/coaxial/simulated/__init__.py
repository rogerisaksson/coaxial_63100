"""A board that was never plugged in, for exercising the rest of this codebase without a serial port.

The REPL, the spinner, a tool call.
"""

from coaxial.simulated.acquire.capture import SimulatedCapture
from coaxial.simulated.acquire.clock import SimulatedClock
from coaxial.simulated.acquire.daq import SimulatedDaq
from coaxial.simulated.analog import SimulatedAfe, SimulatedAnalog, SimulatedCalibration
from coaxial.simulated.board import SimulatedBoard, SimulatedSession
from coaxial.simulated.drive.device import SimulatedDrive
from coaxial.simulated.link import (BROADCAST_REFUSAL, DEFAULT_BUS, SIMULATED_BUSES, SimulatedLink,
                                    bus_nodes)
from coaxial.simulated.power import SimulatedGateDrivers, SimulatedPower
from coaxial.simulated.sensors import SimulatedAngle, SimulatedImu
from coaxial.simulated.system import (DIGITAL, PARTS, RESERVED, SUBSYSTEMS, SimulatedGpio,
                                      SimulatedSystem, UNITS)
from coaxial.simulated.thermal.observer import SimulatedThermal
from coaxial.simulated.values import (CHANNELS, DCBUS_V, DRIFT, GUST, GUST_CHANCE, NOMINAL,
                                      PITCH_TURNS, RIPPLE, ROLL_TURNS, SWEEP_HZ, SWING, _tumble)
