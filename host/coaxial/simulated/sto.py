"""The stand-in's STO chain on the C core (`world/src/world_sto.c`, `tools.cores.stand`)."""
import time
from typing import Callable

from tools.cores.stand import StoCore

#: The master's pilot (sto.asc's PAM8406): amplitude V, Hz.
PILOT_VOLTS, PILOT_HZ = 1.5, 5000.0


class SimulatedSto:
    """One board's chain (`world_sto.c`): stepped to the wall's time at every look, its inputs
    as they stood since the last."""

    def __init__(self, link_volts):
        self._core = StoCore()
        self.pilot = (PILOT_VOLTS, PILOT_HZ, 0.0)
        #: +5 (AFE_ON), and whether main() runs to toggle PA10 - asleep in WFI it does not;
        #: the board wires both.
        self.rail5: Callable[[], bool] = lambda: False
        self.pumping: Callable[[], bool] = lambda: True
        self.link_volts = link_volts
        self.cinj, self.clevel, self.faultout, self.vgate = 0.0, 0.0, False, 0.0
        self._at = time.monotonic()

    def set_pilot(self, volts, hz=PILOT_HZ, noise=0.0):
        """The master's pilot from now: amplitude V (0 none), Hz; the far end's noise, V."""
        self.advance()
        self.pilot = (float(volts), float(hz), float(noise))

    def advance(self):
        """The chain on to now."""
        now = time.monotonic()
        dt, self._at = now - self._at, now
        self.cinj, self.clevel, self.faultout, self.vgate = self._core.step(
            dt, self.pilot, self.link_volts, self.rail5(), self.pumping())
