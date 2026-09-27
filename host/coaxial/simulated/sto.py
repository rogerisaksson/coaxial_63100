"""The STO chain reduced: world_sto.c's levels, thresholds and time constants on the wall's clock.

electronic_simulations/sto/sto.asc's circuit as world/src/world_sto.c steps it: the master's
pilot into Cinj, U4 on Cinj, the keepalive's pump into Clevel, Q10A and U11 onto FAULTOUT, U9's
+15V7.
"""
import math
import time
from typing import Callable

#: The master's pilot (sto.asc's PAM8406): amplitude V, Hz.
PILOT_VOLTS, PILOT_HZ = 1.5, 5000.0

#: Where a pilot holds Cinj up, world_sto.c on a clean bus: 1.5 V from 2 to 10 kHz, 5 kHz from
#: 0.9 V up.
PILOT_BAND_HZ = (2000.0, 10000.0)
PILOT_FLOOR_V = 0.9

#: world_sto.c settled at the nominal pilot, the keepalive every 5 us, V: Cinj, Clevel, +15V7;
#: Cinj with no pilot (R43 over R96) and with U16 unpowered (D13 into its output); Clevel with
#: RESET low (D10's clamp) and with no keepalive.
CINJ_UP, CLEVEL_UP, VGATE_UP = 3.704, 2.857, 14.90
CINJ_REST, CINJ_DOWN = 0.421, 0.022
CLEVEL_HELD, CLEVEL_STOPPED = 0.318, 0.037

#: Its time constants, s: Cinj pumped, leaking (C102 on R96 || R43) and dumped through D13;
#: Clevel (C106 on R98); +15V7 charged (C9 on U9's 2 ohm), sagging (C9 on R28) and crowbarred
#: (Q10B's 3.9 ohm).
TAU_PUMP, TAU_LEAK, TAU_DUMP = 0.74e-3, 2.98e-3, 20e-6
TAU_LEVEL = 0.185e-3
TAU_GATE_UP, TAU_GATE_DOWN, TAU_CROWBAR = 58e-6, 1.5e-3, 108e-6

#: U4 (TPS3840PL30): VIT+ and VIT- on Cinj, its CT's delay and its fall's; Q10A through U11's
#: Schmitt input, Clevel releasing FAULTOUT and tripping it; the LM5069's PGD window, V.
VIT_HI, VIT_LO, CT_S, FALL_S = 3.1, 3.0, 216e-6, 30e-6
RELEASE_V, TRIP_V = 1.550, 1.520
UV_ON, UV_OFF, OV_ON, OV_OFF = 20.3, 18.6, 68.9, 70.8

#: FAULTOUT high, V: U11's 3.3 V into R34.
FAULT_HIGH_V = 3.265

#: The chain's step, s, and how near its targets it counts as settled, V.
STEP_S = 20e-6
SETTLED_V = 1e-4


def _toward(x, target, tau, dt):
    return target + (x - target) * math.exp(-dt / tau)


class SimulatedSto:
    """One board's chain: stepped to the wall's time at every look, its inputs as they stood
    since the last."""

    def __init__(self, link_volts):
        self.pilot = (PILOT_VOLTS, PILOT_HZ, 0.0)
        #: +5 (AFE_ON), and whether main() runs to toggle PA10 - asleep in WFI it does not;
        #: the board wires both.
        self.rail5: Callable[[], bool] = lambda: False
        self.pumping: Callable[[], bool] = lambda: True
        self.link_volts = link_volts
        self.cinj, self.clevel, self.vgate = 0.0, 0.0, 0.0
        self.released = False           # U4's RESET let go
        self._sensed_for = None         # s Cinj has stood over VIT+, U4's CT charging
        self._falling_for = 0.0
        self._c_high = True             # U11's C: Q10A's drain high
        self._pgood = False
        self._at = time.monotonic()

    def set_pilot(self, volts, hz=PILOT_HZ, noise=0.0):
        """The master's pilot from now: amplitude V (0 none), Hz; the far end's noise, V."""
        self.advance()
        self.pilot = (float(volts), float(hz), float(noise))

    @property
    def faultout(self):
        """U11's Y: PE15, BKIN and U9's enable."""
        return (not self._c_high) and self._pgood

    def advance(self):
        """The chain on to now."""
        now = time.monotonic()
        left, self._at = now - self._at, now
        rail5, pumping = self.rail5(), self.pumping()
        volts, hz, _ = self.pilot
        heard = rail5 and volts >= PILOT_FLOOR_V and PILOT_BAND_HZ[0] <= hz <= PILOT_BAND_HZ[1]
        link = self.link_volts
        while left > 0.0:
            dt = min(STEP_S, left)
            left -= dt
            target, tau = ((CINJ_UP, TAU_PUMP) if heard else
                           (CINJ_REST, TAU_LEAK) if rail5 else (CINJ_DOWN, TAU_DUMP))
            self.cinj = _toward(self.cinj, target, tau, dt)
            self._u4(dt)
            level = (CLEVEL_UP if self.released else CLEVEL_HELD) if pumping else CLEVEL_STOPPED
            self.clevel = _toward(self.clevel, level, TAU_LEVEL, dt)
            self._c_high = (self.clevel < RELEASE_V) if self._c_high else (self.clevel < TRIP_V)
            self._pgood = ((UV_OFF < link < OV_OFF) if self._pgood else (UV_ON < link < OV_ON))
            gate, tau = ((VGATE_UP, TAU_GATE_UP) if self.faultout else
                         (0.0, TAU_CROWBAR) if self._c_high else (0.0, TAU_GATE_DOWN))
            self.vgate = _toward(self.vgate, gate, tau, dt)
            if (abs(self.cinj - target) < SETTLED_V and abs(self.clevel - level) < SETTLED_V
                    and abs(self.vgate - gate) < SETTLED_V and self._sensed_for is None
                    and self._falling_for == 0.0):
                break

    def _u4(self, dt):
        """U4 on Cinj: released CT_S after it passes VIT+, held down FALL_S under VIT-."""
        if self.cinj >= VIT_HI:
            self._falling_for = 0.0
            if not self.released:
                self._sensed_for = (self._sensed_for or 0.0) + dt
                if self._sensed_for >= CT_S:
                    self.released, self._sensed_for = True, None
        elif self.cinj < VIT_LO:
            self._sensed_for = None
            if self.released:
                self._falling_for += dt
                if self._falling_for >= FALL_S:
                    self.released, self._falling_for = False, 0.0
        else:
            self._falling_for = 0.0
