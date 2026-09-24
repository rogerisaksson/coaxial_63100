"""The stand-in's captures: the window, the moments, the phase currents with pickup."""
import math
import random
import time

from coaxial.model.sensorless import HALF_SQRT3
from coaxial.simulated.values import DCBUS_V, NOMINAL


class DriveCapture:

    """What the drive records for the host: windows and moments at the sample point."""

    SIGMA_I = 0.02      #: current noise on the shunts, A rms

    #: Raw codes at rest - THE SAME rest point every other path reads
    #: (`NOMINAL`), U, V, W and the DC link. It was a second table with
    #: its own numbers, and `offsets()` tared through this one while the
    #: DAQ records sat on the other: -500 codes of phantom current on
    #: every phase of a recorded frame.
    CENTRE = (NOMINAL[0], NOMINAL[1], NOMINAL[2], NOMINAL[5])

    NOISE = (24.0, 22.0, 25.0, 8.0)              #: codes rms at the quiet point

    #: Amps per code, the stand-in's own scaling (scaling.PHASE_ONBOARD):
    #: 3.3 V over 32768 codes, through 3.5 mohm x 4.5455.
    APC = 3.3 / 32768.0 / (0.0035 * 1500.0 / 330.0)

    #: The shunt chains' gain mismatch the commissioning has to find.
    GAIN = (1.0, 1.01, 0.995)

    def _take_window(self):
        self._converge()
        n = max(1, self._periods_since(self._window_at))
        self._window_at = time.time()
        iid, iq, vd, vq = self._dq()
        ih, eps_amps = self._ih()
        gain = self._p('drv_eps_gain_ua_per_rad', 0.0)
        n_inj = self._p('drv_inj_periods', 1.0) or 1.0
        sd_eps = (self.SIGMA_I / n_inj / gain) if gain else 0.0
        fields = {
            'id': {'n': n, 'mean': iid, 'sd': self.SIGMA_I},
            'iq': {'n': n, 'mean': iq, 'sd': self.SIGMA_I},
            'vd': {'n': n, 'mean': vd, 'sd': 0.01},
            'vq': {'n': n, 'mean': vq, 'sd': 0.01},
            'eps': {'n': n // 2, 'mean': 0.0, 'sd': sd_eps},
            'ih': {'n': n // 2, 'mean': ih, 'sd': self.SIGMA_I / math.sqrt(2)},
            'vdc': {'n': n, 'mean': DCBUS_V, 'sd': 0.003},
        }
        rng = random.Random(n)
        return {'n': n, 'fields': fields,
                'rho': [rng.uniform(-0.008, 0.008) for _ in range(7)],
                'i_peak': math.hypot(iid, iq) + 3 * self.SIGMA_I}

    def _phase_amps(self):
        """The held current vector as the three phase currents, with the gain
        mismatch each shunt chain puts on its own.
        """
        iid, iq, _, _ = self._dq()
        if self._mode not in ('hold', 'volt'):
            return (0.0, 0.0, 0.0)
        th = self._sp['theta']
        ia = iid * math.cos(th) - iq * math.sin(th)
        ib = iid * math.sin(th) + iq * math.cos(th)
        return (ia, -0.5 * ia + HALF_SQRT3 * ib, -0.5 * ia - HALF_SQRT3 * ib)

    def _pickup(self):
        """Switching pickup at the sample point: a bump mid-period, where a 50
        % edge sits, and least near the top of the triangle.
        """
        x = (self._trigger - self.PERIOD / 2.0) / (self.PERIOD / 4.0)
        return 90.0 * math.exp(-x * x)

    def _arm_moments(self, periods):
        self._mom = (time.time(), int(periods))
        return True

    def _read_moments(self):
        if self._mom is None:
            n, want = 0, 0
        else:
            n = min(self._periods_since(self._mom[0]), self._mom[1])
            want = self._mom[1]
        rng = random.Random(self._trigger + n)
        channels = {}
        iabc = self._phase_amps()
        for k, name in enumerate(('Phase U', 'Phase V', 'Phase W', 'DC bus')):
            sd = self.NOISE[k] + (self._pickup() if k < 3 else 0.0)
            mean = self.CENTRE[k] + rng.gauss(0.0, sd / math.sqrt(max(n, 1)))
            if k < 3:
                mean += self.GAIN[k] * iabc[k] / self.APC
            channels[name] = {'mean': mean, 'sd': sd,
                              'lo': int(mean - 3.5 * sd), 'hi': int(mean + 3.5 * sd)}
        return {'done': bool(want) and n >= want, 'n': n, 'want': want,
                'trigger': self._trigger, 'channels': channels}

    def reload(self):
        return True

    def reset_cycles(self):
        self._cycles_max = 0
        return True

    def _move_trigger(self, ticks):
        """The stand-in's sample point, moved by its gate drivers' trigger()."""
        self._trigger = int(ticks)
