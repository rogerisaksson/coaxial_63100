"""The stand-in's captures: the law's window, and the converters' moments with their pickup."""
import math
import random
import time
from typing import Any

from coaxial.simulated.drive.locked import _rotor_locked
from coaxial.simulated.values import NOMINAL


class DriveCapture:

    """What the drive records for the host: the law's own window, and the raw codes at the
    sample point - the stand-in's front end, invented: its rest, its noise, its gains."""

    # What the class this mixes into brings.
    FS: Any
    PERIOD: Any
    _law: Any
    _now: Any

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

    @_rotor_locked
    def _take_window(self):
        """The law's window since the last take (drive_window_take), as cmd_drive.c's op 5
        answers it: each field's count, mean and deviation, the innovation's autocorrelation."""
        self._now()
        window = self._law.window()
        lag = window.pop('lag')
        window['rho'] = [x / lag[0] if lag[0] > 0.0 else 0.0 for x in lag[1:]]
        return window

    def _phase_amps(self):
        """The three phase currents a held vector leaves on the shunts; none on one that turns,
        whose mean over a count is none."""
        law, row = self._now()
        if law['mode'] not in (1.0, 2.0) or law['omega_cmd']:
            return (0.0, 0.0, 0.0)
        return (row['amps_u'], row['amps_v'], row['amps_w'])

    def _pickup(self):
        """Switching pickup at the sample point: a bump mid-period, where a 50
        % edge sits, and least near the top of the triangle.
        """
        x = (self._trigger - self.PERIOD / 2.0) / (self.PERIOD / 4.0)
        return 90.0 * math.exp(-x * x)

    def _arm_moments(self, periods):
        self._mom = (time.time(), int(periods))
        return True

    @_rotor_locked
    def _read_moments(self):
        if self._mom is None:
            n, want = 0, 0
        else:
            n = min(int((time.time() - self._mom[0]) * self.FS), self._mom[1])
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

    def reset_cycles(self):
        self._cycles_max = 0
        return True

    def _move_trigger(self, ticks):
        """The stand-in's sample point, moved by its gate drivers' trigger()."""
        self._trigger = int(ticks)
