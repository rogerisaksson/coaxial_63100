"""The stand-in's observer chain: the firmware's two observers stepped on its rotor."""
import math
import time

from coaxial.model import sensorless
from coaxial.simulated.drive.locked import _rotor_locked


class DriveObservers:

    """The chain the firmware runs, stepped, skipped in closed form, and blended."""

    #: DRIVE_OBS_WC, the leak the firmware's chain is scaled off.
    OBS_WC = 20.0

    OBS_CROSS = 20.0

    OBS_PLL_KP = 200.0

    OBS_PLL_KI = 8000.0

    OBS_BLEND_LO = 40.0

    OBS_BLEND_HI = 150.0

    #: The longest gap the chain integrates in one call. Half a second
    #: because a step is some 5 us of Python against a 20 us period, so
    #: this stand-in walks the chain at several times real time and a poll
    #: gap costs a fraction of itself. The cap is for a caller that
    #: stopped asking, not for the usual case.
    OBS_WINDOW = 0.5

    @_rotor_locked
    def _observer_chain(self):
        """The two observers themselves, built on the first ask."""

        if self._obs is None:
            self._obs = (
                sensorless.DualFluxObserver(self._r, self._ld(0.0), self._lam,
                                            cross=self.OBS_CROSS,
                                            kp=self.OBS_PLL_KP,
                                            ki=self.OBS_PLL_KI),
                sensorless.FluxObserver(self._r, self._ld(0.0),
                                        wc=self.OBS_WC))
            self._obs_at = time.time()
            self._obs_synced = None
        return self._obs

    def _observer_sync(self, theta, omega):
        """The hand-over `drive_observer_sync` is, for the same reason."""
        dual, flux = self._observer_chain()
        dual.theta = flux.theta = theta
        dual.omega = flux.omega = omega
        dual.psi_alpha = self._lam * math.cos(theta)
        dual.psi_beta = self._lam * math.sin(theta)
        flux.psi_alpha = flux.psi_beta = 0.0
        self._obs_frame = theta
        self._obs_flux_omega = omega
        self._obs_synced = self._mode_at

    @_rotor_locked
    def _read_observers(self):
        """The chain the firmware runs, stepped on this stand-in's rotor."""
        if self._source == 'model':
            self._read_model()
        dual, flux = self._observer_chain()
        # THE HAND-OVER, AND THE ONE THE STAND-IN NEEDS AND THE BOARD DOES NOT.
        if (self._obs_synced != self._mode_at
                or abs(dual.omega) < self.OBS_WC <= abs(self._omega_hat)):
            self._observer_sync(self._theta_hat, self._omega_hat)

        motor = self._machine() if self._source == 'model' else None
        now = time.time()
        elapsed = max(0.0, now - self._obs_at)
        self._obs_at = now
        window = min(elapsed, self.OBS_WINDOW)
        omega = motor.omega if motor is not None else self._omega_hat
        theta = motor.theta if motor is not None else self._theta_hat

        self._observers_skip(dual, flux, elapsed - window)
        self._observers_window(dual, flux, int(window / self.TS), theta, omega)
        w, lo, hi, blend, theta = self._observers_blend(dual, flux)
        return {'valid': w > self.OBS_WC,
                'theta': theta,
                'omega': ((1.0 - blend) * dual.omega
                          + blend * self._obs_flux_omega),
                'blend': blend,
                'dual_theta': dual.theta % (2.0 * math.pi),
                'dual_omega': dual.omega,
                'flux_theta': flux.theta % (2.0 * math.pi),
                'flux_omega': self._obs_flux_omega,
                'lambda_hat': flux.lam_hat,
                'theta_hat': self._theta_hat, 'omega_hat': self._omega_hat,
                'blend_lo': lo, 'blend_hi': hi, 'wc': self.OBS_WC,
                'error': ((theta - self._theta_hat + math.pi)
                          % (2.0 * math.pi) - math.pi)}

    def _observers_skip(self, dual, flux, skipped):
        """THE PERIODS THIS STAND-IN DID NOT STEP, IN CLOSED FORM."""
        if skipped <= 0.0:
            return
        for obs, w in ((dual, dual.omega), (flux, self._obs_flux_omega)):
            turn = w * skipped
            c, s = math.cos(turn), math.sin(turn)
            a, b = obs.psi_alpha, obs.psi_beta
            obs.psi_alpha, obs.psi_beta = a * c - b * s, a * s + b * c
            obs.theta = (obs.theta + turn + math.pi) % (2.0 * math.pi) \
                - math.pi

    def _observers_window(self, dual, flux, n, theta, omega):
        """The window at the end, integrated period by period at the
        firmware's own step, ending at the rotor: the dq solution rotated
        back out to the stationary frame, which is what the board's chain
        gets too.
        """
        iid, iq, vd, vq = self._dq()
        ts = self.TS
        frame = theta - n * ts * omega       # the window ENDS at the rotor
        for _ in range(n):
            c, s = math.cos(frame), math.sin(frame)
            va, vb = vd * c - vq * s, vd * s + vq * c
            ia, ib = iid * c - iq * s, iid * s + iq * c
            dual.update(va, vb, ia, ib, ts)
            # The lag correction rests on the PLL's speed, not on this model's
            # own: at rest `atan(wc/w)` is a quarter turn, and an observer
            # reading its own derivative starts there and cannot get back.
            flux.omega = dual.omega
            flux.update(va, vb, ia, ib, ts)
            self._obs_flux_omega = flux.omega
            frame += omega * ts
        self._obs_frame = frame % (2.0 * math.pi)

    def _observers_blend(self, dual, flux):
        """The two observers' angles blended by the PLL's speed between the two
        marks: (speed, low mark, high mark, blend, angle).
        """
        w = abs(dual.omega)
        lo = self.OBS_BLEND_LO * self.OBS_WC
        hi = self.OBS_BLEND_HI * self.OBS_WC
        blend = min(1.0, max(0.0, (w - lo) / (hi - lo)))
        x = (1.0 - blend) * math.cos(dual.theta) + blend * math.cos(flux.theta)
        y = (1.0 - blend) * math.sin(dual.theta) + blend * math.sin(flux.theta)
        return w, lo, hi, blend, math.atan2(y, x) % (2.0 * math.pi)
