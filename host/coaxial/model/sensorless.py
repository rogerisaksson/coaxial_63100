"""The sensorless design arithmetic: what the measurements buy."""
import math

from motor.pmsm import RAD_S_PER_RPM, TWO_PI


#: chi-squared at p = 0.05 for 1..7 degrees of freedom, for Ljung-Box.
CHI2_05 = (3.841, 5.991, 7.815, 9.488, 11.070, 12.592, 14.067)

#: The injection sits this many times above the current loop's bandwidth,
#: so the loop's feedback does not answer it. Eight, not ten: a loop at a
#: twentieth of fs and an injection at fs/2 are 10.0 apart, and a floating
#: 1/ts put that comparison on the wrong side by 1e-12 - measured, the
#: budget found nothing on a stand-in whose ts was 2e-5.
INJ_OVER_LOOP = 8.0


def enob(sd_codes, bits=16):
    """Effective bits from the noise floor alone: log2(FS / (sd sqrt 12))."""
    if sd_codes <= 0.0:
        return float(bits)
    return math.log2((1 << bits) / (sd_codes * math.sqrt(12.0)))


def demod_gain(v_inj, ts, ld, lq):
    """Amps per radian of angle error out of the demodulator."""
    if ld <= 0.0 or lq <= 0.0:
        return 0.0
    return 2.0 * v_inj * ts * (lq - ld) / 2.0 / (ld * lq)


def hf_current(v_inj, ts, periods, l_axis):
    """Peak of the injection's current triangle: N steps of V T / L, so N V T /
    (2 L) either side of the fundamental.
    """
    return periods * v_inj * ts / (2.0 * l_axis)


def error_noise(sigma_i, periods, bw_hz, fs):
    """sd of the demodulated error, A, after an estimator of `bw_hz`."""
    return sigma_i * math.sqrt(4.0 * bw_hz / (periods * fs))


def snr(v_inj, periods, ld, lq, sigma_i, bw_hz, fs, ts):
    """(snr_db, sigma_theta_rad) for one injection at one bandwidth."""
    gain = abs(demod_gain(v_inj, ts, ld, lq))
    noise = error_noise(sigma_i, periods, bw_hz, fs)
    if gain <= 0.0 or noise <= 0.0:
        return -100.0, math.pi
    sigma_theta = noise / gain
    return 20.0 * math.log10(1.0 / sigma_theta), sigma_theta


def choose_injection(ld, lq, sigma_i, fs, bw_hz, vdc, i_h_max, f_min_hz=0.0,
                     bw_i_hz=0.0, v_headroom=0.2, max_periods=8,
                     target_db=20.0):
    """The injection with the best SNR under the constraints."""
    ts = 1.0 / fs
    best = None
    for periods in range(1, max_periods + 1):
        f_inj = fs / (2.0 * periods)
        if f_inj < f_min_hz or f_inj < INJ_OVER_LOOP * bw_i_hz:
            continue
        v_by_current = i_h_max * 2.0 * ld / (periods * ts)
        v_inj = min(v_headroom * vdc / math.sqrt(3.0), v_by_current)
        if v_inj <= 0.0:
            continue
        db, sigma_theta = snr(v_inj, periods, ld, lq, sigma_i, bw_hz, fs, ts)
        row = {'periods': periods, 'f_inj_hz': f_inj, 'v_inj': v_inj,
               'gain': demod_gain(v_inj, ts, ld, lq),
               'sigma_theta': sigma_theta, 'snr_db': db,
               'i_h_peak': hf_current(v_inj, ts, periods, ld),
               'limited_by': 'current' if v_by_current < v_headroom * vdc / math.sqrt(3.0) else 'voltage'}
        if best is None or db > best['snr_db']:
            best = row
    if best is not None and best['snr_db'] > target_db:
        # SNR is proportional to the volts: back off to the target.
        scale = 10.0 ** ((target_db - best['snr_db']) / 20.0)
        v_inj = best['v_inj'] * scale
        db, sigma_theta = snr(v_inj, best['periods'], ld, lq, sigma_i, bw_hz,
                              fs, ts)
        best.update(v_inj=v_inj, gain=demod_gain(v_inj, ts, ld, lq),
                    sigma_theta=sigma_theta, snr_db=db,
                    i_h_peak=hf_current(v_inj, ts, best['periods'], ld),
                    limited_by='target')
    return best


def current_loop(r, l, fs, sigma_i, vdc, noise_frac=0.02, max_frac=0.05):
    """Bandwidth and PI gains for the current loop."""
    bw_sample = fs * max_frac
    kp_max = noise_frac * vdc / sigma_i if sigma_i > 0.0 else float('inf')
    bw_noise = kp_max / (TWO_PI * l) if l > 0.0 else float('inf')
    bw = min(bw_sample, bw_noise)
    w = TWO_PI * bw
    return {'bw_hz': bw, 'kp': l * w, 'ki': r * w,
            'limited_by': 'sampling' if bw_sample <= bw_noise else 'noise'}


def kalman_gains(sigma_theta_upd, t_upd, accel_sd, iterations=2000):
    """Steady-state Kalman gains for a constant-velocity angle model."""
    t = t_upd
    q11, q12, q22 = (accel_sd ** 2 * t ** 3 / 3.0, accel_sd ** 2 * t ** 2 / 2.0,
                     accel_sd ** 2 * t)
    r = sigma_theta_upd ** 2
    p11, p12, p22 = 1.0, 0.0, 1.0
    k1 = k2 = 0.0
    for _ in range(iterations):
        # predict: P = F P F' + Q with F = [[1, t], [0, 1]]
        a11 = p11 + 2.0 * t * p12 + t * t * p22 + q11
        a12 = p12 + t * p22 + q12
        a22 = p22 + q22
        # update with H = [1, 0]
        s = a11 + r
        k1, k2 = a11 / s, a12 / s
        p11 = a11 - k1 * a11
        p12 = a12 - k1 * a12
        p22 = a22 - k2 * a12
    wn = math.sqrt(k2 / t) if k2 > 0.0 else 0.0
    zeta = k1 / (2.0 * wn * t) if wn > 0.0 else 0.0
    return {'l1': k1, 'l2': k2, 'wn_hz': wn / TWO_PI,
            'zeta': zeta, 'sigma_theta_est': math.sqrt(max(p11, 0.0))}


def crossover(lam, r, i_max, v_dt_residual, r_uncertainty=0.1, margin=3.0,
              pole_pairs=1.0):
    """The speed where omega lambda clearly exceeds the voltage error floor."""
    floor = v_dt_residual + r_uncertainty * r * i_max
    omega = margin * floor / lam if lam > 0.0 else float('inf')
    return {'floor_volts': floor, 'omega_e': omega,
            'rpm': omega / pole_pairs / RAD_S_PER_RPM}


def decide(snr_db, threshold_db=10.0):
    """'injection' when the budget clears the threshold, else 'if_start'."""
    return 'injection' if snr_db >= threshold_db else 'if_start'


def ljung_box(rho, n):
    """Whiteness of the innovation from its autocorrelation at lags 1..k."""
    rho = list(rho)[:len(CHI2_05)]
    q = sum(n * (n + 2.0) * (rj ** 2) / (n - j - 1)
            for j, rj in enumerate(rho) if n - j - 1 > 0)
    threshold = CHI2_05[len(rho) - 1] if rho else 0.0
    return {'q': q, 'threshold': threshold, 'lags': len(rho),
            'white': q < threshold}


def summary(zero_speed, snr_db, min_speed_pct, iloop_hz, sigma_theta_deg,
            at_pct):
    """The one line a report ends with."""
    return ('zero-speed: %s (SNR %.0f dB), min closed-loop %.0f %%, iloop '
            '%.0f Hz, sigma_theta %.1f deg at %.0f %%'
            % ('yes' if zero_speed else 'no', snr_db, min_speed_pct,
               iloop_hz, sigma_theta_deg, at_pct))


#: A back-EMF observer's speed estimate comes out of its own angle, so it
#: needs a filter to be usable: this is the corner, in rad/s electrical.
#: Fast enough to follow the ramps the drive commands, slow enough that the
#: angle's own noise does not become speed noise.
OMEGA_FILTER_RAD_S = 300.0


def _wrap(angle):
    """To (-pi, pi]."""
    return (angle + math.pi) % TWO_PI - math.pi


class _BackEmfObserver:
    """What the two back-EMF observers share: a filtered speed off their own
    angle, and the wrapping that goes with it.
    """

    def __init__(self):
        self.theta = 0.0
        self.omega = 0.0

    def _advance(self, theta, dt):
        """Take the new angle, update the speed, return the angle."""
        if dt > 0.0:
            raw = _wrap(theta - self.theta) / dt
            alpha = min(1.0, OMEGA_FILTER_RAD_S * dt)
            self.omega += alpha * (raw - self.omega)
        self.theta = theta
        return theta


class FluxObserver(_BackEmfObserver):

    """The rotor angle from the stator flux linkage, stationary frame."""

    def __init__(self, r, l, wc=20.0):
        super().__init__()
        self.r, self.l, self.wc = r, l, wc
        self.psi_alpha = self.psi_beta = 0.0
        self.lam_hat = 0.0

    def update(self, v_alpha, v_beta, i_alpha, i_beta, dt):
        """One step. Returns the estimated electrical angle."""
        self.psi_alpha += dt * (v_alpha - self.r * i_alpha
                                - self.wc * self.psi_alpha)
        self.psi_beta += dt * (v_beta - self.r * i_beta
                               - self.wc * self.psi_beta)
        w = abs(self.omega)
        gain, lead = 1.0, 0.0
        if w > 0.0:
            gain = math.sqrt(1.0 + (self.wc / w) ** 2)
            lead = math.atan2(self.wc, w) * (1.0 if self.omega >= 0.0 else -1.0)
        cos_l, sin_l = math.cos(lead), math.sin(lead)
        psi_a = gain * (self.psi_alpha * cos_l - self.psi_beta * sin_l)
        psi_b = gain * (self.psi_alpha * sin_l + self.psi_beta * cos_l)
        rotor_a = psi_a - self.l * i_alpha
        rotor_b = psi_b - self.l * i_beta
        #: The magnitude the rotor flux carries, which IS lambda wherever
        #: `R i` is small against `v` - the one quantity here that sees the
        #: magnets. Low down an error in R lands in it instead.
        self.lam_hat = math.hypot(rotor_a, rotor_b)
        return self._advance(math.atan2(rotor_b, rotor_a), dt)


class ExtendedStateObserver(_BackEmfObserver):

    """The rotor angle from an extended state observer - ADRC's estimator."""

    def __init__(self, r, l, wo=3000.0):
        super().__init__()
        self.r, self.l, self.wo = r, l, wo
        self.i_alpha = self.i_beta = 0.0
        self.f_alpha = self.f_beta = 0.0

    def update(self, v_alpha, v_beta, i_alpha, i_beta, dt):
        """One step. Returns the estimated electrical angle."""
        b0 = 1.0 / self.l
        beta1, beta2 = 2.0 * self.wo, self.wo * self.wo
        err_a = self.i_alpha - i_alpha
        err_b = self.i_beta - i_beta
        self.i_alpha += dt * (self.f_alpha + b0 * v_alpha - beta1 * err_a)
        self.i_beta += dt * (self.f_beta + b0 * v_beta - beta1 * err_b)
        self.f_alpha -= dt * beta2 * err_a
        self.f_beta -= dt * beta2 * err_b
        # f absorbed -(R i + e)/L, so the back-EMF is what is left of it.
        e_alpha = -(self.l * self.f_alpha + self.r * i_alpha)
        e_beta = -(self.l * self.f_beta + self.r * i_beta)
        return self._advance(math.atan2(-e_alpha, e_beta), dt)


class AdaptiveLuenberger(_BackEmfObserver):

    """A current observer that adapts R while it runs."""

    #: Corner of the low-pass the R adaptation reads its residual
    #: through, rad/s. Far below the back-EMF integrator's own rate: the
    #: two feed on one residual, and whichever is allowed to move fast
    #: owns it.
    ADAPT_CORNER = 20.0

    def __init__(self, r, l, gain=3000.0, ki=3000.0, gamma=0.0):
        super().__init__()
        self.r_hat, self.l = r, l
        self.gain, self.ki, self.gamma = gain, ki, gamma
        self.i_alpha = self.i_beta = 0.0
        self.e_alpha = self.e_beta = 0.0
        self.residual = 0.0

    def update(self, v_alpha, v_beta, i_alpha, i_beta, dt):
        """One step. Returns the estimated electrical angle."""
        err_a = i_alpha - self.i_alpha
        err_b = i_beta - self.i_beta
        self.i_alpha += dt * ((v_alpha - self.r_hat * self.i_alpha
                               - self.e_alpha) / self.l + self.gain * err_a)
        self.i_beta += dt * ((v_beta - self.r_hat * self.i_beta
                              - self.e_beta) / self.l + self.gain * err_b)
        # MINUS, not plus.
        self.e_alpha -= dt * self.ki * err_a
        self.e_beta -= dt * self.ki * err_b
        # The error projected on the current is what a resistance error looks
        # like; the part across it belongs to the back-EMF.
        size = i_alpha * i_alpha + i_beta * i_beta
        if size > 0.0 and self.gamma:
            now = (err_a * i_alpha + err_b * i_beta) / size
            alpha = min(1.0, self.ADAPT_CORNER * dt)
            self.residual += alpha * (now - self.residual)
            self.r_hat = max(0.0, self.r_hat - dt * self.gamma * self.residual)
        return self._advance(math.atan2(-self.e_alpha, self.e_beta), dt)


class DualFluxObserver(_BackEmfObserver):

    """Two flux models correcting each other, with a PLL on the result."""

    def __init__(self, r, l, lam, cross=200.0, kp=200.0, ki=8000.0):
        super().__init__()
        self.r, self.l, self.lam = r, l, lam
        self.cross, self.kp, self.ki = cross, kp, ki
        self.psi_alpha = lam
        self.psi_beta = 0.0

    def update(self, v_alpha, v_beta, i_alpha, i_beta, dt):
        """One step. Returns the estimated electrical angle."""
        # The current model at the angle the PLL is holding.
        model_a = self.l * i_alpha + self.lam * math.cos(self.theta)
        model_b = self.l * i_beta + self.lam * math.sin(self.theta)
        self.psi_alpha += dt * (v_alpha - self.r * i_alpha
                                + self.cross * (model_a - self.psi_alpha))
        self.psi_beta += dt * (v_beta - self.r * i_beta
                               + self.cross * (model_b - self.psi_beta))
        rotor_a = self.psi_alpha - self.l * i_alpha
        rotor_b = self.psi_beta - self.l * i_beta
        size = math.hypot(rotor_a, rotor_b)
        if size <= 0.0:
            return self.theta
        # The PLL's error: the rotor flux's component across the angle the loop
        # holds, which is sin(difference) and needs no atan2.
        eps = (rotor_b * math.cos(self.theta)
               - rotor_a * math.sin(self.theta)) / size
        self.omega += self.ki * eps * dt
        self.theta = _wrap(self.theta + (self.omega + self.kp * eps) * dt)
        return self.theta


class SlidingModeObserver(_BackEmfObserver):

    """The rotor angle from a sliding-mode current observer."""

    def __init__(self, r, l, k, wc=500.0, boundary=0.5):
        super().__init__()
        self.r, self.l, self.k, self.wc = r, l, k, wc
        self.boundary = boundary
        self.i_alpha = self.i_beta = 0.0
        self.e_alpha = self.e_beta = 0.0

    def _switch(self, error):
        """The switching term, saturated over the boundary layer."""
        if self.boundary <= 0.0:
            return self.k * (1.0 if error > 0.0 else -1.0 if error else 0.0)
        return self.k * max(-1.0, min(1.0, error / self.boundary))

    def update(self, v_alpha, v_beta, i_alpha, i_beta, dt):
        """One step. Returns the estimated electrical angle."""
        z_alpha = self._switch(self.i_alpha - i_alpha)
        z_beta = self._switch(self.i_beta - i_beta)
        self.i_alpha += dt * (v_alpha - self.r * self.i_alpha - z_alpha) / self.l
        self.i_beta += dt * (v_beta - self.r * self.i_beta - z_beta) / self.l
        alpha = min(1.0, self.wc * dt)
        self.e_alpha += alpha * (z_alpha - self.e_alpha)
        self.e_beta += alpha * (z_beta - self.e_beta)
        theta = math.atan2(-self.e_alpha, self.e_beta)
        # The low-pass is a lag of atan(w / wc) on the back-EMF, so it is a lag
        # of the same on the angle taken out of it.
        return self._advance(_wrap(theta + math.atan2(self.omega, self.wc)),
                             dt)
