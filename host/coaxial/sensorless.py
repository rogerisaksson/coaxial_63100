"""The sensorless design arithmetic: what the measurements buy.

No board. The functions are pure; the two observers at the end carry the
state an observer is. The commissioning (commission.py) measures sigma_i,
R, Ld, Lq, lambda and the dead-time curve; this turns them into the
injection to run, the loop and rotor observer gains to run it with, the speed the
back-EMF takes over at, and the decision between injection and an I/f start.

The one definition that runs through all of it: the injection's signal is
the demodulated amps per radian of angle error, K = 2 V T |L_delta| / (Ld Lq)
(drive.c's demodulator, with a square wave of `periods` PWM periods per half
cycle), and its noise after the estimator is sigma_i sqrt(4 BW / (N fs)).
The differences the demodulator sums telescope, so one cycle's estimate
rests on four samples however many periods it spans - the signal is the
same per period, the noise falls as 1/N, and the update rate as 1/N with it.
SNR here is 20 log10(1 rad / sigma_theta): 20 dB is 5.7 degrees of angle
noise at the estimator's bandwidth, 0 dB is a radian, useless.
"""
import math

TWO_PI = 2.0 * math.pi

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
    """Amps per radian of angle error out of the demodulator. Signed: Ld
    below Lq is the usual case and reads positive; the drive takes the sign
    as given so a motor with Ld above Lq still converges."""
    if ld <= 0.0 or lq <= 0.0:
        return 0.0
    return 2.0 * v_inj * ts * (lq - ld) / 2.0 / (ld * lq)


def hf_current(v_inj, ts, periods, l_axis):
    """Peak of the injection's current triangle: N steps of V T / L, so
    N V T / (2 L) either side of the fundamental."""
    return periods * v_inj * ts / (2.0 * l_axis)


def error_noise(sigma_i, periods, bw_hz, fs):
    """sd of the demodulated error, A, after an estimator of `bw_hz`.

    One cycle's sum telescopes to four samples, so its noise is 2 sigma_i
    over 2N periods - sigma_i / N per update - and the estimator at rate
    fs/(2N) passes 2 BW / (fs / 2N) of that power: sigma_i sqrt(4 BW/(N fs)).
    """
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
    """The injection with the best SNR under the constraints.

    fs/2 with a small amplitude when the AFE is quiet; fewer, larger cycles
    when it is not - the same volts buy N times the current, the estimate
    rests on the same four samples, and the noise falls as 1/N. Constraints:
    `i_h_max` caps the HF current peak, `f_min_hz` keeps it out of the
    audible band, the injection stays INJ_OVER_LOOP times above the current
    loop's bandwidth, and the amplitude stays inside `v_headroom` of
    Vdc/sqrt3. None when nothing fits.

    Which way the trade goes depends on what binds. Under the current
    ceiling the volts fall as 1/N and the SNR with them, so fs/2 wins; under
    the voltage headroom the same volts buy N times the current and lower
    frequencies win. A quiet AFE clears `target_db` at fs/2 and is given
    the smallest amplitude that does - less loss, less noise from the
    stator - rather than the largest the constraints allow.
    """
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
    """Bandwidth and PI gains for the current loop.

    Pole-zero cancellation: kp = L w, ki = R w. The bandwidth is the lower
    of a twentieth of the sampling rate - two periods of delay want the
    phase margin - and the point where the noise the loop feeds back,
    kp sigma_i, reaches `noise_frac` of the link.
    """
    bw_sample = fs * max_frac
    kp_max = noise_frac * vdc / sigma_i if sigma_i > 0.0 else float('inf')
    bw_noise = kp_max / (TWO_PI * l) if l > 0.0 else float('inf')
    bw = min(bw_sample, bw_noise)
    w = TWO_PI * bw
    return {'bw_hz': bw, 'kp': l * w, 'ki': r * w,
            'limited_by': 'sampling' if bw_sample <= bw_noise else 'noise'}


def kalman_gains(sigma_theta_upd, t_upd, accel_sd, iterations=2000):
    """Steady-state Kalman gains for a constant-velocity angle model.

    States theta, omega; the measurement is the angle error with sd
    `sigma_theta_upd` per update; the process noise is a white acceleration
    of sd `accel_sd` rad/s^2. The Riccati recursion is iterated to its
    fixed point, so the bandwidth that comes out is the noise's, not a
    knob's: quieter shunts, faster rotor observer.
    """
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
    """The speed where omega lambda clearly exceeds the voltage error floor.

    The floor is what the back-EMF has to be read against: the dead-time
    residual the table leaves, plus the uncertainty in R i at the largest
    current. `margin` times that, over lambda, is the electrical speed the
    back-EMF error takes over at.
    """
    floor = v_dt_residual + r_uncertainty * r * i_max
    omega = margin * floor / lam if lam > 0.0 else float('inf')
    return {'floor_volts': floor, 'omega_e': omega,
            'rpm': omega / TWO_PI / pole_pairs * 60.0}


def decide(snr_db, threshold_db=10.0):
    """'injection' when the budget clears the threshold, else 'if_start'."""
    return 'injection' if snr_db >= threshold_db else 'if_start'


def ljung_box(rho, n):
    """Whiteness of the innovation from its autocorrelation at lags 1..k.

    Q = n (n+2) sum rho_j^2 / (n - j) against chi-squared at p = 0.05 with k
    degrees of freedom. `white` is the executive's verdict; the numbers are
    what it rests on.
    """
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
    """What the two back-EMF observers share: a filtered speed off their
    own angle, and the wrapping that goes with it.

    Both estimate the rotor's angle from the stator equation rather than
    from an injected signal, so both need the speed for their own lag
    compensation, and neither has a speed to start from. The estimate is
    the angle's own derivative, filtered at OMEGA_FILTER_RAD_S.
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

    """The rotor angle from the stator flux linkage, stationary frame.

    `psi = integral of (v - R i)` is the stator's flux and `psi - L i` is
    the rotor's, whose angle is the rotor's. A pure integrator walks away
    on any offset in v, in the current, or in R, so the integrator is a
    low-pass at `wc` instead - and that costs exactly what it saves: at
    electrical speed w the estimate is short by `sqrt(1 + (wc/w)^2)` and
    late by `atan(wc/w)`, which this puts back.

    That correction is the observer's floor. At w = wc it is 45 degrees
    and a factor of 1.41; below wc it is neither small nor knowable,
    because the speed it rests on is the one being estimated. `wc` is
    therefore the speed this observer stops holding at, and it cannot be
    lowered without giving the integrator back its drift.
    """

    def __init__(self, r, l, wc=20.0):
        super().__init__()
        self.r, self.l, self.wc = r, l, wc
        self.psi_alpha = self.psi_beta = 0.0

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
        return self._advance(math.atan2(psi_b - self.l * i_beta,
                                        psi_a - self.l * i_alpha), dt)


class SlidingModeObserver(_BackEmfObserver):

    """The rotor angle from a sliding-mode current observer.

    The observer runs the stator's own current equation and drives the
    error to zero with a switching term. Once it is sliding, that term
    IS the back-EMF - it is the only thing the model was missing - so a
    low-pass on it is the estimate, and `e = lambda w (-sin, cos)` gives
    the angle.

    `k` has to exceed the back-EMF the machine can make or the error
    cannot be driven to zero, which is why it is sized from `lambda w_max`
    rather than tuned. `boundary` replaces `sign` with a saturation over
    that many amps: pure switching at a finite step rate chatters, and the
    chatter lands in the estimate. The low-pass costs `atan(w/wc)` of lag,
    which this puts back, and the same trade as the flux observer's sits
    underneath it - the compensation needs the speed it is estimating.
    """

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
        # The low-pass is a lag of atan(w / wc) on the back-EMF, so it is
        # a lag of the same on the angle taken out of it.
        return self._advance(_wrap(theta + math.atan2(self.omega, self.wc)),
                             dt)
