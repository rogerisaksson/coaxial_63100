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


class ExtendedStateObserver(_BackEmfObserver):

    """The rotor angle from an extended state observer - ADRC's estimator.

    The stator's current equation is `di/dt = v/L + f`, where `f` carries
    everything that is not the applied voltage: the resistive drop, the
    back-EMF, the parameter error, the switching pickup, whatever the
    sense chain is doing. ADRC's move is to stop modelling those
    separately and estimate `f` itself as a state - a total disturbance -
    which is why this is the one observer here with **no low-pass on the
    signal it wants**.

    That matters because the low-pass is what the other two pay for. The
    sliding-mode observer filters its switching term and gets `atan(w/wc)`
    of lag; the flux observer leaks its integrator and gets
    `atan(wc/w)`. Both then correct for the lag using the speed they are
    estimating. This one has no filter to correct, so nothing in its
    angle rests on its own speed estimate.

    One knob, the observer bandwidth `wo`: `beta1 = 2 wo`, `beta2 = wo^2`
    places both poles there. Above the current loop and below the
    switching frequency, or it estimates the ripple as signal.
    """

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

    """A current observer that adapts R while it runs.

    The plain observers are told R once and believe it. A winding goes up
    by a third of a percent per kelvin, so an hour into a mission the R
    they were given is not the R they are looking at - and every one of
    them integrates `v - R i`, so that error lands straight in the
    estimate.

    Here the same current error drives two integrators: `e_hat`, which is
    the back-EMF and moves at the machine's electrical rate, and `r_hat`,
    which moves at a thermal one. `gamma` is small for exactly that
    reason - the two would otherwise explain each other's error, and the
    one that is allowed to move fast wins. R drifts in minutes; nothing
    is lost by adapting it in seconds.
    """

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
        # MINUS, not plus. An `e_hat` that is too small lets the model
        # current run away from the measured one, so a positive error
        # means the back-EMF being subtracted is too small - the sign the
        # other way round is positive feedback, and the observer leaves
        # for infinity in a few hundred steps.
        self.e_alpha -= dt * self.ki * err_a
        self.e_beta -= dt * self.ki * err_b
        # The error projected on the current is what a resistance error
        # looks like; the part across it belongs to the back-EMF. The
        # projection is low-passed before it moves R, because R drifts on
        # a thermal timescale and the residual it is read from is mostly
        # the electrical one.
        size = i_alpha * i_alpha + i_beta * i_beta
        if size > 0.0 and self.gamma:
            now = (err_a * i_alpha + err_b * i_beta) / size
            alpha = min(1.0, self.ADAPT_CORNER * dt)
            self.residual += alpha * (now - self.residual)
            self.r_hat = max(0.0, self.r_hat - dt * self.gamma * self.residual)
        return self._advance(math.atan2(-self.e_alpha, self.e_beta), dt)


class DualFluxObserver(_BackEmfObserver):

    """Two flux models correcting each other, with a PLL on the result.

    The voltage model `integral of (v - R i)` is right at speed and drifts
    at rest; the current model `L i + lambda` is right at rest and wrong
    wherever L or lambda are. Running both and feeding the difference back
    into the integrator is what removes the DC drift without the leak the
    plain flux observer pays for - the current model, not a high-pass, is
    what holds the integrator down.

    The angle comes off a PLL rather than an `atan2` of the flux. An
    `atan2` passes every bit of noise on the flux straight into the angle
    and, through it, into the speed; the PLL is a second-order filter with
    the angle as its state, so the estimate stays smooth across a noisy
    sample and the speed comes out of the loop rather than out of a
    difference.
    """

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
        # The PLL's error: the rotor flux's component across the angle the
        # loop holds, which is sin(difference) and needs no atan2.
        eps = (rotor_b * math.cos(self.theta)
               - rotor_a * math.sin(self.theta)) / size
        self.omega += self.ki * eps * dt
        self.theta = _wrap(self.theta + (self.omega + self.kp * eps) * dt)
        return self.theta


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
