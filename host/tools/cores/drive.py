"""The drive/ core through ctypes: one controller with named access, and a loop closed around a motor."""
import ctypes
import math
import os
import random

from tools import REPO

DRIVE = os.path.join(REPO, 'drive')
SOURCES = [os.path.join(DRIVE, 'test', 'harness.c'),
           os.path.join(DRIVE, 'src', 'drive.c'),
           os.path.join(DRIVE, 'src', 'drive_math.c'),
           os.path.join(DRIVE, 'src', 'drive_model.c'),
           os.path.join(DRIVE, 'src', 'drive_observer.c')]

TS = 20e-6
TWO_PI = 2.0 * math.pi

#: The orders harness.c crosses its arrays in.
PARAMS = ('r', 'ld', 'lq', 'lambda', 'pole_pairs', 'kp', 'ki', 'l1', 'l2',
          'inj_volts', 'inj_periods', 'inj_phase', 'eps_gain', 'i_max',
          'i_trip', 'v_frac', 'sign', 'w_lo', 'w_hi', 'dt_step') + tuple(
              'dt%d' % k for k in range(8))
SETPOINTS = ('id_ref', 'iq_ref', 'theta', 'omega_target', 'accel', 'vd', 'vq',
             'pol_volts', 'pol_periods', 'pol_gap')
STATES = ('theta_hat', 'omega_hat', 'theta_cmd', 'omega_cmd', 'id', 'iq',
          'vd', 'vq', 'eps', 'eps_amps', 'ih', 'mode', 'fault', 'pol_pos',
          'pol_neg', 'periods', 'demod_d', 'demod_q', 'vdc', 'e_bemf',
          'xd', 'xq')
FIELDS = ('id', 'iq', 'vd', 'vq', 'eps', 'ih', 'vdc')
MODEL = ('r', 'ld', 'lq', 'lambda', 'pole_pairs', 'sat', 'i_sat', 'j', 'b',
         'load', 'v_dt', 'i_knee', 'vdc', 'noise', 'theta0', 'sub')
MODEL_DEFAULTS = {'r': 0.05, 'ld': 20e-6, 'lq': 25e-6, 'lambda': 0.005,
                  'pole_pairs': 7.0, 'sat': 0.0, 'i_sat': 5.0, 'j': 2e-5,
                  'b': 1e-5, 'load': 0.0, 'v_dt': 0.0, 'i_knee': 0.3,
                  'vdc': 24.0, 'noise': 0.0, 'theta0': 0.0, 'sub': 4.0}

OFF, VOLT, HOLD, SENSORLESS, POLARITY = range(5)


def wrap_pi(x):
    """To (-pi, pi]."""
    return x - TWO_PI * math.floor((x + math.pi) / TWO_PI)


class Drive:
    """One controller behind the harness, with named access."""

    def __init__(self, lib, ts=TS):
        self.lib = lib
        lib.drv_new.restype = ctypes.c_void_p
        lib.drv_set_mode.restype = ctypes.c_char_p
        lib.drv_svm.restype = ctypes.c_float
        lib.drv_wrap.restype = ctypes.c_float
        lib.drv_dt_volts.restype = ctypes.c_float
        self.h = ctypes.c_void_p(lib.drv_new(ctypes.c_float(ts)))
        self.n_params = lib.drv_param_count()
        self.n_states = lib.drv_state_count()
        self.n_window = lib.drv_window_count()
        self.n_moments = lib.drv_moments_count()
        assert self.n_params == len(PARAMS), self.n_params
        assert self.n_states == len(STATES), self.n_states

    def obs_sync(self, theta, omega):
        """Hand the chain an estimate, the way the drive does on a mode."""
        self.lib.drv_obs_sync(self.h, ctypes.c_float(theta),
                              ctypes.c_float(omega))

    def obs_step(self, va, vb, ia, ib):
        """One observer step on stationary-frame voltage and current."""
        self.lib.drv_obs_step(self.h, ctypes.c_float(va), ctypes.c_float(vb),
                              ctypes.c_float(ia), ctypes.c_float(ib))

    def obs(self):
        """The chain's state: what drv_obs writes, named."""
        buf = (ctypes.c_float * 8)()
        self.lib.drv_obs(self.h, buf, 8)
        return dict(zip(('theta', 'omega', 'blend', 'dual_theta',
                         'dual_omega', 'flux_theta', 'flux_omega',
                         'lambda_hat'), list(buf)))

    def close(self):
        self.lib.drv_free(self.h)

    def params(self, **kw):
        arr = (ctypes.c_float * self.n_params)()
        self.lib.drv_params_get(self.h, arr, self.n_params)
        got = dict(zip(PARAMS, arr))
        got.update(kw)
        arr = (ctypes.c_float * self.n_params)(*[got[k] for k in PARAMS])
        self.lib.drv_params_set(self.h, arr, self.n_params)
        return got

    def setpoints(self, **kw):
        got: dict = dict.fromkeys(SETPOINTS, 0.0)
        got.update(getattr(self, '_sp', {}))
        got.update(kw)
        self._sp = got
        arr = (ctypes.c_float * len(SETPOINTS))(*[got[k] for k in SETPOINTS])
        self.lib.drv_setpoints_set(self.h, arr, len(SETPOINTS))

    def mode(self, mode, enabled=True, powered=True):
        why = self.lib.drv_set_mode(self.h, mode, int(enabled), int(powered))
        return why.decode() if why else None

    def set_theta(self, theta):
        self.lib.drv_set_theta(self.h, ctypes.c_float(theta))

    def step(self, iabc, vdc, enabled=True):
        i = (ctypes.c_float * 3)(*iabc)
        duty = (ctypes.c_float * 3)()
        trip = self.lib.drv_step(self.h, i, ctypes.c_float(vdc),
                                 int(enabled), duty)
        return bool(trip), tuple(duty)

    def state(self):
        arr = (ctypes.c_float * self.n_states)()
        self.lib.drv_state(self.h, arr, self.n_states)
        return dict(zip(STATES, arr))

    def window(self):
        arr = (ctypes.c_double * self.n_window)()
        self.lib.drv_window(self.h, arr, self.n_window)
        v = list(arr)
        out = {'n': int(v[0]), 'fields': {}, 'lag': [], 'i_peak': v[-1]}
        at = 1
        for name in FIELDS:
            n, s, sq = v[at:at + 3]
            at += 3
            mean = s / n if n else None
            var = (sq / n - mean * mean) if mean is not None else None
            out['fields'][name] = {'n': int(n), 'mean': mean,
                                   'sd': (math.sqrt(max(0.0, var))
                                          if var is not None else None)}
        out['lag'] = v[at:at + 8]
        return out

    def moments_arm(self, periods):
        self.lib.drv_moments_arm(self.h, ctypes.c_uint(periods))

    def moments_feed(self, codes):
        self.lib.drv_moments_feed(self.h, (ctypes.c_int * 4)(*codes))

    def moments(self):
        arr = (ctypes.c_double * self.n_moments)()
        self.lib.drv_moments(self.h, arr, self.n_moments)
        v = list(arr)
        out = {'n': int(v[0]), 'want': int(v[1]), 'channels': []}
        for k in range(4):
            s, sq, lo, hi = v[2 + 4 * k:6 + 4 * k]
            out['channels'].append({'sum': s, 'sumsq': sq, 'lo': lo, 'hi': hi})
        return out

    # -- the model as the source -------------------------------------------
    def model_params(self, **kw):
        n = self.lib.drv_model_param_count()
        assert n == len(MODEL), n
        got = dict(MODEL_DEFAULTS)
        got.update(kw)
        arr = (ctypes.c_float * n)(*[got[k] for k in MODEL])
        self.lib.drv_model_params_set(self.h, arr, n)
        return got

    def source(self, model):
        self.lib.drv_source(self.h, int(model))

    def model_state(self):
        arr = (ctypes.c_float * 4)()
        self.lib.drv_model_state(self.h, arr)
        return dict(zip(('theta', 'omega', 'id', 'iq'), arr))

    def step_virtual(self):
        duty = (ctypes.c_float * 3)()
        trip = self.lib.drv_step_virtual(self.h, duty)
        return bool(trip), tuple(duty)

    # -- the arithmetic on its own ---------------------------------------
    def svm(self, va, vb, vdc):
        duty = (ctypes.c_float * 3)()
        scale = self.lib.drv_svm(ctypes.c_float(va), ctypes.c_float(vb),
                                 ctypes.c_float(vdc), duty)
        return scale, tuple(duty)

    def clarke(self, iabc):
        ab = (ctypes.c_float * 2)()
        self.lib.drv_clarke((ctypes.c_float * 3)(*iabc), ab)
        return tuple(ab)

    def park(self, a, b, theta):
        dq = (ctypes.c_float * 2)()
        self.lib.drv_park(ctypes.c_float(a), ctypes.c_float(b),
                          ctypes.c_float(theta), dq)
        return tuple(dq)

    def inv_park(self, d, q, theta):
        ab = (ctypes.c_float * 2)()
        self.lib.drv_inv_park(ctypes.c_float(d), ctypes.c_float(q),
                              ctypes.c_float(theta), ab)
        return tuple(ab)

    def wrap(self, theta):
        return self.lib.drv_wrap(ctypes.c_float(theta))

    def dt_volts(self, amps):
        return self.lib.drv_dt_volts(self.h, ctypes.c_float(amps))


def run(drive, motor, seconds, vdc=24.0, noise=0.0, enabled=True, seed=1,
        watch=None):
    """Close the loop for `seconds` around `motor`, a `coaxial.model.motor.Motor`:
    the machine the DAQ stand-in, the system identification and a notebook
    close their loops around."""
    rng = random.Random(seed)
    prev = (0.0, 0.0, 0.0)
    steps = int(round(seconds / TS))
    for k in range(steps):
        i = motor.currents()
        if noise:
            i = tuple(x + rng.gauss(0.0, noise) for x in i)
        trip, duty = drive.step(i, vdc, enabled)
        motor.advance(prev, vdc, TS)
        prev = duty
        if watch:
            watch(k, drive, motor)
        if trip:
            return True
    return False


def loop_gains(r, l, bw_hz):
    """Pole-zero cancellation: kp = L w, ki = R w."""
    w = TWO_PI * bw_hz
    return {'kp': l * w, 'ki': r * w}


def pll_gains(f_hz, t_update, zeta=0.7):
    """A second-order PLL's per-update gains at natural frequency f."""
    wn = TWO_PI * f_hz
    return {'l1': 2.0 * zeta * wn * t_update, 'l2': wn * wn * t_update}


def eps_gain(v_inj, ld, lq):
    """Demodulated amps per radian: 2 V T |L_delta| / (Ld Lq)."""
    return 2.0 * v_inj * TS * abs(ld - lq) / 2.0 / (ld * lq)
