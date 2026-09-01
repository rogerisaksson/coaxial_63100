#!/usr/bin/env python3
"""The control law, on this machine, against a motor that exists only here.

`Drive/` is hardware-free like the Modbus core, and for the same reason: so
the current loop, the injection demodulator, the rotor observer and the I/f ramp
can be run and judged without a motor, a power stage or a cable. This builds
it with the host gcc, drives it through ctypes, and closes the loop through a
PMSM model integrated in Python - saliency, saturation, back-EMF, a dead-time
voltage error, and the two-period pipeline the firmware has between a duty
asked for and the sample that shows it (`PIPELINE` in drive.c).

Every tolerance below is against the model's own truth, never against a
bench number: the model knows its Ld, and the demodulator has to find it.

Run from the host directory:  python tests/test_drive_core.py
"""
import ctypes
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))

from test_modbus_core import Report, build, find_cc          # noqa: E402
sys.path.insert(0, os.path.dirname(HERE))
from coaxial.motor import Motor                              # noqa: E402

REPO = os.path.dirname(os.path.dirname(HERE))
DRIVE = os.path.join(REPO, 'Drive')
SOURCES = [os.path.join(DRIVE, 'test', 'harness.c'),
           os.path.join(DRIVE, 'Src', 'drive.c'),
           os.path.join(DRIVE, 'Src', 'drive_math.c'),
           os.path.join(DRIVE, 'Src', 'drive_model.c')]

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
        got = dict.fromkeys(SETPOINTS, 0.0)
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
            var = (sq / n - mean * mean) if n else None
            out['fields'][name] = {'n': int(n), 'mean': mean,
                                   'sd': math.sqrt(max(0.0, var)) if n else None}
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


# The motor lives in `coaxial.motor` now: the DAQ stand-in, the system
# identification and a notebook all close a loop around the same one,
# and four copies of a machine is four places for an inductance to
# drift. Imported at the top.


def run(drive, motor, seconds, vdc=24.0, noise=0.0, enabled=True, seed=1,
        watch=None):
    """Close the loop for `seconds`. The pipeline is the firmware's: the
    duty a step asks for shapes the period after the next one."""
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


# ---- the tests -----------------------------------------------------------

def test_math(r, lib):
    d = Drive(lib)
    try:
        a, b = d.clarke((1.0, -0.5, -0.5))
        r.check('clarke: a balanced set puts its amplitude on alpha',
                abs(a - 1.0) < 1e-5 and abs(b) < 1e-5, (a, b))
        a, b = d.clarke((1.0 + 0.3, -0.5 + 0.3, -0.5 + 0.3))
        r.check('clarke: a common mode on all three cancels',
                abs(a - 1.0) < 1e-5 and abs(b) < 1e-5, (a, b))
        dq = d.park(*d.inv_park(0.7, -0.2, 1.1), 1.1)
        r.check('park undoes inverse park',
                abs(dq[0] - 0.7) < 1e-5 and abs(dq[1] + 0.2) < 1e-5, dq)
        scale, duty = d.svm(0.0, 0.0, 24.0)
        r.check('svm: the zero vector is 50 % on every leg',
                all(abs(x - 0.5) < 1e-6 for x in duty) and scale == 1.0, duty)
        # The linear range is a line-to-line span of Vdc: at 30 degrees a
        # vector of Vdc/sqrt3 spans exactly that, one leg at 100 % and one
        # at 0. On a phase axis the same vector spans only 1.5 of itself.
        v = 24.0 / math.sqrt(3.0)
        scale, duty = d.svm(v * math.cos(math.pi / 6), v * math.sin(math.pi / 6),
                            24.0)
        r.check('svm: Vdc/sqrt3 at 30 degrees just fits, 100 % and 0 %',
                abs(scale - 1.0) < 1e-5 and abs(max(duty) - 1.0) < 1e-4
                and abs(min(duty)) < 1e-4, (scale, duty))
        scale, duty = d.svm(30.0, 0.0, 24.0)
        r.check('svm: past the linear range the vector is scaled, not clipped',
                abs(scale - 24.0 / 45.0) < 1e-4
                and 0.0 <= min(duty) and max(duty) <= 1.0, (scale, duty))
        scale, duty = d.svm(5.0, 0.0, 0.0)
        r.check('svm: no link means no duty', duty == (0.0, 0.0, 0.0), duty)
        r.check('wrap keeps [0, 2 pi)',
                abs(d.wrap(-0.5) - (TWO_PI - 0.5)) < 1e-5
                and d.wrap(TWO_PI) == 0.0 and abs(d.wrap(7.0) - (7.0 - TWO_PI)) < 1e-5)
        d.params(dt_step=1.0, **{'dt%d' % k: [0.0, 0.4, 0.6, 0.7, 0.75, 0.78,
                                               0.8, 0.8][k] for k in range(8)})
        r.check('dead-time table: interpolated, odd, held past the end',
                abs(d.dt_volts(0.5) - 0.2) < 1e-5
                and abs(d.dt_volts(-1.5) + 0.5) < 1e-5
                and abs(d.dt_volts(20.0) - 0.8) < 1e-5,
                (d.dt_volts(0.5), d.dt_volts(-1.5), d.dt_volts(20.0)))
    finally:
        d.close()


def test_mode_refusals(r, lib):
    d = Drive(lib)
    try:
        why = d.mode(HOLD, powered=False)
        r.check('a mode that measures is refused with the AFE off, and says so',
                why is not None and 'AFE_ON' in why, why)
        why = d.mode(HOLD, enabled=False)
        r.check('a mode that switches is refused with MOE clear, naming arm()',
                why is not None and 'gates.arm()' in why, why)
        r.check('OFF is never refused', d.mode(OFF, False, False) is None)
        r.check('a mode past the list is refused', d.mode(9) is not None)
        d.setpoints(pol_periods=0)
        r.check('polarity with no pulse length is refused',
                d.mode(POLARITY) is not None)
        d.setpoints(pol_periods=5)
        r.check('and taken with one', d.mode(POLARITY) is None)
    finally:
        d.close()


def test_current_loop(r, lib):
    """HOLD on a locked rotor: the dq currents reach their references."""
    d = Drive(lib)
    m = Motor(locked=True, theta=0.3)
    try:
        d.params(**loop_gains(m.r, m.ld0, 1000.0))
        d.setpoints(id_ref=2.0, iq_ref=-1.0, theta=0.3)
        d.mode(HOLD)
        run(d, m, 0.005)
        s = d.state()
        r.check('id reaches 2 A within 5 ms at a 1 kHz loop',
                abs(s['id'] - 2.0) < 0.05, s['id'])
        r.check('iq reaches -1 A', abs(s['iq'] + 1.0) < 0.05, s['iq'])
        r.check('the motor agrees with the controller about the current',
                abs(m.id - 2.0) < 0.05 and abs(m.iq + 1.0) < 0.05, (m.id, m.iq))
        r.check('the demand is about R.i, the integrator carrying it',
                abs(s['vd'] - m.r * 2.0) < 0.02, s['vd'])

        # step response: 63 % inside one time constant of the bandwidth
        d.setpoints(id_ref=4.0)
        tau = 1.0 / (TWO_PI * 1000.0)
        seen = []
        run(d, m, 3.0 * tau, watch=lambda k, dd, mm: seen.append(mm.id))
        at_tau = seen[int(tau / TS) + 2]      # plus the pipeline
        r.check('a 2 A step is 63 %% of the way at one time constant',
                abs((at_tau - 2.0) / 2.0 - 0.63) < 0.12, at_tau)
    finally:
        d.close()


def test_trip_and_stage(r, lib):
    d = Drive(lib)
    m = Motor(locked=True)
    try:
        d.params(i_trip=3.0, **loop_gains(m.r, m.ld0, 1000.0))
        d.setpoints(id_ref=2.0)
        d.mode(HOLD)
        trip, _ = d.step((3.5, -1.75, -1.75), 24.0)
        s = d.state()
        r.check('a phase past i_trip trips, and the mode is OFF',
                trip and s['mode'] == OFF and s['fault'] == 1, s)
        d.mode(HOLD)
        trip, duty = d.step((0.0, 0.0, 0.0), 24.0, enabled=False)
        s = d.state()
        r.check('MOE gone under a running mode: OFF, fault STAGE, no duty',
                not trip and s['mode'] == OFF and s['fault'] == 2
                and duty == (0.0, 0.0, 0.0), s)
        d.params(sign=-1.0)
        d.mode(HOLD)
        run(d, m, 0.0005)
        r.check('with the sign wrong the loop runs away instead of settling',
                abs(m.id) > 0.5 or d.state()['fault'] == 1,
                (m.id, d.state()['fault']))
    finally:
        d.close()


def test_if_spin(r, lib):
    """I/f: a current vector ramped to speed drags the rotor with it, and
    the back-EMF observer finds it once the speed is above w_hi."""
    d = Drive(lib)
    m = Motor(j=1e-5, b=2e-6)
    try:
        d.params(w_lo=60.0, w_hi=120.0,
                 **loop_gains(m.r, m.ld0, 1000.0), **pll_gains(50.0, TS))
        d.setpoints(id_ref=3.0, omega_target=400.0, accel=2000.0)
        d.mode(HOLD)
        run(d, m, 0.4)
        d.window()                              # the ramp is not the run
        run(d, m, 0.1)
        s = d.state()
        r.check('the command frame reached its target speed',
                abs(s['omega_cmd'] - 400.0) < 1e-3, s['omega_cmd'])
        r.check('the rotor follows the current vector',
                abs(m.omega - 400.0) < 20.0, m.omega)
        r.check('omega_hat from the back-EMF is the rotor speed',
                abs(s['omega_hat'] - m.omega) < 0.05 * m.omega,
                (s['omega_hat'], m.omega))
        err = wrap_pi(s['theta_hat'] - m.theta)
        r.check('and theta_hat is the rotor angle, within 0.15 rad',
                abs(err) < 0.15, err)
        # the load angle the host reads lambda from: E in the command frame
        w = d.window()
        vd, vq = w['fields']['vd']['mean'], w['fields']['vq']['mean']
        iid, iq = w['fields']['id']['mean'], w['fields']['iq']['mean']
        omega = s['omega_cmd']
        ed = vd - m.r * iid + omega * m.lq * iq
        eq = vq - m.r * iq - omega * m.ld0 * iid
        lam = math.hypot(ed, eq) / omega
        r.check('lambda recovered from the I/f window within 5 %',
                abs(lam - m.lam) < 0.05 * m.lam, lam)
    finally:
        d.close()


def test_injection_map(r, lib):
    """The demodulator reads the inductance along the injection axis, and
    its q output is the saliency's sine of twice the angle error."""
    d = Drive(lib)
    m = Motor(locked=True, theta=0.7)
    v_inj = 2.0
    try:
        d.params(inj_volts=v_inj, inj_periods=1,
                 **loop_gains(m.r, m.ld0, 500.0))
        d.setpoints(id_ref=0.0, theta=0.7)
        got = {}
        for phi in (0.0, math.pi / 4, math.pi / 2, -math.pi / 4):
            d.params(inj_phase=phi)
            d.mode(HOLD)
            run(d, m, 0.01)
            s = d.state()
            got[phi] = (s['ih'], s['demod_q'])
        ld_seen = v_inj * TS / got[0.0][0]
        lq_seen = v_inj * TS / got[math.pi / 2][0]
        r.check('injection on d: V.T/i_h is Ld within 3 %',
                abs(ld_seen - m.ld0) < 0.03 * m.ld0, ld_seen)
        r.check('injection on q: V.T/i_h is Lq within 3 %',
                abs(lq_seen - m.lq) < 0.03 * m.lq, lq_seen)
        want = v_inj * TS * abs(m.ld0 - m.lq) / 2.0 / (m.ld0 * m.lq)
        r.check('45 degrees off: demod_q is V.T.L_delta/(Ld.Lq), Ld<Lq negative',
                abs(-got[math.pi / 4][1] - want) < 0.1 * want,
                (got[math.pi / 4][1], want))
        r.check('and the other way round it is positive',
                abs(got[-math.pi / 4][1] - want) < 0.1 * want,
                got[-math.pi / 4][1])
        r.check('on axis the q output is zero',
                abs(got[0.0][1]) < 0.05 * want, got[0.0][1])
        # fs/4: two periods a half cycle, same inductance
        d.params(inj_phase=0.0, inj_periods=2)
        d.mode(HOLD)
        d.window()
        run(d, m, 0.01)
        ld2 = v_inj * TS / d.state()['ih']
        r.check('at fs/4 the same inductance comes back',
                abs(ld2 - m.ld0) < 0.03 * m.ld0, ld2)
        w = d.window()
        r.check('the window counted i_h once per four-period cycle',
                abs(w['fields']['ih']['n'] - w['n'] / 4) < 4,
                (w['fields']['ih']['n'], w['n']))
    finally:
        d.close()


def test_saturation_map(r, lib):
    """A bias current on d bends Ld, which is the saliency an SPM has."""
    d = Drive(lib)
    m = Motor(locked=True, theta=0.0, ld=25e-6, lq=25e-6, sat=0.3, i_sat=4.0)
    v_inj = 2.0
    try:
        d.params(inj_volts=v_inj, inj_periods=1, i_max=10.0,
                 **loop_gains(m.r, m.ld0, 500.0))
        seen = {}
        for bias in (-4.0, 0.0, 4.0):
            d.setpoints(id_ref=bias, theta=0.0)
            d.mode(HOLD)
            run(d, m, 0.01)
            seen[bias] = v_inj * TS / d.state()['ih']
        r.check('no bias, no saliency: Ld reads Lq',
                abs(seen[0.0] - 25e-6) < 0.03 * 25e-6, seen[0.0])
        r.check('positive d current saturates: Ld falls',
                seen[4.0] < 0.9 * seen[0.0], seen)
        r.check('negative d current: Ld rises', seen[-4.0] > 1.1 * seen[0.0],
                seen)
    finally:
        d.close()


def test_observer_standstill(r, lib):
    """SENSORLESS at zero speed: injection on theta_hat pulls it onto the
    rotor from 0.4 rad away, with noise on the shunts."""
    d = Drive(lib)
    m = Motor(locked=True, theta=1.0)
    v_inj = 2.0
    try:
        d.params(inj_volts=v_inj, inj_periods=1,
                 eps_gain=eps_gain(v_inj, m.ld0, m.lq),
                 **loop_gains(m.r, m.ld0, 500.0), **pll_gains(80.0, 2 * TS))
        d.setpoints(id_ref=1.0, iq_ref=0.0)
        d.set_theta(0.6)
        d.mode(SENSORLESS)
        run(d, m, 0.1, noise=0.03)
        s = d.state()
        err = wrap_pi(s['theta_hat'] - m.theta)
        r.check('theta_hat converges onto the rotor within 0.03 rad',
                abs(err) < 0.03, err)
        r.check('and omega_hat stays near zero', abs(s['omega_hat']) < 20.0,
                s['omega_hat'])
        d.window()                              # discard the transient
        run(d, m, 0.1, noise=0.03, seed=2)
        w = d.window()
        e = w['fields']['eps']
        rho1 = w['lag'][1] / w['lag'][0] if w['lag'][0] else 1.0
        r.check('the innovation is small and about white at rest',
                e['sd'] < 0.05 and abs(rho1) < 0.5, (e['sd'], rho1))
        r.check('the innovation has no bias', abs(e['mean']) < 0.01, e['mean'])
        # the other basin: pi off, which is what the polarity test is for
        d.set_theta(1.0 + math.pi + 0.3)
        d.mode(SENSORLESS)
        run(d, m, 0.1, noise=0.03)
        err = wrap_pi(d.state()['theta_hat'] - m.theta)
        r.check('started pi away it settles pi away - saliency is even',
                abs(abs(err) - math.pi) < 0.05, err)
    finally:
        d.close()


def test_polarity(r, lib):
    """Two voltage pulses along theta_hat: the one that adds to the magnet
    saturates and peaks higher."""
    d = Drive(lib)
    m = Motor(locked=True, theta=1.0, ld=25e-6, lq=25e-6, sat=0.3, i_sat=4.0)
    try:
        d.params(i_trip=50.0)
        d.setpoints(pol_volts=6.0, pol_periods=8, pol_gap=40)
        d.set_theta(1.0)
        d.mode(POLARITY)
        run(d, m, (2 * 8 + 2 * 40 + 4) * TS)
        s = d.state()
        r.check('polarity ends in OFF on its own', s['mode'] == OFF, s['mode'])
        r.check('aligned: the positive pulse peaks higher',
                s['pol_pos'] > 1.05 * s['pol_neg'] > 0.0,
                (s['pol_pos'], s['pol_neg']))
        m.id = m.iq = 0.0
        d.set_theta(1.0 + math.pi)
        d.mode(POLARITY)
        run(d, m, (2 * 8 + 2 * 40 + 4) * TS)
        s = d.state()
        r.check('pi off: the negative pulse peaks higher',
                s['pol_neg'] > 1.05 * s['pol_pos'] > 0.0,
                (s['pol_pos'], s['pol_neg']))
    finally:
        d.close()


def test_deadtime(r, lib):
    """The inverter's voltage error shows in the demand, and the table
    takes it out again."""
    d = Drive(lib)
    m = Motor(locked=True, v_dt=0.5, i_knee=0.3)
    try:
        d.params(**loop_gains(m.r, m.ld0, 1000.0))
        d.setpoints(id_ref=2.0, theta=0.0)
        d.mode(HOLD)
        run(d, m, 0.01)
        d.window()
        run(d, m, 0.01)
        vd_raw = d.window()['fields']['vd']['mean']
        # A vector on phase a puts I on a and -I/2 on b and c, so the three
        # per-phase errors (-f(I), +f(I/2), +f(I/2)) land on d as
        # (2/3)(f(I) + f(I/2)) - the 4/3 V_dt the textbooks quote, once the
        # smaller current is out of the knee. What the identification has
        # to unfold, and why it sweeps I rather than reading one point.
        f = lambda i: 0.5 * math.tanh(i / 0.3)
        want = m.r * 2.0 + (2.0 / 3.0) * (f(2.0) + f(1.0))
        r.check('uncompensated, vd carries R.i plus (2/3)(f(I) + f(I/2))',
                abs(vd_raw - want) < 0.03, (vd_raw, want))
        table = {'dt%d' % k: 0.5 * math.tanh(k * 0.25 / 0.3) for k in range(8)}
        d.params(dt_step=0.25, **table)
        run(d, m, 0.01)
        d.window()
        run(d, m, 0.01)
        vd_comp = d.window()['fields']['vd']['mean']
        r.check('with the measured table in, vd is R.i again',
                abs(vd_comp - m.r * 2.0) < 0.03, vd_comp)
    finally:
        d.close()


def test_sensorless_run(r, lib):
    """Torque from standstill under injection, through the crossover, onto
    the back-EMF - the rotor observer keeps the rotor the whole way."""
    d = Drive(lib)
    # Friction sets the speed 0.6 A of torque reaches: 0.063 N.m over
    # 5e-4 is 126 rad/s mechanical, 882 electrical - past the crossover
    # and under the voltage limit, so the loop can hold its reference.
    m = Motor(j=2e-5, b=5e-4, theta=2.0)
    v_inj = 2.0
    worst = [0.0]

    def track(k, dd, mm):
        if k > 500:
            worst[0] = max(worst[0], abs(wrap_pi(dd.state()['theta_hat']
                                                 - mm.theta)))
    try:
        d.params(inj_volts=v_inj, inj_periods=1, w_lo=150.0, w_hi=300.0,
                 eps_gain=eps_gain(v_inj, m.ld0, m.lq),
                 **loop_gains(m.r, m.ld0, 500.0), **pll_gains(60.0, 2 * TS))
        d.setpoints(id_ref=0.0, iq_ref=0.0)
        d.set_theta(2.0 + 0.3)
        d.mode(SENSORLESS)
        run(d, m, 0.05, noise=0.02)                 # find the rotor first
        d.setpoints(iq_ref=0.6)
        run(d, m, 0.4, noise=0.02, watch=track)
        s = d.state()
        r.check('the rotor is turning above the crossover',
                m.omega > 300.0, m.omega)
        r.check('omega_hat tracks it within 10 %',
                abs(s['omega_hat'] - m.omega) < 0.1 * m.omega,
                (s['omega_hat'], m.omega))
        r.check('theta_hat never strayed past 0.5 rad after the lock',
                worst[0] < 0.5, worst[0])
        w = d.window()
        r.check('i_q held its reference through the run',
                abs(w['fields']['iq']['mean'] - 0.6) < 0.1,
                w['fields']['iq']['mean'])
    finally:
        d.close()


def test_moments(r, lib):
    d = Drive(lib)
    try:
        d.moments_arm(3)
        for codes in ((10, -5, 7, 30000), (12, -7, 7, 30010), (8, -3, 7, 29990),
                      (999, 999, 999, 999)):
            d.moments_feed(codes)
        m = d.moments()
        u, v, w, dc = m['channels']
        r.check('moments stop at the count asked for', m['n'] == 3, m['n'])
        r.check('sum and sum of squares', u['sum'] == 30 and u['sumsq'] == 308,
                u)
        r.check('lowest and highest', v['lo'] == -7 and v['hi'] == -3, v)
        r.check('a still channel has zero spread', w['sumsq'] * 3 == w['sum'] ** 2,
                w)
        r.check('the DC bus is single-ended and large',
                dc['sum'] == 90000 and dc['lo'] == 29990, dc)
        d.moments_arm(0)
        d.moments_feed((1, 1, 1, 1))
        r.check('zero disarms', d.moments()['n'] == 0)
    finally:
        d.close()


def test_model_agrees(r, lib):
    """The C model in drive_model.c against the Python Motor above, driven
    by the same duties from the same controller."""
    d = Drive(lib)
    m = Motor(j=2e-5, b=1e-5, theta=0.4, sat=0.3, i_sat=4.0, v_dt=0.3,
              i_knee=0.3, sub=4)
    try:
        d.model_params(theta0=0.4, sat=0.3, i_sat=4.0, v_dt=0.3, i_knee=0.3,
                       sub=4.0)
        d.source(True)
        d.params(**loop_gains(m.r, m.ld0, 500.0))
        d.setpoints(id_ref=2.0, iq_ref=0.5, theta=0.0)
        d.mode(HOLD)
        # the Python motor runs on the duties the C step asked for, with the
        # same one-step lag the C model applies
        prev = (0.0, 0.0, 0.0)
        worst = 0.0
        for _ in range(int(0.05 / TS)):
            _, duty = d.step_virtual()
            m.advance(prev, 24.0, TS)
            prev = duty
            c = d.model_state()
            worst = max(worst, abs(c['id'] - m.id), abs(c['iq'] - m.iq),
                        abs(wrap_pi(c['theta'] - m.theta)))
        r.check('C and Python integrate to the same currents and angle '
                '(worst gap under 0.05 A, 0.05 rad)', worst < 0.05, worst)
        # The loop's own frame is the command frame; the model reports the
        # rotor's, which the free rotor has turned away from it.
        s = d.state()
        # A tenth of an ampere: the rotor is free and turning under the
        # torque, so the command frame's back-EMF term keeps moving.
        r.check('the loop holds its reference against the C model',
                abs(s['id'] - 2.0) < 0.1 and abs(s['iq'] - 0.5) < 0.1,
                (s['id'], s['iq']))
    finally:
        d.close()


def test_virtual_sensorless(r, lib):
    """The board-side path: source model, a sensorless lock and a spin,
    the rotor observer judged against the model's own rotor."""
    d = Drive(lib)
    v_inj = 2.0
    try:
        d.model_params(theta0=1.0, b=5e-4, noise=0.02)
        d.source(True)
        d.params(inj_volts=v_inj, inj_periods=1, w_lo=150.0, w_hi=300.0,
                 eps_gain=eps_gain(v_inj, 20e-6, 25e-6),
                 **loop_gains(0.05, 20e-6, 500.0), **pll_gains(60.0, 2 * TS))
        d.setpoints(id_ref=0.0, iq_ref=0.0)
        d.set_theta(1.3)
        why = d.mode(SENSORLESS, enabled=False, powered=False)
        r.check('with the model as source a mode needs neither MOE nor the AFE',
                why is None, why)
        for _ in range(int(0.05 / TS)):
            d.step_virtual()
        err = wrap_pi(d.state()['theta_hat'] - d.model_state()['theta'])
        r.check('the rotor observer locks onto the modelled rotor at rest',
                abs(err) < 0.05, err)
        d.setpoints(iq_ref=0.6)
        worst = 0.0
        for k in range(int(0.4 / TS)):
            trip, _ = d.step_virtual()
            if k > 500:
                worst = max(worst, abs(wrap_pi(d.state()['theta_hat']
                                               - d.model_state()['theta'])))
        s, ms = d.state(), d.model_state()
        r.check('the modelled rotor turns above the crossover',
                ms['omega'] > 300.0, ms['omega'])
        r.check('and the rotor observer follows it through', worst < 0.5
                and abs(s['omega_hat'] - ms['omega']) < 0.1 * ms['omega'],
                (worst, s['omega_hat'], ms['omega']))
        d.source(False)
        why = d.mode(HOLD, enabled=False, powered=False)
        r.check('back on the converters the refusals are back', why is not None)
    finally:
        d.close()


ROSTER = (test_math, test_mode_refusals, test_current_loop,
          test_trip_and_stage, test_if_spin, test_injection_map,
          test_saturation_map, test_observer_standstill, test_polarity,
          test_deadtime, test_sensorless_run, test_moments,
          test_model_agrees, test_virtual_sensorless)


def main():
    cc = find_cc()
    if cc is None:
        print('  SKIP  no host C compiler; setup.ps1 installs one')
        print('\n0 passed, 0 failed')
        return 0
    lib_path, warnings = build(cc, SOURCES, [os.path.join(DRIVE, 'Inc')],
                               name='drivecore')
    lib = ctypes.CDLL(lib_path)
    report = Report()
    report.check('Drive/ builds warning-free with the firmware flags',
                 not warnings, '; '.join(warnings[:3]))
    for test in ROSTER:
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report, lib)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
