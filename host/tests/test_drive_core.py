#!/usr/bin/env python3
"""The control law, on this host, against a motor that exists only here: modes, the current
loop, an I/f start, trips, polarity, dead time, moments, the model."""
import ctypes
import math
import sys

from machine import pull
from motor.pmsm import Motor
from tools.cores.drive import (HOLD, OFF, POLARITY, SENSORLESS, TS, TWO_PI, Drive, eps_gain,
                               loop_gains, pll_gains, run, wrap_pi)

from drive_kit import suite


def test_math(r, lib):
    d = Drive(lib)
    try:
        a, b = d.clarke((1.0, -0.5, -0.5))
        r.check('clarke: a balanced set puts its amplitude on alpha',
                abs(a - 1.0) < 1e-5 and abs(b) < 1e-5, (a, b))
        a, b = d.clarke((1.0 + 0.3, -0.5 + 0.3, -0.5 + 0.3))
        r.check('clarke: a common mode on all three cancels',
                abs(a - 1.0) < 1e-5 and abs(b) < 1e-5, (a, b))
        alpha, beta = d.inv_park(0.7, -0.2, 1.1)
        dq = d.park(alpha, beta, 1.1)
        r.check('park undoes inverse park',
                abs(dq[0] - 0.7) < 1e-5 and abs(dq[1] + 0.2) < 1e-5, dq)
        scale, duty = d.svm(0.0, 0.0, 24.0)
        r.check('svm: the zero vector is 50 % on every leg',
                all(abs(x - 0.5) < 1e-6 for x in duty) and scale == 1.0, duty)
        # The linear range is a line-to-line span of Vdc: at 30 degrees a
        # vector of Vdc/sqrt3 spans exactly that, one leg at 100 % and one at
        # 0.
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
        lib.drv_tanh.restype, lib.drv_tanh.argtypes = ctypes.c_float, [ctypes.c_float]
        worst = max(abs(lib.drv_tanh(x / 64.0) - math.tanh(x / 64.0))
                    for x in range(-1024, 1025))
        r.check('tanh: within a float\'s few ulps of the library\'s over +-16', worst < 1e-6,
                '%.2e at the worst' % worst)
        r.check('wrap keeps [0, 2 pi), including boundaries past +-10 turns',
                abs(d.wrap(-0.5) - (TWO_PI - 0.5)) < 1e-5
                and d.wrap(TWO_PI) == 0.0 and abs(d.wrap(7.0) - (7.0 - TWO_PI)) < 1e-5
                and d.wrap(-10.0 * TWO_PI - 2e-6) > TWO_PI - 1e-4
                and 0.0 <= d.wrap(10.0 * TWO_PI + 2e-6) < 1e-4
                and d.inv_park(1.0, 0.0, -10.0 * TWO_PI + math.pi - 2e-6)[0] < -0.9999)
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
        r.check('a mode that switches is refused with MOE clear, naming on()',
                why is not None and 'gates.on()' in why, why)
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


def test_off_lets_the_rotor_go(r, lib):
    """OFF at speed: the bridge open, no current, the rotor on its drag, J dw/dt = -b w. The
    zero triple the drive let go with shorted the windings: this model's flywheel from 2 171 to
    255 rpm in 0.5 s, native's from 865 rpm to rest in a second."""
    d = Drive(lib)
    v_inj, j, b = 0.585, 8e-3, 5e-4
    try:
        d.model_params(theta0=0.0, j=j, b=b, noise=0.0)
        d.source(True)
        d.params(inj_volts=v_inj, inj_periods=1, w_lo=180.0, w_hi=360.0, i_max=40.0,
                 i_trip=70.0, eps_gain=eps_gain(v_inj, 20e-6, 25e-6), r=0.05, ld=20e-6,
                 lq=25e-6, **{'lambda': 0.005}, **loop_gains(0.05, 20e-6, 0.05 / TS),
                 l1=0.043, l2=23.6)
        d.setpoints(id_ref=12.0, iq_ref=0.0, theta=0.0, omega_target=0.0)
        d.mode(HOLD, enabled=False, powered=False)
        for _ in range(int(0.2 / TS)):
            d.step_virtual()
        d.setpoints(id_ref=0.0, iq_ref=10.0)
        d.mode(SENSORLESS, enabled=False, powered=False)
        for _ in range(int(1.0 / TS)):
            d.step_virtual()
        w0 = d.model_state()['omega']
        d.mode(OFF, enabled=False, powered=False)
        amps = 0.0
        for k in range(int(0.5 / TS)):
            d.step_virtual()
            if k >= 2:                    # the pipeline: the period asked for before OFF
                m = d.model_state()
                amps = max(amps, math.hypot(m['id'], m['iq']))
        w1 = d.model_state()['omega']
        want = w0 * math.exp(-b / j * 0.5)
        r.check('off at speed: the bridge open, no current, the rotor on its drag alone',
                amps == 0.0 and abs(w1 - want) <= 0.001 * abs(w0),
                '%.0f -> %.0f rpm, the drag %.0f; %.2f A'
                % tuple([w * 60.0 / math.tau / 7.0 for w in (w0, w1, want)] + [amps]))
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
        # per-phase errors (-f(I), +f(I/2), +f(I/2)) land on d as (2/3)(f(I) +
        # f(I/2)) - the 4/3 V_dt the textbooks quote, once the smaller current
        # is out of the knee.
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
    """The C model in drive_model.c against `motor.pmsm.Motor`, driven
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
        # A tenth of an ampere: the rotor is free and turning under the torque,
        # so the command frame's back-EMF term keeps moving.
        r.check('the loop holds its reference against the C model',
                abs(s['id'] - 2.0) < 0.1 and abs(s['iq'] - 0.5) < 0.1,
                (s['id'], s['iq']))
    finally:
        d.close()


def test_montecarlo(r, lib):
    """One of tools/sim/montecarlo.py's jobs, in process: a drawn plant the
    controller was not told about, locked, spun and brought back."""
    from tools.sim import montecarlo as mc
    mc.hold(lib)
    job = {'vdc': 43.0, 'knobs': mc.candidates(4, 1)[0], 'seed': 3}
    row = mc.run_job(job)
    r.check('a drawn plant locks, spins and returns without a trip',
            not row['trip'] and row['sigma_theta'] < 0.35,
            (row['trip'], row['sigma_theta']))
    r.check('the initial error is the draw, and the lock closes it',
            1.0 < row['lock0'] < 1.2 and row['lock'] < 0.2,
            (row['lock0'], row['lock']))
    bemf = mc.run_job(dict(job, bemf_only=True))
    r.check('back-EMF alone loses the same rotor on the way down',
            bemf['min_rpm'] > 0.0, bemf['min_rpm'])
    from motor.catalog import BENCH_MOTOR
    little = {'name': BENCH_MOTOR.name, 'r': BENCH_MOTOR.r,
              'ld': BENCH_MOTOR.ld, 'lq': BENCH_MOTOR.lq,
              'lam': BENCH_MOTOR.lam, 'poles': BENCH_MOTOR.poles,
              'j': BENCH_MOTOR.j, 'b': BENCH_MOTOR.b,
              'sat': BENCH_MOTOR.sat, 'i_sat': BENCH_MOTOR.i_sat}
    row = mc.run_job({'vdc': 24.8, 'knobs': mc.candidates(4, 2)[1], 'seed': 5,
                      'motor': little, 'i_max': 5.0, 'i_trip': 8.0,
                      'i_h_max': 0.8, 'k_prop': 2e-7})
    r.check('a job carries another motor and the run sizes itself to it',
            not row['trip'] and row['i_peak'] < 5.5,
            (row['trip'], row['i_peak']))


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


def test_the_voltage_limit(r, lib):
    """Asked more torque than its link has volts for, a rotor under a propeller's drag: the
    d current stays its reference, the vector is the limit's, and the rotor turns as fast as
    those volts carry its load's current with no d current beside it."""
    d = Drive(lib)
    # A motor whose d current costs it speed: Ld i over lambda a tenth at 7 A.
    m = Motor(r=0.03, ld=33e-6, lq=41e-6, lam=0.00207, p=14, j=1e-4, b=2e-6, k_load=1.7e-5)
    vdc, frac, asked = 12.0, 0.95, 60.0
    try:
        d.params(r=m.r, ld=m.ld0, lq=m.lq, pole_pairs=m.p, w_lo=300.0, w_hi=600.0,
                 v_frac=frac, i_max=90.0, i_trip=200.0, **{'lambda': m.lam},
                 **loop_gains(m.r, m.ld0, 1000.0), **pll_gains(50.0, TS))
        d.setpoints(id_ref=15.0, omega_target=1200.0, accel=4000.0)
        d.mode(HOLD)
        run(d, m, 0.5, vdc=vdc)
        d.setpoints(id_ref=0.0, iq_ref=asked)
        d.mode(SENSORLESS)
        run(d, m, 2.0, vdc=vdc)
        s, vmax = d.state(), frac * vdc / math.sqrt(3.0)
        kt = 1.5 * m.p * m.lam

        def volts(w):
            iq = (m.k_load * (w / m.p) ** 2 + m.b * w / m.p) / kt
            return math.hypot(w * m.lq * iq, m.r * iq + w * m.lam)
        low, high = 0.0, 1e5
        while high - low > 0.1:
            low, high = ((low + high) / 2.0, high) if volts((low + high) / 2.0) <= vmax else (
                low, (low + high) / 2.0)
        r.check('asked %.0f A it has not the volts for, the vector is the limit\'s' % asked,
                abs(math.hypot(s['vd'], s['vq']) - vmax) < 0.01 * vmax and m.iq < 0.9 * asked,
                '%.2f V of %.2f, %.2f A' % (math.hypot(s['vd'], s['vq']), vmax, m.iq))
        r.check('the d current stays its reference there',
                abs(m.id) < 0.05 * abs(m.iq), '%+.3f A beside %.2f A' % (m.id, m.iq))
        r.check('the rotor as fast as those volts carry its load with no d current',
                abs(m.omega - low) < 0.01 * low, '%.0f of %.0f rad/s' % (m.omega, low))
        space = pull.room(s['omega_hat'] / m.p, vdc, pull.Rotor(m.r, m.lq, m.lam, m.p, 0.0, 0.0,
                                                                0.0))
        r.check('the room it reckons there is machine.pull\'s, the current it carries within '
                '5 %', abs(d.room() - space) < 1e-3 * space
                and abs(d.room() - m.iq) < 0.05 * m.iq,
                '%.2f A, %.2f by machine.pull, %.2f carried' % (d.room(), space, m.iq))
    finally:
        d.close()


def test_thermal_observer_and_ceiling(r, lib):
    """Real-time Cauer junction observer, positive-tempco R_DS(on), T_NTC anchor, and I_max(dt)."""
    d, m = Drive(lib), Motor(locked=True, theta=0.0)
    try:
        d.thermal_params(r_ds0=0.015, alpha_r=0.005, t_ref=25.0, e_sw=1e-7, r_th0=1.5, r_th1=2.0,
                         r_th2=4.0, c_th0=0.002, c_th1=0.010, c_th2=0.050, tj_nom=90.0,
                         tj_max=150.0, horizon_s=0.005)
        d.thermal_init(30.0)
        d.params(i_max=40.0, i_trip=100.0, **loop_gains(m.r, m.ld0, 1000.0))
        d.setpoints(id_ref=30.0, iq_ref=0.0, theta=0.0)
        d.mode(HOLD)
        th0 = d.thermal_state()
        r0 = 0.015 * (1.0 + 0.005 * (30.0 - 25.0))
        r.check('thermal init anchors Cauer nodes to T_NTC and computes R_DS(on)(T_0)',
                abs(th0['tj_hat'] - 30.0) < 1e-4 and abs(th0['r_dson'] - r0) < 1e-6, th0)
        trip = run(d, m, 0.08, vdc=24.0)
        th1, s1 = d.thermal_state(), d.state()
        r.check('heavy load heats junction to T_{j,limit} without binary I2T_FAULT trip',
                not trip and s1['fault'] == 0 and s1['mode'] == HOLD
                and 75.0 < th1['tj_hat'] <= th1['tj_limit'] + 0.5, (th1['tj_hat'], s1['fault']))
        r.check('positive tempco raises R_DS(on)(\\hat{T}_j) with junction temperature',
                th1['r_dson'] > 1.2 * th0['r_dson'], (th0['r_dson'], th1['r_dson']))
        r.check('inverse ceiling I_max(Delta_t) throttles current smoothly to hold T_{j,limit}',
                0.0 < th1['i_max_dyn'] < 30.0 and abs(s1['id'] - th1['i_max_dyn']) < 1.5,
                (th1['i_max_dyn'], s1['id']))
        d.set_pcb_temp(65.0)
        run(d, m, 0.05, vdc=24.0)
        th2 = d.thermal_state()
        r.check('anchoring slowest node to higher T_NTC lowers available I_max(Delta_t)',
                th2['t_pcb'] == 65.0 and th2['i_max_dyn'] < th1['i_max_dyn'], th2['i_max_dyn'])
    finally:
        d.close()


def test_wep_pwm_and_overmodulation(r, lib):
    """Continuous WEP override gamma_WEP in [0, 1]: T_{j,limit}, dynamic f_sw, and hex clamp."""
    d = Drive(lib)
    try:
        d.thermal_params(tj_nom=125.0, tj_max=175.0, f_sw_nom=50000.0, f_sw_min=15000.0,
                         omega_sw_knee=100.0)
        d.thermal_init(25.0)
        d.params(i_max=20.0, i_trip=80.0, v_frac=1.0)
        lims = []
        for g in (0.0, 0.5, 1.0):
            d.set_wep(g)
            lims.append(d.thermal_state()['tj_limit'])
        r.check('T_{j,limit} interpolates linearly from T_{j,nom} to T_{j,max} with gamma_WEP',
                lims == [125.0, 150.0, 175.0], lims)
        d.setpoints(id_ref=20.0, omega_target=0.0)
        d.mode(HOLD)
        d.step((20.0, -10.0, -10.0), 24.0)
        th_s = d.thermal_state()
        r.check('at stall (omega_e=0) and high current with WEP=1, f_sw scales to f_{sw,min}',
                abs(th_s['f_sw'] - 15000.0) < 1.0 and abs(th_s['ts_active'] - 1 / 15000.0) < 1e-7,
                th_s['f_sw'])
        d.setpoints(id_ref=20.0, omega_target=900.0, accel=1e7)
        d.step((20.0, -10.0, -10.0), 24.0)
        d.step((20.0, -10.0, -10.0), 24.0)
        r.check('at high electrical speed |omega_e|, f_sw returns toward f_{sw,nom}',
                d.thermal_state()['f_sw'] > 40000.0, d.thermal_state()['f_sw'])
        sc, ab = d.hex_clamp(16.0, 0.0, 24.0, 1.0)
        r.check('hexagon corner (2/3 Vdc) fits inside SVM hexagon above inscribed circle',
                abs(sc - 1.0) < 1e-5 and abs(ab[0] - 16.0) < 1e-4 and ab[0] > 24.0 / math.sqrt(3),
                (sc, ab))
        d.params(kp=2.0, i_max=20.0, i_trip=80.0, v_frac=1.0)
        d.setpoints(id_ref=20.0, iq_ref=0.0, theta=0.0, omega_target=0.0)
        d.mode(HOLD)
        d.set_wep(0.0)
        d.step((0.0, 0.0, 0.0), 24.0)
        v0 = math.hypot(d.state()['vd'], d.state()['vq'])
        d.set_wep(1.0)
        d.step((0.0, 0.0, 0.0), 24.0)
        v1 = math.hypot(d.state()['vd'], d.state()['vq'])
        r.check('WEP overmodulation expands voltage vector from Vdc/sqrt3 to 2/3 Vdc hex corner',
                abs(v0 - 24.0 / math.sqrt(3.0)) < 0.05 and abs(v1 - 16.0) < 0.05, (v0, v1))
    finally:
        d.close()


ROSTER = (test_math, test_mode_refusals, test_current_loop, test_trip_and_stage,
          test_off_lets_the_rotor_go, test_polarity, test_deadtime, test_moments,
          test_model_agrees, test_montecarlo, test_if_spin, test_the_voltage_limit,
          test_thermal_observer_and_ceiling, test_wep_pwm_and_overmodulation)


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n, on the drive
    core built for this host."""
    return suite(ROSTER, 'drivecore', argv)


if __name__ == '__main__':
    sys.exit(main())
