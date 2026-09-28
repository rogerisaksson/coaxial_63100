#!/usr/bin/env python3
"""The rotor observer, on this host, against a motor that exists only here: injection, the
back-EMF, the handovers between them and the estimate's turns."""
import math
import sys

from motor.pmsm import Motor
from tools.cores.drive import (HOLD, SENSORLESS, TS, TWO_PI, Drive, eps_gain, loop_gains,
                               pll_gains, run, wrap_pi)

from drive_kit import suite


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


def test_observer_standstill_wide(r, lib):
    """SENSORLESS at zero speed on the loop a good injection earns - 400 Hz, the Kalman gains
    2 V gives on native - no current asked, the drive knowing the motor's flux. The back-EMF
    feed-forward off an estimate at rest stepped the voltage each update, the demodulator read
    the current it drove as angle: on native the estimate ran to -8 000 rad/s with 16 A in the
    windings inside 20 ms (2026-09-28)."""
    d = Drive(lib)
    v_inj = 2.0
    try:
        # Native's world: the core's own model, the demo's flywheel, the world's noise.
        d.model_params(theta0=1.0, j=8e-3, b=5e-4, noise=0.02)
        d.source(True)
        d.params(inj_volts=v_inj, inj_periods=1, w_lo=150.0, w_hi=300.0,
                 eps_gain=eps_gain(v_inj, 20e-6, 25e-6), r=0.05, ld=20e-6, lq=25e-6,
                 **{'lambda': 0.005}, **loop_gains(0.05, 20e-6, 0.05 / TS),
                 **pll_gains(400.0, 2 * TS))
        d.setpoints(id_ref=0.0, iq_ref=0.0)
        d.set_theta(1.0)
        d.mode(SENSORLESS, enabled=False, powered=False)
        worst = 0.0
        for _ in range(int(0.2 / TS)):
            d.step_virtual()
            worst = max(worst, abs(d.state()['omega_hat']))
        err = wrap_pi(d.state()['theta_hat'] - d.model_state()['theta'])
        r.check('the speed estimate at rest stays under w_lo, the injection\'s range',
                worst < 150.0, '%.0f rad/s at worst' % worst)
        r.check('and theta_hat stays on the rotor within 0.05 rad', abs(err) < 0.05, err)
    finally:
        d.close()


def test_handover_weighs_the_back_emf(r, lib):
    """At rest with the crossover under the estimate's noise at 400 Hz (w_lo 20, w_hi 40 rad/s): the
    handover weighs the back-EMF the rotor makes, not the speed the estimate says, so the
    injection keeps the estimate on the rotor. Weighed by the estimate, noise past w_hi handed
    over to a back-EMF of nothing, which read back the drive's own feed-forward (2026-09-28)."""
    d = Drive(lib)
    v_inj = 2.0
    try:
        d.model_params(theta0=1.0, j=8e-3, b=5e-4, noise=0.02)
        d.source(True)
        d.params(inj_volts=v_inj, inj_periods=1, w_lo=20.0, w_hi=40.0,
                 eps_gain=eps_gain(v_inj, 20e-6, 25e-6), r=0.05, ld=20e-6, lq=25e-6,
                 **{'lambda': 0.005}, **loop_gains(0.05, 20e-6, 0.05 / TS),
                 **pll_gains(400.0, 2 * TS))
        d.setpoints(id_ref=0.0, iq_ref=0.0)
        d.set_theta(1.0)
        d.mode(SENSORLESS, enabled=False, powered=False)
        worst = 0.0
        for _ in range(int(0.3 / TS)):
            d.step_virtual()
            worst = max(worst, abs(d.state()['omega_hat']))
        err = wrap_pi(d.state()['theta_hat'] - d.model_state()['theta'])
        r.check('noise past w_hi at rest hands nothing over: the estimate keeps to its own noise, '
                'a seventh of the 2 243 rad/s it ran to handed over',
                worst < 300.0, '%.0f rad/s at worst' % worst)
        r.check('and on the rotor within 0.05 rad', abs(err) < 0.05, err)
    finally:
        d.close()


def test_handover_under_d_current(r, lib):
    """The demo's align into up: HOLD at 12 A of d current, then SENSORLESS with it still on.
    The demodulator differences samples stationary: a difference of currents each rotated into
    its own frame read the frame's turn under the 12 A as angle, 4.4 of each correction fed
    back - the estimate ran a pole off and the demo started backwards (2026-09-28)."""
    d = Drive(lib)
    v_inj = 0.585
    try:
        d.model_params(theta0=0.0, j=8e-3, b=5e-4, noise=0.02)
        d.source(True)
        d.params(inj_volts=v_inj, inj_periods=1, w_lo=180.0, w_hi=360.0, i_max=40.0,
                 i_trip=70.0, eps_gain=eps_gain(v_inj, 20e-6, 25e-6), r=0.05, ld=20e-6,
                 lq=25e-6, **{'lambda': 0.005}, **loop_gains(0.05, 20e-6, 0.05 / TS),
                 l1=0.043, l2=23.6)
        d.setpoints(id_ref=12.0, iq_ref=0.0, theta=0.0, omega_target=0.0)
        d.mode(HOLD, enabled=False, powered=False)
        for _ in range(int(0.2 / TS)):
            d.step_virtual()
        d.mode(SENSORLESS, enabled=False, powered=False)
        worst = 0.0
        for _ in range(int(0.1 / TS)):
            d.step_virtual()
            worst = max(worst, abs(wrap_pi(d.state()['theta_hat'] - d.model_state()['theta'])))
        r.check('with 12 A of d current on, the estimate stays on its pole: within 0.3 rad, '
                "the torque 96 % of the current's",
                worst < 0.3, '%.3f rad at worst' % worst)
    finally:
        d.close()


def test_sensorless_run(r, lib):
    """Torque from standstill under injection, through the crossover, onto
    the back-EMF - the rotor observer keeps the rotor the whole way."""
    d = Drive(lib)
    # Friction sets the speed 0.6 A of torque reaches: 0.063 N.m over 5e-4 is
    # 126 rad/s mechanical, 882 electrical - past the crossover and under the
    # voltage limit, so the loop can hold its reference.
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


def test_hold_handover(r, lib):
    """Hold to sensorless at standstill: the estimate starts on the held frame,
    not where it free-ran - the rotor observer page's demo tripped there."""
    d = Drive(lib)
    v_inj = 2.0
    try:
        d.model_params(theta0=1.0, b=5e-4)
        d.source(True)
        d.params(inj_volts=v_inj, inj_periods=1, w_lo=150.0, w_hi=300.0,
                 eps_gain=eps_gain(v_inj, 20e-6, 25e-6),
                 **loop_gains(0.05, 20e-6, 500.0), **pll_gains(60.0, 2 * TS))
        d.set_theta(3.0)
        d.setpoints(id_ref=5.0, iq_ref=0.0, theta=1.0, omega_target=0.0)
        d.mode(HOLD)
        for _ in range(int(0.05 / TS)):
            d.step_virtual()
        d.setpoints(id_ref=0.0)
        d.mode(SENSORLESS)
        r.check('the estimate starts on the frame the rotor was held in',
                abs(wrap_pi(d.state()['theta_hat'] - 1.0)) < 1e-3, d.state()['theta_hat'])
        tripped = any(d.step_virtual()[0] for _ in range(int(0.2 / TS)))
        err = wrap_pi(d.state()['theta_hat'] - d.model_state()['theta'])
        r.check('and stays on the rotor', not tripped and abs(err) < 0.1, (tripped, err))
    finally:
        d.close()


def test_turns_carry_the_estimate(r, lib):
    """theta_hat + 2 pi turns, the estimate unwrapped on the board (op 0, MINOR 24): a held
    frame turned nine times each way moves it a step's worth at a time, never a turn - the rotor
    page counted them at 20 frames a second and its mark leapt 68.8 degrees (2026-09-28)."""
    d = Drive(lib)
    try:
        d.source(True)
        d.params(w_lo=1e4, w_hi=2e4, **loop_gains(0.05, 20e-6, 500.0))
        d.setpoints(id_ref=2.0, iq_ref=0.0, theta=0.0, omega_target=0.0, accel=1e5)
        d.mode(HOLD)
        paths = []
        for target in (200.0, -200.0):
            d.setpoints(omega_target=target)
            for _ in range(int(0.3 / TS)):
                d.step_virtual()
                s = d.state()
                paths.append(s['theta_hat'] + TWO_PI * s['turns'])
        leap = max(abs(b - a) for a, b in zip(paths, paths[1:]))
        r.check('the path runs 60 rad out and back in steps of 200 rad/s, no turn at once',
                leap < 2.0 * 200.0 * TS and max(paths) - paths[0] > 55.0
                and abs(paths[-1] - paths[0]) < 1.0,
                'out %.1f rad, back to %+.2f, the largest step %.4f rad'
                % (max(paths) - paths[0], paths[-1] - paths[0], leap))
    finally:
        d.close()


def test_down_through_if(r, lib):
    """Sensorless at speed into hold: the command frame starts on the estimate and its ramp
    carries the rotor to rest - a jump to the setpoint's angle at speed slipped poles."""
    d = Drive(lib)
    v_inj = 2.0
    try:
        d.model_params(theta0=1.0, b=5e-4, j=8e-3)
        d.source(True)
        d.params(inj_volts=v_inj, inj_periods=1, w_lo=150.0, w_hi=300.0, i_max=30.0, i_trip=60.0,
                 eps_gain=eps_gain(v_inj, 20e-6, 25e-6),
                 **loop_gains(0.05, 20e-6, 500.0), **pll_gains(60.0, 2 * TS))
        d.set_theta(1.0)
        d.setpoints(id_ref=0.0, iq_ref=20.0)
        d.mode(SENSORLESS)
        for _ in range(int(2.0 / TS)):
            d.step_virtual()
            if d.model_state()['omega'] > 1000.0:
                break
        s = d.state()
        # 500 rad/s^2 on 8e-3 kg m^2 asks 0.57 N m of the 1.05 20 A holds.
        d.setpoints(id_ref=20.0, iq_ref=0.0, theta=0.0, omega_target=0.0, accel=500.0)
        d.mode(HOLD)
        c = d.state()
        r.check('the command frame starts on the estimate, turning with it',
                abs(wrap_pi(c['theta_cmd'] - s['theta_hat'])) < 1e-3
                and abs(c['omega_cmd'] - s['omega_hat']) < 1e-3,
                (c['theta_cmd'], s['theta_hat'], c['omega_cmd']))
        tripped, lag = False, 0.0
        for k in range(int(2.5 / TS)):
            tripped = tripped or d.step_virtual()[0]
            if k % 10 == 0:
                lag = max(lag, abs(wrap_pi(d.model_state()['theta'] - d.state()['theta_cmd'])))
        r.check('and its ramp carries the rotor down, never a pole behind',
                not tripped and lag < math.pi / 2.0 and d.state()['omega_cmd'] == 0.0,
                (tripped, lag, d.model_state()['omega']))
    finally:
        d.close()


def test_observer_chain(r, lib):
    """The firmware's observer chain against the Python it was ported from."""
    from coaxial.model.blocks import CurrentLoop, Plant, Signals
    from coaxial.model import sensorless
    from motor.catalog import BENCH_MOTOR

    motor = BENCH_MOTOR
    for w_e in (100.0, 2000.0, 10000.0):
        d = Drive(lib)
        d.params(r=motor.r, ld=motor.ld, lq=motor.lq, **{'lambda': motor.lam})
        d.obs_sync(0.0, w_e)
        dual = sensorless.DualFluxObserver(motor.r, motor.ld, motor.lam,
                                           cross=20.0)
        flux = sensorless.FluxObserver(motor.r, motor.ld, wc=20.0)
        dual.omega = flux.omega = w_e
        loop = CurrentLoop(hz=800.0, motor=motor, vdc=24.0)
        plant = Plant(motor, vdc=24.0, noise=0.0, sub=4, locked=True)
        plant.motor.omega = w_e
        s = Signals()
        s.iq_ref = 2.0
        c_err, py_err = [], []
        steps = int(0.4 / TS)
        for k in range(steps):
            s.t = k * TS
            loop(s, TS)
            plant(s, TS)
            cs, sn = math.cos(s.theta), math.sin(s.theta)
            va, vb = s.vd * cs - s.vq * sn, s.vd * sn + s.vq * cs
            ia, ib = s.id * cs - s.iq * sn, s.id * sn + s.iq * cs
            d.obs_step(va, vb, ia, ib)
            th_d = dual.update(va, vb, ia, ib, TS)
            th_f = flux.update(va, vb, ia, ib, TS)
            g = min(1.0, max(0.0, (abs(dual.omega) - 800.0) / 2200.0))
            x = (1 - g) * math.cos(th_d) + g * math.cos(th_f)
            y = (1 - g) * math.sin(th_d) + g * math.sin(th_f)
            if k > steps // 2:
                c_err.append(sensorless._wrap(d.obs()['theta'] - s.theta))
                py_err.append(sensorless._wrap(math.atan2(y, x) - s.theta))
        rms = lambda v: math.degrees(math.sqrt(sum(e * e for e in v) / len(v)))
        got, want = rms(c_err), rms(py_err)
        r.check('observer chain at %.0f rad/s: the C matches the Python'
                % w_e, abs(got - want) < 0.5,
                '%.2f deg against %.2f' % (got, want))
        # An angle error under the notebook's 20 degree line, where the torque
        # is still there.
        r.check('observer chain at %.0f rad/s holds the rotor' % w_e,
                got < 20.0, '%.2f deg' % got)
        # The flux model's magnitude is lambda, the one thing on this board
        # that can see the magnets.
        seen = d.obs()['lambda_hat'] / motor.lam
        r.check('observer chain at %.0f rad/s recovers lambda' % w_e,
                0.8 < seen < 1.25, '%.3f of the truth' % seen)
        d.close()


def test_observer_needs_a_handover(r, lib):
    """It cannot acquire a speed from nothing, and that is by construction."""
    from coaxial.model.blocks import CurrentLoop, Plant, Signals
    from coaxial.model import sensorless
    from motor.catalog import BENCH_MOTOR

    motor = BENCH_MOTOR
    w_e = 4000.0
    got = {}
    for name, seed in (('cold', 0.0), ('handed over', w_e)):
        d = Drive(lib)
        d.params(r=motor.r, ld=motor.ld, lq=motor.lq, **{'lambda': motor.lam})
        d.obs_sync(0.0, seed)
        loop = CurrentLoop(hz=800.0, motor=motor, vdc=24.0)
        plant = Plant(motor, vdc=24.0, noise=0.0, sub=4, locked=True)
        plant.motor.omega = w_e
        s = Signals()
        s.iq_ref = 2.0
        err = []
        steps = int(0.3 / TS)
        for k in range(steps):
            s.t = k * TS
            loop(s, TS)
            plant(s, TS)
            cs, sn = math.cos(s.theta), math.sin(s.theta)
            d.obs_step(s.vd * cs - s.vq * sn, s.vd * sn + s.vq * cs,
                       s.id * cs - s.iq * sn, s.id * sn + s.iq * cs)
            if k > steps // 2:
                err.append(sensorless._wrap(d.obs()['theta'] - s.theta))
        got[name] = math.degrees(math.sqrt(sum(e * e for e in err) / len(err)))
        d.close()
    r.check('the chain handed an estimate holds the rotor',
            got['handed over'] < 20.0, '%.1f deg' % got['handed over'])
    r.check('and cold from rest it does not, which is why sync exists',
            got['cold'] > got['handed over'] * 3.0,
            'cold %.1f deg against %.1f handed over'
            % (got['cold'], got['handed over']))


ROSTER = (test_injection_map, test_saturation_map, test_observer_standstill,
          test_observer_standstill_wide, test_handover_weighs_the_back_emf,
          test_handover_under_d_current, test_sensorless_run, test_virtual_sensorless,
          test_hold_handover, test_turns_carry_the_estimate, test_down_through_if,
          test_observer_chain, test_observer_needs_a_handover)


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n, on the drive
    core built for this host."""
    return suite(ROSTER, 'drivecore_observer', argv)


if __name__ == '__main__':
    sys.exit(main())
