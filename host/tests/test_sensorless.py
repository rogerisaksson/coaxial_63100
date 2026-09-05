#!/usr/bin/env python3
"""The sensorless design arithmetic, and the commissioning against the
stand-in. No board, no compiler, no model.

Two halves. `sensorless.py` is pure functions with closed-form answers to
check: the Kalman recursion against its own fixed point, the SNR budget
against the telescoping argument, the crossover against its definition.
`commission.py` is procedures over a rig; run against the stand-in, whose
motor has known constants, every step has a number it must recover.

Run from the host directory:  python tests/test_sensorless.py
"""
import io
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial import Coaxial63100                          # noqa: E402
from coaxial import sensorless                            # noqa: E402
from coaxial.commission import Commissioning, _fit_deadtime, _harmonics  # noqa: E402
from coaxial.drive import from_wire, to_wire              # noqa: E402
from coaxial.errors import RigError                       # noqa: E402
from coaxial.simulated import SimulatedDrive              # noqa: E402


class Report:
    def __init__(self):
        self.passed = self.failed = 0

    def check(self, what, ok, detail=''):
        self.passed += bool(ok)
        self.failed += not ok
        print('  %s  %-58s %s' % ('PASS' if ok else 'FAIL', what,
                                  '' if ok else detail))


def test_inverter(r):
    from coaxial import inverter
    r.check('Coss at zero volts is the model CJO',
            inverter.coss(0.0) == inverter.CJO)
    h = 1e-3
    dq = (inverter.qoss(30.0 + h) - inverter.qoss(30.0 - h)) / (2.0 * h)
    r.check('Qoss is the junction law integrated: dQ/dV is Coss',
            abs(dq / inverter.coss(30.0) - 1.0) < 1e-6, dq)
    ring = inverter.ring(44.4)
    r.check('the ring at the no-load link: 36 MHz off 4 nH against 2 Coss',
            36e6 < ring['f_hz'] < 37e6, ring['f_hz'])
    r.check('the ring settles before the sample across the whole sweep',
            all(inverter.blanking(v) > 100e-9 for v in (23.0, 43.0, 63.0)),
            [inverter.blanking(v) for v in (23.0, 63.0)])
    r.check('v_dt is Vdc t_dead / Ts: 75 mV at the no-load link',
            abs(inverter.dead_time_volts(44.4) - 0.0748) < 5e-4,
            inverter.dead_time_volts(44.4))
    r.check('the knee is 2 Qoss over the dead time',
            abs(inverter.knee_amps(63.0)
                - 2.0 * inverter.qoss(63.0) / inverter.T_DEAD) < 1e-9)
    step, table = inverter.dt_table(44.4)
    r.check('the table: zero first, half a knee a step, saturating to v_dt',
            step == inverter.knee_amps(44.4) / 2.0 and table[0] == 0.0
            and all(b > a for a, b in zip(table, table[1:]))
            and table[-1] < inverter.dead_time_volts(44.4),
            (step, table[-1]))


def test_loop(r):
    from coaxial.loop import (CurrentLoop, Machine, Probe, Ramp, Signals,
                              SpeedLoop, identify)
    from coaxial.motor import BENCH_MOTOR
    s = Signals()
    try:
        s.wref = 1.0
        ok = False
    except AttributeError:
        ok = True
    r.check('a typo on the bus fails instead of vanishing', ok)
    from coaxial.motor import Propeller
    prop = Propeller(k=3.2e-6)      # q excitation: without a load iq spans
    chain = (Ramp(250.0, 0.4) >> Probe(1.0, 300.0)      # 0.4 A and the fit
             >> SpeedLoop(8.0, 60.0, BENCH_MOTOR, load=prop)  # returns a
             >> CurrentLoop(2000.0, BENCH_MOTOR, 24.0)  # CONFIDENT Lq 50 %
             >> Machine(BENCH_MOTOR, 24.0, load=prop, noise=0.02))  # off
    run = chain.run(0.8, 5e-5, every=5)
    at_top = run['w'][abs(run['t'] - 0.4) < 0.01]
    r.check('the speed loop tracks the raised cosine to its top',
            abs(at_top.mean() / 250.0 - 1.0) < 0.05, at_top.mean())
    r.check('and brings the machine back to rest',
            abs(run['w'][-1]) < 10.0, run['w'][-1])
    r.check('an easy ramp never hits the voltage ceiling',
            not run['v_sat'].any())
    fit, got = identify(run, BENCH_MOTOR.poles)
    r.check('the run identifies the machine it closed around',
            abs(got['r'] / BENCH_MOTOR.r - 1.0) < 0.10
            and abs(got['lam'] / BENCH_MOTOR.lam - 1.0) < 0.02
            and abs(got['ld'] / BENCH_MOTOR.ld - 1.0) < 0.15,
            {k: got[k] for k in ('r', 'ld', 'lam')})
    r.check('Lq, riding only on the ramp, still lands inside a quarter',
            abs(got['lq'] / BENCH_MOTOR.lq - 1.0) < 0.25, got['lq'])
    r.check('with the probe, Ld comes out trusted',
            got['trusted']['ld'], got['uncertainty']['ld'])


def test_arithmetic(r):
    r.check('ENOB: 16 bits with no noise, 12.6 at 4 codes rms',
            sensorless.enob(0.0) == 16.0
            and abs(sensorless.enob(4.0) - 12.21) < 0.05, sensorless.enob(4.0))
    gain = sensorless.demod_gain(2.0, 20e-6, 20e-6, 30e-6)
    r.check('demod gain is 2 V T |L_delta| / (Ld Lq)',
            abs(gain - 2.0 * 2.0 * 20e-6 * 5e-6 / (20e-6 * 30e-6)) < 1e-9, gain)
    r.check('and negative when Ld is the larger',
            sensorless.demod_gain(2.0, 20e-6, 30e-6, 20e-6) < 0.0)
    n1 = sensorless.error_noise(0.1, 1, 50.0, 50000.0)
    n4 = sensorless.error_noise(0.1, 4, 50.0, 50000.0)
    r.check('error noise falls as 1/sqrt(N): the sums telescope',
            abs(n1 / n4 - 2.0) < 1e-9, (n1, n4))
    db, sigma = sensorless.snr(2.0, 1, 20e-6, 30e-6, 0.1, 50.0, 50000.0, 20e-6)
    r.check('SNR is 20 log10(1 rad / sigma_theta)',
            abs(db - 20.0 * math.log10(1.0 / sigma)) < 1e-9, (db, sigma))
    r.check('no gain, no SNR', sensorless.snr(0.0, 1, 20e-6, 30e-6, 0.1, 50.0,
                                              50000.0, 20e-6)[0] == -100.0)


def test_budget(r):
    quiet = sensorless.choose_injection(20e-6, 30e-6, 0.005, 50000.0, 50.0,
                                        24.0, 1.0)
    capped = sensorless.choose_injection(20e-6, 30e-6, 0.3, 50000.0, 50.0,
                                         24.0, 1.0)
    noisy = sensorless.choose_injection(20e-6, 30e-6, 0.3, 50000.0, 50.0,
                                        24.0, 10.0)
    r.check('a quiet AFE gets fs/2 and a small amplitude at the target SNR',
            quiet['periods'] == 1 and quiet['limited_by'] == 'target'
            and abs(quiet['snr_db'] - 20.0) < 0.01 and quiet['v_inj'] < 0.5,
            quiet)
    r.check('under a current ceiling a noisy AFE still gets fs/2, backed '
            'off to the target',
            capped['periods'] == 1 and capped['v_inj'] < 2.0, capped)
    r.check('under the voltage headroom it gets more periods and a better '
            'SNR than fs/2 gave',
            noisy['periods'] > 1
            and noisy['snr_db'] > sensorless.snr(noisy['v_inj'], 1, 20e-6, 30e-6,
                                                 0.3, 50.0, 50000.0, 20e-6)[0],
            noisy)
    r.check('the HF current stays under its ceiling',
            capped['i_h_peak'] <= 1.0 + 1e-9 and noisy['i_h_peak'] <= 10.0 + 1e-9)
    audible = sensorless.choose_injection(20e-6, 30e-6, 0.3, 50000.0, 50.0,
                                          24.0, 1.0, f_min_hz=20000.0)
    r.check('an audibility floor keeps it at fs/2', audible['periods'] == 1,
            audible)
    r.check('eight times the current loop is the other floor',
            sensorless.choose_injection(20e-6, 30e-6, 0.3, 50000.0, 50.0, 24.0,
                                        1.0, bw_i_hz=4000.0) is None)
    loop = sensorless.current_loop(0.05, 20e-6, 50000.0, 0.05, 24.0)
    r.check('current loop: kp = L w, ki = R w at the bandwidth',
            abs(loop['kp'] - 20e-6 * 2 * math.pi * loop['bw_hz']) < 1e-9
            and abs(loop['ki'] - 0.05 * 2 * math.pi * loop['bw_hz']) < 1e-9, loop)
    r.check('a twentieth of fs when the shunts are quiet',
            loop['bw_hz'] == 2500.0 and loop['limited_by'] == 'sampling', loop)
    loud = sensorless.current_loop(0.05, 20e-6, 50000.0, 2.0, 24.0)
    r.check('and the noise decides when they are not',
            loud['bw_hz'] < 2500.0 and loud['limited_by'] == 'noise', loud)


def test_kalman(r):
    k = sensorless.kalman_gains(0.05, 40e-6, 2000.0)
    r.check('gains are positive and the angle gain under one',
            0.0 < k['l1'] < 1.0 and k['l2'] > 0.0, k)
    quieter = sensorless.kalman_gains(0.005, 40e-6, 2000.0)
    r.check('quieter shunts, faster rotor observer',
            quieter['wn_hz'] > k['wn_hz'], (quieter['wn_hz'], k['wn_hz']))
    r.check('the posterior angle noise is under the measurement noise',
            k['sigma_theta_est'] < 0.05, k['sigma_theta_est'])
    # the fixed point: one more prediction and update leaves the gains alone
    again = sensorless.kalman_gains(0.05, 40e-6, 2000.0, iterations=4000)
    r.check('the Riccati recursion has converged',
            abs(again['l1'] - k['l1']) < 1e-9 and abs(again['l2'] - k['l2']) < 1e-6)
    r.check('damping near critical for a CV model at this noise ratio',
            0.3 < k['zeta'] < 1.5, k['zeta'])


def test_crossover_and_verdicts(r):
    c = sensorless.crossover(0.005, 0.05, 5.0, 0.1, pole_pairs=7.0)
    r.check('the floor is the residual plus the R uncertainty at i_max',
            abs(c['floor_volts'] - (0.1 + 0.1 * 0.05 * 5.0)) < 1e-12, c)
    r.check('crossover is margin times floor over lambda, electrical',
            abs(c['omega_e'] - 3.0 * c['floor_volts'] / 0.005) < 1e-9, c)
    r.check('and in rpm through the pole pairs',
            abs(c['rpm'] - c['omega_e'] / (2 * math.pi) / 7.0 * 60.0) < 1e-9)
    r.check('decision: injection over the threshold, I/f under it',
            sensorless.decide(12.0) == 'injection'
            and sensorless.decide(8.0) == 'if_start')
    white = sensorless.ljung_box([0.01] * 7, 5000)
    tinted = sensorless.ljung_box([0.2, 0.1, 0.05, 0.0, 0.0, 0.0, 0.0], 5000)
    r.check('Ljung-Box: small autocorrelations are white, a 0.2 is not',
            white['white'] and not tinted['white'], (white['q'], tinted['q']))
    line = sensorless.summary(False, 2.0, 8.0, 800.0, 4.0, 50.0)
    r.check('the summary line reads as specified',
            line == 'zero-speed: no (SNR 2 dB), min closed-loop 8 %, iloop '
                    '800 Hz, sigma_theta 4.0 deg at 50 %', line)


def test_record_units(r):
    r.check('a negative sign travels as its two\'s complement',
            to_wire('drv_sign', -1) == 0xFFFFFFFF
            and from_wire('drv_sign', 0xFFFFFFFF) == -1.0)
    r.check('inductance in nanohenry round-trips',
            abs(from_wire('motor_ld_nh', to_wire('motor_ld_nh', 23.4e-6))
                - 23.4e-6) < 1e-12)
    r.check('a phase in milliradians keeps its sign',
            from_wire('drv_inj_phase_mrad', to_wire('drv_inj_phase_mrad', -0.7))
            == -0.7)


def test_fits(r):
    f = lambda i: 0.5 * math.tanh(i / 0.3)
    points = [(i, 0.05 * i + (2.0 / 3.0) * (f(i) + f(i / 2.0)))
              for i in (0.25, 0.5, 1.0, 1.5, 2.0, 3.0, 4.0)]
    rr, v_dt, i_knee, residual = _fit_deadtime(points)
    r.check('the dead-time fit recovers R, V_dt and the knee',
            abs(rr - 0.05) < 0.003 and abs(v_dt - 0.5) < 0.02
            and abs(i_knee - 0.3) < 0.05, (rr, v_dt, i_knee))
    r.check('with a small residual', residual < 0.01, residual)
    angles = [math.pi * k / 8 for k in range(8)]
    h = _harmonics([25e-6 - 5e-6 * math.cos(2 * a) + 1e-6 * math.cos(4 * a)
                    for a in angles], angles)
    r.check('harmonics: the mean, the second and the fourth',
            abs(h['mean'] - 25e-6) < 1e-12 and abs(h['h2'] - 5e-6) < 1e-12
            and abs(h['h4'] - 1e-6) < 1e-12, h)


def test_commissioning_refuses_to_switch(r):
    rig = Coaxial63100(simulated_device=True, power_afe=True).open()
    try:
        c = Commissioning(rig)
        got = c.afe_noise(zero_vector=False)
        r.check('the noise floor needs no stage',
                got['sigma_i'] > 0.0 and 'gates_off' in got, got['sigma_i'])
        r.check('and reports ENOB and the latency it could measure',
                got['enob'] > 0.0 and got['latency']['to_effect_periods'] == 2)
        try:
            c.deadtime()
            refused = None
        except RigError as exc:
            refused = str(exc)
        r.check('a switching step refuses without arm=, and says what to pass',
                refused is not None and 'arm=' in refused, refused)
        r.check('the stage stayed down', rig.gates.armed() is False)
    finally:
        rig.close()


def test_commissioning_recovers_the_stand_in(r):
    """Every step against the stand-in's known constants."""
    rig = Coaxial63100(simulated_device=True, power_afe=True).open()
    S = SimulatedDrive
    try:
        c = Commissioning(rig, arm=dict(bypass_sto=True, ignore_interlock=True))
        afe = c.afe_noise()
        r.check('zero vector on: pickup is reported per phase',
                set(afe['pickup_amps']) == {'Phase U', 'Phase V', 'Phase W'})
        scan = c.sample_point_scan(periods=50)
        r.check('the scan lands near the quiet top of the period',
                scan['best'] > 0.8 * scan['period'], scan['best'])
        off = c.offsets(periods=50)
        r.check('Phase V\'s -8030 codes is flagged, not zeroed',
                off['Phase V']['suspect'] and not off['Phase V']['applied']
                and off['Phase U']['applied'], off)
        g = c.gain_mismatch(periods=50)
        r.check('gain mismatch: 1.5 % between the chains, found',
                g['measured'] and abs(g['mismatch_pct'] - 1.5) < 0.2, g)
        sign = c.sign_check()
        r.check('the shunt sign reads positive', sign['measured'] and sign['sign'] == 1)
        dt = c.deadtime()
        r.check('R within 5 % and V_dt within 5 % of the stand-in\'s',
                dt['measured'] and abs(dt['r'] - S.R) < 0.05 * S.R
                and abs(dt['v_dt'] - S.V_DT) < 0.05 * S.V_DT, (dt['r'], dt['v_dt']))
        lm = c.l_map(points=8)
        r.check('Ld and Lq within 5 %',
                lm['measured'] and abs(lm['ld'] - S.LD) < 0.05 * S.LD
                and abs(lm['lq'] - S.LQ) < 0.05 * S.LQ, (lm['ld'], lm['lq']))
        r.check('dL/L is the saliency, about 0.2',
                abs(lm['dl_over_l'] - 0.2) < 0.03, lm['dl_over_l'])
        r.check('a bias current bends Ld',
                lm['rows'][3.0]['ld'] < 0.85 * lm['rows'][0.0]['ld'],
                (lm['rows'][3.0]['ld'], lm['rows'][0.0]['ld']))
        fl = c.flux()
        r.check('lambda within 10 %',
                fl['measured'] and abs(fl['lambda'] - S.LAMBDA) < 0.1 * S.LAMBDA,
                fl['lambda'])
        b = c.budget()
        r.check('the budget names an injection', b['choice'] is not None, b)
        gains = c.gains()
        r.check('gains written: loop, Kalman, injection, crossover',
                gains['kalman'] is not None
                and rig.board.drive.params()['drv_inj_mv'] > 0.0
                and gains['crossover']['rpm'] > 0.0, gains['written'])
        d = c.decide()
        r.check('the decision follows the SNR',
                d['method'] == ('injection' if d['snr_db'] >= 10.0 else 'if_start'))
        v = c.verify(seconds=0.3)
        r.check('verification reports whiteness and a sigma_theta',
                'white' in v['ljung_box'] and v['sigma_theta_deg'] >= 0.0, v)
        line = c.report()['line']
        r.check('and the report ends in the one line',
                line.startswith('zero-speed: ') and 'iloop' in line, line)
        c._rest()                       # what run() does in its finally
        r.check('the stage is down afterwards', rig.gates.armed() is False)
    finally:
        rig.close()


def test_autodetect_recovers_each_machine(r):
    """`observer.autodetect` against five different simulated outrunners.

    A ROUND TRIP, and the only kind of check that can catch what it
    caught. The profiles under `host/motors/` are shapes, not
    measurements - what makes them useful is that they are DIFFERENT, so
    a step that silently answers the stand-in's own defaults is visible.
    Two did: the stand-in's electrical model and its dead-time voltage
    both read class constants rather than the machine that had been
    written to them, and the identifier recovered 0.051 ohm and 19.5 uH
    from every one of five machines in turn - which reads as a working
    identifier until a second machine goes through it.

    The pole count is the one the board measures itself, against the
    shaft sensor; the rest come off the commissioning steps and land in
    the record. Both are held here.
    """
    import glob
    import json
    import os

    here = os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), 'motors')
    profiles = sorted(glob.glob(os.path.join(here, 'outrunner_*.json')))
    r.check('there is more than one machine to identify', len(profiles) > 1,
            '%d profiles' % len(profiles))
    for path in profiles:
        want = json.load(io.open(path, encoding='utf-8'))
        rig = Coaxial63100(simulated_device=True, power_afe=True).open()
        try:
            rig.board.drive.profile(path)
            rig.board.drive.source('model')
            got = rig.observer.autodetect(
                arm=dict(bypass_sto=True, ignore_interlock=True),
                slots=want.get('slots'))
        finally:
            rig.close()
        name = os.path.basename(path)
        for field, key, tol in (('r', 'motor_r_uohm', 0.05),
                                ('ld', 'motor_ld_nh', 0.05),
                                ('lq', 'motor_lq_nh', 0.05),
                                ('lam', 'motor_lambda_uvs', 0.05)):
            truth = want['drive'][key]
            found = getattr(got, field)
            r.check('%s: %s within %d%%' % (name, field, 100 * tol),
                    abs(found - truth) <= tol * truth,
                    '%.6g against %.6g' % (found, truth))
        r.check('%s: the pole count is the one on the shaft' % name,
                got.poles == want['drive']['motor_pole_pairs'],
                '%d against %d' % (got.poles, want['drive']['motor_pole_pairs']))
        r.check('%s: and it says it was measured' % name, got.measured is True)


def test_motion(r):
    """The three motion verbs close their loops on the stand-in's own
    virtual rotor - the shaft sensor reads what the drive torques."""
    from coaxial import Coaxial63100
    from coaxial.errors import RigError
    rig = Coaxial63100(port='COM99', simulated_device=True,
                       power_afe=False).open()
    try:
        rig.drive.source('model')
        try:
            rig.motion.stepper(amps=2.0)
            r.check('motion refuses an unarmed stage', False)
        except RigError as exc:
            r.check('motion refuses an unarmed stage',
                    'gates.arm' in str(exc), exc)
        rig.gates.arm(bypass_sto=True, ignore_interlock=True)
        with rig.motion.stepper(amps=3.0, deg_s=120.0) as m:
            got = m.to(45.0)
            r.check('the stepper slews the command where it was asked',
                    abs(got - 45.0) < 0.3, got)
        with rig.motion.servo(amps=3.0) as s:
            got = s.to(30.0, tol=0.7)
            r.check('the servo lands the SHAFT inside tolerance',
                    abs(got - 30.0) < 0.7, got)
            got = s.to(-10.0, tol=0.7)
            r.check('and comes back through zero', abs(got + 10.0) < 0.7, got)
        with rig.motion.velocity(amps=4.0, hz=2.0) as v:
            got = v.rpm(900, seconds=2.0)
            r.check('the velocity loop reaches the asked speed',
                    abs(got - 900.0) < 60.0, got)
            got = v.stop(1.2)
            r.check('and brings the rotor back to rest', abs(got) < 60.0, got)
        r.check('the drive is OFF after every block',
                rig.drive.state()['mode'] == 'off')

        # THE DANGEROUS PATHS: what a block does when the plant misbehaves.
        with rig.motion.servo(amps=3.0, settle=0.15) as s:
            rig.drive.model_param(load=0.06)     # a load pulse winds it
            time.sleep(0.3)
            sagged = s._measure()
            rig.drive.model_param(load=0.0)
            got = s.to(0.0, tol=0.8)
            r.check('a load pulse sags the hold and the servo takes it back',
                    sagged < -1.0 and abs(got) <= 0.8, (sagged, got))
            # PAST 3 A OF HOLDING TORQUE BY A MARGIN THAT NO TIMING CAN
            # CLOSE. It was 0.4 N.m against the 0.18 that 3 A makes on this
            # machine - twice over, and still it returned about one run in
            # four, but only inside the full offline gate and never in six
            # runs of this suite alone. The stand-in integrates against the
            # WALL CLOCK, so a loaded machine hands each try a different
            # amount of model time and a wound-up servo can swing through
            # the target. A load this far past the torque cannot be held
            # for any amount of model time, which is what the check is
            # about; the margin is the fix, not a longer timeout.
            rig.drive.model_param(load=1.2)
            try:
                got = s.to(30.0, tol=0.5, tries=2)
                # WITH WHAT IT SAW. This returned about one run in four
                # inside the full gate and never on its own, and a bare
                # False said nothing about why - the next occurrence will
                # carry the angle that passed for a hold.
                r.check('an overpowered servo raises, not returns', False,
                        'returned %.2f deg under 1.2 N.m at %.1f A'
                        % (got, s.amps))
            except RigError as exc:
                r.check('an overpowered servo raises, not returns',
                        'holding torque' in str(exc), exc)
            rig.drive.model_param(load=0.0)
        sim = rig.board.drive
        try:
            with rig.motion.velocity(amps=4.0, hz=2.0) as v:
                def trip(_):
                    sim._fault = 'overcurrent'
                v.rpm(600, seconds=2.0, watch=trip)
            r.check('a trip mid-spin ends the loop with the reason', False)
        except RigError as exc:
            r.check('a trip mid-spin ends the loop with the reason',
                    'overcurrent' in str(exc), exc)
        sim._fault = None
        r.check('and the drive is OFF after the aborted block',
                rig.drive.state()['mode'] == 'off')

        # The aborted block leaves a coasting flywheel (tau = j/b is
        # seconds); the next check wants a known rotor, as a bench block
        # would brake first.
        rig.drive.model_reset()

        # ONE ROTOR, TWO THREADS. A reader hammering the shaft sensor -
        # the DAQ path's own route into the rotor - while a motion loop
        # runs it: before the lock, two interleaved advances double-
        # integrated and the shaft read in megaradians.
        import threading
        stop, seen = [False], []

        def reader():
            while not stop[0]:
                seen.append(rig.angle.state()['degrees'])
                time.sleep(0.001)

        t = threading.Thread(target=reader)
        t.start()
        try:
            with rig.motion.velocity(amps=4.0, hz=2.0) as v:
                got = v.rpm(600, seconds=1.5)
        finally:
            stop[0] = True
            t.join()
        r.check('the loop holds under a concurrent reader',
                abs(got - 600.0) < 60.0, got)
        r.check('and every concurrent shaft read stays a real angle',
                seen and all(-1.0 <= d <= 361.0 for d in seen),
                (len(seen), max(seen, default=0)))
    finally:
        rig.close()


def pick_and_place():
    """Every placement in `electronics/`, by designator, as the exporter's
    (x, y) in millimetres - `(None, path)` when the file is not in the
    tree. THE PICK AND PLACE IS THE AUTHORITY ON WHERE THINGS ARE, and two
    tests below hold the host's copies to it."""
    import csv
    import os

    here = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))))
    path = os.path.join(here, 'electronics', 'Coaxial 63100 Pick-Place.csv')
    if not os.path.exists(path):
        return None, path
    at = {}
    with open(path, encoding='latin-1') as handle:
        for line in handle:
            if not line.startswith('"'):
                continue
            row = next(csv.reader([line]))
            try:
                at[row[0]] = (float(row[4]), float(row[5]))
            except ValueError:
                continue
    return at, path


def test_the_map_places_its_parts_from_the_file(r):
    """The thermal picture's parts sit where the pick and place puts them.

    `thermalmap.PLACED` is a copy of the file's coordinates for the parts
    the model heats and the picture marks, and `PNP_CENTRE` the board's
    centre in the exporter's frame. A copy drifts; this holds it to the
    file to a hundredth of a millimetre, the centre to the placements'
    extents, and the switch pairs symmetric about it - which is the check
    that the centre is the board's and not merely the parts' box's. And
    every label lands on the board, clear of the rim and the bore.
    """
    from coaxial import thermalmap

    at, path = pick_and_place()
    if at is None:
        r.check('the pick and place is in the tree', False, path)
        return

    off = [(ref, x, y) for ref, (x, y, _w, _h) in thermalmap.PLACED.items()
           if ref not in at or abs(at[ref][0] - x) > 0.01
           or abs(at[ref][1] - y) > 0.01]
    r.check('every part the map places is where the file puts it, '
            'to a hundredth of a millimetre', not off, str(off[:3]))

    xs = [p[0] for p in at.values()]
    ys = [p[1] for p in at.values()]
    mid = ((min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0)
    r.check('the centre is the placements\' midpoint to half a millimetre',
            abs(mid[0] - thermalmap.PNP_CENTRE[0]) < 0.5
            and abs(mid[1] - thermalmap.PNP_CENTRE[1]) < 0.5,
            '(%.2f, %.2f) against %s' % (mid + (thermalmap.PNP_CENTRE,)))
    u = (thermalmap.placed('Q1U')[0] + thermalmap.placed('Q2U')[0]) / 2.0
    w = (thermalmap.placed('Q1W')[0] + thermalmap.placed('Q2W')[0]) / 2.0
    r.check('and the U and W switch pairs sit either side of it, to half '
            'a millimetre', abs(u + w) < 1.0 and u < -20.0 < 20.0 < w,
            'U at %.2f, W at %.2f' % (u, w))

    # EACH PHASE'S FRAME TAKES ITS SHUNTS - the bench's word - and they
    # are the phase's own: the U shunts left of the V shunts left of the
    # W shunts, each pair within its leg's frame.
    frames = {label: thermalmap.frame(refs, margin)
              for label, refs, _where, margin in thermalmap.MARKS}
    shunts = {leg: [thermalmap.placed('R%s%d' % (leg, i)) for i in (1, 2)]
              for leg in 'UVW'}
    inside = all(abs(x - frames[leg][0]) <= frames[leg][2]
                 and abs(y - frames[leg][1]) <= frames[leg][3]
                 for leg in 'UVW' for x, y in shunts[leg])
    r.check('each phase\'s two shunts sit inside that phase\'s frame',
            inside and max(x for x, _ in shunts['U'])
            < min(x for x, _ in shunts['V'])
            and max(x for x, _ in shunts['V'])
            < min(x for x, _ in shunts['W']),
            str({leg: [round(x, 1) for x, _ in shunts[leg]]
                 for leg in 'UVW'}))

    for label, refs, where, margin in thermalmap.MARKS:
        lx, ly = thermalmap.label_at(thermalmap.frame(refs, margin), where,
                                     label)
        reach = math.hypot(lx, ly)
        r.check('%s is labelled on the board, clear of the rim and the bore'
                % label,
                thermalmap.BORE_MM + 2.0 < reach < thermalmap.OUTER_MM - 3.0,
                '(%.1f, %.1f), %.1f mm out' % (lx, ly, reach))


def test_the_placements_behind_the_thermal_model(r):
    """What `electronics/` places, and what the model claims about it.

    THE PICK AND PLACE IS THE AUTHORITY ON WHERE THINGS ARE, the way the
    parts list is the authority on what is fitted. Two of the thermal
    model's numbers rest on it now - which leg the thermistor anchors,
    and how much of that leg's rise it sees - so both are checked against
    the file rather than against a comment.

    ONLY DIFFERENCES ARE USED. The exporter's origin is offset, and every
    quantity here is either a distance between two parts or the extent of
    the whole set, so a constant shift falls out of both.
    """
    from coaxial import thermal

    at, path = pick_and_place()
    if at is None:
        r.check('the pick and place is in the tree', False, path)
        return

    def away(ref):
        return math.hypot(at[ref][0] - at['NTC1'][0],
                          at[ref][1] - at['NTC1'][1])

    drivers = ('U1U', 'U1V', 'U1W')
    r.check('every part the thermal model names is placed',
            all(ref in at for ref in drivers + ('NTC1', 'Q1V', 'Q2V')),
            '%d placements' % len(at))

    near = min(drivers, key=away)
    second = sorted(away(ref) for ref in drivers)[1]
    r.check('the thermistor anchors the leg it is actually next to - '
            'THERMAL_NTC_NEIGHBOUR is driver_v',
            near == 'U1V' and second > 3.0 * away(near),
            '%s at %.1f mm, next at %.1f' % (near, away(near), second))

    # THE FRACTION, from two-dimensional radial spreading in a plate:
    # `f = ln(R/r) / ln(R/a)`. R is half the short side of the placement
    # extent - the parts' box, not the board outline, so it is a floor -
    # and `a` is a package's own radius.
    xs = [p[0] for p in at.values()]
    ys = [p[1] for p in at.values()]
    reach = min(max(xs) - min(xs), max(ys) - min(ys)) / 2.0
    source = 1.5

    def share(ref):
        return math.log(reach / away(ref)) / math.log(reach / source)

    # At 100 A the two FETs make 18.4 W of the leg node's 18.6, so the
    # fraction is theirs and not the driver IC's - which is the whole
    # correction the placements bought.
    weighted = (0.2 * share('U1V') + 9.2 * share('Q2V')
                + 9.2 * share('Q1V')) / 18.6
    r.check('the model fraction is what the placements imply under load',
            abs(thermal.NTC_SEES_DRIVERS - weighted) < 0.05,
            'model %.2f against %.3f from geometry'
            % (thermal.NTC_SEES_DRIVERS, weighted))
    r.check('and it is well below what the driver IC alone would give, '
            'which is what the model used to carry',
            weighted < 0.8 * share('U1V'),
            '%.3f against the IC own %.3f' % (weighted, share('U1V')))


def test_the_board_stays_in_the_laminar_regime(r):
    """The convection exponent is the regime, not a choice.

    `board_to_ambient_at` scales convection as the fourth root of the
    rise, which is `Nu = C Ra^n` with n = 1/4 - true while the flow is
    laminar and 1/3 once it is not. Whether this board is ever anywhere
    near that boundary is a computable question and this computes it,
    from the placements' own extent, so a bigger board or a hotter rise
    fails here rather than quietly using the wrong power.
    """
    import math

    from coaxial import thermal

    # The board's extent, off the same file the element fraction uses.
    side = 0.093
    plate = (0.092 * side) / (2 * (0.092 + side))     # A/P, horizontal
    worst = 0.0
    for rise in (10.0, 40.0, 85.0):
        film = 293.15 + rise / 2.0
        nu = 1.46e-5 * (film / 293.15) ** 1.75
        def ra(length):
            return (9.81 * (1.0 / film) * rise * length ** 3
                    / (nu * nu) * 0.71)
        # Horizontal turns turbulent at 1e7, vertical at 1e9: the margin
        # is how far the worse of the two is from its own boundary.
        worst = max(worst, ra(plate) / 1e7, ra(side) / 1e9)

    r.check('the board is laminar at every rise the ceilings allow, so '
            'the quarter power is the regime',
            worst < 0.1, 'closest approach %.1f %% of a boundary'
            % (100.0 * worst))
    r.check('and the model uses that exponent',
            abs(thermal.CONVECTION_EXPONENT - 0.25) < 1e-9,
            '%.3f' % thermal.CONVECTION_EXPONENT)


def test_the_datasheet_against_the_thermal_model(r):
    """What `datasheets/mosfet/` settles, and where it disagrees.

    THREE NUMBERS THAT WERE SAID TO NEED A BENCH DAY and were in the tree
    all along - the sheet is `IAUCN10S7N021-Datasheet.pdf` Rev 1.2, and
    the arithmetic here is what it does to the model.
    """
    from coaxial import inverter, thermal

    # THE DIE, which the network has no node for. At 100 A each FET
    # carries its half of the period, so about 9 W, and Rth JC puts the
    # junction that far above its own case.
    watt = 100.0 ** 2 * inverter.RDS_ON * 0.5
    over = watt * inverter.RTH_JC
    r.check('one FET at 100 A puts its junction a few K over its case, '
            'not tens',
            5.0 < over < 8.0, '%.1f W, %.1f K over' % (watt, over))
    r.check('so a 125 C ceiling on the copper is well under the 175 C the '
            'sheet allows - the ceiling is conservative, not optimistic',
            125.0 + over < inverter.T_J_MAX - 30.0,
            '%.0f C junction against %.0f C' % (125.0 + over,
                                                inverter.T_J_MAX))

    # THE SPREADING RESISTANCE, and here the sheet and the model fight.
    # One FET's whole path to air on a JEDEC 2s2p board is 25.9 K/W. The
    # star's spreading term alone was 45.6, then 28, on a board with
    # heavier copper than 2s2p; the graph's leg is its edge into its
    # patch, the patch's neighbours in parallel and the bulk to the air -
    # about 35, still longer than the sheet's whole path, by a third
    # where the star was double. FINDINGS has what that means.
    patch = 0.0
    for (a, b, _r), r_edge in zip(thermal.EDGES, thermal.CFG['edges']):
        if r_edge > 0.0 and 'patch_v' in (a, b) \
                and {a, b} <= set(thermal.LAMINATE):
            patch += 1.0 / r_edge
    leg = (thermal.CFG['to_board']['driver_v'] + 1.0 / patch
           + thermal.CFG['board_to_ambient'])
    r.check('the model\'s whole leg path to the air still exceeds the '
            'datasheet\'s junction-to-air on a coupon, by less than the '
            'star did',
            inverter.RTH_JA_JEDEC < leg < 2.0 * inverter.RTH_JA_JEDEC,
            '%.1f K/W against %.1f' % (leg, inverter.RTH_JA_JEDEC))

    # AND THE CONDUCTION IS BOOKED ON THE TYPICAL. The sheet's maximum is
    # within spec for a part that ships, and an envelope built on typ
    # under-books it.
    r.check('Rds(on) is the typical, so the envelope under-books a '
            'worst-case part by about a sixth',
            abs(2.1e-3 / inverter.RDS_ON - 1.167) < 0.01,
            '%.1f mOhm typ against 2.1 max' % (inverter.RDS_ON * 1e3))


def test_the_stand_in_thermistor_stays_between_its_nodes(r):
    """The stand-in's own copy of the thermistor lag carries the chain's bound.

    `SimulatedThermal` integrates `coaxial.thermal`'s network itself and
    lags its NTC at `NTC_TAU_S` toward the algebra - the same shape as
    `thermal.c`, and it had the same defect: measured on the core, 25 A
    for two minutes then off read the thermistor 6 K over the leg it
    sits beside, 29 K at 60 A. The leg sheds only into the board, through
    the copper the thermistor is on, so the reading cannot leave the pair
    (FINDINGS, the thermistor). Held here on the stand-in, in model time,
    so the page's demo cannot show an NTC warmer than its switches.
    """
    from coaxial import thermal
    from coaxial.simulated.power import SimulatedThermal

    model = SimulatedThermal()
    worst, lagged = -1e9, False
    for on in (True, False):
        seen = {'amps': (0.0, 25.0, 0.0) if on else (0.0, 0.0, 0.0),
                'switching': on}
        for _ in range(int(120.0 / 0.05)):
            model._integrate(0.05, seen)
            leg = model._node[thermal.NTC_NEIGHBOUR]
            board, ntc = model._node['board'], model._ntc
            worst = max(worst, ntc - max(leg, board), min(leg, board) - ntc)
            if on:
                lagged = (lagged or
                          ntc < thermal.expected_ntc(board, leg - board) - 1.0)
    r.check('25 A for two minutes then off: the stand-in\'s reading never '
            'leaves the pair it sits between', worst <= 1e-6,
            '%+.3f K outside' % worst)
    r.check('and it still lags on the way up', lagged)


def test_the_stand_in_throttles_on_the_winding_too(r):
    """The stand-in's stage backs off on the motor's SOA as well as the
    switches', the way `board_thermal.c` does since MINOR 12.

    The board's ten nodes are lifted out of the way - ceilings the
    copper cannot reach - so what is left is the winding: 60 A rms into
    the record's 50 mOhm is 540 W into 180 J/K, three kelvin a second,
    and the ceiling is 95 K up. The budget must carry the winding, its
    OWN factor must be the one the stage gets once the board's is still
    open, and the ceiling must trip the stage - through the same gate
    the nodes use.
    """
    from coaxial.simulated.power import SimulatedThermal
    from coaxial.thermal_device import THROTTLE_AT

    model = SimulatedThermal()
    # The board's ceilings lifted out of the way; the winding keeps its
    # own - it is a node of the same graph since the graph, and its
    # ceiling is the record's 120.
    model.LIMIT, model.DEFAULT_LIMIT = {'winding': 120.0}, 1e4
    got, gate = [], []
    model._derate_to = got.append
    model._gate = lambda: gate.append(True) or True
    # One instant of a balanced 60 A rms three-phase current: the peak
    # on one leg and half of it back on the other two.
    peak = 60.0 * math.sqrt(2.0)
    seen = {'amps': (peak, -peak / 2.0, -peak / 2.0), 'switching': True}
    cold = model.budget()
    r.check('the budget carries the winding: at rest it is at ambient with '
            'nothing spent and its clamp open',
            abs(cold['winding_c'] - 25.0) < 1e-6 and cold['winding_used'] == 0.0
            and cold['winding_derate'] == 1.0, str(cold))
    throttled_at, tripped_at, stage_got = None, None, None
    for i in range(int(120.0 / 0.1)):
        model._integrate(0.1, seen)
        model._envelope()
        b = model.budget()
        if throttled_at is None and b['winding_derate'] < 1.0:
            # The factor the stage held AT THAT MOMENT: by the end of the
            # loop the winding is at its ceiling and the clamp is shut.
            throttled_at, stage_got = b, got[-1]
        if tripped_at is None and gate:
            tripped_at = b
            break
    from coaxial import thermal as thermal_mirror
    board_only = (max(v for n, v in throttled_at['used'].items()
                      if n not in thermal_mirror.MOTOR)
                  if throttled_at else None)
    r.check('60 A warms the winding and the board\'s nodes stay clear',
            throttled_at is not None and board_only < THROTTLE_AT,
            str(throttled_at and (throttled_at['winding_c'], board_only)))
    r.check('the winding throttles first, and what the stage got is the '
            'winding\'s own factor - the smaller of the two',
            throttled_at is not None and stage_got is not None
            and abs(stage_got - throttled_at['winding_derate']) < 1e-6
            and throttled_at['throttling'],
            '%s vs %s' % (stage_got,
                          throttled_at and throttled_at['winding_derate']))
    r.check('and at its ceiling the stage is dropped through the same gate',
            tripped_at is not None and tripped_at['tripped']
            and tripped_at['winding_used'] >= 1.0,
            str(tripped_at and tripped_at['winding_c']))


ROSTER = (test_inverter, test_the_placements_behind_the_thermal_model,
          test_the_map_places_its_parts_from_the_file,
          test_the_board_stays_in_the_laminar_regime,
          test_the_datasheet_against_the_thermal_model,
          test_the_stand_in_thermistor_stays_between_its_nodes,
          test_the_stand_in_throttles_on_the_winding_too,
          test_loop, test_motion,
          test_autodetect_recovers_each_machine,
          test_arithmetic, test_budget, test_kalman,
          test_crossover_and_verdicts, test_record_units, test_fits,
          test_commissioning_refuses_to_switch,
          test_commissioning_recovers_the_stand_in)


def main():
    report = Report()
    for test in ROSTER:
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
