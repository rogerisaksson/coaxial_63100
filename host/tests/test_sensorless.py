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
import math
import os
import sys

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


ROSTER = (test_arithmetic, test_budget, test_kalman,
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
