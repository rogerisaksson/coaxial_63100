"""Commissioning a motor on this board: the eight steps, against a rig.

    1. AFE   - noise floor per channel, gates off and on the zero vector;
               the sample point with the least variance; offsets; gain
               mismatch from ia + ib + ic = 0; sigma_i, ENOB, latency.
    2. Inverter - the dead-time voltage error against current, and R.
    3. Motor - the L map (dL/L against bias current and injection angle,
               with its harmonics) and lambda from an I/f spin.
    4. Budget - f_inj and amplitude for the best SNR under the constraints.
    5. Gains - current loop from R, L, sigma_i; Kalman gains from sigma_i;
               the crossover speed.
    6. Decision - injection, or an I/f start with a saturation pulse.
    7. Verification - a sensorless run, the innovation's whiteness, a
               sigma_theta proxy, and the one-line report.

Every number is the board's measurement; every verdict is this executive's
(invariant 10 keeps them out of the firmware). Nothing here arms the stage
unless `arm` says how; a step that switches refuses without it. On the
unmodified bench board AFE_ON high unpowers the gate drivers, so the
switching steps run dry there - `gate_supply()` says so, measured.
"""
import math
import time

from . import sensorless
from .errors import RigError

from .sensorless import TWO_PI

PHASES = ('Phase U', 'Phase V', 'Phase W')

#: Volts on the gate driver supply below which the 2EDL8034 is in UVLO
#: (7.3 V rising): under it nothing switches whatever TIM1 does.
GATE_UVLO_V = 7.3


def _fit_deadtime(points):
    """(r, v_dt, i_knee, residual) from (amps, volts) pairs.

    Model: vd = R I + (2/3)(f(I) + f(I/2)), f = v_dt tanh(I / i_knee) - a
    vector on phase a puts I on a and -I/2 on b and c, so the three
    per-phase errors land on d as two thirds of their sum. R is linear
    least squares at each (v_dt, i_knee) of a grid, refined once.
    """
    def cost(v_dt, i_knee):
        f = lambda i: v_dt * math.tanh(i / i_knee)
        xs = [(i, v - (2.0 / 3.0) * (f(i) + f(i / 2.0))) for i, v in points]
        sxx = sum(i * i for i, _ in xs)
        r = sum(i * y for i, y in xs) / sxx if sxx > 0.0 else 0.0
        return sum((y - r * i) ** 2 for i, y in xs), r

    i_top = max(i for i, _ in points)
    v_top = max(abs(v) for _, v in points)
    best = None
    for pass_ in range(2):
        span_v = (0.0, 2.0 * v_top + 0.1) if pass_ == 0 else (
            max(0.0, best[1] * 0.7), best[1] * 1.3 + 1e-3)
        span_k = (0.02, i_top) if pass_ == 0 else (
            max(0.01, best[2] * 0.7), best[2] * 1.3)
        for a in range(41):
            v_dt = span_v[0] + (span_v[1] - span_v[0]) * a / 40.0
            for b in range(41):
                i_knee = span_k[0] + (span_k[1] - span_k[0]) * b / 40.0
                err, r = cost(v_dt, i_knee)
                if best is None or err < best[0]:
                    best = (err, v_dt, i_knee, r)
    err, v_dt, i_knee, r = best
    return r, v_dt, i_knee, math.sqrt(err / len(points))


def _harmonics(values, angles):
    """Mean, and the amplitude and phase of cos(2 phi) and cos(4 phi) over
    an evenly spread set of injection angles."""
    n = len(values)
    mean = sum(values) / n
    out = {'mean': mean}
    for order in (2, 4):
        re = sum((v - mean) * math.cos(order * a) for v, a in zip(values, angles))
        im = sum((v - mean) * math.sin(order * a) for v, a in zip(values, angles))
        out['h%d' % order] = 2.0 * math.hypot(re, im) / n
        out['h%d_phase' % order] = math.atan2(im, re)
    return out


class Commissioning:

    """The steps, on one rig, and what each of them learnt.

    `arm` is the dict `gates.arm()` is called with when a step needs the
    stage - `dict(bypass_sto=True, ignore_interlock=True)` on this bench -
    or None, in which case those steps refuse and say so.
    """

    def __init__(self, rig, arm=None, log=None, i_h_max=1.0, f_min_hz=0.0,
                 bw_est_hz=50.0, accel_sd=2000.0, rated_rpm=3000.0):
        self.rig = rig
        self.arm = arm
        self.log = log or (lambda line: None)
        self.i_h_max = i_h_max
        self.f_min_hz = f_min_hz
        self.bw_est_hz = bw_est_hz
        self.accel_sd = accel_sd
        self.rated_rpm = rated_rpm
        self.results = {}
        self.fs = 1.0 / rig.board.drive.state()['ts']

    # -- helpers ----------------------------------------------------------

    @property
    def drive(self):
        return self.rig.board.drive

    def amps_per_code(self):
        """The phase scaling out of the board's own record."""
        shunt = self.rig.board.analog.scaling()['phase']
        return shunt.amps(1) - shunt.amps(0)

    def gate_supply(self):
        """Volts on the gate driver supply, and whether it clears UVLO."""
        rows = {r['signal']: r for r in
                self.rig.board.analog.read_all(nr_of_samples=16)['channels']}
        row = rows.get('Vgate')
        volts = row['volts_at_pin'] * 6.7 if row else None
        return {'volts': volts, 'powered': volts is not None and volts > GATE_UVLO_V}

    def _stage(self):
        if self.rig.gates.armed():
            return
        if self.arm is None:
            raise RigError('this step switches the stage and arming was not '
                           'authorised - pass arm=dict(bypass_sto=..., '
                           'ignore_interlock=...) to Commissioning')
        self.rig.gates.arm(**self.arm)

    def _rest(self):
        """Drive off, stage down, converters back to the meter."""
        self.drive.off()
        if self.rig.gates.armed():
            self.rig.gates.disarm()
        self.rig.board.gate_drivers.disarm()

    def _window(self, settle, seconds):
        """A window over `seconds`, after `settle` seconds discarded."""
        time.sleep(settle)
        self.drive.window()
        time.sleep(seconds)
        return self.drive.window()

    def _hold(self, settle=0.05, seconds=0.1, **setpoints):
        """HOLD with these setpoints, and the window it produced."""
        self.drive.setpoint(**setpoints)
        self.drive.mode('hold')
        return self._window(settle, seconds)

    # -- step 1: the AFE --------------------------------------------------

    def _noise_rows(self, moments):
        apc = self.amps_per_code()
        rows = {}
        for name, ch in moments['channels'].items():
            row = dict(ch, enob=sensorless.enob(ch['sd']))
            if name in PHASES:
                row['sd_amps'] = ch['sd'] * apc
            rows[name] = row
        return rows

    def afe_noise(self, periods=2000, zero_vector=True):
        """Noise floor per channel with the gates off, and on the zero
        vector with the stage armed - the difference is switch pickup."""
        self.rig.board.afe.enable()
        time.sleep(0.3)
        gd = self.rig.board.gate_drivers
        gd.arm()
        off = self._noise_rows(self.drive.moments_run(periods))
        out = {'gates_off': off, 'periods': periods,
               'trigger': gd.state()['trigger']}
        if zero_vector:
            self._stage()
            half = (gd.state()['period'] - 1) // 2
            gd.duty((half, half, half))
            time.sleep(0.05)
            zv = self._noise_rows(self.drive.moments_run(periods))
            gd.duty((0, 0, 0))
            self.rig.gates.disarm()
            out['zero_vector'] = zv
            out['pickup_amps'] = {
                n: math.sqrt(max(0.0, zv[n]['sd_amps'] ** 2
                                 - off[n]['sd_amps'] ** 2)) for n in PHASES}
        sd = [(out.get('zero_vector') or off)[n]['sd_amps'] for n in PHASES]
        out['sigma_i'] = math.sqrt(sum(s * s for s in sd) / 3.0)
        out['enob'] = sum(off[n]['enob'] for n in PHASES) / 3.0
        out['latency'] = self.latency()
        self.results['afe'] = out
        return out

    def latency(self):
        """From the sample to the duty that answers it, measured where it
        can be: the interrupt's own entry offset and its cost in cycles,
        and the two periods the pipeline adds by construction."""
        gd = self.rig.board.gate_drivers.state()
        ds = self.drive.state()
        sysclk = self.rig.board.system.clock()['sysclk_hz']
        tim_hz = sysclk / 2.0
        entry = ((gd['trigger'] - gd['at']) % gd['period']) / tim_hz
        return {'isr_entry_us': entry * 1e6,
                'isr_cost_us': ds['isr_cycles_max'] / sysclk * 1e6,
                'isr_cost_cycles': ds['isr_cycles_max'],
                'to_effect_periods': 2, 'to_effect_us': 2.0 * ds['ts'] * 1e6}

    def sample_point_scan(self, ticks=None, periods=500):
        """Move CCR5 across the period, on the zero vector, and keep the
        point with the least phase variance - after the ringing, before
        the next edge. Needs the stage: with nothing switching the scan
        is a walk through noise, and on this bench it picked 990 of 2376
        off exactly that."""
        gd = self.rig.board.gate_drivers
        period = gd.state()['period']
        if ticks is None:
            ticks = [int(period * k / 12.0) for k in range(1, 12)] + [period - 16]
        was = gd.state()['trigger']
        self._stage()
        half = (period - 1) // 2
        gd.duty((half, half, half))
        time.sleep(0.05)
        table = []
        for t in ticks:
            gd.trigger(t)
            m = self.drive.moments_run(periods)
            var = sum(m['channels'][n]['sd'] ** 2 for n in PHASES)
            table.append({'trigger': t, 'variance': var,
                          'sd': {n: m['channels'][n]['sd'] for n in PHASES}})
        gd.duty((0, 0, 0))
        best = min(table, key=lambda row: row['variance'])
        gd.trigger(best['trigger'])
        self.drive.set_params(drv_trigger_ticks=best['trigger'])
        out = {'table': table, 'best': best['trigger'], 'was': was,
               'period': period}
        self.results['sample_point'] = out
        return out

    def offsets(self, periods=2000, apply=True, limit_codes=3000):
        """Each phase's code at zero current becomes its offset. One past
        `limit_codes` looks like a fault and is reported, not applied - the
        reference board's Phase V op-amp reads -52 A with nothing connected,
        and zeroing it would hide that."""
        m = self.drive.moments_run(periods)
        cal = self.rig.board.calibration.read()['channels']
        out = {}
        for k, name in enumerate(PHASES):
            mean = m['channels'][name]['mean']
            suspect = abs(mean) > limit_codes
            out[name] = {'offset_raw': int(round(mean)), 'suspect': suspect,
                         'applied': False}
            if apply and not suspect:
                self.rig.board.calibration.set_channel(
                    k, int(round(mean)), cal[k]['gain_ppm'])
                out[name]['applied'] = True
        self.results['offsets'] = out
        return out

    def gain_mismatch(self, amps=1.0, periods=1000, apply=True):
        """Relative gains from ia + ib + ic = 0 with current in all three:
        a current vector held on each phase axis in turn, and the three
        sums solved for the two gain ratios in least squares."""
        self._stage()
        cal = self.rig.board.calibration.read()['channels']
        # The offsets as MEASURED at zero current, not as the record holds
        # them: one a step declined to apply still sits under the reading.
        zero = self.results.get('offsets') or {}
        rows = []
        for k in range(3):
            self.drive.setpoint(id_ref=amps, iq_ref=0.0, theta=k * TWO_PI / 3.0)
            self.drive.mode('hold')
            time.sleep(0.05)
            m = self.drive.moments_run(periods)
            rows.append([m['channels'][n]['mean']
                         - (zero[n]['offset_raw'] if n in zero
                            else cal[i]['offset_raw'])
                         for i, n in enumerate(PHASES)])
        self.drive.off()
        noise = max(m['channels'][n]['sd'] for n in PHASES)
        if max(abs(v) for row in rows for v in row) < 5.0 * noise:
            out = {'measured': False,
                   'why': 'no current flowed - the stage is unpowered on '
                          'this bench, so the sums say nothing'}
            self.results['gains'] = out
            return out
        # m_a + m_b u_b + m_c u_c = 0 per row, u = 1/g relative to phase a
        a11 = sum(r[1] * r[1] for r in rows)
        a12 = sum(r[1] * r[2] for r in rows)
        a22 = sum(r[2] * r[2] for r in rows)
        b1 = -sum(r[1] * r[0] for r in rows)
        b2 = -sum(r[2] * r[0] for r in rows)
        det = a11 * a22 - a12 * a12
        if abs(det) < 1e-9 * abs(a11 * a22) or det == 0.0:
            out = {'measured': False,
                   'why': 'the three sums are dependent - the current vector '
                          'did not move between the axes'}
            self.results['gains'] = out
            return out
        ub = (b1 * a22 - b2 * a12) / det
        uc = (a11 * b2 - a12 * b1) / det
        gains = [1.0, 1.0 / ub, 1.0 / uc]
        mean = sum(gains) / 3.0
        rel = [g / mean for g in gains]
        out = {'measured': True, 'relative_gain': dict(zip(PHASES, rel)),
               'mismatch_pct': 100.0 * (max(rel) - min(rel))}
        if apply:
            for k, name in enumerate(PHASES):
                ppm = int(round((1.0 / rel[k] - 1.0) * 1e6))
                self.rig.board.calibration.set_channel(
                    k, cal[k]['offset_raw'], ppm)
        self.results['gains'] = out
        return out

    # -- step 2: the inverter ---------------------------------------------

    def sign_check(self, volts=0.3, seconds=0.05):
        """A small positive d voltage on phase a: the current it makes says
        which way the shunts read. Sets drv_sign."""
        self._stage()
        self.drive.setpoint(vd=volts, vq=0.0, theta=0.0)
        self.drive.mode('volt')
        w = self._window(0.02, seconds)
        self.drive.off()
        iid = w['fields']['id']['mean']
        sd = w['fields']['id']['sd']
        if abs(iid) < 3.0 * sd:
            out = {'measured': False, 'id': iid, 'sign': +1,
                   'why': 'no current answered the voltage - the stage is '
                          'unpowered, or the winding is open'}
        else:
            sign = 1 if iid > 0.0 else -1
            self.drive.set_params(drv_sign=sign)
            out = {'measured': True, 'id': iid, 'sign': sign}
        self.results['sign'] = out
        return out

    def deadtime(self, currents=(0.25, 0.5, 1.0, 1.5, 2.0, 3.0, 4.0),
                 settle=0.05, seconds=0.1):
        """vd against a held d current on phase a: R from the slope and the
        dead-time curve from what is left, unfolded per phase into the
        board's table. Mandatory at weak saliency - the voltage error is an
        angle error in every estimate built on the applied voltage."""
        self._stage()
        points = []
        for amps in currents:
            w = self._hold(settle, seconds, id_ref=amps, iq_ref=0.0, theta=0.0)
            points.append((w['fields']['id']['mean'], w['fields']['vd']['mean']))
        self.drive.off()
        if max(abs(i) for i, _ in points) < 0.05:
            out = {'measured': False, 'points': points,
                   'why': 'no current flowed - the stage is unpowered'}
            self.results['deadtime'] = out
            return out
        r, v_dt, i_knee, residual = _fit_deadtime(points)
        step = max(currents) / 7.0
        table = [v_dt * math.tanh(k * step / i_knee) for k in range(8)]
        params = {'motor_r_uohm': r, 'drv_dt_step_ma': step}
        params.update({'drv_dt_mv%d' % k: v for k, v in enumerate(table)})
        self._set_table(params)
        out = {'measured': True, 'r': r, 'v_dt': v_dt, 'i_knee': i_knee,
               'residual_volts': residual, 'points': points,
               'table': table, 'step': step}
        self.results['deadtime'] = out
        return out

    def _set_table(self, params):
        """The dead-time table by id, since the record names its rows."""
        from .drive import to_wire
        plain = {k: v for k, v in params.items() if not k.startswith('drv_dt_mv')}
        self.drive.set_params(**plain)
        for k in range(8):
            self.rig.board.calibration.set_param(
                'drv_dt_mv%d' % k, to_wire('drv_inj_mv', params['drv_dt_mv%d' % k]))
        self.drive.reload()

    # -- step 3: the motor ------------------------------------------------

    def l_map(self, biases=(0.0, 1.0, 2.0, 3.0), points=8, v_inj=1.0,
              periods=1, settle=0.05, seconds=0.1):
        """L against bias current and injection angle, from V T / i_h.

        dL/L per bias is the second harmonic over the mean - the saturation
        saliency an SPM has and a salient rotor adds to - and the fourth is
        the secondary harmonic the demodulator has to live with.
        """
        self._stage()
        ts = 1.0 / self.fs
        angles = [math.pi * k / points for k in range(points)]
        self.drive.set_params(drv_inj_mv=v_inj, drv_inj_periods=periods)
        rows = {}
        for bias in biases:
            ls = []
            for phi in angles:
                self.drive.set_params(drv_inj_phase_mrad=phi)
                w = self._hold(settle, seconds, id_ref=bias, iq_ref=0.0, theta=0.0)
                ih = w['fields']['ih']['mean'] or 0.0
                ls.append(v_inj * ts / ih if ih > 0.0 else float('nan'))
            self.drive.off()
            if any(math.isnan(x) for x in ls):
                rows[bias] = {'measured': False, 'l': ls}
                continue
            h = _harmonics(ls, angles)
            rows[bias] = {'measured': True, 'l': ls, 'mean': h['mean'],
                          'dl': h['h2'], 'dl_over_l': h['h2'] / h['mean'],
                          'h4_over_l': h['h4'] / h['mean'],
                          # L(phi) = mean + h2 cos(2 phi - phase): the d axis
                          # is phi = 0, the q axis a quarter turn on
                          'ld': h['mean'] + h['h2'] * math.cos(h['h2_phase']),
                          'lq': h['mean'] - h['h2'] * math.cos(h['h2_phase'])}
        self.drive.set_params(drv_inj_mv=0.0, drv_inj_phase_mrad=0.0)
        base = rows.get(biases[0]) or {}
        out = {'angles': angles, 'rows': rows, 'v_inj': v_inj,
               'periods': periods, 'measured': bool(base.get('measured'))}
        if out['measured']:
            out['ld'], out['lq'] = base['ld'], base['lq']
            out['dl_over_l'] = base['dl_over_l']
            self.drive.set_params(motor_ld_nh=base['ld'], motor_lq_nh=base['lq'])
        self.results['l_map'] = out
        return out

    def flux(self, amps=2.0, omega=300.0, accel=1500.0, seconds=0.2):
        """lambda from an I/f spin: the back-EMF in the command frame is
        v - R i - j omega L i, and its magnitude over omega is the flux."""
        self._stage()
        p = self.drive.params()
        self.drive.setpoint(id_ref=amps, iq_ref=0.0, theta=0.0,
                            omega_target=omega, accel=accel)
        self.drive.mode('hold')
        w = self._window(omega / accel + 0.1, seconds)
        state = self.drive.state()
        self.drive.setpoint(omega_target=0.0)
        self.drive.off()
        f = w['fields']
        iid, iq, vd, vq = (f[k]['mean'] for k in ('id', 'iq', 'vd', 'vq'))
        r, ld, lq = p['motor_r_uohm'], p['motor_ld_nh'], p['motor_lq_nh']
        ed = vd - r * iid + omega * lq * iq
        eq = vq - r * iq - omega * ld * iid
        lam = math.hypot(ed, eq) / omega
        measured = abs(iid) > 0.05 and lam > 0.0
        out = {'measured': measured, 'lambda': lam,
               'load_angle': math.atan2(-ed, eq), 'omega': omega,
               'omega_hat': state['omega_hat'], 'e': (ed, eq)}
        if measured:
            self.drive.set_params(motor_lambda_uvs=lam)
        self.results['flux'] = out
        return out

    # -- steps 4 to 6: arithmetic on what was measured ---------------------

    def _known(self):
        p = self.drive.params()
        afe = self.results.get('afe') or {}
        return {'r': p['motor_r_uohm'], 'ld': p['motor_ld_nh'],
                'lq': p['motor_lq_nh'], 'lambda': p['motor_lambda_uvs'],
                'pole_pairs': p['motor_pole_pairs'] or 1.0,
                'sigma_i': afe.get('sigma_i') or p['drv_sigma_i_ua'] or 0.05,
                'vdc': self.drive.state()['vdc'] or 24.0,
                'i_max': p['drv_i_max_ma'] or 5.0}

    def budget(self):
        """f_inj and amplitude for the best SNR under the constraints."""
        k = self._known()
        loop = sensorless.current_loop(k['r'], k['ld'], self.fs, k['sigma_i'],
                                       k['vdc'])
        choice = sensorless.choose_injection(
            k['ld'], k['lq'], k['sigma_i'], self.fs, self.bw_est_hz, k['vdc'],
            self.i_h_max, f_min_hz=self.f_min_hz, bw_i_hz=loop['bw_hz'])
        out = {'known': k, 'loop': loop, 'choice': choice,
               'afe': 'sublime' if k['sigma_i'] < 0.01 else 'noisy'}
        self.results['budget'] = out
        return out

    def gains(self):
        """Loop gains, Kalman gains from the measured noise, the crossover.
        Written to the record, then the drive reloads them."""
        b = self.results.get('budget') or self.budget()
        k, loop, c = b['known'], b['loop'], b['choice']
        dt = self.results.get('deadtime') or {}
        cross = sensorless.crossover(
            k['lambda'], k['r'], k['i_max'],
            v_dt_residual=0.1 * (dt.get('v_dt') or 0.0) + 0.05,
            pole_pairs=k['pole_pairs'])
        params = {'drv_kp_mv_per_a': loop['kp'], 'drv_ki_v_per_as': loop['ki'],
                  'drv_sigma_i_ua': k['sigma_i'],
                  'drv_w_lo_mrad_s': cross['omega_e'],
                  'drv_w_hi_mrad_s': 2.0 * cross['omega_e']}
        kal = None
        if c is not None:
            t_upd = 2.0 * c['periods'] / self.fs
            sigma_upd = k['sigma_i'] / c['periods'] / abs(c['gain'])
            kal = sensorless.kalman_gains(sigma_upd, t_upd, self.accel_sd)
            params.update({'drv_l1_milli': kal['l1'], 'drv_l2_milli': kal['l2'],
                           'drv_inj_mv': c['v_inj'],
                           'drv_inj_periods': c['periods'],
                           'drv_eps_gain_ua_per_rad': c['gain']})
        self.drive.set_params(**params)
        out = {'loop': loop, 'kalman': kal, 'crossover': cross, 'written': params}
        self.results['gains'] = out
        return out

    def decide(self, threshold_db=10.0):
        """Injection when the budget clears the threshold at the largest
        amplitude the constraints allow; otherwise an I/f start and a
        saturation pulse for the polarity."""
        b = self.results.get('budget') or self.budget()
        snr_db = b['choice']['snr_db'] if b['choice'] else -100.0
        out = {'snr_db': snr_db, 'threshold_db': threshold_db,
               'method': sensorless.decide(snr_db, threshold_db)}
        self.results['decision'] = out
        return out

    # -- step 7: verification ---------------------------------------------

    def polarity(self, volts=3.0, periods=8, gap=40):
        """Two pulses along theta_hat; the one that saturates peaks higher.
        Flips theta_hat by pi when the negative one did."""
        self._stage()
        self.drive.setpoint(pol_volts=volts, pol_periods=periods, pol_gap=gap)
        before = self.drive.state()['theta_hat']
        self.drive.mode('polarity')
        time.sleep((2 * periods + 2 * gap + 8) / self.fs + 0.02)
        s = self.drive.state()
        flipped = s['pol_neg'] > s['pol_pos']
        if flipped:
            self.drive.set_theta(before + math.pi)
        out = {'pol_pos': s['pol_pos'], 'pol_neg': s['pol_neg'],
               'flipped': flipped, 'theta_hat': self.drive.state()['theta_hat']}
        self.results['polarity'] = out
        return out

    def verify(self, iq=0.5, seconds=1.0, lock=0.3):
        """Run sensorless, then judge the innovation: white by Ljung-Box,
        and its deviation as the sigma_theta proxy."""
        self._stage()
        d = self.results.get('decision') or self.decide()
        if d['method'] == 'injection':
            self.drive.setpoint(id_ref=0.0, iq_ref=0.0)
            self.drive.mode('sensorless')
            time.sleep(lock)
            self.polarity()
            self.drive.mode('sensorless')
        else:
            cross = (self.results.get('gains') or self.gains())['crossover']
            self.drive.setpoint(id_ref=self._known()['i_max'] / 2.0,
                                omega_target=1.5 * cross['omega_e'],
                                accel=cross['omega_e'] * 2.0, theta=0.0)
            self.drive.mode('hold')
            time.sleep(0.75 + lock)
            self.drive.mode('sensorless')
        self.drive.setpoint(iq_ref=iq)
        w = self._window(seconds / 2.0, seconds / 2.0)
        state = self.drive.state()
        self.drive.off()
        eps = w['fields']['eps']
        lb = sensorless.ljung_box(w['rho'], eps['n'] or 1)
        out = {'method': d['method'], 'window': w, 'ljung_box': lb,
               'sigma_theta_deg': math.degrees(eps['sd'] or 0.0),
               'omega_hat': state['omega_hat'], 'fault': state['fault'],
               'iq': w['fields']['iq']['mean']}
        self.results['verify'] = out
        return out

    def report(self):
        """The numbers that decide, and the one line."""
        r = self.results
        b = r.get('budget') or {}
        c = b.get('choice') or {}
        g = r.get('gains') or {}
        v = r.get('verify') or {}
        d = r.get('decision') or {}
        cross = g.get('crossover') or {}
        rpm = cross.get('rpm', float('nan'))
        min_pct = 100.0 * rpm / self.rated_rpm if self.rated_rpm else float('nan')
        line = sensorless.summary(
            d.get('method') == 'injection', c.get('snr_db', -100.0), min_pct,
            (g.get('loop') or {}).get('bw_hz', 0.0),
            v.get('sigma_theta_deg', float('nan')),
            100.0 * (v.get('omega_hat', 0.0) / (TWO_PI / 60.0)
                     / (self._known()['pole_pairs'] or 1.0)) / self.rated_rpm
            if self.rated_rpm else float('nan'))
        return {'line': line, 'results': r}

    def run(self, zero_vector=True, verify_seconds=1.0):
        """Every step in order; the stage down whatever happens."""
        try:
            self.log('gate supply %s' % self.gate_supply())
            self.afe_noise(zero_vector=zero_vector)
            self.sample_point_scan()
            self.offsets()
            self.gain_mismatch()
            self.sign_check()
            self.deadtime()
            self.l_map()
            self.flux()
            self.budget()
            self.gains()
            self.decide()
            self.verify(seconds=verify_seconds)
        finally:
            self._rest()
        return self.report()
