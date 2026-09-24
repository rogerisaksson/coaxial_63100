"""Commissioning: the bench day end to end, the record written and verified."""
from .parts import code, md, section

TITLE = 'Commissioning'
SUMMARY = 'The machine measured step by step, identified, a tune searched against it, the record written, the drive verified.'

SECTIONS = [
    section(
        'The converters: the floor, the sample point, the offsets',
        md('`Commissioning(rig, arm=...)`: `arm` is what `gates.on()` gets when a step '
           'switches. `gate_supply()` first: under 7.3 V the 2EDL8034 is in UVLO. Then the '
           'noise floor, gates off and on the zero vector.'),
        code('''from coaxial.control.commission import Commissioning

c = Commissioning(device, arm=dict(bypass_sto=True, ignore_interlock=True),
                  rated_rpm=3000.0)
print('gate supply', c.gate_supply())
afe = c.afe_noise()
for name, off in afe['gates_off'].items():
    on = afe['zero_vector'][name]
    print('%-8s %5.1f codes rms gates off, %5.1f on the zero vector, ENOB %.2f%s'
          % (name, off['sd'], on['sd'], off['enob'],
             ', %.4f A' % on['sd_amps'] if 'sd_amps' in on else ''))
print('sigma_i %.4f A over the three phases, ENOB %.1f; pickup %s A'
      % (afe['sigma_i'], afe['enob'],
         [round(v, 4) for v in afe['pickup_amps'].values()]))
lat = afe['latency']
print('ISR entry %.1f us after the trigger, cost %.1f us (%d cycles); '
      'to effect %d periods = %.0f us'
      % (lat['isr_entry_us'], lat['isr_cost_us'], lat['isr_cost_cycles'],
         lat['to_effect_periods'], lat['to_effect_us']))'''),
        md('`sample_point_scan`: CCR5 across the period on the zero vector, the tick with '
           'the least phase variance -> `drv_trigger_ticks`. Not switching, it once picked '
           '990 of 2376.'),
        code('''from coaxial.draw.figures import figure, show

scan = c.sample_point_scan()
ticks = [row['trigger'] for row in scan['table']]
variance = [row['variance'] for row in scan['table']]
fig, (panel,) = figure()
panel.plot(ticks, variance, '.-')
panel.axvline(scan['best'], linestyle=':', color='C3')
panel.set_xlabel('CCR5, ticks of %d' % scan['period'])
panel.set_ylabel('phase variance, codes^2')
show(fig)
print('sample point CCR5 %d of %d (was %d): variance %.0f there, %.0f at tick %d'
      % (scan['best'], scan['period'], scan['was'], min(variance), max(variance),
         ticks[variance.index(max(variance))]))'''),
        md('Offsets applied unless past `limit_codes` (-52 A with nothing connected is a '
           'fault). Gain ratios from `ia + ib + ic = 0`, a vector on each phase axis.'),
        code('''offsets = c.offsets()
for name, row in offsets.items():
    print('%-8s %+6d codes  %s' % (name, row['offset_raw'],
                                   'SUSPECT - reported, not applied' if row['suspect'] else 'applied'))
mismatch = c.gain_mismatch()
if mismatch['measured']:
    print('relative gains %s, mismatch %.2f %%'
          % ({n[-1]: round(g, 4) for n, g in mismatch['relative_gain'].items()},
             mismatch['mismatch_pct']))
else:
    print('gain mismatch not measured -', mismatch['why'])'''),
    ),
    section(
        'The inverter: the sign and the dead time',
        md('The shunt sign from a small positive vd. Then vd against id 0.25-4 A: R the '
           "slope, the rest `v_dt tanh(I / i_knee)`, into the record's eight-row table."),
        code('''sign = c.sign_check()
print('sign %+d, from id %.3f A' % (sign['sign'], sign['id']) if sign['measured']
      else 'sign not measured - ' + sign['why'])
dead = c.deadtime()
if dead['measured']:
    print('R %.4f ohm, V_dt %.3f V, knee %.2f A; fit residual %.4f V rms over %d points'
          % (dead['r'], dead['v_dt'], dead['i_knee'], dead['residual_volts'],
             len(dead['points'])))
    print('table written, %.3f A a row: %s' % (dead['step'], [round(v, 3) for v in dead['table']]))
else:
    print('dead time not measured -', dead['why'])'''),
        code('''import math

def bend(amps):
    return dead['v_dt'] * math.tanh(amps / dead['i_knee'])

def folded(amps):
    return (2.0 / 3.0) * (bend(amps) + bend(amps / 2.0))

amps = [i for i, _ in dead['points']]
grid = [max(amps) * k / 100.0 for k in range(1, 101)]
fig, (panel,) = figure()
panel.plot(amps, [v for _, v in dead['points']], 'o', label='vd held, measured')
panel.plot(grid, [dead['r'] * i + folded(i) for i in grid], label='R I + dead time, fitted')
panel.plot(grid, [dead['r'] * i for i in grid], ':', label='R I alone')
panel.set_xlabel('id, A')
panel.set_ylabel('vd, V')
panel.legend(loc='lower right')
show(fig)
i_top, v_top = dead['points'][-1]
print('at %.2f A: %.3f V held, of which R I is %.3f V and the dead time %.3f V'
      % (i_top, v_top, dead['r'] * i_top, folded(i_top)))'''),
    ),
    section(
        'The motor: the inductance map and the flux',
        md('L against injection angle at four d biases: the mean L, the 2nd harmonic dL/L. '
           'Lambda from an I/f spin, 300 rad/s at 2 A.'),
        code('''lmap = c.l_map()
if lmap['measured']:
    print('Ld %.1f uH, Lq %.1f uH at zero bias; dL/L %.3f, fourth harmonic %.3f of L'
          % (lmap['ld'] * 1e6, lmap['lq'] * 1e6, lmap['dl_over_l'],
             lmap['rows'][0.0]['h4_over_l']))
    for bias, row in lmap['rows'].items():
        if row.get('measured'):
            print('  %.0f A of d bias: L %.2f uH mean, Ld %.2f, Lq %.2f, dL/L %.3f'
                  % (bias, row['mean'] * 1e6, row['ld'] * 1e6, row['lq'] * 1e6, row['dl_over_l']))
else:
    print('L map not measured - the injection saw no current')
flux = c.flux()
print('lambda %.5f V.s from a spin at %.0f rad/s; load angle %.2f rad, back-EMF (%.3f, %.3f) V'
      % (flux['lambda'], flux['omega'], flux['load_angle'], flux['e'][0], flux['e'][1])
      if flux['measured'] else 'flux not measured - no current under I/f')'''),
        code('''angles = [math.degrees(a) for a in lmap['angles']]
fig, (panel,) = figure()
for bias, row in lmap['rows'].items():
    if row.get('measured'):
        panel.plot(angles, [x * 1e6 for x in row['l']], '.-', label='%.0f A of d bias' % bias)
panel.set_xlabel('injection angle from the d axis, electrical degrees')
panel.set_ylabel('L, uH')
panel.legend(loc='upper left')
show(fig)'''),
        md('The four constants into a `Parameters`; a step that saw no current: `measured: '
           "False`, the record's value stands in. The stand-in's machine is `BENCH_MOTOR`."),
        code('''from coaxial.model.motor import BENCH_MOTOR, Parameters

p = device.drive.params()

def got(step, key, fallback):
    block = c.results.get(step) or {}
    return block[key] if block.get('measured') is not False and key in block else fallback

identified = Parameters(
    name='commissioned',
    r=got('deadtime', 'r', p['motor_r_uohm']),
    ld=got('l_map', 'ld', p['motor_ld_nh']),
    lq=got('l_map', 'lq', p['motor_lq_nh']),
    lam=got('flux', 'lambda', p['motor_lambda_uvs']),
    poles=int(p['motor_pole_pairs']),
    sat=0.3, i_sat=4.0,      # the bend's shape as the record has it; the map shows it, no step fits it
    measured=True, source='commissioning on this rig')
print(identified)
vdc = device.drive.state()['vdc']
print('link %.2f V; saliency Lq/Ld %.3f' % (vdc, identified.saliency))
truth = BENCH_MOTOR if SIMULATED else None
if truth:
    for name in ('r', 'ld', 'lq', 'lam'):
        print('   %-3s identified %.6g against the stand-in\\'s %.6g: %+.1f %%'
              % (name, getattr(identified, name), getattr(truth, name),
                 100.0 * (getattr(identified, name) / getattr(truth, name) - 1.0)))'''),
    ),
    section(
        'The budget, the gains and the decision',
        md('`coaxial.model.sensorless`: the injection budget, the gains (PI, PLL, '
           'crossover), the decision - injection if the budget clears the threshold, else '
           'I/f.'),
        code('''budget = c.budget()
choice, loop = budget['choice'], budget['loop']
print('AFE %s: sigma_i %.4f A' % (budget['afe'], budget['known']['sigma_i']))
print('current loop %.0f Hz, limited by %s: kp %.4f V/A, ki %.1f V/(A s)'
      % (loop['bw_hz'], loop['limited_by'], loop['kp'], loop['ki']))
if choice:
    print('injection %.0f Hz over %d period(s), %.3f V, %.3f A peak, SNR %.1f dB, limited by %s'
          % (choice['f_inj_hz'], choice['periods'], choice['v_inj'], choice['i_h_peak'],
             choice['snr_db'], choice['limited_by']))
else:
    print('no injection clears the constraints')
gains = c.gains()
closed = dict(gains['written'])
kal, cross = gains['kalman'], gains['crossover']
if kal:
    print('PLL %.0f Hz, zeta %.2f: l1 %.4f, l2 %.2f; sigma_theta estimate %.3f rad'
          % (kal['wn_hz'], kal['zeta'], kal['l1'], kal['l2'], kal['sigma_theta_est']))
print('crossover: %.3f V of floor, %.1f rad/s electrical = %.0f rpm; blend %.0f to %.0f rad/s'
      % (cross['floor_volts'], cross['omega_e'], cross['rpm'],
         closed['drv_w_lo_mrad_s'], closed['drv_w_hi_mrad_s']))
decision = c.decide()
print('decision: %s (SNR %.1f dB against %.0f)'
      % (decision['method'], decision['snr_db'], decision['threshold_db']))'''),
    ),
    section(
        'A first run, under the closed-form tune',
        md('`verify`: sensorless under the record; innovation white by Ljung-Box over 7 '
           'lags, its deviation the `sigma_theta` proxy. Polarity by two saturation pulses.'),
        code('''first = c.verify(iq=0.5, seconds=1.0)
pol = c.results.get('polarity')
if pol:
    print('polarity pulses: aligned %.0f, opposed %.0f, %s'
          % (pol['pol_pos'], pol['pol_neg'], 'theta_hat flipped by pi' if pol['flipped'] else 'theta_hat kept'))
lb = first['ljung_box']
print('%s: sigma_theta %.2f deg, iq %.2f A, omega_hat %.1f rad/s, fault %s'
      % (first['method'], first['sigma_theta_deg'], first['iq'], first['omega_hat'], first['fault']))
print('innovation Ljung-Box Q %.2f against %.2f at %d lags: %s'
      % (lb['q'], lb['threshold'], lb['lags'], 'white' if lb['white'] else 'NOT white'))
print(c.report()['line'])'''),
    ),
    section(
        'A tune searched against this machine',
        md("`run_job` scores the firmware's C on plants drawn around this machine. Cost "
           '`sigma_theta + speed_err + 10 x trip`; `robust` = mean + 90th percentile. 6 '
           'candidates x 3 draws here; the tool runs 48 x 16.'),
        code('''import os
import sys

sys.path.insert(0, os.path.join('..', 'host'))   # the Monte Carlo is a tool beside the library, not part of it
from tools.sim import montecarlo as mc'''),
        code('''import time

fields = {k: getattr(identified, k) for k in ('name', 'r', 'ld', 'lq', 'lam', 'poles',
                                             'j', 'b', 'sat', 'i_sat', 'measured', 'source')}
i_max, i_trip = p['drv_i_max_ma'], p['drv_i_trip_ma']
knobs = mc.candidates(6, seed=3)
jobs = [{'vdc': vdc, 'knobs': k, 'seed': 1000 * i + s, 'motor': fields,
         'i_max': i_max, 'i_trip': i_trip, 'i_h_max': 1.0, 'k_prop': 0.0}
        for i, k in enumerate(knobs) for s in range(3)]
t0 = time.perf_counter()
with mc.pool() as pool:
    runs = mc.sweep(pool, jobs, progress=False)
took = time.perf_counter() - t0
score = mc.score(runs).sort_values('robust')
best = score.iloc[0]
print(score.to_string(index=False, columns=['robust', 'mean', 'p90'] + list(mc.KNOBS),
                      float_format='%.4f'))
runs['candidate'] = runs.seed // 1000
by = runs.groupby('candidate').agg(cost=('cost', 'mean'), trips=('trip', 'sum'),
                                   i_peak=('i_peak', 'max'), i_h=('i_h', 'max'))
print(by.round(3).to_string())
print('%d runs in %.1f s, %d tripped the stage at i_trip %.0f A; '
      'best robust %.3f = mean %.3f + p90 %.3f, i_max %.0f A'
      % (len(runs), took, int(runs.trip.sum()), i_trip, best['robust'], best['mean'],
         best['p90'], i_max))'''),
    ),
    section(
        'The record written, and the drive verified under it',
        md('`design` -> firmware parameters; `configure` writes them in SI and answers what '
           'the record holds after rounding. Then `verify` again.'),
        code('''tune = mc.design({k: float(best[k]) for k in mc.KNOBS}, vdc, identified, i_max, i_trip, 1.0)
written = device.drive.configure(
    motor_r_uohm=identified.r, motor_ld_nh=identified.ld, motor_lq_nh=identified.lq,
    motor_lambda_uvs=identified.lam,
    drv_kp_mv_per_a=tune['kp'], drv_ki_v_per_as=tune['ki'],
    drv_l1_milli=tune['l1'], drv_l2_milli=tune['l2'],
    drv_inj_mv=tune['inj_volts'], drv_inj_periods=tune['inj_periods'],
    drv_eps_gain_ua_per_rad=tune['eps_gain'],
    drv_w_lo_mrad_s=tune['w_lo'], drv_w_hi_mrad_s=tune['w_hi'])['params']
print('%-24s %-12s %s' % ('written for %.1f V' % vdc, 'searched', 'closed form'))
for name, value in written.items():
    print('%-24s %-12.6g %s' % (name, value, '%.6g' % closed[name] if name in closed else '-'))
check = c.verify(iq=0.5, seconds=1.0)
lb = check['ljung_box']
print('%s under the searched tune: sigma_theta %.2f deg (was %.2f), Q %.2f against %.2f, %s, fault %s'
      % (check['method'], check['sigma_theta_deg'], first['sigma_theta_deg'], lb['q'],
         lb['threshold'], 'white' if lb['white'] else 'NOT white', check['fault']))'''),
        md('`calibration.save()` keeps the record across a reset.'),
        code('''print('saved:', device.calibration.save())
after = device.drive.params()
for name in sorted(after):
    print('   %-26s %-14.6g %s' % (name, after[name], 'written today' if name in written else ''))
print('%d parameters in the record, %d of them written today' % (len(after), len(written)))
device.drive.off()
device.gates.off()
device.gates.control.configure(sync=False)
print('stage armed:', device.gates.is_on())'''),
    ),
]

RESULTS = [
    code('''def line(number, name, block, text):
    if isinstance(block, dict) and block.get('measured') is False:
        print('%2d. %-13s not measured - %s' % (number, name, block.get('why', 'no current')))
    else:
        print('%2d. %-13s %s' % (number, name, text()))

line(1, 'AFE', afe, lambda: 'sigma_i %.4f A, ENOB %.1f, pickup %.4f A; ISR %.1f us of a %.0f us period'
     % (afe['sigma_i'], afe['enob'], max(afe['pickup_amps'].values()), lat['isr_cost_us'], 1e6 / c.fs))
line(2, 'sample point', scan, lambda: 'CCR5 %d of %d (was %d); variance %.0f there, %.0f mid-period'
     % (scan['best'], scan['period'], scan['was'], min(variance), max(variance)))
line(3, 'offsets', offsets, lambda: ' '.join(
    '%s %+d%s' % (n[-1], v['offset_raw'], ' SUSPECT' if v['suspect'] else '')
    for n, v in offsets.items()))
line(4, 'chains', mismatch, lambda: 'relative gains %s, mismatch %.2f %%'
     % (' '.join('%.4f' % g for g in mismatch['relative_gain'].values()), mismatch['mismatch_pct']))
line(5, 'sign', sign, lambda: '%+d, id %.3f A' % (sign['sign'], sign['id']))
line(6, 'dead time', dead, lambda: 'R %.4f ohm, V_dt %.3f V, knee %.2f A, residual %.4f V'
     % (dead['r'], dead['v_dt'], dead['i_knee'], dead['residual_volts']))
line(7, 'L map', lmap, lambda: 'Ld %.1f uH, Lq %.1f uH, dL/L %.3f; Ld %.1f uH at 3 A of bias'
     % (lmap['ld'] * 1e6, lmap['lq'] * 1e6, lmap['dl_over_l'], lmap['rows'][3.0]['ld'] * 1e6))
line(8, 'flux', flux, lambda: 'lambda %.5f V.s = KV %.0f, load angle %.2f rad'
     % (flux['lambda'], identified.kv, flux['load_angle']))
line(9, 'budget', budget, lambda: 'f_inj %.0f Hz, V %.2f, i_h %.3f A, SNR %.1f dB, limited by %s'
     % (choice['f_inj_hz'], choice['v_inj'], choice['i_h_peak'], choice['snr_db'], choice['limited_by']))
line(10, 'gains', gains, lambda: 'iloop %.0f Hz, PLL %.0f Hz, crossover %.0f rpm = %.1f %% of rated'
     % (loop['bw_hz'], (kal or {}).get('wn_hz', 0.0), cross['rpm'], 100.0 * cross['rpm'] / c.rated_rpm))
line(11, 'decision', decision, lambda: '%s (SNR %.1f dB against %.0f)'
     % (decision['method'], decision['snr_db'], decision['threshold_db']))
line(12, 'first run', first, lambda: 'sigma_theta %.2f deg, innovation %s, fault %s'
     % (first['sigma_theta_deg'], 'white' if first['ljung_box']['white'] else 'NOT white', first['fault']))
print('13. search        %d runs, %d tripped; best robust %.3f (mean %.3f, p90 %.3f): '
      'bw_i %.0f Hz, PLL %.0f Hz, zeta %.2f, v_inj %.3f x %d, w_lo %.0f, ratio %.2f, bw_w %.1f Hz'
      % (len(runs), int(runs.trip.sum()), best['robust'], best['mean'], best['p90'],
         best['bw_i'], best['f_pll'], best['zeta'], best['v_inj'], int(best['n_inj']),
         best['w_lo'], best['w_ratio'], best['bw_w']))
print('14. tune          closed form -> searched: kp %.3f -> %.3f V/A, ki %.0f -> %.0f, '
      'l1 %.4f -> %.4f, l2 %.2f -> %.2f'
      % (closed['drv_kp_mv_per_a'], written['drv_kp_mv_per_a'], closed['drv_ki_v_per_as'],
         written['drv_ki_v_per_as'], closed['drv_l1_milli'], written['drv_l1_milli'],
         closed['drv_l2_milli'], written['drv_l2_milli']))
print('                  injection %.3f V x %d -> %.3f V x %d, eps gain %.3f -> %.3f, '
      'blend %.0f-%.0f -> %.0f-%.0f rad/s'
      % (closed['drv_inj_mv'], closed['drv_inj_periods'], written['drv_inj_mv'],
         written['drv_inj_periods'], closed['drv_eps_gain_ua_per_rad'],
         written['drv_eps_gain_ua_per_rad'], closed['drv_w_lo_mrad_s'], closed['drv_w_hi_mrad_s'],
         written['drv_w_lo_mrad_s'], written['drv_w_hi_mrad_s']))
print('15. verified      sigma_theta %.2f -> %.2f deg, innovation %s, fault %s; '
      'saved, %d parameters read back, %d written today'
      % (first['sigma_theta_deg'], check['sigma_theta_deg'],
         'white' if check['ljung_box']['white'] else 'NOT white', check['fault'],
         len(after), len(written)))
if truth:
    print('16. truth         R %+.1f %%, Ld %+.1f %%, Lq %+.1f %%, lambda %+.1f %% off the stand-in\\'s'
          % tuple(100.0 * (getattr(identified, k) / getattr(truth, k) - 1.0)
                  for k in ('r', 'ld', 'lq', 'lam')))'''),
    md('- Noise floor 0.16 A rms, 9.5 ENOB; sample point 2360 of 2376.\n- R, Ld, Lq within '
       '3 % of the truth; lambda 9 % high.\n- Dead time 0.50 V, knee 0.30 A; injection 20 '
       "dB against 10.\n- Searched tune 1.7 deg against the closed form's 11.2; a third of "
       'the 18 runs tripped at 100 A.\n- Record ids 15-44 are placeholders until a motor is '
       'commissioned (TODO).'),
]

BENCH = ('`tools/bench/commission.py --arm --port COM4` is the twelve steps in one command. '
         'Finding 3 first: SUSPECT is a fault, not an offset.')

REFERENCES = [
    ('host/coaxial/control/commission.py', 'the twelve steps, and the one-line report'),
    ('host/coaxial/model/sensorless.py', 'the arithmetic: the budget, the loop and PLL gains, the crossover, Ljung-Box'),
    ('host/tools/sim/montecarlo.py', 'the search: the plants drawn around the machine, the cost, `design`'),
    ('host/tools/bench/commission.py', 'the procedure as one command at the bench'),
    ('host/coaxial/model/motor.py', '`Parameters`, and `BENCH_MOTOR` - the stand-in\'s truth'),
    ('host/coaxial/simulated/drive/', 'the stand-in this ran on: the machine, the pickup, the polarity readings'),
    ('host/tests/test_sensorless.py', 'the arithmetic and the commissioning pinned against the stand-in'),
    ('host/tests/test_drive_core.py', 'the firmware\'s law through the host gcc, which the search drives'),
]

