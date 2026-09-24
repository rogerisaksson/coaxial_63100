"""Motion: stepper, servo and velocity, the loop identified, the 5230SL against its stand."""
from .parts import code, md, section

TITLE = 'Motion'
SUMMARY = 'A stepper and its ring, a servo and its sag, a sensorless speed loop against the shaft, the loop identified, a propeller.'

SECTIONS = [
    section(
        'The stage, armed',
    md("`device.gates.on()` arms; the drive on the `model` source turns the stand-in's "
       'rotor and its shaft sensor. `Kt = 1.5 P lambda` from the record.'),
    code('''import math
import time

drive = device.drive
drive.configure(source='model')
J, B = 2e-5, 1e-5
drive.model.configure(j=J, b=B, load=0.0)
stage = device.gates.on(bypass_sto=True, ignore_interlock=True)
params = drive.params()
poles = int(params['motor_pole_pairs'])
kt = 1.5 * poles * params['motor_lambda_uvs']
print('armed:', stage['pwm_enabled'])
print('%d pole pairs, lambda %.5f Wb, Kt = 1.5 P lambda = %.4f N.m/A; J %.0e kg.m^2, B %.0e N.m.s'
      % (poles, params['motor_lambda_uvs'], kt, J, B))'''),
    ),
    section(
        'The stepper: a move and its ring',
        md('HOLD drags the rotor on the load-angle spring `amps Kt sin(delta)`: open loop. '
           'Ring `sqrt(Kt I P / J)` ~30 Hz, read every 2 ms (20 ms aliased it, 2026-09-07).'),
        code('''def trace(seconds, zero, every=0.002):
    """The shaft every `every` seconds, mechanical degrees from `zero`."""
    rows = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        deg = device.angle.state()['degrees']
        rows.append((time.monotonic() - t0, (deg - zero + 180.0) % 360.0 - 180.0))
        time.sleep(every)
    return rows

with device.motion.stepper(amps=2.0, deg_s=90.0) as m:
    detent = device.angle.state()['degrees']
    print('detented: command at %.2f deg, shaft at %.2f' % (m.position, detent))
    m.to(90.0)
    ring = trace(0.6, detent)
    m.step(10)
    print('ten full steps of 1.8 deg: command at %.1f deg' % m.position)
ys = [r[1] for r in ring]
level = sum(ys) / len(ys)
crossings = sum(1 for a, b in zip(ys, ys[1:]) if (a - level) * (b - level) < 0)
ring_hz = crossings / 2.0 / ring[-1][0]
spring_hz = math.sqrt(kt * 2.0 * poles / J) / (2.0 * math.pi)
print('after the 90 deg move: shaft %.2f deg from the detent, ringing %.2f deg '
      'peak to peak at about %.0f Hz; the spring says %.1f Hz'
      % (level, max(ys) - min(ys), ring_hz, spring_hz))
print('%d reads in %.2f s = %.0f Hz' % (len(ring), ring[-1][0], len(ring) / ring[-1][0]))'''),
        code('''from coaxial.draw.figures import figure, show

fig, (panel,) = figure(rows=1)
panel.plot([r[0] for r in ring], ys, linewidth=0.8)
panel.set_xlabel('s after the move')
panel.set_ylabel('shaft, deg from the detent')
show(fig)'''),
    ),
    section(
        'The servo: sag under load, corrected',
        md('The servo: slew, let the ring die, read a mean over it, correct. Sag = `asin(T '
           '/ I Kt) / P`.'),
        code('''with device.motion.servo(amps=2.0, settle=0.3) as s:
    landed = s.to(45.0, tol=0.5)
    print('no load:   shaft %.2f deg, error %.2f, swing %.2f deg' % (landed, s.error, s.swing))
    first = device.angle.state()['degrees']
    before = trace(0.2, first)
    held = first + sum(r[1] for r in before) / len(before)
    drive.model.configure(load=0.02)
    pulled = trace(1.0, held)
    tail = pulled[len(pulled) // 2:]
    sag = sum(r[1] for r in tail) / len(tail)
    back = s.to(45.0, tol=0.5)
    print('0.02 N.m:  sagged %.2f deg, ringing %.2f deg peak to peak under the load'
          % (sag, max(r[1] for r in tail) - min(r[1] for r in tail)))
    print('corrected: shaft %.2f deg, error %.2f, swing %.2f deg' % (back, s.error, s.swing))
    drive.model.configure(load=0.0)
spring_sag = math.degrees(math.asin(0.02 / (2.0 * kt))) / poles
print('the spring says %.2f deg mechanical: 0.02 N.m against 2.0 A x Kt %.4f = %.3f N.m'
      % (spring_sag, kt, 2.0 * kt))'''),
    ),
    section(
        'Sensorless: the observer against the shaft',
        md("The observer's electrical angle against the A1335's mechanical one: angles "
           'compared at rest, speeds on the way (600 rpm = 3.6 deg/ms). The stage is off '
           'after this section.'),
        code('''from coaxial.model.sensorless import RAD_S_PER_RPM

rows = []

def watch(v):
    m = drive.model.read()
    shaft = device.angle.state()['degrees']
    now = time.monotonic() - t0
    shaft_rpm = float('nan')
    if rows:
        then, was = rows[-1][0], rows[-1][7]
        shaft_rpm = ((shaft - was + 90.0) % 360.0 - 90.0) / (now - then) / 6.0
    bus = v.loop.read()
    rows.append((now, bus['w_ref'] / RAD_S_PER_RPM, m['omega'] / poles / RAD_S_PER_RPM,
                 v.rpm_now, shaft_rpm, math.degrees(m['error']), bus['iq_ref'], shaft))

t0 = time.monotonic()
with device.motion.velocity(amps=1.0, hz=3.0) as v:
    v.rpm(600.0, seconds=3.0, watch=watch)
    print('settled at %.0f rpm' % v.rpm_now)
    v.rpm(0.0, seconds=1.5, watch=watch)
    print('stopped at %.0f rpm' % v.rpm_now)
    hat = math.degrees(drive.model.read()['theta_hat']) / poles
    shaft = device.angle.state()['degrees']
pitch = 360.0 / poles
offset = (shaft - hat + pitch / 2.0) % pitch - pitch / 2.0
print('%d passes in %.2f s = %.1f Hz' % (len(rows), rows[-1][0], len(rows) / rows[-1][0]))
print('at rest: observer %.2f deg electrical / %d = %.2f deg; shaft %.2f deg, folded onto '
      'the %.2f deg pole pitch %.2f; %.2f deg apart' % (hat * poles, poles, hat, shaft,
                                                         pitch, shaft % pitch, offset))
print('disarmed:', not device.gates.off()['pwm_enabled'])
drive.configure(source='adc')
print('source:', drive.model.read()['source'])'''),
        code('''t = [r[0] for r in rows]
fig, (speed, error, current) = figure(rows=3, sharex=True)
speed.plot(t, [r[1] for r in rows], label='asked')
speed.plot(t, [r[2] for r in rows], label='rotor (model)')
speed.plot(t, [r[3] for r in rows], '.', markersize=3, label='observer')
speed.plot(t, [r[4] for r in rows], linewidth=0.8, label='shaft, stepped')
speed.set_ylabel('rpm')
speed.legend(loc='upper right')
error.plot(t, [r[5] for r in rows])
error.set_ylabel('observer error, el deg')
current.plot(t, [r[6] for r in rows])
current.set_ylabel('iq_ref, A')
current.set_xlabel('s')
show(fig)'''),
    ),
    section(
        'The speed loop identified out of its own run',
        md('Ramp, d-axis probe, speed loop, current loop, machine on one bus; `identify` '
           'returns R, Ld, Lq, lambda, each with its standard error.'),
        code('''from coaxial.control.loop import Ramp, Probe, SpeedLoop, CurrentLoop, Machine, identify
from coaxial.model.motor import BENCH_MOTOR
from coaxial.model import inverter

TWO_PI = 2.0 * math.pi
bench = BENCH_MOTOR
print(bench)
vdc = 24.0
chain = (Ramp(top=300.0 * TWO_PI, rise=0.6)
         >> Probe(amps=1.0, hz=40.0)
         >> SpeedLoop(hz=8.0, limit=6.0, motor=bench)
         >> CurrentLoop(hz=1000.0, motor=bench, vdc=vdc)
         >> Machine(bench, vdc=vdc, noise=0.02, sub=4))
run = chain.run(seconds=1.4, dt=2.0 * inverter.TS)
print('samples %d at %.0f us; top %.0f rad/s mechanical'
      % (len(run['t']), 2e6 * inverter.TS, run['w'].max()))'''),
        code('''fig, (speed, current, voltage) = figure(rows=3, sharex=True)
speed.plot(run['t'], run['w_ref'], label='w_ref')
speed.plot(run['t'], run['w'], label='w')
speed.set_ylabel('rad/s')
speed.legend()
current.plot(run['t'], run['id'], label='id')
current.plot(run['t'], run['iq'], label='iq')
current.set_ylabel('A')
current.legend()
voltage.plot(run['t'], run['vd'], label='vd')
voltage.plot(run['t'], run['vq'], label='vq')
voltage.set_ylabel('V')
voltage.set_xlabel('s')
voltage.legend()
show(fig)'''),
        code('''fit, got = identify(run, bench.poles)
print(fit)
print(fit.source)
print('condition %.2e  residual %.4f V' % (got['condition'], got['residual_v']))
TRUTH = (('r', bench.r), ('ld', bench.ld), ('lq', bench.lq), ('lam', bench.lam))
for name, truth in TRUTH:
    print('%-4s truth %.5g  fit %.5g  %+.1f %%  uncertainty %.1f %%  trusted %s'
          % (name, truth, got[name], 100.0 * (got[name] / truth - 1.0),
             100.0 * got['uncertainty'][name], got['trusted'][name]))'''),
        md('Without the probe: `did/dt` unexcited.'),
        code('''still = (Ramp(top=300.0 * TWO_PI, rise=0.6)
         >> SpeedLoop(hz=8.0, limit=6.0, motor=bench)
         >> CurrentLoop(hz=1000.0, motor=bench, vdc=vdc)
         >> Machine(bench, vdc=vdc, noise=0.02, sub=4))
run2 = still.run(seconds=1.4, dt=2.0 * inverter.TS)
_, got2 = identify(run2, bench.poles)
for name, truth in TRUTH:
    print('%-4s fit %.5g  %+.1f %%  uncertainty %.1f %%  trusted %s'
          % (name, got2[name], 100.0 * (got2[name] / truth - 1.0),
             100.0 * got2['uncertainty'][name], got2['trusted'][name]))'''),
    ),
    section(
        'The 5230SL and its propeller against the stand',
        md('`coaxial.model.motor`: the 5230SL 190KV from the sheet, R/Ld/Lq/J from the size '
           "class; the APC20x10E fitted over the stand's 22 rows at 37 V; up to 6717 rpm "
           'and back.'),
        code('''from coaxial.model.motor import PLATINUM_5230SL, APC20x10E, APC20X10E_CURVE, RATINGS, KT_NM_PER_AMP

motor = PLATINUM_5230SL
print(motor)
print(motor.source)
print(APC20x10E, '-', APC20x10E.source)
print({k: RATINGS[k] for k in ('slots_poles', 'kv', 'i_max', 'p_max', 'source')})
print('Kt %.4f N.m/A' % KT_NM_PER_AMP)'''),
        code('''import numpy as np

top = 6717.0 * TWO_PI / 60.0
sweep = (Ramp(top, rise=2.0)
         >> SpeedLoop(hz=4.0, limit=RATINGS['i_max'], motor=motor, load=APC20x10E)
         >> CurrentLoop(hz=800.0, motor=motor, vdc=37.0)
         >> Machine(motor, vdc=37.0, load=APC20x10E, noise=0.0, sub=4))
prop = sweep.run(seconds=4.5, dt=4.0 * inverter.TS, every=25)
rpm = prop['w'] * 60.0 / TWO_PI
torque = APC20x10E.k * prop['w'] ** 2
electrical = 1.5 * (prop['vd'] * prop['id'] + prop['vq'] * prop['iq'])
peak = int(rpm.argmax())
w_e = prop['w_e'][peak]
print('reached %.0f rpm, iq peak %.1f A, v_sat %.0f %% of the run'
      % (rpm.max(), abs(prop['iq']).max(), 100.0 * prop['v_sat'].mean()))
print('at the top: back-EMF %.1f V, omega Lq iq %.1f V, |v| %.1f of the %.1f V a sine gets from 37 V'
      % (w_e * motor.lam, w_e * motor.lq * prop['iq'][peak],
         math.hypot(prop['vd'][peak], prop['vq'][peak]), 37.0 / math.sqrt(3.0)))'''),
        code('''stand = np.array(APC20X10E_CURVE)
fig, (shaft, power) = figure(rows=1, cols=2)
shaft.plot(rpm, torque, label='model, k w^2')
shaft.plot(stand[:, 0], stand[:, 1], 'o', label='Hobbywing stand')
shaft.set_xlabel('rpm')
shaft.set_ylabel('N.m')
shaft.legend()
power.plot(rpm, electrical, label='model, 1.5 (vd id + vq iq)')
power.plot(stand[:, 0], stand[:, 2], 'o', label='stand input W')
power.set_xlabel('rpm')
power.set_ylabel('W')
power.legend()
show(fig)'''),
        code('''print(' rpm    stand N.m   model N.m   stand W   model W')
for row_rpm, row_torque, row_watts in APC20X10E_CURVE[::4]:
    w = row_rpm * TWO_PI / 60.0
    near = np.argmin(np.abs(rpm[:peak] - row_rpm))
    print('%5d   %8.2f   %8.2f   %8.0f   %8.0f'
          % (row_rpm, row_torque, APC20x10E.k * w * w, row_watts, electrical[near]))
worst = 0.0
for row_rpm, row_torque, _watts in APC20X10E_CURVE:
    w = row_rpm * TWO_PI / 60.0
    worst = max(worst, abs(APC20x10E.k * w * w / row_torque - 1.0))
print('fit: k %.3e N.m/(rad/s)^2, worst row %.1f %% off a pure square' % (APC20x10E.k, 100.0 * worst))'''),
    ),
]

RESULTS = [
    code('''errs = [abs(r[5]) for r in rows]
rms = math.sqrt(sum(e * e for e in errs) / len(errs))
settled = [r for r in rows if abs(r[1] - 600.0) < 1.0 and not math.isnan(r[4])]
tach = [r[3] - r[4] for r in settled]
top_rpm = max(r[2] for r in rows)
top_row = APC20X10E_CURVE[-1]
print('1. stepper     ring %.2f deg peak to peak at about %.0f Hz after a 90 deg move, the spring says %.1f Hz;'
      % (max(ys) - min(ys), ring_hz, spring_hz))
print('               shaft %.2f deg from the detent against a command of 90.0; read at %.0f Hz'
      % (level, len(ring) / ring[-1][0]))
print('2. servo       no load: shaft %.2f deg, error %.2f; 0.02 N.m: sag %.2f deg measured, %.2f by the spring'
      % (landed, 45.0 - landed, sag, spring_sag))
print('               corrected: shaft %.2f deg, error %.2f, swing %.2f deg; holding torque 2.0 A x Kt %.4f = %.3f N.m'
      % (back, 45.0 - back, s.swing, kt, 2.0 * kt))
print('3. sensorless  %d passes at %.1f Hz; top %.0f rpm mechanical = %.0f rad/s electrical at %d pole pairs'
      % (len(rows), len(rows) / rows[-1][0], top_rpm, top_rpm * RAD_S_PER_RPM * poles, poles))
print('               observer error %.3f deg electrical rms, worst %.3f = %.4f deg mechanical; zero at constant speed'
      % (rms, max(errs), max(errs) / poles))
print('               observer minus shaft speed, settled: %+.1f rpm mean, %.1f rms over %d passes; at rest %.2f deg apart on the pole pitch'
      % (sum(tach) / len(tach), math.sqrt(sum(x * x for x in tach) / len(tach)), len(tach), offset))
print('4. identified  %-6s %-10s %-10s %-10s %s' % ('', 'truth', 'probe', 'no probe', 'probe -> none'))
for name, truth in TRUTH:
    print('               %-6s %-10.5g %-10.5g %-10.5g %+.1f %% -> %+.1f %%'
          % (name, truth, got[name], got2[name],
             100.0 * (got[name] / truth - 1.0), 100.0 * (got2[name] / truth - 1.0)))
print('               trusted with the probe %s, without %s; condition %.2e and %.2e'
      % ([n for n, _ in TRUTH if got['trusted'][n]], [n for n, _ in TRUTH if got2['trusted'][n]],
         got['condition'], got2['condition']))
print('5. propeller   k %.3e N.m/(rad/s)^2, worst row %.1f %% off a square; at %d rpm %.2f N.m needs %.1f A at Kt %.4f'
      % (APC20x10E.k, 100.0 * worst, top_row[0], top_row[1], top_row[1] / KT_NM_PER_AMP, KT_NM_PER_AMP))
print('               the model reached %.0f rpm at 37 V, iq peak %.1f A, v_sat %.0f %% of the run; the board 100 A rated, the motor %.1f A burst'
      % (rpm.max(), abs(prop['iq']).max(), 100.0 * prop['v_sat'].mean(), RATINGS['i_max']))
print('               lambda %.5f Wb from %d KV at %d pole pairs; Kt = 1.5 P lambda = %.4f N.m/A'
      % (motor.lam, RATINGS['kv'], motor.poles, 1.5 * motor.poles * motor.lam))'''),
    md('- The servo closes once per move, not per pass: a 25 Hz loop pumps the 30 Hz ring '
       'until poles slip (FINDINGS 2026-09-24).\n- With the probe R, Ld, lambda are trusted, R 63 % high '
       '(the discretisation); Lq not. Without it, lambda alone.\n- Out of voltage short of '
       '6717 rpm: Lq drops 11 of 21.4 V at 46 A.\n- 100 A inverter before 112.5 A motor.'),
]

BENCH = ('The ring against `sqrt(Kt I P / J)` gives the real J. The servo needs a magnet, '
         'velocity a commissioned record.')

REFERENCES = [
    ('host/coaxial/control/motion.py', 'the three verbs: the slew, the soft energize, the ring-aware measurement, the fault read a pass'),
    ('host/coaxial/control/loop.py', 'the blocks on one bus: ramp, probe, speed loop, current loop, machine, and `identify`'),
    ('host/coaxial/model/sysid.py', 'the least squares behind `identify`: two equations a sample, an error bar per column'),
    ('host/coaxial/model/motor.py', 'the 5230SL as the sheet gives it, the propeller and its 22-row curve, `Kt = 1.5 P lambda`'),
    ('host/coaxial/simulated/drive/', 'the stand-in\'s rotor, the spring under HOLD, and the PLL lag that stands in for the observer'),
    ('host/tests/test_sensorless.py', 'the verbs pinned on the stand-in, the dangerous paths included: a load past the holding torque, a trip mid-spin'),
    ('host/tools/sim/observer_run.py', 'the firmware\'s own observer, run on the host, and the crossover it computes'),
    ('docs/FINDINGS.md', 'the stand-in\'s 2 A hold, 2026-09-24: zeta 0.0013, a 30 Hz ring a 25 Hz loop pumps'),
]

