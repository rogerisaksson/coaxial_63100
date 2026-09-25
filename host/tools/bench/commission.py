"""Commission a motor: characterise, budget the injection, set gains, decide, verify, print the line.

It characterises the AFE, the inverter and the motor.

    python tools/bench/commission.py --simulated                 # the stand-in
    python tools/bench/commission.py --step afe                  # no switching
    python tools/bench/commission.py --arm --step all --iq 0.5   # the lot, armed
    python tools/bench/commission.py --arm --step deadtime,l_map --json out.json

`--arm` is what lets a step set MOE: gates.on(bypass_sto=True,
ignore_interlock=True), the unmodified bench board's combination. Without
it every switching step refuses and says so. On this bench AFE_ON high
unpowers the gate drivers, so the switching steps run dry - the gate
supply is read and printed first, and a step that saw no current says
`measured: False` rather than a number.
"""
import argparse
import json
import math
import os

from coaxial import Coaxial63100
from coaxial.control.commission import Commissioning
from coaxial.errors import RigError
from machine.modes import HARDWARE, SIMULATED

STEPS = ('afe', 'sample_point', 'offsets', 'gains_afe', 'sign', 'deadtime',
         'l_map', 'flux', 'budget', 'gains', 'decide', 'verify')


def run_steps(c, steps, iq, seconds):
    """The named steps, in the order the pipeline has them."""
    calls = {
        'afe': lambda: c.afe_noise(zero_vector=c.arm is not None),
        'sample_point': c.sample_point_scan,
        'offsets': c.offsets,
        'gains_afe': c.gain_mismatch,
        'sign': c.sign_check,
        'deadtime': c.deadtime,
        'l_map': c.l_map,
        'flux': c.flux,
        'budget': c.budget,
        'gains': c.gains,
        'decide': c.decide,
        'verify': lambda: c.verify(iq=iq, seconds=seconds),
    }
    for name in STEPS:
        if name not in steps:
            continue
        try:
            got = calls[name]()
        except RigError as exc:
            print('%-13s refused: %s' % (name, exc))
            continue
        print('%-13s %s' % (name, _brief(name, got)))


def _budget_line(got):
    """The injection budget's choice, or that nothing fits."""
    c = got['choice']
    if c is None:
        return 'nothing fits the constraints'
    return '%s AFE: f_inj %.0f Hz  V %.2f  SNR %.1f dB  sigma_theta %.1f deg  (%s)' % (
        got['afe'], c['f_inj_hz'], c['v_inj'], c['snr_db'],
        math.degrees(c['sigma_theta']), c['limited_by'])


def _brief(name, got):
    """One line per step, the numbers that matter."""
    if name == 'afe':
        rows = got.get('zero_vector') or got['gates_off']
        return 'sigma_i %.4f A  ENOB %.1f  %s  isr %.1f us' % (
            got['sigma_i'], got['enob'],
            ' '.join('%s %.1f' % (n[-1], rows[n]['sd']) for n in rows
                     if n.startswith('Phase')),
            got['latency']['isr_cost_us'])
    if name == 'sample_point':
        return 'best CCR5 %d of %d (was %d)' % (got['best'], got['period'], got['was'])
    if name == 'offsets':
        return ' '.join('%s %+d%s' % (n[-1], r['offset_raw'],
                                      ' SUSPECT' if r['suspect'] else '')
                        for n, r in got.items())
    if name in ('gains_afe', 'sign', 'deadtime', 'l_map', 'flux') \
            and not got.get('measured'):
        return 'not measured - %s' % got.get('why', 'no current')
    if name == 'gains_afe':
        return 'mismatch %.2f %%' % got['mismatch_pct']
    if name == 'sign':
        return 'sign %+d (id %.3f A)' % (got['sign'], got['id'])
    if name == 'deadtime':
        return 'R %.4f ohm  V_dt %.3f V  knee %.2f A  residual %.3f V' % (
            got['r'], got['v_dt'], got['i_knee'], got['residual_volts'])
    if name == 'l_map':
        return 'Ld %.1f uH  Lq %.1f uH  dL/L %.3f' % (
            got['ld'] * 1e6, got['lq'] * 1e6, got['dl_over_l'])
    if name == 'flux':
        return 'lambda %.5f V.s  load angle %.2f rad' % (
            got['lambda'], got['load_angle'])
    if name == 'budget':
        return _budget_line(got)
    if name == 'gains':
        k = got['kalman'] or {}
        return 'iloop %.0f Hz kp %.3f ki %.1f  PLL %.0f Hz zeta %.2f  crossover %.0f rpm' % (
            got['loop']['bw_hz'], got['loop']['kp'], got['loop']['ki'],
            k.get('wn_hz', 0.0), k.get('zeta', 0.0), got['crossover']['rpm'])
    if name == 'decide':
        return '%s (SNR %.1f dB against %.0f)' % (
            got['method'], got['snr_db'], got['threshold_db'])
    if name == 'verify':
        lb = got['ljung_box']
        return '%s  sigma_theta %.1f deg  innovation %s (Q %.1f / %.1f)  omega_hat %.0f  fault %s' % (
            got['method'], got['sigma_theta_deg'],
            'white' if lb['white'] else 'NOT white', lb['q'], lb['threshold'],
            got['omega_hat'], got['fault'])
    return str(got)[:100]


def supply_line(supply):
    volts = '%.2f' % supply['volts'] if supply['volts'] is not None else '?'
    return 'gate supply %s V - %s' % (volts, 'powered' if supply['powered']
                                      else 'UNPOWERED: switching steps run dry')


def arguments(argv):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--port', default='COM4')
    p.add_argument('--simulated', action='store_true')
    p.add_argument('--arm', action='store_true',
                   help='authorise gates.on(bypass_sto=True, ignore_interlock=True)')
    p.add_argument('--step', default='all', help='comma list of %s, or all' % ', '.join(STEPS))
    p.add_argument('--iq', type=float, default=0.5, help='A, for verify')
    p.add_argument('--seconds', type=float, default=1.0, help='verify run')
    p.add_argument('--i-h-max', type=float, default=1.0, help='HF current ceiling, A')
    p.add_argument('--f-min', type=float, default=0.0, help='lowest f_inj, Hz')
    p.add_argument('--rated-rpm', type=float, default=3000.0)
    p.add_argument('--json', help='write every result here')
    args = p.parse_args(argv)
    args.steps = STEPS if args.step == 'all' else tuple(s.strip() for s in args.step.split(','))
    unknown = [s for s in args.steps if s not in STEPS]
    if unknown:
        p.error('unknown step %s; pick from %s' % (unknown, ', '.join(STEPS)))
    args.arm = dict(bypass_sto=True, ignore_interlock=True) if args.arm else None
    return args


def main(argv=None):
    args = arguments(argv)
    with Coaxial63100(port=args.port, execution_mode=SIMULATED if args.simulated else HARDWARE, power_afe=True) as rig, \
            Commissioning(rig, arm=args.arm, log=print, i_h_max=args.i_h_max,
                          f_min_hz=args.f_min, rated_rpm=args.rated_rpm) as c:
        print('%s  fs %.0f Hz' % (rig, c.fs))
        print(supply_line(c.gate_supply()))
        run_steps(c, args.steps, args.iq, args.seconds)
        report = c.report()

    print('\n' + report['line'])
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as out:
            json.dump(report, out, indent=1, default=str)
        print('written', os.path.abspath(args.json))


if __name__ == '__main__':
    main()
