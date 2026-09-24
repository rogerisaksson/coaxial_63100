#!/usr/bin/env python3
"""The target's rotor observer, against a motor, on this machine.

The C is the firmware's: `drive/` is hardware-free. `tools/cores/build.py`
builds it and the harness with the host gcc; `tools/cores/drive.py` lists the
sources and wraps the library through ctypes. The question is not whether it
works but how hard it can be driven before it does not.

A settled run at a commanded q current, with the rotor's speed coming from
torque against friction and a propeller rather than from a setpoint, and
the observer given only the phase currents. The angle error reported is
against the model's own truth, which the observer never sees.

Every constant comes from `coaxial.model.motor`, where the 5230SL
carries `measured=False`: R, Ld, Lq and J are estimates from a size class.
The saliency Lq/Ld is what an injection observer lives on and it is the
least trustworthy number here, so read a standstill result as arithmetic
about a plausible machine, not about the one on the bench.

    python tools/sim/observer_run.py                    # the 5230SL, speed sweep
    python tools/sim/observer_run.py --bandwidth        # how aggressive is too
    python tools/sim/observer_run.py --motor bench      # the stand-in's machine
"""
import argparse
import ctypes
import math
import os
import sys

from coaxial.model import sensorless
from coaxial.model.motor import APC20x10E, BENCH_MOTOR, Motor, PLATINUM_5230SL
from tools.cores import drive as H
from tools.cores.build import build, find_cc

MOTORS = {'5230': PLATINUM_5230SL, 'bench': BENCH_MOTOR}

#: 10S LiPo, the link Hobbywing's thrust table was taken on: back-EMF at the
#: top of that table is 20.4 V peak against the 21.4 V an amplitude-invariant
#: SVM can reach off 37 V. A 24 V link cannot turn this motor that fast, and
#: the run would report a voltage limit as an observer failure.
VDC = 37.0

#: Sized from `sensorless`, not chosen: the current loop an order above the
#: fastest electrical frequency, the PLL at the Kalman fixed point.
I_LOOP_HZ = 2000.0
PLL_HZ = 332.0
V_INJ = 2.0


def spin(params, iq, seconds=0.35, vdc=VDC, pll_hz=PLL_HZ, noise=0.02,
         prop=True, lib=None):
    """Hold `iq` from standstill and report what the observer did."""
    m = Motor.of(params, theta=2.0,
                 k_load=APC20x10E.k if prop else 0.0)
    x = sensorless.crossover(lam=params.lam, r=params.r, i_max=60.0,
                             v_dt_residual=0.5, pole_pairs=params.poles)
    d = H.Drive(lib)
    #: Two figures: the handover from injection to back-EMF is a transient,
    #: the tracking above it a steady state. One worst case conflates them,
    #: a hard start read as "the observer is bad at 40 A". `steady` counts
    #: only once the rotor is past w_hi.
    worst, steady, reached = [0.0], [0.0], [False]
    settled = int(0.05 / H.TS)

    def track(k, dd, mm):
        if k <= settled:
            return
        err = abs(H.wrap_pi(dd.state()['theta_hat'] - mm.theta))
        worst[0] = max(worst[0], err)
        if abs(mm.omega) > 1.3 * x['omega_e']:
            reached[0] = True
            steady[0] = max(steady[0], err)
    try:
        d.params(r=params.r, ld=params.ld, lq=params.lq,
                 **{'lambda': params.lam},
                 pole_pairs=float(params.poles),
                 inj_volts=V_INJ, inj_periods=1,
                 eps_gain=H.eps_gain(V_INJ, m.ld0, m.lq),
                 i_max=80.0, i_trip=140.0,
                 w_lo=0.7 * x['omega_e'], w_hi=1.3 * x['omega_e'],
                 **H.loop_gains(params.r, params.ld, I_LOOP_HZ),
                 **H.pll_gains(pll_hz, 2 * H.TS))
        d.setpoints(id_ref=0.0, iq_ref=0.0)
        d.set_theta(2.0 + 0.3)
        d.mode(H.SENSORLESS)
        H.run(d, m, 0.05, vdc=vdc, noise=noise)      # injection finds the axis

        # Polarity pulses: the pi ambiguity injection cannot resolve.
        theta_hat = d.state()['theta_hat']
        d.setpoints(pol_volts=6.0, pol_periods=8, pol_gap=40)
        d.mode(H.POLARITY)
        H.run(d, m, (2 * 8 + 2 * 40 + 4) * H.TS, vdc=vdc, noise=noise)
        s = d.state()
        flipped = s['pol_neg'] > s['pol_pos']
        d.set_theta(theta_hat + math.pi if flipped else theta_hat)
        d.setpoints(pol_volts=0.0, pol_periods=0, pol_gap=0)
        d.mode(H.SENSORLESS)
        H.run(d, m, 0.02, vdc=vdc, noise=noise)      # settle after the pulse
        d.setpoints(iq_ref=iq)
        tripped = H.run(d, m, seconds, vdc=vdc, noise=noise, watch=track)
        s = d.state()
        wm = m.omega / params.poles
        return {
            'iq': iq, 'tripped': tripped,
            'rpm': wm * 60.0 / (2.0 * math.pi),
            'f_e': m.omega / (2.0 * math.pi),
            'omega': m.omega, 'omega_hat': s['omega_hat'],
            'worst_rad': worst[0],
            'worst_deg': math.degrees(worst[0]),
            'steady_deg': math.degrees(steady[0]),
            # Lock is judged on the steady error and on the rotor turning the
            # way it was asked to.
            'reached': reached[0],
            # A rotor that never passed w_hi has no steady error to be judged
            # on: 50 A reported 0.00 degrees because it stalled at 27 rpm and
            # the metric was never sampled once.
            'locked': (reached[0] and steady[0] < 0.5 and not tripped
                       and m.omega > 0.0),
            'crossover_rpm': x['rpm'], 'flipped': flipped,
        }
    finally:
        d.close()


def sweep_speed(params, lib):
    print('%-5s %8s %8s %8s %9s %9s  %s'
          % ('iq A', 'rpm', 'f_e Hz', 'PWM/rev', 'start deg', 'run deg',
             'lock'))
    for iq in (5.0, 10.0, 20.0, 35.0, 45.0, 50.0, 58.0):
        g = spin(params, iq, lib=lib)
        print('%-5.0f %8.0f %8.1f %8.1f %9.2f %9.2f  %s'
              % (iq, g['rpm'], g['f_e'],
                 50000.0 / g['f_e'] if g['f_e'] > 1.0 else float('inf'),
                 g['worst_deg'], g['steady_deg'],
                 'yes' if g['locked'] else
                 ('stall' if not g['reached'] else 'LOST')
                 + (' (trip)' if g['tripped'] else '')))


def sweep_bandwidth(params, lib):
    """How aggressive is too aggressive: the PLL swept, the rest held."""
    print('the PLL at %.0f A of q current, everything else held\n' % 35.0)
    print('%-9s %9s %9s %9s  %s'
          % ('PLL Hz', 'start deg', 'run deg', 'rpm', 'lock'))
    for hz in (60.0, 150.0, 332.0, 600.0, 1200.0, 2500.0):
        g = spin(params, 35.0, pll_hz=hz, lib=lib)
        print('%-9.0f %9.2f %9.2f %9.0f  %s'
              % (hz, g['worst_deg'], g['steady_deg'], g['rpm'],
                 'yes' if g['locked'] else 'LOST'))


def main():
    ap = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    ap.add_argument('--motor', choices=sorted(MOTORS), default='5230')
    ap.add_argument('--bandwidth', action='store_true',
                    help='sweep the PLL instead of the q current')
    args = ap.parse_args()

    params = MOTORS[args.motor]
    print(params)
    print('propeller %r, link %.0f V, current loop %.0f Hz\n'
          % (APC20x10E, VDC, I_LOOP_HZ))

    cc = find_cc()
    if cc is None:
        print('no host C compiler; setup.ps1 installs one')
        return 1
    path, warnings = build(cc, H.SOURCES,
                           [os.path.join(H.DRIVE, 'inc')], name='drivecore')
    if warnings:
        print('BUILD WARNINGS: %s' % '; '.join(warnings[:3]))
    lib = ctypes.CDLL(path)
    if args.bandwidth:
        sweep_bandwidth(params, lib)
    else:
        sweep_speed(params, lib)
    return 0


if __name__ == '__main__':
    sys.exit(main())
