#!/usr/bin/env python3
"""Monte Carlo over the firmware's FOC loop, one process per core.

THE C IS THE FIRMWARE'S, through `tests/test_drive_core.py`'s bench: the
current loop, the injection demodulator, the rotor observer, the dead-time
table, against `drive_model.c` with a plant drawn around `PLATINUM_5230SL`
and the stage in `coaxial.inverter`. A host speed loop (`coaxial.loop`,
the same law the notebook runs) closes over the observer's own speed.

A run: injection finds the rotor from a random error, a raised cosine to
`TOP` of the link's no-load speed, a hold, a descent to rest under
injection again. Its cost is one number, `sigma_theta + speed_err +
10 trip`: the truth angle error in radians rms after the lock, the
speed error over the top speed rms, and a stage drop. The plant is never
what the controller was told, so the cost is a robustness figure.

    python tools/montecarlo.py                   # 43 V, a small search
    python tools/montecarlo.py --vdc 63 --candidates 32 --draws 16
"""
import argparse
import concurrent.futures
import ctypes
import math
import os
import random
import sys
import time

sys.path.insert(0, __file__.rsplit('tools', 1)[0])
sys.path.insert(0, os.path.join(__file__.rsplit('tools', 1)[0], 'tests'))

from coaxial import inverter, sensorless                         # noqa: E402
from coaxial.loop import Signals, SpeedLoop                      # noqa: E402
from coaxial.motor import APC20x10E, PLATINUM_5230SL             # noqa: E402
import test_drive_core as H                                      # noqa: E402
from test_modbus_core import build, find_cc                      # noqa: E402

TS = inverter.TS
TWO_PI = sensorless.TWO_PI
VDC_SWEEP = (23.0, 33.0, 43.0, 53.0, 63.0)
TICK = 40                  # periods per speed-loop step: 1.25 kHz
TOP = 0.5                  # of the link's no-load speed the profile reaches
I_MAX, I_TRIP, I_H_MAX = 60.0, 100.0, 5.0
LOST = 0.35                # rad of truth angle error that is a lost rotor

#: The search box: name -> (lo, hi, log). n_inj is 1, 2 or 4.
KNOBS = {'bw_i': (300.0, 2500.0, True),     # current loop, Hz
         'f_pll': (20.0, 400.0, True),      # observer, Hz at the injection rate
         'zeta': (0.5, 1.2, False),
         'v_inj': (0.02, 0.2, True),        # of Vdc/sqrt3
         'n_inj': (1, 4, True),
         'w_lo': (50.0, 800.0, True),       # rad/s electrical
         'w_ratio': (1.5, 4.0, True),       # w_hi over w_lo
         'bw_w': (1.0, 20.0, True)}         # speed loop, Hz

_LIB = None


def library():
    """Build once in the parent; the workers only load."""
    return build(find_cc(), H.SOURCES, [os.path.join(H.DRIVE, 'Inc')],
                 name='drivecore_mc')[0]


def _load(path):
    # A job must hold only builtin floats: one numpy scalar smuggled into a
    # knob makes every worker import numpy at once, and 16 OpenBLAS buffer
    # pools spiking together took a 24 GB machine down. The env var tames
    # the pool if one gets through anyway.
    os.environ['OPENBLAS_NUM_THREADS'] = '1'
    global _LIB
    _LIB = ctypes.CDLL(path)


def wrap(x):
    return (x + math.pi) % TWO_PI - math.pi


def draw(seed, vdc, motor=PLATINUM_5230SL):
    """A plant the controller was not told about. Copper to 125 C on R,
    the size-class estimates' quarter on L, a saliency from barely there to
    1.5, the dead time either side of the commissioned one, the AFE at its
    measured floor, the rotor within the injection's pull-in of theta_hat
    (the polarity pulse's job is done)."""
    u = random.Random(seed).uniform
    ld = motor.ld * u(0.75, 1.25)
    t_dead = inverter.T_DEAD * u(0.8, 1.2)
    return {'r': motor.r * u(0.9, 1.45), 'ld': ld, 'lq': ld * u(1.05, 1.5),
            'lambda': motor.lam * u(0.9, 1.1), 'pole_pairs': float(motor.poles),
            'sat': motor.sat, 'i_sat': motor.i_sat,
            'j': motor.j * u(0.5, 2.0), 'b': motor.b * u(0.5, 2.0), 'load': 0.0,
            'v_dt': inverter.dead_time_volts(vdc, t_dead),
            'i_knee': inverter.knee_amps(vdc, t_dead) * u(0.7, 1.4),
            'vdc': vdc, 'noise': u(*inverter.NOISE_A), 'theta0': u(0.0, TWO_PI),
            'sub': 4.0, 'theta_err0': u(-1.2, 1.2), 'k_prop': APC20x10E.k * u(0.8, 1.2)}


def design(knobs, vdc, motor=PLATINUM_5230SL):
    """The firmware's parameters from the knobs and what it believes."""
    w = TWO_PI * knobs['bw_i']
    n = int(knobs['n_inj'])
    v_inj = min(knobs['v_inj'] * vdc / math.sqrt(3.0),
                2.0 * motor.ld * I_H_MAX / (n * TS))
    wn = TWO_PI * knobs['f_pll']
    t_upd = 2 * n * TS
    step, table = inverter.dt_table(vdc)
    p = {'r': motor.r, 'ld': motor.ld, 'lq': motor.lq, 'lambda': motor.lam,
         'pole_pairs': float(motor.poles), 'kp': motor.ld * w, 'ki': motor.r * w,
         'l1': 2.0 * knobs['zeta'] * wn * t_upd, 'l2': wn * wn * t_upd,
         'inj_volts': v_inj, 'inj_periods': float(n), 'inj_phase': 0.0,
         'eps_gain': sensorless.demod_gain(v_inj, TS, motor.ld, motor.lq),
         'i_max': I_MAX, 'i_trip': I_TRIP, 'v_frac': inverter.V_FRAC, 'sign': 1.0,
         'w_lo': knobs['w_lo'], 'w_hi': knobs['w_lo'] * knobs['w_ratio'],
         'dt_step': step}
    p.update(('dt%d' % k, v) for k, v in enumerate(table))
    return p


def top_speed(vdc, motor=PLATINUM_5230SL):
    """rad/s electrical: TOP of what the link can hold against lambda."""
    return TOP * inverter.V_FRAC * vdc / math.sqrt(3.0) / motor.lam


def profile(t, w_top, t_lock=0.15, rise=0.8, hold=0.4, fall=1.0):
    """(w_ref, a_ref) at t: rest, a raised cosine up, a hold, a line down."""
    if t < t_lock:
        return 0.0, 0.0
    if t < t_lock + rise:
        ph = math.pi * (t - t_lock) / rise
        return (w_top * 0.5 * (1.0 - math.cos(ph)),
                w_top * 0.5 * math.pi / rise * math.sin(ph))
    if t < t_lock + rise + hold:
        return w_top, 0.0
    if t < t_lock + rise + hold + fall:
        return w_top * (1.0 - (t - t_lock - rise - hold) / fall), -w_top / fall
    return 0.0, 0.0


def run_job(job):
    """One run. `job`: vdc, knobs, seed; `bemf_only` descends with the
    injection off from the hold on, and reports where the rotor was lost."""
    vdc, knobs, seed = job['vdc'], job['knobs'], job['seed']
    motor = PLATINUM_5230SL
    plant = draw(seed, vdc)
    d = H.Drive(_LIB, TS)
    try:
        d.model_params(**{k: plant[k] for k in H.MODEL})
        d.source(True)
        params = design(knobs, vdc)
        d.params(**params)
        d.setpoints(id_ref=0.0, iq_ref=0.0)
        d.set_theta(plant['theta0'] + plant['theta_err0'])
        d.mode(H.SENSORLESS, enabled=False, powered=False)
        speed = SpeedLoop(knobs['bw_w'], I_MAX, motor, load=APC20x10E)
        s = Signals()
        w_top = top_speed(vdc)
        t_hold = 0.15 + 0.8
        model = {k: plant[k] for k in H.MODEL}
        sq_th = sq_w = n = 0
        worst = lock = 0.0
        trip = False
        lost_at = None
        bemf_only = job.get('bemf_only', False)
        every = int(job.get('trace', 0))    # speed-loop ticks per trace row
        trace = [] if every else None
        for k in range(int(round(2.5 / TS))):
            t = k * TS
            if k % TICK == 0:
                st, ms = d.state(), d.model_state()
                err = wrap(st['theta_hat'] - ms['theta'])
                w_ref, a_ref = profile(t, w_top)
                if bemf_only and t >= t_hold and params['inj_volts']:
                    params.update(inj_volts=0.0, w_lo=0.0, w_hi=0.0)
                    d.params(**params)
                if t >= 0.15:
                    sq_th += err * err
                    sq_w += ((ms['omega'] - w_ref) / w_top) ** 2
                    n += 1
                    worst = max(worst, abs(err))
                    if bemf_only and t > t_hold and lost_at is None and (
                            abs(err) > LOST or ms['omega'] < 0.5 * w_ref):
                        lost_at = ms['omega'] / motor.poles * 60.0 / TWO_PI
                elif k == 0:
                    lock = abs(err)
                if trace is not None and (k // TICK) % every == 0:
                    trace.append((t, w_ref, ms['omega'], st['omega_hat'],
                                  err, st['iq'], st['ih']))
                s.w_ref, s.a_ref = w_ref / motor.poles, a_ref / motor.poles
                s.w = st['omega_hat'] / motor.poles
                speed(s, TICK * TS)
                d.setpoints(iq_ref=s.iq_ref)
                wm = ms['omega'] / motor.poles
                model['load'] = plant['k_prop'] * wm * abs(wm)
                d.model_params(**model)
            if d.step_virtual()[0]:
                trip = True
                break
        st = d.state()
        sigma = math.sqrt(sq_th / n) if n else math.pi
        speed_err = math.sqrt(sq_w / n) if n else 1.0
        out = {'vdc': vdc, 'seed': seed, 'trip': trip, 'sigma_theta': sigma,
               'worst_theta': worst, 'speed_err': speed_err,
               'lock0': lock, 'lock': abs(wrap(st['theta_hat'] - d.model_state()['theta'])),
               'i_peak': d.window()['i_peak'], 'i_h': params['inj_volts'] * int(knobs['n_inj']) * TS / (2.0 * motor.ld),
               'cost': sigma + speed_err + 10.0 * trip,
               'min_rpm': lost_at if lost_at is not None else 0.0}
        out.update(knobs)
        out.update(('plant_' + k, plant[k]) for k in ('r', 'ld', 'lq', 'lambda', 'j', 'v_dt', 'noise', 'theta_err0'))
        out['saliency'] = plant['lq'] / plant['ld']
        if trace is not None:
            out['trace'] = trace
        return out
    finally:
        d.close()


def pool(workers=None, lib=None):
    """One process per core, each holding the library; open it once for a
    session - a pool per round respawned 61 interpreters three times and
    the third spawn died of commit charge on a machine with no page-file
    headroom. Windows caps a pool at 61."""
    return concurrent.futures.ProcessPoolExecutor(
        max_workers=workers or min(os.cpu_count(), 61), initializer=_load,
        initargs=(lib or library(),))


def sweep(pool, jobs, progress=True):
    """Every job through the pool; a DataFrame back."""
    import pandas as pd
    rows, t0, seen = [], time.perf_counter(), 0
    for row in pool.map(run_job, jobs, chunksize=4):
        rows.append(row)
        seen += 1
        if progress and seen % max(1, len(jobs) // 10) == 0:
            print('  %5d / %d runs, %.0f s' % (seen, len(jobs), time.perf_counter() - t0))
    return pd.DataFrame(rows)


def candidates(count, seed, box=None):
    """Latin hypercube over the box (default KNOBS); n_inj snaps to 1, 2, 4."""
    box = box or KNOBS
    rng = random.Random(seed)
    cols = {}
    for name, (lo, hi, log) in box.items():
        slots = [(k + rng.random()) / count for k in range(count)]
        rng.shuffle(slots)
        if log:
            vals = [lo * (hi / lo) ** u for u in slots]
        else:
            vals = [lo + (hi - lo) * u for u in slots]
        if name == 'n_inj':
            vals = [min((1, 2, 4), key=lambda c, v=v: abs(math.log(c) - math.log(v))) for v in vals]
        cols[name] = vals
    return [{k: cols[k][i] for k in box} for i in range(count)]


def around(best, shrink=0.5):
    """A box `shrink` as wide (in log where log) about `best`, inside KNOBS."""
    box = {}
    for name, (lo, hi, log) in KNOBS.items():
        c = float(best[name])
        if log:
            half = (hi / lo) ** (shrink / 2.0)
            box[name] = (max(lo, c / half), min(hi, c * half), True)
        else:
            half = (hi - lo) * shrink / 2.0
            box[name] = (max(lo, c - half), min(hi, c + half), False)
    return box


def score(frame):
    """Per (vdc, candidate): the cost's mean and 90th percentile, robust =
    their sum."""
    keys = ['vdc'] + list(KNOBS)
    g = frame.groupby(keys)['cost']
    out = g.mean().to_frame('mean')
    out['p90'] = g.quantile(0.9)
    out['runs'] = g.size()
    out['robust'] = out['mean'] + out['p90']
    return out.reset_index()


def search(pool, vdcs=VDC_SWEEP, candidates_n=48, draws=16, refine=24, seed=1):
    """Two rounds per link voltage: a hypercube over KNOBS, then a half-box
    about each round's best three. Returns (best per vdc, every run)."""
    import pandas as pd
    first = candidates(candidates_n, seed)
    jobs = [{'vdc': v, 'knobs': c, 'seed': 1000 * i + s}
            for v in vdcs for i, c in enumerate(first) for s in range(draws)]
    print('round 1: %d runs' % len(jobs))
    runs = sweep(pool, jobs)
    runs['round'] = 1
    sc = score(runs)
    jobs = []
    for v in vdcs:
        top = sc[sc.vdc == v].nsmallest(3, 'robust')
        for j, (_, row) in enumerate(top.iterrows()):
            box = around({k: row[k] for k in KNOBS})
            for i, c in enumerate(candidates(refine // 3, seed + 7 * j + int(v), box)):
                jobs += [{'vdc': v, 'knobs': c, 'seed': 1000 * (100 + 10 * j + i) + s}
                         for s in range(draws)]
    print('round 2: %d runs' % len(jobs))
    second = sweep(pool, jobs)
    second['round'] = 2
    runs = pd.concat([runs, second], ignore_index=True)
    sc = score(runs)
    best = sc.loc[sc.groupby('vdc')['robust'].idxmin()].reset_index(drop=True)
    return best, runs


def verify(pool, best, draws=48, seed=5):
    """The best of each link against fresh draws, with and without the
    injection through the descent: the cost's spread, and the speed the
    back-EMF alone loses the rotor at."""
    jobs = []
    for _, row in best.iterrows():
        knobs = {k: float(row[k]) for k in KNOBS}
        for s in range(draws):
            jobs.append({'vdc': float(row.vdc), 'knobs': knobs, 'seed': 7000 + 100 * seed + s})
            jobs.append({'vdc': float(row.vdc), 'knobs': knobs, 'seed': 7000 + 100 * seed + s, 'bemf_only': True})
    frame = sweep(pool, jobs)
    frame['bemf_only'] = [j.get('bemf_only', False) for j in jobs]
    return frame


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--vdc', type=float, nargs='+', default=[43.0])
    ap.add_argument('--candidates', type=int, default=8)
    ap.add_argument('--draws', type=int, default=4)
    ap.add_argument('--refine', type=int, default=6)
    ap.add_argument('--workers', type=int, default=None)
    args = ap.parse_args()
    import pandas as pd
    pd.set_option('display.width', 200)
    t0 = time.perf_counter()
    with pool(args.workers) as p:
        best, runs = search(p, args.vdc, args.candidates, args.draws, args.refine)
    print(best.round(4).to_string(index=False))
    print('%d runs in %.0f s' % (len(runs), time.perf_counter() - t0))
    return 0


if __name__ == '__main__':
    sys.exit(main())
