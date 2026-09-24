#!/usr/bin/env python3
"""Monte Carlo over the firmware's FOC loop, one process per core.

The C is the firmware's, through `tools/cores/drive.py`: the current loop,
the injection demodulator, the rotor observer, the dead-time table, against
`drive_model.c` with a plant drawn around `PLATINUM_5230SL` and the stage in
`coaxial.model.inverter`. A host speed loop (`coaxial.model.blocks`, the
same law the notebook runs) closes over the observer's own speed.

A run: injection finds the rotor from a random error, a raised cosine to
`TOP` of the link's no-load speed, a hold, a descent to rest under
injection again. Its cost is one number, `sigma_theta + speed_err +
10 trip`: the truth angle error in radians rms after the lock, the
speed error over the top speed rms, and a stage drop. The plant is never
what the controller was told, so the cost is a robustness figure.

    python tools/sim/montecarlo.py                   # 43 V, a small search
    python tools/sim/montecarlo.py --vdc 63 --candidates 32 --draws 16
"""
import argparse
import concurrent.futures
import ctypes
import math
import os
import random
import sys
import time

from coaxial.model.blocks import Signals, SpeedLoop
from coaxial.model import inverter, sensorless
from motor.catalog import PLATINUM_5230SL
from motor.loads import APC20x10E, Propeller
from motor.pmsm import TWO_PI, Parameters
from tools.cores import drive as H
from tools.cores.build import build, find_cc

TS = inverter.TS
VDC_SWEEP = (23.0, 33.0, 43.0, 53.0, 63.0)
TICK = 40                  # periods per speed-loop step: 1.25 kHz
TOP = 0.5                  # of the link's no-load speed the profile reaches
I_MAX, I_TRIP, I_H_MAX = 60.0, 100.0, 5.0
LOST = 0.35                # rad of truth angle error that is a lost rotor
#: The run's clock, s: the lock before the profile starts, the raised
#: cosine up, the hold at the top, the line down, and the whole run.
#: The statistics start after the lock; a `bemf_only` run drops the
#: injection once the top is reached.
LOCK_S, RISE_S, HOLD_S, FALL_S, RUN_S = 0.15, 0.8, 0.4, 1.0, 2.5
#: Under this fraction of the reference speed, past the top, the rotor
#: counts as lost; and what a stage trip adds to the cost.
LOST_SPEED = 0.5
TRIP_COST = 10.0
#: The plant's draws that ride along in the row, as plant_<name>.
PLANT_COLUMNS = ('r', 'ld', 'lq', 'lambda', 'j', 'v_dt', 'noise', 'theta_err0')

#: The search box: name -> (lo, hi, log). n_inj is 1, 2 or 4.
KNOBS = {'bw_i': (300.0, 2500.0, True),     # current loop, Hz
         'f_pll': (20.0, 400.0, True),      # observer, Hz at the injection rate
         'zeta': (0.5, 1.2, False),
         'v_inj': (0.02, 0.2, True),        # of Vdc/sqrt3
         'n_inj': (1, 4, True),
         'w_lo': (50.0, 800.0, True),       # rad/s electrical
         'w_ratio': (1.5, 4.0, True),       # w_hi over w_lo
         'bw_w': (1.0, 20.0, True)}         # speed loop, Hz

class _Worker:
    """The drive core's library, one per process: the parent builds it,
    `_load` opens it in each worker, `hold` hands an open one in."""
    lib = None


def hold(lib):
    """Use an already-open library - the test bench's, in process."""
    _Worker.lib = lib


def library():
    """Build once in the parent; the workers only load."""
    return build(find_cc(), H.SOURCES, [os.path.join(H.DRIVE, 'inc')],
                 name='drivecore_mc')[0]


def _load(path):
    # A job must hold only builtin floats: one numpy scalar smuggled into a
    # knob makes every worker import numpy at once, and 16 OpenBLAS buffer
    # pools spiking together took a 24 GB host down.
    os.environ['OPENBLAS_NUM_THREADS'] = '1'
    hold(ctypes.CDLL(path))


def wrap(x):
    return (x + math.pi) % TWO_PI - math.pi


def draw(seed, vdc, motor=PLATINUM_5230SL, k_prop=APC20x10E.k):
    """A plant the controller was not told about."""
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
            'sub': 4.0, 'theta_err0': u(-1.2, 1.2), 'k_prop': k_prop * u(0.8, 1.2)}


def design(knobs, vdc, motor=PLATINUM_5230SL, i_max=I_MAX, i_trip=I_TRIP,
           i_h_max=I_H_MAX):
    """The firmware's parameters from the knobs and what it believes."""
    w = TWO_PI * knobs['bw_i']
    n = int(knobs['n_inj'])
    v_inj = min(knobs['v_inj'] * vdc / math.sqrt(3.0),
                2.0 * motor.ld * i_h_max / (n * TS))
    wn = TWO_PI * knobs['f_pll']
    t_upd = 2 * n * TS
    step, table = inverter.dt_table(vdc)
    p = {'r': motor.r, 'ld': motor.ld, 'lq': motor.lq, 'lambda': motor.lam,
         'pole_pairs': float(motor.poles), 'kp': motor.ld * w, 'ki': motor.r * w,
         'l1': 2.0 * knobs['zeta'] * wn * t_upd, 'l2': wn * wn * t_upd,
         'inj_volts': v_inj, 'inj_periods': float(n), 'inj_phase': 0.0,
         'eps_gain': sensorless.demod_gain(v_inj, TS, motor.ld, motor.lq),
         'i_max': i_max, 'i_trip': i_trip, 'v_frac': inverter.V_FRAC, 'sign': 1.0,
         'w_lo': knobs['w_lo'], 'w_hi': knobs['w_lo'] * knobs['w_ratio'],
         'dt_step': step}
    p.update(('dt%d' % k, v) for k, v in enumerate(table))
    return p


def top_speed(vdc, motor=PLATINUM_5230SL):
    """rad/s electrical: TOP of what the link can hold against lambda."""
    return TOP * inverter.V_FRAC * vdc / math.sqrt(3.0) / motor.lam


def profile(t, w_top, t_lock=LOCK_S, rise=RISE_S, hold=HOLD_S, fall=FALL_S):
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


class Run:
    """One job's simulation: the firmware's drive stepping the model at TS,
    the host speed loop ticking every TICK steps, the statistics taken
    after the lock.
    """

    def __init__(self, job):
        self.vdc, self.knobs, self.seed = job['vdc'], job['knobs'], job['seed']
        self.motor = (Parameters(**job['motor']) if 'motor' in job
                      else PLATINUM_5230SL)
        i_max = job.get('i_max', I_MAX)
        i_trip = job.get('i_trip', I_TRIP)
        k_prop = job.get('k_prop', APC20x10E.k)
        self.plant = draw(self.seed, self.vdc, self.motor, k_prop)
        self.model = {k: self.plant[k] for k in H.MODEL}
        self.d = H.Drive(_Worker.lib, TS)
        self.d.model_params(**self.model)
        self.d.source(True)
        self.params = design(self.knobs, self.vdc, self.motor, i_max, i_trip,
                             job.get('i_h_max', I_H_MAX))
        self.d.params(**self.params)
        self.d.setpoints(id_ref=0.0, iq_ref=0.0)
        self.d.set_theta(self.plant['theta0'] + self.plant['theta_err0'])
        self.d.mode(H.SENSORLESS, enabled=False, powered=False)
        self.speed = SpeedLoop(self.knobs['bw_w'], i_max, self.motor,
                               load=Propeller(k_prop))
        self.signals = Signals()
        self.w_top = top_speed(self.vdc, self.motor)
        self.sq_th = self.sq_w = self.n = 0
        self.worst = self.lock = 0.0
        self.trip = False
        self.lost_at = None
        self.bemf_only = job.get('bemf_only', False)
        self.every = int(job.get('trace', 0))    # speed-loop ticks per trace row
        self.trace = [] if self.every else None

    def go(self):
        """The whole run, or until the stage trips."""
        for k in range(int(round(RUN_S / TS))):
            if k % TICK == 0:
                self.tick(k)
            if self.d.step_virtual()[0]:
                self.trip = True
                return

    def tick(self, k):
        """One speed-loop step: the observer's error against the truth,
        the injection dropped at the top when the job says so, the
        statistics, the trace row, and the loop's own setpoint."""
        t = k * TS
        st, ms = self.d.state(), self.d.model_state()
        err = wrap(st['theta_hat'] - ms['theta'])
        w_ref, a_ref = profile(t, self.w_top)
        if self.bemf_only and t >= LOCK_S + RISE_S and self.params['inj_volts']:
            self.params.update(inj_volts=0.0, w_lo=0.0, w_hi=0.0)
            self.d.params(**self.params)
        self.tally(k, t, err, ms['omega'], w_ref)
        if self.trace is not None and (k // TICK) % self.every == 0:
            self.trace.append((t, w_ref, ms['omega'], st['omega_hat'],
                               err, st['iq'], st['ih']))
        s, poles = self.signals, self.motor.poles
        s.w_ref, s.a_ref = w_ref / poles, a_ref / poles
        s.w = st['omega_hat'] / poles
        self.speed(s, TICK * TS)
        self.d.setpoints(iq_ref=s.iq_ref)
        wm = ms['omega'] / poles
        self.model['load'] = self.plant['k_prop'] * wm * abs(wm)
        self.d.model_params(**self.model)

    def tally(self, k, t, err, omega, w_ref):
        """The first tick's error is the lock; after it every tick
        counts, and past the top a `bemf_only` run notes the speed the
        rotor was lost at."""
        if k == 0:
            self.lock = abs(err)
        if t < LOCK_S:
            return
        self.sq_th += err * err
        self.sq_w += ((omega - w_ref) / self.w_top) ** 2
        self.n += 1
        self.worst = max(self.worst, abs(err))
        lost = (self.bemf_only and t > LOCK_S + RISE_S and self.lost_at is None
                and (abs(err) > LOST or omega < LOST_SPEED * w_ref))
        if lost:
            self.lost_at = omega / self.motor.poles * 60.0 / TWO_PI

    def result(self):
        """The row: the cost, and everything that shaped it."""
        st = self.d.state()
        sigma = math.sqrt(self.sq_th / self.n) if self.n else math.pi
        speed_err = math.sqrt(self.sq_w / self.n) if self.n else 1.0
        out = {'vdc': self.vdc, 'seed': self.seed, 'trip': self.trip,
               'bemf_only': self.bemf_only, 'sigma_theta': sigma,
               'worst_theta': self.worst, 'speed_err': speed_err,
               'lock0': self.lock,
               'lock': abs(wrap(st['theta_hat'] - self.d.model_state()['theta'])),
               'i_peak': self.d.window()['i_peak'],
               'i_h': (self.params['inj_volts'] * int(self.knobs['n_inj']) * TS
                       / (2.0 * self.motor.ld)),
               'cost': sigma + speed_err + TRIP_COST * self.trip,
               'min_rpm': self.lost_at if self.lost_at is not None else 0.0}
        out.update(self.knobs)
        out.update(('plant_' + k, self.plant[k]) for k in PLANT_COLUMNS)
        out['saliency'] = self.plant['lq'] / self.plant['ld']
        if self.trace is not None:
            out['trace'] = self.trace
        return out


def run_job(job):
    """One run."""
    run = Run(job)
    try:
        run.go()
        return run.result()
    finally:
        run.d.close()


def pool(workers=None, lib=None):
    """One process per core, each holding the library; open it once for a
    session - a pool per round respawned 61 interpreters three times and
    the third spawn died of commit charge on a host with no page-file
    headroom.
    """
    return concurrent.futures.ProcessPoolExecutor(
        max_workers=workers or min(os.cpu_count() or 1, 61), initializer=_load,
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
    about each round's best three.
    """
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
        top = pd.DataFrame(sc[sc.vdc == v]).nsmallest(3, 'robust')
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
    return sweep(pool, jobs)


def main():
    ap = argparse.ArgumentParser(description=(__doc__ or '').split('\n')[0])
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
