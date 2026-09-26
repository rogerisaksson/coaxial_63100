"""A machine's cycle, data-oriented: the bus in static arrays, pure steps in a table, one edge.

    f = frame('w_target', 'omega_hat', 'w_ref', 'iq_ref', rows=len(nodes))
    e = edge(nodes, 'drive', ins=('omega_hat',), outs=('iq_ref',))
    table = ((slew, state('Slew', rate=150.0), ('dt', 'w_target'), 'w_ref'),
             (pi, state('PI', kp=0.01, limit=2.0), ('dt', 'w_ref', 'omega_hat'), 'iq_ref'))
    log = run((e,), table, f, cycles=500, period=0.004)

A frame is one structured array: a field a channel, a row a node. A step is `step(s, u) -> y`:
`s` its own rows, `u` its inputs a row each - numbers in, numbers out, nothing else touched, every
row at once. Only `poll` and `push` meet a board: a frame lost on the wire leaves its row as it
was, counted in `lost`, never raised into a step. The steps are machine.parts' step for step,
over rows (tests/test_cyclic.py); ctrl.c is the same table in C.
"""
import math
import os
import time
from dataclasses import dataclass

# One thread: OpenBLAS's pool is ~499 MB of commit a process on a laptop with no page file.
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np  # noqa: E402
from numpy.typing import NDArray  # noqa: E402

from machine.controller import flat  # noqa: E402

# -- memory ----------------------------------------------------------------------------------


def frame(*channels, rows=1):
    """The cycle's memory: `t` and `dt` first, then a float64 field a channel, a row a node."""
    names = ('t', 'dt') + tuple(c for c in channels if c not in ('t', 'dt'))
    return np.zeros(rows, np.dtype([(name, 'f8') for name in names]))


def state(kind, rows=1, **params):
    """A step's own memory: `kind`'s fields, `params` on every row, at rest."""
    s = np.zeros(rows, STEPS[kind][1])
    for name, value in params.items():
        s[name] = value
    if kind == 'SpeedKalman':
        s['p'] = s['r']
    return s


# -- logic: the parts, over rows -------------------------------------------------------------

GAIN = np.dtype([('k', 'f8')])
SLEW = np.dtype([('rate', 'f8'), ('y', 'f8')])
WRAP = np.dtype([('zero', 'f8')])
LOW_PASS = np.dtype([('tau', 'f8'), ('y', 'f8')])
SPEED_KALMAN = np.dtype([(n, 'f8') for n in ('kt', 'j', 'b', 'q', 'r', 'w', 'p', 'primed')])
PI = np.dtype([(n, 'f8') for n in ('kp', 'ki', 'limit', 'x')])
ANGLE_HOLD = np.dtype([(n, 'f8') for n in ('poles', 'theta0', 'ki', 'trim', 'most', 'x', 'at')])
DIRECT = np.dtype([('limit', 'f8')])
SPEED_PI = np.dtype([(n, 'f8') for n in ('hz', 'limit', 'kt', 'j', 'b', 'load_k', 'scale', 'x',
                                         'was')])


def gain(s: NDArray, u: NDArray) -> NDArray:
    """u: dt, x."""
    return s['k'] * u[1]


def slew(s: NDArray, u: NDArray) -> NDArray:
    """u: dt, x. y follows x at most `rate` a second."""
    most = s['rate'] * u[0]
    s['y'] += np.clip(u[1] - s['y'], -most, most)
    return s['y']


def wrap(s: NDArray, u: NDArray) -> NDArray:
    """u: dt, x. Degrees from `zero`, folded into -180..180."""
    return (u[1] - s['zero'] + 180.0) % 360.0 - 180.0


def low_pass(s: NDArray, u: NDArray) -> NDArray:
    """u: dt, x. First order, `tau` s."""
    s['y'] += u[0] / (s['tau'] + u[0]) * (u[1] - s['y'])
    return s['y']


def speed_kalman(s: NDArray, u: NDArray) -> NDArray:
    """u: dt, measured, command. Predicts on w' = (kt iq - b w) / j, corrects on the measurement;
    the first seen is taken whole."""
    dt, measured, command = u[0], u[1], u[2]
    w = s['w'] + dt * (s['kt'] * command - s['b'] * s['w']) / s['j']
    p = s['p'] + s['q'] * dt
    k = p / (p + s['r'])
    first = s['primed'] == 0.0
    s['w'] = np.where(first, measured, w + k * (measured - w))
    s['p'] = np.where(first, s['p'], p * (1.0 - k))
    s['primed'] = 1.0
    return s['w']


def pi(s: NDArray, u: NDArray) -> NDArray:
    """u: dt, setpoint, measured. Clamped to +/-limit, the integrator held while clamped."""
    e = u[1] - u[2]
    raw = s['kp'] * e + s['x']
    y = np.clip(raw, -s['limit'], s['limit'])
    s['x'] += np.where(y == raw, s['ki'] * e * u[0], 0.0)
    return y


def angle_hold(s: NDArray, u: NDArray) -> NDArray:
    """u: dt, setpoint, measured. A joint's degrees as the drive's hold angle, rad electrical."""
    s['x'] = np.clip(s['x'] + s['ki'] * (u[1] - u[2]) * u[0], -s['trim'], s['trim'])
    s['at'] += np.clip(u[1] + s['x'] - s['at'], -s['most'], s['most'])
    return s['theta0'] + np.radians(s['at']) * s['poles']


def direct(s: NDArray, u: NDArray) -> NDArray:
    """u: dt, setpoint. The setpoint within +/-limit."""
    return np.clip(u[1], -s['limit'], s['limit'])


def speed_pi(s: NDArray, u: NDArray) -> NDArray:
    """u: dt, setpoint, measured, accel (NaN: the setpoint's own slope), held. iq from w, the
    mechanical pole cancelled, drag fed forward, the integrator held while clamped or held."""
    dt, held = u[0], u[4]
    sp, pv = u[1] * s['scale'], u[2] * s['scale']
    slope = np.divide(sp - s['was'], dt, out=np.zeros_like(sp), where=dt != 0.0)
    accel = np.where(np.isnan(u[3]), slope, u[3] * s['scale'])
    s['was'] = sp
    w0 = math.tau * s['hz']
    err = sp - pv
    damp = s['b'] + 2.0 * s['load_k'] * np.abs(sp)
    ff = (s['j'] * accel + s['b'] * sp + s['load_k'] * sp * np.abs(sp)) / s['kt']
    raw = w0 * s['j'] / s['kt'] * err + s['x'] + ff
    y = np.clip(raw, -s['limit'], s['limit'])
    s['x'] += np.where((y == raw) & (held == 0.0), w0 * damp / s['kt'] * err * dt, 0.0)
    return y


#: The kinds, as ctrl.c's table has them: a name, its step, its memory.
STEPS = {'Gain': (gain, GAIN), 'Slew': (slew, SLEW), 'Wrap': (wrap, WRAP),
         'LowPass': (low_pass, LOW_PASS), 'SpeedKalman': (speed_kalman, SPEED_KALMAN),
         'PI': (pi, PI), 'AngleHold': (angle_hold, ANGLE_HOLD), 'Direct': (direct, DIRECT),
         'SpeedPI': (speed_pi, SPEED_PI)}


# -- the edge --------------------------------------------------------------------------------


@dataclass(slots=True)
class Edge:
    """Where the cycle meets the boards: a read and a writer a row, the (field, key) pairs each
    carries; whether a row's last poll answered, and the frames lost on it."""
    reads: tuple
    writers: tuple
    ins: tuple
    outs: tuple
    fresh: NDArray
    lost: NDArray


def edge(nodes, module, ins=(), outs=()):
    """Each node's `module` a row: `ins` read into the frame, `outs` written from it; a key, or
    a (field, key) pair where the frame names it otherwise."""
    rows = list(nodes)

    def pairs(spec):
        return tuple((s, s) if isinstance(s, str) else tuple(s) for s in spec)

    return Edge(tuple(n.modules[module].read for n in rows),
                tuple(n.modules[module].writer for n in rows), pairs(ins), pairs(outs),
                np.zeros(len(rows), bool), np.zeros(len(rows), np.int64))


def poll(e: Edge, f: NDArray):
    """1 and 2: every row's read into memory. A row that did not answer keeps its last."""
    for r, read in enumerate(e.reads):
        try:
            got = flat(read() or {})
            for field, key in e.ins:
                f[field][r] = got[key]
            e.fresh[r] = True
        except Exception:
            e.fresh[r] = False
            e.lost[r] += 1


def push(e: Edge, f: NDArray):
    """4: memory onto the bus, a row's outputs in one write."""
    if not e.outs:
        return
    for r, writer in enumerate(e.writers):
        try:
            writer.write(**{key: float(f[field][r]) for field, key in e.outs})
        except Exception:
            e.lost[r] += 1


# -- the flow --------------------------------------------------------------------------------


def cycle(edges, table, f, buffers):
    """One pass: poll, the steps in table order - each its inputs copied into its buffer, its
    output into the frame - push."""
    for e in edges:
        poll(e, f)
    for (step, s, ins, out), u in zip(table, buffers):
        for i, name in enumerate(ins):
            u[i] = f[name] if isinstance(name, str) else name
        f[out] = step(s, u)
    for e in edges:
        push(e, f)


def run(edges, table, f, cycles, period, clock=time.monotonic, sleep=time.sleep):
    """`cycles` passes `period` s apart, the frame after each logged - a structured array,
    (cycles, rows). Every buffer is laid before the first pass."""
    log = np.zeros((cycles, f.size), f.dtype)
    buffers = tuple(np.zeros((len(ins), f.size)) for _step, _s, ins, _out in table)
    start = last = clock()
    for k in range(cycles):
        now = clock()
        f['t'], f['dt'] = now - start, now - last
        last = now
        cycle(edges, table, f, buffers)
        log[k] = f
        sleep(max(0.0, period - (clock() - now)))
    return log
