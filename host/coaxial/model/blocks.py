"""The control loops as blocks on one bus, closing around `motor.pmsm`."""
import math
import os
import random

# numpy's OpenBLAS commits 32 MB a core on import: a scratch buffer per
# worker thread, never touched, charged against the host's commit limit.
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy

from machine.parts import SpeedPI
from motor import sysid
from motor.pmsm import TWO_PI, Motor, Parameters

SQRT3 = math.sqrt(3.0)


class Signals:

    """The bus. Slots, not a dict: a typo fails instead of vanishing."""

    __slots__ = ('t', 'w_ref', 'a_ref', 'w', 'w_e', 'theta',
                 'id_ref', 'iq_ref', 'id', 'iq', 'vd', 'vq', 'v_sat')

    def __init__(self):
        for name in self.__slots__:
            setattr(self, name, 0.0)


class Block:

    """One step of work on the bus: `__call__(s, dt)` mutates `s`."""

    def __rshift__(self, other):
        return Chain((self,)) >> other


class Chain(Block):

    """Blocks in order. `run` steps them and records the bus."""

    def __init__(self, blocks):
        self.blocks = tuple(blocks)

    def __rshift__(self, other):
        more = other.blocks if isinstance(other, Chain) else (other,)
        return Chain(self.blocks + tuple(more))

    def __call__(self, s, dt):
        for block in self.blocks:
            block(s, dt)

    def run(self, seconds, dt, every=1):
        """The chain for `seconds` at `dt`, every `every`th bus recorded."""
        s = Signals()
        rows = {name: [] for name in Signals.__slots__}
        for k in range(int(round(seconds / dt))):
            s.t = k * dt
            self(s, dt)
            if k % every == 0:
                for name in Signals.__slots__:
                    rows[name].append(getattr(s, name))
        return {name: numpy.asarray(v) for name, v in rows.items()}


class Ramp(Block):

    """w_ref: a raised cosine to `top` over `rise`, and back down over the
    next `rise`.
    """

    def __init__(self, top, rise):
        self.top, self.rise = top, rise

    def __call__(self, s, dt):
        t, up = s.t, self.rise
        if t < 2.0 * up:
            ph = math.pi * (t if t < up else 2.0 * up - t) / up
            s.w_ref = self.top * 0.5 * (1.0 - math.cos(ph))
            sign = 1.0 if t < up else -1.0
            s.a_ref = sign * self.top * 0.5 * math.pi / up * math.sin(ph)
        else:
            s.w_ref = s.a_ref = 0.0


class Probe(Block):

    """d-axis excitation, `amps` at `hz`: torque-free, so the speed loop
    never sees it, and the one thing that lets Ld out of a fit - without
    did/dt the inductance column is R's (`motor.sysid`).
    """

    def __init__(self, amps, hz):
        self.amps, self.hz = amps, hz

    def __call__(self, s, dt):
        s.id_ref = self.amps * math.sin(TWO_PI * self.hz * s.t)


class SpeedLoop(Block):

    """iq_ref from w_ref: `machine.parts.SpeedPI` on the bus."""

    def __init__(self, hz, limit, motor, load=None):
        self.law = SpeedPI.of(hz, limit, motor, load)

    def __call__(self, s, dt):
        s.iq_ref = self.law.step(dt, setpoint=s.w_ref, measured=s.w, accel=s.a_ref,
                                 held=s.v_sat)['command']


class CurrentLoop(Block):

    """vd, vq from the current error: kp = L w0 and ki = R w0 per axis, the
    speed cross-terms fed forward, the pair clamped to link/sqrt(3) as a
    vector; while the clamp acts the integrators hold: drive.c's
    conditional integration.
    """

    def __init__(self, hz, motor, vdc):
        w0 = TWO_PI * hz
        self.kpd, self.kpq = motor.ld * w0, motor.lq * w0
        self.ki = motor.r * w0
        self.m, self.vmax = motor, vdc / SQRT3
        self.xd = self.xq = 0.0

    def __call__(self, s, dt):
        ed, eq = s.id_ref - s.id, s.iq_ref - s.iq
        vd = self.kpd * ed + self.xd - s.w_e * self.m.lq * s.iq
        vq = self.kpq * eq + self.xq + s.w_e * (self.m.ld * s.id + self.m.lam)
        norm = math.hypot(vd, vq)
        s.v_sat = norm > self.vmax
        if s.v_sat:
            vd, vq = vd * self.vmax / norm, vq * self.vmax / norm
        else:
            self.xd += self.ki * ed * dt
            self.xq += self.ki * eq * dt
        s.vd, s.vq = vd, vq


class Plant(Block):

    """`motor.pmsm.Motor` behind the bus: vd/vq become duties at the
    rotor's own angle, one PWM period advances, and what comes back out
    carries `noise` amps of gaussian on each current - the AFE's floor,
    on what the loop sees, never on the motor itself.
    """

    def __init__(self, params, vdc, load=None, noise=0.0, seed=2, **kw):
        kw.setdefault('k_load', load.k if load else 0.0)
        self.motor = Motor.of(params, **kw)
        self.vdc, self.noise = vdc, noise
        self.rng = random.Random(seed)

    def __call__(self, s, dt):
        m = self.motor
        g = (lambda: self.rng.gauss(0.0, self.noise)) if self.noise else float
        s.id, s.iq = m.id + g(), m.iq + g()
        s.w_e, s.theta = m.omega, m.theta
        s.w = m.omega / m.p
        # Half a period of angle advance: the vector is held in the stator
        # frame while the rotor turns w*dt through it, so aiming at the middle
        # makes the mean dq voltage the commanded one.
        ahead = m.theta + 0.5 * m.omega * dt
        c, sn = math.cos(ahead), math.sin(ahead)
        va, vb = s.vd * c - s.vq * sn, s.vd * sn + s.vq * c
        v = (va, -0.5 * va + 0.5 * SQRT3 * vb, -0.5 * va - 0.5 * SQRT3 * vb)
        shift = 0.5 * (max(v) + min(v))
        m.advance([0.5 + (x - shift) / self.vdc for x in v], self.vdc, dt)


def identify(run, poles, **kw):
    """`motor.sysid` over a run: (Parameters, the fit record)."""
    got = sysid.identify(run['vd'], run['vq'], run['id'], run['iq'],
                         run['w'] * poles, run['t'], **kw)
    fit = Parameters(
        name='identified', r=got['r'], ld=got['ld'], lq=got['lq'],
        lam=got['lam'], poles=poles, measured=False,
        source='least squares over a simulated run - %d samples, '
               'condition %.1e' % (got['samples'], got['condition']))
    return fit, got
