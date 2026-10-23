"""The anti-spasm insert's numbers laid as her master bus (anti_spasm_supervisor.lisp).

    k = supervisor.insert()           # {node: {param: value}}: the master insert on her nominal, as written
    supervisor.checked(g)             # a gait's insert against the one she runs: refused by name where it differs
    laid = supervisor.bus(names, rate)        # its numbers and state, a driven joint each (wbc_bus.c)

The insert's stages on her loop, its numbers its text's: her ask's jerk and her torque command's slew
soft-clipped, the command through the tank (wbc_bus.c, at her drives' 1 kHz pass); the joints'
buffers before their stops in her step (`wrench`). Open (docs/todo/scenarios.md): the velocity gate,
the D low-pass, the damped least squares, the zero crossing's bridge, the handover, the contact
comparators. A node of the suite is her joints of its kind (NODE_KINDS).
"""
import ctypes
import functools
import math
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine import drives, wbc
from machine.errors import MachineError

#: The suite's nodes as her joints' kinds (`drives.kind`).
NODE_KINDS = {':coxofemoral': ('hip_yaw', 'hip_roll', 'hip'), ':tibiofemoral': ('knee',), ':talocrural': ('ankle',),
              ':l4-l5-lumbar': ('spine',), ':glenohumeral': ('shoulder',), ':c1-c2-atlas': ('neck', 'head')}
#: The forms the bus is, as the text must name them: (node, param) -> its value.
FORMS = {('velocity-gate', 'target-nodes'): ':all-actuated-joints', ('velocity-gate', 'curve'): ':smooth-step',
         ('derivative-channel', 'topology'): ':butterworth-2nd-order', ('derivative-channel', 'zero-phase-compensation'): False,
         ('torque-transients', 'clip-topology'): ':cubic-soft-knee',
         ('torque-transients', 'max-torque-slew-nmps'): ':drive-current-loop',
         ('torque-transients', 'max-jerk-radps3'): ':drive-current-loop',
         ('zero-crossing', 'co-contraction-bias-nm'): ':none', ('jacobian-solver', 'method'): ':damped-least-squares',
         ('jacobian-solver', 'activation-curve'): ':raised-cosine',
         ('bumpless-handover', 'continuity-order'): ':c2-acceleration-continuous',
         ('bumpless-handover', 'decay-envelope'): ':exponential-critically-damped'}
#: Her own damping over a drive's inertia, 1/s: the injected damping's unit.
POSTURE_KD = wbc.POSTURE_KD[1]


class Bus(ctypes.Structure):

    """wbc_bus_k: the insert's numbers."""

    _fields_ = [(name, ctypes.c_double) for name in ('tank_max_j', 'tank_min_j')] + [
        ('hp_b', ctypes.c_double * 3), ('hp_a', ctypes.c_double * 2), ('lp_b', ctypes.c_double * 3),
        ('lp_a', ctypes.c_double * 2)] + [
        (name, ctypes.c_double) for name in ('period_s', 'reversals', 'trip_w', 'inject', 'rollback', 'recover_s')]


class State(ctypes.Structure):

    """wbc_bus_t: the bus's state, pointing at its arrays."""

    _fields_ = [('n', ctypes.c_int)] + [(name, ctypes.c_void_p) for name in (
        'slew', 'jerk', 'qa_out', 'tau_out', 'bv', 'bt', 'sign', 'half_e', 'half_t', 'count',
        'alpha', 'given')] + [
        ('energy', ctypes.c_double), ('power', ctypes.c_double), ('primed', ctypes.c_int)]


@functools.lru_cache(maxsize=1)
def insert():
    """{node: {param: value}} of the master insert on her nominal manifold, as the suite reads it."""
    from machine import dsp
    got = dsp.read(dsp.SUITE)
    g = dsp.value(['->', 'nominal-biped-manifold', ['master-anti-spasm-insert']], got)
    if not isinstance(g, dict):
        raise MachineError('master-anti-spasm-insert: a gait, not %r' % type(g).__name__)
    return {node: g[node] for node in dsp.GUARDS.values()}


def named(node, param):
    """Whether the insert's nodes hold `param` of `node`: voicing's cover."""
    return node in insert() and param.split('.')[0] in insert()[node]


def checked(g):
    """Gait `g`'s insert against the one she runs (`insert`): a param said otherwise refused by name."""
    for node, params in insert().items():
        said = g.get(node)
        if said is not None and said != params:
            differs = sorted(p for p in set(said) | set(params) if said.get(p) != params.get(p))
            raise MachineError('refused: %s %s - her master insert says %s' % (
                node, ', '.join('%s %r' % (p, said.get(p)) for p in differs), ', '.join('%r' % params.get(p) for p in differs)))


def _formed(k):
    """The text's forms against the bus's: refused by name where it names another."""
    for (node, param), form in FORMS.items():
        if k[node][param] != form:
            raise MachineError('refused: %s.%s %r - the bus is %r' % (node, param, k[node][param], form))


def low_pass(hz, rate):
    """(b, a): a second-order Butterworth low-pass at `hz` on samples `rate` Hz apart, bilinear, prewarped."""
    c = math.tan(math.pi * hz / rate)
    norm = 1.0 / (1.0 + math.sqrt(2.0) * c + c * c)
    b0 = c * c * norm
    return (b0, 2.0 * b0, b0), (2.0 * (c * c - 1.0) * norm, (1.0 - math.sqrt(2.0) * c + c * c) * norm)


def high_pass(hz, rate):
    """(b, a): a second-order Butterworth high-pass at `hz` on samples `rate` Hz apart, bilinear, prewarped.
    A window of one band-pass biquad (Q 0.5 about 14.5 Hz) let her stride's 1.8 Hz through at -12 dB:
    her walk's own 4.5 W, the tank at its floor in 2.6 s of it."""
    c = math.tan(math.pi * hz / rate)
    norm = 1.0 / (1.0 + math.sqrt(2.0) * c + c * c)
    return (norm, -2.0 * norm, norm), (2.0 * (c * c - 1.0) * norm, (1.0 - math.sqrt(2.0) * c + c * c) * norm)


def joints_of(nodes, names):
    """The indices among `names` of the joints of the suite's `nodes`."""
    kinds = {kind for node in nodes for kind in NODE_KINDS[node]}
    return [i for i, name in enumerate(names) if drives.kind(name) in kinds]


def constants(rate):
    """The bus's numbers (Bus) of the insert's text, her drives' pass `rate` Hz."""
    k = insert()
    _formed(k)
    tank = k['energy-tank']
    detector, violation = tank['limit-cycle-detector'], tank['on-passivity-violation']
    lo, hi = detector['bandpass-window-hz']
    hp_b, hp_a = high_pass(lo, rate)
    lp_b, lp_a = low_pass(hi, rate)
    return Bus(tank_max_j=tank['tank-capacity-joules'], tank_min_j=tank['lower-depletion-bound-joules'], hp_b=hp_b, hp_a=hp_a,
               lp_b=lp_b, lp_a=lp_a,
               period_s=1.0 / lo, reversals=detector['max-consecutive-reversals'],
               trip_w=detector['trigger-power-threshold-watts'], inject=violation['damping-injection-gain'],
               rollback=10.0 ** (violation['stiffness-rollback-db'] / 20.0), recover_s=1e-3 * violation['recovery-release-ms'])


#: The bus's arrays laid once, kept by a reset.
LAID = ('slew', 'jerk')


def bus(names, rate):
    """(Bus, State, its arrays): the master bus over the driven joints `names`, laid from the insert's
    text, the tank at its initial energy."""
    from tools.cores import wbc as cores
    k, n = constants(rate), len(names)
    arrays = dict(slew=np.array([drives.slew(name) for name in names]),
                  jerk=np.array([drives.jerk(name) for name in names]), bv=np.zeros(4 * n), bt=np.zeros(4 * n), **{name: np.zeros(n) for name in (
        'qa_out', 'tau_out', 'sign', 'half_e', 'half_t', 'count', 'alpha', 'given')})
    s = State(n=n, energy=insert()['energy-tank']['initial-energy-joules'], power=0.0, primed=0)
    for name, a in arrays.items():
        setattr(s, name, cores.address(a))
    return k, s, arrays


def reset(laid):
    """The bus `laid` (`bus`'s) as at its lay: nothing given, the tank at its initial energy."""
    _k, s, arrays = laid
    for name, a in arrays.items():
        a[:] = a if name in LAID else 0.0
    s.energy, s.power, s.primed = insert()['energy-tank']['initial-energy-joules'], 0.0, 0


#: A low end before the model's: the knee's straight lock, its singularity, rad.
LOCKS = {'knee': 0.0}


def buffers(names, stops):
    """((driven index, stop, side: 1 a low end, -1 a high), ..), (width, stiffness, damping): the
    insert's buffers before the stops `stops` ({index: (lo, hi)}, her body's) of the driven joints
    `names` its nodes are, critically damped as its text says. A joint with none has none: at her
    law's room, her shoulders' rows threw a jump's landing 0.35 m on."""
    b = insert()['joint-boundary']
    k = b['boundary-repulsion-stiffness']
    rows = []
    for i in (i for i in joints_of(b['nodes'], names) if i in stops):
        lo, hi = stops[i]
        rows += [(i, LOCKS.get(drives.kind(names[i]), float(lo)), 1.0), (i, float(hi), -1.0)]
    return tuple(rows), (b['extension-soft-knee-rad'], k, 2.0 * b['boundary-damping-ratio'] * math.sqrt(k))
