"""Her latent space, stepped: rows blended into one vector, her rhythm turned, her modes asked.

    st = latent.state(rest)                       # (STATE,) at her rest vector (`modal.rest`)
    tab = latent.table(wire, begin, rest)         # (n, TABLE): rows (`ortho.compiled`) from `begin` s
    out = latent.step(st, tab, now, dt, out)      # (OUT,): what her modes are asked this pass

Flat arrays, laid once, and arithmetic on them: no branch, no state but `st`. A row whose time
has come lays what it says over where her vector was going (a mask), and her vector goes there
in a straight line over the row's blend. Her rhythm is one angle turning at its frequency; a
mode's reference is its target and its amplitude's sine at that angle and the mode's phase, with
the rate and the acceleration that makes. Out go each mode's reference, stiffness and damping
and her rhythm's phases: what a mode is on her body, and what her drives can give of it, are
downstream's (`machine.modal`, `machine.wbc`). One row a tick. In a synthesizer's terms - a
sequencer of notes, an oscillator, envelopes, the generator's out - docs/architecture/synth.md.
"""
import math
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine import curves, ortho

N, P = ortho.N, ortho.P
#: A row's parameters as slices (`ortho`'s layout): a mode each, then four numbers.
Z, K, D, AMP, PHASE = (slice(at, at + N) for at in (ortho.Z, ortho.K, ortho.D, ortho.AMP, ortho.PHASE))
GAIN, HZ, ZMP, YAW, DUCK, MANIFOLD, SLIDE = ortho.GAIN, ortho.HZ, ortho.ZMP, ortho.YAW, ortho.DUCK, ortho.MANIFOLD, ortho.SLIDE
#: Her state: her vector as it is, where it came from and where it goes; when it set out, how
#: long it takes, her rhythm's angle.
NOW, FROM, TO = (slice(i * P, (i + 1) * P) for i in range(3))
T0, BLEND, THETA = 3 * P, 3 * P + 1, 3 * P + 2
#: A blend's rates at its start and its end, a param each; whether the row after begins where
#: this blend ends - a sample of a curve, not a step.
V0, V1 = slice(3 * P + 3, 4 * P + 3), slice(4 * P + 3, 5 * P + 3)
CONTIG = 5 * P + 3
#: The blend's shape: its share of the way at u, a quintic's coefficients on u..u^5 (SHAPES).
SHAPE = slice(5 * P + 4, 5 * P + 9)
STATE = 5 * P + 9
#: The most rows her table holds (wbc_imp.h's WBC_WAIT_ROWS): a take's frames are thinned to fit.
ROWS = 512
#: A table's row: what it says, which of it is said (1) or left (0), when it comes, its blend,
#: whether a curve's sample, whether it waits for a seat or a bed to bear her (`ortho.CONTACT`),
#: whether it latches her vector where she is (`ortho.LATCH`), its blend's shape (SHAPES), whether it waits
#: while her centre of mass sinks (`ortho.RISE`).
VALS, SAID = slice(0, P), slice(P, 2 * P)
AT, LERP, CURVE, CONTACT, LATCH = 2 * P, 2 * P + 1, 2 * P + 2, 2 * P + 3, 2 * P + 4
SHAPED = slice(2 * P + 5, 2 * P + 10)
RISES = 2 * P + 10
TABLE = 2 * P + 11
#: What a pass gives: each mode's reference, its rate and acceleration, its stiffness (N/m) and
#: damping (N s/m), its rhythm's phase (rad) and amplitude; her rhythm's rate (rad/s), her
#: ZMP's polygon as told (her soles' weight laid on it after, `observer.weighed`), her yaw momentum's limit; and each mode's curve's own acceleration (BEND),
#: the acceleration's less the rhythm's: the touch filter rebuilds the acceleration from it;
#: her sidechain as said (DUCKED, `ortho.DUCK`); what bears her as told (MANIFOLDED, `ortho.MANIFOLD`) and
#: her slide over it (SLID, `ortho.SLIDE`).
REF, RATE, ACC, KS, DS, PSI, SWAY = (slice(i * N, (i + 1) * N) for i in range(7))
TURNS, HELD, YAWED = 7 * N, 7 * N + 1, 7 * N + 2
BEND = slice(7 * N + 3, 8 * N + 3)
DUCKED = slice(8 * N + 3, 8 * N + 6)
MANIFOLDED = slice(8 * N + 6, 8 * N + 6 + len(ortho.MANIFOLDS))
SLID = MANIFOLDED.stop
OUT = SLID + 1
#: A row as the wire carries it.
WIRE = np.dtype([('id', '<u4'), ('at', '<u4'), ('blend', '<u4'), ('flags', '<u4'), ('p', '<f4', (P,))])
#: A row's time is met within this, s.
EDGE_S = 1e-9


#: Each blend's shape (`ortho.SHAPES`), a quintic's coefficients on u..u^5 (`curves.QUINTICS`).
SHAPES = np.array([curves.QUINTICS[name] for name in ortho.SHAPES])


def state(rest):
    """Her state at `rest` (P,), going nowhere, her rhythm's angle 0."""
    st = np.zeros(STATE)
    st[NOW] = st[FROM] = st[TO] = rest
    st[SHAPE] = SHAPES[0]
    return st


def table(wire, begin, rest):
    """The rows of `wire` (`ortho.ROW`) as a table, each `begin` s past its own time; a row
    flagged `ortho.REST` says all of `rest`, one flagged `ortho.CURVE` is a curve's sample."""
    rows = np.frombuffer(wire, WIRE)
    vals = rows['p'].astype(float)
    said = ~np.isnan(vals)
    back = (rows['flags'] & ortho.REST).astype(bool)[:, None]
    tab = np.empty((len(rows), TABLE))
    tab[:, VALS] = np.where(back, rest, np.where(said, vals, 0.0))
    tab[:, SAID] = said | back
    tab[:, AT] = begin + 1e-3 * rows['at']
    tab[:, LERP] = 1e-3 * rows['blend']
    tab[:, CURVE] = (rows['flags'] & ortho.CURVE).astype(bool)
    tab[:, CONTACT] = ((rows['flags'] & ortho.CONTACT) > 0) + 2.0 * ((rows['flags'] & ortho.PALMED) > 0) - (
        (rows['flags'] & ortho.SOLES) > 0) * ((rows['flags'] >> ortho.SHARE_BIT) & 1023) / 1e3
    tab[:, LATCH] = (rows['flags'] & ortho.LATCH).astype(bool)
    tab[:, SHAPED] = SHAPES[(rows['flags'] >> ortho.SHAPE_BIT) & 15]
    tab[:, RISES] = (rows['flags'] & ortho.RISE).astype(bool)
    return tab


#: No row: nothing said, never due.
_NONE = np.zeros((1, TABLE))
_NONE[0, AT] = math.inf


def _shaped(st, u):
    """(its share of the way, its slope, its bend) of the blend's shape (SHAPE) at `u`."""
    c1, c2, c3, c4, c5 = (float(c) for c in st[SHAPE])
    return (u * (c1 + u * (c2 + u * (c3 + u * (c4 + u * c5)))),
            c1 + u * (2.0 * c2 + u * (3.0 * c3 + u * (4.0 * c4 + u * (5.0 * c5)))),
            2.0 * c2 + u * (6.0 * c3 + u * (12.0 * c4 + u * (20.0 * c5))))


def _rate(st, u, span):
    """A blend's rate at `u` of its `span`: its shape's slope while it blends, a contiguous curve's
    (CONTIG) the Hermite's from FROM to TO, V0 to V1."""
    line = (st[TO] - st[FROM]) / span * _shaped(st, u)[1] * (u < 1.0)
    hermite = ((6.0 * u * u - 6.0 * u) * st[FROM] + (3.0 * u * u - 4.0 * u + 1.0) * (span * st[V0])
               + (6.0 * u - 6.0 * u * u) * st[TO] + (3.0 * u * u - 2.0 * u) * (span * st[V1])) / span
    return line + st[CONTIG] * (hermite - line)


def step(st, tab, now, dt, out):
    """`out` this pass: the row of `tab` whose time `now` is laid, `st` blended on a tick `dt`,
    her rhythm turned. A text's row blends linearly to its target. A row flagged a curve's sample
    (`ortho.CURVE`, a take's frame) that the next begins on as it ends (CONTIG) blends as a
    Hermite curve (a text's row begun on as it ends, rise_chair's lean, curved too, bent its rise
    with the arriving rate's span and she fell), leaving
    at the rate she had (V0: the rate given, not the Hermite's end slope of a row blended
    linearly - the push's first row left a held crouch at 4.6 a second, 709 a second squared)
    and arriving at the rate the row after asks (V1, Catmull-Rom's: the way from FROM to that
    row over both spans), so her rate and acceleration are the curve's own: blended linearly, a
    sit's rows braked 4 m/s^2 and her height, a/kp = 7 cm behind, met the seat at 1.2 m/s where
    the human's hips arrive still; the texts' blends curved, seven held scenes fell. A curve's
    row begins where the last ended (FROM its TO) and runs on through the tick its row ends in,
    dt/span past its end at most: begun from the reference at the tick that met it, 20 ms rows
    on a 3 ms tick held one tick in seven and leaped in rate at every row, the push's
    acceleration -228 to +98 for its -36. A blend begins at the tick that met its row (T0):
    begun at the row's own time, up to a tick earlier, the chair's rise fell in 1 of 2 runs -
    a knife edge, not a law."""
    hit = ((tab[:, AT] <= now + EDGE_S) & (tab[:, AT] > now - dt + EDGE_S)).astype(float)
    came = np.minimum(hit.sum(), 1.0)
    span = np.maximum(st[BLEND], dt)
    leaving = _rate(st, np.clip((now - st[T0]) / span, 0.0, 1.0), span)
    st[V0] += came * (leaving - st[V0])
    st[FROM] += came * (st[NOW] + st[CONTIG] * (st[TO] - st[NOW]) - st[FROM])
    st[TO] += np.minimum(hit @ tab[:, SAID], 1.0) * (hit @ tab[:, VALS] - st[TO])
    st[T0] += came * (now - st[T0])
    st[BLEND] += came * (hit @ tab[:, LERP] - st[BLEND])
    st[SHAPE] += came * (hit @ tab[:, SHAPED] - st[SHAPE])
    span = np.maximum(st[BLEND], dt)
    ahead = tab[:, AT] > now + EDGE_S
    has = float(ahead.any())
    rows = np.concatenate([tab, _NONE])                          # a table may be empty
    k = int(np.argmin(np.where(np.concatenate([ahead, [False]]), rows[:, AT], math.inf)))
    after = np.where(rows[k, SAID] > 0.5, rows[k, VALS], st[TO])
    st[V1] += came * (has * ((after - st[FROM]) / (span + np.maximum(rows[k, LERP], dt))) - st[V1])
    contig = has * float(abs(rows[k, AT] - (hit @ tab[:, AT] + st[BLEND])) <= dt) * float(hit @ tab[:, CURVE])
    st[CONTIG] += came * (contig - st[CONTIG])
    u = np.clip((now - st[T0] + dt) / span, 0.0, 1.0 + st[CONTIG] * dt / span)
    p = st[NOW]
    share, _slope, bent = _shaped(st, u)
    line = st[FROM] + (st[TO] - st[FROM]) * share
    curve = ((2.0 * u * u * u - 3.0 * u * u + 1.0) * st[FROM] + (u * u * u - 2.0 * u * u + u) * (span * st[V0])
             + (3.0 * u * u - 2.0 * u * u * u) * st[TO] + (u * u * u - u * u) * (span * st[V1]))
    p[:] = line + st[CONTIG] * (curve - line)
    rate = _rate(st, u, span)[Z]
    flat = (st[TO] - st[FROM]) / (span * span) * bent * (u < 1.0)
    bend = (flat + st[CONTIG] * (((12.0 * u - 6.0) * st[FROM] + (6.0 * u - 4.0) * (span * st[V0]) + (6.0 - 12.0 * u)
                                  * st[TO] + (6.0 * u - 2.0) * (span * st[V1])) / (span * span) - flat))[Z]
    turns = math.tau * p[HZ]
    st[THETA] = (st[THETA] + turns * dt) % math.tau
    psi = st[THETA] + p[PHASE]
    sway = p[GAIN] * p[AMP]
    out[REF] = p[Z] + sway * np.sin(psi)
    out[RATE] = rate + sway * turns * np.cos(psi)
    out[ACC] = bend - sway * turns * turns * np.sin(psi)
    out[KS], out[DS], out[PSI], out[SWAY] = p[K], p[D], psi, sway
    out[TURNS], out[HELD], out[YAWED] = turns, p[ZMP], p[YAW]
    out[BEND] = bend
    out[DUCKED] = p[DUCK]
    out[MANIFOLDED] = p[MANIFOLD]
    out[SLID] = p[SLIDE]
    return out

