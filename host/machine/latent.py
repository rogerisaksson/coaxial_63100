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
downstream's (`machine.modal`, `machine.wbc`). One row a tick.
"""
import math
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np  # noqa: E402

from machine import ortho  # noqa: E402

N, P = ortho.N, ortho.P
#: A row's parameters as slices (`ortho`'s layout): a mode each, then four numbers.
Z, K, D, AMP, PHASE = (slice(at, at + N) for at in (ortho.Z, ortho.K, ortho.D, ortho.AMP, ortho.PHASE))
GAIN, HZ, ZMP, YAW = ortho.GAIN, ortho.HZ, ortho.ZMP, ortho.YAW
#: Her state: her vector as it is, where it came from and where it goes; when it set out, how
#: long it takes, her rhythm's angle.
NOW, FROM, TO = (slice(i * P, (i + 1) * P) for i in range(3))
T0, BLEND, THETA = 3 * P, 3 * P + 1, 3 * P + 2
STATE = 3 * P + 3
#: A table's row: what it says, which of it is said (1) or left (0), when it comes, its blend.
VALS, SAID = slice(0, P), slice(P, 2 * P)
AT, LERP = 2 * P, 2 * P + 1
TABLE = 2 * P + 2
#: What a pass gives: each mode's reference, its rate and acceleration, its stiffness (N/m) and
#: damping (N s/m), its rhythm's phase (rad) and amplitude; her rhythm's rate (rad/s), her
#: ZMP's polygon, her yaw momentum's limit.
REF, RATE, ACC, KS, DS, PSI, SWAY = (slice(i * N, (i + 1) * N) for i in range(7))
TURNS, HELD, YAWED = 7 * N, 7 * N + 1, 7 * N + 2
OUT = 7 * N + 3
#: A row as the wire carries it.
WIRE = np.dtype([('id', '<u4'), ('at', '<u4'), ('blend', '<u4'), ('flags', '<u4'), ('p', '<f4', (P,))])
#: A row's time is met within this, s.
EDGE_S = 1e-9


def state(rest):
    """Her state at `rest` (P,), going nowhere, her rhythm's angle 0."""
    st = np.zeros(STATE)
    st[NOW] = st[FROM] = st[TO] = rest
    return st


def table(wire, begin, rest):
    """The rows of `wire` (`ortho.ROW`) as a table, each `begin` s past its own time; a row
    flagged `ortho.REST` says all of `rest`."""
    rows = np.frombuffer(wire, WIRE)
    vals = rows['p'].astype(float)
    said = ~np.isnan(vals)
    back = (rows['flags'] & ortho.REST).astype(bool)[:, None]
    tab = np.empty((len(rows), TABLE))
    tab[:, VALS] = np.where(back, rest, np.where(said, vals, 0.0))
    tab[:, SAID] = said | back
    tab[:, AT] = begin + 1e-3 * rows['at']
    tab[:, LERP] = 1e-3 * rows['blend']
    return tab


def step(st, tab, now, dt, out):
    """`out` this pass: the row of `tab` whose time `now` is laid, `st` blended on a tick `dt`,
    her rhythm turned."""
    hit = ((tab[:, AT] <= now + EDGE_S) & (tab[:, AT] > now - dt + EDGE_S)).astype(float)
    came = np.minimum(hit.sum(), 1.0)
    st[TO] += np.minimum(hit @ tab[:, SAID], 1.0) * (hit @ tab[:, VALS] - st[TO])
    st[FROM] += came * (st[NOW] - st[FROM])
    st[T0] += came * (now - st[T0])
    st[BLEND] += came * (hit @ tab[:, LERP] - st[BLEND])
    span = np.maximum(st[BLEND], dt)
    u = np.clip((now - st[T0] + dt) / span, 0.0, 1.0)
    p = st[NOW]
    p[:] = st[FROM] + (st[TO] - st[FROM]) * u
    turns = math.tau * p[HZ]
    st[THETA] = (st[THETA] + turns * dt) % math.tau
    psi = st[THETA] + p[PHASE]
    sway = p[GAIN] * p[AMP]
    out[REF] = p[Z] + sway * np.sin(psi)
    out[RATE] = (st[TO][Z] - st[FROM][Z]) / span * (u < 1.0) + sway * turns * np.cos(psi)
    out[ACC] = -sway * turns * turns * np.sin(psi)
    out[KS], out[DS], out[PSI], out[SWAY] = p[K], p[D], psi, sway
    out[TURNS], out[HELD], out[YAWED] = turns, p[ZMP], p[YAW]
    return out

