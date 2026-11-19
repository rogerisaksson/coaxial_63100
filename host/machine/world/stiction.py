"""Friction at rest and sliding: MuJoCo's contact rows stuck where they hold, sliding where they yield.

    state = stiction.laid(model, shares)         # a world's contacts' slips, none yet; each geom's sliding share
    stiction.home(state)                         # ... none again (the world laid anew)
    stiction.stepped(mj, model, data, state)     # MuJoCo's step whole about them: stuck between its halves, slipped after
    stiction.stuck(data, state)                  # after mj_step1: each friction row's reference sprung on its slip
    stiction.slipped(data, state)                # after mj_step2: each slip yielded to what its cone holds

MuJoCo lays one friction a contact, on rows that are dampers alone: a row's reference acceleration is -B v, its
violation naught (its K laid 0). So it knows neither friction at rest nor sliding: a contact that holds a load f
along its plane creeps at R f / B for good - the robot's pelvis lain on a bed 0.58 mm/s under 12 N of its 102, sat
on a chair till it slid off its seat (`tools/sim/creep.py`) - and one that slides slowly bears less than its cone.

Static. A row whose violation is its slip s since the contact stuck - its reference -B v - K I s, K the stiffest
its own damper holds without ringing ((B / 2)^2), I the row's impedance, as MuJoCo lays any row's reference on its
violation - holds its load at rest, R f / (K I) off where it stuck (tens of um). Past what its cone holds the slip
yields: a contact's rows' pulls K I s / R over their friction's, past its normal force, are scaled back onto it.

Dynamic. The friction a contact's geoms state is its friction at rest (a sole's, a garment's, a foot of the
furniture's as `loose.GRIPS` has it). A contact that yielded slides, on its sliding share of that - the world's
`kinetic_share` of a sole's on a floor, a furniture foot's own (`loose.shares`), the share of the geom whose
friction MuJoCo took - at any speed; and sticks again once its pull is back inside its sliding cone; a contact
its geoms had none of lands sliding, and sticks once it has held. So a load
between the two holds at rest and, once moved, slides on: Coulomb's two coefficients, the state between them the
contact's own slip. No threshold, no speed, no dwell: MuJoCo's own solver holds or slides each row on its cone,
and not one solve is added to its step.

A contact's slip is kept in the world's frame - along its plane, about its normal and about its plane - with its
two geoms, where it bore and what of its slip its cone held; a step's contact takes those of the last step's
nearest contact of its two geoms (a box's corners keep their own, a new corner its neighbour's: a body stuck slips
as one), none where its geoms bore on nothing. Rolling carries no slip: its rows' speeds are naught. Between
MuJoCo's step's halves and after it in C (wbc_world_stuck, wbc_world_slipped - with the robot's world's steps,
`steps.advanced`), `_stuck` and `_slipped` their twins to the bit (test_tick, test_stiction).

Measured and left out. A softer spring - the contact's normal stiffness sheared, a third to a tenth of this - kept
cm of slip in a mattress and threw the robot as it lay down. A joint's dry friction row creeps the same way - a
limp elbow sinks 0.30 deg/s under 0.34 N m through a gearbox whose friction is 1.23 - and is left MuJoCo's: stuck
as stiffly, the robot's law, which counts on no stiction, lost seven of its checks.
"""

import math

from machine import libm, libm_bound
from machine.body import mechatronics

#: The contacts kept a geom of the model: a contact past them keeps MuJoCo's rows.
A_GEOM = 4
#: A sliding contact's friction over its friction at rest where its geom states none of its own: the share of a
#: rubber sole's static friction on a floor that holds once it slides (`world.toml`).
KINETIC = float(mechatronics.environment()['kinetic_share'])
#: The friction's fade from rest to its sliding share by a contact's slip speed, 1 / v^2 (`world.toml`'s
#: stribeck_m_s): the SE(3) core's law (se3_rubs).
FADE = 1.0 / libm.sq(float(mechatronics.environment()['stribeck_m_s']))
#: A surface's own sliding share, by its geom's name: ice bears a sole on a film of its own melt, at rest as
#: sliding (on KINETIC of its 0.1 the robot fell on the ice in each of its six scenarios).
SURFACES = {'ice': 1.0}
#: Which of a contact's two slips each of its friction rows reads - along its plane (0) or about (3) - and on which
#: of its frame's rows: its two tangents, its normal, its two tangents (wbc_world.c's slip_of, slip_on).
OF, ON = (0, 0, 3, 3, 3), (3, 6, 0, 3, 6)


def laid(m, shares=None):
    """(wbc_stuck_t, each contact's geoms (cap, 2), where it bore (cap, 3), its slip (cap, 6), this step's friction
    rows' slips (cap, 5), its rows sprung in C, its slips yielded in C, what of each contact's slip its cone held
    (cap,), MuJoCo's geoms' friction (ngeom, 3), each geom's sliding share (ngeom,), each contact's normal force
    (cap,)) of model `m`: its contacts' slips, none yet - A_GEOM contacts a geom kept; `shares` each geom's sliding
    friction over its friction at rest, KINETIC every geom's where none is given, a surface SURFACES names its own."""
    import numpy as np
    from tools.cores import world
    cap = A_GEOM * m.ngeom
    pair, at, slip, row = np.full((cap, 2), -1, np.int32), np.zeros((cap, 3)), np.zeros((cap, 6)), np.zeros((cap, 5))
    held, borne, grip = np.ones(cap), np.zeros(cap), m.geom_friction
    share = np.full(m.ngeom, KINETIC) if shares is None else np.ascontiguousarray(shares, float)
    for g in range(m.ngeom):
        share[g] = SURFACES.get(m.geom(g).name, share[g])
    s, sprung, yielded = world.stuck(pair, at, slip, row, held, borne, grip, share, float(m.opt.timestep), FADE)
    return s, pair, at, slip, row, sprung, yielded, held, grip, share, borne


def home(state):
    """`state`'s contacts' slips none again: the world laid anew bears on nothing yet."""
    state[0].n = 0


def stuck(d, state, in_c=True):
    """Between MuJoCo's step's halves on data `d`: each contact's friction rows sprung on their slip (`state`), a
    contact that yielded on its sliding friction; in C where `in_c` (wbc_world_stuck), else its twin (`_stuck`)."""
    if in_c:
        state[5](d)
    else:
        _stuck(d, state)


def slipped(d, state, in_c=True):
    """After MuJoCo's step on data `d`: each contact's slip yielded to what its cone holds (`state`), in C where
    `in_c` (wbc_world_slipped), else its twin (`_slipped`)."""
    if in_c:
        state[6](d)
    else:
        _slipped(d, state)


def stepped(mj, m, d, state, in_c=True):
    """MuJoCo's step whole on model `m` and data `d`, its contacts stuck: its first half (their rows laid), their
    friction rows sprung (`stuck`), its second half (its solver, its integrator), their slips yielded (`slipped`) -
    MuJoCo's own `mj_step` operation for operation but for the rows' references and a sliding contact's friction."""
    mj.mj_step1(m, d)
    stuck(d, state, in_c)
    mj.mj_step2(m, d)
    slipped(d, state, in_c)


def _gripped(d, at, dim, geoms, elliptic):
    """Whether MuJoCo's constraints hold a contact's friction rows, an elliptic cone's between two geoms: its
    address `at`, its `dim`, its `geoms`."""
    return at >= 0 and dim >= 3 and geoms[0] >= 0 and geoms[1] >= 0 and int(d.efc_type[at]) == elliptic


def _sprung(kbip, e):
    """Friction row `e`'s spring on its slip (wbc_world.c's sprung): (B / 2)^2 at its impedance."""
    half = 0.5 * float(kbip[e, 1])
    return (half * half) * float(kbip[e, 2])


def _stuck(d, state):
    """wbc_world_stuck's twin: each of data `d`'s contacts' friction rows' slip - the last step's nearest contact of
    its two geoms' (`state`), carried on by the row's speed a step - and the row's reference less K I s, s kept to
    the reach its load bore the last step (the SE(3) core's se3_kept); its friction MuJoCo laid falling to its
    sliding share by its slip's speed (se3_rubs)."""
    from tools.cores import se3
    s, pair, at, slip, row = state[:5]
    grip, share, borne = state[8:11]
    n, last, dt = min(d.ncon, s.cap), s.n, s.step_s
    if not n:
        return
    elliptic = int(libm_bound.mujoco().mjtConstraint.mjCNSTR_CONTACT_ELLIPTIC)
    con, kbip, vel, aref, res = d.contact, d.efc_KBIP, d.efc_vel, d.efc_aref, d.efc_R
    geoms, pos, frames = con.geom[:n].tolist(), con.pos[:n].tolist(), con.frame[:n].tolist()
    dims, adr = con.dim[:n].tolist(), con.efc_address[:n].tolist()
    start = 0
    for i in range(n):
        g0, g1 = geoms[i]
        was, j, seen, bore = [0.0] * 6, start, 0, 0.0
        while seen < last and (pair[j, 0] != g0 or pair[j, 1] != g1):
            j = j + 1 if j + 1 < last else 0
            seen += 1
        if seen < last:
            start, least = j, math.inf
            while j < last and pair[j, 0] == g0 and pair[j, 1] == g1:
                x, y, z = pos[i][0] - float(at[j, 0]), pos[i][1] - float(at[j, 1]), pos[i][2] - float(at[j, 2])
                far = (x * x + y * y) + z * z
                if far < least:
                    least, was, bore = far, slip[j].tolist(), float(borne[j])
                j += 1
        row[i] = 0.0
        if _gripped(d, adr[i], dims[i], geoms[i], elliptic):
            f = frames[i]
            g = g1 if float(grip[g1, 0]) == float(con.friction[i, 0]) else g0
            a0 = adr[i]
            u2 = (float(vel[a0 + 1]) * float(vel[a0 + 1])) + (float(vel[a0 + 2]) * float(vel[a0 + 2]))
            slides = se3.rubs_twin(1.0, float(share[g]), s.fade, u2)
            con.friction[i] = con.friction[i] * slides
            con.mu[i] = con.mu[i] * slides
            pulls = 0.0
            for r in range(dims[i] - 1):
                e, o, t = a0 + 1 + r, OF[r], ON[r]
                went = ((was[o] * f[t] + was[o + 1] * f[t + 1]) + was[o + 2] * f[t + 2]) + float(vel[e]) * dt
                row[i, r] = went
                pull = (_sprung(kbip, e) * went) / (float(res[e]) * float(con.friction[i, r]))
                pulls = pulls + pull * pull
            kept = float(se3.kept_twin(max(bore, 0.0), 0.0, pulls, 1.0, 0.0, 0.0, 1.0))
            for r in range(dims[i] - 1):
                e = a0 + 1 + r
                aref[e] -= _sprung(kbip, e) * (kept * float(row[i, r]))


def _slipped(d, state):
    """wbc_world_slipped's twin: each of data `d`'s contacts' slip (`state`) yielded to what its cone holds - its
    rows' pulls K I s / R over their friction's, past its normal force, scaled back onto it: the SE(3) core's
    radial return (se3_kept) -, kept in the world's frame with its geoms, where it bore and what of it its cone
    held."""
    from tools.cores import se3
    s, pair, at, slip, row = state[:5]
    yielded, borne_of = state[7], state[10]
    n = min(d.ncon, s.cap)
    if n:
        elliptic = int(libm_bound.mujoco().mjtConstraint.mjCNSTR_CONTACT_ELLIPTIC)
        con, kbip, res, force = d.contact, d.efc_KBIP, d.efc_R, d.efc_force
        frames, dims, adr = con.frame[:n].tolist(), con.dim[:n].tolist(), con.efc_address[:n].tolist()
        mu, geoms = con.friction[:n].tolist(), con.geom[:n].tolist()
        pair[:n], at[:n] = con.geom[:n], con.pos[:n]
        for i in range(n):
            f, a, rows, held = frames[i], adr[i], row[i].tolist(), 1.0
            borne_of[i] = 0.0
            if _gripped(d, a, dims[i], geoms[i], elliptic):
                pulls, borne = 0.0, float(force[a])
                borne_of[i] = borne
                for r in range(dims[i] - 1):
                    e = a + 1 + r
                    pull = (_sprung(kbip, e) * rows[r]) / (float(res[e]) * mu[i][r])
                    pulls = pulls + pull * pull
                held = float(se3.kept_twin(max(borne, 0.0), 0.0, pulls, 1.0, 0.0, 0.0, 1.0))
            yielded[i] = held
            for x in range(3):
                slip[i, x] = held * (rows[0] * f[3 + x] + rows[1] * f[6 + x])
                slip[i, 3 + x] = held * ((rows[2] * f[x] + rows[3] * f[3 + x]) + rows[4] * f[6 + x])
    s.n = n
