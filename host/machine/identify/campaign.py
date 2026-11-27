"""The stand's campaign: the robot's pose, a limb a stage at tones, step by step; its gearboxes read.

    world.reset(campaign.pose(), where=(0.0, mjcf.STAND_M, 0.0))   # on the stand (mjcf.STAND)
    ref = campaign.setpoints(world, seconds, rng)   # a stage a limb, its drives at tones
    ys, taus, touched = campaign.excite(world, ref) # the robot on the stand, played a step at a time
    ys, taus = campaign.smoothed(ys, taus, dt)      # low-passed both ways, the rotors' ring out
    campaign.boxes(body, ys, taus, dt, armature)    # each gearbox's wind-up, play, damping, mesh; its rotor

What `machine.identify.sysid` reads its rows off: the robot's edge arrays (`machine.senses.edge`) and the robot's
drives' torques a step, on the world or on a robot of its build, its boards' currents its torques.
"""
import math

from machine import libm
from machine.body import drives, figure, peripherals, terms
from machine.senses import edge
from machine.errors import MachineError
from machine.body.figure import JOINTS, LEG

#: Gearbox wind-up threshold, backlash-play rotor window, smoothing corner, coprime multi-sine
#: excitation tones, travel swing/margin and stand pose (`peripherals.identify()['campaign']`).
_camp = peripherals.identify()['campaign']
WOUND_NM, WOUND_ROWS, SMOOTH_HZ = _camp['wound_nm'], _camp['wound_rows'], _camp['smooth_hz']
INSIDE, ROTOR_ROWS, ROTOR_NM = _camp['inside'], _camp['rotor_rows'], _camp['rotor_nm']
BOX = ('k', 'play', 'c', 'mesh', 'J', 'zero')
TONES, SWING, MARGIN, FREE_RAD = _camp['tones'], _camp['swing'], _camp['margin'], _camp['free_rad']
ABDUCTED, RAMP_S = ('hip',), _camp['ramp_s']
POSE = _camp['pose']
#: The campaign's stages, a limb each: its driven joints swung, the rest held where they stand.
STAGES = (('spine_roll', 'spine', 'waist', 'neck', 'head'),) + tuple(
    tuple(side + k for k in kinds) for kinds in (('shoulder', 'elbow'), tuple(k[1:] for k in LEG))
    for side in ('left_', 'right_'))


def pose():
    """{joint: deg} the robot takes on the stand: the stand's, POSE's movements on it, every limb's."""
    degrees = figure.stand()
    for part, movement, deg in POSE:
        found = terms.find(part, movement)
        if found is None:
            raise MachineError('it has no %s of the %s to pose' % (movement, part))
        kind_, sign = found[0], found[1]
        for name in JOINTS:
            if drives.kind(name) == kind_:
                degrees[name] = degrees[name] + sign * deg
    return degrees


def lowpass_py(x, dt, hz=SMOOTH_HZ):
    """`lowpass` in pure NumPy/Python, kept as fallback and bit-exact parity oracle."""
    import numpy as np
    a = libm.exp(-2.0 * math.pi * hz * dt)
    x = np.array(x, float)
    for k in range(1, len(x)):
        x[k] += a * (x[k - 1] - x[k])
    for k in range(len(x) - 2, -1, -1):
        x[k] += a * (x[k + 1] - x[k])
    return x


def lowpass(x, dt, hz=SMOOTH_HZ):
    """`x` (N, ..) low-passed at `hz` both ways: no phase, first order each way."""
    from tools.cores import identify as c_id
    a = libm.exp(-2.0 * math.pi * hz * dt)
    got = c_id.lowpass(x, a)
    return lowpass_py(x, dt, hz) if got is None else got


def smoothed(ys, taus, dt, hz=SMOOTH_HZ):
    """(ys, taus) low-passed at `hz` both ways (`lowpass`), every sensed column and every torque
    alike: a gearbox's rotor rings at its mode in the motor's encoder and the torque that
    drives it, far above the campaign's tones, and the drives' armature read off its second
    differences came out negative on a third of them."""
    import numpy as np
    ys, taus = np.asarray(ys, float), np.asarray(taus, float)
    out = ys.copy()
    out[:, :edge.QM.stop] = lowpass(ys[:, :edge.QM.stop], dt, hz)
    out[:, edge.QUAT] /= np.linalg.norm(out[:, edge.QUAT], axis=1, keepdims=True)
    return out, lowpass(taus, dt, hz)


def tone(t, tones, phase, amp):
    """(len(t),) rad: sines at `tones` Hz with `phase` (rad) each, the first `amp` rad and the
    rest as much slower - every tone at one speed -, eased in and out over RAMP_S: one drive's
    excitation on its board (`coaxial.control.identify`), where a record's mean current turns
    with the shaft's speed."""
    import numpy as np
    ease = np.minimum(1.0, np.minimum(t - t[0], t[-1] - t) / RAMP_S)
    tones = np.asarray(tones, float)
    return ease * (amp * tones[0] / tones * libm.sin(2.0 * math.pi * np.outer(t, tones) + phase)).sum(axis=1)


def _box(delta, rel, passed, wound_nm=WOUND_NM):
    """(k, play, c, mesh, zero) of one box on the steps it is wound up, by least squares: past its
    play about `zero` (its encoder's slip, rad) it passes k (delta - zero - play sign(delta -
    zero)) + c rel + mesh tanh(rel / MESH_RAD_S) to its link. The mesh's friction is not read:
    within a step the rotor's ring swings the sliding across the rounding, and the step's mean
    sliding tells nothing of it (identification.md)."""
    import numpy as np
    from machine.body.gearbox import MESH_RAD_S
    wound = np.abs(passed) > wound_nm
    if wound.sum() < 4 * WOUND_ROWS or (np.abs(rel[wound]) > 1e-6).sum() < WOUND_ROWS:
        return 0.0, 0.0, 0.0, 0.0, float(delta[-1]) if len(delta) else 0.0
    pos, neg = passed > wound_nm, passed < -wound_nm
    if pos.sum() < WOUND_ROWS or neg.sum() < WOUND_ROWS:
        A = np.column_stack([delta[wound], -np.sign(delta[wound]), rel[wound], np.tanh(rel[wound] / MESH_RAD_S)])
        fit = np.linalg.lstsq(A, passed[wound], rcond=None)[0]
        return float(fit[0]), float(fit[1] / fit[0]) if fit[0] > 0.0 else 0.0, float(fit[2]), float(fit[3]), 0.0
    m0 = 0.5 * (float(np.median(delta[pos])) + float(np.median(delta[neg])))
    A = np.column_stack([delta[wound], -np.sign(delta[wound] - m0), rel[wound],
                         np.tanh(rel[wound] / MESH_RAD_S), -np.ones(wound.sum())])
    fit = np.linalg.lstsq(A, passed[wound], rcond=None)[0]
    k = float(fit[0])
    return (k, float(fit[1]) / k if k > 0.0 else 0.0, float(fit[2]), float(fit[3]),
            float(fit[4]) / k if k > 0.0 else 0.0)


def _deadband(delta, rel, passed, wound_nm):
    """(k, play, c, mesh, zero) when a narrow wound cluster leaves `_box`'s play non-positive:
    `play` and `zero` from consecutive steps inside the zero-torque deadband, `k` past them."""
    import numpy as np
    from machine.body.gearbox import MESH_RAD_S
    z = (np.abs(passed) < 0.1 * wound_nm) & (np.abs(rel) > 0.2)
    z3 = np.zeros_like(z)
    z3[1:-1] = z[:-2] & z[1:-1] & z[2:]
    if z3.sum() < ROTOR_ROWS:
        return None
    lo, hi = float(np.percentile(delta[z3], 2)), float(np.percentile(delta[z3], 98))
    play, zero = 0.5 * (hi - lo), 0.5 * (hi + lo)
    past = np.sign(delta - zero) * np.maximum(np.abs(delta - zero) - play, 0.0)
    high = np.abs(passed) > wound_nm
    keep = high & (past * passed > 0.0)
    if keep.sum() < ROTOR_ROWS // 2 or keep.sum() < 0.8 * high.sum():
        return None
    f = np.linalg.lstsq(np.column_stack([past[keep], rel[keep], np.tanh(rel[keep] / MESH_RAD_S)]),
                        passed[keep], rcond=None)[0]
    return float(f[0]), play, float(f[1]), float(f[2]), zero


def boxes(body, ys, taus, dt, armature, every=1):
    """Each drive's gearbox and rotor from its two encoders: (k N m/rad, play rad, c N m s/rad,
    mesh N m, J kg m^2, zero rad) a drive (driven, BOX). The box first with `armature` (`identify`'s
    J, the smoothed rows') the rotor's share of the motor's torque, for its play; the rotor's
    inertia on the steps inside the play, where the box passes nothing and the motor's torque is
    the rotor's alone - a step's second difference of the motor's angle, QM, is its torque's mean
    over the two steps about it, over J (`ROTOR_ROWS` at least, the torque past ROTOR_NM) -; the
    box again with that rotor (`_box`): delta the motor's angle past the joint's (QM - Q), rel
    their rates' difference, the torque's held over its step. Rigid, nothing winds up and only
    `zero` is read."""
    import numpy as np
    at = np.arange(1, len(ys) - 2, every)
    driven = np.flatnonzero(body.driven)
    ys, taus = np.asarray(ys), np.asarray(taus, float)
    # a torque holds over its step: the wind-up, its rate and the motor's acceleration at the
    # step's middle
    wound_up = ys[:, edge.QM] - ys[:, edge.Q]
    delta = 0.5 * (wound_up[at] + wound_up[at + 1])[:, driven]
    rel = (wound_up[at + 1] - wound_up[at])[:, driven] / dt
    qm = ys[:, edge.QM]
    wacc = (qm[at + 2] - qm[at + 1] - qm[at] + qm[at - 1])[:, driven] / (2.0 * (dt * dt))
    second = (qm[at + 1] - 2.0 * qm[at] + qm[at - 1])[:, driven] / (dt * dt)
    mean = 0.5 * (taus[at] + taus[at - 1])[:, driven]
    motor = taus[at][:, driven]
    mot3 = (0.25 * taus[at - 1] + 0.5 * taus[at] + 0.25 * taus[at + 1])[:, driven]
    out = np.zeros((len(driven), len(BOX)))
    for i in range(len(driven)):
        joint_name = JOINTS[driven[i]]
        J = float(armature[i])
        w_nm = min(WOUND_NM, round(drives.mesh(joint_name), 2))
        r_nm = min(ROTOR_NM, 3.0 * drives.armature(joint_name))
        _, play, _, _, zero = _box(delta[:, i], rel[:, i], motor[:, i] - J * wacc[:, i], w_nm)
        if play <= 0.0:
            got_z = _deadband(delta[:, i], rel[:, i], mot3[:, i] - J * wacc[:, i], w_nm)
            if got_z is not None:
                _, play, _, _, zero = got_z
        if play > 0.0:
            inside = np.abs(wound_up[:, driven[i]] - zero) < INSIDE * play
            rows = inside[at - 1] & inside[at] & inside[at + 1] & (np.abs(mean[:, i]) > ROTOR_NM)
            if rows.sum() < ROTOR_ROWS:
                rows = inside[at - 1] & inside[at] & inside[at + 1] & (np.abs(mean[:, i]) > r_nm)
            if rows.sum() >= ROTOR_ROWS and (mean[rows, i] @ second[rows, i]) > 0.0:
                J = float(mean[rows, i] @ mean[rows, i] / (mean[rows, i] @ second[rows, i]))
        k, play, c, mesh, zero = _box(delta[:, i], rel[:, i], motor[:, i] - J * wacc[:, i], w_nm)
        if play <= 0.0:
            got_z = _deadband(delta[:, i], rel[:, i], mot3[:, i] - J * wacc[:, i], w_nm)
            if got_z is not None:
                k, play, c, mesh, zero = got_z
        ks = drives.series(joint_name)
        if ks > 0.0 and (k == 0.0 or drives.track(joint_name) > 0.0) and len(taus):
            k0 = 1.0 / (1.0 / drives.wind(joint_name) + 1.0 / ks)
            zero = float(np.median(wound_up[:-1, driven[i]] - taus[:, driven[i]] / k0))
        out[i] = k, play, c, mesh, J, zero
    return out


def swing(world, index):
    """(middle rad, half swing rad) of drive `index`'s excitation: SWING of its range, MARGIN
    from either stop, from where it stands as far as that leaves it; FREE_RAD with no stop,
    about where it stands - a joint of ABDUCTED swung abducted from there (`machine.body.terms`)."""
    m = world.model
    name = JOINTS[index]
    j, q0 = m.joint(name).id, float(world.target[index])
    if m.jnt_limited[j]:
        lo, hi = m.jnt_range[j]
        amp, keep = SWING * (hi - lo), MARGIN * (hi - lo)
        return min(max(q0, lo + keep + amp), hi - keep - amp), amp
    for part in ABDUCTED:
        found = terms.find(part, 'abduction')
        if found and drives.kind(name) == found[0]:
            return q0 + found[1] * FREE_RAD, FREE_RAD
    return q0, FREE_RAD


def setpoints(world, seconds, rng):
    """(N + 1, JOINTS) rad, the campaign's setpoints a step of `seconds`: a stage a limb
    (STAGES) in turn, its drives each at TONES about its swing's middle, the phases drawn, eased
    in and out over RAMP_S; the rest where they stand."""
    import numpy as np
    from machine.world import physics
    h = physics.STEP_S
    per = int(round(seconds / len(STAGES) / h))
    ref = np.tile(world.target, (per * len(STAGES) + 1, 1))
    t = np.arange(per) * h
    ease = np.minimum(1.0, np.minimum(t, t[-1] - t) / RAMP_S)
    for s, stage in enumerate(STAGES):
        for name in stage:
            i = JOINTS.index(name)
            mid, amp = swing(world, i)
            hz = np.array(TONES[i % len(TONES)])
            phase = rng.uniform(0.0, 2.0 * math.pi, size=len(hz))
            swung = mid + amp * libm.sin(2.0 * math.pi * np.outer(t, hz) + phase).sum(axis=1) / len(hz)
            ref[s * per:(s + 1) * per, i] = world.target[i] + ease * (swung - world.target[i])
    return ref


def excite(world, ref, pulls=None):
    """(ys (N + 1, SENSED), taus (N, JOINTS), touched) of `ref` (N + 1, JOINTS) played on the
    stand a step each: the robot's edge read every step and its drives' torques over it; `touched` the
    steps something touched the robot. Hung, a list `pulls` takes what the harness's load cell reads
    each step (`World.pull`)."""
    import numpy as np
    from machine.world import physics
    y = edge.laid(world)
    driven = np.flatnonzero(world.driven)
    now = [world.at]
    world.clock = lambda: now[0]
    ys, taus, touched = [], [], 0
    for k in range(len(ref)):
        ys.append(edge.felt(world, y).copy())
        if pulls is not None:
            pulls.append(world.pull())
        touched += int(world.robot_own[world.model.geom_bodyid[world.data.contact.geom[:world.data.ncon]]].any())
        if k == len(ref) - 1:
            break
        world.target[driven] = ref[k + 1, driven]
        now[0] = world.at + physics.STEP_S
        world.advance()
        taus.append(world.motor.copy())
    world.clock = lambda: world.data.time
    return np.array(ys), np.array(taus), touched


def rotations(q):
    """(..., 3, 3) of quaternions `q` (..., 4), w first."""
    import numpy as np
    w, x, y, z = (np.asarray(q, float)[..., k] for k in range(4))
    return np.stack([np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], -1),
                     np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], -1),
                     np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], -1)], -2)


def hemming(lay, body, ys, dt, every, state_holder=None):
    """The robot's unmeasured 2-DoF pendulums (`lay['hems']`) integrated at `dt` over `ys` with their stops,
    sampled every `every`th, yielding after each: ((qadr, vadr, q, qd, qdd), free). Each one's anchor
    follows its chain down from the pelvis, a link's every joint in its order (the torso's spine and
    waist both), its IMU and its encoders the chain's motion."""
    import numpy as np
    from tools.cores import wbc as cores
    hems = lay.get('hems', ())
    m, at = body.m, np.arange(0, len(ys) - 1, every)
    jids = [int(lay['jnt'][h]) for k in hems for h in range(lay['span'][k, 0], lay['span'][k, 1])]
    qadr, vadr = np.array(m.jnt_qposadr[jids], int), np.array(m.jnt_dofadr[jids], int)
    if len(ys) < 2 or not jids or float(m.body_mass[m.jnt_bodyid[jids[0]]]) <= 0.0:
        z = np.zeros((len(at), len(jids)))
        return (qadr, vadr, z, z, z), np.ones((len(at), len(hems)), bool)
    n, rot = len(ys) - 1, rotations
    R0 = rot(ys[:n, edge.QUAT])
    w0 = np.einsum('nij,nj->ni', R0, ys[:n, edge.GYRO])
    al0 = np.einsum('nij,nj->ni', R0, (ys[1:n + 1, edge.GYRO] - ys[:n, edge.GYRO]) / dt)
    a0 = np.einsum('nij,nj->ni', rot(ys[1:n + 1, edge.QUAT]), ys[1:n + 1, edge.ACCEL])
    qd_all = ys[:n, edge.QD]
    qdd_all = (ys[1:n + 1, edge.QD] - qd_all) / dt
    q_out, v_out, a_out = np.zeros((n, len(jids))), np.zeros((n, len(jids))), np.zeros((n, len(jids)))
    hit, prev = np.zeros((n, len(hems)), bool), state_holder.get('_hem_prev') if state_holder is not None else None
    yield
    cont = prev is not None and float(np.max(np.abs(ys[0, edge.Q] - prev[0]))) < 1e-9
    next_state = []
    for s_idx, k in enumerate(hems):
        b = int(lay['bodies'][k])
        mass = float(m.body_mass[b])
        if mass <= 0.0:
            next_state.append((0.0, 0.0, 0.0, 0.0))
            continue
        chain, p = [], int(lay['parent'][k])
        while p > 0:
            chain.append(int(lay['bodies'][p]))
            p = int(lay['parent'][p])
        R, wv, al, a = R0, w0, al0, a0
        for link in reversed(chain):
            dp = np.einsum('nij,j->ni', R, m.body_pos[link])
            a = a + np.cross(al, dp) + np.cross(wv, np.cross(wv, dp))
            R = R @ rot(m.body_quat[link])
            first = int(m.body_jntadr[link])
            for j in range(first, first + int(m.body_jntnum[link])):
                ax, ji = m.jnt_axis[j], int(np.flatnonzero(lay['vadr'] == m.jnt_dofadr[j])[0])
                q, qd, qdd_j = ys[:n, edge.Q][:, ji], qd_all[:, ji:ji + 1], qdd_all[:, ji:ji + 1]
                sw = np.einsum('nij,j->ni', R, ax)
                al, wv = al + sw * qdd_j + np.cross(wv, sw) * qd, wv + sw * qd
                c, s = libm.cos(0.5 * q), libm.sin(0.5 * q)
                R = R @ rot(np.column_stack([c, s * ax[0], s * ax[1], s * ax[2]]))
        dp = np.einsum('nij,j->ni', R, m.body_pos[b])
        ap = np.einsum('nji,nj->ni', R, a + np.cross(al, dp) + np.cross(wv, np.cross(wv, dp)))
        om, alb = np.einsum('nji,nj->ni', R, wv), np.einsum('nji,nj->ni', R, al)
        L, I0 = -float(m.body_ipos[b, 1]), float(m.body_inertia[b, 0])
        J, jx, jz = I0 + mass * L * L, jids[2 * s_idx], jids[2 * s_idx + 1]
        stiff, damp = float(m.jnt_stiffness[jx]), float(m.dof_damping[vadr[2 * s_idx]])
        tc, dr = float(m.jnt_solref[jx, 0]), float(m.jnt_solref[jx, 1])
        dmin, dmax, width = float(m.jnt_solimp[jx, 0]), float(m.jnt_solimp[jx, 1]), float(m.jnt_solimp[jx, 2])
        invw = (float(m.dof_invweight0[vadr[2 * s_idx]]), float(m.dof_invweight0[vadr[2 * s_idx + 1]]))
        lo, hi = (float(m.jnt_range[jx, 0]), float(m.jnt_range[jz, 0])), (float(m.jnt_range[jx, 1]), float(m.jnt_range[jz, 1]))
        k_imp, b_imp = 1.0 / (dmax * dmax * tc * tc * dr * dr), 2.0 / (dmax * tc)
        state = np.array(prev[1][s_idx] if cont and prev is not None else (0.0, 0.0, 0.0, 0.0), float)
        steps, hits = np.empty((3, n, 2)), np.zeros(n, np.uint8)
        consts = np.array([L, mass, I0, J, stiff, damp, dmin, dmax, width, k_imp, b_imp, *invw, *lo, *hi])
        ap, om, alb = (np.ascontiguousarray(x, dtype=np.float64) for x in (ap, om, alb))
        cores.library().wbh_pass_hem(n, dt, ap.ctypes.data, om.ctypes.data, alb.ctypes.data, consts.ctypes.data,
                                     state.ctypes.data, steps[0].ctypes.data, steps[1].ctypes.data,
                                     steps[2].ctypes.data, hits.ctypes.data)
        q_out[:, 2 * s_idx:2 * s_idx + 2], v_out[:, 2 * s_idx:2 * s_idx + 2], a_out[:, 2 * s_idx:2 * s_idx + 2] = steps
        hit[:, s_idx] = hits.astype(bool)
        next_state.append(tuple(state.tolist()))
        yield
    if state_holder is not None:
        state_holder['_hem_prev'] = (ys[-1, edge.Q].copy(), next_state)
    pad = np.zeros((len(at), len(hems)), bool)
    for d_step in range(-5, 6):
        pad |= hit[np.clip(at + d_step, 0, n - 1)]
    return (qadr, vadr, q_out[at], v_out[at], a_out[at]), ~pad


def capsules(m, bodies, above, driven, vadr):
    """The robot's colliding capsules (`momentum._capsules`) on `bodies` and the pairs whose chains
    differ across `driven`, laid once for `self_hit`."""
    import numpy as np
    from machine.senses import momentum
    caps, pairs = momentum._capsules(m)
    place = {m.body(int(b)).name: k for k, b in enumerate(bodies)}
    b_pos, b_rot = np.array(m.body_pos[bodies], float), rotations(m.body_quat[bodies])
    jnts = tuple(tuple((int(np.flatnonzero(vadr == m.jnt_dofadr[j])[0]),
                        tuple(float(v) for v in m.jnt_axis[j]), tuple(float(v) for v in m.jnt_pos[j]))
                       for j in range(int(m.body_jntadr[b]), int(m.body_jntadr[b] + m.body_jntnum[b]))
                       if np.any(vadr == m.jnt_dofadr[j])) for b in bodies)
    ck = np.array([place[seg] for seg, *_ in caps], int)
    cr = np.array([r for _s, r, *_ in caps], float)
    ca = np.array([a for _s, _r, a, _b in caps], float)
    cb = np.array([b for _s, _r, _a, b in caps], float)
    pr = np.array(pairs, int)
    pdrv = above[ck[pr[:, 0]]][:, driven] ^ above[ck[pr[:, 1]]][:, driven]
    keep = pdrv.any(axis=1)
    pr, pdrv = pr[keep], pdrv[keep]
    half = 0.5 * np.sqrt(np.sum(libm.sq(cb - ca), axis=1)) + cr
    return (b_pos, b_rot, jnts, ck, cr, ca, cb, pr, pdrv,
            libm.sq(half[pr[:, 0]] + half[pr[:, 1]]), libm.sq(cr[pr[:, 0]] + cr[pr[:, 1]]))


def self_hit_py(lay, ys_arr, chunk=400):
    """`self_hit` in pure NumPy/Python, kept as fallback and parity oracle."""
    import numpy as np
    caps = lay.get('caps')
    if caps is None or not len(ys_arr):
        return np.zeros((len(ys_arr), len(lay['driven'])), bool)
    b_pos, b_rot, jnts, ck, _cr, ca, cb, pr, pdrv, reach_sq, rad_sq = caps
    n, nb, pdrv_i = len(ys_arr), len(lay['bodies']), pdrv.astype(int)
    ia, ib, out = pr[:, 0], pr[:, 1], np.zeros((n, pdrv.shape[1]), bool)
    for start in range(0, n, chunk):
        ys = ys_arr[start:min(start + chunk, n)]
        m = len(ys)
        xpos, xmat = np.zeros((m, nb, 3)), np.zeros((m, nb, 3, 3))
        xmat[:, 0] = rotations(ys[:, edge.QUAT])
        for k in range(1, nb):
            p_idx = int(lay['parent'][k])
            Rp, pp = xmat[:, p_idx], xpos[:, p_idx]
            R, p = Rp @ b_rot[k], pp + np.einsum('nij,j->ni', Rp, b_pos[k])
            for ji, ax, jp in jnts[k]:
                q = ys[:, edge.Q][:, ji]
                c, s = libm.cos(0.5 * q), libm.sin(0.5 * q)
                dR = rotations(np.column_stack([c, s * ax[0], s * ax[1], s * ax[2]]))
                if any(jp):
                    jp_a = np.array(jp)
                    p = p + np.einsum('nij,j->ni', R, jp_a) - np.einsum('nij,j->ni', R @ dR, jp_a)
                R = R @ dR
            xpos[:, k], xmat[:, k] = p, R
        pa = xpos[:, ck] + np.einsum('ncij,cj->nci', xmat[:, ck], ca)
        pb = xpos[:, ck] + np.einsum('ncij,cj->nci', xmat[:, ck], cb)
        mid = 0.5 * (pa + pb)
        step_i, pair_i = np.nonzero(np.sum(libm.sq(mid[:, ia] - mid[:, ib]), axis=2) < reach_sq)
        if not len(step_i):
            continue
        p1, q1, p2, q2 = pa[step_i, ia[pair_i]], pb[step_i, ia[pair_i]], pa[step_i, ib[pair_i]], pb[step_i, ib[pair_i]]
        d1, d2, r = q1 - p1, q2 - p2, p1 - p2
        a, e = np.sum(d1 * d1, axis=1), np.sum(d2 * d2, axis=1)
        f, c, b = np.sum(d2 * r, axis=1), np.sum(d1 * r, axis=1), np.sum(d1 * d2, axis=1)
        den = a * e - b * b
        s = np.where(den > 1e-12, np.clip((b * f - c * e) / np.maximum(den, 1e-12), 0.0, 1.0), 0.0)
        t = np.where(e > 1e-12, (b * s + f) / np.maximum(e, 1e-12), 0.0)
        lo, hi = t < 0.0, t > 1.0
        t = np.clip(t, 0.0, 1.0)
        s = np.where(lo, np.where(a > 1e-12, np.clip(-c / np.maximum(a, 1e-12), 0.0, 1.0), 0.0), s)
        s = np.where(hi, np.where(a > 1e-12, np.clip((b - c) / np.maximum(a, 1e-12), 0.0, 1.0), 0.0), s)
        over = np.sum(libm.sq((p1 + s[:, None] * d1) - (p2 + t[:, None] * d2)), axis=1) < rad_sq[pair_i]
        hit = np.zeros((m, len(pr)), bool)
        hit[step_i[over], pair_i[over]] = True
        out[start:start + m] = (hit.astype(int) @ pdrv_i) > 0
    return out


def self_hit(lay, ys_arr, chunk=400):
    """(N, driven) bool: which drives lie between two capsules of the robot that overlap at each
    step of `ys_arr`, where internal self-contact torques enter without touching `edge.TOUCH`."""
    from tools.cores import identify as c_id
    got = c_id.self_hit(lay, ys_arr, edge.QUAT.start, edge.Q.start)
    return self_hit_py(lay, ys_arr, chunk) if got is None else got
