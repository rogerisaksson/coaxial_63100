"""The stand's campaign: her pose, a limb a stage at tones, step by step; her gearboxes read.

    world.reset(campaign.pose(), where=(0.0, mjcf.STAND_M, 0.0))   # on the stand (mjcf.STAND)
    ref = campaign.setpoints(world, seconds, rng)   # a stage a limb, its drives at tones
    ys, taus, touched = campaign.excite(world, ref) # her on the stand, played a step at a time
    ys, taus = campaign.smoothed(ys, taus, dt)      # low-passed both ways, the rotors' ring out
    campaign.boxes(body, ys, taus, dt, armature)    # each gearbox's wind-up, play, damping, mesh

What `machine.sysid` reads its rows off: her edge arrays (`machine.edge`) and her drives' torques
a step, on the world or on a robot of her build, its boards' currents its torques.
"""
import math

from machine import drives, edge, figure, terms
from machine.errors import MachineError
from machine.figure import JOINTS, LEG

#: A gearbox counts as wound up passing more than this, N m, and is read on this many rows at
#: least (`boxes`); a campaign's arrays low-passed at this, Hz, both ways, before its rows
#: (`smoothed`: the tones under 1.4, a rotor's mode 30).
WOUND_NM, WOUND_ROWS, SMOOTH_HZ = 1.0, 25, 10.0
#: The excitation's sines a drive, Hz; the share of a joint's range it swings through and the
#: share kept from either stop; an unlimited joint's swing, rad; the parts whose unlimited
#: joint swings abducted; a stage's ease in and out, s.
TONES, SWING, MARGIN, FREE_RAD = ((0.23, 0.61, 1.37), (0.31, 0.79, 1.13)), 0.35, 0.05, 0.5
ABDUCTED, RAMP_S = ('hip',), 0.5
#: Her pose on the stand: the stand's (`figure.stand`) with each (part, movement, deg) of
#: POSE laid on it (`machine.terms`) - her arms held out before her, clear of her thighs.
POSE = (('shoulder', 'flexion', 90.0),)
#: The campaign's stages, a limb each: its driven joints swung, the rest held where they stand.
STAGES = (('spine_roll', 'spine', 'waist', 'neck', 'head'),) + tuple(
    tuple(side + k for k in kinds) for kinds in (('shoulder', 'elbow'), tuple(k[1:] for k in LEG))
    for side in ('left_', 'right_'))


def pose():
    """{joint: deg} she takes on the stand: the stand's, POSE's movements on it, every limb's."""
    degrees = figure.stand()
    for part, movement, deg in POSE:
        found = terms.find(part, movement)
        if found is None:
            raise MachineError('she has no %s of the %s to pose' % (movement, part))
        kind_, sign = found[0], found[1]
        for name in JOINTS:
            if drives.kind(name) == kind_:
                degrees[name] = degrees[name] + sign * deg
    return degrees


def lowpass(x, dt, hz=SMOOTH_HZ):
    """`x` (N, ..) low-passed at `hz` both ways: no phase, first order each way."""
    import numpy as np
    a = math.exp(-2.0 * math.pi * hz * dt)
    x = np.array(x, float)
    for sweep in (range(1, len(x)), range(len(x) - 2, -1, -1)):
        for k in sweep:
            x[k] = a * x[k - 1 if sweep.step > 0 else k + 1] + (1.0 - a) * x[k]
    return x


def smoothed(ys, taus, dt, hz=SMOOTH_HZ):
    """(ys, taus) low-passed at `hz` both ways (`lowpass`), every sensed column and every torque
    alike: a gearbox's rotor rings at its mode in the motor's encoder and the torque that
    drives it, far above the campaign's tones, and the drives' armature read off its second
    differences came out negative on a third of them."""
    import numpy as np
    ys, taus = np.asarray(ys, float), np.asarray(taus, float)
    out = ys.copy()
    for cols in (edge.QUAT, edge.GYRO, edge.ACCEL, edge.Q, edge.QD, edge.QM):
        out[:, cols] = lowpass(ys[:, cols], dt, hz)
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
    return ease * (amp * tones[0] / tones * np.sin(2.0 * math.pi * np.outer(t, tones) + phase)).sum(axis=1)


def boxes(body, ys, taus, dt, armature, every=1):
    """Each drive's gearbox from its two encoders: (k N m/rad, play rad, c N m s/rad, mesh N m)
    a drive (driven, 4) by least squares on the steps it is wound up - past its play the box
    passes k (delta - play sign delta) + c rel + mesh sign rel to its link, delta the motor's
    angle past the joint's (QM - Q), rel their rates' difference; what it passes is the motor's
    torque less its armature's (`identify`'s J) share of its acceleration. Rigid, nothing winds
    up and every number is 0."""
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
    wacc = (qm[at + 2] - qm[at + 1] - qm[at] + qm[at - 1])[:, driven] / (2.0 * dt ** 2)
    passed = taus[at][:, driven] - np.asarray(armature) * wacc
    out = np.zeros((len(driven), 4))
    for i in range(len(driven)):
        wound = np.abs(passed[:, i]) > WOUND_NM
        if wound.sum() < 4 * WOUND_ROWS:
            continue
        A = np.column_stack([delta[wound, i], -np.sign(delta[wound, i]), rel[wound, i], np.sign(rel[wound, i])])
        fit, _r, _k, _s = np.linalg.lstsq(A, passed[wound, i], rcond=None)
        k = fit[0]
        out[i] = (k, fit[1] / k if k > 0.0 else 0.0, fit[2], fit[3])
    return out


def swing(world, index):
    """(middle rad, half swing rad) of drive `index`'s excitation: SWING of its range, MARGIN
    from either stop, from where it stands as far as that leaves it; FREE_RAD with no stop,
    about where it stands - a joint of ABDUCTED swung abducted from there (`machine.terms`)."""
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
    from machine import physics
    h = physics.STEP_S
    per = int(round(seconds / len(STAGES) / h))
    ref = np.tile(world.target, (per * len(STAGES) + 1, 1))
    t = np.arange(per) * h
    ease = np.minimum(1.0, np.minimum(t, t[-1] - t) / RAMP_S)
    for s, stage in enumerate(STAGES):
        for name in stage:
            i = JOINTS.index(name)
            mid, amp = swing(world, i)
            tone = np.array(TONES[i % len(TONES)])
            phase = rng.uniform(0.0, 2.0 * math.pi, size=len(tone))
            swung = mid + amp * np.sin(2.0 * math.pi * np.outer(t, tone) + phase).sum(axis=1) / len(tone)
            ref[s * per:(s + 1) * per, i] = world.target[i] + ease * (swung - world.target[i])
    return ref


def excite(world, ref):
    """(ys (N + 1, SENSED), taus (N, JOINTS), touched) of `ref` (N + 1, JOINTS) played on the
    stand a step each: her edge read every step and her drives' torques over it; `touched` the
    steps something touched her."""
    import numpy as np
    from machine import physics
    y = edge.laid(world)
    driven = np.flatnonzero(world.driven)
    now = [world.at]
    world.clock = lambda: now[0]
    ys, taus, touched = [], [], 0
    for k in range(len(ref)):
        ys.append(edge.felt(world, y).copy())
        touched += int(world.hers[world.model.geom_bodyid[world.data.contact.geom[:world.data.ncon]]].any())
        if k == len(ref) - 1:
            break
        world.target[driven] = ref[k + 1, driven]
        now[0] = world.at + physics.STEP_S
        world.advance()
        taus.append(world.motor.copy())
    world.clock = lambda: world.data.time
    return np.array(ys), np.array(taus), touched
