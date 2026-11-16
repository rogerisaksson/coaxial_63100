"""The robot's joints' loops, solved: each of what its drive turns and the control design's targets.

    rows = looped.table()       # {kind: (rotor, limb, most, Loop, needs, ceilings)}
    kp_kd = looped.gains()      # {kind: (kp N m/rad, kd N m s/rad)}

One rule for every kind, no formula and no damping ratio a joint (`machine.solver.loops`). A drive's
stiffness is the stiffer of two needs: its limb's - what hangs on it with its rotor, followed at the
swing's frequency, the cascade's factor over the stance loop's - and, where it carries the robot, its
stance's - the body it carries held at the fall's rate by its factor, with the stiffness gravity tips
that body by (`carried.table`). A drive's ceilings are its structure's series stiffness and the phase
margin its bus's dead time leaves on the least it sees: its own rotor alone, inside its gearbox's play
(behind a series spring what the rotor's feedforward leaves of it). A drive's damping is the ratio
asked on the geometric mean of that least and what its stiffness is set on - its limb, or the body it
carries -, no more than its rotor alone takes within the phase its dead time may take.
The targets are `control.toml`'s, stated once for every machine. The table stands beside the servo
table the compiler lays by hand (`compile.servos`), a row a kind to compare: the robot's law does not
run on it. Laid in its place, five damping rules each left a check at its limit red - a rotor's
inertia or a drive's friction read back off a campaign (test_sysid), the rise from the chair
(test_errands) -, a different one each time, while its stiffnesses stood every bench.
"""
from math import radians, sqrt

from machine.body import carried, compile, drives, figure, mechatronics
from machine.solver import loops

#: A sampled loop's dead time in its bus's periods: half of one its hold, one the answer's way back.
DEAD_PERIODS = 1.5


def _joint(kind):
    return kind if kind in figure.JOINTS else 'left_' + kind


def rotor(kind):
    """A kind's drive's own inertia at its joint, kg m^2: its rotor and gearbox through its ratio."""
    joint = _joint(kind)
    own = drives.armature(joint)
    if own > 0.0:
        return own
    box = drives._box(joint)
    return (1.0 + float(box['input_j_share'])) * drives.size(kind).rotor * float(box['ratio']) ** 2


def felt(kind):
    """The armature a kind's loop feels, kg m^2: what the rotor's feedforward leaves of its rotor's."""
    box = drives._box(_joint(kind))
    return (compile.ROTOR_FF + float(box['input_j_share'])) * drives.size(kind).rotor * float(box['ratio']) ** 2


def table():
    """{kind: (rotor, limb, most - kg m^2 -, its Loop, its needs, its ceilings)} of every driven kind and
    the hand's two, off the compiled subtrees and what each drive turns (`carried.table`)."""
    st, ctl = compile.subtrees(), mechatronics.control()
    zeta, stance_w = float(ctl['damping_ratio']), float(ctl['fall_factor']) * st['omega']
    swing_w = float(ctl['cascade_factor']) * stance_w
    taken = radians(float(ctl['delay_phase_deg']))
    dead_w = loops.natural(DEAD_PERIODS / drives.BUS_HZ, zeta, loops.margin(zeta) - taken)
    limbs = {**carried.table(compile.G_M_S2), 'wrist': ((st['hand'][2], 0.0, 0.0),),
             'gripper': ((st['fingers'][2], 0.0, 0.0),)}
    out = {}
    for kind, postures in limbs.items():
        joint = _joint(kind)
        own = felt(kind) if drives.series(joint) > 0.0 else rotor(kind)
        limb = max(swing for swing, _stance, _tip in postures) + own
        borne = [(stance + own, tip) for _swing, stance, tip in postures if stance]
        needs = {'swing': loops.stiffness(limb, swing_w)}
        if borne:
            needs['stance'] = max(loops.stiffness(body, stance_w) + tip for body, tip in borne)
        ceilings = {'dead time': loops.stiffness(own, dead_w)}
        if drives.wind(joint) > 0.0:
            ceilings['series'] = drives.wind(joint)
        most = max([limb] + [body for body, _tip in borne])
        out[kind] = (own, limb, most, loops.design((own, most), zeta, needs, ceilings), needs, ceilings)
    return out


def gains():
    """{kind: (kp N m/rad, kd N m s/rad)}: each loop's stiffness under its least ceiling, its damping the
    ratio asked on the mean of its rotor alone and what its stiffness is set on - its limb where the swing
    set it, the body it carries where its stance did -, no more than its rotor's dead time takes."""
    ctl = mechatronics.control()
    zeta = float(ctl['damping_ratio'])
    damp_w = radians(float(ctl['delay_phase_deg'])) * drives.BUS_HZ / DEAD_PERIODS
    out = {}
    for kind, (own, limb, most, loop, _needs, ceilings) in table().items():
        kp = float(round(min(loop.kp, ceilings[loop.ceiling])))
        set_on = limb if loop.need == 'swing' else most
        out[kind] = (kp, float(round(min(2.0 * zeta * sqrt(kp * sqrt(own * set_on)), own * damp_w), 1)))
    return out
