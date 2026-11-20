"""The robot's servo table, solved: each joint's loop of what its drive turns and the design's targets.

    rows = looped.table(st)       # {kind: (rotor, limb, most, Loop, needs, ceilings)}, st compile's subtrees
    kp_kd = looped.gains(st)      # {kind: (kp N m/rad, kd N m s/rad)}: compile.servos' stiffness and damping

One rule for every kind, no formula and no damping ratio a joint (`machine.solver.loops`). A drive's
stiffness is the stiffer of two needs: its limb's - what hangs on it with its rotor, followed at the
swing's frequency, the cascade's factor over the stance loop's - and, where it carries the robot, its
stance's - the body it carries held at the fall's rate by its factor, with the stiffness gravity tips
that body by (`carried.table`). A drive's ceilings are its structure's series stiffness and the phase
margin its bus's dead time leaves on the least it sees: its own rotor alone, inside its gearbox's play
(behind a series spring what the rotor's feedforward leaves of it). A drive's damping is the ratio
asked on the geometric mean of that least and what its stiffness is set on - its limb, or the body it
carries -, no more than its rotor alone takes within the phase its dead time may take.
The targets are `control.toml`'s, stated once for every machine. The table's clients are the moves and
the programs a robot plays on its boards' servos (`machine.process.joints`: the chair rise's scoot, the
bed's moves) and the stand's campaign (`machine.identify.campaign`); the robot's law standing and
walking runs on none of it (`machine.process.sprung`: its gains are zero there).
"""
from math import radians, sqrt

from machine.body import carried, drives, figure, mechatronics
from machine.solver import loops

#: A sampled loop's dead time in its bus's periods: half of one its hold, one the answer's way back.
DEAD_PERIODS = 1.5
#: What its law's feedforward leaves of a rotor's inertia for the loop to feel: a law's period in the bus's.
ROTOR_FF = round(1.0 / (1.0 + float(drives.LAW_DIV)), 2)


def _joint(kind):
    return kind if kind in figure.JOINTS else 'left_' + kind


def rotor(kind):
    """A kind's drive's own inertia at its joint, kg m^2: its rotor and gearbox through its ratio."""
    joint = _joint(kind)
    own = drives.armature(joint)
    if own > 0.0:
        return own
    box = drives._box(joint)
    sh = float(box.get('input_j_share', drives.wave_box(box).input_j_share))
    return drives.motors(joint) * (1.0 + sh) * drives.size(kind).rotor * drives.ratio(joint) ** 2


def felt(kind):
    """The armature a kind's loop feels, kg m^2: what the rotor's feedforward leaves of its rotor's."""
    joint = _joint(kind)
    box = drives._box(joint)
    sh = float(box.get('input_j_share', drives.wave_box(box).input_j_share))
    base = (ROTOR_FF + sh) * drives.size(kind).rotor * float(box['ratio']) ** 2
    return min(base, (ROTOR_FF + sh) / (1.0 + sh) * drives.armature(joint)) if drives.series(joint) > 0.0 else base


def table(st):
    """{kind: (rotor, limb, most - kg m^2 -, its Loop, its needs, its ceilings)} of every driven kind and
    the hand's two, off the compiled subtrees `st` and what each drive turns (`carried.table`)."""
    ctl = mechatronics.control()
    zeta, stance_w = float(ctl['damping_ratio']), float(ctl['fall_factor']) * st['omega']
    swing_w = float(ctl['cascade_factor']) * stance_w
    taken = radians(float(ctl['delay_phase_deg']))
    dead_w = loops.natural(DEAD_PERIODS / drives.BUS_HZ, zeta, loops.margin(zeta) - taken)
    gravity = float(mechatronics.environment()['g_m_s2'])
    limbs = {**carried.table(gravity), 'wrist': ((st['hand'][2], 0.0, 0.0),),
             'gripper': ((st['fingers'][2], 0.0, 0.0),)}
    for k in mechatronics.robot()['stacks']:
        if k != 'foot' and k not in limbs:
            limbs[k] = ((carried.swing(k), 0.0, 0.0),)
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


def gains(st):
    """{kind: (kp N m/rad, kd N m s/rad)}: each loop's stiffness under its least ceiling, its damping the
    ratio asked on the mean of its rotor alone and what its stiffness is set on - its limb where the swing
    set it, the body it carries where its stance did -, no more than its rotor's dead time takes."""
    ctl = mechatronics.control()
    zeta = float(ctl['damping_ratio'])
    damp_w = radians(float(ctl['delay_phase_deg'])) * drives.BUS_HZ / DEAD_PERIODS
    out = {}
    for kind, (own, limb, most, loop, _needs, ceilings) in table(st).items():
        kp = float(round(min(loop.kp, ceilings[loop.ceiling])))
        set_on = limb if loop.need == 'swing' else most
        out[kind] = (kp, float(round(min(2.0 * zeta * sqrt(kp * sqrt(own * set_on)), own * damp_w), 1)))
    return out
