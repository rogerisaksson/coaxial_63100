"""What each of the robot's drives turns, a posture at a time: its limb, the body it carries planted.

    seen = carried.table(gravity)     # {kind: ((swing kg m^2, stance kg m^2, gravity N m/rad), ..)}, a posture each

No formula a joint: every drive's inertias are read off the robot's own tree of segments as built
(`build.segments`, `figure.frames`). A joint turns what hangs on it - its own stage and those after it,
its segment and every segment below - about its axis: with the rest of the robot held that is its limb
swinging. Where what hangs on it ends in a sole, the sole planted turns the same joint the other way
round: it carries the rest of the robot about its axis - a stance joint's inertia -, and gravity tips
that rest over it with the stiffness m g times its centre's height over the joint, across the axis.
The links' inertias alone: a drive's own rotor is its compiler's to add. Kinds by their left side's joint,
a row a posture (`postures`).
"""
from machine.body import build, drives, figure, mechatronics
from machine.body.compile import crb as about

#: Up, the world's: gravity pulls against it. A joint's axis by its letter, in its own frame.
UP = figure.IDENT[1]
ALONG = dict(zip('xyz', figure.IDENT))


def _hung(segs):
    """{joint: (its segment, the bodies that hang on it)}: its own stage and those after it of its
    segment, the segment, and every segment below with its stages."""
    kids, stages = {}, {}
    for name, parent, joints, *_rest in figure.SEGMENTS:
        kids.setdefault(parent, []).append(name)
        stages[name] = [joint for joint, _axis, _sign in joints]
    below = {}
    for name, *_rest in reversed(figure.SEGMENTS):
        own = [name] + [stage for stage in stages[name][:-1] if stage in segs]
        below[name] = own + [body for kid in kids.get(name, ()) for body in below[kid]]
    hung = {}
    for name, _parent, joints, *_rest in figure.SEGMENTS:
        for k, (joint, _axis, _sign) in enumerate(joints):
            before = set(stages[name][:k])
            hung[joint] = (name, [b for b in below[name] if b not in before])
    return hung


def postures():
    """The postures the table is laid in, {joint: deg} each: the robot standing. In its deep squat a folded
    leg stands twenty times further from its hip's yaw: a stiffness set there over-damps the leg standing,
    and the campaign on the stand read that drive's friction 24 % off (test_sysid)."""
    return (figure.stand(),)


def table(gravity):
    """{kind: ((swing, stance, gravity), ..)} of every driven kind, a row a posture (`postures`): the
    inertia of what its drive turns with the rest held, kg m^2; the inertia of the rest about its axis
    where a sole hangs on it, else 0; and the stiffness, N m/rad, `gravity` (m/s^2) tips that rest over
    it with."""
    segs, origin = build.segments(), (0.0, figure.standing(), 0.0)
    hung, soles = _hung(segs), {side + '_' + figure.CONTACTS[0][0] for side in ('left', 'right')}
    axes = {joint: axis for _name, _parent, joints, *_rest in figure.SEGMENTS for joint, axis, _sign in joints}
    out = {}
    for pose in postures():
        placed = figure.frames(pose, origin, figure.IDENT)
        for kind in mechatronics.robot()['stacks']:
            joint = kind if kind in figure.JOINTS else 'left_' + kind
            if drives.passive(joint):
                continue
            name, moved = hung[joint]
            pivot = placed[name][0]
            axis = figure.apply(placed[joint if joint in placed else name][1], ALONG[axes[joint]])
            swing = about(segs, placed, moved, pivot, axis)[2]
            stance = tip = 0.0
            if soles & set(moved):
                rest = [n for n in segs if n not in moved]
                mass, _lever, stance = about(segs, placed, rest, pivot, axis)
                centre = [sum(segs[n][0] * figure.add(placed[n][0], figure.apply(placed[n][1], segs[n][1]))[i]
                              for n in rest) / mass - pivot[i] for i in range(3)]
                along = sum(centre[i] * axis[i] for i in range(3))
                tip = max(0.0, mass * gravity * sum((centre[i] - along * axis[i]) * UP[i] for i in range(3)))
            out[kind] = out.get(kind, ()) + ((swing, stance, tip),)
    return out
