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

#: Up, the world's: gravity pulls against it. A joint's axis by its letter, in its own frame.
UP = figure.IDENT[1]
ALONG = dict(zip('xyz', figure.IDENT))


def about(segs, placed, names, pivot, axis):
    """(mass kg, its centre's distance from the axis m, inertia kg m^2) of the segments `names` as one
    body about the line through `pivot` along `axis` (unit), world."""
    present = [n for n in names if n in segs]
    m = sum(segs[n][0] for n in present)
    com = [sum(segs[n][0] * figure.add(placed[n][0], figure.apply(placed[n][1], segs[n][1]))[i]
               for n in present) / m for i in range(3)]
    j_ax = 0.0
    for n in present:
        kg, c, (ixx, iyy, izz, ixy, ixz, iyz) = segs[n]
        at, rot = placed[n]
        pw = figure.add(at, figure.apply(rot, c))
        i_w = figure.mul(figure.mul(rot, ((ixx, ixy, ixz), (ixy, iyy, iyz), (ixz, iyz, izz))), figure.t(rot))
        d = [pw[i] - pivot[i] for i in range(3)]
        du = sum(d[i] * axis[i] for i in range(3))
        u_iu = sum(axis[i] * sum(i_w[i][k] * axis[k] for k in range(3)) for i in range(3))
        j_ax += u_iu + kg * (sum(v * v for v in d) - du * du)
    dc = [com[i] - pivot[i] for i in range(3)]
    du_c = sum(dc[i] * axis[i] for i in range(3))
    return m, (max(0.0, sum(v * v for v in dc) - du_c * du_c)) ** 0.5, j_ax


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
    """The postures the table is laid in, {joint: deg} each: the robot standing and in its deep squat
    (`figure.SQUAT`), the ends of what its moves fold its legs through. Laid on the leg standing alone, a
    hip's yaw (170 N m/rad) gave 5.9 degrees under the bed's sit-up and the robot slid off the bed's
    side to the floor; on the folded leg too (523), it stood up on its soles, 274 N."""
    return (figure.stand(), figure.SQUAT)


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
