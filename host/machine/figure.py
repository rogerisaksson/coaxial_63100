"""The gynoid's figure: segments, joints and masses; a leg's six joints from its foot, and back.

    SEGMENTS                                   # parents first; `machine.physics` builds on them
    angles = leg(1.0, pelvis, turn, ankle, foot)   # the left leg's six joints, radians
    at = ball(1.0, pelvis, turn, angles)           # where its ball stands, world

Frames: x her left, y up, z ahead; a turn a 3x3 of rows. A joint turns its segment by sign x
angle about its axis; the legs' lengths are `machine.gait`'s.
"""
import math

from machine.gait import ANKLE_H, BALL, HEEL, HIP_DROP, HIP_HALF, SHANK, THIGH

#: Her weight, kg; each segment's share of it is de Leva's (1996) for a woman.
MASS_KG = 55.0

#: The toes' joint over the sole, metres.
TOE_RY = 0.014


def _sides():
    out = []
    for side, x in (('left', 1.0), ('right', -1.0)):
        s = int(x)
        out += [
            (side + '_upper_arm', 'torso', ((side + '_shoulder', 'x', -1),),
             (0.148 * x, 0.325, -0.005), 6.0 * x, 0.0255, (0.0, -0.118, 0.0), (0.075, 0.035, 0.075)),
            (side + '_forearm', side + '_upper_arm', ((side + '_elbow', 'x', -1),),
             (0.0, -0.27, 0.0), 0.0, 0.0138, (0.0, -0.10, 0.0), (0.063, 0.025, 0.063)),
            (side + '_hand', side + '_forearm', ((side + '_wrist', 'x', -1),),
             (0.0, -0.24, 0.0), 0.0, 0.0036, (0.0, -0.043, 0.0), (0.03, 0.015, 0.03)),
            (side + '_fingers', side + '_hand', ((side + '_gripper', 'x', -1),),
             (0.0, -0.086, 0.004), 0.0, 0.002, (0.0, -0.035, 0.0), (0.02, 0.01, 0.02)),
            (side + '_thigh', 'pelvis', ((side + '_hip_yaw', 'y', s), (side + '_hip_roll', 'z', s),
                                        (side + '_hip', 'x', 1)),
             (HIP_HALF * x, -HIP_DROP, 0.0), 0.0, 0.1478, (0.0, -0.36 * THIGH, 0.0),
             (0.144, 0.065, 0.144)),
            (side + '_shank', side + '_thigh', ((side + '_knee', 'x', 1),),
             (0.0, -THIGH, 0.0), 0.0, 0.0481, (0.0, -0.44 * SHANK, 0.0), (0.103, 0.035, 0.103)),
            (side + '_foot', side + '_shank', ((side + '_ankle', 'x', 1), (side + '_ankle_roll', 'z', s)),
             (0.0, -SHANK, 0.0), 0.0, 0.011, (0.0, -0.045, 0.03), (0.055, 0.03, 0.055)),
            (side + '_toes', side + '_foot', ((side + '_foot', 'x', 1),),
             (0.0, -(ANKLE_H - TOE_RY), BALL), 0.0, 0.0019, (0.0, 0.0, 0.03), (0.02, 0.01, 0.02))]
    return out


#: (segment, parent, joints ((joint, axis, sign), ..) applied in order, offset from the parent's
#: frame, rest turn about z (deg), mass share, centre of mass, radii of gyration x y z (m)): parents
#: first. The spine bends forward positive, its roll to her right; a hip's yaw and roll turn the
#: leg out positive, an ankle's roll the sole out.
SEGMENTS = tuple([
    ('pelvis', None, (), (0.0, 0.0, 0.0), 0.0, 0.1247, (0.0, 0.0, 0.0), (0.09, 0.07, 0.08)),
    ('torso', 'pelvis', (('spine', 'x', 1), ('spine_roll', 'z', 1), ('waist', 'y', 1)),
     (0.0, 0.12, 0.0), 0.0, 0.301,
     (0.0, 0.17, 0.0), (0.12, 0.09, 0.11)),
    ('neck', 'torso', (('neck', 'x', 1),), (0.0, 0.385, 0.006), 0.0, 0.006, (0.0, 0.05, 0.0),
     (0.02, 0.02, 0.02)),
    ('head', 'neck', (('head', 'y', 1),), (0.0, 0.09, 0.012), 0.0, 0.0608, (0.0, 0.095, 0.012),
     (0.075, 0.07, 0.075))] + _sides())

#: What touches the floor: (segment, shape, size, centre) in the segment's frame - the soles and
#: toes on their undersides, the knee's front to kneel on, the knuckles to lean on.
CONTACTS = (('foot', 'box', (0.035, 0.01, (BALL + HEEL) / 2.0), (0.0, -ANKLE_H + 0.01, (BALL - HEEL) / 2.0)),
            ('toes', 'box', (0.035, 0.007, 0.0325), (0.0, -0.007, 0.0325)),
            ('shank', 'sphere', (0.035,), (0.0, -0.06, 0.03)),
            ('hand', 'sphere', (0.03,), (0.0, -0.07, 0.01)))

#: Every joint, in the order the segments carry them.
JOINTS = tuple(j for seg in SEGMENTS for j, _axis, _sign in seg[2])

#: A leg's joints after its side, the order `leg` answers them in.
LEG = ('_hip_yaw', '_hip_roll', '_hip', '_knee', '_ankle', '_ankle_roll')

#: The ball's and the heel's points on the sole, the foot's frame.
SOLE_BALL, SOLE_HEEL = (0.0, -ANKLE_H, BALL), (0.0, -ANKLE_H, -HEEL)


def quat(w, x, y, z):
    """The turn of a unit quaternion (w, x, y, z), a 3x3 of rows."""
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
            (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
            (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)))


def rx(a):
    c, s = math.cos(a), math.sin(a)
    return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def mul(a, b):
    """The product of two 3x3s."""
    (a0, a1, a2), (a3, a4, a5), (a6, a7, a8) = a
    (b0, b1, b2), (b3, b4, b5), (b6, b7, b8) = b
    return ((a0 * b0 + a1 * b3 + a2 * b6, a0 * b1 + a1 * b4 + a2 * b7, a0 * b2 + a1 * b5 + a2 * b8),
            (a3 * b0 + a4 * b3 + a5 * b6, a3 * b1 + a4 * b4 + a5 * b7, a3 * b2 + a4 * b5 + a5 * b8),
            (a6 * b0 + a7 * b3 + a8 * b6, a6 * b1 + a7 * b4 + a8 * b7, a6 * b2 + a7 * b5 + a8 * b8))


def t(a):
    return ((a[0][0], a[1][0], a[2][0]), (a[0][1], a[1][1], a[2][1]), (a[0][2], a[1][2], a[2][2]))


def apply(a, v):
    x, y, z = v
    return (a[0][0] * x + a[0][1] * y + a[0][2] * z, a[1][0] * x + a[1][1] * y + a[1][2] * z,
            a[2][0] * x + a[2][1] * y + a[2][2] * z)


def add(*vs):
    return (sum(v[0] for v in vs), sum(v[1] for v in vs), sum(v[2] for v in vs))


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def hip(sign, pelvis, turn):
    """The hip joint's place, world, for the pelvis at `pelvis` turned `turn`."""
    return add(pelvis, apply(turn, (sign * HIP_HALF, -HIP_DROP, 0.0)))


def leg(sign, pelvis, turn, ankle, foot):
    """(hip_yaw, hip_roll, hip, knee, ankle, ankle_roll) radians, the joints' signs, for the ankle
    at `ankle` and the foot turned `foot` with the pelvis at `pelvis` turned `turn` (world): from
    the ankle back to the hip (Kajita's order). Out of reach, the leg straightens toward it."""
    rf = mul(t(turn), foot)
    r = apply(t(foot), sub(hip(sign, pelvis, turn), ankle))
    rho_a = math.atan2(r[0], r[1])
    u_y = math.sin(rho_a) * r[0] + math.cos(rho_a) * r[1]
    length = min(math.hypot(u_y, r[2]), THIGH + SHANK)
    kappa = math.acos(max(-1.0, min(1.0, (length * length - SHANK * SHANK - THIGH * THIGH)
                                    / (2.0 * SHANK * THIGH))))
    theta_a = (math.atan2(-THIGH * math.sin(kappa), SHANK + THIGH * math.cos(kappa))
               - math.atan2(r[2], u_y))
    q = mul(mul(rf, rz(-rho_a)), rx(-theta_a - kappa))
    rho = math.asin(max(-1.0, min(1.0, q[1][0])))
    return (sign * math.atan2(-q[2][0], q[0][0]), sign * rho, math.atan2(-q[1][2], q[1][1]),
            kappa, theta_a, sign * rho_a)


def foot_of(sign, pelvis, turn, angles):
    """(ankle, the foot's turn), world, for the leg's six joints `angles` (as `leg` answers)."""
    yaw, roll, pitch, knee, ankle, ankle_roll = angles
    thigh = mul(mul(mul(turn, ry(sign * yaw)), rz(sign * roll)), rx(pitch))
    at = add(hip(sign, pelvis, turn), apply(thigh, (0.0, -THIGH, 0.0)))
    shank = mul(thigh, rx(knee))
    return (add(at, apply(shank, (0.0, -SHANK, 0.0))),
            mul(mul(shank, rx(ankle)), rz(sign * ankle_roll)))


def ball(sign, pelvis, turn, angles):
    """The ball's point on the sole, world, for the leg's six joints `angles`, the foot rolled flat
    about whichever of its heel and ball is lower."""
    ankle, foot = foot_of(sign, pelvis, turn, angles)
    heel, at = (add(ankle, apply(foot, p)) for p in (SOLE_HEEL, SOLE_BALL))
    if at[1] <= heel[1]:
        return at
    return add(heel, apply(ry(math.atan2(foot[0][2], foot[2][2])), sub(SOLE_BALL, SOLE_HEEL)))


AXES = {'x': rx, 'y': ry, 'z': rz}


def frames(degrees, pelvis, turn):
    """{segment: (place, turn)}, world, for every joint at {joint: degrees} and the pelvis at
    `pelvis` turned `turn`."""
    out = {}
    for name, parent, joints, offset, rest, *_mass in SEGMENTS:
        if parent is None:
            at, here = pelvis, turn
        else:
            above, where = out[parent][1], out[parent][0]
            at = add(where, apply(above, offset))
            here = mul(above, rz(math.radians(rest))) if rest else above
        for joint, axis, sign in joints:
            here = mul(here, AXES[axis](sign * math.radians(degrees.get(joint, 0.0))))
        out[name] = (at, here)
    return out


def com(degrees, pelvis, turn):
    """Her centre of mass, world, for every joint at {joint: degrees} and the pelvis placed."""
    placed = frames(degrees, pelvis, turn)
    total = [0.0, 0.0, 0.0]
    for name, _parent, _joints, _offset, _rest, share, centre, _gyr in SEGMENTS:
        at, here = placed[name]
        point = add(at, apply(here, centre))
        total = [s + share * c for s, c in zip(total, point)]
    return tuple(total)
