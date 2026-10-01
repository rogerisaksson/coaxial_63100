"""The gynoid's figure: segments, joints and masses; a leg's six joints from its foot, and back.

    SEGMENTS                                   # parents first; `machine.physics` builds on them
    angles = leg(1.0, pelvis, turn, ankle, foot)   # the left leg's six joints, radians
    at = ball(1.0, pelvis, turn, angles)           # where its ball stands, world

Frames: x her left, y up, z ahead; a turn a 3x3 of rows. A joint turns its segment by sign x
angle about its axis; the legs' lengths are `machine.gait`'s.
"""
import math

from machine.gait import ANKLE_H, BALL, HEEL, HIP_DROP, HIP_HALF, SHANK, THIGH, TOE_M, TOE_RY

#: Her weight, kg; each segment's share of it is de Leva's (1996) for a woman.
MASS_KG = 55.0

#: Her jeans' wide legs hang from HEM_AT under the knees, metres (`physics.HEMS`).
HEM_AT = 0.12
#: Her hair's fall hangs from HAIR_AT on her head, its frame, metres (`physics.HAIRS`): at the
#: back of her skull, level with her ears.
HAIR_AT = (0.0, 0.075, -0.02)

#: Her upper arm and forearm, shoulder to elbow and elbow to wrist, m: 0.27 and 0.24 until
#: 2026-10-01, a little longer to push herself up by.
UPPER_ARM, FOREARM = 0.28, 0.25


def _sides():
    out = []
    for side, x in (('left', 1.0), ('right', -1.0)):
        s = int(x)
        out += [
            (side + '_upper_arm', 'torso', ((side + '_shoulder', 'x', -1),),
             (0.148 * x, 0.325, -0.005), 6.0 * x, 0.0255, (0.0, -0.437 * UPPER_ARM, 0.0),
             (0.075, 0.035, 0.075)),
            (side + '_forearm', side + '_upper_arm', ((side + '_elbow', 'x', -1),),
             (0.0, -UPPER_ARM, 0.0), 0.0, 0.0138, (0.0, -0.417 * FOREARM, 0.0),
             (0.063, 0.025, 0.063)),
            (side + '_hand', side + '_forearm', ((side + '_wrist', 'x', -1),),
             (0.0, -FOREARM, 0.0), 0.0, 0.0036, (0.0, -0.043, 0.0), (0.03, 0.015, 0.03)),
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
    ('head', 'neck', (('head', 'y', 1),), (0.0, 0.065, 0.012), 0.0, 0.0608, (0.0, 0.095, 0.012),
     (0.075, 0.07, 0.075))] + _sides())

#: The sole's half-width, metres.
SOLE_HALF = 0.038

#: The soles and the toes, their undersides: (segment, 'box', half sizes, centre), its frame.
CONTACTS = (('foot', 'box', (SOLE_HALF, 0.03, (BALL + HEEL) / 2.0), (0.0, -ANKLE_H + 0.03, (BALL - HEEL) / 2.0)),
            ('toes', 'box', (0.036, 0.015, TOE_M / 2.0), (0.0, 0.001, TOE_M / 2.0)))

#: The seat's two buttocks, capsules along her way SEAT_R round at SEAT_Y from SEAT_Z to SEAT_Z:
#: sat on one 0.10 m sphere, the heels in and leaning 15 degrees on, she rolled onto her back;
#: on these she sat (2026-09-30). The fist, FIST_R at FIST_AT, the knuckles to lean on.
SEAT_R, SEAT_X, SEAT_Y, SEAT_Z = 0.06, 0.055, -0.05, (-0.06, 0.01)
FIST_R, FIST_AT = 0.03, (0.0, -0.07, 0.01)

#: Her body, on the floor and on itself: (segment, radius, from, to), its frame - a capsule, a
#: sphere where the ends meet - the drawn skin, the clothes not: the hips across, the waist, the
#: ribs, the chest and the shoulders across, the bust, the neck, the skull and the jaw, every
#: limb its length. A sphere or two a segment, lying her arms went 40-43 mm into the floor, her
#: head 40, her shins 100 through each other and her hand 41 into its upper arm; on these, 2-9
#: (2026-09-30).
BODY = (('pelvis', SEAT_R, (SEAT_X, SEAT_Y, SEAT_Z[0]), (SEAT_X, SEAT_Y, SEAT_Z[1])),
        ('pelvis', SEAT_R, (-SEAT_X, SEAT_Y, SEAT_Z[0]), (-SEAT_X, SEAT_Y, SEAT_Z[1])),
        ('pelvis', 0.09, (-0.06, -0.01, -0.01), (0.06, -0.01, -0.01)),
        ('torso', 0.066, (-0.03, 0.0, 0.0), (0.03, 0.0, 0.0)),
        ('torso', 0.08, (-0.038, 0.15, 0.0), (0.038, 0.15, 0.0)),
        ('torso', 0.078, (-0.052, 0.26, 0.0), (0.052, 0.26, 0.0)),
        ('torso', 0.066, (-0.074, 0.33, 0.0), (0.074, 0.33, 0.0)),
        ('torso', 0.045, (0.055, 0.208, 0.058), (0.055, 0.208, 0.058)),
        ('torso', 0.045, (-0.055, 0.208, 0.058), (-0.055, 0.208, 0.058)),
        ('neck', 0.04, (0.0, 0.0, 0.0), (0.0, 0.07, 0.0)),
        ('head', 0.085, (0.0, 0.095, 0.012), (0.0, 0.095, 0.012)),
        ('head', 0.045, (0.0, 0.042, 0.03), (0.0, 0.042, 0.03)),
        ('upper_arm', 0.028, (0.0, -0.02, 0.0), (0.0, 0.02 - UPPER_ARM, 0.0)),
        ('forearm', 0.022, (0.0, -0.02, 0.0), (0.0, 0.02 - FOREARM, 0.0)),
        ('hand', FIST_R, FIST_AT, FIST_AT),
        ('fingers', 0.014, (0.0, -0.01, 0.0), (0.0, -0.06, 0.0)),
        ('thigh', 0.058, (0.0, -0.03, 0.0), (0.0, -0.36, 0.0)),
        ('shank', 0.05, (0.0, -0.04, 0.0), (0.0, -0.25, 0.0)),
        ('shank', 0.036, (0.0, -0.25, 0.0), (0.0, -0.33, 0.0)))

#: Pads where her falls land, 5 mm of gel in TPU under her skin and clothes (`physics.PAD_*`):
#: (segment, the point of its capsule's axis under it, the capsule's radius, toward where the
#: landings cluster, the pad's radius and half-width across), the left side's. Over 26 bare falls,
#: each segment's first 0.2 s on the floor: the elbow's point 3068 N s, 7.2 kN at most; the hip's
#: front and side 2800, 8.4; the knee 1300, 7.4; the rest of her under 240 N s (2026-10-01). The
#: knee's across its front: on a sphere there, and one at the thigh's end, knees under rolled her
#: 22-84 degrees, 5 of 12 falls up (2026-10-01).
PADS = (('upper_arm', (0.0, 0.02 - UPPER_ARM, 0.0), 0.028, (0.01, -UPPER_ARM, -0.01), 0.023, 0.0),
        ('forearm', (0.0, -0.02, 0.0), 0.022, (0.0, -0.01, -0.01), 0.017, 0.0),
        ('pelvis', (0.06, -0.01, -0.01), 0.09, (0.07, -0.02, 0.07), 0.085, 0.0),
        ('pelvis', (0.06, -0.01, -0.01), 0.09, (0.13, -0.03, 0.03), 0.085, 0.0),
        ('thigh', (0.0, -0.03, 0.0), 0.058, (0.05, -0.03, 0.02), 0.053, 0.0),
        ('shank', (0.0, -0.04, 0.0), 0.05, (0.0, -0.03, 0.04), 0.025, 0.025))
#: A pad stands PAD_M proud of the skin.
PAD_M = 0.005


def pad(axis, radius, toward, size, x=1.0):
    """A pad's centre: its face, `size` round, PAD_M proud of its capsule toward `toward`; `x` -1
    the right side's."""
    d = [t - a for t, a in zip(toward, axis)]
    n = math.sqrt(sum(v * v for v in d)) or 1.0
    c = [a + (radius + PAD_M - size) * v / n for a, v in zip(axis, d)]
    return c[0] * x, c[1], c[2]


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
    # Sat, the hip at or under the ankle: atan2's other root turned the leg 179 degrees about its
    # yaw, the knee 0.24 m through the floor (2026-09-30).
    rho_a = math.atan2(r[0], r[1]) if r[1] > 0.0 else math.atan2(-r[0], -r[1])
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
