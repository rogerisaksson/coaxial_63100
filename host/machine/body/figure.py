"""The gynoid's figure: segments, joints and masses; a leg's six joints from its foot, and back.

    SEGMENTS                                   # parents first; `machine.world.physics` builds on them
    angles = leg(1.0, pelvis, turn, ankle, foot)   # the left leg's six joints, radians
    at = ball(1.0, pelvis, turn, angles)           # where its ball stands, world

Frames: x her left, y up, z ahead; a turn a 3x3 of rows. A joint turns its segment by sign x
angle about its axis.
"""
import math

#: Her legs, metres (`coaxial.graphics.gynoid` is built on them): the hips apart (half) and
#: below the pelvis's centre, thigh and shank, the ankle over the sole, the ball ahead of it
#: (the toes' joint) and the heel behind.
HIP_HALF, HIP_DROP, THIGH, SHANK = 0.095, 0.055, 0.39, 0.38
#: A 24 cm sole with the toes (CONTACTS), a sneaker in size 37-38 - 27 cm read as boats; a
#: 38's, 237.
ANKLE_H, BALL, HEEL = 0.075, 0.117, 0.055
#: The toes' joint over the sole, and the toes' length from it.
TOE_RY, TOE_M = 0.014, 0.065
#: A stand's soft knee, degrees; the hip and the ankle take half of it each way, the shin upright.
STAND_KNEE = 6.0
#: Her deep squat, deg: what of her touches in it is kept apart as what touches standing
#: (`machine.body.mjcf`).
SQUAT = {
    'spine_roll': 0.0, 'spine': 45.0, 'waist': 0.0, 'neck': 35.0, 'head': 0.0,
    'left_shoulder': 35.0, 'left_elbow': 70.0, 'left_wrist': 10.0, 'left_gripper': 30.0,
    'left_hip_yaw': -1.107, 'left_hip_roll': -2.372, 'left_hip': -115.939, 'left_knee': 123.069,
    'left_ankle': -32.153, 'left_ankle_roll': 2.617, 'left_foot': 0.0, 'right_shoulder': 60.0,
    'right_elbow': 30.0, 'right_wrist': 0.0, 'right_gripper': 80.0, 'right_hip_yaw': -1.198,
    'right_hip_roll': -2.567, 'right_hip': -115.935, 'right_knee': 123.061, 'right_ankle': -32.153,
    'right_ankle_roll': 2.832, 'right_foot': 0.0}

#: Her weight, kg (gynoid_mesomorph_medium, 165 cm); each segment's share of it is de Leva's
#: (1996) for a woman. 55 kg is chosen so the structural chassis is robust rather than overly
#: fragile, while providing sufficient internal and external envelope for all structural 3D
#: prints, actuators, harmonic drives, battery packs, TPU gel pads, and clothing.
MASS_KG = 55.0

#: Her jeans' wide legs hang from HEM_AT under the knees, metres (`mjcf.HEMS`).
HEM_AT = 0.12
#: Her hair's fall hangs from HAIR_AT on her head, its frame, metres (`mjcf.HAIRS`): at the
#: back of her skull, level with her ears.
HAIR_AT = (0.0, 0.075, -0.02)

#: Her upper arm and forearm, shoulder to elbow and elbow to wrist, m: 0.27 and 0.24 until
#: A little longer to push herself up by.
UPPER_ARM, FOREARM = 0.28, 0.25

#: Her forearm's carrying angle, deg: out from the upper arm's line straight, none bent 90, a
#: woman's 10-16.
CARRY_DEG = 12.0


def _sides():
    out = []
    for side, x in (('left', 1.0), ('right', -1.0)):
        s = int(x)
        out += [
            (side + '_upper_arm', 'torso', ((side + '_shoulder', 'x', -1),),
             (0.148 * x, 0.325, -0.005), 6.0 * x, 0.0255, (0.0, -0.437 * UPPER_ARM, 0.0),
             (0.075, 0.035, 0.075)),
            (side + '_forearm', side + '_upper_arm', ((side + '_elbow', 'x', -1),),
             (0.0, -UPPER_ARM, 0.0), CARRY_DEG * x, 0.0138, (0.0, -0.417 * FOREARM, 0.0),
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
#: first. The spine bends forward positive, its roll to her right, the roll first - its drive on
#: the pelvis on a spur pair (`machine.body.drives`), she leans aside in her pelvis's frame, then bends;
#: a hip's yaw and roll turn the leg out positive, an ankle's roll the sole out.
SEGMENTS = tuple([
    ('pelvis', None, (), (0.0, 0.0, 0.0), 0.0, 0.1247, (0.0, 0.0, 0.0), (0.09, 0.07, 0.08)),
    ('torso', 'pelvis', (('spine_roll', 'z', 1), ('spine', 'x', 1), ('waist', 'y', 1)),
     (0.0, 0.12, 0.0), 0.0, 0.301,
     (0.0, 0.17, 0.0), (0.12, 0.09, 0.11)),
    ('neck', 'torso', (('neck', 'x', 1),), (0.0, 0.385, 0.006), 0.0, 0.006, (0.0, 0.05, 0.0),
     (0.02, 0.02, 0.02)),
    ('head', 'neck', (('head', 'y', 1),), (0.0, 0.065, 0.012), 0.0, 0.0608, (0.0, 0.095, 0.012),
     (0.075, 0.07, 0.075))] + _sides())

#: The sole's half-width, metres.
SOLE_HALF = 0.045

#: The soles and the toes, their undersides: (segment, shape, sizes, centre), its frame, a
#: sneaker's 90 wide. Its heel's edge as two 12 mm spheres, to roll the strike, doubled it
#: (1634 N against 1108) and the parry never caught: 797 and 49.8 % against 604 and 73.7.
CONTACTS = (('foot', 'box', (SOLE_HALF, 0.03, (BALL + HEEL) / 2.0),
             (0.0, -ANKLE_H + 0.03, (BALL - HEEL) / 2.0)),
            ('toes', 'box', (0.045, 0.015, TOE_M / 2.0), (0.0, 0.001, TOE_M / 2.0)))

#: Her feet as point contacts, a sphere POINT_R under each sole's centre alone (`contacts`): the
#: stilts her standing counts on (`machine.stand`), 1 to build her so.
POINT_FEET, POINT_R = 0.0, 0.01


def contacts():
    """The undersides as built: CONTACTS, or a point under each sole's centre (POINT_FEET)."""
    if POINT_FEET:
        return (('foot', 'sphere', (POINT_R,), (0.0, -ANKLE_H + POINT_R, (BALL - HEEL) / 2.0)),)
    return CONTACTS

#: The seat's two buttocks, capsules along her way SEAT_R round at SEAT_Y from SEAT_Z to SEAT_Z:
#: sat on one 0.10 m sphere, the heels in and leaning 15 degrees on, she rolled onto her back;
#: on these she sat. The fist, FIST_R at FIST_AT, the knuckles to lean on.
SEAT_R, SEAT_X, SEAT_Y, SEAT_Z = 0.07, 0.068, -0.058, (-0.078, 0.0)
FIST_R, FIST_AT = 0.03, (0.0, -0.07, 0.01)

#: Her body, on the floor and on itself: (segment, radius, from, to), its frame - a capsule, a
#: sphere where the ends meet - the drawn skin, the clothes not: the hips across, the waist, the
#: ribs, the chest and the shoulders across, the bust, the neck, the skull and the jaw, every
#: limb its length. A sphere or two a segment, lying her arms went 40-43 mm into the floor, her
#: head 40, her shins 100 through each other and her hand 41 into its upper arm; on these, 2-9.
BODY = (('pelvis', SEAT_R, (SEAT_X, SEAT_Y, SEAT_Z[0]), (SEAT_X, SEAT_Y, SEAT_Z[1])),
        ('pelvis', SEAT_R, (-SEAT_X, SEAT_Y, SEAT_Z[0]), (-SEAT_X, SEAT_Y, SEAT_Z[1])),
        ('pelvis', 0.095, (-0.07, -0.012, -0.02), (0.07, -0.012, -0.02)),
        ('torso', 0.07, (-0.032, 0.0, 0.0), (0.032, 0.0, 0.0)),
        ('torso', 0.08, (-0.038, 0.15, 0.0), (0.038, 0.15, 0.0)),
        ('torso', 0.078, (-0.052, 0.26, 0.0), (0.052, 0.26, 0.0)),
        ('torso', 0.066, (-0.074, 0.33, 0.0), (0.074, 0.33, 0.0)),
        ('torso', 0.052, (0.056, 0.208, 0.062), (0.056, 0.208, 0.062)),
        ('torso', 0.052, (-0.056, 0.208, 0.062), (-0.056, 0.208, 0.062)),
        ('neck', 0.045, (0.0, 0.0, 0.0), (0.0, 0.07, 0.0)),
        ('head', 0.085, (0.0, 0.095, 0.012), (0.0, 0.095, 0.012)),
        ('head', 0.045, (0.0, 0.042, 0.03), (0.0, 0.042, 0.03)),
        ('upper_arm', 0.034, (0.0, -0.02, 0.0), (0.0, 0.02 - UPPER_ARM, 0.0)),
        ('forearm', 0.025, (0.0, -0.02, 0.0), (0.0, 0.02 - FOREARM, 0.0)),
        ('hand', FIST_R, FIST_AT, FIST_AT),
        ('fingers', 0.014, (0.0, -0.01, 0.0), (0.0, -0.06, 0.0)),
        ('thigh', 0.072, (0.012, -0.04, 0.022), (0.012, -0.22, 0.022)),
        ('thigh', 0.058, (0.006, -0.22, 0.012), (0.0, -0.36, 0.0)),
        ('shank', 0.05, (0.0, -0.04, 0.0), (0.0, -0.25, 0.0)),
        ('shank', 0.036, (0.0, -0.25, 0.0), (0.0, -0.33, 0.0)))

#: Her widest half across, m: her trunk's capsules' reach along x (BODY's hips row), what an
#: aperture must clear each side of her - her arms hang within it.
WIDE_HALF = max(max(abs(fro[0]), abs(to[0])) + r for seg, r, fro, to in BODY
                if seg in ('pelvis', 'torso', 'neck', 'head'))

#: Pads where her falls land, 5 mm of gel in TPU under her skin and clothes (`mjcf.PAD_*`):
#: (segment, the point of its capsule's axis under it, the capsule's radius, toward where the
#: landings cluster, the pad's radius and half-width across), the left side's. Over 26 bare falls,
#: each segment's first 0.2 s on the floor: the elbow's point 3068 N s, 7.2 kN at most; the hip's
#: front and side 2800, 8.4; the knee 1300, 7.4; the rest of her under 240 N s. The
#: knee's across its front: on a sphere there, and one at the thigh's end, knees under rolled her
#: 22-84 degrees, 5 of 12 falls up.
PADS = (('upper_arm', (0.0, 0.02 - UPPER_ARM, 0.0), 0.034, (0.01, -UPPER_ARM, -0.01), 0.028, 0.0),
        ('forearm', (0.0, -0.02, 0.0), 0.025, (0.0, -0.01, -0.01), 0.02, 0.0),
        ('pelvis', (0.07, -0.012, -0.02), 0.095, (0.08, -0.022, 0.06), 0.09, 0.0),
        ('pelvis', (0.07, -0.012, -0.02), 0.095, (0.14, -0.032, 0.02), 0.09, 0.0),
        ('thigh', (0.012, -0.04, 0.022), 0.072, (0.062, -0.04, 0.042), 0.066, 0.0),
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

#: Each segment's joints before its last, each a stage, the frame it turns (`frames`) - not the
#: toes' joint, named for the foot.
STAGES = tuple(j for seg in SEGMENTS for j, _axis, _sign in seg[2][:-1])

#: A leg's joints after its side, the order `leg` answers them in.
LEG = ('_hip_yaw', '_hip_roll', '_hip', '_knee', '_ankle', '_ankle_roll')

#: The ball's and the heel's points on the sole, the foot's frame.
SOLE_BALL, SOLE_HEEL = (0.0, -ANKLE_H, BALL), (0.0, -ANKLE_H, -HEEL)

#: No turn.
IDENT = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


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


def cross(a, b):
    """a x b."""
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


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
    # yaw, the knee 0.24 m through the floor.
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
    `pelvis` turned `turn` - and each stage, a segment's joint before its last by its name, the
    frame that joint turns (a gimbal's, `drives.mount`)."""
    out = {}
    for name, parent, joints, offset, rest, *_mass in SEGMENTS:
        if parent is None:
            at, here = pelvis, turn
        else:
            above, where = out[parent][1], out[parent][0]
            at = add(where, apply(above, offset))
            here = mul(above, rz(math.radians(rest))) if rest else above
        for k, (joint, axis, sign) in enumerate(joints):
            here = mul(here, AXES[axis](sign * math.radians(degrees.get(joint, 0.0))))
            if k < len(joints) - 1:
                out[joint] = (at, here)
        out[name] = (at, here)
    return out


def com(degrees, pelvis, turn):
    """Her centre of mass, world, for every joint at {joint: degrees} and the pelvis placed."""
    placed = frames(degrees, pelvis, turn)
    total = [0.0, 0.0, 0.0]
    for name, (share, centre) in shares().items():
        at, here = placed[name]
        point = add(at, apply(here, centre))
        total = [s + share * c for s, c in zip(total, point)]
    return tuple(total)


def shares():
    """{segment: (share of her mass, centre in its frame)}: as built (`machine.body.build`) or a
    woman's."""
    from machine.body import build
    if not build.SHELLS:
        return {s[0]: (s[5], s[6]) for s in SEGMENTS}
    built = build.segments()
    total = sum(kg for kg, _c, _i in built.values())
    return {name: (kg / total, centre) for name, (kg, centre, _i) in built.items()}


def stand():
    """Every joint at rest, deg: the knees soft, arms down and soft at the elbow, the fingers
    curled."""
    out = {name: 0.0 for name in JOINTS}
    out.update({'left_elbow': 10.0, 'right_elbow': 10.0, 'left_wrist': 5.0, 'right_wrist': 5.0,
                'left_gripper': 18.0, 'right_gripper': 18.0})
    for side in ('left', 'right'):
        out.update({side + '_hip': -STAND_KNEE / 2.0, side + '_knee': STAND_KNEE,
                    side + '_ankle': -STAND_KNEE / 2.0})
    return out


def pelvis_at(knee):
    """Her pelvis's height on flat soles with her knees bent `knee` rad, m: her legs' geometry."""
    return ANKLE_H + math.sqrt(THIGH ** 2 + SHANK ** 2 + 2.0 * THIGH * SHANK * math.cos(knee)) + HIP_DROP


def standing():
    """Her pelvis's height at a stand, m: both feet flat under her hips, the knees as `stand`
    bends them."""
    return pelvis_at(math.radians(STAND_KNEE))


def mass():
    """Her weight, kg: as built (`machine.body.build`) or MASS_KG."""
    from machine.body import build
    return build.mass() if build.SHELLS else MASS_KG


def fist(degrees, pelvis=(0.0, 0.0, 0.0), turn=IDENT, side='left'):
    """Her fist's centre, world, at {joint: degrees}, the pelvis placed: what her hand bears on."""
    at, here = frames(degrees, pelvis, turn)[side + '_hand']
    return add(at, apply(here, FIST_AT))


#: Her fist's centre aside from her middle, m, her arm hanging as she stands.
ASIDE = fist(stand())[0]


def palm(degrees, pelvis=(0.0, 0.0, 0.0), turn=IDENT, side='left'):
    """Her hand's lowest point, world, at {joint: degrees}, the pelvis placed: of its fist's and its
    fingers' segments (`BODY`), what of it meets what she reaches down to - curled, her fingers."""
    placed, lows = frames(degrees, pelvis, turn), []
    for seg, r, a, b in BODY:
        if seg in ('hand', 'fingers'):
            at, here = placed[side + '_' + seg]
            low = min((add(at, apply(here, a)), add(at, apply(here, b))), key=lambda p: p[1])
            lows.append((low[0], low[1] - r, low[2]))
    return min(lows, key=lambda p: p[1])
