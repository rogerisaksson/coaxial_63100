"""The robot's figure: segments, joints and masses; a leg's six joints from its foot, and back.

    SEGMENTS                                   # parents first; `machine.world.physics` builds on them
    angles = leg(1.0, pelvis, turn, ankle, foot)   # the left leg's six joints, radians
    at = ball(1.0, pelvis, turn, angles)           # where its ball stands, world

Frames: x the robot's left, y up, z ahead; a turn a 3x3 of rows. A joint turns its segment by sign x
angle about its axis.
"""
import functools
import math

from machine.body import mechatronics

#: The robot's figure in its build (`[figure]`, `machine.body.mechatronics`), metres of its own: every
#: length of its own `scale` times its own (1 the robot's 165 cm), its angles, its mass shares
#: and its pads' gel as they are; its masses as built follow its capsules
#: (`machine.body.build`).
_F = mechatronics.robot()['figure']
SCALE = float(_F.get('scale', 1.0))

#: The robot's legs, metres (`coaxial.graphics.gynoid` is built on them): the hips apart (half) and
#: below the pelvis's centre, thigh and shank, the ankle over the sole, the ball ahead of it
#: (the toes' joint) and the heel behind.
HIP_HALF, HIP_DROP, THIGH, SHANK = (SCALE * _F[k] for k in ('hip_half', 'hip_drop', 'thigh', 'shank'))
ANKLE_H, BALL, HEEL = (SCALE * _F[k] for k in ('ankle_h', 'ball', 'heel'))
#: The toes' joint over the sole, and the toes' length from it.
TOE_RY, TOE_M = SCALE * _F['toe_ry'], SCALE * _F['toe_m']
#: A stand's soft knee, degrees; the hip and the ankle take half of it each way, the shin upright.
STAND_KNEE = float(_F.get('stand_knee', round(math.degrees(math.atan2(ANKLE_H, THIGH + SHANK)))))

#: The robot's weight as a woman, kg, at SCALE as its cube; each segment's share of it de Leva's (`SEGMENTS`).
MASS_KG = _F['mass_kg'] * SCALE ** 3

#: The robot's jeans' wide legs hang from HEM_AT under the knees, metres (`mjcf.HEMS`); its hair's
#: fall from HAIR_AT on its head, its frame (`mjcf.HAIRS`).
HEM_AT = SCALE * _F['hem_at']
HAIR_AT = tuple(SCALE * m for m in _F['hair_at'])

#: The robot's upper arm and forearm, shoulder to elbow and elbow to wrist, m; its forearm's carrying angle,
#: deg.
UPPER_ARM, FOREARM = SCALE * _F['upper_arm'], SCALE * _F['forearm']
CARRY_DEG = float(_F['carry_deg'])
#: Each limb's length by its segment: what a capsule or a pad along it is measured from its far joint by.
_LONG = {'upper_arm': UPPER_ARM, 'forearm': FOREARM, 'thigh': THIGH, 'shank': SHANK}
#: The robot's segments' own numbers by name (`[figure.segments]`), the left side's.
_SEG = _F['segments']

def _m(*lengths):
    """`lengths`, metres of the robot's own, at SCALE."""
    return tuple(SCALE * m for m in lengths)


def _vec3(v):
    """3D offset `(x, y, z)` at `SCALE`: zero when omitted, sagittal `(0, y, 0)` when scalar."""
    if v is None:
        return (0.0, 0.0, 0.0)
    if isinstance(v, (int, float)):
        return _m(0.0, float(v), 0.0)
    return _m(float(v[0]), float(v[1]), float(v[2]))


def _gyr3(g):
    """3D radii of gyration `(gx, gy, gz)` at `SCALE`: isotropic scalar, axisymmetric `[g_perp, gy]`, or 3-tuple."""
    if isinstance(g, (int, float)):
        v = float(g)
        return _m(v, v, v)
    if len(g) == 2:
        gp, ga = float(g[0]), float(g[1])
        return _m(gp, ga, gp)
    return _m(float(g[0]), float(g[1]), float(g[2]))


def _sides():
    out = []
    a, f, h, g, th, sh, ft, to = (_SEG[k] for k in ('upper_arm', 'forearm', 'hand', 'fingers', 'thigh', 'shank',
                                                    'foot', 'toes'))
    for side, x in (('left', 1.0), ('right', -1.0)):
        s = int(x)
        out += [
            (side + '_upper_arm', 'torso', ((side + '_shoulder', 'x', -1),),
             _m(a['at'][0] * x, a['at'][1], a['at'][2]), a['turn'] * x, a['share'],
             (0.0, a['com_along'] * UPPER_ARM, 0.0), _gyr3(a['gyration'])),
            (side + '_forearm', side + '_upper_arm', ((side + '_elbow', 'x', -1),),
             (0.0, -UPPER_ARM, 0.0), CARRY_DEG * x, f['share'], (0.0, f['com_along'] * FOREARM, 0.0),
             _gyr3(f['gyration'])),
            (side + '_hand', side + '_forearm', ((side + '_wrist', 'x', -1),),
             (0.0, -FOREARM, 0.0), 0.0, h['share'], _vec3(h.get('com')), _gyr3(h['gyration'])),
            (side + '_fingers', side + '_hand', ((side + '_gripper', 'x', -1),),
             _vec3(g.get('at')), 0.0, g['share'], _vec3(g.get('com')), _gyr3(g['gyration'])),
            (side + '_thigh', 'pelvis', ((side + '_hip_yaw', 'y', s), (side + '_hip_roll', 'z', s),
                                        (side + '_hip', 'x', 1)),
             (HIP_HALF * x, -HIP_DROP, 0.0), 0.0, th['share'], (0.0, th['com_along'] * THIGH, 0.0),
             _gyr3(th['gyration'])),
            (side + '_shank', side + '_thigh', ((side + '_knee', 'x', 1),),
             (0.0, -THIGH, 0.0), 0.0, sh['share'], (0.0, sh['com_along'] * SHANK, 0.0), _gyr3(sh['gyration'])),
            (side + '_foot', side + '_shank', ((side + '_ankle', 'x', 1), (side + '_ankle_roll', 'z', s)),
             (0.0, -SHANK, 0.0), 0.0, ft['share'], _vec3(ft.get('com')), _gyr3(ft['gyration'])),
            (side + '_toes', side + '_foot', ((side + '_foot', 'x', 1),),
             (0.0, -(ANKLE_H - TOE_RY), BALL), 0.0, to['share'], _vec3(to.get('com')), _gyr3(to['gyration']))]
    return out


#: (segment, parent, joints ((joint, axis, sign), ..) applied in order, offset from the parent's
#: frame, rest turn about z (deg), mass share, centre of mass, radii of gyration x y z (m)): parents
#: first. The spine bends forward positive, its roll to the robot's right, the roll first - its drive on
#: the pelvis on a spur pair (`machine.body.drives`), the robot leans aside in its pelvis's frame,
#: then bends; a hip's yaw and roll turn the leg out positive, an ankle's roll the sole out.
_TORSO_JOINTS = tuple((str(j), str(a), int(s)) for j, a, s in _SEG['torso'].get(
    'joints', (('spine_roll', 'z', 1), ('spine', 'x', 1), ('waist', 'y', 1))))
SEGMENTS = tuple([
    ('pelvis', None, (), (0.0, 0.0, 0.0), 0.0, _SEG['pelvis']['share'], _vec3(_SEG['pelvis'].get('com')),
     _gyr3(_SEG['pelvis']['gyration'])),
    ('torso', 'pelvis', _TORSO_JOINTS,
     _vec3(_SEG['torso'].get('at')), 0.0, _SEG['torso']['share'], _vec3(_SEG['torso'].get('com')), _gyr3(_SEG['torso']['gyration'])),
    ('neck', 'torso', (('neck', 'x', 1),), _vec3(_SEG['neck'].get('at')), 0.0, _SEG['neck']['share'],
     _vec3(_SEG['neck'].get('com')), _gyr3(_SEG['neck']['gyration'])),
    ('head', 'neck', (('head', 'y', 1),), _vec3(_SEG['head'].get('at')), 0.0, _SEG['head']['share'],
     _vec3(_SEG['head'].get('com')), _gyr3(_SEG['head']['gyration']))] + _sides())

#: The sole's half-width, metres.
SOLE_HALF = SCALE * _F['sole_half']

#: The soles and the toes, their undersides: (segment, shape, sizes, centre), its frame, a
#: sneaker's 90 wide (`sole_box` the sole's half-height, `toes_box` the toes' half-width, half-height and rise).
#: The sole's heel's edge as two 12 mm spheres, to roll the strike, doubled it (1634 N against 1108) and the parry
#: never caught: 797 and 49.8 % against 604 and 73.7.
CONTACTS = (('foot', 'box', (SOLE_HALF, SCALE * _F['sole_box'], (BALL + HEEL) / 2.0),
             (0.0, -ANKLE_H + SCALE * _F['sole_box'], (BALL - HEEL) / 2.0)),
            ('toes', 'box', (SCALE * _F['toes_box'][0], SCALE * _F['toes_box'][1], TOE_M / 2.0),
             (0.0, SCALE * _F['toes_box'][2], TOE_M / 2.0)))

#: The robot's feet as point contacts, a sphere POINT_R under each sole's centre alone (`contacts`): the
#: stilts its standing counts on (`machine.stand`), 1 to build it so.
POINT_FEET, POINT_R = 0.0, SCALE * _F['point_r']

def contacts():
    """The undersides as built: CONTACTS, or a point under each sole's centre (POINT_FEET)."""
    if POINT_FEET:
        return (('foot', 'sphere', (POINT_R,), (0.0, -ANKLE_H + POINT_R, (BALL - HEEL) / 2.0)),)
    return CONTACTS


def sole_torsion_m():
    """Polar radius of gyration of the sole's bearing rim about the ankle yaw axis, m."""
    rim_r = (SOLE_HALF + 0.5 * (BALL + HEEL)) / math.sqrt(3.0)
    return round(math.hypot(rim_r, 0.5 * (BALL - HEEL)) / SCALE, 2) * SCALE

#: The seat's two buttocks, capsules along the robot's way SEAT_R round at SEAT_Y from SEAT_Z to SEAT_Z:
#: sat on one 0.10 m sphere, the heels in and leaning 15 degrees on, it rolled onto its back;
#: on these it sat. The fist, FIST_R at FIST_AT, the knuckles to lean on.
SEAT_R, SEAT_X, SEAT_Y, SEAT_Z = (SCALE * _F['seat'][0], SCALE * _F['seat'][1], SCALE * _F['seat'][2],
                                  _m(*_F['seat'][3:]))
FIST_R, FIST_AT = SCALE * _F['fist'][0], _m(*_F['fist'][1:])


def _capsules(row):
    """BODY's rows of a `[figure] body` row: ['seat'] the robot's buttocks, ['fist'] its fist,
    ['pair', segment, radius, [x, y, z]] a bilateral sphere pair, [segment, radius, [half_x, y(, z)]]
    an across-x capsule or midline sphere, [segment, radius, inset] a limb's along its length from
    either joint, else [segment, radius, from, to]."""
    if row[0] == 'seat':
        return [('pelvis', SEAT_R, (x, SEAT_Y, SEAT_Z[0]), (x, SEAT_Y, SEAT_Z[1])) for x in (SEAT_X, -SEAT_X)]
    if row[0] == 'fist':
        return [('hand', FIST_R, FIST_AT, FIST_AT)]
    if row[0] == 'pair':
        _tag, seg, r, pt = row
        px, py, pz = float(pt[0]), float(pt[1]), float(pt[2])
        return [(seg, SCALE * r, _m(sx, py, pz), _m(sx, py, pz)) for sx in (px, -px)]
    if len(row) == 3 and isinstance(row[2], list):
        seg, r, pt = row
        hx, py, pz = float(pt[0]), float(pt[1]), (float(pt[2]) if len(pt) > 2 else 0.0)
        return [(seg, SCALE * r, _m(-hx if hx else 0.0, py, pz), _m(hx, py, pz))]
    if len(row) == 3:
        return [(row[0], SCALE * row[1], _m(0.0, -row[2], 0.0), (0.0, SCALE * row[2] - _LONG[row[0]], 0.0))]
    return [(row[0], SCALE * row[1], _m(*row[2]), _m(*row[3]))]


#: The robot's body, on the floor and on itself: (segment, radius, from, to), its frame - a capsule, a
#: sphere where the ends meet - the drawn skin, the clothes not: the hips across, the waist, the
#: ribs, the chest and the shoulders across, the bust, the neck, the skull and the jaw, every
#: limb its length. A sphere or two a segment, lying the robot's arms went 40-43 mm into the floor, the
#: robot's head 40, its shins 100 through each other and its hand 41 into its upper arm; on
#: these, 2-9.
BODY = tuple(capsule for row in _F['body'] for capsule in _capsules(row))


def skin_of(seg):
    """[(radius, from, to)]: segment `seg`'s capsules (BODY, the left side's) in its frame, a right side's
    mirrored across: its thigh's stand 12 mm out of its axis, and the right's laid as the left's stood 24 mm
    inward of the mirror, its guard 17 mm under it sat on the bed's edge (`machine.body.guards`)."""
    s = -1.0 if seg.startswith('right_') else 1.0
    part = seg.split('_', 1)[1] if seg.startswith(('left_', 'right_')) else seg
    return [(r, (s * a[0], a[1], a[2]), (s * b[0], b[1], b[2])) for p, r, a, b in BODY if p == part]

#: The robot's widest half across, m: its trunk's capsules' reach along x (BODY's hips row), what an
#: aperture must clear each side of it - its arms hang within it.
WIDE_HALF = max(max(abs(fro[0]), abs(to[0])) + r for seg, r, fro, to in BODY
                if seg in ('pelvis', 'torso', 'neck', 'head'))


def _padded(row):
    """A pad of a `[figure] pads` row: its axis point and its toward from its segment's joint, or from its far
    joint ('end')."""
    seg, of, axis, radius, toward, size, wide = row
    base = (0.0, -_LONG[seg], 0.0) if of == 'end' else None
    at = (lambda v: _m(*v)) if base is None else (lambda v: tuple(b + m for b, m in zip(base, _m(*v))))
    return seg, at(axis), SCALE * radius, at(toward), SCALE * size, SCALE * wide


#: Pads where the robot's falls land, 5 mm of gel in TPU under its skin and clothes (`mjcf.PAD_*`):
#: (segment, the point of its capsule's axis under it, the capsule's radius, toward where the
#: landings cluster, the pad's radius and half-width across), the left side's. Over 26 bare falls,
#: each segment's first 0.2 s on the floor: the elbow's point 3068 N s, 7.2 kN at most; the hip's
#: front and side 2800, 8.4; the knee 1300, 7.4; the rest of the robot under 240 N s. The
#: knee's across its front: on a sphere there, and one at the thigh's end, knees under rolled the robot's
#: 22-84 degrees, 5 of 12 falls up.
PADS = tuple(_padded(row) for row in _F['pads'])
#: A pad stands PAD_M proud of the skin.
PAD_M = float(_F['pad_m'])

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
    """The robot's centre of mass, world, for every joint at {joint: degrees} and the pelvis placed."""
    placed = frames(degrees, pelvis, turn)
    total = [0.0, 0.0, 0.0]
    for name, (share, centre) in shares().items():
        at, here = placed[name]
        point = add(at, apply(here, centre))
        total = [s + share * c for s, c in zip(total, point)]
    return tuple(total)


def shares():
    """{segment: (share of the robot's mass, centre in its frame)}: as built (`machine.body.build`) or a
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
    fing_len = abs(next(c[3][1] for c in BODY if c[0] == 'fingers'))
    palm_span = abs(_m(*_SEG['fingers']['at'])[1]) + FIST_R
    defaults = {
        'elbow': round(math.degrees(math.atan2(FIST_R + FIST_AT[2] + PAD_M, FOREARM))),
        'wrist': round(math.degrees(math.atan2(FIST_AT[2], palm_span))),
        'gripper': round(math.degrees(math.atan2(FIST_R - FIST_AT[2], fing_len))),
    }
    raw_st = _F.get('stand', {})
    out.update({side + '_' + j: float(raw_st.get(j, defaults[j]))
                for j in ('elbow', 'wrist', 'gripper') for side in ('left', 'right')})
    for side in ('left', 'right'):
        out.update({side + '_hip': -STAND_KNEE / 2.0, side + '_knee': STAND_KNEE,
                    side + '_ankle': -STAND_KNEE / 2.0})
    return out


def pelvis_at(knee):
    """The robot's pelvis's height on flat soles with its knees bent `knee` rad, m: its legs' geometry."""
    return ANKLE_H + math.sqrt(THIGH ** 2 + SHANK ** 2 + 2.0 * THIGH * SHANK * math.cos(knee)) + HIP_DROP


def standing():
    """The robot's pelvis's height at a stand, m: both feet flat under its hips, the knees as `stand`
    bends them."""
    return pelvis_at(math.radians(STAND_KNEE))


def mass():
    """The robot's weight, kg: as built (`machine.body.build`) or MASS_KG."""
    from machine.body import build
    return build.mass() if build.SHELLS else MASS_KG


def fist(degrees, pelvis=(0.0, 0.0, 0.0), turn=IDENT, side='left'):
    """The robot's fist's centre, world, at {joint: degrees}, the pelvis placed: what its hand bears on."""
    at, here = frames(degrees, pelvis, turn)[side + '_hand']
    return add(at, apply(here, FIST_AT))


#: The robot's fist's centre aside from its middle, m, its arm hanging as it stands.
ASIDE = fist(stand())[0]


def palm(degrees, pelvis=(0.0, 0.0, 0.0), turn=IDENT, side='left'):
    """The robot's hand's lowest point, world, at {joint: degrees}, the pelvis placed: of its fist's and its
    fingers' segments (`BODY`), what of it meets what the robot reaches down to - curled, the robot's
    fingers."""
    placed, lows = frames(degrees, pelvis, turn), []
    for seg in ('hand', 'fingers'):
        at, here = placed[side + '_' + seg]
        for r, a, b in skin_of(side + '_' + seg):
            low = min((add(at, apply(here, a)), add(at, apply(here, b))), key=lambda p: p[1])
            lows.append((low[0], low[1] - r, low[2]))
    return min(lows, key=lambda p: p[1])


def squat(arm=(35.0, 70.0, 10.0, 30.0)):
    """Static-equilibrium deep squat pose, deg, with the robot's centre of mass (`com`: as built or a
    woman's) balanced over the sole's midpoint."""
    out = {name: 0.0 for name in JOINTS}
    pelvis_rad = math.atan2(BALL + HEEL, THIGH)
    out.update(spine=round(math.degrees(0.25 * math.pi)),
               neck=round(math.degrees(pelvis_rad) + STAND_KNEE))
    for side in ('left', 'right'):
        for k, v in zip(('shoulder', 'elbow', 'wrist', 'gripper'), arm):
            out[side + '_' + k] = float(v)
    turn = rx(pelvis_rad)
    yp = ANKLE_H + HIP_DROP + (THIGH + SHANK) * math.sin(pelvis_rad)
    xa, zp = HIP_HALF - 3.0 * PAD_M, -HEEL
    for _ in range(6):
        for side, sgn in (('left', 1.0), ('right', -1.0)):
            angles = leg(sgn, (0.0, yp, zp), turn, (sgn * xa, ANKLE_H, 0.0), IDENT)
            for k, a in zip(LEG, angles):
                out[side + k] = round(math.degrees(a), 3)
        zp += 0.5 * (BALL - HEEL) - com(out, (0.0, yp, zp), turn)[2]
    return out


@functools.cache
def _squatted():
    return {joint: float(deg) for joint, deg in _F['squat'].items()} if 'squat' in _F else squat()


def __getattr__(name):
    """SQUAT, the robot's deep squat, deg (`squat`, laid at its first asking: on the robot as built, the
    build the figure's own): what of it touches in it is kept apart as what touches standing
    (`machine.body.mjcf`)."""
    if name == 'SQUAT':
        return _squatted()
    raise AttributeError(name)
