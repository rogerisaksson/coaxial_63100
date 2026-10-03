"""Her drives: each a coaxial stack of an inverter disc, a pancake outrunner, a one-stage gearbox.

Two frames, three boxes and two inverters at one ratio, each winding picked per joint.

    size = drives.of('left_knee')      # (frame, Size)
    drives.kt('left_knee')             # N m of joint torque an amp of q current
    drives.peak('left_knee')           # N m at the board's amps
    drives.armature('left_knee')       # kg m^2, the rotor and the gearbox seen through it
    drives.speed('left_knee')          # deg/s at the supply's lowest, unloaded
    drives.backdrive('left_knee')      # N m to turn it by its output, unpowered
    drives.shock('left_knee')          # N m its gearbox takes momentarily, a fall's blow

A size: the inverter (its amps; its heat as the 63 V 100 A board's scaled to them, its laminate
bolted to the stack's housing), the outrunner (Kt, the winding's resistance, KV, its rotor's
inertia, the winding's heat), the gearbox (RATIO, its efficiency, the drag at its input and the
torque it takes momentarily at its output), the stack's parts, diameter, length and mass. Every
joint has one (`STACKS`, `JOINTS`), on its axis or, where one there would look odd, mounted on a
segment and driving it through a rod. The gearbox is a wave drive with rolling elements - a
wave generator pushing rollers in a cage against a lobed ring, rolling where a cycloid slides,
many rollers sharing a blow - backdrivable at the ratios here.
"""
from machine import linkage
from machine.gait import HIP_DROP, HIP_HALF
from motor.pmsm import TORQUE_FACTOR

#: The supply's lowest, V: 48-63, the boards' top 63.
PACK_V = 48.0


class Size:

    """One stack: its inverter, its outrunner, its gearbox, its envelope; `parts` along its axis
    from its input end, (part, radius m, length m)."""

    __slots__ = ('amps', 'kt_motor', 'r', 'kv', 'rotor', 'winding', 'efficiency', 'housing_k_w',
                 'diameter', 'length', 'mass', 'drag', 'shock', 'board', 'parts', 'source')

    def __init__(self, amps, kt_motor, r, kv, rotor, winding, efficiency, housing_k_w,
                 diameter, length, mass, drag, shock, board, parts, source):
        self.amps, self.kt_motor, self.r, self.kv, self.rotor = amps, kt_motor, r, kv, rotor
        self.winding, self.efficiency, self.housing_k_w = winding, efficiency, housing_k_w
        self.diameter, self.length, self.mass, self.source = diameter, length, mass, source
        self.drag, self.shock, self.board, self.parts = drag, shock, board, parts


#: Every gearbox's ratio; its input side - the wave generator, the rollers - seen at the motor as
#: GEAR_J of the rotor's inertia (estimated). Copper watts go as 1/ratio^2, the inertia a drive
#: puts on its joint as ratio^2. At 1:30 on 72 x 28 mm and 60 x 12 frames, their ankles' and
#: wrists' and toes' stacks 0.4-0.5 kg over the drives as they stood, she held 61.3 % of the gait
#: Monte Carlo's trials, 62.3 at 1:36, 74.6 on those drives' masses, 70.5 on those drives; on
#: these at 1:36 73.7, but felled by P she stayed down both ways - their rotors seen 1.4-1.5 times
#: those drives', the old seen up at 22.5 s -, at 1:30 up at 22.9 and 23.7 s, at 1:33 one way
#: (docs/findings/drives.md, 2026-10-03). On the 5230SL the knee at 1:22 shoved past saving her
#: head met the floor at 2.93 m/s once in 16, at 28 1.0 at most; the hips at 1:30 folded past
#: -40 deg rising from the squat, her walk down at 5.6 s, at 36 she walked 16 s (2026-10-02).
RATIO = 30.0
GEAR_J = 0.05

#: The pancake frames, (rotor's D, stack) m. Their law fitted on the makers' pages (MN3508,
#: MN5008, MN6007 II, M8108, M8110, U12 II; R line to line halved): Km 1.6e-5 Ds^1.8 L^0.8 (the
#: stator Ds the rotor less 6 mm, mm), kg 2.52e-6 Ds^2 (L + 6) + 0.039, its peak 1.1e-4 Ds^2 L N m
#: (2.5x its 180 s), the can L + 18.5 mm tall, 0.45 of the kg turning at its rotor's radius less
#: 3 mm, the winding 310 J/K a kg and to the air the 5230SL's 2.2 K/W over 60 x 45 mm as 1/(D H).
FRAMES = {'A': (0.068, 0.030), 'B': (0.060, 0.016)}

#: The gearboxes' diameters, m: a rolling-element box's momentary 250 N m at 80 mm, five times its
#: rated, 0.45 kg, its drag at its input - its rollers' start, the motor's cogging - 0.08 N m, all
#: as D^3, and 0.28 D long (estimated).
BOXES = {'A': 0.084, 'B': 0.064, 'C': 0.044}

#: The inverters by disc mm: (disc D m, amps, kg, its laminate's K/W to the air through the
#: housing it is bolted to): the 63100's, parts' centres 92 x 93 mm, the housing's skin at
#: 10 W/m^2 K still and the pad 0.3; a 70 mm at 50 A (estimated).
INVERTERS = {100: (0.100, 100.0, 0.2, 3.6), 70: (0.070, 50.0, 0.08, 6.5)}

#: Each kind's stack: (frame, KV, box, inverter), each KV in its window of 1.5x on the walk's
#: torque at its speed, each box's momentary 1.5x its peak and its rated over its rms. At 1:30
#: frame B's windows met at KV 76-95 but the ankles' (x1.43-1.48), their copper 1.05x their rms
#: and the hip roll's 1.17x; A's spine on 100 A to KV 104, hip 52-85, knee none - x1.41 at KV 90,
#: its first step's 144 N m at 704 deg/s, 2.0 kW, on 100 A at 48 V. The hip's and the knee's at
#: KV 120, x1.09 over her clamp, 960 deg/s: at 90 and 70 she held 70.7 % of the gait Monte
#: Carlo, its knee run into its SOA felled her 2 of 3; at 120 74.2 %, 1 of 3; at 125 71.8 %. The
#: spine at KV 40 on 50 A, 360 deg/s, felled her in the knee's SOA (docs/findings/drives.md,
#: 2026-10-03).
STACKS = {'spine': ('A', 100.0, 'A', 100), 'spine_roll': ('B', 90.0, 'B', 70),
          'waist': ('B', 90.0, 'C', 70), 'neck': ('B', 90.0, 'C', 70),
          'head': ('B', 90.0, 'C', 70), 'shoulder': ('B', 90.0, 'C', 70),
          'elbow': ('B', 90.0, 'C', 70), 'wrist': ('B', 90.0, 'C', 70),
          'gripper': ('B', 90.0, 'C', 70), 'hip_yaw': ('B', 90.0, 'B', 70),
          'hip_roll': ('B', 90.0, 'B', 70), 'hip': ('A', 120.0, 'A', 100),
          'knee': ('A', 120.0, 'A', 100), 'ankle': ('B', 90.0, 'B', 70),
          'ankle_roll': ('B', 90.0, 'B', 70), 'foot': ('B', 90.0, 'C', 70)}

#: The inverters out of their stacks: (segment, offset m in its frame, the axis its disc faces),
#: else in its stack. The knee's and the ankle's split (the user, 2026-10-02), two discs facing
#: out on the femur's outer side: round the tibia under the knee the folded femur met them,
#: 10-16 mm, round the femur over it the folded tibia, 13-14; facing forward 15 cm over the knee
#: they stood 12 mm out of her (`tools/sim/fit.py`). The spine's 100 mm on the torso's back; the
#: hip's in its stack, 100 mm round - on the pelvis's or the torso's back it stood 14-35 mm out of
#: her, beside the knee's 6, lower on the thigh 16 mm into the shank folded (2026-10-03).
BOARDS = {'knee': ('thigh', (0.035, -0.18, 0.01), 'x'),
          'ankle': ('thigh', (0.049, -0.18, 0.01), 'x'),
          'spine': ('torso', (0.0, 0.22, -0.066), 'z')}

#: Each gearbox's play at its output, deg (estimated: a rolling-element wave drive's few arcmin,
#: worn a little).
BACKLASH_DEG = 0.1

#: Joints without a drive to begin with (the user, 2026-10-02): the toes (TOES) and the fingers
#: (FINGERS) driven, 0; on a spring, 1 - PASSIVE's stiffness N m/rad and damping N m s/rad about
#: its rest, deg; held at it, 2. On the scoreboard, their four drives' 0.88 kg gone: the fingers
#: held open at 20 deg 212 -> 349, held 79.5 -> 68.0 %, every rise down at 7.1 s; held a fist,
#: 238 and 81.0 %; the toes sprung, 622 and 11.4 %, every walk down within 0.8 s - the walker's
#: push-off asks them and its legs' reach counts on them (2026-10-02).
TOES, FINGERS = 0.0, 2.0
PASSIVE = {'foot': (40.0, 1.0, 0.0), 'gripper': (40.0, 1.0, 80.0)}


def _stack(kind):
    """A kind's Size from its stack (`STACKS`): its frame's law, its winding, its box, its
    inverter in it or apart (`BOARDS`)."""
    frame, kv, box, inverter = STACKS[kind]
    rotor, stack = FRAMES[frame]
    ds, mm = (rotor - 0.006) * 1e3, stack * 1e3
    km = 1.6e-5 * ds ** 1.8 * mm ** 0.8
    kg = 2.52e-6 * ds * ds * (mm + 6.0) + 0.039
    can = stack + 0.0185
    kt = 8.27 / kv
    disc, amps, b_kg, laminate = INVERTERS[inverter]
    scale = (BOXES[box] / 0.08) ** 3
    parts = ((() if kind in BOARDS else (('board', disc / 2.0, 0.010),))
             + (('motor', rotor / 2.0 + 0.002, can),
                ('gear', BOXES[box] / 2.0 + 0.002, 0.28 * BOXES[box] + 0.004)))
    return Size(min(amps, 1.1e-4 * ds * ds * mm / kt), kt, (kt / km) ** 2 / TORQUE_FACTOR, kv,
                0.45 * kg * (rotor / 2.0 - 0.003) ** 2,
                (310.0 * kg, 2.2 * 0.060 * 0.045 / (rotor * can)), 0.9, laminate,
                2.0 * max(r for _p, r, _l in parts), sum(l for _p, _r, l in parts),
                1.1 * (kg + 0.45 * scale), 0.08 * scale, 250.0 * scale, (b_kg, disc / 2.0),
                parts, 'frame %s %.0f x %.0f mm KV %.0f, box %.0f mm, %.0f A' % (
                    frame, rotor * 1e3, stack * 1e3, kv, BOXES[box] * 1e3, amps))


SIZES = {k: _stack(k) for k in STACKS}

#: Where each joint's stack sits: None on the joint's own axis; else (segment,
#: offset m in its frame[, its axis, '-x' its gearbox's end toward -x]) - its output turns what it
#: drives about that axis from that end, nothing radial off an axial drive's (the user,
#: 2026-10-03: the ankle's L lay along the shin, its crank about the knee's axis). The hip a
#: gimbal (`skeleton.gimbal`), each drive on the stage before its joint's - a segment's segment
#: named for that joint, its frame the hip's centre: the yaw's on the pelvis above, clear of the
#: pitch's swing (69 mm), turning the fork; the roll's on the fork behind, up and in on a spur
#: pair into the cradle, her seat's fullest - an 80 mm drum there stood 67 mm out of it; its
#: crank-rocker (docs/findings/drives.md, 2026-10-03) wanted the pitch's stack 20 mm in, where
#: its inverter stood 16 mm into the pelvis boom squatting -; the pitch's in the cradle on the
#: hip's centre; the knee's on its axis inside it - a belt's give showed in her
#: walk (the user, 2026-10-02), no four-bar kept its 163 degrees over a 12 degree transmission,
#: coupling rods stood as wide as her knee; 100 mm round there the capture law flagged 260
#: catches walking in 6 s, 80 none -; the elbow's under the arm's quick-release, a belt to the
#: joint; the ankle's pair under the knee, one over the other on the shin's axis, each turning
#: the foot through its rod (`linkage.PAIRS`) - an L at the foot 15 mm ahead put her head down at
#: 1.6 and 1.9 m/s in two falls of four, 10 ahead with the roll's 10 back a derated knee's walk
#: fell -; the trunk's roll first (`figure.SEGMENTS`), its drive on the pelvis's top between the
#: hips' yaws on a four-bar into it (`linkage.PLANAR`), the pitch's on its stage: on its axis the
#: roll's 60 mm drum stood 80 mm out of her back, 47 out of her shell; across the pitch a rod or a
#: differential failed (docs/findings/body.md); the wrist's and the fingers' in the forearm, the
#: toes' in the foot. Off the thigh, the hip's three took 3 kg out of its swing. An elbow pushing
#: her up from the floor asked 20 N m rms over 2 s, the neck holding her head 6 (2026-10-01). The
#: waist's on its axis 120 mm up the torso, its gearbox down to the spine's bracket - at the
#: torso's foot sits the pitch's,
#: and its bracket clears the roll's bearings to 85 deg; the shoulder's 15 mm in from its joint
#: (`skeleton.TRUNK`, 2026-10-03).
JOINTS = {
    'spine': ('spine_roll', (0.0, 0.0, 0.0)), 'spine_roll': ('pelvis', (0.0, 0.02, 0.005)),
    'waist': ('torso', (0.0, 0.12, 0.0), '-y'),
    'neck': None, 'head': None,
    'shoulder': ('torso', (0.133, 0.325, -0.005)),
    'elbow': ('upper_arm', (0.0, -0.115, 0.0), '-y'),
    'wrist': ('forearm', (0.0, -0.12, 0.0), '-y'),
    'gripper': ('forearm', (0.0, -0.165, 0.0)),
    'hip_yaw': ('pelvis', (HIP_HALF, 0.103 - HIP_DROP, 0.0), '-y'),
    'hip_roll': ('hip_yaw', (-0.025, 0.035, -0.078)),
    'hip': ('hip_roll', (0.0, 0.0, 0.0)),
    'knee': None,
    'ankle': ('shank', (0.002, -0.080, 0.015), 'x'),
    'ankle_roll': ('shank', (-0.004, -0.150, 0.0), '-x'),
    'foot': ('foot', (0.0, -0.04, 0.045)),
}


def kind(joint):
    """A joint's kind: its name after the side."""
    for k in ('hip_yaw', 'hip_roll', 'ankle_roll', 'spine_roll'):
        if joint.endswith(k):
            return k
    return joint.rsplit('_', 1)[-1]


def of(joint):
    """(frame, Size) of a joint's drive."""
    k = kind(joint)
    return STACKS[k][0], SIZES[k]


def output(joint):
    """(axis letter, end) where `JOINTS` lays its drum - its gearbox's end +1 or -1 along it -,
    else None: on its joint's own axis, the end toward +."""
    where = JOINTS[kind(joint)]
    if where is None or len(where) < 3:
        return None
    return where[2][-1], -1.0 if where[2].startswith('-') else 1.0


def toward(joint, letter):
    """+1 or -1 along `letter`, its drum's axis, toward its gearbox's end: `output`'s, the right
    side's x mirrored."""
    end = (output(joint) or (letter, 1.0))[1]
    return -end if letter == 'x' and joint.startswith('right_') else end


def outlet(joint):
    """(segment, point) its output turns about, as `mount`: its drum's centre, or past its
    gearbox's end, a pitch radius over its bevel pair (`linkage.BEVELS`)."""
    where, laid = mount(joint), output(joint)
    if where is None or laid is None or kind(joint) not in linkage.BEVELS:
        return where
    (seg, at), (letter, end) = where, laid
    i = 'xyz'.index(letter)
    reach = length(joint) / 2.0 + linkage.BEVEL_T / 2.0 + linkage.BEVELS[kind(joint)]
    return seg, tuple(v + end * reach if k == i else v for k, v in enumerate(at))


def pivot(joint):
    """Its joint's centre in the frame its drive rides (`mount`): its segment's place on its
    parent, or a stage's origin."""
    from machine.figure import SEGMENTS, STAGES
    where = mount(joint)
    if where is not None and where[0] in STAGES:
        return (0.0, 0.0, 0.0)
    return next(s[3] for s in SEGMENTS if joint in [j for j, *_ in s[2]])


def mount(joint):
    """Where a joint's assembly sits: None on its axis, else (segment, offset) - its own side's,
    the right's x mirrored; the pelvis and the trunk have none."""
    where = JOINTS[kind(joint)]
    if where is None:
        return None
    side = joint[:-len(kind(joint))]
    segment, (x, y, z) = where[:2]
    unsided = segment in ('pelvis', 'torso', 'neck', 'head')
    return (segment if unsided else side + segment), (-x if side == 'right_' else x, y, z)


def ratio(joint, deg=None):
    """The joint's total ratio at rest - its gearbox's (`RATIO`) through its transmission's
    (`linkage`) -, or at `deg` along its rod's or four-bar's stroke."""
    k = kind(joint)
    total = RATIO * linkage.lever(joint)
    if deg is None or (k not in linkage.RODS and k not in linkage.PLANAR):
        return total
    return total * linkage.lever(joint, deg) / linkage.lever(joint)


def motors(joint):
    """The drives turning it: a parallel pair's joint 2 (`linkage.PAIRS`), each through its own
    rod - its torque, the inertia and the drag it carries theirs together -, else 1."""
    k = kind(joint)
    return 2 if k in linkage.PAIRS or k in linkage.PAIRS.values() else 1


def emf(joint):
    """Its motor's back-EMF through its size's ratio, V a rad/s of the joint: p lambda N."""
    s = of(joint)[1]
    return s.kt_motor / TORQUE_FACTOR * ratio(joint)


def kt(joint):
    """Joint torque an amp of q current in each of its drives, N m/A."""
    s = of(joint)[1]
    return (motors(joint) * s.kt_motor * ratio(joint) * s.efficiency
            * (linkage.BEVEL_EFF if kind(joint) in linkage.BEVELS else 1.0))


def length(joint):
    """Its stack's length, m."""
    return of(joint)[1].length


def mass(joint):
    """Its stack's mass, kg, its inverter apart (`board`)."""
    return of(joint)[1].mass


def along(joint):
    """[(part, radius m, centre m, length m)] of its stack, its centre along its axis from the
    stack's middle toward its gearbox's end."""
    s, at, out = of(joint)[1], -of(joint)[1].length / 2.0, []
    for part, radius, long in s.parts:
        out.append((part, radius, at + long / 2.0, long))
        at += long
    return out


def board(joint):
    """(segment, offset, kg, radius, axis) of its board: BOARDS', its side's, else None - with
    its drive."""
    where = BOARDS.get(kind(joint))
    if where is None:
        return None
    side = joint[:-len(kind(joint))]
    x, y, z = where[1]
    seg = where[0] if where[0] in ('pelvis', 'torso', 'neck', 'head') else side + where[0]
    return (seg, (-x if side == 'right_' else x, y, z)) + of(joint)[1].board + (where[2],)


def boards():
    """{joint: its board apart (`board`)}, both sides'."""
    from machine.figure import JOINTS
    return {j: b for j in JOINTS if not passive(j) and (b := board(j)) is not None}


def r_ohm(joint):
    """The winding's copper watts an amp squared (1.5 r), ohm."""
    return TORQUE_FACTOR * of(joint)[1].r


def peak(joint):
    """The joint torque at the board's amps, N m."""
    return kt(joint) * of(joint)[1].amps


def passive(joint):
    """(stiffness N m/rad - None held -, damping N m s/rad, rest deg) of a joint with no drive
    (`PASSIVE`), else None."""
    way = {'foot': TOES, 'gripper': FINGERS}.get(kind(joint), 0.0)
    if not way:
        return None
    stiffness, damping, rest = PASSIVE[kind(joint)]
    return (stiffness if way == 1.0 else None), damping, rest


def armature(joint):
    """The rotor's and the gearbox's inertia as the joint feels them, kg m^2; none undriven."""
    return 0.0 if passive(joint) else (motors(joint) * (1.0 + GEAR_J) * of(joint)[1].rotor
                                       * ratio(joint) ** 2)


def backdrive(joint):
    """The torque that turns the joint by its output, unpowered: its gearbox's drag through its
    ratio, N m; none undriven."""
    return 0.0 if passive(joint) else motors(joint) * of(joint)[1].drag * ratio(joint)


def shock(joint):
    """The torque its gearbox takes momentarily, at the joint through its transmission, N m."""
    return motors(joint) * of(joint)[1].shock * linkage.lever(joint)


def speed(joint):
    """The joint's speed at PACK_V with no load, deg/s."""
    return of(joint)[1].kv * PACK_V * 6.0 / ratio(joint)


def heat(joint):
    """(kt N m/A, winding ohm, the board's amps against the 100 A board's, the winding's J/K and
    K/W, the laminate's K/W to the air): what `machine.heat` keeps a joint's drive by."""
    s = of(joint)[1]
    return (kt(joint), r_ohm(joint), 100.0 / s.amps) + tuple(s.winding) + (s.housing_k_w,)
