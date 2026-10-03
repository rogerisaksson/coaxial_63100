"""Her drives sized: three assemblies, each an outrunner on its two-stage gearbox, its board apart.

    size = drives.of('left_knee')      # (name, Size)
    drives.kt('left_knee')             # N m of joint torque an amp of q current
    drives.peak('left_knee')           # N m at the board's amps
    drives.armature('left_knee')       # kg m^2, the rotor and the gearbox seen through it
    drives.speed('left_knee')          # deg/s at the supply's lowest, unloaded
    drives.backdrive('left_knee')      # N m to turn it by its output, unpowered
    drives.shock('left_knee')          # N m its gearbox takes momentarily, a fall's blow

A size: the board (its amps; its heat as the 63 V 100 A board's scaled to them, its laminate
bolted to the assembly's housing), the outrunner (Kt, the winding's resistance, KV, its rotor's
inertia, the winding's heat), the gearbox (its ratio, RATIO, its efficiency, the drag at its
input and the torque it takes momentarily at its output), the assembly's diameter and length
and mass. Every joint has one (`JOINTS`), on its axis or, where one there would look odd,
mounted on a segment and driving it through a rod. The gearbox is a wave drive with rolling
elements - a wave generator pushing rollers in a cage against a toothed ring, one stage to 1:60,
rolling where a cycloid slides, many rollers sharing a blow - backdrivable at the ratios here.
"""
from machine import linkage
from machine.gait import HIP_DROP, HIP_HALF
from motor.catalog import PLATINUM_5230SL
from motor.pmsm import TORQUE_FACTOR, WINDING_J_PER_K, WINDING_K_PER_W

#: The supply's lowest, V: 48-63, the boards' top 63.
PACK_V = 48.0


class Size:

    """One assembly: its board, its outrunner, its gearbox, its envelope."""

    __slots__ = ('amps', 'kt_motor', 'r', 'kv', 'rotor', 'winding', 'efficiency', 'housing_k_w',
                 'diameter', 'length', 'mass', 'drag', 'shock', 'source')

    def __init__(self, amps, kt_motor, r, kv, rotor, winding, efficiency, housing_k_w,
                 diameter, length, mass, drag, shock, source):
        self.amps, self.kt_motor, self.r, self.kv, self.rotor = amps, kt_motor, r, kv, rotor
        self.winding, self.efficiency, self.housing_k_w = winding, efficiency, housing_k_w
        self.diameter, self.length, self.mass, self.source = diameter, length, mass, source
        self.drag, self.shock = drag, shock


#: Each size's gearbox, its ratio; its input side - the wave generator, the rollers - seen at the
#: motor as GEAR_J of the rotor's inertia (estimated). A size's copper watts go as 1/ratio^2, the
#: inertia it puts on its joint as ratio^2: with only the rotors in the model the walk fell at
#: L 1:64 with no derate, its joints 6.5 deg off what they were asked. As built
#: (`physics.REFLECTED` ..) at 30, 40, 40 she walked 30 s from the squat on built boards, a hip
#: derated to 0.63 from 9.3 s, the rest at most 103 C; the strike 996 N, 1231 at 64, 76, 101
#: unbuilt; felled by the hole and a P shove, up and walking again (2026-10-01).
RATIO_L, RATIO_M, RATIO_S = 30.0, 40.0, 40.0
GEAR_J = 0.05

#: A gearbox's last stage is a ball stage - one eccentric's balls in a cage against a lobed ring,
#: its ratio balls + 1 - at most BALLS by size, its balls printable: L's 11 of 12 mm on a 6005's
#: race, M's 11 of 8 on a 6805's, S's 9 of 7 on a 6900's (docs/findings/body.md).
#: Past that a printable planetary before it, STAGE_M longer, STAGE_KG of the drive's mass heavier,
#: STAGE_EFF of the torque through it (estimated). One stage to 1:60 put 5 mm balls in L, 1-2.5 in
#: M and under 1 in S (the user, 2026-10-02).
BALLS = {'L': 12.0, 'M': 12.0, 'S': 10.0}
STAGE_M, STAGE_KG, STAGE_EFF = 0.015, 0.08, 0.97

#: The boards out of their drives' stacks - the 63100's 100 mm disc made L's 100 mm round -, each
#: size's BOARD (kg, its disc's radius m, estimated): a joint's board where BOARDS says, (segment,
#: offset m in its frame, the axis its disc faces), else with its drive. The knee's and the
#: ankle's split (the user, 2026-10-02), two discs facing out on the femur's outer side: round the
#: tibia under the knee the folded femur met them, 10-16 mm, round the femur over it the folded
#: tibia, 13-14; facing forward 15 cm over the knee they stood 12 mm out of her (`tools/sim/fit.py`).
BOARD = {'L': (0.2, 0.05), 'M': (0.08, 0.035), 'S': (0.03, 0.021)}
BOARDS = {'knee': ('thigh', (0.035, -0.18, 0.01), 'x'), 'ankle': ('thigh', (0.049, -0.18, 0.01), 'x')}

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


#: L: the 63 V 100 A board, its parts' centres 92 x 93 mm (the pick-and-place), a disc of 100 mm,
#: driving the 5230SL, the drive 80 mm round; M and S: that board scaled to 25 and 6.8 A, a 43 and
#: a 35 mm stator
#: wound for KV 140 and 160 - the burst torque and the copper a N m^2 of a KV 280 at 50 A and a
#: KV 470 at 20 A, not 13 400 and 22 600 rpm at 48 V unloaded but 6 720 and 7 680 - estimated
#: from their size classes (Kt 8.27/KV, R by the class times KV^2, rotor and mass by the class,
#: the winding's heat by its copper's mass). Each gearbox's drag at its input, N m - its rollers'
#: start and the motor's cogging - and the torque it takes momentarily at its output, N m, a
#: rolling-element reducer's five times its rated (estimated). A size's mass without its board.
#: The laminate's path to the air through the housing it is bolted to, K/W: the housing's skin
#: (0.03 m^2 for L) at 10 W/m^2 K still, the pad 0.3 - against the bare board's 11.7 in still air.
#: L's 5230SL wound WIND_L its catalogue's turns: Kt that times, R its square, KV over it - the
#: same copper a N m, 137 -> 171 N m at the board's 100 A. From the squat on the catalogue's she
#: fell at 9.45 s, wound so she walked 30 s; the hip yaw at 1:60 beside it, 5.82 (2026-10-02).
#: An M's amps scale with its winding: wound more it gains nothing.
WIND_L = 1.25
SIZES = {
    'L': Size(100.0, TORQUE_FACTOR * PLATINUM_5230SL.poles * PLATINUM_5230SL.lam * WIND_L,
              PLATINUM_5230SL.r * WIND_L ** 2, 190.0 / WIND_L, PLATINUM_5230SL.j,
              (WINDING_J_PER_K, WINDING_K_PER_W),
              0.9, 3.6, 0.080, 0.08, 1.3, 0.08, 250.0, 'the 63100 board and its 5230SL'),
    'M': Size(50.0 * 140.0 / 280.0, 8.27 / 140.0, 0.08 * (280.0 / 140.0) ** 2, 140.0, 3.5e-5,
              (70.0, 4.0), 0.9, 6.5, 0.060, 0.055, 0.57, 0.02, 100.0,
              'estimated: a 43 mm stator, KV 140'),
    'S': Size(20.0 * 160.0 / 470.0, 8.27 / 160.0, 0.25 * (470.0 / 160.0) ** 2, 160.0, 8.0e-6,
              (25.0, 7.0), 0.85, 16.0, 0.042, 0.04, 0.19, 0.0065, 30.0,
              'estimated: a 35 mm stator, KV 160'),
}

#: Each joint's size, and where its assembly sits: None on the joint's own axis; else (segment,
#: offset m in its frame[, its axis, '-x' its gearbox's end toward -x]) - its output turns what it
#: drives about that axis from that end, nothing radial off an axial drive's (the user,
#: 2026-10-03: the ankle's L lay along the shin, its crank about the knee's axis). The hip a
#: gimbal on its axes: the pitch's L centred on the
#: hip, the roll's M behind it at 1:100, up and in on a spur pair into her seat's fullest - an L
#: there stood 67 mm out of it -, the yaw's M above, clear of the pitch's swing (69 mm), all three
#: riding the pelvis; the knee's on its axis, 80 mm round inside it - a belt's give showed in her
#: walk (the user, 2026-10-02), no four-bar kept its 163 degrees over a 12 degree transmission,
#: coupling rods stood as wide as her knee; 100 mm round there the capture law flagged 260
#: catches walking in 6 s, 80 none -; the elbow's under the arm's quick-release, a belt to the
#: joint; the ankle's pair under the knee, one over the other on the shin's axis, each turning
#: the foot through its rod (`linkage.PAIRS`) - an L at the foot 15 mm ahead put her head down at
#: 1.6 and 1.9 m/s in two falls of four, 10 ahead with the roll's 10 back a derated knee's walk
#: fell -; the spine roll's M on the pelvis's top between the hips' yaws - on its axis
#: it stood 80 mm out of her back -, the wrist's and the fingers' in the forearm, the toes' in the
#: foot. Off the thigh, the hip's three took 3 kg out of its swing. The elbow and the neck M: on
#: S an elbow pushing her up from the floor asked 20 N m rms over 2 s, the neck holding her head
#: 6, their copper past what an S's winding sheds (2026-10-01).
JOINTS = {
    'spine': ('L', None), 'spine_roll': ('M', ('pelvis', (0.0, 0.03, 0.012))), 'waist': ('M', None),
    'neck': ('M', None), 'head': ('S', None),
    'shoulder': ('M', None), 'elbow': ('M', ('upper_arm', (0.0, -0.115, 0.0), '-y')),
    'wrist': ('S', ('forearm', (0.0, -0.12, 0.0), '-y')),
    'gripper': ('S', ('forearm', (0.0, -0.165, 0.0))),
    'hip_yaw': ('M', ('pelvis', (HIP_HALF, 0.103 - HIP_DROP, 0.0), '-y')),
    'hip_roll': ('M', ('pelvis', (HIP_HALF - 0.025, 0.035 - HIP_DROP, -0.078))),
    'hip': ('L', ('pelvis', (HIP_HALF, -HIP_DROP, 0.0))),
    'knee': ('L', None),
    'ankle': ('M', ('shank', (0.002, -0.080, 0.015), 'x')),
    'ankle_roll': ('M', ('shank', (-0.004, -0.150, 0.0), '-x')),
    'foot': ('S', ('foot', (0.0, -0.04, 0.045))),
}


#: A joint's total ratio at rest, by kind, its size's (RATIO_L ..) elsewhere: its gearbox's times
#: its transmission's (`linkage`) - the ankle's L on one rod at 1:36; the spine roll's and the waist's M at 1:60, past their 16.2 and 21.5 N m rms an M sheds at 1:40, 15, the
#: hip roll's at 1:100 its 36.6 (2026-10-02). As built the knee asked 117 N m, its clamp, at 0-40
#: deg and 90-100, 41-81 at 40-90, never more than 512 deg/s; at 1:22 mid-stroke shoved past
#: saving her head met the floor at 2.93 m/s once in 16, at 28 1.0 at most (2026-10-02). The
#: hip's pair at 1:30 folded past -40 deg, the rise from the squat set her walk to fall at 5.6 s;
#: at 36, she walked 16 s.
TOTALS = {'knee': 36.0, 'hip': 36.0, 'hip_roll': 100.0, 'spine_roll': 60.0, 'waist': 60.0}

#: A parallel pair's (`linkage.PAIRS`) two drives' gearboxes, their rods' levers after them:
#: two M at 1:68, the pitch's 1:60 through the rods' 0.88. At 1:60 on a roll lever of 0.64, from
#: the squat walking each asked 111 N m at peak and 23 rms, past an M's 77 N m 3 % of the time and
#: its 672 deg/s 0.36 %; two L were 2.8 kg against the L and the M's 2.02 (2026-10-02).
PAIRED = 68.0


def kind(joint):
    """A joint's kind: its name after the side."""
    for k in ('hip_yaw', 'hip_roll', 'ankle_roll', 'spine_roll'):
        if joint.endswith(k):
            return k
    return joint.rsplit('_', 1)[-1]


def of(joint):
    """(size name, Size) of a joint's drive."""
    name = JOINTS[kind(joint)][0]
    return name, SIZES[name]


def output(joint):
    """(axis letter, end) where `JOINTS` lays its drum - its gearbox's end +1 or -1 along it -,
    else None: on its joint's own axis, the end toward +."""
    where = JOINTS[kind(joint)][1]
    if where is None or len(where) < 3:
        return None
    return where[2][-1], -1.0 if where[2].startswith('-') else 1.0


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


def mount(joint):
    """Where a joint's assembly sits: None on its axis, else (segment, offset) - its own side's,
    the right's x mirrored; the pelvis and the trunk have none."""
    where = JOINTS[kind(joint)][1]
    if where is None:
        return None
    side = joint[:-len(kind(joint))]
    segment, (x, y, z) = where[:2]
    unsided = segment in ('pelvis', 'torso', 'neck', 'head')
    return (segment if unsided else side + segment), (-x if side == 'right_' else x, y, z)


def ratio(joint, deg=None):
    """The joint's total ratio (`TOTALS`) at rest, or at `deg` along its rod's stroke."""
    k = kind(joint)
    total = (PAIRED * linkage.lever(joint) if motors(joint) == 2 else TOTALS.get(k)
             or {'L': RATIO_L, 'M': RATIO_M, 'S': RATIO_S}[of(joint)[0]])
    if deg is None or k not in linkage.RODS:
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
            * STAGE_EFF ** (stages(joint) - 1)
            * (linkage.BEVEL_EFF if kind(joint) in linkage.BEVELS else 1.0))


def stages(joint):
    """Its gearbox's stages: its ball stage to BALLS, a planetary before it past."""
    return 1 if ratio(joint) / linkage.lever(joint) <= BALLS[of(joint)[0]] else 2


def length(joint):
    """Its assembly's length, m: its size's and a second stage's."""
    return of(joint)[1].length + STAGE_M * (stages(joint) - 1)


def mass(joint):
    """Its assembly's mass, kg: its size's and a second stage's, its board apart (`board`)."""
    return of(joint)[1].mass * (1.0 + STAGE_KG * (stages(joint) - 1))


def board(joint):
    """(segment, offset, kg, radius, axis) of its board: BOARDS', its side's, else None - with
    its drive."""
    where = BOARDS.get(kind(joint))
    if where is None:
        return None
    side = joint[:-len(kind(joint))]
    x, y, z = where[1]
    return (side + where[0], (-x if side == 'right_' else x, y, z)) + BOARD[of(joint)[0]] + (
        where[2],)


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
