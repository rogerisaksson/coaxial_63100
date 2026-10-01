"""Her drives sized: three assemblies, each a board behind an outrunner on its gearbox, coaxial.

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

#: Each gearbox's play at its output, deg (estimated: a rolling-element wave drive's few arcmin,
#: worn a little).
BACKLASH_DEG = 0.1


#: L: the 63 V 100 A board, its parts' centres 92 x 93 mm (the pick-and-place), a disc of 100 mm
#: behind the 5230SL; M and S: that board scaled to 25 and 6.8 A behind a 43 and a 35 mm stator
#: wound for KV 140 and 160 - the burst torque and the copper a N m^2 of a KV 280 at 50 A and a
#: KV 470 at 20 A, not 13 400 and 22 600 rpm at 48 V unloaded but 6 720 and 7 680 - estimated
#: from their size classes (Kt 8.27/KV, R by the class times KV^2, rotor and mass by the class,
#: the winding's heat by its copper's mass). Each gearbox's drag at its input, N m - its rollers'
#: start and the motor's cogging - and the torque it takes momentarily at its output, N m, a
#: rolling-element reducer's five times its rated (estimated).
#: The laminate's path to the air through the housing it is bolted to, K/W: the housing's skin
#: (0.03 m^2 for L) at 10 W/m^2 K still, the pad 0.3 - against the bare board's 11.7 in still air.
SIZES = {
    'L': Size(100.0, TORQUE_FACTOR * PLATINUM_5230SL.poles * PLATINUM_5230SL.lam,
              PLATINUM_5230SL.r, 190.0, PLATINUM_5230SL.j, (WINDING_J_PER_K, WINDING_K_PER_W),
              0.9, 3.6, 0.100, 0.095, 1.5, 0.08, 250.0, 'the 63100 board and its 5230SL'),
    'M': Size(50.0 * 140.0 / 280.0, 8.27 / 140.0, 0.08 * (280.0 / 140.0) ** 2, 140.0, 3.5e-5,
              (70.0, 4.0), 0.9, 6.5, 0.070, 0.068, 0.65, 0.02, 100.0,
              'estimated: a 43 mm stator, KV 140'),
    'S': Size(20.0 * 160.0 / 470.0, 8.27 / 160.0, 0.25 * (470.0 / 160.0) ** 2, 160.0, 8.0e-6,
              (25.0, 7.0), 0.85, 16.0, 0.042, 0.048, 0.22, 0.0065, 30.0,
              'estimated: a 35 mm stator, KV 160'),
}

#: Each joint's size, and where its assembly sits: None on the joint's own axis; else (segment,
#: offset m in its frame) where it is mounted, a rod to the joint - the hip's in the seat behind
#: it and its roll's on the pelvis's side over it, the femur's ends theirs (`LINKS`), as the
#: gluteals'; the ankle's at the knee, the knee's in the thigh under the hip; the ankle's roll in
#: the calf, the wrist's and the fingers' in the forearm, the toes' in the foot. Off the thigh,
#: the hip's pair took 3 kg out of its swing. The elbow and the neck M: on S
#: an elbow pushing her up from the floor asked 20 N m rms over 2 s, the neck holding her head
#: 6, their copper past what an S's winding sheds (2026-10-01).
JOINTS = {
    'spine': ('L', None), 'spine_roll': ('L', None), 'waist': ('M', None),
    'neck': ('M', None), 'head': ('S', None),
    'shoulder': ('M', None), 'elbow': ('M', None),
    'wrist': ('S', ('forearm', (0.0, -0.09, 0.0))),
    'gripper': ('S', ('forearm', (0.0, -0.165, 0.0))),
    'hip_yaw': ('M', None), 'hip_roll': ('L', ('pelvis', (0.105, 0.05, 0.0))),
    'hip': ('L', ('pelvis', (0.075, -0.03, -0.08))),
    'knee': ('L', ('thigh', (0.0, -0.10, 0.02))), 'ankle': ('L', ('shank', (0.0, -0.03, -0.035))),
    'ankle_roll': ('M', ('shank', (0.0, -0.19, 0.0))),
    'foot': ('S', ('foot', (0.0, -0.04, 0.045))),
}


#: A joint's total ratio over its stroke, by kind, (deg, ratio) knots between which it runs
#: straight: a rod's linkage kinematic, nonlinear - high where the stroke asks torque, low where
#: it asks speed. As built the knee asked 117 N m, its clamp, at 0-40 deg and 90-100, 41-81 at
#: 40-90, and never more than 512 deg/s; the ankle its clamp at -20..-10 and 10..20 (2026-10-02):
#: the knee 1:36 standing and folded, 1:28 swinging - at 1:22 shoved past saving her head met
#: the floor at 2.93 m/s once in 16, at 28 1.0 at most, at 30 1.07, at 36 1.32. The hip's pair
#: flat at 1:36: eased to 1:30 folded past -40 deg, the rise from the squat set her walk to
#: fall at 5.6 s; flat, she walked 16 s (2026-10-02).
STROKES = {'knee': ((-10.0, 36.0), (30.0, 36.0), (45.0, 28.0), (80.0, 28.0), (95.0, 36.0),
                    (170.0, 36.0)),
           'hip': ((-150.0, 36.0), (40.0, 36.0)),
           'hip_roll': ((-40.0, 36.0), (40.0, 36.0)),
           'ankle': ((-50.0, 30.0), (-25.0, 36.0), (25.0, 36.0), (35.0, 30.0))}

#: A rod's lever ratio between a drive's output and its joint, by kind: the ankle's from a 30 mm
#: crank at the knee to the heel's tuberosity 50 mm behind the ankle, the Achilles' line; the
#: knee's from a 28 mm crank in the thigh to the tibial tuberosity 45 mm before the knee, the
#: patellar tendon's - a joint's ratio (RATIO) the gearbox's times its rod's, constant over the
#: stroke (estimated).
LINKS = {'ankle': 50.0 / 30.0, 'knee': 45.0 / 28.0, 'hip': 1.6, 'hip_roll': 1.6}


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


def mount(joint):
    """Where a joint's assembly sits: None on its axis, else (segment, offset) - its own side's,
    the right's x mirrored; the pelvis and the trunk have none."""
    where = JOINTS[kind(joint)][1]
    if where is None:
        return None
    side = joint[:-len(kind(joint))]
    segment, (x, y, z) = where
    unsided = segment in ('pelvis', 'torso', 'neck', 'head')
    return (segment if unsided else side + segment), (-x if side == 'right_' else x, y, z)


def ratio(joint, deg=None):
    """The joint's total ratio: its size's, or at `deg` along its stroke (`STROKES`)."""
    knots = STROKES.get(kind(joint))
    if deg is None or knots is None:
        return {'L': RATIO_L, 'M': RATIO_M, 'S': RATIO_S}[of(joint)[0]]
    if deg <= knots[0][0]:
        return knots[0][1]
    for (a, ra), (b, rb) in zip(knots, knots[1:]):
        if deg <= b:
            return ra + (rb - ra) * (deg - a) / (b - a)
    return knots[-1][1]


def emf(joint):
    """Its motor's back-EMF through its size's ratio, V a rad/s of the joint: p lambda N."""
    s = of(joint)[1]
    return s.kt_motor / TORQUE_FACTOR * ratio(joint)


def kt(joint):
    """Joint torque an amp of q current, N m/A."""
    s = of(joint)[1]
    return s.kt_motor * ratio(joint) * s.efficiency


def r_ohm(joint):
    """The winding's copper watts an amp squared (1.5 r), ohm."""
    return TORQUE_FACTOR * of(joint)[1].r


def peak(joint):
    """The joint torque at the board's amps, N m."""
    return kt(joint) * of(joint)[1].amps


def armature(joint):
    """The rotor's and the gearbox's inertia as the joint feels them, kg m^2."""
    return (1.0 + GEAR_J) * of(joint)[1].rotor * ratio(joint) ** 2


def backdrive(joint):
    """The torque that turns the joint by its output, unpowered: its gearbox's drag through its
    ratio, N m."""
    return of(joint)[1].drag * ratio(joint)


def shock(joint):
    """The torque its gearbox takes momentarily, at the joint through its rod (`LINKS`), N m."""
    return of(joint)[1].shock * LINKS.get(kind(joint), 1.0)


def speed(joint):
    """The joint's speed at PACK_V with no load, deg/s."""
    return of(joint)[1].kv * PACK_V * 6.0 / ratio(joint)


def heat(joint):
    """(kt N m/A, winding ohm, the board's amps against the 100 A board's, the winding's J/K and
    K/W, the laminate's K/W to the air): what `machine.heat` keeps a joint's drive by."""
    s = of(joint)[1]
    return (kt(joint), r_ohm(joint), 100.0 / s.amps) + tuple(s.winding) + (s.housing_k_w,)
