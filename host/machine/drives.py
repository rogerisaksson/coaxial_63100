"""Her drives sized: three assemblies, each a board behind an outrunner on a cycloid, coaxial.

    size = drives.of('left_knee')      # (name, Size)
    drives.kt('left_knee')             # N m of joint torque an amp of q current
    drives.peak('left_knee')           # N m at the board's amps
    drives.armature('left_knee')       # kg m^2, the rotor seen through the cycloid
    drives.speed('left_knee')          # deg/s at the pack's volts, unloaded

A size: the board (its amps; its heat as the 63 V 100 A board's scaled to them, its laminate
bolted to the assembly's housing), the outrunner (Kt, the winding's resistance, KV, its rotor's
inertia, the winding's heat), the cycloid's ratio (RATIO) and efficiency, the assembly's
diameter and length and mass. Every joint has one (`JOINTS`), on its axis or, where one there
would look odd, mounted on a segment and driving it through a rod.
"""
from motor.catalog import PLATINUM_5230SL
from motor.pmsm import TORQUE_FACTOR, WINDING_J_PER_K, WINDING_K_PER_W

#: The pack: 12S at its nominal 3.7 V a cell, V.
PACK_V = 44.4


class Size:

    """One assembly: its board, its outrunner, its cycloid, its envelope."""

    __slots__ = ('amps', 'kt_motor', 'r', 'kv', 'rotor', 'winding', 'efficiency', 'housing_k_w',
                 'diameter', 'length', 'mass', 'source')

    def __init__(self, amps, kt_motor, r, kv, rotor, winding, efficiency, housing_k_w,
                 diameter, length, mass, source):
        self.amps, self.kt_motor, self.r, self.kv, self.rotor = amps, kt_motor, r, kv, rotor
        self.winding, self.efficiency, self.housing_k_w = winding, efficiency, housing_k_w
        self.diameter, self.length, self.mass, self.source = diameter, length, mass, source


#: Each size's cycloid, its ratio.
RATIO_L, RATIO_M, RATIO_S = 64.0, 76.0, 101.0


#: L: the 63 V 100 A board, its parts' centres 92 x 93 mm (the pick-and-place), a disc of 100 mm
#: behind the 5230SL; M and S: that board scaled to 50 and 20 A behind a 43 and a 35 mm stator,
#: estimated from their size classes (Kt 8.27/KV, R, rotor and mass by the class, the winding's
#: heat by its copper's mass).
#: The laminate's path to the air through the housing it is bolted to, K/W: the housing's skin
#: (0.03 m^2 for L) at 10 W/m^2 K still, the pad 0.3 - against the bare board's 11.7 in still air.
SIZES = {
    'L': Size(100.0, TORQUE_FACTOR * PLATINUM_5230SL.poles * PLATINUM_5230SL.lam,
              PLATINUM_5230SL.r, 190.0, PLATINUM_5230SL.j, (WINDING_J_PER_K, WINDING_K_PER_W),
              0.9, 3.6, 0.100, 0.095, 1.5, 'the 63100 board and its 5230SL'),
    'M': Size(50.0, 8.27 / 280.0, 0.08, 280.0, 3.5e-5, (70.0, 4.0), 0.9, 6.5, 0.070, 0.068,
              0.65, 'estimated: a 43 mm stator, KV 280'),
    'S': Size(20.0, 8.27 / 470.0, 0.25, 470.0, 8.0e-6, (25.0, 7.0), 0.85, 16.0, 0.042, 0.048,
              0.22, 'estimated: a 35 mm stator, KV 470'),
}

#: Each joint's size, and where its assembly sits: None on the joint's own axis; else (segment,
#: offset m in its frame) where it is mounted, a rod to the joint - the ankle's pair inside the
#: calf under the knee; the wrist's and the fingers' in the forearm; the toes' in the foot. On the
#: calf's back the ankle's stood 3 cm proud, a lump (2026-09-28).
JOINTS = {
    'spine': ('L', None), 'spine_roll': ('L', None), 'waist': ('M', None),
    'neck': ('S', None), 'head': ('S', None),
    'shoulder': ('M', None), 'elbow': ('S', None),
    'wrist': ('S', ('forearm', (0.0, -0.09, 0.0))),
    'gripper': ('S', ('forearm', (0.0, -0.165, 0.0))),
    'hip_yaw': ('M', None), 'hip_roll': ('L', None), 'hip': ('L', None), 'knee': ('L', None),
    'ankle': ('L', ('shank', (0.0, -0.10, -0.008))),
    'ankle_roll': ('M', ('shank', (0.0, -0.19, 0.0))),
    'foot': ('S', ('foot', (0.0, -0.035, 0.07))),
}


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
    """Where a joint's assembly sits: None on its axis, else (segment, offset)."""
    where = JOINTS[kind(joint)][1]
    if where is None:
        return None
    side = joint[:-len(kind(joint))]
    return side + where[0], where[1]


def ratio(joint):
    """The joint's cycloid's ratio."""
    return {'L': RATIO_L, 'M': RATIO_M, 'S': RATIO_S}[of(joint)[0]]


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
    """The rotor's inertia as the joint feels it through the cycloid, kg m^2."""
    return of(joint)[1].rotor * ratio(joint) ** 2


def speed(joint):
    """The joint's speed at PACK_V with no load, deg/s."""
    return of(joint)[1].kv * PACK_V * 6.0 / ratio(joint)


def heat(joint):
    """(kt N m/A, winding ohm, the board's amps against the 100 A board's, the winding's J/K and
    K/W, the laminate's K/W to the air): what `machine.heat` keeps a joint's drive by."""
    s = of(joint)[1]
    return (kt(joint), r_ohm(joint), 100.0 / s.amps) + tuple(s.winding) + (s.housing_k_w,)
