"""Her drives: each a coaxial stack of an inverter disc, a pancake outrunner, a one-stage gearbox.

Two motors as bought, one box at one ratio, one board - the 63100: a stack for the legs and the
trunk, one for the arms and the head.

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
import math

from machine import linkage, mechatronics
from machine.figure import HIP_DROP, HIP_HALF
from motor.pmsm import TORQUE_FACTOR

#: The parts library and her build of it (machine/library, machine/robots): the tables below are
#: theirs, read once.
_LIB, _BUILD = mechatronics.library(), mechatronics.robot()
PACK_V = _BUILD['pack_v']


class Size:

    """One stack: its inverter, its outrunner, its gearbox, its envelope; `parts` along its axis
    from its input end, (part, radius m, length m)."""

    __slots__ = ('amps', 'kt_motor', 'r', 'kv', 'rotor', 'winding', 'efficiency', 'housing_k_w',
                 'diameter', 'length', 'mass', 'drag', 'shock', 'board', 'parts', 'source',
                 'rated')

    def __init__(self, amps, kt_motor, r, kv, rotor, winding, efficiency, housing_k_w,
                 diameter, length, mass, drag, shock, board, parts, source, rated):
        self.amps, self.kt_motor, self.r, self.kv, self.rotor = amps, kt_motor, r, kv, rotor
        self.rated = rated
        self.winding, self.efficiency, self.housing_k_w = winding, efficiency, housing_k_w
        self.diameter, self.length, self.mass, self.source = diameter, length, mass, source
        self.drag, self.shock, self.board, self.parts = drag, shock, board, parts


#: Her stacks' parts as the library has them: a motor's (can D m, height m, kg, line-to-line
#: ohm, peak N m, rotor kg m^2) and its KV; a gearbox's diameter and stiffness; an inverter's
#: (disc D m, amps, kg, its laminate's K/W to the air, its height m). Each kind's stack: (motor,
#: gearbox, inverter). One gearbox for every stack: its ratio, its input's share of the rotor's
#: inertia, its wind-up's damping ratio, its play and its mesh's friction.
FRAMES = {k: (m['can_m'], m['height_m'], m['kg'], m['ohm'], m['peak_nm'], m['rotor_kgm2'])
          for k, m in _LIB['motors'].items()}
KV = {k: m['kv'] for k, m in _LIB['motors'].items()}
BOXES = {k: b['diameter_m'] for k, b in _LIB['gearboxes'].items()}
BOX_K = {k: b['stiffness_nm_rad'] for k, b in _LIB['gearboxes'].items()}
INVERTERS = {k: (v['disc_m'], v['amps'], v['kg'], v['laminate_k_w'], v['height_m'])
             for k, v in _LIB['inverters'].items()}
STACKS = {kind: tuple(stack) for kind, stack in _BUILD['stacks'].items()}
_BOX = _LIB['gearboxes'][STACKS['knee'][1]]
RATIO, GEAR_J, BOX_ZETA = _BOX['ratio'], _BOX['input_j_share'], _BOX['zeta']
BACKLASH_DEG, BOX_FRICTION = _BOX['backlash_deg'], _BOX['friction_share']
COOLING, RIGID_PLAY_DEG = _BUILD['cooling'], _BUILD['rigid_play_deg']
AHEAD_WIND, AHEAD_PLAY = _BUILD['ahead_wind'], _BUILD['ahead_play']
#: Each kind's structure wound up a N m, mrad (`flex`).
WIND = dict(_BUILD['wind'])

#: The inverters out of their stacks: (segment, offset m in its frame, the axis its disc faces),
#: else in its stack. The knee's and the ankle's split, two discs facing
#: out on the femur's outer side: round the tibia under the knee the folded femur met them,
#: 10-16 mm, round the femur over it the folded tibia, 13-14; facing forward 15 cm over the knee
#: they stood 12 mm out of her (`tools/sim/fit.py`). The spine's and the neck's flush in her
#: back on its middle line, the one over the other: a hand's breadth off it a 100 mm disc
#: stands 17-20 mm out of her shell, at her shoulders' height 13-28 on the line itself; the
#: head's under the neck's - in her skull, 0.2 kg more of head, a fall put her head on the
#: floor at 1.49 m/s. The hip's in its stack - on the pelvis's or the torso's back it stood
#: 14-35 mm out of her, lower on the thigh 16 mm into the shank folded. In their
#: stacks, standing, the elbow's is 20 mm past her shell, the shoulder's 15, the hip's yaw's 9.
BOARDS = {'knee': ('thigh', (0.035, -0.18, 0.01), 'x'),
          'ankle': ('thigh', (0.051, -0.18, 0.01), 'x'),
          'spine': ('torso', (0.0, 0.14, -0.066), 'z'),
          'neck': ('torso', (0.0, 0.245, -0.066), 'z'),
          'head': ('torso', (0.0, 0.245, -0.049), 'z')}

def flex(joint):
    """rad a N m its gearbox and the structure on to its limb wind up."""
    k = kind(joint)
    return 1.0 / BOX_K[STACKS[k][1]] + WIND[k] * 1e-3


def play(joint):
    """Half its gearbox's backlash at the joint, rad, through its transmission's lever; none
    undriven."""
    return 0.0 if passive(joint) else math.radians(BACKLASH_DEG) / 2.0 / linkage.lever(joint)


def wind(joint):
    """N m/rad the joint winds up by past its play: its gearbox's and its structure's (`flex`)
    through its lever squared; none undriven."""
    return 0.0 if passive(joint) else linkage.lever(joint) ** 2 / flex(joint)


def mesh(joint):
    """The friction in its gearbox's mesh at the joint, N m: the hysteresis loop's half width;
    none undriven."""
    return 0.0 if passive(joint) else BOX_FRICTION * peak(joint)

#: Joints without a drive (`passive`): a kind's way - 1 on a spring, 2 held - and a spring's
#: (stiffness N m/rad, damping N m s/rad, rest deg); the toes' spring, its loss factor at its
#: push-off's rad/s and its rest.
WAYS = {k: v['way'] for k, v in _BUILD['passive'].items()}
PASSIVE = {k: (v['stiffness'], v['damping'], v['rest']) for k, v in _BUILD['passive'].items()
           if 'stiffness' in v}
TOE_K, TOE_LOSS, TOE_RAD_S, TOE_REST = (_BUILD['toes'][k] for k in ('stiffness', 'loss', 'rad_s', 'rest'))


def _stack(kind):
    """A kind's Size from its stack (`STACKS`): its frame's law, its winding (`KV`), its box,
    its inverter in it or apart (`BOARDS`)."""
    frame, box, inverter = STACKS[kind]
    kv = KV[frame]
    rotor, can, kg, ohm, top, inertia = FRAMES[frame]
    # N m an amp of q current, a sine's peak; a phase's ohm, half the pair's
    kt = 8.27 / kv
    disc, amps, b_kg, laminate, tall = INVERTERS[inverter]
    scale = (BOXES[box] / 0.08) ** 3
    parts = ((() if kind in BOARDS else (('board', disc / 2.0, tall),))
             + (('motor', rotor / 2.0 + 0.002, can),
                ('gear', BOXES[box] / 2.0 + 0.002, 0.28 * BOXES[box] + 0.004)))
    return Size(min(amps, top / kt), kt, 0.5 * ohm, kv, inertia,
                (310.0 * kg, 2.2 * 0.060 * 0.045 / (rotor * can)), 0.9, laminate,
                2.0 * max(r for _p, r, _l in parts), sum(l for _p, _r, l in parts),
                1.1 * (kg + 0.45 * scale), 0.08 * scale, 250.0 * scale, (b_kg, disc / 2.0),
                parts, 'motor %s %.0f x %.0f mm KV %.0f, box %.0f mm, %.0f A' % (
                    frame, rotor * 1e3, can * 1e3, kv, BOXES[box] * 1e3, amps), amps)


_SIZED = {}


def size(kind):
    """A kind's Size, laid once a stack and winding (`_stack`): STACKS and KV are knobs."""
    key = (kind,) + STACKS[kind] + (KV[STACKS[kind][0]],)
    if key not in _SIZED:
        _SIZED[key] = _stack(kind)
    return _SIZED[key]

#: Where each joint's stack sits: None on the joint's own axis; else (segment,
#: offset m in its frame[, its axis, '-x' its gearbox's end toward -x]) - its output turns what it
#: drives about that axis from that end, nothing radial off an axial drive's 
#: (the ankle's L lay along the shin, its crank about the knee's axis). The hip a
#: gimbal (`skeleton.gimbal`), each drive on the stage before its joint's - a segment's segment
#: named for that joint, its frame the hip's centre: the yaw's on the pelvis above, clear of the
#: pitch's swing (69 mm), turning the fork; the roll's on the fork behind, up and in on a spur
#: pair into the cradle, her seat's fullest - an 80 mm drum there stood 67 mm out of it; its
#: crank-rocker (docs/findings/drives.md) wanted the pitch's stack 20 mm in, where
#: its inverter stood 16 mm into the pelvis boom squatting -; the pitch's in the cradle on the
#: hip's centre; the knee's on its axis inside it - a belt's give showed in her
#: walk, no four-bar kept its 163 degrees over a 12 degree transmission,
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
#: her up from the floor asked 20 N m rms over 2 s, the neck holding her head 6. The
#: waist's on its axis 120 mm up the torso, its gearbox down to the spine's bracket - at the
#: torso's foot sits the pitch's,
#: and its bracket clears the roll's bearings to 85 deg; the shoulder's 15 mm in from its joint
#: (`skeleton.TRUNK`). The elbow's on its own axis, its belt and bevel pair gone
#: (`linkage.BELTS`).
JOINTS = {
    'spine': ('spine_roll', (0.0, 0.0, 0.0)), 'spine_roll': ('pelvis', (0.0, 0.02, 0.005)),
    'waist': ('torso', (0.0, 0.12, 0.0), '-y'),
    'neck': None, 'head': None,
    'shoulder': ('torso', (0.133, 0.325, -0.005)),
    'elbow': ('upper_arm', (0.0, -0.28, 0.0), '-x'),
    'wrist': ('forearm', (0.0, -0.12, 0.0), '-y'),
    'gripper': ('forearm', (0.0, -0.165, 0.0)),
    'hip_yaw': ('pelvis', (HIP_HALF, 0.103 - HIP_DROP, 0.0), '-y'),
    'hip_roll': ('hip_yaw', (-0.025, 0.035, -0.078)),
    'hip': ('hip_roll', (0.0, 0.0, 0.0)),
    'knee': None,
    'ankle': ('shank', (0.002, -0.090, 0.015), 'x'),
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
    return STACKS[k][0], size(k)


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


def tall(joint):
    """Its board's height with its parts, m."""
    return INVERTERS[STACKS[kind(joint)][2]][4]


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
    (`WAYS`), else None."""
    way = WAYS.get(kind(joint), 0.0)
    if not way:
        return None
    stiffness, damping, rest = ((TOE_K, TOE_LOSS * TOE_K / TOE_RAD_S, TOE_REST) if kind(joint) == 'foot'
                                else PASSIVE.get(kind(joint), (40.0, 1.0, 0.0)))
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


#: Clothing's insulation over bare air by kind (ISO 9920); each motor's housing's thermal mass,
#: J/K, and a board's K/W to its stator on its axis and off it.
CLOTH_AIR = dict(_BUILD['cloth'])
STATOR_J_K = {k: m['stator_j_k'] for k, m in _LIB['motors'].items()}
MOUNT_K_W, OFF_MOUNT_K_W = _BUILD['mount_k_w'], _BUILD['off_mount_k_w']


def cloth_air(joint=None):
    """Clothing thermal insulation multiplier (`>= 1.0`) for `joint` (or mean dressed factor)."""
    return 1.35 if joint is None else CLOTH_AIR.get(kind(joint), 1.20)


def stator(joint):
    """(stator_j_k J/K, mount_k_w K/W, cloth_air) of `joint`'s coaxial stator-housing assembly."""
    k = kind(joint)
    return STATOR_J_K[STACKS[k][0]], (OFF_MOUNT_K_W if k in BOARDS else MOUNT_K_W), cloth_air(joint)


def heat(joint):
    """(kt N m/A, winding ohm, the board's amps against the 100 A board's, the winding's J/K and
    K/W, the laminate's K/W to the air): what `machine.heat` keeps a joint's drive by."""
    s = of(joint)[1]
    return (kt(joint), r_ohm(joint), 100.0 / s.rated, s.winding[0], s.winding[1] * COOLING,
            s.housing_k_w * COOLING)
