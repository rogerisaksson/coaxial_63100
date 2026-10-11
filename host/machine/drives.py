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

from machine import linkage
from machine.gait import HIP_DROP, HIP_HALF
from motor.pmsm import TORQUE_FACTOR

#: The supply's lowest, V: 48-63, the boards' top 63.
PACK_V = 48.0


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

#: The motors as bought, a row a part: (the can's D m, its height m, kg, ohm line to line, N m
#: at its peak, its rotor's kg m^2) - T-Motor's U8 II Lite and MN6007 II, complete with their
#: bearings (the user, 2026-10-05: no frameless kit; motors cost, so two): 87.1 x 27 mm, 253 g
#: and 134-141 mohm at KV 100, USD 300; 67.2 x 26.1 mm, 159 g and 178 mohm at KV 160, USD 130
#: (the maker's pages, 2026-10-05; the U8's can by its model, docs/HARDWARE.md). Neither page
#: gives a peak: the U8's 4 N m is CubeMars' for the same 36N42P stator in the R80 at 50 A, the
#: MN6007's 1.9 that maker's RO60's 2.4 by its stack, 7 mm of 8.7; the rotors 0.45 of the kg at
#: the can's radius less 3 mm (so reckoned, the RO60 and the RO80 10 and 14 % over their pages'
#: 1.16e-4 and 2.61e-4). The frame
#: before them, 60 x 20 mm at KV 90 by a law fitted on T-Motor's pages, is no part, its rotor
#: 0.4 of the U8's (docs/findings/stacks.md). The winding 310 J/K a kg and to the air the
#: 5230SL's 2.2 K/W over 60 x 45 mm as 1/(D H).
FRAMES = {'U8': (0.0871, 0.0270, 0.253, 0.1375, 4.0, 1.87e-4),
          'MN6007': (0.0672, 0.0261, 0.159, 0.178, 1.9, 0.67e-4)}

#: The gearbox's diameter, m: a rolling-element box's momentary 250 N m at 80 mm, five times its
#: rated, 0.45 kg, its drag at its input - its rollers' start, the motor's cogging - 0.08 N m, all
#: as D^3, and 0.28 D long (estimated). A fall asked a knee's 763 N m of its 128, a 60 x 30's
#: rotor spun up through it by the blow (docs/findings/stacks.md).
BOXES = {'B': 0.064}
#: Each box's torsional stiffness at its output, N m/rad (estimated: a harmonic drive of 70 mm
#: gives 16-25 kN m/rad; a roller stage on a lobed ring, no flexspline, the same order), and its
#: wind-up's damping ratio. At a printed cycloid's 1.5 kN m/rad the stance ankle sagged 3-4 deg
#: under its 60 N m and the walk on setpoints fell at its first steps (2026-10-11).
BOX_K, BOX_ZETA = {'B': 9.0e3}, 0.3

#: The inverter by its disc mm: (disc D m, amps, kg, its laminate's K/W to the air through the
#: housing it is bolted to, its height m): the 63100 as built - 100 mm, 100 A, 0.2 kg, 3.6 K/W
#: (parts' centres 92 x 93 mm, the housing's skin at 10 W/m^2 K still and the pad 0.3), 14.7 mm
#: over all by its model (docs/HARDWARE.md) - on every drive (the user, 2026-10-05: boards and
#: gearboxes in variants are the pain, motors are bought): behind the U8's 87 mm, and 33 mm
#: wider than the MN6007. A motor's peak is under its amps: the clamp is the motor's.
INVERTERS = {100: (0.100, 100.0, 0.2, 3.6, 0.015)}

#: The drives' cooling: each winding's and laminate's K/W to the air times this - 1 in air. Her
#: joints air-cooled, a board's back on an aluminium holder over a third of it, on its motor or
#: gearbox, its only cooling (the user, 2026-10-10; 0.3 in oil, 2026-10-04).
COOLING = 1.0

#: Each kind's stack: (frame, box, inverter), its winding its frame's KV: 100 turns the knee
#: 960 deg/s at 48 V where it asks 860-1007, 85 would 816 - a joint's torque times its speed over
#: the pack's volts is its amps whatever the winding. The U8 where a joint asks over 50 N m -
#: the legs, the trunk -, the MN6007 for the arms and the head: 15 and 6 of her 21; her 30.8 kg
#: where 28.4, a hip's rotor 0.177 kg m^2 at its joint where 0.071. What they replaced and why
#: each stood: docs/findings/stacks.md.
KV = {'U8': 100.0, 'MN6007': 160.0}
STACKS = {**dict.fromkeys(('spine', 'spine_roll', 'waist', 'hip_yaw', 'hip_roll', 'hip', 'knee',
                           'ankle', 'ankle_roll'), ('U8', 'B', 100)),
          **dict.fromkeys(('neck', 'head', 'shoulder', 'elbow', 'wrist', 'gripper', 'foot'),
                          ('MN6007', 'B', 100))}

#: The inverters out of their stacks: (segment, offset m in its frame, the axis its disc faces),
#: else in its stack. The knee's and the ankle's split (the user, 2026-10-02), two discs facing
#: out on the femur's outer side: round the tibia under the knee the folded femur met them,
#: 10-16 mm, round the femur over it the folded tibia, 13-14; facing forward 15 cm over the knee
#: they stood 12 mm out of her (`tools/sim/fit.py`). The spine's and the neck's flush in her
#: back on its middle line, the one over the other: a hand's breadth off it a 100 mm disc
#: stands 17-20 mm out of her shell, at her shoulders' height 13-28 on the line itself; the
#: head's under the neck's - in her skull, 0.2 kg more of head, a fall put her head on the
#: floor at 1.49 m/s. The hip's in its stack - on the pelvis's or the torso's back it stood
#: 14-35 mm out of her, lower on the thigh 16 mm into the shank folded (2026-10-03). In their
#: stacks, standing, the elbow's is 20 mm past her shell, the shoulder's 15, the hip's yaw's 9
#: (2026-10-05).
BOARDS = {'knee': ('thigh', (0.035, -0.18, 0.01), 'x'),
          'ankle': ('thigh', (0.051, -0.18, 0.01), 'x'),
          'spine': ('torso', (0.0, 0.14, -0.066), 'z'),
          'neck': ('torso', (0.0, 0.245, -0.066), 'z'),
          'head': ('torso', (0.0, 0.245, -0.049), 'z')}

#: Each gearbox's play at its output, deg, and its mesh's friction as a share of its drive's
#: peak: a good printed box's (the user, 2026-10-11; estimated - PETG cycloid discs on steel pins
#: print at 0.3-1 deg, 1-3 % of their rated torque lost in the mesh; `physics.BOXED`). Rigid
#: (BOXED 0) the boards' encoders see the joint held within RIGID_PLAY_DEG, a wave drive's few
#: arcmin, as her walk's band was measured.
BACKLASH_DEG, BOX_FRICTION, RIGID_PLAY_DEG = 0.5, 0.02, 0.1
#: The wind-up and the play a board asks its rotor ahead of its setpoint by (`gearbox.Boxes.ahead`):
#: both, her toes dragged 16 mm at lift for 19, the walk 730 J/m for 607; the wind-up's alone
#: 8-14 mm at 0.1-0.3 deg of play (2026-10-11).
AHEAD_WIND, AHEAD_PLAY = 0.0, 0.0

#: Each joint's structure between its gearbox and its limb wound up a N m, mrad, its members in
#: series (`tools/sim/members.py`, 2026-10-03: the pitch's bracket at 12 mm gave the spine 3.3,
#: 27 deg at its clamp; sized, 0.085). With the box's own (BOX_K) a board sees its joint wound by
#: its last torque (`flex`, `physics.WOUND`).
WIND = {'spine': 0.085, 'spine_roll': 0.148, 'waist': 0.035, 'neck': 0.239, 'head': 0.239,
        'shoulder': 0.122, 'elbow': 0.128, 'wrist': 0.173, 'gripper': 0.173, 'hip_yaw': 0.116,
        'hip_roll': 0.085, 'hip': 0.061, 'knee': 0.051, 'ankle': 0.218, 'ankle_roll': 0.032,
        'foot': 0.155}


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

#: Joints without a drive, a kind's way (WAYS): driven, 0; on a spring, 1 - PASSIVE's stiffness
#: N m/rad and damping N m s/rad about its rest, deg, a kind not listed 40, 1 and 0 -; held at it,
#: 2, between stops (`mjcf.HELD_DEG`). Any kind, for the fewest drives (the user, 2026-10-03);
#: the toes and the fingers to begin with (the user, 2026-10-02): on the scoreboard, their four
#: drives' 0.88 kg gone, the fingers held open at 20 deg 212 -> 349, held 79.5 -> 68.0 %, every
#: rise down at 7.1 s; held a fist, 238 and 81.0 %; the toes sprung, 622 and 11.4 %, every walk
#: down within 0.8 s - the walker's push-off asks them and its legs' reach counts on them. Each
#: of the rest held in turn against 730 and 74.1 % (2026-10-03): the wrists 602 and 74.2, every
#: rise and walk standing - held; the head's turn 699 and 69.4 (a walk fell), the neck 628 and
#: 71.2 (a rise), the waist 727 and 71.6 (a rise), the spine's roll 1293 and 23.0, the hips' yaw
#: 1337 and 12.1.
WAYS = {'foot': 1.0, 'gripper': 2.0, 'wrist': 2.0}
PASSIVE = {'gripper': (40.0, 1.0, 80.0)}
#: The toes' spring about flat, N m/rad, and its damping, N m s/rad: a sneaker's forefoot, 0.2-0.5
#: N m a degree (a plated one's past 200 stood her on her toe tips, 2026-10-04) - a thin
#: carbon-fibre sandwich with a TPU core, or TPU printed round carbon rods glued in with silicone
#: (the user, 2026-10-04): springy but damped, the TPU's loss factor TOE_LOSS at the push-off's
#: TOE_RAD_S, c = loss k / omega (`passive`). The motors off them (the user, 2026-10-04).
#: TOE_REST deg its rest, toes up under 0 - a sneaker's toe spring (docs/findings/feet.md).
TOE_K, TOE_LOSS, TOE_RAD_S, TOE_REST = 10.0, 0.3, 20.0, 0.0


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
#: (`skeleton.TRUNK`, 2026-10-03). The elbow's on its own axis, its belt and bevel pair gone
#: (2026-10-04, `linkage.BELTS`).
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


def heat(joint):
    """(kt N m/A, winding ohm, the board's amps against the 100 A board's, the winding's J/K and
    K/W, the laminate's K/W to the air): what `machine.heat` keeps a joint's drive by."""
    s = of(joint)[1]
    return (kt(joint), r_ohm(joint), 100.0 / s.rated, s.winding[0], s.winding[1] * COOLING,
            s.housing_k_w * COOLING)
