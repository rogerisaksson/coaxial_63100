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
    drives.slew('left_knee')           # N m/s its torque slews at most, its board's current loop
    drives.jerk('left_knee')           # rad/s^3 its joint's at most, that over its rotor's inertia

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
from typing import Any

from machine.body import linkage, mechatronics
from machine.body.figure import ANKLE_H, BALL, SEGMENTS, SHANK, THIGH
from motor.pmsm import TORQUE_FACTOR

#: The parts library and her build of it (machine/library, machine/humanoids): the tables below are
#: theirs, read once.
_LIB, _BUILD = mechatronics.library(), mechatronics.robot()
_MAT, _WORLD = mechatronics.materials(), mechatronics.environment()
_PACK = _BUILD['pack']
PACK_V = float(_BUILD.get('pack_v', _PACK['series'] * _PACK['cell_v_min']))


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
_ROTOR_RING, _KV_REF = float(_MAT['rotor_ring_share']), float(_MAT['kv_ref'])
_RHO_STEEL = float(_MAT['steel_kg_m3'])
_CP_WIND, _CP_HOUS = float(_MAT['copper_winding_j_k_kg']), float(_MAT['aluminium_housing_j_k_kg'])
_H_SLOT, _H_DISC = float(_MAT['slot_w_m2_k']), float(_MAT['disc_w_m2_k'])
_SHELL = _BUILD['shell']
_LIP_M, _FASTENER_SHARE = float(_SHELL['lip_m']), float(_SHELL['fastener_share'])
_TIM_M, _K_TIM, _K_CF_THRU = (float(_SHELL['tim_m']), float(_SHELL['tim_w_m_k']),
                              float(_SHELL['cf_thru_w_m_k']))


def _rotor_j(m):
    """Outrunner rotor inertia, kg m^2, from can radius, mass and 1/6 pole-ring scaling."""
    j = _ROTOR_RING * min(1.0, (_KV_REF / m['kv']) ** (1.0 / 6.0)) * m['kg'] * (0.5 * m['can_m']) ** 2
    return round(j, -int(math.floor(math.log10(j))) + 2)


FRAMES = {k: (m['can_m'], m['height_m'], m['kg'], m['ohm'], m['peak_nm'],
              float(m.get('rotor_kgm2', _rotor_j(m))))
          for k, m in _LIB['motors'].items()}
KV = {k: m['kv'] for k, m in _LIB['motors'].items()}
BOXES = {k: b['diameter_m'] for k, b in _LIB['gearboxes'].items()}
BOX_K = {k: float(b.get('stiffness_nm_rad',
                        round(float(b['stiffness_nm_rad_m2']) * b['diameter_m'] ** 2, -2)))
         for k, b in _LIB['gearboxes'].items()}
INVERTERS = {k: (v['disc_m'], v['amps'], v['kg'],
                 float(v.get('laminate_k_w', round(1.0 / (_H_DISC * math.pi * (0.5 * v['disc_m']) ** 2), 1))),
                 v['height_m'])
             for k, v in _LIB['inverters'].items()}
LOOP_HZ = {k: float(v.get('loop_hz', float(v['current_bw_share']) * float(v['pwm_hz'])))
           for k, v in _LIB['inverters'].items()}
STACKS = {kind: tuple(stack) for kind, stack in _BUILD['stacks'].items()}
_BOX = _LIB['gearboxes'][STACKS['knee'][1]]
RATIO, GEAR_J, BOX_ZETA = _BOX['ratio'], _BOX['input_j_share'], _BOX['zeta']
BACKLASH_DEG, BOX_FRICTION = _BOX['backlash_deg'], _BOX['friction_share']
COOLING = float(_BUILD.get('cooling', 1.0))
RIGID_PLAY_DEG = float(_BUILD.get('rigid_play_deg', 0.2 * BACKLASH_DEG))
AHEAD_WIND = float(_BUILD.get('ahead_wind', 0.0))
AHEAD_PLAY = float(_BUILD.get('ahead_play', 0.0))

def _boards():
    """Remote split inverter placements from `[boards]`, stacking twin discs by `height_m + gap_m`."""
    raw, out = _BUILD.get('boards', {}), {}
    for k, b in raw.items():
        if 'stack_on' not in b:
            out[k] = (str(b['on']), tuple(float(v) for v in b['at']), str(b['face']))
    for k, b in raw.items():
        if 'stack_on' in b:
            seg, base, face = out[b['stack_on']]
            ax = 'xyz'.index(face[-1])
            step = (0.5 * INVERTERS[STACKS[b['stack_on']][2]][4] + float(b['gap_m'])
                    + 0.5 * INVERTERS[STACKS[k][2]][4])
            sign = -1.0 if face.startswith('-') else 1.0
            out[k] = (seg, tuple(round(v + sign * step, 6) if i == ax else v for i, v in enumerate(base)), face)
    return out


#: Split inverter discs out of their stacks: (segment, offset m in its frame, normal axis).
BOARDS = _boards()

def flex(joint):
    """rad a N m its gearbox and the structure on to its limb wind up."""
    k = kind(joint)
    return 1.0 / BOX_K[STACKS[k][1]] + _structural()[0][k] * 1e-3


def play(joint):
    """Half its gearbox's backlash at the joint, rad, through its transmission's lever; none
    undriven."""
    return 0.0 if passive(joint) else math.radians(BACKLASH_DEG) / 2.0 / linkage.lever(joint)


def wind(joint):
    """N m/rad the joint winds up by past its play: its gearbox's and its structure's (`flex`)
    through its lever squared; none undriven."""
    return 0.0 if passive(joint) else linkage.lever(joint) ** 2 / flex(joint)


def series(joint):
    """N m/rad of the spring in series between the joint's drive and the joint (SERIES), at the
    joint; 0 none, and none undriven."""
    return 0.0 if passive(joint) else float(_structural()[1].get(kind(joint), 0.0))


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
TOE_K, TOE_LOSS, TOE_REST = (float(_BUILD['toes'][k]) for k in ('stiffness', 'loss', 'rest'))
TOE_RAD_S = float(_BUILD['toes'].get(
    'rad_s', round(math.sqrt(float(_WORLD['g_m_s2']) * (THIGH + SHANK)) / math.hypot(BALL, ANKLE_H))))


def _stack(kind):
    """A kind's Size from its stack (`STACKS`): its frame's law, its winding (`KV`), its box,
    its inverter in it or apart (`BOARDS`)."""
    frame, box, inverter = STACKS[kind]
    kv = KV[frame]
    rotor, can, kg, ohm, top, inertia = FRAMES[frame]
    # N m an amp of q current, a sine's peak; a phase's ohm, half the pair's
    kt = round(TORQUE_FACTOR * 30.0 / (math.pi * math.sqrt(3.0)), 2) / kv
    disc, amps, b_kg, laminate, tall = INVERTERS[inverter]
    b_spec = _LIB['gearboxes'][box]
    box_d = BOXES[box]
    len_share = float(b_spec['length_share'])
    box_kg = float(b_spec['fill_share']) * _RHO_STEEL * 0.25 * math.pi * len_share * box_d ** 3
    parts = ((() if kind in BOARDS else (('board', disc / 2.0, tall),))
             + (('motor', rotor / 2.0 + _LIP_M, can),
                ('gear', box_d / 2.0 + _LIP_M, len_share * box_d + 2.0 * _LIP_M)))
    return Size(min(amps, top / kt), kt, 0.5 * ohm, kv, inertia,
                (_CP_WIND * kg, 1.0 / (_H_SLOT * math.pi * rotor * can)), float(b_spec['efficiency']),
                laminate, 2.0 * max(r for _p, r, _l in parts), sum(l for _p, _r, l in parts),
                _FASTENER_SHARE * (kg + box_kg), float(b_spec['drag_nm_m3']) * box_d ** 3,
                round(float(b_spec['shock_pa']) * box_d ** 3, 1), (b_kg, disc / 2.0),
                parts, 'motor %s %.0f x %.0f mm KV %.0f, box %.0f mm, %.0f A' % (
                    frame, rotor * 1e3, can * 1e3, kv, box_d * 1e3, amps), amps)


_SIZED = {}


def size(kind):
    """A kind's Size, laid once a stack and winding (`_stack`): STACKS and KV are knobs."""
    key = (kind,) + STACKS[kind] + (KV[STACKS[kind][0]],)
    if key not in _SIZED:
        _SIZED[key] = _stack(kind)
    return _SIZED[key]

def _mounts():
    """Drive assembly mounts derived from `figure.SEGMENTS`, `linkage` and `[mounts]`."""
    raw, out = _BUILD.get('mounts', {}), {}
    j_info = {}
    for seg in SEGMENTS:
        if seg[0].startswith('right_'):
            continue
        for idx, (j, ax, _s) in enumerate(seg[2]):
            prev = seg[2][idx - 1][0].replace('left_', '') if idx > 0 else None
            j_info[j.replace('left_', '')] = (seg[0].replace('left_', ''),
                                              (seg[1] or '').replace('left_', ''), seg[3], ax, prev)
    for k in STACKS:
        seg_name, parent_name, offset, ax, prev = j_info[k]
        if k not in raw:
            out[k] = (prev, (0.0, 0.0, 0.0)) if prev else None
            continue
        m = raw[k]
        on, face = str(m['on']), (str(m['face']),) if 'face' in m else ()
        if 'at' in m:
            at = tuple(float(v) for v in m['at'])
        elif 'z_m' in m:
            cx, cy = linkage.PLANAR[k][0]
            at = (cx, cy, float(m['z_m']))
        elif 'x_m' in m:
            up, ahead = linkage.RODS[k][0][:2]
            at = (float(m['x_m']), round(up - SHANK + float(m.get('dy_m', 0.0)), 6), ahead)
        else:
            piv = offset if on == parent_name else (0.0, 0.0, 0.0)
            idx = 'xyz'.index(ax)
            at = tuple(round(v + (float(m['along_m']) if i == idx else 0.0), 6)
                       for i, v in enumerate(piv))
        out[k] = (on, at) + face
    return out


#: Where each joint's stack sits: None on its own axis, else (segment, offset m[, face]).
JOINTS = _mounts()

#: Each stopped kind's travel, (low, high) deg: mechanical stops (`[stops]`) and linkage dead centres.
TRAVEL = {**{k: (float(s[0]), float(s[1])) for k, s in _BUILD.get('stops', {}).items()},
          **{k: p[7] for k, p in linkage.PLANAR.items()},
          **{k: r[1] for k, r in linkage.RODS.items() if k in linkage.PAIRS},
          **{v: (-linkage.ROLLED[k], linkage.ROLLED[k]) for k, v in linkage.PAIRS.items()}}


def kind(joint):
    """A joint's kind: its name after the side."""
    for k in ('hip_yaw', 'hip_roll', 'ankle_roll', 'spine_roll'):
        if joint.endswith(k):
            return k
    return joint.rsplit('_', 1)[-1]


def travel(joint):
    """Its travel, (low, high) deg (`TRAVEL`); None where it turns on."""
    return TRAVEL.get(kind(joint))


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
    from machine.body.figure import SEGMENTS, STAGES
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
    from machine.body.figure import JOINTS
    return {j: b for j in JOINTS if not passive(j) and (b := board(j)) is not None}


def r_ohm(joint):
    """The winding's copper watts an amp squared (1.5 r), ohm."""
    return TORQUE_FACTOR * of(joint)[1].r


def peak(joint):
    """The joint torque at the board's amps, N m, no more than its gearbox takes momentarily (`shock`): its
    stack's weakest part."""
    return min(kt(joint) * of(joint)[1].amps, shock(joint))


def wep_scale(joint):
    """Stator tooth-saturation torque gain sqrt(I_board / I_linear) from linear knee to board ceiling."""
    if passive(joint):
        return 1.0
    s = of(joint)[1]
    return math.sqrt(s.rated / s.amps)


def slew(joint):
    """Its torque's fastest slew, N m/s: its peak through its board's current loop, first order at
    its bandwidth (`LOOP_HZ`)."""
    return peak(joint) * 2.0 * math.pi * LOOP_HZ[STACKS[kind(joint)][2]]


def jerk(joint):
    """Its joint's fastest angular jerk, rad/s^3: its torque's slew over its rotor's inertia
    through its box, unloaded (`armature`)."""
    return slew(joint) / armature(joint)


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
STATOR_J_K = {k: float(m.get('stator_j_k', round(_CP_HOUS * m['kg']))) for k, m in _LIB['motors'].items()}
_DISC_M, _REF_AMPS = INVERTERS[STACKS['knee'][2]][0], INVERTERS['coaxial_63100'][1]
_CONTACT_M2 = math.pi * (0.5 * _DISC_M) ** 2 / 3.0
MOUNT_K_W = float(_BUILD['mount_k_w']) if 'mount_k_w' in _BUILD else round(_TIM_M / (_K_TIM * _CONTACT_M2), 2)
OFF_MOUNT_K_W = (float(_BUILD['off_mount_k_w']) if 'off_mount_k_w' in _BUILD
                 else round(MOUNT_K_W + float(_BUILD['shell']['wall_m']) / (_K_CF_THRU * _CONTACT_M2), 2))


def cloth_air(joint=None):
    """Clothing thermal insulation multiplier (`>= 1.0`) for `joint` (or mean dressed factor)."""
    return sum(CLOTH_AIR.values()) / len(CLOTH_AIR) if joint is None else CLOTH_AIR.get(kind(joint), 1.20)


def stator(joint):
    """(stator_j_k J/K, mount_k_w K/W, cloth_air) of `joint`'s coaxial stator-housing assembly."""
    k = kind(joint)
    return STATOR_J_K[STACKS[k][0]], (OFF_MOUNT_K_W if k in BOARDS else MOUNT_K_W), cloth_air(joint)


def heat(joint):
    """(kt N m/A, winding ohm, the board's amps against the 100 A board's, the winding's J/K and
    K/W, the laminate's K/W to the air): what `machine.body.heat` keeps a joint's drive by."""
    s = of(joint)[1]
    return (kt(joint), r_ohm(joint), _REF_AMPS / s.rated, s.winding[0], s.winding[1] * COOLING,
            s.housing_k_w * COOLING)


_STRUCTURAL = None


def _structural():
    global _STRUCTURAL
    if _STRUCTURAL is None:
        from machine.body import structural
        wind_map = dict(_BUILD['wind']) if 'wind' in _BUILD else structural.wind_mrad()
        ser_map = ({k: v for k, v in _BUILD['series'].items() if k != 'track'}
                   if 'series' in _BUILD else structural.leaf_series(_BUILD.get('leaf')))
        trk = (float(_BUILD['series'].get('track', 0.0))
               if 'series' in _BUILD else structural.leaf_track(ser_map, armature))
        _STRUCTURAL = (wind_map, ser_map, trk)
    return _STRUCTURAL


def __getattr__(name: str) -> Any:
    if name in ('WIND', 'SERIES', 'TRACK'):
        return _structural()[('WIND', 'SERIES', 'TRACK').index(name)]
    raise AttributeError(name)
