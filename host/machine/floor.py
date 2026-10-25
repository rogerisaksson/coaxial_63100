"""The floor and what befalls her on it: a slab, a sill, a patch, a rug, a stair, bricks, a board.

Bodies (`ground`, `rug`) placed on her way (`place`) or parked (`park`), drawn as boxes (`props`).
"""
import math

from machine import figure
from machine.errors import MachineError

HOLE_M, HOLE_LONG_M, SILL_M, SILL_LONG_M = 0.10, 0.40, 0.035, 0.04
SLIP_LONG_M, SLIP_FRICTION = 0.5, 0.06
RUG_LONG_M, RUG_M, RUG_KG, RUG_FRICTION = 0.9, 0.01, 1.5, 0.55
SLAB_FROM_M, SEAM_M, SLAB_TO_M, PARKED_M = -20.0, 30.0, 10000.0, -50.0
#: A prop is drawn higher than this, m: parked at -2 (steps) and -5; a stair down's lowest step at -1.1.
SHOWN_Y = -1.5
DEEP_M = 0.2
#: A flight of stairs: steps, rise and run, m, its landing; a house's.
STEPS, RISE_M, RUN_M, TOP_M = 5, 0.15, 0.30, 1.5
#: The stairs takes' flight at her size: Ascending Stairs rises 0.421 m, runs 0.451 a stride.
TAKE_RISE_M, TAKE_RUN_M = 0.21, 0.225
#: The floor a flight or ramp reaches goes on this far, m: the base plane (the hole's bottom, 10 cm down)
#: lay over a descent's lower half; a 1.2 m top ended in a drop.
UPPER_M = 6.0
BRICK, BOARD, BOARD_HINGE_M, BOARD_KG = (0.06, 0.031, 0.125), (0.25, 0.005, 0.20), 0.02, 1.2
BOARD_K, BOARD_C = 3000.0, 20.0
#: The chair, as wide as her hands hang and a fist: 0.44 m wide they hung 4-5 cm past it.
CHAIR_SEAT, CHAIR_SEAT_H, CHAIR_BACK = (figure.ASIDE + figure.FIST_R, 0.02, 0.22), 0.45, (figure.ASIDE + figure.FIST_R, 0.20, 0.02)
CHAIR_LEG, CHAIR_LEG_IN = 0.02, 0.03
#: The low chair's seat at her popliteal height, her knee less her thigh's radius there: a human's deep sit,
#: soles flat; the lodge's is 5.3 cm over it.
CHAIR_LOW_H = figure.ANKLE_H + figure.SHANK - [c for c in figure.BODY if c[0] == 'thigh'][-1][1]
BED_MATTRESS, BED_MATTRESS_H, BED_FRAME = (0.50, 0.12, 1.0), 0.45, (0.52, 0.08, 1.02)
#: What lies on the floor and bears her soles alone.
TERRAIN = ('sill', 'slip', 'brick_left', 'brick_right', 'board_base', 'slope_ramp', 'slope_top', 'ice', 'snow', 'rug')
#: A ramp's grade, deg, and half sizes: 10, a steep street's.
SLOPE_ANGLE_DEG, SLOPE_HALF = 10.0, (0.60, 0.05, 1.44)
SLOPE_TOP_HALF, SLOPE_TOP_H = (0.60, 0.05, 0.60), 0.10
#: Ice under a rubber sole: 0.1; 0.02 is a skate blade's.
ICE_HALF, ICE_FRICTION = (0.60, 0.02, 1.25), 0.1
SNOW_HALF, SNOW_FRICTION = (0.60, 0.03, 1.25), 0.35
#: A patch's top over the floor, m: a loaded sole sinks 1-3 mm, more in snow; at 1 mm a sole stood through
#: the ice onto the floor and walked on its friction, 1.0 where 0.02.
SLIP_TOP_M, ICE_TOP_M, SNOW_TOP_M = 0.005, 0.005, 0.04

SCENARIOS = (
    'flat', 'lodge', 'chair', 'chair_low', 'bed', 'mattress', 'slope', 'slope_down', 'cross_slope',
    'stairs', 'stairs_down', 'stairs_take', 'ice', 'snow', 'asphalt', 'slip', 'obstacle', 'wall', 'tree',
    'sill', 'hole', 'rug', 'edge_behind',
)


def _rock():
    return math.degrees(math.atan(BOARD_HINGE_M / BOARD[0]))


def _steps(top=TOP_M, rise=RISE_M, run=RUN_M):
    return [('step%d' % k, (0.5, (k + 1) * rise / 2.0,
                             (run + (top if k == STEPS - 1 else 0.0)) / 2.0),
             (0.0, (k + 1) * rise / 2.0,
              k * run + (run + (top if k == STEPS - 1 else 0.0)) / 2.0))
            for k in range(STEPS)]


def told_stairs(rise=RISE_M, run=RUN_M):
    """The stairs told her (`worker.told_floor`): level to the first riser 0.30 m on (`events`), STEPS
    treads of `run` at `rise`, the landing TOP_M long."""
    pts = [(-2.0, 0.0), (0.30, 0.0)]
    for k in range(STEPS):
        pts += [(0.30 + k * run, (k + 1) * rise), (0.30 + (k + 1) * run + TOP_M * (k == STEPS - 1), (k + 1) * rise)]
    return pts


def grounds(m):
    """Her ground's bodies in `m`: the floor and its slabs, TERRAIN, the steps, `room.GROUND`."""
    from machine import room
    ids = {m.body(b).name: b for b in range(m.nbody)}
    laid = {ids[n] for n in TERRAIN + tuple(n for n, _h, _a in _steps()) + room.GROUND if n in ids}
    return {0} | {b for b in range(m.nbody) if m.body_rootid[b] in laid}


def lay(name):
    """(x, z) she is laid at for scene `name`: the lodge's door line, else her origin."""
    from machine import room
    return {'lodge': (room.STAIR_X, 0.0)}.get(name, (0.0, 0.0))


def _sized(world, body, half):
    """`body`'s box `half` its sizes, its contact bounds with them: grown 6 m past its compiled 1.2, a
    slope's top bore nothing 0.85 m from its middle and she fell through its edge."""
    m = world.model
    b = m.body(body).id
    g = m.body_geomadr[b]
    m.geom_size[g][:3], m.geom_aabb[g][3:], m.bvh_aabb[m.body_bvhadr[b]][3:] = half, half, half
    m.geom_rbound[g] = math.sqrt(sum(h * h for h in half))


def _beyond(world, along, ahead, pad):
    """The slab taken away from `along` on, `ahead` the way she goes along z (1 or -1)."""
    slab(world, along - pad, SLAB_TO_M) if ahead >= 0.0 else slab(world, SLAB_FROM_M, along + pad)


def _base(world, y):
    """The base plane `y` m up: the hole's bottom."""
    world.model.geom_pos[world.model.geom('floor').id][1] = y


def _chair(name, seat_h, contacts):
    """A chair's body, parked: its seat `seat_h` m up, its back, four legs."""
    leg_h = (seat_h - 2 * CHAIR_SEAT[1]) / 2.0
    return ('<body name="%s" mocap="true" pos="0 -5 0">'
            '<geom name="%s_seat" type="box" size="%g %g %g" pos="0 %g 0"%s/>'
            '<geom name="%s_back" type="box" size="%g %g %g" pos="0 %g %g"%s/>' % (
                (name, name) + CHAIR_SEAT + (seat_h - CHAIR_SEAT[1], contacts, name)
                + CHAIR_BACK + (seat_h + CHAIR_BACK[1], CHAIR_SEAT[2], contacts))
            + ''.join('<geom name="%s_leg_%s" type="box" size="%g %g %g" pos="%g %g %g"%s/>' % (
                name, leg, CHAIR_LEG, leg_h, CHAIR_LEG, sx * (CHAIR_SEAT[0] - CHAIR_LEG_IN), leg_h,
                sz * (CHAIR_SEAT[2] - CHAIR_LEG_IN), contacts)
                for leg, sz, sx in (('fl', -1, -1), ('fr', -1, 1), ('bl', 1, -1), ('br', 1, 1))) + '</body>')


def ground(contacts, give, torsion):
    """The worldbody's floor before her figure: the plane, the slabs, the mocap props."""
    return [
        '<geom name="floor" type="plane" size="100 100 0.1" pos="0 %g 0" '
        'quat="0.7071068 -0.7071068 0 0"%s/>' % (-HOLE_M, contacts),
        '<geom name="slab_a" type="box" size="50 %g %g" pos="0 %g %g"%s/>' % (
            DEEP_M / 2.0, (SLAB_TO_M - SLAB_FROM_M) / 2.0, -DEEP_M / 2.0,
            (SLAB_FROM_M + SLAB_TO_M) / 2.0, contacts),
        '<geom name="slab_b" type="box" size="50 %g %g" pos="0 %g %g"%s/>' % (
            DEEP_M / 2.0, (SLAB_TO_M - SLAB_FROM_M) / 2.0, -DEEP_M / 2.0,
            (SLAB_FROM_M + SLAB_TO_M) / 2.0, contacts),
        '<body name="sill" mocap="true" pos="0 -1 0"><geom type="box" size="0.5 %g %g" '
        'pos="0 %g 0"%s/></body>' % ((SILL_M + DEEP_M) / 2.0, SILL_LONG_M / 2.0,
                                     (SILL_M - DEEP_M) / 2.0, contacts),
        '<body name="slip" mocap="true" pos="0 -1 0"><geom type="box" size="0.5 %g %g" '
        'pos="0 %g 0" priority="1" condim="4" friction="%g %g 0.001"%s%s/></body>' % (
            (SLIP_TOP_M + DEEP_M) / 2.0, SLIP_LONG_M / 2.0, (SLIP_TOP_M - DEEP_M) / 2.0,
            SLIP_FRICTION, torsion, give, contacts)] + [
        '<body name="%s" mocap="true" pos="0 -2 0"><geom type="box" size="%g %g %g"%s/></body>'
        % ((name,) + half + (contacts,)) for name, half, _at in _steps()] + [
        '<body name="brick_%s" mocap="true" pos="0 -1 0"><geom type="box" size="%g %g %g"%s/>'
        '</body>' % ((side,) + BRICK + (contacts,)) for side in ('left', 'right')] + [
        '<body name="wall" mocap="true" pos="0 -5 0"><geom type="box" size="2.0 1.2 0.05"%s/></body>'
        % (contacts,),
        '<body name="tree" mocap="true" pos="0 -5 0"><geom type="cylinder" size="0.15 1.5" '
        'quat="0.7071068 -0.7071068 0 0"%s/></body>' % (contacts,),
        _chair('chair', CHAIR_SEAT_H, contacts), _chair('chair_low', CHAIR_LOW_H, contacts),
        '<body name="prop_bed" mocap="true" pos="0 -5 0">'
        # a foam's give: she lies 7.5 cm in, 5.5 at 0.04 s and 5 cm, and a quicker lie fell there
        '<geom name="bed_mattress" type="box" size="%g %g %g" pos="0 %g 0" priority="2" condim="4"'
        ' solref="0.08 1" solimp="0.8 0.95 0.10"%s/>'
        '<geom name="bed_frame" type="box" size="%g %g %g" pos="0 %g 0"%s/>'
        '<geom name="bed_headboard" type="box" size="%g 0.25 0.03" pos="0 %g %g"%s/>'
        '</body>' % (
            BED_MATTRESS + (BED_MATTRESS_H - BED_MATTRESS[1], contacts)
            + BED_FRAME + (BED_FRAME[1], contacts)
            + (BED_FRAME[0], BED_MATTRESS_H + 0.25, BED_MATTRESS[2] + 0.03, contacts)),
        '<body name="slope_ramp" mocap="true" pos="0 -5 0">'
        '<geom name="slope_ramp" type="box" size="%g %g %g"%s/></body>' % (SLOPE_HALF + (contacts,)),
        '<body name="slope_top" mocap="true" pos="0 -5 0">'
        '<geom name="slope_top" type="box" size="%g %g %g"%s/></body>' % (SLOPE_TOP_HALF + (contacts,)),
        '<body name="ice" mocap="true" pos="0 -5 0">'
        '<geom name="ice" type="box" size="%g %g %g" priority="2" condim="4" friction="%g %g 0.001"%s%s/></body>' % (
            ICE_HALF + (ICE_FRICTION, torsion, give, contacts)),
        '<body name="snow" mocap="true" pos="0 -5 0">'
        '<geom name="snow" type="box" size="%g %g %g" priority="2" condim="4" friction="%g %g 0.001"'
        ' solref="0.04 0.8" solimp="0.8 0.95 0.05"%s/></body>' % (
            SNOW_HALF + (SNOW_FRICTION, torsion, contacts))]


def rug(give, friction, torsion):
    """The rug's body and the board's, after her figure."""
    return ['<body name="board_base" mocap="true" pos="0 -1 0"><body name="board" pos="0 %g 0">'
            '<joint name="board" type="hinge" axis="0 0 1" limited="true" range="%g %g" '
            'stiffness="%g" damping="%g"/><geom type="box" size="%g %g %g" pos="0 %g 0" '
            'mass="%g" contype="1" conaffinity="2"/></body></body>' % (
                (BOARD_HINGE_M, -_rock(), _rock(), BOARD_K, BOARD_C) + BOARD + (BOARD[1], BOARD_KG)),
            '<body name="rug" pos="0 0.1 %g"><freejoint name="rug"/>' % PARKED_M,
            '<geom name="rug_under" type="box" size="0.3 %g %g" pos="0 %g 0" mass="%g" '
            'priority="1" friction="%g 0.005 0.0001" contype="3" conaffinity="3"/>' % (
                RUG_M / 4.0, RUG_LONG_M / 2.0, -RUG_M / 4.0, RUG_KG / 2.0, RUG_FRICTION),
            '<geom name="rug_top" type="box" size="0.3 %g %g" pos="0 %g 0" mass="%g" '
            'priority="2" condim="4" friction="%g %g 0.001" contype="3" conaffinity="3"%s/>' % (
                RUG_M / 4.0, RUG_LONG_M / 2.0, RUG_M / 4.0, RUG_KG / 2.0, friction, torsion, give),
            '</body>']


def slab(world, a_to, b_from):
    m = world.model
    for name, z0, z1 in (('slab_a', SLAB_FROM_M, a_to), ('slab_b', b_from, SLAB_TO_M)):
        g = m.geom(name).id
        m.geom_size[g][2], m.geom_pos[g][2] = (z1 - z0) / 2.0, (z0 + z1) / 2.0


def park(world):
    slab(world, SEAM_M, SEAM_M)
    _base(world, -HOLE_M)
    _sized(world, 'slope_top', SLOPE_TOP_HALF)
    for name, half, _at in _steps():
        _sized(world, name, half)
    world.hole_x = 0.0
    for name in ('sill', 'slip', 'brick_left', 'brick_right', 'board_base', 'wall', 'tree',
                 'chair', 'chair_low', 'prop_bed', 'slope_ramp', 'slope_top', 'ice', 'snow'):
        mocap(world, name, (0.0, -5.0, 0.0))
    for name, _half, _at in _steps():
        mocap(world, name, (0.0, -2.0, 0.0))
    lay_rug(world, (0.0, PARKED_M))


def _turn(heading):
    return (math.cos(heading / 2.0), 0.0, math.sin(heading / 2.0), 0.0)


def _on(at, heading, x, y, z):
    c, s = math.cos(heading), math.sin(heading)
    return (at[0] + x * c + z * s, y, at[1] - x * s + z * c)


def mocap(world, name, at, heading=0.0):
    k = world.model.body_mocapid[world.model.body(name).id]
    world.data.mocap_pos[k], world.data.mocap_quat[k] = at, _turn(heading)


def lay_rug(world, at, heading=0.0):
    m, d = world.model, world.data
    joint = m.joint('rug').id
    adr, dof = m.jnt_qposadr[joint], m.jnt_dofadr[joint]
    d.qpos[adr:adr + 7] = _on(at, heading, 0.0, RUG_M / 2.0 + 0.001, RUG_LONG_M / 2.0) + _turn(heading)
    d.qvel[dof:dof + 6] = 0.0


def _place_slope(world, kind, at, heading, top):
    rad = math.radians(SLOPE_ANGLE_DEG)
    c, s = math.cos(heading), math.sin(heading)
    ch, sh = math.cos(heading / 2.0), math.sin(heading / 2.0)
    hz, hy = SLOPE_HALF[2] * math.cos(rad), SLOPE_HALF[2] * math.sin(rad)
    k_ramp = world.model.body_mocapid[world.model.body('slope_ramp').id]
    if kind == 'slope':
        cr, sr = math.cos(-rad / 2.0), math.sin(-rad / 2.0)
        ramp_at = _on(at, heading, 0.0, hy - SLOPE_HALF[1] * math.cos(rad), hz + SLOPE_HALF[1] * math.sin(rad))
        world.data.mocap_pos[k_ramp], world.data.mocap_quat[k_ramp] = ramp_at, (ch * cr, ch * sr, sh * cr, -sh * sr)
        _sized(world, 'slope_top', (SLOPE_TOP_HALF[0], SLOPE_TOP_HALF[1], top / 2.0))
        mocap(world, 'slope_top', _on(at, heading, 0.0, 2.0 * hy - SLOPE_TOP_HALF[1], 2.0 * hz + top / 2.0), heading)
        return
    cr, sr = math.cos(rad / 2.0), math.sin(rad / 2.0)
    pad = abs(s) * SLOPE_HALF[0]
    if kind == 'slope_down':
        _beyond(world, at[1], c, pad)
        _base(world, -2.0 * hy)
    else:
        span = 2.0 * SLOPE_HALF[2] * c
        slab(world, min(at[1], at[1] + span) - pad, max(at[1], at[1] + span) + pad)
    world.hole_x = None
    if kind == 'slope_down':
        ramp_at = _on(at, heading, 0.0, -hy - SLOPE_HALF[1] * math.cos(rad), hz - SLOPE_HALF[1] * math.sin(rad))
        q = (ch * cr, ch * sr, sh * cr, -sh * sr)
    else:
        ramp_at = _on(at, heading, 0.0, -SLOPE_HALF[1] * math.cos(rad), SLOPE_HALF[2])
        q = (ch * cr, sh * sr, sh * cr, ch * sr)
    world.data.mocap_pos[k_ramp], world.data.mocap_quat[k_ramp] = ramp_at, q


_SIMPLE_PROPS = {
    'sill': ('sill', 0.0, 0.0), 'slip': ('slip', 0.0, 0.0), 'chair': ('chair', 0.0, 0.0),
    'wall': ('wall', 1.2, 0.0), 'tree': ('tree', 1.5, 0.0),
    'bed': ('prop_bed', 0.0, 0.0), 'prop_bed': ('prop_bed', 0.0, 0.0),
    'mattress': ('prop_bed', 0.004 - BED_MATTRESS_H, 0.0),
    'asphalt': ('slope_top', 0.002 - SLOPE_TOP_HALF[1], SLOPE_TOP_HALF[2]),
    'ice': ('ice', ICE_TOP_M - ICE_HALF[1], ICE_HALF[2]),
    'snow': ('snow', SNOW_TOP_M - SNOW_HALF[1], SNOW_HALF[2]),
}


def place(world, kind, z, x=0.0, heading=0.0, top=None, rise=RISE_M, run=RUN_M):
    c, s = math.cos(heading), math.sin(heading)
    at = (x * c + z * s, -x * s + z * c)
    if kind in _SIMPLE_PROPS:
        body, dy, dz = _SIMPLE_PROPS[kind]
        mocap(world, body, (at[0], dy, at[1] + dz), heading)
        return
    if kind in ('slope', 'slope_down', 'cross_slope'):
        _place_slope(world, kind, at, heading, 2.0 * SLOPE_TOP_HALF[2] if top is None else top)
        return
    if kind == 'hole':
        slab(world, at[1] - HOLE_LONG_M / 2.0, at[1] + HOLE_LONG_M / 2.0)
        world.hole_x = at[0]
        return
    if kind == 'rug':
        lay_rug(world, at, heading)
        return
    if kind == 'stairs':
        for name, half, (dx, y, dz) in _steps(TOP_M if top is None else top, rise, run):
            _sized(world, name, half)
            mocap(world, name, _on(at, heading, dx, y, dz), heading)
        return
    if kind == 'stairs_down':
        _beyond(world, at[1], c, abs(s) * 0.5)
        _base(world, -STEPS * RISE_M)
        world.hole_x = None
        for k, (name, half, (dx, _y, dz)) in enumerate(_steps(TOP_M if top is None else top)):
            _sized(world, name, half)
            mocap(world, name, _on(at, heading, dx, -(k + 1) * RISE_M - half[1], dz), heading)
        return
    if kind == 'board':
        gap = max(BOARD[0], BOARD[2]) + 0.02
        slab(world, at[1] - gap, at[1] + gap)
        world.hole_x = at[0]
        mocap(world, 'board_base', (at[0], -HOLE_M, at[1]), heading)
        return
    raise MachineError('no floor event %r' % kind)



def behind(stunt, half):
    """m along her way from her ankles to a prop's middle, `half` m in from its edge behind her, her
    heels the stunt's standoff off it (`maneuver.standoff`)."""
    from machine import maneuver
    return -(figure.HEEL + maneuver.standoff(stunt) + half)


def _on_chair(world):
    mocap(world, 'chair', (0.0, 0.0, 0.0), math.pi)
    world.data.qpos[1] += CHAIR_SEAT_H


def _edge_behind(world):
    """A floor a step's rise up ends half a tread behind her heels: a step back lands off its edge."""
    _sized(world, 'slope_top', (SLOPE_TOP_HALF[0], RISE_M / 2.0, UPPER_M / 2.0))
    mocap(world, 'slope_top', (0.0, RISE_M / 2.0, UPPER_M / 2.0 - figure.HEEL - RUN_M / 2.0))
    world.data.qpos[1] += RISE_M


def scenario(world, name, z=1.2, x=0.0):
    park(world)
    if name in ('flat', 'clear', 'none'):
        return
    from machine import room
    low = -STEPS * RISE_M
    cases = {
        # the deck's stair down on the door's line, snow, a spruce; the rug, the sill
        'lodge': lambda: (
            place(world, 'stairs_down', z=room.DECK_TO, x=room.STAIR_X, top=UPPER_M),
            mocap(world, 'snow', (room.STAIR_X, low + SNOW_TOP_M - SNOW_HALF[1],
                                  room.DECK_TO + STEPS * RUN_M + SNOW_HALF[2])),
            mocap(world, 'tree', (-1.6, low + 1.5, room.DECK_TO + 3.4)),
            place(world, 'rug', z=1.5, x=0.0), place(world, 'sill', z=room.TO, x=room.STAIR_X)),
        'chair': lambda: place(world, 'chair', z=0.88, x=x),
        'chair_behind': lambda: mocap(world, 'chair', (0.0, 0.0, behind('sit_chair', CHAIR_SEAT[2])), math.pi),
        'chair_low': lambda: place(world, 'chair_low', z=0.88, x=x),
        'chair_low_behind': lambda: mocap(world, 'chair_low', (0.0, 0.0, behind('sit_chair', CHAIR_SEAT[2])), math.pi),
        'chair_ahead': lambda: mocap(world, 'chair', (0.0, 0.0, 0.40), math.pi),
        'on_chair': lambda: _on_chair(world),
        'edge_behind': lambda: _edge_behind(world),
        'bed': lambda: place(world, 'bed', z=1.8, x=x),
        'bed_behind': lambda: mocap(world, 'prop_bed', (0.0, 0.0, -0.10 - BED_MATTRESS[0]), math.pi / 2.0),
        'mattress': lambda: place(world, 'mattress', z=0.6, x=x),
        'mattress_under': lambda: mocap(world, 'prop_bed', (0.0, 0.004 - BED_MATTRESS_H, -0.20), 0.0),
        'slope': lambda: place(world, 'slope', z=0.25, x=x, top=UPPER_M),
        'slope_down': lambda: place(world, 'slope_down', z=0.25, x=x),
        'cross_slope': lambda: place(world, 'cross_slope', z=0.25, x=x),
        'stairs': lambda: place(world, 'stairs', z=0.30, x=x, top=UPPER_M),
        'stairs_down': lambda: place(world, 'stairs_down', z=0.30, x=x, top=UPPER_M),
        'stairs_take': lambda: place(world, 'stairs', z=0.30, x=x, top=UPPER_M, rise=TAKE_RISE_M, run=TAKE_RUN_M),
        'ice': lambda: place(world, 'ice', z=0.15, x=x),
        'snow': lambda: place(world, 'snow', z=0.15, x=x),
        'asphalt': lambda: place(world, 'asphalt', z=0.15, x=x),
        'slip': lambda: place(world, 'slip', z=0.35 if z == 1.2 else z, x=x),
        'wall': lambda: place(world, 'wall', z=0.50, x=x),
        'tree': lambda: place(world, 'tree', z=0.50, x=x),
        'obstacle': lambda: (place(world, 'wall', z=1.10, x=x), place(world, 'tree', z=0.55, x=x + 0.25)),
        'obstacles': lambda: (place(world, 'wall', z=1.10, x=x), place(world, 'tree', z=0.55, x=x + 0.25)),
        'urban': lambda: (place(world, 'wall', z=1.10, x=x), place(world, 'tree', z=0.55, x=x + 0.25)),
        'sill': lambda: place(world, 'sill', z=0.35 if z == 1.2 else z, x=x),
        'hole': lambda: place(world, 'hole', z=0.55 if z == 1.2 else z, x=x),
        'rug': lambda: place(world, 'rug', z=0.20 if z == 1.2 else z, x=x),
    }
    if name in cases:
        cases[name]()
        return
    raise MachineError('unknown scenario %r: one of %s' % (name, ', '.join(SCENARIOS)))


def bricks(world, soles, heading=0.0):
    for name, at in zip(('brick_left', 'brick_right'), soles):
        mocap(world, name, (at[0], BRICK[1], at[1]), heading)


def take(world, name):
    mocap(world, name, (0.0, -1.0, 0.0))


def board(world, k, c):
    m, j = world.model, world.model.joint('board').id
    m.jnt_stiffness[j], m.dof_damping[m.jnt_dofadr[j]] = k, c


def _ids(world):
    """The props' ids, looked up once."""
    ids = getattr(world, '_prop_ids', None)
    if ids is None:
        m = world.model
        events = [('stairs' if n.startswith('step') else n.split('_')[0], n, m.body(n).id)
                  for n in ('sill', 'slip', 'brick_left', 'brick_right', 'board', 'wall', 'tree',
                            'ice', 'snow') + tuple(n for n, _h, _a in _steps())]
        ids = world._prop_ids = {
            'slabs': (m.geom('slab_a').id, m.geom('slab_b').id),
            'events': [(kind, b, m.body_geomadr[b], name in ('board', 'wall', 'tree') or name.startswith('step'))
                       for kind, name, b in events],
            'rug': m.body('rug').id,
            'chair': (m.body('chair').id, m.geom('chair_seat').id, m.geom('chair_back').id),
            'chair_low': (m.body('chair_low').id, m.geom('chair_low_seat').id, m.geom('chair_low_back').id),
            'bed': (m.body('prop_bed').id, m.geom('bed_mattress').id, m.geom('bed_headboard').id),
            'slopes': [(m.body(n).id, m.geom(n).id) for n in ('slope_ramp', 'slope_top')]}
    return ids


def dents(world, np):
    """[(x, y, z, m)]: each contact in her mattress, world, and how deep."""
    d, mat, ncon = world.data, _ids(world)['bed'][1], world.data.ncon
    if not ncon:
        return []
    geom, dist = d.contact.geom[:ncon], d.contact.dist[:ncon]
    hit = np.flatnonzero((geom == mat).any(axis=1) & (dist < 0.0))
    pos = d.contact.pos[:ncon]
    return [tuple(float(v) for v in pos[i]) + (-float(dist[i]),) for i in hit]


def props(world):
    m, d, out = world.model, world.data, []
    ids = _ids(world)
    a, b = ids['slabs']
    gap = (m.geom_pos[a][2] + m.geom_size[a][2], m.geom_pos[b][2] - m.geom_size[b][2])
    if gap[1] - gap[0] > 1e-6 and getattr(world, 'hole_x', 0.0) is not None:
        out.append(('hole', (getattr(world, 'hole_x', 0.0), -HOLE_M / 2.0, float(sum(gap) / 2.0)),
                    (0.5, HOLE_M / 2.0, float((gap[1] - gap[0]) / 2.0)), ((1, 0, 0), (0, 1, 0), (0, 0, 1))))
    for kind, body, g, whole in ids['events']:
        if d.xpos[body][1] > SHOWN_Y:
            c, half = d.geom_xpos[g].tolist(), m.geom_size[g].tolist()
            top = c[1] + half[1]
            low = c[1] - half[1] if whole else max(0.0, c[1] - half[1])
            out.append((kind, (c[0], (top + low) / 2.0, c[2]), (half[0], (top - low) / 2.0, half[2] or half[0]),
                        tuple(map(tuple, d.xmat[body].reshape(3, 3).tolist()))))
    rug_id = ids['rug']
    if d.xpos[rug_id][2] > PARKED_M + 1.0:
        out.append(('rug', tuple(d.xpos[rug_id].tolist()), (0.3, RUG_M / 2.0, RUG_LONG_M / 2.0),
                    tuple(map(tuple, d.xmat[rug_id].reshape(3, 3).tolist()))))
    for chair_id, seat, back in (ids['chair'], ids['chair_low']):
        if d.xpos[chair_id][1] > SHOWN_Y:
            c_mat = tuple(map(tuple, d.xmat[chair_id].reshape(3, 3).tolist()))
            for g in (seat, back):
                out.append(('chair', tuple(d.geom_xpos[g].tolist()), tuple(m.geom_size[g].tolist()), c_mat))
    bed_id, g_mat, g_hb = ids['bed']
    if d.xpos[bed_id][1] > SHOWN_Y:
        b_mat = tuple(map(tuple, d.xmat[bed_id].reshape(3, 3).tolist()))
        out.append(('bed', tuple(d.geom_xpos[g_mat].tolist()), tuple(m.geom_size[g_mat].tolist()), b_mat))
        if d.xpos[bed_id][1] > -0.1:
            out.append(('bed', tuple(d.geom_xpos[g_hb].tolist()), tuple(m.geom_size[g_hb].tolist()), b_mat))
    for bid, g in ids['slopes']:
        if d.xpos[bid][1] > SHOWN_Y:
            out.append(('slope', tuple(d.geom_xpos[g].tolist()), tuple(m.geom_size[g].tolist()),
                        tuple(map(tuple, d.xmat[bid].reshape(3, 3).tolist()))))
    return out

