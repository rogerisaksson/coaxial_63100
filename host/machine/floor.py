"""The floor and what befalls her on it: a slab, a sill, a patch, a rug, a stair, bricks, a board.

The model's bodies (`ground` before her figure, `rug` after it), each event placed on the
walk's line (`place`) or parked out of her way (`park`), and drawn as boxes (`props`) -
functions of the `physics.World`.
"""
import math

from machine.errors import MachineError

#: The floor's events, placed on the walk's line by `World.terrain`: a hole HOLE_M deep and
#: HOLE_LONG_M long - the slab in two, the plane below showing through the gap; a sill SILL_M
#: high and SILL_LONG_M long; a patch SLIP_LONG_M long at SLIP_FRICTION; a loose rug RUG_LONG_M
#: long, RUG_M thick and RUG_KG, gripping the sole as the floor does and sliding on the floor at
#: RUG_FRICTION. Whole, the slab's halves meet at SEAM_M; it ends at SLAB_TO_M, past any walk -
#: at 80 m a walk stepped off it 100 s in, 3 cm down, and fell or sank into a crouch.
#: The halves are
#: compiled over the whole span and cut to size, the sill and the patch are mocap bodies: a
#: geom moved or grown past its compiled bounds is missed by the broadphase (the rug fell
#: through a slab grown 27 m, a box through a sill moved 1 m). At 0.15 the patch let the stance
#: foot creep 3 mm (the walk asks 0.17), at 0.06 it slid 10 cm back under the push-off; the rug
#: at 0.3 lay still under a landing and a push-off (the sole's shear 100 N, the rug's hold 165).
#: The skimming toes brushed a 4 cm sill, 350 N for 20 ms, unseen; 6 cm caught
#: them for 0.11 s and she fell, or stumbled over it lifted (`landing.tripped`).
HOLE_M, HOLE_LONG_M, SILL_M, SILL_LONG_M = 0.10, 0.40, 0.06, 0.04
SLIP_LONG_M, SLIP_FRICTION = 0.5, 0.06
RUG_LONG_M, RUG_M, RUG_KG, RUG_FRICTION = 0.9, 0.01, 1.5, 0.1
SLAB_FROM_M, SEAM_M, SLAB_TO_M, PARKED_M = -20.0, 30.0, 10000.0, -50.0
#: The slab, the sill and the patch reach DEEP_M under the floor: the slab 3 cm deep, a landing
#: arm passed its middle and was held inside, its underside pressing it onto the plane - 15 kN,
#: 2 falls in 13. The patch stands SLIP_TOP_M proud.
DEEP_M, SLIP_TOP_M = 0.2, 0.001

#: A stair of STEPS steps, each RISE_M up and RUN_M on - a stride's step, as she has no eyes to
#: fit her steps to it - and TOP_M of landing at the top: a box a step, mocap bodies too.
STEPS, RISE_M, RUN_M, TOP_M = 5, 0.08, 0.425, 1.5

#: Two bricks, BRICK half sizes (250 long along her way, 120 wide, 62 high), one under each
#: sole as she lands (`bricks`); one taken away (`take`) is parked under the plane. A balance
#: board (`place` 'board'): a plank BOARD half sizes, BOARD_KG, hinged BOARD_HINGE_M over the
#: plane in a gap of the slab, its top at the floor, rocking about her way - turned a quarter,
#: across it - until an edge meets the plane; its hinge sprung BOARD_K N m/rad - stiff, it
#: gives a little; free at 0 - and damped BOARD_C N m s/rad (`board`).
BRICK, BOARD, BOARD_HINGE_M, BOARD_KG = (0.06, 0.031, 0.125), (0.25, 0.005, 0.20), 0.02, 1.2
BOARD_K, BOARD_C = 3000.0, 20.0

#: A chair for sitting and hopping: seat at SEAT_H, size (width, thickness, depth).
CHAIR_SEAT, CHAIR_SEAT_H, CHAIR_BACK = (0.22, 0.02, 0.22), 0.45, (0.22, 0.20, 0.02)
#: A soft bed: mattress half size (width, half-thickness, half-length), frame.
BED_MATTRESS, BED_MATTRESS_H, BED_FRAME = (0.50, 0.12, 1.0), 0.45, (0.52, 0.08, 1.02)
#: A slope ramp: 10 degree incline, half length, half width, half thickness.
SLOPE_ANGLE_DEG, SLOPE_HALF = 10.0, (0.60, 0.05, 1.44)
SLOPE_TOP_HALF, SLOPE_TOP_H = (0.60, 0.25, 0.60), 0.50
#: An ice sheet: low friction (0.02), width 1.2m, length 2.5m.
ICE_HALF, ICE_FRICTION = (0.60, 0.02, 1.25), 0.02
#: A snow patch: compliant and high rolling drag, width 1.2m, length 2.5m.
SNOW_HALF, SNOW_FRICTION = (0.60, 0.03, 1.25), 0.35

#: The environment scenarios that can be placed on the floor:
SCENARIOS = ('flat', 'glass_room', 'chair', 'bed', 'slope', 'stairs', 'ice', 'snow', 'obstacle', 'sill', 'hole', 'rug')


def _rock():
    """How far the board rocks either way, degrees: an edge on the plane."""
    return math.degrees(math.atan(BOARD_HINGE_M / BOARD[0]))


def _steps():
    """Each step's (name, half sizes, centre over the first riser): up from the floor."""
    return [('step%d' % k, (0.5, (k + 1) * RISE_M / 2.0,
                             (RUN_M + (TOP_M if k == STEPS - 1 else 0.0)) / 2.0),
             (0.0, (k + 1) * RISE_M / 2.0,
              k * RUN_M + (RUN_M + (TOP_M if k == STEPS - 1 else 0.0)) / 2.0))
            for k in range(STEPS)]


def ground(contacts, give, torsion):
    """The worldbody's floor before her figure: the plane, the slab's halves, the sill and
    the patch parked under the plane; `contacts` and `give` the floor's contact attributes."""
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
        '<body name="tree" mocap="true" pos="0 -5 0"><geom type="cylinder" size="0.15 1.5"%s/></body>'
        % (contacts,),
        '<body name="chair" mocap="true" pos="0 -5 0">'
        '<geom name="chair_seat" type="box" size="%g %g %g" pos="0 %g 0"%s/>'
        '<geom name="chair_back" type="box" size="%g %g %g" pos="0 %g %g"%s/>'
        '<geom name="chair_leg_fl" type="box" size="0.02 %g 0.02" pos="-0.19 %g -0.19"%s/>'
        '<geom name="chair_leg_fr" type="box" size="0.02 %g 0.02" pos="0.19 %g -0.19"%s/>'
        '<geom name="chair_leg_bl" type="box" size="0.02 %g 0.02" pos="-0.19 %g 0.19"%s/>'
        '<geom name="chair_leg_br" type="box" size="0.02 %g 0.02" pos="0.19 %g 0.19"%s/>'
        '</body>' % (
            CHAIR_SEAT + (CHAIR_SEAT_H - CHAIR_SEAT[1], contacts)
            + CHAIR_BACK + (CHAIR_SEAT_H + CHAIR_BACK[1], CHAIR_SEAT[2], contacts)
            + ((CHAIR_SEAT_H - 2 * CHAIR_SEAT[1]) / 2.0, (CHAIR_SEAT_H - 2 * CHAIR_SEAT[1]) / 2.0, contacts)
            + ((CHAIR_SEAT_H - 2 * CHAIR_SEAT[1]) / 2.0, (CHAIR_SEAT_H - 2 * CHAIR_SEAT[1]) / 2.0, contacts)
            + ((CHAIR_SEAT_H - 2 * CHAIR_SEAT[1]) / 2.0, (CHAIR_SEAT_H - 2 * CHAIR_SEAT[1]) / 2.0, contacts)
            + ((CHAIR_SEAT_H - 2 * CHAIR_SEAT[1]) / 2.0, (CHAIR_SEAT_H - 2 * CHAIR_SEAT[1]) / 2.0, contacts)),
        '<body name="prop_bed" mocap="true" pos="0 -5 0">'
        '<geom name="bed_mattress" type="box" size="%g %g %g" pos="0 %g 0" priority="2" condim="4"'
        ' solref="0.04 1" solimp="0.8 0.95 0.05"%s/>'
        '<geom name="bed_frame" type="box" size="%g %g %g" pos="0 %g 0"%s/>'
        '<geom name="bed_headboard" type="box" size="%g 0.25 0.03" pos="0 %g %g"%s/>'
        '</body>' % (
            BED_MATTRESS + (BED_MATTRESS_H - BED_MATTRESS[1], contacts)
            + BED_FRAME + (BED_FRAME[1], contacts)
            + (BED_FRAME[0], BED_MATTRESS_H + 0.25, BED_MATTRESS[2] + 0.03, contacts)),
        '<body name="slope_ramp" mocap="true" pos="0 -5 0">'
        '<geom name="slope_ramp" type="box" size="%g %g %g"%s/></body>' % (
            SLOPE_HALF + (contacts,)),
        '<body name="slope_top" mocap="true" pos="0 -5 0">'
        '<geom name="slope_top" type="box" size="%g %g %g"%s/></body>' % (
            SLOPE_TOP_HALF + (contacts,)),
        '<body name="ice" mocap="true" pos="0 -5 0">'
        '<geom name="ice" type="box" size="%g %g %g" priority="2" condim="4" friction="%g %g 0.001"%s%s/></body>' % (
            ICE_HALF + (ICE_FRICTION, torsion, give, contacts)),
        '<body name="snow" mocap="true" pos="0 -5 0">'
        '<geom name="snow" type="box" size="%g %g %g" priority="2" condim="4" friction="%g %g 0.001"'
        ' solref="0.04 0.8" solimp="0.8 0.95 0.05"%s/></body>' % (
            SNOW_HALF + (SNOW_FRICTION, torsion, contacts))]


def rug(give, friction, torsion):
    """The rug's body and the board's, after her figure - their joints after hers, the
    pelvis's first (`World.reset`): a free box, its underside sliding on the floor; the plank
    hinged on its parked base."""
    return ['<body name="board_base" mocap="true" pos="0 -1 0"><body name="board" pos="0 %g 0">'
            '<joint name="board" type="hinge" axis="0 0 1" limited="true" range="%g %g" '
            'stiffness="%g" damping="%g"/><geom type="box" size="%g %g %g" pos="0 %g 0" '
            'mass="%g" contype="1" conaffinity="2"/></body></body>' % (
                (BOARD_HINGE_M, -_rock(), _rock(), BOARD_K, BOARD_C) + BOARD
                + (BOARD[1], BOARD_KG)),
            '<body name="rug" pos="0 0.1 %g"><freejoint name="rug"/>' % PARKED_M,
            '<geom name="rug_under" type="box" size="0.3 %g %g" pos="0 %g 0" mass="%g" '
            'priority="1" friction="%g 0.005 0.0001" contype="3" conaffinity="3"/>' % (
                RUG_M / 4.0, RUG_LONG_M / 2.0, -RUG_M / 4.0, RUG_KG / 2.0, RUG_FRICTION),
            '<geom name="rug_top" type="box" size="0.3 %g %g" pos="0 %g 0" mass="%g" '
            'priority="2" condim="4" friction="%g %g 0.001" contype="3" conaffinity="3"%s/>' % (
                RUG_M / 4.0, RUG_LONG_M / 2.0, RUG_M / 4.0, RUG_KG / 2.0, friction, torsion,
                give),
            '</body>']


def slab(world, a_to, b_from):
    """The slab's halves: from SLAB_FROM_M to `a_to` and from `b_from` to SLAB_TO_M, m."""
    m = world.model
    for name, z0, z1 in (('slab_a', SLAB_FROM_M, a_to), ('slab_b', b_from, SLAB_TO_M)):
        g = m.geom(name).id
        m.geom_size[g][2], m.geom_pos[g][2] = (z1 - z0) / 2.0, (z0 + z1) / 2.0


def park(world):
    """The floor whole, its events out of the way: the sill and the patch under the plane,
    the rug on the floor PARKED_M back."""
    slab(world, SEAM_M, SEAM_M)
    for name in ('sill', 'slip', 'brick_left', 'brick_right', 'board_base', 'wall', 'tree',
                 'chair', 'prop_bed', 'slope_ramp', 'slope_top', 'ice', 'snow'):
        mocap(world, name, (0.0, -5.0, 0.0))
    for name, _half, _at in _steps():
        mocap(world, name, (0.0, -2.0, 0.0))
    lay_rug(world, (0.0, PARKED_M))


def _turn(heading):
    """The quaternion turning a box's z onto `heading`, radians from the world's z."""
    return (math.cos(heading / 2.0), 0.0, math.sin(heading / 2.0), 0.0)


def _on(at, heading, x, y, z):
    """World (x, y, z) of a point (x, z) on from `at` (world x, z) in `heading`'s frame."""
    c, s = math.cos(heading), math.sin(heading)
    return (at[0] + x * c + z * s, y, at[1] - x * s + z * c)


def mocap(world, name, at, heading=0.0):
    m = world.model
    k = m.body_mocapid[m.body(name).id]
    world.data.mocap_pos[k], world.data.mocap_quat[k] = at, _turn(heading)


def lay_rug(world, at, heading=0.0):
    """The rug laid still on the floor, its front edge at `at` (world x, z) across `heading`."""
    m, d = world.model, world.data
    joint = m.joint('rug').id
    adr, dof = m.jnt_qposadr[joint], m.jnt_dofadr[joint]
    d.qpos[adr:adr + 7] = _on(at, heading, 0.0, RUG_M / 2.0 + 0.001, RUG_LONG_M / 2.0) + _turn(
        heading)
    d.qvel[dof:dof + 6] = 0.0


def place(world, kind, z, x=0.0, heading=0.0):
    """The floor's event `kind` on the walk's line, `heading` radians from the world's z, (x, z)
    across and along it: a 'hole', a 'sill' or a 'slip' patch centred there, a 'rug' with its
    front edge there, 'stairs' their first riser, a 'board' centred there. The hole's gap
    crosses the world's z where her way does: the slab's halves turned off their compiled
    bounds would be missed."""
    c, s = math.cos(heading), math.sin(heading)
    at = (x * c + z * s, -x * s + z * c)
    if kind == 'hole':
        slab(world, at[1] - HOLE_LONG_M / 2.0, at[1] + HOLE_LONG_M / 2.0)
        world.hole_x = at[0]
        return
    if kind == 'sill':
        mocap(world, 'sill', (at[0], 0.0, at[1]), heading)
        return
    if kind == 'slip':
        mocap(world, 'slip', (at[0], 0.0, at[1]), heading)
        return
    if kind == 'rug':
        lay_rug(world, at, heading)
        return
    if kind == 'stairs':
        for name, _half, (dx, y, dz) in _steps():
            mocap(world, name, _on(at, heading, dx, y, dz), heading)
        return
    if kind == 'board':
        gap = max(BOARD[0], BOARD[2]) + 0.02
        slab(world, at[1] - gap, at[1] + gap)
        world.hole_x = at[0]
        mocap(world, 'board_base', (at[0], -HOLE_M, at[1]), heading)
        return
    if kind == 'wall':
        mocap(world, 'wall', (at[0], 1.2, at[1]), heading)
        return
    if kind == 'tree':
        mocap(world, 'tree', (at[0], 1.5, at[1]), heading)
        return
    if kind == 'chair':
        mocap(world, 'chair', (at[0], 0.0, at[1]), heading)
        return
    if kind in ('bed', 'prop_bed'):
        mocap(world, 'prop_bed', (at[0], 0.0, at[1]), heading)
        return
    if kind == 'slope':
        rad = math.radians(SLOPE_ANGLE_DEG)
        cos_r, sin_r = math.cos(rad / 2.0), math.sin(rad / 2.0)
        ch, sh = math.cos(heading / 2.0), math.sin(heading / 2.0)
        q = (ch * cos_r, ch * sin_r, sh * sin_r, sh * cos_r)
        hz = SLOPE_HALF[2] * math.cos(rad)
        hy = SLOPE_HALF[2] * math.sin(rad)
        ramp_at = _on(at, heading, 0.0, hy, hz)
        m = world.model
        k_ramp = m.body_mocapid[m.body('slope_ramp').id]
        world.data.mocap_pos[k_ramp], world.data.mocap_quat[k_ramp] = ramp_at, q
        top_at = _on(at, heading, 0.0, 2.0 * hy - SLOPE_TOP_HALF[1], 2.0 * hz + SLOPE_TOP_HALF[2])
        mocap(world, 'slope_top', top_at, heading)
        return
    if kind == 'ice':
        mocap(world, 'ice', (at[0], ICE_HALF[1], at[1] + ICE_HALF[2]), heading)
        return
    if kind == 'snow':
        mocap(world, 'snow', (at[0], SNOW_HALF[1], at[1] + SNOW_HALF[2]), heading)
        return
    raise MachineError('no floor event %r: hole, sill, slip, rug, stairs, board, wall, tree, chair, bed, slope, ice or snow' % kind)


def scenario(world, name, z=1.2, x=0.0):
    """Place a complete scenario on the floor: 'flat', 'chair', 'bed', 'slope', 'stairs',
    'ice', 'snow', 'obstacle', 'sill', 'hole', or 'rug'."""
    park(world)
    if name in ('flat', 'clear', 'none'):
        return
    cases = {
        'glass_room': lambda: (
            place(world, 'tree', z=6.0, x=0.88),
            place(world, 'sill', z=4.5, x=0.75),
            place(world, 'rug', z=1.5, x=0.0),
            place(world, 'stairs', z=2.0, x=1.8),
            place(world, 'slope', z=5.5, x=2.5),
            place(world, 'ice', z=5.5, x=-2.0),
            place(world, 'chair', z=0.8, x=1.5),
            place(world, 'bed', z=1.8, x=-1.8),
        ),
        'chair': lambda: place(world, 'chair', z=z, x=x),
        'chair': lambda: place(world, 'chair', z=0.88, x=x),
        'bed': lambda: place(world, 'bed', z=1.8, x=x),
        'slope': lambda: place(world, 'slope', z=1.0, x=x),
        'stairs': lambda: place(world, 'stairs', z=1.0, x=x),
        'ice': lambda: place(world, 'ice', z=0.8, x=x),
        'snow': lambda: place(world, 'snow', z=0.8, x=x),
        'obstacle': lambda: (place(world, 'wall', z=2.5, x=x), place(world, 'tree', z=1.8, x=x + 0.35)),
        'obstacles': lambda: (place(world, 'wall', z=2.5, x=x), place(world, 'tree', z=1.8, x=x + 0.35)),
        'urban': lambda: (place(world, 'wall', z=2.5, x=x), place(world, 'tree', z=1.8, x=x + 0.35)),
        'sill': lambda: place(world, 'sill', z=z, x=x),
        'hole': lambda: place(world, 'hole', z=z, x=x),
        'rug': lambda: place(world, 'rug', z=z, x=x),
    }
    if name in cases:
        cases[name]()
        return
    raise MachineError('unknown scenario %r: one of %s' % (name, ', '.join(SCENARIOS)))


def bricks(world, soles, heading=0.0):
    """Two bricks under her soles, their centres at `soles` [(x, z)] world, along `heading`."""
    for name, at in zip(('brick_left', 'brick_right'), soles):
        mocap(world, name, (at[0], BRICK[1], at[1]), heading)


def take(world, name):
    """The brick `name` ('brick_left', 'brick_right') taken away: parked under the plane."""
    mocap(world, name, (0.0, -1.0, 0.0))


def board(world, k, c):
    """The board's hinge sprung `k` N m/rad and damped `c` N m s/rad."""
    m, j = world.model, world.model.joint('board').id
    m.jnt_stiffness[j], m.dof_damping[m.jnt_dofadr[j]] = k, c


def props(world):
    """What lies on the floor: [(kind, centre, half sizes, turn 3x3)] world, a box each - the
    hole's gap, a sill, a slip patch, a brick, the board, a stair's steps, the rug, a wall, a tree,
    a chair, a bed, a slope, an ice sheet, a snow patch - what of it stands above the floor."""
    m, d = world.model, world.data
    out = []
    a = m.geom('slab_a').id
    gap = (m.geom_pos[a][2] + m.geom_size[a][2], m.geom_pos[m.geom('slab_b').id][2]
           - m.geom_size[m.geom('slab_b').id][2])
    if gap[1] - gap[0] > 1e-6:
        out.append(('hole', (getattr(world, 'hole_x', 0.0), -HOLE_M / 2.0, sum(gap) / 2.0),
                    (0.5, HOLE_M / 2.0, (gap[1] - gap[0]) / 2.0), ((1, 0, 0), (0, 1, 0), (0, 0, 1))))
    for name in ('sill', 'slip', 'brick_left', 'brick_right', 'board', 'wall', 'tree',
                 'ice', 'snow') + tuple(n for n, _h, _a in _steps()):
        body = m.body(name).id
        at = d.xpos[body]
        if at[1] > -0.5:
            g = m.body_geomadr[body]
            c, half = d.geom_xpos[g], m.geom_size[g]
            top, low = c[1] + half[1], c[1] - half[1] if name in ('board', 'wall', 'tree') else max(
                0.0, c[1] - half[1])
            out.append(('stairs' if name.startswith('step') else name.split('_')[0],
                        (c[0], (top + low) / 2.0, c[2]), (half[0], (top - low) / 2.0, half[2]),
                        tuple(map(tuple, d.xmat[body].reshape(3, 3)))))
    rug = m.body('rug').id
    if d.xpos[rug][2] > PARKED_M + 1.0:
        out.append(('rug', tuple(d.xpos[rug]), (0.3, RUG_M / 2.0, RUG_LONG_M / 2.0),
                    tuple(map(tuple, d.xmat[rug].reshape(3, 3)))))
    chair_id = m.body('chair').id
    if d.xpos[chair_id][1] > -0.5:
        c_mat = tuple(map(tuple, d.xmat[chair_id].reshape(3, 3)))
        g_seat, g_back = m.geom('chair_seat').id, m.geom('chair_back').id
        out.append(('chair', tuple(d.geom_xpos[g_seat]), tuple(m.geom_size[g_seat]), c_mat))
        out.append(('chair', tuple(d.geom_xpos[g_back]), tuple(m.geom_size[g_back]), c_mat))
    bed_id = m.body('prop_bed').id
    if d.xpos[bed_id][1] > -0.5:
        b_mat = tuple(map(tuple, d.xmat[bed_id].reshape(3, 3)))
        g_mat, g_hb = m.geom('bed_mattress').id, m.geom('bed_headboard').id
        out.append(('bed', tuple(d.geom_xpos[g_mat]), tuple(m.geom_size[g_mat]), b_mat))
        out.append(('bed', tuple(d.geom_xpos[g_hb]), tuple(m.geom_size[g_hb]), b_mat))
    ramp_id = m.body('slope_ramp').id
    if d.xpos[ramp_id][1] > -0.5:
        r_mat = tuple(map(tuple, d.xmat[ramp_id].reshape(3, 3)))
        g_ramp = m.geom('slope_ramp').id
        out.append(('slope', tuple(d.geom_xpos[g_ramp]), tuple(m.geom_size[g_ramp]), r_mat))
        top_id = m.body('slope_top').id
        t_mat = tuple(map(tuple, d.xmat[top_id].reshape(3, 3)))
        g_top = m.geom('slope_top').id
        out.append(('slope', tuple(d.geom_xpos[g_top]), tuple(m.geom_size[g_top]), t_mat))
    return out
