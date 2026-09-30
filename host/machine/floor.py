"""The floor and what befalls her on it: a slab in two, a sill, a patch, a rug, a stair.

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
#: at 80 m a walk stepped off it 100 s in, 3 cm down, and fell or sank into a crouch
#: (2026-09-28). The halves are
#: compiled over the whole span and cut to size, the sill and the patch are mocap bodies: a
#: geom moved or grown past its compiled bounds is missed by the broadphase (the rug fell
#: through a slab grown 27 m, a box through a sill moved 1 m). At 0.15 the patch let the stance
#: foot creep 3 mm (the walk asks 0.17), at 0.06 it slid 10 cm back under the push-off; the rug
#: at 0.3 lay still under a landing and a push-off (the sole's shear 100 N, the rug's hold 165)
#: (2026-09-27). The skimming toes brushed a 4 cm sill, 350 N for 20 ms, unseen; 6 cm caught
#: them for 0.11 s and she fell, or stumbled over it lifted (`landing.tripped`) (2026-09-28).
HOLE_M, HOLE_LONG_M, SILL_M, SILL_LONG_M = 0.03, 0.40, 0.06, 0.04
SLIP_LONG_M, SLIP_FRICTION = 0.5, 0.06
RUG_LONG_M, RUG_M, RUG_KG, RUG_FRICTION = 0.9, 0.01, 1.5, 0.1
SLAB_FROM_M, SEAM_M, SLAB_TO_M, PARKED_M = -20.0, 30.0, 10000.0, -50.0

#: A stair of STEPS steps, each RISE_M up and RUN_M on - a stride's step, as she has no eyes to
#: fit her steps to it - and TOP_M of landing at the top: a box a step, mocap bodies too.
STEPS, RISE_M, RUN_M, TOP_M = 5, 0.08, 0.425, 1.5


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
            HOLE_M / 2.0, (SLAB_TO_M - SLAB_FROM_M) / 2.0, -HOLE_M / 2.0,
            (SLAB_FROM_M + SLAB_TO_M) / 2.0, contacts),
        '<geom name="slab_b" type="box" size="50 %g %g" pos="0 %g %g"%s/>' % (
            HOLE_M / 2.0, (SLAB_TO_M - SLAB_FROM_M) / 2.0, -HOLE_M / 2.0,
            (SLAB_FROM_M + SLAB_TO_M) / 2.0, contacts),
        '<body name="sill" mocap="true" pos="0 -1 0"><geom type="box" size="0.5 %g %g"%s/>'
        '</body>' % (SILL_M / 2.0, SILL_LONG_M / 2.0, contacts),
        '<body name="slip" mocap="true" pos="0 -1 0"><geom type="box" size="0.5 0.0005 %g" '
        'priority="1" condim="4" friction="%g %g 0.001"%s%s/></body>' % (
            SLIP_LONG_M / 2.0, SLIP_FRICTION, torsion, give, contacts)] + [
        '<body name="%s" mocap="true" pos="0 -2 0"><geom type="box" size="%g %g %g"%s/></body>'
        % ((name,) + half + (contacts,)) for name, half, _at in _steps()]


def rug(give, friction, torsion):
    """The rug's body, after her figure: a free box, its underside sliding on the floor."""
    return ['<body name="rug" pos="0 0.1 %g"><freejoint name="rug"/>' % PARKED_M,
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
    for name in ('sill', 'slip'):
        mocap(world, name, (0.0, -1.0, 0.0))
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
    front edge there, 'stairs' their first riser. The hole's gap crosses the world's z where
    her way does: the slab's halves turned off their compiled bounds would be missed."""
    c, s = math.cos(heading), math.sin(heading)
    at = (x * c + z * s, -x * s + z * c)
    if kind == 'hole':
        slab(world, at[1] - HOLE_LONG_M / 2.0, at[1] + HOLE_LONG_M / 2.0)
        world.hole_x = at[0]
    elif kind == 'sill':
        mocap(world, 'sill', (at[0], SILL_M / 2.0, at[1]), heading)
    elif kind == 'slip':
        mocap(world, 'slip', (at[0], 0.0005, at[1]), heading)
    elif kind == 'rug':
        lay_rug(world, at, heading)
    elif kind == 'stairs':
        for name, _half, (dx, y, dz) in _steps():
            mocap(world, name, _on(at, heading, dx, y, dz), heading)
    else:
        raise MachineError('no floor event %r: hole, sill, slip, rug or stairs' % kind)


def props(world):
    """What lies on the floor: [(kind, centre, half sizes, turn 3x3)] world, a box each - the
    hole's gap, a sill, a slip patch, a stair's steps, the rug."""
    m, d = world.model, world.data
    out = []
    a = m.geom('slab_a').id
    gap = (m.geom_pos[a][2] + m.geom_size[a][2], m.geom_pos[m.geom('slab_b').id][2]
           - m.geom_size[m.geom('slab_b').id][2])
    if gap[1] - gap[0] > 1e-6:
        out.append(('hole', (getattr(world, 'hole_x', 0.0), -HOLE_M / 2.0, sum(gap) / 2.0),
                    (0.5, HOLE_M / 2.0, (gap[1] - gap[0]) / 2.0), ((1, 0, 0), (0, 1, 0), (0, 0, 1))))
    for name in ('sill', 'slip') + tuple(n for n, _h, _a in _steps()):
        body = m.body(name).id
        at = d.xpos[body]
        if at[1] > -0.5:
            g = m.body_geomadr[body]
            out.append(('stairs' if name.startswith('step') else name, tuple(at),
                        tuple(m.geom_size[g]), tuple(map(tuple, d.xmat[body].reshape(3, 3)))))
    rug = m.body('rug').id
    if d.xpos[rug][2] > PARKED_M + 1.0:
        out.append(('rug', tuple(d.xpos[rug]), (0.3, RUG_M / 2.0, RUG_LONG_M / 2.0),
                    tuple(map(tuple, d.xmat[rug].reshape(3, 3)))))
    return out
