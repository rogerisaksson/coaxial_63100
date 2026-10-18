"""A living room round her: glass walls that do not break, furniture, a door, a lamp.

    room.furnish(contacts)                 # the room's bodies for the figure's MJCF (`physics.ROOM`)
    room.props(world)                      # its boxes for the page, the door where it swings, the lamp lit
    room.touched(world, dt)                # the lamp's switch under her hand: toggled, debounced

Y up, z her walk's line from the squat, x to her left. The walls are static boxes with the floor's
contacts and glass's look (`coaxial.graphics.gynoid.PROP_INK`): nothing breaks. The door hangs on a
hinge at its post and swings out as she pushes it; the lamp's switch sits on the wall by it.
"""
import math

#: The room, m: x half width, z from .. to, the walls' height and thickness.
WIDE, FROM, TO, HIGH, THICK = 3.0, -1.5, 3.5, 2.4, 0.05

#: The doorway in the far wall (+z): x from .. to; the door's half sizes, kg, its hinge's damping,
#: N m s/rad, and how far it swings, deg.
DOOR_X, DOOR, DOOR_KG, DOOR_C, DOOR_DEG = (0.3, 1.2), (0.44, 1.0, 0.02), 2.5, 0.8, 110.0

#: The chair: its middle (x, z), facing -z; the seat's half sizes and height, the back's.
CHAIR_AT, SEAT, SEAT_H, BACK = (1.5, 0.8), (0.22, 0.02, 0.22), 0.45, (0.22, 0.25, 0.02)
#: The bed: its middle, its half sizes (the mattress's top at twice the second).
BED_AT, BED = (-1.8, 1.8), (0.45, 0.22, 1.0)
#: The table: its middle, the top's half sizes and height.
TABLE_AT, TOP, TOP_H = (1.5, -0.8), (0.6, 0.015, 0.35), 0.74
#: The lamp: its foot (x, z), the post's half height, the shade's half sizes; its switch on the
#: left wall, (y, z), its half sizes.
LAMP_AT, POST_H, SHADE = (-2.5, -1.0), 0.75, (0.15, 0.12, 0.15)
SWITCH_AT, SWITCH = (1.05, -0.5), (0.03, 0.04, 0.02)
#: A touch of the switch toggles the lamp, and not again for this long, s.
SWITCH_S = 0.6


def _box(name, at, half, contacts, extra=''):
    return '<body name="%s" pos="%g %g %g"><geom name="%s" type="box" size="%g %g %g"%s%s/></body>' % (
        (name,) + tuple(at) + (name,) + tuple(half) + (contacts, extra))


def furnish(contacts, textured=False):
    """The room's bodies, after the floor: the walls, the furniture, the door on its hinge,
    the lamp and its switch; `contacts` the floor's contact attributes; `textured`, each in
    its kind's material for her eyes (`sight.LOOKS`)."""
    mid = (FROM + TO) / 2.0
    out = [
        _box('wall_left', (-WIDE, HIGH / 2.0, mid), (THICK, HIGH / 2.0, (TO - FROM) / 2.0), contacts),
        _box('wall_right', (WIDE, HIGH / 2.0, mid), (THICK, HIGH / 2.0, (TO - FROM) / 2.0), contacts),
        _box('wall_near', (0.0, HIGH / 2.0, FROM), (WIDE, HIGH / 2.0, THICK), contacts),
        _box('wall_far_left', ((-WIDE + DOOR_X[0]) / 2.0, HIGH / 2.0, TO),
             ((DOOR_X[0] + WIDE) / 2.0, HIGH / 2.0, THICK), contacts),
        _box('wall_far_right', ((DOOR_X[1] + WIDE) / 2.0, HIGH / 2.0, TO),
             ((WIDE - DOOR_X[1]) / 2.0, HIGH / 2.0, THICK), contacts),
        _box('lintel', ((DOOR_X[0] + DOOR_X[1]) / 2.0, (HIGH + 2.0 * DOOR[1]) / 2.0, TO),
             ((DOOR_X[1] - DOOR_X[0]) / 2.0, (HIGH - 2.0 * DOOR[1]) / 2.0, THICK), contacts),
        '<body name="door_post" pos="%g 0 %g"><body name="door" pos="0 %g 0">'
        '<joint name="door" type="hinge" axis="0 -1 0" limited="true" range="0 %g" damping="%g"/>'
        '<geom name="door" type="box" size="%g %g %g" pos="%g 0 0" mass="%g"%s/></body></body>' % (
            DOOR_X[0], TO, DOOR[1], DOOR_DEG, DOOR_C, DOOR[0], DOOR[1], DOOR[2],
            DOOR[0], DOOR_KG, contacts),
        _box('seat', (CHAIR_AT[0], SEAT_H, CHAIR_AT[1]), SEAT, contacts),
        _box('back', (CHAIR_AT[0], SEAT_H + BACK[1] + 0.02, CHAIR_AT[1] + SEAT[2]), BACK, contacts),
        _box('bed', (BED_AT[0], BED[1], BED_AT[1]), BED, contacts),
        _box('table', (TABLE_AT[0], TOP_H, TABLE_AT[1]), TOP, contacts),
        _box('post', (LAMP_AT[0], POST_H, LAMP_AT[1]), (0.02, POST_H, 0.02), contacts),
        _box('shade', (LAMP_AT[0], 2.0 * POST_H + SHADE[1], LAMP_AT[1]), SHADE, contacts),
        _box('switch', (-WIDE + THICK + SWITCH[2], SWITCH_AT[0], SWITCH_AT[1]),
             (SWITCH[2], SWITCH[1], SWITCH[0]), contacts),
    ]
    for dx in (-1.0, 1.0):
        for dz in (-1.0, 1.0):
            out.append(_box('chair_leg_%d_%d' % (dx > 0, dz > 0),
                            (CHAIR_AT[0] + dx * 0.19, SEAT_H / 2.0, CHAIR_AT[1] + dz * 0.19),
                            (0.02, SEAT_H / 2.0, 0.02), contacts))
            out.append(_box('table_leg_%d_%d' % (dx > 0, dz > 0),
                            (TABLE_AT[0] + dx * 0.55, TOP_H / 2.0, TABLE_AT[1] + dz * 0.3),
                            (0.02, TOP_H / 2.0, 0.02), contacts))
    if textured:
        out = [s.replace('type="box"', 'type="box" material="%s"' % next(
            (k for head, k in KINDS.items() if s.split('name="')[1].startswith(head)), 'wood'))
               for s in out]
    return out


#: Each body's prop kind for the page, by its name's head.
KINDS = {'wall': 'glass', 'lintel': 'glass', 'door': 'door', 'seat': 'wood', 'back': 'wood',
         'chair': 'wood', 'bed': 'bed', 'table': 'wood', 'post': 'lamp', 'shade': 'lamp',
         'switch': 'switch'}


def props(world):
    """The room's boxes for the page: [(kind, centre, half sizes, turn 3x3)], the door where it
    swings, the lamp 'lit' while it is on."""
    m, d = world.model, world.data
    out = []
    for g in range(m.ngeom):
        name = m.geom(g).name
        kind = KINDS.get(name.split('_')[0])
        if kind is None or d.geom_xpos[g][1] < -0.5 or m.body_mocapid[m.geom_bodyid[g]] >= 0:
            continue
        if kind == 'lamp' and getattr(world, 'lamp', False):
            kind = 'lit'
        out.append((kind, tuple(d.geom_xpos[g]), tuple(m.geom_size[g]),
                    tuple(map(tuple, d.geom_xmat[g].reshape(3, 3)))))
    return out


def touched(world, dt):
    """The lamp toggled where a hand of hers touches its switch, not twice within SWITCH_S;
    whether it is on."""
    m, d = world.model, world.data
    ids = getattr(world, '_room_ids', None)
    if ids is None:
        ids = world._room_ids = (
            m.geom('switch').id,
            frozenset(m.body(side + part).id for side in ('left', 'right') for part in ('_hand', '_fingers')))
    switch, hands = ids
    since = getattr(world, 'switched', SWITCH_S) + dt
    if since >= SWITCH_S and d.ncon:
        g1, g2 = d.contact.geom1[:d.ncon], d.contact.geom2[:d.ncon]
        if (g1 == switch).any() or (g2 == switch).any():
            for i in range(d.ncon):
                c = d.contact[i]
                pair = (c.geom1, c.geom2)
                if switch in pair and any(m.geom_bodyid[g] in hands for g in pair):
                    world.lamp, since = not getattr(world, 'lamp', False), 0.0
                    break
    world.switched = since
    return getattr(world, 'lamp', False)


def door_open(world):
    """How far the door stands open, deg."""
    m = world.model
    return math.degrees(float(world.data.qpos[m.jnt_qposadr[m.joint('door').id]]))
