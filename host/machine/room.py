"""Her glazed mountain lodge: timber walls, a glass gable, a snowy roof, a deck, steps, a loft stair.

    room.furnish(contacts)                 # the lodge's bodies for the figure's MJCF (`physics.ROOM`)
    room.props(world)                      # its boxes for the page, the door where it swings, the lamp lit
    room.touched(world, dt)                # the lamp's switch under her hand: toggled, debounced

Y up, z her walk's line from the squat, x to her left. The walls are static boxes with the floor's
contacts: the sides and the near end dark timber, the far end a gable of glass between mullions
(`coaxial.graphics.props.PROP_INK`) with the door in it, nothing breaks. The roof's two slabs
meet at a ridge over the walls, snow on them, nothing collides with them. Past the gable a deck,
its boards flush with the floor, a glass rail round it with a gap on the door's line where the
wooden stair goes down the slope (`floor.scenario`, 'lodge'); at its left end a stone terrace one
step up. Inside, along the right wall, a stair up to a loft. The door hangs on a hinge at its post
and swings out as she pushes it; the lamp's switch sits on the wall by it.
"""
import math

from machine.floor import CHAIR_LEG_IN, CHAIR_LOW_H, CHAIR_SEAT, RISE_M, RUN_M, STEPS

#: The room, m: x half width, z from .. to, the walls' height at the eaves and thickness.
WIDE, FROM, TO, HIGH, THICK = 3.0, -1.5, 3.5, 2.4, 0.05

#: The doorway in the gable (+z): x from .. to; the door's half sizes, kg, its hinge's damping,
#: N m s/rad, and how far it swings, deg.
DOOR_X, DOOR, DOOR_KG, DOOR_C, DOOR_DEG = (0.3, 1.2), (0.44, 1.0, 0.02), 2.5, 0.8, 110.0

#: The roof: its pitch, deg; the eaves past the walls, m; a slab's half thickness, the snow on it.
PITCH_DEG, EAVE_M, ROOF_T = 28.0, 0.4, 0.08
#: The gable's mullions, each's x, the door's posts among them; a mullion's half width, a pane's
#: half thickness.
MULLIONS, MULLION, PANE = (-WIDE, -1.9, -0.8, DOOR_X[0], DOOR_X[1], 2.1, WIDE), 0.05, 0.01

#: The deck past the gable, to z; its boards' half thickness. The wooden stair down from it on the
#: door's line, x. Its rail's posts' x along its far edge - the gap between the third and the
#: fourth the stair's -, a post's half width and height, the top rail's height, the glass between
#: posts half high.
DECK_TO, DECK_T = TO + 2.0, 0.03
STAIR_X = (DOOR_X[0] + DOOR_X[1]) / 2.0
POSTS, POST, POST_H2, RAIL_H, RAIL_GLASS = (-WIDE, -1.5, STAIR_X - 0.5, STAIR_X + 0.5, WIDE), 0.03, 0.5, 1.0, 0.4
#: The stone terrace at the deck's left end and the step up to it: x from .. to, their heights.
TERRACE_X, STEP_X, TERRACE_H, STEP_H = (-WIDE, -2.0), (-2.0, -1.7), 2.0 * RISE_M, RISE_M
#: The loft stair along the right wall: its middle x, the z it starts at going -z, its steps hers
#: (`floor.STEPS`, RISE_M, RUN_M); the loft's half sizes at its top.
LOFT_X, LOFT_FROM, LOFT = 2.5, 3.0, (0.5, STEPS * RISE_M / 2.0, 0.75)
#: The lodge's bodies she may stand on: her ground's (`floor.grounds`).
GROUND = ('deck', 'stone_step', 'stone_terrace', 'loft') + tuple('stair%d' % k for k in range(STEPS))

#: The chair: its middle (x, z), facing -z; the seat's half sizes and height, the back's. The low chair
#: against the right wall straight to the right of where she starts, facing the room (-x), its seat at her
#: popliteal height (`floor.CHAIR_LOW_H`): in the room's middle (0.0, 0.8) it stood in the door's way and
#: 10 errands failed; against the back wall she walked into it at 0.9 m/s.
CHAIR_AT, SEAT, SEAT_H, BACK = (1.5, 0.8), CHAIR_SEAT, 0.45, (CHAIR_SEAT[0], 0.25, 0.02)
CHAIR_LOW_AT = (WIDE - THICK - 2.0 * BACK[2] - CHAIR_SEAT[2], 0.0)
#: The bed: its middle, its half sizes (the mattress's top at twice the second).
BED_AT, BED = (-1.8, 1.8), (0.45, 0.22, 1.1)
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


def roof_y(x):
    """The roof's underside over the walls at `x`, m: the ridge over the middle, the eaves at HIGH."""
    return HIGH + (WIDE - abs(x)) * math.tan(math.radians(PITCH_DEG))


def _gable(contacts):
    """The far wall glazed: a mullion up to the roof at each of MULLIONS, a pane between each two
    up to the eaves but the door's, the lintel over the door."""
    out = []
    for k, x in enumerate(MULLIONS):
        out.append(_box('mullion_%d' % k, (x, roof_y(x) / 2.0, TO), (MULLION, roof_y(x) / 2.0, THICK), contacts))
    for k, (a, b) in enumerate(zip(MULLIONS, MULLIONS[1:])):
        if (a, b) == DOOR_X:
            continue
        out.append(_box('pane_%d' % k, ((a + b) / 2.0, HIGH / 2.0, TO), ((b - a) / 2.0, HIGH / 2.0, PANE), contacts))
    out.append(_box('lintel', ((DOOR_X[0] + DOOR_X[1]) / 2.0, (HIGH + 2.0 * DOOR[1]) / 2.0, TO),
                    ((DOOR_X[1] - DOOR_X[0]) / 2.0, (HIGH - 2.0 * DOOR[1]) / 2.0, THICK), contacts))
    return out


def _roof():
    """The two slabs, each from its eave up to the ridge, turned about z by the pitch; nothing
    collides with them."""
    p = math.radians(PITCH_DEG)
    reach, half = WIDE + EAVE_M, ((WIDE + EAVE_M) / math.cos(math.radians(PITCH_DEG)) / 2.0, ROOF_T,
                                  (TO - FROM) / 2.0 + EAVE_M)
    y_mid = HIGH + WIDE * math.tan(p) - reach / 2.0 * math.tan(p)
    out = []
    for name, side in (('roof_left', -1.0), ('roof_right', 1.0)):
        at = (side * reach / 2.0 + side * ROOF_T * math.sin(p), y_mid + ROOF_T * math.cos(p), (FROM + TO) / 2.0)
        out.append(_box(name, at, half, '', ' quat="%g 0 0 %g"' % (math.cos(p / 2.0), -side * math.sin(p / 2.0))))
    return out


def _deck(contacts):
    """The deck's boards, its rail's posts, top rails and glass along its far edge and its sides,
    the stair's gap left open; the stone terrace at its left end and the step up to it."""
    z_mid, z_half = (TO + THICK + DECK_TO) / 2.0, (DECK_TO - TO - THICK) / 2.0
    out = [_box('deck', (0.0, -DECK_T, z_mid), (WIDE, DECK_T, z_half), contacts)]
    spans = [((a, DECK_TO), (b, DECK_TO)) for a, b in zip(POSTS, POSTS[1:]) if (a, b) != (POSTS[2], POSTS[3])]
    spans += [((x, TO + THICK), (x, DECK_TO)) for x in (-WIDE, WIDE)]
    for k, x in enumerate(POSTS):
        out.append(_box('rail_post_%d' % k, (x, POST_H2, DECK_TO), (POST, POST_H2, POST), contacts))
    for k, ((xa, za), (xb, zb)) in enumerate(spans):
        at, half = ((xa + xb) / 2.0, (za + zb) / 2.0), (abs(xb - xa) / 2.0 + POST, abs(zb - za) / 2.0 + POST)
        out.append(_box('rail_top_%d' % k, (at[0], RAIL_H, at[1]), (max(half[0], POST), POST, max(half[1], POST)),
                        contacts))
        out.append(_box('pane_rail_%d' % k, (at[0], RAIL_GLASS / 2.0 + RAIL_H - RAIL_GLASS - 0.1, at[1]),
                        (max(half[0] - POST, PANE), RAIL_GLASS / 2.0, max(half[1] - POST, PANE)), contacts))
    out.append(_box('stone_step', (sum(STEP_X) / 2.0, STEP_H / 2.0, z_mid),
                    ((STEP_X[1] - STEP_X[0]) / 2.0, STEP_H / 2.0, z_half), contacts))
    out.append(_box('stone_terrace', (sum(TERRACE_X) / 2.0, TERRACE_H / 2.0, z_mid),
                    ((TERRACE_X[1] - TERRACE_X[0]) / 2.0, TERRACE_H / 2.0, z_half), contacts))
    return out


def _loft(contacts):
    """The stair up along the right wall, a step hers each, and the loft at its top."""
    out = [_box('stair%d' % k, (LOFT_X, (k + 1) * RISE_M / 2.0, LOFT_FROM - (k + 0.5) * RUN_M),
                (LOFT[0], (k + 1) * RISE_M / 2.0, RUN_M / 2.0), contacts) for k in range(STEPS)]
    out.append(_box('loft', (LOFT_X, LOFT[1], LOFT_FROM - STEPS * RUN_M - LOFT[2]), LOFT, contacts))
    return out


def furnish(contacts, textured=False):
    """The lodge's bodies, after the floor: the timber walls, the glass gable with the door on its
    hinge, the roof, the deck with its rail and the stone terrace, the loft stair, the furniture,
    the lamp and its switch; `contacts` the floor's contact attributes; `textured`, each in its
    kind's material for her eyes (`sight.LOOKS`)."""
    mid = (FROM + TO) / 2.0
    out = [
        _box('timber_left', (-WIDE, HIGH / 2.0, mid), (THICK, HIGH / 2.0, (TO - FROM) / 2.0), contacts),
        _box('timber_right', (WIDE, HIGH / 2.0, mid), (THICK, HIGH / 2.0, (TO - FROM) / 2.0), contacts),
        _box('timber_near', (0.0, HIGH / 2.0, FROM), (WIDE, HIGH / 2.0, THICK), contacts),
    ] + _gable(contacts) + _roof() + _deck(contacts) + _loft(contacts) + [
        '<body name="door_post" pos="%g 0 %g"><body name="door" pos="0 %g 0">'
        '<joint name="door" type="hinge" axis="0 -1 0" limited="true" range="0 %g" damping="%g"/>'
        '<geom name="door" type="box" size="%g %g %g" pos="%g 0 0" mass="%g"%s/></body></body>' % (
            DOOR_X[0], TO, DOOR[1], DOOR_DEG, DOOR_C, DOOR[0], DOOR[1], DOOR[2],
            DOOR[0], DOOR_KG, contacts),
        _box('seat', (CHAIR_AT[0], SEAT_H, CHAIR_AT[1]), SEAT, contacts),
        _box('back', (CHAIR_AT[0], SEAT_H + BACK[1] + 0.02, CHAIR_AT[1] + SEAT[2]), BACK, contacts),
        _box('seat_low', (CHAIR_LOW_AT[0], CHAIR_LOW_H - SEAT[1], CHAIR_LOW_AT[1]), (SEAT[2], SEAT[1], SEAT[0]), contacts),
        _box('back_low', (CHAIR_LOW_AT[0] + SEAT[2], CHAIR_LOW_H + BACK[1], CHAIR_LOW_AT[1]), (BACK[2], BACK[1], BACK[0]), contacts),
        _box('bed', (BED_AT[0], BED[1], BED_AT[1]), (BED[0], BED[1] - 0.05, BED[2]), contacts,
             ' pos="0 0.05 0" priority="2" condim="4" solref="0.08 1" solimp="0.8 0.95 0.10"'),
        _box('table', (TABLE_AT[0], TOP_H, TABLE_AT[1]), TOP, contacts),
        _box('post', (LAMP_AT[0], POST_H, LAMP_AT[1]), (0.02, POST_H, 0.02), contacts),
        _box('shade', (LAMP_AT[0], 2.0 * POST_H + SHADE[1], LAMP_AT[1]), SHADE, contacts),
        _box('switch', (-WIDE + THICK + SWITCH[2], SWITCH_AT[0], SWITCH_AT[1]),
             (SWITCH[2], SWITCH[1], SWITCH[0]), contacts),
    ]
    for dx in (-1.0, 1.0):
        for dz in (-1.0, 1.0):
            out.append(_box('chair_leg_%d_%d' % (dx > 0, dz > 0),
                            (CHAIR_AT[0] + dx * (SEAT[0] - CHAIR_LEG_IN), SEAT_H / 2.0,
                             CHAIR_AT[1] + dz * (SEAT[2] - CHAIR_LEG_IN)),
                            (0.02, SEAT_H / 2.0, 0.02), contacts))
            out.append(_box('chair_low_leg_%d_%d' % (dx > 0, dz > 0),
                            (CHAIR_LOW_AT[0] + dx * (SEAT[2] - CHAIR_LEG_IN), CHAIR_LOW_H / 2.0,
                             CHAIR_LOW_AT[1] + dz * (SEAT[0] - CHAIR_LEG_IN)),
                            (0.02, CHAIR_LOW_H / 2.0, 0.02), contacts))
            out.append(_box('table_leg_%d_%d' % (dx > 0, dz > 0),
                            (TABLE_AT[0] + dx * 0.55, TOP_H / 2.0, TABLE_AT[1] + dz * 0.3),
                            (0.02, TOP_H / 2.0, 0.02), contacts))
    if textured:
        out = [s.replace('type="box"', 'type="box" material="%s"' % next(
            (k for head, k in KINDS.items() if s.split('name="')[1].startswith(head)), 'wood'))
               for s in out]
    return out


#: Each body's prop kind for the page, by its name's head.
KINDS = {'timber': 'timber', 'mullion': 'timber', 'pane': 'glass', 'lintel': 'glass', 'door': 'door',
         'roof': 'roof', 'deck': 'wood', 'rail': 'rail', 'stone': 'stone', 'stair': 'wood', 'loft': 'wood',
         'seat': 'wood', 'back': 'wood', 'chair': 'wood', 'bed': 'bed', 'table': 'wood', 'post': 'lamp',
         'shade': 'lamp', 'switch': 'switch'}


def props(world):
    """The lodge's boxes for the page: [(kind, centre, half sizes, turn 3x3)], the door where it
    swings, the lamp 'lit' while it is on; a still box laid once (`_laid_props`)."""
    d, lit = world.data, getattr(world, 'lamp', False)
    out = []
    for g, kind, box in _laid_props(world):
        box = box or _box_of(world.model, d, g)
        if box:
            out.append(('lit' if kind == 'lamp' and lit else kind,) + box)
    return out


def _laid_props(world):
    """(geom, kind, its box if it is still) of each lodge geom the page draws, in the model's order:
    a box welded to the world laid once off the model's own kinematics, the door's read each call."""
    laid = getattr(world, '_room_props', None)
    if laid is None:
        m, mj = world.model, world._mj
        still = mj.MjData(m)
        mj.mj_kinematics(m, still)
        laid = world._room_props = []
        for g in range(m.ngeom):
            kind, body = KINDS.get(m.geom(g).name.split('_')[0]), m.geom_bodyid[g]
            box = _box_of(m, still, g) if m.body_weldid[body] == 0 else None
            if kind is not None and m.body_mocapid[body] < 0 and (box or m.body_weldid[body] != 0):
                laid.append((g, kind, box))
    return laid


def _box_of(m, d, g):
    """(centre, half sizes, turn 3x3) of geom `g` posed on `d`, None below the page's floor."""
    if d.geom_xpos[g][1] < -0.5:
        return None
    return (tuple(d.geom_xpos[g].tolist()), tuple(m.geom_size[g].tolist()),
            tuple(map(tuple, d.geom_xmat[g].reshape(3, 3).tolist())))


def touched(world, dt):
    """The lamp toggled where a hand of hers touches its switch, not twice within SWITCH_S - the
    switch's geom against a hand's, MuJoCo's contacts paired in C (`wbc_pass_paired`); whether it
    is on."""
    m, d = world.model, world.data
    laid = getattr(world, '_room_switch', None)
    if laid is None:
        import numpy as np
        from tools.cores import wbc as cores
        hands = [m.body(side + part).id for side in ('left', 'right') for part in ('_hand', '_fingers')]
        masks = (np.arange(m.ngeom) == m.geom('switch').id, np.isin(m.geom_bodyid, hands))
        laid = world._room_switch = (masks, [cores.address(k) for k in masks], cores.library().wbh_pass_paired,
                                     cores.address)
    _masks, at, paired, address = laid
    since = getattr(world, 'switched', SWITCH_S) + dt
    if since >= SWITCH_S and d.ncon:
        geom, pos = d.contact.geom, d.contact.pos
        if paired(address(geom), address(pos), geom.strides[0], d.ncon, *at, math.inf):
            world.lamp, since = not getattr(world, 'lamp', False), 0.0
    world.switched = since
    return getattr(world, 'lamp', False)


def door_open(world):
    """How far the door stands open, deg."""
    m = world.model
    return math.degrees(float(world.data.qpos[m.jnt_qposadr[m.joint('door').id]]))
