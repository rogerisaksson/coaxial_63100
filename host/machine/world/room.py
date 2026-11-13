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
import re

from machine.world.chair import CHAIR_LEG_IN, CHAIR_SEAT, SEAT_GRIP
from machine.world.floor import CHAIR_LOW_H, FOAM, RISE_M, RUN_M, STEPS, core
from machine.world import loose

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
#: against the wall straight to her right as she starts facing the door (-x; +x is her left), facing the room, its seat at her
#: popliteal height (`floor.CHAIR_LOW_H`): in the room's middle (0.0, 0.8) it stood in the door's way and
#: 10 errands failed; against the back wall she walked into it at 0.9 m/s.
CHAIR_AT, SEAT, SEAT_H, BACK = (1.5, 0.8), CHAIR_SEAT, 0.45, (CHAIR_SEAT[0], 0.25, 0.02)
CHAIR_LOW_AT = (-(WIDE - THICK - 2.0 * BACK[2] - CHAIR_SEAT[2]), 0.0)
#: The bed: its middle, its half sizes (the mattress's top at twice the second), its frame's height under the
#: mattress and how far it stands out of it, m. Its head 0.55 m off the gable fits no dock at that end; 0.20 m
#: further out it fitted none either (from the east she still rounded the foot's corner and fell at 11.1 s) and the
#: south spot's dock came 0.219 rad askew for 0.137, so it stays. The frame fills the slot under the mattress, no
#: slot left (`floor`'s bed): through it her feet came down off its end under the mattress, wedged at 9.7 of her
#: weight, and sat on its edge her soles bore nothing and her thrust waited. The mattress as it was: cut to 0.16 m
#: up over a frame that high, her lie at its end from F's walk, her hips 0.06 m in, slid off it.
BED_AT, BED, BED_FRAME_H, BED_PROUD = (-1.8, 1.8), (0.585, 0.22, 1.1), 0.10, 0.02
#: The table: its middle, the top's half sizes and height.
TABLE_AT, TOP, TOP_H = (1.5, -0.8), (0.6, 0.015, 0.35), 0.74
#: The stool: round, 340 mm across (the text's stool, `enforce-backless-stool-rim-zcbf`), no back, at the table's short
#: side to the room's middle, its rim STOOL_GAP off the table's edge; its seat the chairs' at her popliteal height
#: (`floor.CHAIR_LOW_H`), sat on as the low chair is, no mount on her arms; four round legs the chairs' in from its rim.
STOOL_R, STOOL_GAP, STOOL_LEG = 0.17, 0.18, 0.015
STOOL_AT = (TABLE_AT[0] - TOP[0] - STOOL_GAP - STOOL_R, TABLE_AT[1])
#: The lamp: its foot (x, z), the post's half height, the shade's half sizes; its switch on the
#: left wall, (y, z), its half sizes.
LAMP_AT, POST_H, SHADE = (-2.5, -1.0), 0.75, (0.15, 0.12, 0.15)
SWITCH_AT, SWITCH = (1.05, -0.5), (0.03, 0.04, 0.02)
#: A touch of the switch toggles the lamp, and not again for this long, s.
SWITCH_S = 0.6
#: Her furniture's weights, kg, loose on the floor (`loose`): a wooden chair's 5 (seat, back, a leg), a stool's 3
#: (seat, a leg), a table's 25 (top, a leg), a bed's 60 (frame, core, mattress), a floor lamp's 5.5 (its weighted
#: foot, post, shade); the lamp's foot, half across and half high, m - on its post alone it fell at 3.6 deg.
CHAIR_KG, STOOL_KG, TABLE_KG, BED_KG, LAMP_KG = (2.0, 1.0, 0.5), (1.4, 0.4), (17.0, 2.0), (30.0, 10.0, 20.0), (4.0, 1.0, 0.5)
LAMP_FOOT = (0.14, 0.01)
#: A glide under each leg, the bed frame's corners and the lamp foot's rim, its radius, m: the floor met at a point
#: (one contact; a leg's box on the floor's met at up to four - 96 contacts where 20, MuJoCo's step 172 us where 79).
FOOT_R = 0.01


def _box(name, at, half, contacts, extra=''):
    return '<body name="%s" pos="%g %g %g"><geom name="%s" type="box" size="%g %g %g"%s%s/></body>' % (
        (name,) + tuple(at) + (name,) + tuple(half) + (contacts, extra))


def _round(name, at, radius, half, contacts, extra=''):
    """An upright cylinder's body: its axis along y, `half` its half height."""
    return '<body name="%s" pos="%g %g %g"><geom name="%s" type="cylinder" size="%g %g" zaxis="0 1 0"%s%s/></body>' % (
        (name,) + tuple(at) + (name, radius, half, contacts, extra))


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


def furnish(contacts, textured=False, free=True):
    """The lodge's bodies, after the floor: the timber walls, the glass gable with the door on its
    hinge, the roof, the deck with its rail and the stone terrace, the loft stair, the furniture,
    the lamp and its switch; `contacts` the floor's contact attributes; `textured`, each in its
    kind's material for her eyes (`sight.LOOKS`); its loose things `free`, else welded where they stand."""
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
        _box('switch', (-WIDE + THICK + SWITCH[2], SWITCH_AT[0], SWITCH_AT[1]),
             (SWITCH[2], SWITCH[1], SWITCH[0]), contacts),
    ]
    if textured:
        out = [re.sub(r'type="(box|cylinder)"', r'type="\1" material="%s"' % next(
            (k for head, k in KINDS.items() if s.split('name="')[1].startswith(head)), 'wood'), s)
               for s in out]
    return out + _loose(textured, free)


def _geom(name, half, at, kg, grip='', textured=False, shape='box'):
    """A loose thing's geom (`loose.MEETS`), in its body: its kind's material for her eyes where `textured`."""
    look = ' material="%s"' % next((k for head, k in KINDS.items() if name.startswith(head)), 'wood') if textured else ''
    return '<geom name="%s" type="%s" size="%s" pos="%g %g %g" mass="%g"%s%s%s/>' % (
        (name, shape, ' '.join('%g' % h for h in half)) + tuple(at) + (kg, look, loose.MEETS, grip))


def _loose(textured, free=True):
    """The room's loose things (`loose`), each a free body on the floor its feet's grip alone holds, where they
    stood welded to her world: the chair and the low chair, the stool, the table, the bed, the lamp."""
    def legs(name, high, across, along, kg):
        return [g for dx in (-1.0, 1.0) for dz in (-1.0, 1.0) for g in (
            _geom('%s_%d_%d' % (name, dx > 0, dz > 0), (0.02, (high - FOOT_R) / 2.0, 0.02),
                  (dx * across, (high + FOOT_R) / 2.0, dz * along), kg, '', textured),
            _glide('%s_%d_%d' % (name, dx > 0, dz > 0), dx * across, FOOT_R, dz * along, 'wood'))]
    (seat, back, leg), (top, t_leg), (frame, core_kg, mattress), (foot, post, shade) = CHAIR_KG, TABLE_KG, BED_KG, LAMP_KG
    s_leg, reach = (CHAIR_LOW_H - 2.0 * CHAIR_SEAT[1]) / 2.0, (STOOL_R - CHAIR_LEG_IN) / math.sqrt(2.0)
    up, bed_h, lamp_top = ' zaxis="0 1 0"', BED[1] - BED_FRAME_H / 2.0, FOOT_R + 2.0 * LAMP_FOOT[1]
    corners = lambda name, x, y, z, what: [_glide('%s_%d' % (name, k), dx * x, y, dz * z, what)
                                           for k, (dx, dz) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1)))]
    stool_legs = [_geom('stool_leg_%d' % k, (STOOL_LEG, s_leg - FOOT_R / 2.0), (dx * reach, s_leg + FOOT_R / 2.0, dz * reach),
                        STOOL_KG[1], up, textured, 'cylinder') for k, (dx, dz) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1)))]
    return [
        loose.body('lodge_chair', free, (CHAIR_AT[0], 0.0, CHAIR_AT[1]), [
            _geom('seat', SEAT, (0.0, SEAT_H, 0.0), seat, SEAT_GRIP, textured),
            _geom('back', BACK, (0.0, SEAT_H + BACK[1] + 0.02, SEAT[2]), back, '', textured)]
            + legs('chair_leg', SEAT_H, SEAT[0] - CHAIR_LEG_IN, SEAT[2] - CHAIR_LEG_IN, leg)),
        loose.body('lodge_chair_low', free, (CHAIR_LOW_AT[0], 0.0, CHAIR_LOW_AT[1]), [
            _geom('seat_low', (SEAT[2], SEAT[1], SEAT[0]), (0.0, CHAIR_LOW_H - SEAT[1], 0.0), seat, SEAT_GRIP, textured),
            _geom('back_low', (BACK[2], BACK[1], BACK[0]), (-SEAT[2], CHAIR_LOW_H + BACK[1], 0.0), back, '', textured)]
            + legs('chair_low_leg', CHAIR_LOW_H, SEAT[2] - CHAIR_LEG_IN, SEAT[0] - CHAIR_LEG_IN, leg)),
        loose.body('stool', free, (STOOL_AT[0], 0.0, STOOL_AT[1]), [
            _geom('stool_seat', (STOOL_R, CHAIR_SEAT[1]), (0.0, CHAIR_LOW_H - CHAIR_SEAT[1], 0.0), STOOL_KG[0],
                  up + SEAT_GRIP, textured, 'cylinder')] + stool_legs
            + corners('stool', reach, FOOT_R, reach, 'wood')),
        loose.body('table', free, (TABLE_AT[0], 0.0, TABLE_AT[1]), [_geom('table', TOP, (0.0, TOP_H, 0.0), top, '', textured)]
                   + legs('table_leg', TOP_H, 0.55, 0.3, t_leg)),
        loose.body('bed', free, (BED_AT[0], BED[1], BED_AT[1]), [
            '<geom name="bed" type="box" size="%g %g %g" pos="0 %g 0" mass="%g"%s%s%s/>' % (
                BED[0], bed_h, BED[2], BED_FRAME_H / 2.0, mattress, ' material="bed"' if textured else '', FOAM, loose.MEETS),
            core('bedcore', (BED[0], bed_h, BED[2]), BED_FRAME_H / 2.0,
                 loose.MEETS + ' mass="%g"' % core_kg + (' material="bed"' if textured else '')),
            _geom('bedframe', (BED[0] + BED_PROUD, (BED_FRAME_H - FOOT_R) / 2.0, BED[2] + BED_PROUD),
                  (0.0, (BED_FRAME_H + FOOT_R) / 2.0 - BED[1], 0.0), frame, '', textured)]
            + corners('bed', BED[0] + BED_PROUD - FOOT_R, FOOT_R - BED[1], BED[2] + BED_PROUD - FOOT_R, 'wood')),
        loose.body('lamp', free, (LAMP_AT[0], 0.0, LAMP_AT[1]), [
            _geom('post_foot', (LAMP_FOOT[0], LAMP_FOOT[1], LAMP_FOOT[0]), (0.0, FOOT_R + LAMP_FOOT[1], 0.0), foot, '', textured),
            _geom('post', (0.02, POST_H - lamp_top / 2.0, 0.02), (0.0, POST_H + lamp_top / 2.0, 0.0), post, '', textured),
            _geom('shade', SHADE, (0.0, 2.0 * POST_H + SHADE[1], 0.0), shade, '', textured)]
            + corners('lamp', LAMP_FOOT[0] - FOOT_R, FOOT_R, LAMP_FOOT[0] - FOOT_R, 'felt'))]


def _glide(name, x, y, z, what):
    """A glide (`FOOT_R`) at (`x`, `y`, `z`) in its loose thing, gripping as `what` grips (`loose.foot`): its one
    contact with the floor - no prop of the page's (its name no kind's)."""
    return '<geom name="glide_%s" type="sphere" size="%g" pos="%g %g %g" mass="0.01"%s%s/>' % (
        name, FOOT_R, x, y, z, loose.MEETS, loose.foot(what))


#: Each body's prop kind for the page, by its name's head, a number off it (the loft's `stair0`).
KINDS = {'timber': 'timber', 'mullion': 'timber', 'pane': 'glass', 'lintel': 'glass', 'door': 'door',
         'roof': 'roof', 'deck': 'wood', 'rail': 'rail', 'stone': 'stone', 'stair': 'wood', 'loft': 'wood',
         'seat': 'wood', 'back': 'wood', 'chair': 'wood', 'bed': 'bed', 'table': 'wood', 'post': 'lamp',
         'shade': 'lamp', 'switch': 'switch', 'stool': 'wood'}


def props(world):
    """The lodge's boxes for the page: [(kind, centre, half sizes, turn 3x3)], the door where it
    swings, the lamp 'lit' while it is on; a still box told by the same tuple each call (`_told_props`),
    the door's read anew - each built anew, 35.8 us a call."""
    m, d, lit, told = world.model, world.data, bool(getattr(world, 'lamp', False)), _told_props(world)
    # what moves - the door, her loose things - told by the same tuples while it all stands where it stood (her
    # vision's changed things: told anew each call, the loose things were classed every look, 0.4 s of 2)
    # a body moved, its boxes alone told anew: a bed awake under her rebuilt all 60 each call, her vision sent them
    live = world.__dict__.get('_room_live')
    if live is None or live[0] is not told:
        moving = sorted({int(m.geom_bodyid[g]) for _still, g, _kinds in told if g >= 0})
        owned = {b: [g for _still, g, _kinds in told if g >= 0 and m.geom_bodyid[g] == b] for b in moving}
        live = world._room_live = [told, moving, None, {}, {}, {}, owned]
    pose = d.xpos[live[1]].tobytes() + d.xquat[live[1]].tobytes()
    if live[2] != pose:
        live[2], live[3] = pose, {}
        for b in live[1]:
            at = d.xpos[b].tobytes() + d.xquat[b].tobytes()
            if live[4].get(b) != at:
                live[4][b] = at
                for g in live[6][b]:
                    live[5].pop(g, None)
    if lit not in live[3]:
        out = []
        for still, g, kinds in told:
            out += still[lit]
            if g < 0:
                continue
            built = live[5].setdefault(g, {})
            if lit not in built:
                # a round one (the stool's) the box round it, as a still one's: read as a box, its legs were stairs
                box = _box_of(m, d, g, m.geom_type[g] == world._mj.mjtGeom.mjGEOM_BOX)
                built[lit] = (kinds[lit],) + box if box else None
            if built[lit]:
                out.append(built[lit])
        live[3][lit] = out
    return list(live[3][lit])


def _told_props(world):
    """[(the still boxes before it as told, unlit and lit; a geom read each call, -1 past the last; its kind
    unlit and lit)] in `_laid_props`' order, laid once off it: her vision apart is sent a look's changed
    things alone (`vision.Apart._changed`), each still one the same object."""
    laid = _laid_props(world)
    told = getattr(world, '_room_told', None)
    if told is None or told[0] is not laid:
        runs, still = [], ([], [])
        for g, kind, box, _round in laid:
            kinds = (kind, 'lit' if kind == 'lamp' else kind)
            if box:
                still[0].append((kinds[0],) + box)
                still[1].append((kinds[1],) + box)
            else:
                runs.append((still, g, kinds))
                still = ([], [])
        told = world._room_told = (laid, runs + [(still, -1, None)])
    return told[1]


def _laid_props(world):
    """(geom, kind, its box if it is still, whether it is round) of each lodge geom the page draws, in the model's
    order: a box welded to the world laid once off the model's own kinematics, the door's read each call."""
    laid = getattr(world, '_room_props', None)
    if laid is None:
        m, mj = world.model, world._mj
        still = mj.MjData(m)
        mj.mj_kinematics(m, still)
        laid = world._room_props = []
        for g in range(m.ngeom):
            kind, body = KINDS.get(m.geom(g).name.split('_')[0].rstrip('0123456789')), m.geom_bodyid[g]
            boxed = m.geom_type[g] == mj.mjtGeom.mjGEOM_BOX
            box = _box_of(m, still, g, boxed) if m.body_weldid[body] == 0 else None
            if kind is not None and m.body_mocapid[body] < 0 and (box or m.body_weldid[body] != 0):
                laid.append((g, kind, box, not boxed))
    return laid


def rounds(world):
    """Whether each of `props`' boxes is a round geom's - the stool's -, in its order: the page draws it round."""
    d = world.data
    return [rnd for g, _kind, box, rnd in _laid_props(world) if box or _box_of(world.model, d, g) is not None]


#: A box along the room's axes.
UPRIGHT = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def _box_of(m, d, g, boxed=True):
    """(centre, half sizes, turn 3x3) of geom `g` posed on `d` - a round one's (not `boxed`) the box round it along
    the room's axes, MuJoCo's own (`geom_aabb`) turned -, None below the page's floor."""
    if d.geom_xpos[g][1] < -0.5:
        return None
    turn = d.geom_xmat[g].reshape(3, 3)
    if not boxed:
        return tuple(d.geom_xpos[g].tolist()), tuple((abs(turn) @ m.geom_aabb[g][3:]).tolist()), UPRIGHT
    return tuple(d.geom_xpos[g].tolist()), tuple(m.geom_size[g].tolist()), tuple(map(tuple, turn.tolist()))


def touched(world, dt):
    """The lamp toggled where a hand of hers touches its switch, not twice within SWITCH_S - the
    switch's geom against a hand's, MuJoCo's contacts paired in C (`wbc_pass_paired`); whether it
    is on."""
    d = world.data
    _masks, at, paired, address = _switch_laid(world)
    since = getattr(world, 'switched', SWITCH_S) + dt
    if since >= SWITCH_S and d.ncon:
        geom, pos = d.contact.geom, d.contact.pos
        if paired(address(geom), address(pos), geom.strides[0], d.ncon, *at, math.inf):
            world.lamp, since = not getattr(world, 'lamp', False), 0.0
    world.switched = since
    return getattr(world, 'lamp', False)


def switch(world):
    """(her room's switch's geom, her hands' geoms): a bool a geom each, laid once on `world` (`touched`'s, and her
    world's steps in C, `machine.world.steps`)."""
    return _switch_laid(world)[0]


def _switch_laid(world):
    """(the masks, their addresses, wbc_pass_paired, an array's address) on `world`, laid at the first ask."""
    laid = getattr(world, '_room_switch', None)
    if laid is None:
        import numpy as np
        from tools.cores import wbc as cores
        m = world.model
        hands = [m.body(side + part).id for side in ('left', 'right') for part in ('_hand', '_fingers')]
        masks = (np.arange(m.ngeom) == m.geom('switch').id, np.isin(m.geom_bodyid, hands))
        laid = world._room_switch = (masks, [cores.address(k) for k in masks], cores.library().wbh_pass_paired,
                                     cores.address)
    return laid

