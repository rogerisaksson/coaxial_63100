"""The lodge's boxes as the page and the robot's eyes' stand-in see them, what moves told anew.

    boxed.props(world)          # [(kind, centre, half sizes, turn 3x3)], the door where it swings, the lamp lit
    boxed.rounds(world)         # whether each is a round geom's - the stool's -: the page draws it round
"""
from machine.world.room import KINDS


def props(world):
    """The lodge's boxes for the page: [(kind, centre, half sizes, turn 3x3)], the door where it swings, the lamp
    'lit' while it is on; a still box told by the same tuple each call (`_told_props`), what moves - the door, the
    loose things - by the same tuples while it stands where it stood (the vision's changed things: told anew each
    call, the loose things were classed every look, 0.4 s of 2), a body moved its boxes alone told anew, each in
    its place in the boxes told last where it is still told as it was - a box, or none under the floor (`_moved_in`):
    a chair or a bed creeping under the robot moves every step, its boxes laid whole 177 us a call, at each look
    and each state told its page."""
    m, d, lit, told = world.model, world.data, bool(getattr(world, 'lamp', False)), _told_props(world)
    live = world.__dict__.get('_room_live')
    if live is None or live[0] is not told:
        moving = sorted({int(m.geom_bodyid[g]) for _still, g, _kinds in told if g >= 0})
        owned = {b: [g for _still, g, _kinds in told if g >= 0 and m.geom_bodyid[g] == b] for b in moving}
        live = world._room_live = [told, moving, None, {}, {}, {}, owned, {}, {g: k for _s, g, k in told if g >= 0}]
    pose = d.xpos[live[1]].tobytes() + d.xquat[live[1]].tobytes()
    if live[2] != pose:
        moved = []
        for b in live[1]:
            at = d.xpos[b].tobytes() + d.xquat[b].tobytes()
            if live[4].get(b) != at:
                live[4][b] = at
                moved += live[6][b]
        for g in moved:
            live[5].pop(g, None)
        out, at = live[3].get(lit), live[7].get(lit)
        live[2], live[3], live[7] = pose, {}, {}
        if out is not None and _moved_in(world, live, lit, out, at, moved):
            live[3][lit], live[7][lit] = out, at
    if lit not in live[3]:
        out, at = [], {}
        for still, g, _kinds in told:
            out += still[lit]
            if g < 0:
                continue
            box = _built(world, live, g, lit)
            if box:
                at[g] = len(out)
                out.append(box)
        live[3][lit], live[7][lit] = out, at
    return list(live[3][lit])


def _built(world, live, g, lit):
    """Geom `g`'s box as the page has it, `lit` or not - its kind, centre, half sizes, turn -, None under the page's
    floor: told once a pose of its body (`props`' `live`)."""
    built = live[5].setdefault(g, {})
    if lit not in built:
        m = world.model
        # a round one (the stool's) the box round it, as a still one's: read as a box, its legs were stairs
        box = _box_of(m, world.data, g, m.geom_type[g] == world._mj.mjtGeom.mjGEOM_BOX)
        built[lit] = (live[8][g][lit],) + box if box else None
    return built[lit]


def _moved_in(world, live, lit, out, at, moved):
    """Whether the boxes `out` told last - `at` each moving geom's place in them - take each geom of `moved` told anew
    in its place, each still told as it was (a box, or none): put there where so."""
    boxes = [(g, _built(world, live, g, lit)) for g in moved]
    if any((g in at) != bool(box) for g, box in boxes):
        return False
    for g, box in boxes:
        if box:
            out[at[g]] = box
    return True


def _told_props(world):
    """[(the still boxes before it as told, unlit and lit; a geom read each call, -1 past the last; its kind
    unlit and lit)] in `_laid_props`' order, laid once off it: the robot's vision apart is sent a look's
    changed things alone (`vision.Apart._changed`), each still one the same object."""
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
