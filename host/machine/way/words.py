"""The robot's errand's words: what it goes to, what its planner is asked, what each phase tells it.

The robot's errand (`machine.way.errand`) steps its phases by a table, a row's test and the phase next and its
word; its words are here - each a function of the robot's world `g`, its gym, the errand, the pass's
context and what hears it -, beside the thing an errand goes to among its things (`_seen`,
`_within`, `_near`, `_glimpsed`), the errand laid on it and the robot's planner asked its dock and way
(`_laid`, `_asked`, `_docked`, `_looking`).
"""
import functools
import math
import operator

from machine import libm
from machine.way.tour import STOP
from machine.way.steer import steer, tell


def _fire_errand_stunt(g, gym, kind, heard_say_fn, at=None, thing=None):
    from machine.texts import maneuver
    from machine.eyes import gaze
    g['_errand'] = None
    gaze.rest(g)
    g['_twist_active'], g['_twist_stop_until'], g['_halt_at'], g['has_heading'] = False, 0.0, None, 0.0
    g['twist'] = g['asked'] = STOP
    by = {e[0]: e for e in gym}
    if kind in by:
        heard_say_fn(g, gym, maneuver.said(g, by[kind], at, thing))


def _seen(g, sc, at, swept=0.0):
    """What `sc` goes to nearest `at` among the robot's things - told a place, the thing there (`_there`) till it has
    looked a full turn round (`swept`, rad) -, None with nothing of the kind among them."""
    from machine.way import furniture
    hit = furniture.nearest(g, sc, at)
    return None if hit[4] is None or not _there(hit[6], at, swept) else hit


def _there(box, at, swept=0.0):
    """Whether `box` is the thing told at `at`: none told, a full turn looked round (`swept`, rad), or `at` on its
    footprint a stride round. Taken the first that afforded it, the robot's bed errand went for the loft 4.4 m off
    the bed the robot was told: its textured floor's first look round told the loft a bed before the bed
    came in sight."""
    from machine.way import affordance
    from machine.law import stride
    return at is None or swept >= math.tau or affordance.distance(box, at) <= stride.STRIDE_M


def _ready_here(g, sc, hit, at):
    """Whether `hit` within a stride is already behind the robot's heels on a dockable face so the stunt can run in
    place."""
    from machine.law import se2, stride
    from machine.senses import edge
    from machine.way import furniture
    from machine.world import physics
    if hit[0] > stride.STRIDE_M:
        return False
    if g is None or sc is None or not (physics.ROOM or g.get('eyes')):
        return True
    sf = furniture.seen(g, sc, at, hit[4])
    if sf is None or not (0.0 <= sf[1] <= stride.STRIDE_M and abs(sf[2]) <= 0.5 * sf[5] and abs(sf[3]) < 0.6):
        return False
    yaw = edge.yaw(g['sense'])
    return not hit[5] or any(abs(se2.wrap_angle(fh + _FACES.get(sc, 0.0) - yaw)) < 0.6 for fh in hit[5])


def _within(sc, hit, g=None, at=None):
    """Whether a thing found is within the robot's reach: within three rooms' widths and not yet poised at hand; the door
    anywhere."""
    from machine.world import room
    return sc == 'exit_door' or (not _ready_here(g, sc, hit, at) and hit[0] < 3.0 * room.WIDE)


def _near(hit, g=None, sc=None, at=None):
    """Whether a thing found is poised at hand: within a stride behind the robot's heels."""
    return _ready_here(g, sc, hit, at)


def _glimpsed(g, sc, at):
    """Whether `sc` goes to a thing among the robot's registry's live slots at any salience: a sighting not
    yet its own (`registry.TAKEN`), its look held still while it firms up."""
    from machine.way import furniture
    things = g['vision'].boxes(g, -math.inf)
    if not furniture.slotted(g, sc, at, things=things):
        return False
    box = furniture.nearest(g, sc, at, things=things)[6]
    return box is not None and _there(box, at)


#: The robot's heading docked off the dock's normal, by errand: through the door; its back to a seat, the
#: rest.
_FACES = {'exit_door': math.pi}


def _asked(g, e, kind, to=None, passed=0.0, way=(), dock=None):
    """The robot's planner asked a `kind` (`machine.way.planner`: a dock of `dock` (verb, box, standoff), a
    plan to `to`, a watch of its `way` left) from where it will be at its answer, a watch's
    LATENCY_S on at the robot's velocity and turn rate: from where it stood, a replan walking began 0.2
    m behind it and it fell."""
    from machine.senses import edge
    from machine.law import latent
    from machine.way import furniture, planner
    q, v, on = furniture.where(g), g['est']['vel'], planner.LATENCY_S * float(kind == 'watch')
    g['planner'].ask(g['t'], {
        'kind': kind, 'at': (float(q[0]) + float(v[0]) * on, float(q[2]) + float(v[2]) * on),
        'yaw': edge.yaw(g['sense']) + float(g['sense'][edge.GYRO][1]) * on,
        'still': float(g['st'][latent.NOW][latent.GAIN]) < 1e-6, 'things': list(g['things']),
        'floor': furniture.floor(g), 'seen': None, 'slot': e['slot'], 'face': e['face'],
        'to': to, 'dock': dock, 'warm': e['held'], 'passed': passed, 'way': way})


def _laid(g, sc, found, at):
    """The errand `sc` on the thing `found` (`_seen`'s), its dock and way asked of the robot's planner. Run at
    the door in one motion without the stage the robot slammed the panel to its 1.92 rad stop in 0.5 s and its
    rebound felled the robot outside; settled at the stage first it grinds it open over 3 s and walks
    out."""
    from machine.way import dock, peek
    box = found[6]
    top = peek.level(sc, box)
    standoff = dock.standoff(sc, top, g['suite'])
    er = dict(_looking(g, sc, at), phase='plan', through=sc in _FACES, thing=found[4], standoff=standoff,
              slot=int(box[4]) if len(box) > 4 else -1, top=top)
    _asked(g, er, 'dock', dock=(sc, box, standoff))
    return er


def _docked(sc, laid):
    """The errand's fields of its dock `laid` (`dock.docked`), the robot's heading docked (`_FACES`)."""
    from machine.law import se2, stride
    hn, (x_dock, z_dock), stage = laid['hn'], laid['edge'], laid['stage']
    nx, nz = libm.sin(hn), libm.cos(hn)
    return {'docks': tuple((float(h), float(x), float(z)) for h, (x, z) in laid.get('reach', ())),
            'x_dock': x_dock, 'z_dock': z_dock, 'nx': nx, 'nz': nz, 'tx': nz, 'tz': -nx, 'face': laid['face'],
            'xd': stage[0], 'zd': stage[1], 'hdd': se2.wrap_angle(hn + _FACES.get(sc, 0.0)),
            'off': laid.get('off', 0.0), 'lat_tol': laid['lat_tol'],
            'd_stage': laid['d_stage'], 'd_most': (laid['d_stage'] or 0.0) + stride.STRIDE_M}


def _looking(g, sc, at):
    """The errand `sc` looking round for its thing: its dock where the robot stands, nothing planned, the
    robot's yaw swept from none."""
    from machine.senses import edge
    from machine.way import furniture
    q, yaw = furniture.where(g), edge.yaw(g['sense'])
    return {'kind': sc, 'phase': 'look', 'thing': None, 'at': at, 'swept': 0.0, 'yaw_was': yaw,
            'x_dock': float(q[0]), 'z_dock': float(q[2]), 'nx': libm.sin(yaw), 'nz': libm.cos(yaw),
            'tx': libm.cos(yaw), 'tz': -libm.sin(yaw), 'xd': float(q[0]), 'zd': float(q[2]), 'hdd': yaw,
            'lat_tol': 0.0, 'd_stage': 0.0, 'd_most': 0.0, 'turn_dir': 0.0, 'standoff': 0.0, 'retreats': 0,
            'way': [], 'curve': (), 'k': 0, 'held': (), 'slot': -1, 'face': None, 'docks': (), 'scanned': False}


def _told(twist):
    """A row's word: `twist`, the robot's heading the dock's."""
    return lambda g, gym, er, c, heard_fn, _say_fn: tell(g, gym, heard_fn, {'heading': er['hdd'], 'twist': twist})


def _begun(g, gym, er, c, heard_fn, _say_fn):
    """A thing found looking round, within reach: the errand on it (`_laid`), stood."""
    er.update(_laid(g, er['kind'], c['seen'], er.get('at')), phase=er['phase'])
    g['has_heading'] = 0.0
    tell(g, gym, heard_fn, {'twist': STOP})


def _took(g, gym, er, c, heard_fn, _say_fn):
    """The robot's planner's answer: its dock where it laid one, the robot's way."""
    a = c['answer']
    er.update(_docked(er['kind'], a['laid']) if a.get('laid') else {})
    er.update(way=a['stops'], curve=a['curve'], held=a['held'], k=0, turn_dir=0.0)


def _docking(er):
    """(x, y, z) the place the robot docks at: its dock's edge at the thing's top (`peek.level`)."""
    return er['x_dock'], er['top'], er['z_dock']


def _scanning(g, gym, er, c, heard_fn, _say_fn):
    """The robot's planner's way taken (`_took`), stood: its eyes on each of its places in turn, its
    corners on the robot's floor and its dock (`gaze.scan`) - what they show its own before the
    robot walks it, the robot's way scanned once."""
    from machine.eyes import gaze
    from machine.way import furniture
    _took(g, gym, er, c, heard_fn, _say_fn)
    floor = furniture.floor(g)
    gaze.scan(g, [(x, floor, z) for x, z in er['way']] + [_docking(er)])
    er['scanned'] = True
    tell(g, gym, heard_fn, {'twist': STOP})


def _reported(g, gym, er, c, heard_fn, _say_fn):
    """The robot's way scanned: its planner told what its eyes hold along it now (a watch, `_asked`) -
    barred, a way round."""
    from machine.way import furniture
    q = furniture.where(g)
    _asked(g, er, 'watch', (er['xd'], er['zd']), way=[(float(q[0]), float(q[2]))] + [tuple(p) for p in er['way']])


def _fixed(g, gym, er, c, heard_fn, _say_fn):
    """The robot's planner's word on its way: the way it gave taken where it gave one (`_took`), the robot's
    gaze held on its dock as it walks there led by its way (`gaze.fix`) - on its ground ahead
    of the robot by the time what a look shows there takes to stand it: its own in HELD looks, the
    robot's planner's watch and its answer, the robot's brake's run-out."""
    from machine.eyes import gaze, sight
    from machine.way import planner, route
    if 'stops' in c['answer']:
        _took(g, gym, er, c, heard_fn, _say_fn)
    gaze.fix(g, _docking(er), er['curve'], gaze.HELD * sight.LOOK_S + planner.WATCH_S + planner.LATENCY_S + route.RUN_OUT_S)


def _faces(g, kind, box):
    """The headings `kind` goes to `box` from among the robot's things (`furniture.GOES`), told anew only as its
    things change - the very ones last told, `box` one of them (`furniture._afforded`'s rule): refreshed each pass
    of an errand's settle, a chair's back sought among the lodge's 64 things (`affordance.back_of`) was 365 us a
    pass."""
    from machine.way import furniture
    things = g['things']
    kept = g.get('_faces')
    if (kept is None or kept[0] != kind or kept[1] is not box or len(kept[2]) != len(things)
            or not all(map(operator.is_, kept[2], things))):
        kept = g['_faces'] = (kind, box, tuple(things), furniture.GOES.get(kind, furniture._NONE)[2](box, things))
    return kept[3]


@functools.cache
def _refreshing():
    """What `_refreshed` reads, imported at its first refresh and not looked up again: sat or lain it refreshes
    each pass, its imports 8 of each."""
    from machine.body import figure
    from machine.eyes import voxels
    from machine.law import se2, stride
    from machine.moves import seated
    from machine.way import affordance, dock, furniture, peek
    return figure, voxels, se2, stride, seated, affordance, dock, furniture, peek


def _refreshed(g, er):
    """The robot's dock's edge on its thing as the robot's map holds it close (`_arrived`, `_faced`, `settle`):
    laid from afar or nudged loose as the robot arrives, its edge and heading update before the robot backs and
    sits - unrefreshed, a low chair glimpsed 0.09 m shallow or a loose chair turned and nudged 0.09 m deeper left
    the robot's hips on its lip."""
    things = g.get('things')
    if not things or er['through']:
        return
    suite = g.get('suite')
    state = (er['kind'], er['x_dock'], er['z_dock'], er['nx'], er['nz'], er.get('thing'), er.get('top'),
             er.get('off'), id(suite))
    kept = er.get('_refreshed_key')
    if kept is not None and kept[0] == state and len(kept[1]) == len(things) and all(map(operator.is_, kept[1], things)):
        return
    figure, voxels, se2, stride, seated, affordance, dock, furniture, peek = _refreshing()
    box = furniture.nearest(g, er['kind'], (er['x_dock'], er['z_dock']), thing=er.get('thing'))[6]
    if box is None or affordance.distance(box, (er['x_dock'], er['z_dock'])) > stride.STRIDE_M:
        er['_refreshed_key'] = (state, tuple(things))
        return
    top = peek.level(er['kind'], box)
    if 'top' in er and abs(top - er['top']) > 2.0 * voxels.CELL_M:
        er['_refreshed_key'] = (state, tuple(things))
        return
    hn0 = libm.arctan2(er['nx'], er['nz'])
    faces = _faces(g, er['kind'], box)
    hn = min(faces, key=lambda h: abs(se2.wrap_angle(h - hn0))) if faces else hn0
    nx, nz = libm.sin(hn), libm.cos(hn)
    tx, tz = nz, -nx
    over = er['kind'] == 'sit_chair' and affordance.top(box) > seated.POPLITEAL
    across = seated.across() if over else dock.docks()[er['kind']][0]
    edge, span = affordance.support(box, hn), affordance.support(box, hn + 0.5 * math.pi)
    lat = max(0.0, span - (min(across, span) if span >= figure.HIP_HALF else across))
    mx, mz = float(box[1][0]) + edge * nx, float(box[1][2]) + edge * nz
    was = er['off'] if 'off' in er else (er['x_dock'] - mx) * tx + (er['z_dock'] - mz) * tz
    off = max(-lat, min(lat, was))
    s_key = (er['kind'], top, id(suite))
    standoff = er['standoff'] if er.get('_standoff_key') == s_key else dock.standoff(er['kind'], top, suite)
    lat_tol = min(span - figure.HIP_HALF, figure.SOLE_HALF) if lat == 0.0 else max(0.0, lat - abs(off))
    er.update(x_dock=mx + off * tx, z_dock=mz + off * tz, nx=nx, nz=nz, tx=tx, tz=tz,
              hdd=se2.wrap_angle(hn + _FACES.get(er['kind'], 0.0)), lat_tol=lat_tol,
              top=top, standoff=standoff, _standoff_key=s_key,
              thing=box[4] if len(box) > 4 else er.get('thing'))
    er['_refreshed_key'] = ((er['kind'], er['x_dock'], er['z_dock'], er['nx'], er['nz'], er.get('thing'),
                             er.get('top'), er.get('off'), id(suite)), tuple(things))


def _arrived(g, gym, er, c, heard_fn, _say_fn):
    """Walked to the robot's stage (`_stood`), its gaze let go (`gaze.rest`): its dock its body's now."""
    from machine.eyes import gaze
    _refreshed(g, er)
    _stood(g, gym, er, c, heard_fn, _say_fn)
    gaze.rest(g)


def _ran(g, gym, er, c, heard_fn, _say_fn):
    """The robot's planner's run out taken, no curve: the door's push is contact (`_out`)."""
    a = c['answer']
    er.update(way=a['stops'], curve=(), held=a['held'], k=0, turn_dir=0.0)


def _held(g, gym, er, c, heard_fn, _say_fn):
    """A glimpse of the kind not yet the robot's own: stood still while it firms up."""
    tell(g, gym, heard_fn, {'twist': STOP})


def _at_hand(g, gym, er, c, heard_fn, heard_say_fn):
    """A thing found looking round within a stride: no errand, the stunt on it where the robot stands, as a
    say's with one at hand (`start_errand` refused, `worker._apply_say`)."""
    _fire_errand_stunt(g, gym, er['kind'], heard_say_fn, er.get('at'), c['seen'][4])


def _to_look(g, gym, er, c, heard_fn, _say_fn):
    """Turning in place for a thing the verb goes to, the robot's yaw's sweep summed."""
    from machine.law import se2
    er['swept'] += abs(se2.wrap_angle(c['pyaw'] - er['yaw_was']))
    er['yaw_was'] = c['pyaw']
    tell(g, gym, heard_fn, {'twist': (0.0, 0.0, se2.YAW_RATE)})


def _stood(g, gym, er, c, heard_fn, _say_fn):
    """Walked to the box: stood, the robot's heading released."""
    er['turn_dir'], g['has_heading'] = 0.0, 0.0
    tell(g, gym, heard_fn, {'twist': STOP})


def _cornered(g, gym, er, c, heard_fn, _say_fn):
    """Stood still on the robot's way: the rest asked of its planner, its stage last - laid
    inside its map's grown cells by its turn's own clearance (`dock.docked`), not moved for them."""
    from machine.eyes import voxels
    from machine.law import stride
    _asked(g, er, 'plan', (er['xd'], er['zd']), stride.STRIDE_M + voxels.CELL_M)
    er['turn_dir'] = 0.0


def _halted(g, gym, er, c, heard_fn, _say_fn):
    """Settling: told to stand, once - laid within the robot's run-out of its stage it may have been
    walking."""
    tell(g, gym, heard_fn, {'twist': STOP})


def _faced(g, gym, er, c, heard_fn, _say_fn):
    """Faced away from the prop with the dock's heading."""
    _refreshed(g, er)
    er['turn_dir'] = 0.0
    tell(g, gym, heard_fn, {'heading': er['hdd'], 'twist': STOP})


def _ended(g, gym, er, c, heard_fn, _say_fn):
    """Out of the door, or a look round with nothing in reach found: the errand over, stood, the robot's gaze
    let go."""
    from machine.eyes import gaze
    g['_errand'], g['has_heading'] = None, 0.0
    gaze.rest(g)
    tell(g, gym, heard_fn, {'twist': STOP})


def _stunt(g, gym, er, c, heard_fn, heard_say_fn):
    """Docked and still: the stunt begins on the thing as the robot sees it, the errand over."""
    _fire_errand_stunt(g, gym, er['kind'], heard_say_fn, (er['x_dock'], er['z_dock']), er.get('thing'))


def _nothing(g, gym, er, c, heard_fn, heard_say_fn):
    return None


def _out(g, gym, er, c, heard_fn, _say_fn):
    """Settled at the stage facing out: the run out laid on the robot's way's corners round its things
    through the doorway to the dock's far side (`route.planned`; hand-aimed from a stage reached
    aslant it jammed on the wall at x 1.33 for the opening's 0.3-1.2, 24 s). No curve: the push
    through the yielding door is contact, aimed at the next corner far ahead - the curve's near
    tracking fought the opening panel's shove and wedged the robot on the hinge-side post at 0.92 rad."""
    goal = (er['x_dock'] - er['d_most'] * er['nx'], er['z_dock'] - er['d_most'] * er['nz'])
    er['xd'], er['zd'] = goal
    _asked(g, er, 'plan', goal)
    er['turn_dir'] = 0.0


def _passed(g, gym, er, c, heard_fn, _say_fn):
    """A corner of the run out reached: the next is the aim."""
    er['way'].pop(0)


@functools.cache
def _modal():
    """machine.law.modal, imported at a first aim and not looked up again each pass."""
    from machine.law import modal
    return modal


def _to_aim(g, gym, er, c, heard_fn, _say_fn):
    """Walking to the robot's aim (`step_errand`'s) at its way's pace there (`route.rounded`'s, the
    robot's walk's without a curve); its turn's way held while it is all but behind the robot."""
    modal = _modal()
    err_w, behind = c['err_w'], abs(c['err_w']) > math.pi - modal.LIST_RAD
    er['turn_dir'] = (er['turn_dir'] or math.copysign(1.0, err_w)) if behind else math.copysign(1.0, err_w)
    steer(g, gym, heard_fn, err_w, er['turn_dir'], c['pace'], c['off_w'], c['aim_m'], er['phase'] == 'door')


def _to_face(g, gym, er, c, heard_fn, _say_fn):
    """Turning in place to face away from the prop, the way that clears the robot's things, held once
    begun."""
    er['turn_dir'] = er['turn_dir'] or math.copysign(1.0, c['err_yaw'])
    steer(g, gym, heard_fn, c['err_yaw'], er['turn_dir'], 0.0, c['err_yaw'])


def _turning(g, gym, er, c, heard_fn, _say_fn):
    """Still, the robot's turn clear of its things: turning the way that clears them (`dock.turns`)."""
    er['turn_dir'] = c['turns']


def _retreat(g, gym, er, c, heard_fn, _say_fn):
    """Still facing the face where no turn onto the dock clears the robot's things: backing off it, the
    robot's heading as it is - the robot's walk back stops in BACK_STOP_M."""
    from machine.law import se2
    er['retreats'] += 1
    g['has_heading'] = 0.0
    tell(g, gym, heard_fn, {'twist': (-se2.BACK_SPEED, 0.0, 0.0)})


def _to_line(g, gym, er, c, heard_fn, _say_fn):
    """Sidestepping toward the dock's line, facing away from the prop."""
    from machine.law import modal, se2
    er['stepped'] = bool(er.get('stepped') or float(g['a'][modal.STANCE].min()) < 0.5)
    g['has_heading'], g['target_heading'] = 1.0, er['hdd']
    tell(g, gym, heard_fn, {'heading': er['hdd'], 'twist': (0.0, math.copysign(se2.STRAFE_SPEED, c['way_left']), 0.0)})


def _to_edge(g, gym, er, c, heard_fn, _say_fn):
    """Backing onto the edge, the robot's heading bent toward the dock's line as it bears from the robot's
    stage: bent as the dock bears from its, it came to it 0.5 rad askew and sat on its edge."""
    from machine.law import modal, stride
    er['stepped'] = bool(er.get('stepped') or float(g['a'][modal.STANCE].min()) < 0.5)
    g['has_heading'] = 1.0
    g['target_heading'] = er['hdd'] + libm.arctan2(c['s_perp'], max(er['d_stage'], stride.STRIDE_M))
