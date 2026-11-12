"""Her errand's words: the thing she goes to, what her planner is asked, what each phase tells her.

Her errand (`machine.way.errand`) steps its phases by a table, a row's test and the phase next and its word;
its words are here - each a function of her world `g`, her gym, the errand, the pass's context and what
hears her -, beside the thing an errand goes to among her things (`_seen`, `_within`, `_near`, `_glimpsed`),
the errand laid on it and her planner asked its dock and way (`_laid`, `_asked`, `_docked`, `_looking`).
"""
import functools
import math

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
    """What `sc` goes to nearest `at` among her things - told a place, the thing there (`_there`) till she has looked a
    full turn round (`swept`, rad) -, None with nothing of the kind among them."""
    from machine.way import furniture
    hit = furniture.nearest(g, sc, at)
    return None if hit[4] is None or not _there(hit[6], at, swept) else hit


def _there(box, at, swept=0.0):
    """Whether `box` is the thing told at `at`: none told, a full turn looked round (`swept`, rad), or `at` on its
    footprint a stride round. Taken the first that afforded it, her bed errand went for the loft 4.4 m off the bed
    she was told: her textured floor's first look round told the loft a bed before the bed came in sight."""
    from machine.way import affordance
    from machine.law import stride
    return at is None or swept >= math.tau or affordance.distance(box, at) <= stride.STRIDE_M


def _within(sc, hit):
    """Whether a thing found is within her reach: past a stride, within three rooms' widths; the door anywhere."""
    from machine.world import room
    from machine.law import stride
    return sc == 'exit_door' or stride.STRIDE_M < hit[0] < 3.0 * room.WIDE


def _near(hit):
    """Whether a thing found is at hand: within a stride."""
    from machine.law import stride
    return hit[0] <= stride.STRIDE_M


def _glimpsed(g, sc, at):
    """Whether `sc` goes to a thing among her registry's live slots at any salience: a sighting not yet
    hers (`registry.TAKEN`), her look held still while it firms up."""
    from machine.way import furniture
    things = g['vision'].boxes(g, -math.inf)
    if not furniture.slotted(g, sc, at, things=things):
        return False
    box = furniture.nearest(g, sc, at, things=things)[6]
    return box is not None and _there(box, at)


#: Her heading docked off the dock's normal, by errand: through the door; her back to a seat, the rest.
_FACES = {'exit_door': math.pi}


def _asked(g, e, kind, to=None, passed=0.0, way=(), dock=None):
    """Her planner asked a `kind` (`machine.way.planner`: a dock of `dock` (verb, box, standoff), a plan to
    `to`, a watch of her `way` left) from where she will be at its answer, a watch's LATENCY_S on at her
    velocity and turn rate: from where she stood, a replan walking began 0.2 m behind her and she fell."""
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
    """The errand `sc` on the thing `found` (`_seen`'s), its dock and way asked of her planner. Run at the
    door in one motion without the stage she slammed the panel to its 1.92 rad stop in 0.5 s and its
    rebound felled her outside; settled at the stage first she grinds it open over 3 s and walks out."""
    from machine.texts import maneuver
    from machine.way import peek
    standoff = maneuver.standoff(sc, g['suite']) if sc in maneuver.PLAYED else 0.0
    box = found[6]
    er = dict(_looking(g, sc, at), phase='plan', through=sc in _FACES, thing=found[4], standoff=standoff,
              slot=int(box[4]) if len(box) > 4 else -1, top=peek.level(sc, box))
    _asked(g, er, 'dock', dock=(sc, box, standoff))
    return er


def _docked(sc, laid):
    """The errand's fields of its dock `laid` (`dock.docked`), her heading docked (`_FACES`)."""
    from machine.law import se2
    hn, (x_dock, z_dock), stage = laid['hn'], laid['edge'], laid['stage']
    nx, nz = math.sin(hn), math.cos(hn)
    return {'docks': tuple((float(h), float(x), float(z)) for h, (x, z) in laid.get('reach', ())),
            'x_dock': x_dock, 'z_dock': z_dock, 'nx': nx, 'nz': nz, 'tx': nz, 'tz': -nx, 'face': laid['face'],
            'xd': stage[0], 'zd': stage[1], 'hdd': se2.wrap_angle(hn + _FACES.get(sc, 0.0)), 'lat_tol': laid['lat_tol'],
            'd_stage': laid['d_stage'], 'd_most': laid['d_stage']}


def _looking(g, sc, at):
    """The errand `sc` looking round for its thing: its dock where she stands, nothing planned, her yaw
    swept from none."""
    from machine.senses import edge
    from machine.way import furniture
    q, yaw = furniture.where(g), edge.yaw(g['sense'])
    return {'kind': sc, 'phase': 'look', 'thing': None, 'at': at, 'swept': 0.0, 'yaw_was': yaw,
            'x_dock': float(q[0]), 'z_dock': float(q[2]), 'nx': math.sin(yaw), 'nz': math.cos(yaw),
            'tx': math.cos(yaw), 'tz': -math.sin(yaw), 'xd': float(q[0]), 'zd': float(q[2]), 'hdd': yaw,
            'lat_tol': 0.0, 'd_stage': 0.0, 'd_most': 0.0, 'turn_dir': 0.0, 'standoff': 0.0, 'retreats': 0,
            'way': [], 'curve': (), 'k': 0, 'held': (), 'slot': -1, 'face': None, 'docks': (), 'scanned': False}


def _told(twist):
    """A row's word: `twist`, her heading the dock's."""
    return lambda g, gym, er, c, heard_fn, _say_fn: tell(g, gym, heard_fn, {'heading': er['hdd'], 'twist': twist})


def _begun(g, gym, er, c, heard_fn, _say_fn):
    """A thing found looking round, within reach: the errand on it (`_laid`), stood."""
    er.update(_laid(g, er['kind'], c['seen'], er.get('at')), phase=er['phase'])
    g['has_heading'] = 0.0
    tell(g, gym, heard_fn, {'twist': STOP})


def _took(g, gym, er, c, heard_fn, _say_fn):
    """Her planner's answer: her dock where it laid one, her way."""
    a = c['answer']
    er.update(_docked(er['kind'], a['laid']) if a.get('laid') else {})
    er.update(way=a['stops'], curve=a['curve'], held=a['held'], k=0, turn_dir=0.0)


def _docking(er):
    """(x, y, z) the place she docks at: her dock's edge at the thing's top (`peek.level`)."""
    return er['x_dock'], er['top'], er['z_dock']


def _scanning(g, gym, er, c, heard_fn, _say_fn):
    """Her planner's way taken (`_took`), stood: her eyes on each of its places in turn, its corners on her floor
    and her dock (`gaze.scan`) - what they show hers before she walks it, her way scanned once."""
    from machine.eyes import gaze
    from machine.way import furniture
    _took(g, gym, er, c, heard_fn, _say_fn)
    floor = furniture.floor(g)
    gaze.scan(g, [(x, floor, z) for x, z in er['way']] + [_docking(er)])
    er['scanned'] = True
    tell(g, gym, heard_fn, {'twist': STOP})


def _reported(g, gym, er, c, heard_fn, _say_fn):
    """Her way scanned: her planner told what her eyes hold along it now (a watch, `_asked`) - barred, a way round."""
    from machine.way import furniture
    q = furniture.where(g)
    _asked(g, er, 'watch', (er['xd'], er['zd']), way=[(float(q[0]), float(q[2]))] + [tuple(p) for p in er['way']])


def _fixed(g, gym, er, c, heard_fn, _say_fn):
    """Her planner's word on her way: the way it gave taken where it gave one (`_took`), her gaze held on her dock as
    she walks there led by her way (`gaze.fix`) - on its ground ahead of her by the time what a look shows there takes
    to stand her: hers in HELD looks, her planner's watch and its answer, her brake's run-out."""
    from machine.eyes import gaze, sight
    from machine.way import planner, route
    if 'stops' in c['answer']:
        _took(g, gym, er, c, heard_fn, _say_fn)
    gaze.fix(g, _docking(er), er['curve'], gaze.HELD * sight.LOOK_S + planner.WATCH_S + planner.LATENCY_S + route.RUN_OUT_S)


def _arrived(g, gym, er, c, heard_fn, _say_fn):
    """Walked to her stage (`_stood`), her gaze let go (`gaze.rest`): her dock her body's now."""
    from machine.eyes import gaze
    _stood(g, gym, er, c, heard_fn, _say_fn)
    gaze.rest(g)


def _ran(g, gym, er, c, heard_fn, _say_fn):
    """Her planner's run out taken, no curve: the door's push is contact (`_out`)."""
    a = c['answer']
    er.update(way=a['stops'], curve=(), held=a['held'], k=0, turn_dir=0.0)


def _held(g, gym, er, c, heard_fn, _say_fn):
    """A glimpse of the kind not yet hers: stood still while it firms up."""
    tell(g, gym, heard_fn, {'twist': STOP})


def _at_hand(g, gym, er, c, heard_fn, heard_say_fn):
    """A thing found looking round within a stride: no errand, the stunt on it where she stands, as a
    say's with one at hand (`start_errand` refused, `worker._apply_say`)."""
    _fire_errand_stunt(g, gym, er['kind'], heard_say_fn, er.get('at'), c['seen'][4])


def _to_look(g, gym, er, c, heard_fn, _say_fn):
    """Turning in place for a thing the verb goes to, her yaw's sweep summed."""
    from machine.law import se2
    er['swept'] += abs(se2.wrap_angle(c['pyaw'] - er['yaw_was']))
    er['yaw_was'] = c['pyaw']
    tell(g, gym, heard_fn, {'twist': (0.0, 0.0, se2.YAW_RATE)})


def _stood(g, gym, er, c, heard_fn, _say_fn):
    """Walked to the box: stood, her heading released."""
    er['turn_dir'], g['has_heading'] = 0.0, 0.0
    tell(g, gym, heard_fn, {'twist': STOP})


def _cornered(g, gym, er, c, heard_fn, _say_fn):
    """Stood still on her way: the rest asked of her planner, her stage last - laid inside her map's
    grown cells by its turn's own clearance (`dock.docked`), not moved for them."""
    from machine.eyes import voxels
    from machine.law import stride
    _asked(g, er, 'plan', (er['xd'], er['zd']), stride.STRIDE_M + voxels.CELL_M)
    er['turn_dir'] = 0.0


def _halted(g, gym, er, c, heard_fn, _say_fn):
    """Settling: told to stand, once - laid within her run-out of her stage she may have been walking."""
    tell(g, gym, heard_fn, {'twist': STOP})


def _faced(g, gym, er, c, heard_fn, _say_fn):
    """Faced away from the prop with the dock's heading."""
    er['turn_dir'] = 0.0
    tell(g, gym, heard_fn, {'heading': er['hdd'], 'twist': STOP})


def _ended(g, gym, er, c, heard_fn, _say_fn):
    """Out of the door, or a look round with nothing in reach found: the errand over, stood, her gaze let go."""
    from machine.eyes import gaze
    g['_errand'], g['has_heading'] = None, 0.0
    gaze.rest(g)
    tell(g, gym, heard_fn, {'twist': STOP})


def _stunt(g, gym, er, c, heard_fn, heard_say_fn):
    """Docked and still: the stunt begins on the thing as she sees it, the errand over."""
    _fire_errand_stunt(g, gym, er['kind'], heard_say_fn, (er['x_dock'], er['z_dock']), er.get('thing'))


def _nothing(g, gym, er, c, heard_fn, heard_say_fn):
    return None


def _out(g, gym, er, c, heard_fn, _say_fn):
    """Settled at the stage facing out: the run out laid on her way's corners round her things
    through the doorway to the dock's far side (`route.planned`; hand-aimed from a stage reached
    aslant she jammed on the wall at x 1.33 for the opening's 0.3-1.2, 24 s). No curve: the push
    through the yielding door is contact, aimed at the next corner far ahead - the curve's near
    tracking fought the opening panel's shove and wedged her on the hinge-side post at 0.92 rad."""
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
    """Walking to her aim (`step_errand`'s) at her way's pace there (`route.rounded`'s, her walk's
    without a curve); her turn's way held while it is all but behind her."""
    modal = _modal()
    err_w, behind = c['err_w'], abs(c['err_w']) > math.pi - modal.LIST_RAD
    er['turn_dir'] = (er['turn_dir'] or math.copysign(1.0, err_w)) if behind else math.copysign(1.0, err_w)
    steer(g, gym, heard_fn, err_w, er['turn_dir'], c['pace'], c['off_w'], c['bend'], c['aim_m'])


def _to_face(g, gym, er, c, heard_fn, _say_fn):
    """Turning in place to face away from the prop, the way that clears her things, held once begun."""
    er['turn_dir'] = er['turn_dir'] or math.copysign(1.0, c['err_yaw'])
    steer(g, gym, heard_fn, c['err_yaw'], er['turn_dir'], 0.0, c['err_yaw'], 0.0)


def _turning(g, gym, er, c, heard_fn, _say_fn):
    """Still, her turn clear of her things: turning the way that clears them (`dock.turns`)."""
    er['turn_dir'] = c['turns']


def _retreat(g, gym, er, c, heard_fn, _say_fn):
    """Still facing the face where no turn onto the dock clears her things: backing off it, her heading
    as it is - her walk back stops in BACK_STOP_M."""
    from machine.law import se2
    er['retreats'] += 1
    g['has_heading'] = 0.0
    tell(g, gym, heard_fn, {'twist': (-se2.BACK_SPEED, 0.0, 0.0)})


def _to_line(g, gym, er, c, heard_fn, _say_fn):
    """Sidestepping toward the dock's line, facing away from the prop."""
    from machine.law import se2
    g['has_heading'], g['target_heading'] = 1.0, er['hdd']
    tell(g, gym, heard_fn, {'heading': er['hdd'], 'twist': (0.0, math.copysign(se2.STRAFE_SPEED, c['way_left']), 0.0)})


def _to_edge(g, gym, er, c, heard_fn, _say_fn):
    """Backing onto the edge, her heading bent toward the dock's line as it bears from her stage:
    bent as the dock bears from her she came to it 0.5 rad askew and sat on its edge."""
    g['has_heading'] = 1.0
    g['target_heading'] = er['hdd'] + math.atan2(c['s_perp'], er['d_stage'])
