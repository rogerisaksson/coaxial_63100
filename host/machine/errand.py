"""Her errands: told a verb, she walks to its thing among her things, docks and plays the stunt.

    errand.start_errand(g, gym, 'sit_chair', heard, heard_say, at)   # on her things; none, a look, a look round
    errand.step_errand(g, gym, heard, heard_say)                      # a pass: the phase's first true row taken

The phases a table (`_on_rows`): look (still while a glimpse of the kind firms up to hers, turning in
place till a thing affords the verb, a full turn at most), plan (her planner's answer awaited), walk
(her way round her things to the stage), corner, settle, turn, side, then back and done at a seat or a
bed, out and door out of the door (`_FACES`); the thing hers by what it affords (`machine.furniture`),
her dock and way her planner's (`machine.planner`), the stunt on it (`machine.maneuver`).
"""
import math

from machine.tour import STOP


def errands():
    """The verbs she walks to a thing for (`start_errand`)."""
    from machine import dock
    return tuple(dock.docks())


def _fire_errand_stunt(g, gym, kind, heard_say_fn, at=None, thing=None):
    from machine import maneuver
    g['_errand'] = None
    g['_twist_active'], g['_twist_stop_until'], g['_halt_at'], g['has_heading'] = False, 0.0, None, 0.0
    g['twist'] = g['asked'] = STOP
    by = {e[0]: e for e in gym}
    if kind in by:
        heard_say_fn(g, gym, maneuver.said(g, by[kind], at, thing))


def _seen(g, sc, at):
    """What `sc` goes to nearest `at` among her things, None with nothing of the kind among them."""
    from machine import furniture
    hit = furniture.nearest(g, sc, at)
    return None if hit[4] is None else hit


def _within(sc, hit):
    """Whether a thing found is within her reach: past a stride, within three rooms' widths; the door anywhere."""
    from machine import room, stride
    return sc == 'exit_door' or stride.STRIDE_M < hit[0] < 3.0 * room.WIDE


def _near(hit):
    """Whether a thing found is at hand: within a stride."""
    from machine import stride
    return hit[0] <= stride.STRIDE_M


def _glimpsed(g, sc, at):
    """Whether `sc` goes to a thing among her registry's live slots at any salience: a sighting not yet
    hers (`registry.TAKEN`), her look held still while it firms up."""
    from machine import furniture, registry
    return furniture.nearest(g, sc, at, things=registry.boxes(g['registry'], -math.inf))[4] is not None


#: Her heading docked off the dock's normal, by errand: through the door; her back to a seat, the rest.
_FACES = {'exit_door': math.pi}


def _asked(g, e, kind, to=None, passed=0.0, way=(), dock=None):
    """Her planner asked a `kind` (`machine.planner`: a dock of `dock` (verb, box, standoff), a plan to
    `to`, a watch of her `way` left) from where she will be at its answer, a watch's LATENCY_S on at her
    velocity and turn rate: from where she stood, a replan walking began 0.2 m behind her and she fell."""
    from machine import edge, latent, planner
    d, v, on = g['world'].data, g['est']['vel'], planner.LATENCY_S * float(kind == 'watch')
    g['planner'].ask(g['t'], {
        'kind': kind, 'at': (float(d.qpos[0]) + float(v[0]) * on, float(d.qpos[2]) + float(v[2]) * on),
        'yaw': edge.yaw(g['sense']) + float(g['sense'][edge.GYRO][1]) * on,
        'still': float(g['st'][latent.NOW][latent.GAIN]) < 1e-6, 'things': list(g['things']),
        'floor': min(float(f['sole'][1]) for f in g['s']['feet']), 'seen': None, 'slot': e['slot'], 'face': e['face'],
        'to': to, 'dock': dock, 'warm': e['held'], 'passed': passed, 'way': way})


def _laid(g, sc, found, at):
    """The errand `sc` on the thing `found` (`_seen`'s), its dock and way asked of her planner. Run at the
    door in one motion without the stage she slammed the panel to its 1.92 rad stop in 0.5 s and its
    rebound felled her outside; settled at the stage first she grinds it open over 3 s and walks out."""
    from machine import maneuver
    standoff = maneuver.standoff(sc, g['suite']) if sc in maneuver.PLAYED else 0.0
    box = found[6]
    er = dict(_looking(g, sc, at), phase='plan', through=sc in _FACES, thing=found[4], standoff=standoff,
              slot=int(box[4]) if len(box) > 4 else -1)
    _asked(g, er, 'dock', dock=(sc, box, standoff))
    return er


def _docked(sc, laid):
    """The errand's fields of its dock `laid` (`dock.docked`), her heading docked (`_FACES`)."""
    from machine import se2
    hn, (x_dock, z_dock), stage = laid['hn'], laid['edge'], laid['stage']
    nx, nz = math.sin(hn), math.cos(hn)
    return {'x_dock': x_dock, 'z_dock': z_dock, 'nx': nx, 'nz': nz, 'tx': nz, 'tz': -nx, 'face': laid['face'],
            'xd': stage[0], 'zd': stage[1], 'hdd': se2.wrap_angle(hn + _FACES.get(sc, 0.0)), 'lat_tol': laid['lat_tol'],
            'd_stage': laid['d_stage'], 'd_most': laid['d_stage']}


def _looking(g, sc, at):
    """The errand `sc` looking round for its thing: its dock where she stands, nothing planned, her yaw
    swept from none."""
    from machine import edge
    d, yaw = g['world'].data, edge.yaw(g['sense'])
    return {'kind': sc, 'phase': 'look', 'thing': None, 'at': at, 'swept': 0.0, 'yaw_was': yaw,
            'x_dock': float(d.qpos[0]), 'z_dock': float(d.qpos[2]), 'nx': math.sin(yaw), 'nz': math.cos(yaw),
            'tx': math.cos(yaw), 'tz': -math.sin(yaw), 'xd': float(d.qpos[0]), 'zd': float(d.qpos[2]), 'hdd': yaw,
            'lat_tol': 0.0, 'd_stage': 0.0, 'd_most': 0.0, 'turn_dir': 0.0, 'standoff': 0.0, 'retreats': 0,
            'way': [], 'curve': (), 'k': 0, 'held': (), 'slot': -1, 'face': None}


def start_errand(g, gym, sc, heard_fn, heard_say_fn, at=None):
    """An errand `sc` begun to the thing it goes to nearest `at` (x, z), her without one; nothing of the
    kind among her things, a look now (`sprung.looked`), nothing in it, begun looking round for one; one
    found out of her reach refused."""
    from machine import dock, latent, sprung
    if sc not in dock.docks():
        return False
    seen = _seen(g, sc, at)
    if seen is None:
        sprung.looked(g)
        seen = _seen(g, sc, at)
    if seen is not None and not _within(sc, seen):
        return False
    laid = _looking(g, sc, at) if seen is None else _laid(g, sc, seen, at)
    g['st'][latent.THETA] *= float(g['st'][latent.NOW][latent.GAIN] >= 1e-6)
    g['_errand'] = laid
    step_errand(g, gym, heard_fn, heard_say_fn)
    return True


def _tell(g, gym, heard_fn, cmd):
    """`cmd` as the errand's word to her, once: the same twist again would restart her gait."""
    if g.get('asked') != cmd.get('twist'):
        heard_fn(g, gym, dict(cmd, _errand_cmd=True))


def _told(twist):
    """A row's word: `twist`, her heading the dock's."""
    return lambda g, gym, er, c, heard_fn, _say_fn: _tell(g, gym, heard_fn, {'heading': er['hdd'], 'twist': twist})


def _begun(g, gym, er, c, heard_fn, _say_fn):
    """A thing found looking round, within reach: the errand on it (`_laid`), stood."""
    er.update(_laid(g, er['kind'], c['seen'], er.get('at')), phase=er['phase'])
    g['has_heading'] = 0.0
    _tell(g, gym, heard_fn, {'twist': STOP})


def _took(g, gym, er, c, heard_fn, _say_fn):
    """Her planner's answer: her dock where it laid one, her way."""
    a = c['answer']
    er.update(_docked(er['kind'], a['laid']) if a.get('laid') else {})
    er.update(way=a['stops'], curve=a['curve'], held=a['held'], k=0, turn_dir=0.0)


def _ran(g, gym, er, c, heard_fn, _say_fn):
    """Her planner's run out taken, no curve: the door's push is contact (`_out`)."""
    a = c['answer']
    er.update(way=a['stops'], curve=(), held=a['held'], k=0, turn_dir=0.0)


def _held(g, gym, er, c, heard_fn, _say_fn):
    """A glimpse of the kind not yet hers: stood still while it firms up."""
    _tell(g, gym, heard_fn, {'twist': STOP})


def _at_hand(g, gym, er, c, heard_fn, heard_say_fn):
    """A thing found looking round within a stride: no errand, the stunt on it where she stands, as a
    say's with one at hand (`start_errand` refused, `worker._apply_say`)."""
    _fire_errand_stunt(g, gym, er['kind'], heard_say_fn, er.get('at'), c['seen'][4])


def _to_look(g, gym, er, c, heard_fn, _say_fn):
    """Turning in place for a thing the verb goes to, her yaw's sweep summed."""
    from machine import se2
    er['swept'] += abs(se2.wrap_angle(c['pyaw'] - er['yaw_was']))
    er['yaw_was'] = c['pyaw']
    _tell(g, gym, heard_fn, {'twist': (0.0, 0.0, se2.YAW_RATE)})


def _stood(g, gym, er, c, heard_fn, _say_fn):
    """Walked to the box: stood, her heading released."""
    er['turn_dir'], g['has_heading'] = 0.0, 0.0
    _tell(g, gym, heard_fn, {'twist': STOP})


def _cornered(g, gym, er, c, heard_fn, _say_fn):
    """Stood still on her way: the rest asked of her planner, her stage last - laid inside her map's
    grown cells by its turn's own clearance (`dock.docked`), not moved for them."""
    from machine import stride
    _asked(g, er, 'plan', (er['xd'], er['zd']), stride.STRIDE_M)
    er['turn_dir'] = 0.0


def _halted(g, gym, er, c, heard_fn, _say_fn):
    """Settling: told to stand, once - laid within her run-out of her stage she may have been walking."""
    _tell(g, gym, heard_fn, {'twist': STOP})


def _faced(g, gym, er, c, heard_fn, _say_fn):
    """Faced away from the prop with the dock's heading."""
    er['turn_dir'] = 0.0
    _tell(g, gym, heard_fn, {'heading': er['hdd'], 'twist': STOP})


def _ended(g, gym, er, c, heard_fn, _say_fn):
    """Out of the door, or a look round with nothing in reach found: the errand over, stood."""
    g['_errand'], g['has_heading'] = None, 0.0
    _tell(g, gym, heard_fn, {'twist': STOP})


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


def _steer(g, gym, heard_fn, err, wz_dir, speed, off, lag):
    """Turn in place toward `err` rad while she is `off` her way past her list, standing, or past
    `route.CONE` as far as her rhythm runs (`latent.GAIN`) toward `speed`; within it, walk `speed`,
    on her arc while she `lag`s `route.ARCED` or more, else by `err`."""
    from machine import edge, latent, modal, route, se2
    w = min(1.0, float(g['st'][latent.NOW][latent.GAIN])) * float(speed > 0.0)
    turn = abs(off) > (1.0 - w) * modal.LIST_RAD + w * route.CONE
    arc = not turn and abs(lag) >= route.ARCED
    steer = float(not turn) * float(not arc)
    g['has_heading'] = steer * float(speed > 0.0)
    g['target_heading'] = se2.wrap_angle(edge.yaw(g['sense']) + err) * steer + float(g.get('target_heading', 0.0)) * (1.0 - steer)
    twists = ((turn, (0.0, 0.0, se2.YAW_RATE * wz_dir)), (arc, (speed, 0.0, math.copysign(se2.YAW_RATE, lag))), (True, (speed, 0.0, 0.0)))
    _tell(g, gym, heard_fn, {'twist': next(t for on, t in twists if on)})


def _to_aim(g, gym, er, c, heard_fn, _say_fn):
    """Walking to her aim (`step_errand`'s); her turn's way held while it is all but behind her."""
    from machine import modal, se2
    err_w, behind = c['err_w'], abs(c['err_w']) > math.pi - modal.LIST_RAD
    er['turn_dir'] = (er['turn_dir'] or math.copysign(1.0, err_w)) if behind else math.copysign(1.0, err_w)
    _steer(g, gym, heard_fn, err_w, er['turn_dir'], se2.WALK_SPEED, c['off_w'], c['lag'])


def _to_face(g, gym, er, c, heard_fn, _say_fn):
    """Turning in place to face away from the prop, the way that clears her things, held once begun."""
    er['turn_dir'] = er['turn_dir'] or math.copysign(1.0, c['err_yaw'])
    _steer(g, gym, heard_fn, c['err_yaw'], er['turn_dir'], 0.0, c['err_yaw'], 0.0)


def _turning(g, gym, er, c, heard_fn, _say_fn):
    """Still, her turn clear of her things: turning the way that clears them (`dock.turns`)."""
    er['turn_dir'] = c['turns']


def _retreat(g, gym, er, c, heard_fn, _say_fn):
    """Still facing the face where no turn onto the dock clears her things: backing off it, her heading
    as it is - her walk back stops in BACK_STOP_M."""
    from machine import se2
    er['retreats'] += 1
    g['has_heading'] = 0.0
    _tell(g, gym, heard_fn, {'twist': (-se2.BACK_SPEED, 0.0, 0.0)})


def _to_line(g, gym, er, c, heard_fn, _say_fn):
    """Sidestepping toward the dock's line, facing away from the prop."""
    from machine import se2
    g['has_heading'], g['target_heading'] = 1.0, er['hdd']
    _tell(g, gym, heard_fn, {'heading': er['hdd'], 'twist': (0.0, math.copysign(se2.STRAFE_SPEED, c['way_left']), 0.0)})


def _to_edge(g, gym, er, c, heard_fn, _say_fn):
    """Backing onto the edge, her heading bent toward the dock's line as it bears from her stage:
    bent as the dock bears from her she came to it 0.5 rad askew and sat on its edge."""
    g['has_heading'] = 1.0
    g['target_heading'] = er['hdd'] + math.atan2(c['s_perp'], er['d_stage'])


def _on_rows():
    """The errand's phases, a row each: (on what -> the phase next, told what) in order, the first
    true row taken; and what she is told while none is. Read: looking, she stands while a glimpse
    of the kind firms up to hers, else turns in place till a thing in reach affords the verb - one
    at hand, its stunt where she stands -, a full turn at most; walking, she stands where her
    stopping point
    enters the box; stood still (`idle`), she walks again past her stopping distance from it,
    turns to face her heading docked once her rhythm is still and her soles bear alike, the way that
    clears her things - none
    clearing, backs off the face once where she faces it, then turns the shorter way; facing all but
    away, takes the stunt as she stands -, sidesteps while her stop would end past the dock's tolerance off its line
    till that stop is on it, then runs out through a door, else
    backs till her heels are the stunt's standoff off the edge (`maneuver.standoff`), then the stunt
    (findings/errands.md)."""
    from machine import modal, poise, se2, stride
    back = (-se2.BACK_SPEED, 0.0, 0.0)
    return {
        'look': (((lambda c, er: c['seen'] is not None and _within(er['kind'], c['seen']), 'plan', _begun),
                  (lambda c, er: c['seen'] is not None and _near(c['seen']), 'done', _at_hand),
                  (lambda c, er: c['seen'] is not None or er['swept'] >= math.tau, 'done', _ended),
                  (lambda c, er: c['glimpse'], 'look', _held)), _to_look),
        'plan': (((lambda c, er: c['answer'] is not None and c['answer'].get('laid', True) is None, 'done', _ended),
                  (lambda c, er: c['answer'] is not None, 'walk', _took)), _nothing),
        'out': (((lambda c, er: c['answer'] is not None, 'door', _ran),), _nothing),
        'door': (((lambda c, er: -c['s_par'] > er['d_stage'] - 1.5 * stride.STRIDE_M, 'door', _ended),
                  (lambda c, er: c['corner'] and len(er['way']) > 1, 'door', _passed),
                  (lambda c, er: 'stops' in (c['answer'] or {}), 'door', _ran)), _to_aim),
        'walk': (((lambda c, er: c['corner'], 'corner', _stood), (lambda c, er: c['stops_in'], 'settle', _stood),
                  (lambda c, er: 'stops' in (c['answer'] or {}), 'walk', _took)), _to_aim),
        'corner': (((lambda c, er: c['idle'] and c['still'], 'plan', _cornered),), _nothing),
        'turn': (((lambda c, er: abs(c['err_yaw']) <= modal.LIST_RAD, 'settle', _faced),), _to_face),
        'settle': (((lambda c, er: c['idle'] and c['s_par'] > er['d_stage'] + se2.STOP_M, 'walk', _nothing),
                   (lambda c, er: c['idle'] and c['off'] and not (c['still'] and c['even']), 'settle', _nothing),
                   (lambda c, er: c['idle'] and c['off'] and c['turns'] != 0.0, 'turn', _turning),
                   (lambda c, er: c['idle'] and c['off'] and c['away'], 'done', _stunt),
                   (lambda c, er: c['idle'] and c['off'] and c['facing'] and er['retreats'] < 1, 'clear', _retreat),
                   (lambda c, er: c['idle'] and c['off'], 'turn', _nothing),
                   (lambda c, er: c['idle'] and abs(c['s_perp_side']) > er['lat_tol'], 'side', _nothing),
                   (lambda c, er: c['idle'] and er['through'], 'out', _out),
                   (lambda c, er: c['idle'] and c['s_par'] > poise.HALF_M + er['standoff'] + se2.BACK_STOP_M, 'back', _told(back)),
                   (lambda c, er: c['idle'] and c['still'], 'done', _stunt)), _halted),
        'clear': (((lambda c, er: c['turns'] != 0.0 or c['s_par'] > er['d_stage'] + stride.STRIDE_M, 'settle', _stood),), _nothing),
        'side': (((lambda c, er: c['s_perp_side'] * c['s_perp'] <= 0.0, 'settle', _told(STOP)),), _to_line),
        'back': (((lambda c, er: c['s_par'] <= poise.HALF_M + er['standoff'] + se2.BACK_STOP_M, 'done', _told(STOP)),), _to_edge),
        'done': (((lambda c, er: c['idle'] and (abs(c['err_yaw']) > modal.LIST_RAD or abs(c['s_perp_side']) > er['lat_tol']), 'settle', _nothing), (lambda c, er: c['idle'] and c['still'], 'done', _stunt)), _nothing),
    }


#: What a phase looks for each pass: the look's its verb's thing among hers, and a glimpse of one among
#: her registry's live slots; the rest nothing.
_SEEN = {'look': lambda g, e: _seen(g, e['kind'], e.get('at'))}
_GLIMPSE = {'look': lambda g, e: _glimpsed(g, e['kind'], e.get('at'))}


def _turn_way(g, e, c):
    """The way her turn in place onto the dock's heading from where she stands clears her things, the
    shorter first (`dock.turns`); 0 where neither does."""
    from machine import dock
    return dock.turns(g, (c['gx'], c['gz']), c['pyaw'], e['hdd'], math.copysign(1.0, c['err_yaw']))


#: Which way her turn clears her things, by phase: settled still before a turn, and backing off till
#: one does; elsewhere none asked.
_TURNS = {'settle': lambda g, e, c: float(c['idle'] and c['still'] and c['even']) and _turn_way(g, e, c), 'clear': _turn_way}


def _watched(g, e):
    """Her way left watched by her planner (`_asked`) once WATCH_S walking, none in flight: replanned
    where her things bar it (`route.barred`) - a new way only where the old stops existing."""
    from machine import planner
    due = e['phase'] in ('walk', 'door') and g['planner'].asked_at is None
    if not due or g['t'] - e.get('checked_at', -math.inf) < planner.WATCH_S:
        return
    e['checked_at'] = g['t']
    d = g['world'].data
    curve = [tuple(p) for p in e['curve'][e['k']:, :2]] if len(e['curve']) else []
    pts = [(float(d.qpos[0]), float(d.qpos[2]))] + curve + [tuple(p) for p in e['way'][bool(curve):]]
    _asked(g, e, 'watch', (e['xd'], e['zd']), way=pts)


def _settled(g):
    """m, positive where her latent loop stands her still enough for a stunt to begin: both soles
    stand, her capture point `xi = c + v / omega` inside their hull by more than her rhythm sways
    her, her drift `v / omega` within a sole's half-length."""
    from machine import figure, latent, modal, poise
    y, fr, a, sway = g['y'], g['fr'], g['a'], g['out'][latent.SWAY]
    rot = fr[modal.ROT].reshape(3, 3)
    c, v = rot @ y[modal.COM], rot @ y[modal.VCOM]
    drift = v[:2] / float(fr[modal.OMEGA])
    xi = c[:2] + drift
    lo, hi = poise.boxes((y[modal.SOLE].reshape(2, 3) @ rot.T)[:, :2], a[modal.EDGE], y[modal.TILT])
    inside = min(float((xi - lo.min(axis=0)).min()), float((hi.max(axis=0) - xi).min()))
    swayed = math.hypot(float(sway[modal.DRIVE]), float(sway[modal.LATERAL]))
    return min(inside - swayed, figure.SOLE_HALF - math.hypot(*drift)) * float(a[modal.STANCE].prod())


def step_errand(g, gym, heard_fn, heard_say_fn):
    """Advance an active room errand (`sit_chair`, `lie_bed`, `exit_door`): her place in the dock's
    frame, her stop, her aim - her curve, her next stop, else its normal line -, what the phase looks
    for (`_SEEN`, `_GLIMPSE`), her planner's answer, the row (`_on_rows`), her way watched (`_watched`)."""
    from machine import dock, edge, latent, modal, route, se2, stride, voxels
    er = g.get('_errand')
    if not er:
        return
    rows, meanwhile = _on_rows()[er['phase']]
    d = g['world'].data
    gx, gz = float(d.qpos[0]), float(d.qpos[2])
    vx_w, vz_w = float(g['est']['vel'][0]), float(g['est']['vel'][2])
    pyaw = se2.wrap_angle(edge.yaw(g['sense']))
    e = er
    dx_d, dz_d = gx - e['x_dock'], gz - e['z_dock']
    s_par, s_perp = dx_d * e['nx'] + dz_d * e['nz'], dx_d * e['tx'] + dz_d * e['tz']
    t_stop = se2.STOP_M / se2.WALK_SPEED
    s_par_stop = s_par + (vx_w * e['nx'] + vz_w * e['nz']) * t_stop
    s_perp_stop = s_perp + (vx_w * e['tx'] + vz_w * e['tz']) * t_stop
    s_perp_side = s_perp + (vx_w * e['tx'] + vz_w * e['tz']) * se2.SIDE_STOP_M / se2.STRAFE_SPEED
    a_par = max(s_par - se2.STOP_M, e['d_stage'])
    stop = e['way'][0] if e['way'] else (e['x_dock'] + a_par * e['nx'], e['z_dock'] + a_par * e['nz'])
    e['k'], (ax, az), along, there = (route.ahead(e['curve'], e['k'], (gx, gz)) if len(e['curve'])
                                      else (e['k'], stop, math.atan2(stop[0] - gx, stop[1] - gz), pyaw))
    gap, closing = se2.closing(vx_w, vz_w, stop[0] - gx, stop[1] - gz)
    ctx = {
        'gx': gx, 'gz': gz, 'pyaw': pyaw, 's_par': s_par, 's_perp': s_perp, 's_perp_stop': s_perp_stop, 's_perp_side': s_perp_side,
        'way_left': -s_perp * (e['tx'] * math.cos(pyaw) - e['tz'] * math.sin(pyaw)),
        'stops_in': not e['way'] and (s_par_stop <= e['d_stage'] and abs(s_perp_stop) <= e['lat_tol']
                                      or math.hypot(s_par_stop - e['d_stage'], s_perp_stop) <= stride.STRIDE_M),
        # her run-out her speed's over t_stop - 0.87 m at 0.81 m/s, 1.17 at 0.92 - and a stride at least, her
        # brake's: told 0.31 m off her stage at 0.25 m/s she went on 0.5 m onto the bed; a cell early at a
        # corner, none at her stage: a cell short there she backed it after her turn
        'corner': bool(e['way']) and gap <= max(closing * t_stop, stride.STRIDE_M * float(closing > 0.0))
        + voxels.CELL_M * float(len(e['way']) > 1),
        'err_w': se2.wrap_angle(math.atan2(ax - gx, az - gz) - pyaw), 'off_w': se2.wrap_angle(along - pyaw),
        'lag': se2.wrap_angle(there - pyaw),
        'err_yaw': se2.wrap_angle(e['hdd'] - pyaw),
        'off': abs(se2.wrap_angle(e['hdd'] - pyaw)) > modal.LIST_RAD,
        'facing': e['nx'] * math.sin(pyaw) + e['nz'] * math.cos(pyaw) < -math.cos(dock.ARRIVALS[1]),
        'away': e['nx'] * math.sin(pyaw) + e['nz'] * math.cos(pyaw) > math.cos(dock.ARRIVALS[1]),
        'idle': _settled(g) > 0.0,
        'still': float(g['st'][latent.NOW][latent.GAIN]) < 1e-6,
        'even': dock.even(g, e),
        'answer': g['planner'].answer(g['t']),
        'seen': _SEEN.get(e['phase'], lambda _g, _e: None)(g, e),
        'glimpse': _GLIMPSE.get(e['phase'], lambda _g, _e: False)(g, e),
    }
    ctx['turns'] = _TURNS.get(e['phase'], lambda _g, _e, _c: 0.0)(g, e, ctx)
    nxt, say = next(((phase, say) for on, phase, say in rows if on(ctx, e)), (e['phase'], meanwhile))
    e['phase'] = nxt
    say(g, gym, e, ctx, heard_fn, heard_say_fn)
    _watched(g, e)
