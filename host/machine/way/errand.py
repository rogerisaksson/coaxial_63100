"""Her errands: told a verb, she walks to its thing among her things, docks and plays the stunt.

    errand.start_errand(g, gym, 'sit_chair', heard, heard_say, at)   # on her things; none, a look, a look round
    errand.step_errand(g, gym, heard, heard_say)                      # a pass: the phase's first true row taken

The phases a table (`_on_rows`): look (still while a glimpse of the kind firms up to hers, turning in
place till a thing affords the verb, a full turn at most), plan (her planner's answer awaited), scan and
check where her eyes see (stood, her eyes on her way's places in turn, then her planner told what they showed:
a way round what bars it - `machine.eyes.gaze`), walk (her way round her things to the stage, her gaze held on
her dock and led by her way's ground ahead, let go there), corner, settle, turn, side, then back and done at a seat or a
bed, out and door out of the door (`words._FACES`); the thing hers by what it affords (`machine.way.furniture`),
her dock and way her planner's (`machine.way.planner`), the stunt on it (`machine.texts.maneuver`); the
words its rows say, the thing it goes to and what her planner is asked `machine.way.words`'.
"""
import functools
import math

from machine.way.tour import STOP
from machine.way.words import (
    _arrived, _asked, _at_hand, _begun, _cornered, _ended, _faced, _fixed, _glimpsed, _halted, _held, _laid, _looking,
    _near, _nothing, _out, _passed, _ran, _reported, _retreat, _scanning, _seen, _stood, _stunt, _to_aim, _to_edge,
    _to_face, _to_line, _to_look, _told, _turning, _within,
)


def errands():
    """The verbs she walks to a thing for (`start_errand`)."""
    from machine.way import dock
    return tuple(dock.docks())


def start_errand(g, gym, sc, heard_fn, heard_say_fn, at=None):
    """An errand `sc` begun to the thing it goes to nearest `at` (x, z), her without one; nothing of the
    kind among her things, a look now (`sprung.looked`), nothing in it, begun looking round for one; one
    found out of her reach refused."""
    from machine.way import dock
    from machine.law import latent
    from machine.process import sprung
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


@functools.cache
def _on_rows():
    """The errand's phases, a row each: (on what -> the phase next, told what) in order, the first
    true row taken; and what she is told while none is - laid once, its tests and words functions of a
    pass's context, the table of none of it (laid each pass, 6.6 us of her 1 kHz). Read: looking, she stands while a glimpse
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
    from machine.law import modal, poise, se2, stride
    back = (-se2.BACK_SPEED, 0.0, 0.0)
    return {
        'look': (((lambda c, er: c['seen'] is not None and _within(er['kind'], c['seen']), 'plan', _begun),
                  (lambda c, er: c['seen'] is not None and _near(c['seen']), 'done', _at_hand),
                  (lambda c, er: c['seen'] is not None or er['swept'] >= math.tau, 'done', _ended),
                  (lambda c, er: c['glimpse'], 'look', _held)), _to_look),
        'plan': (((lambda c, er: c['answer'] is not None and c['answer'].get('laid', True) is None, 'done', _ended),
                  (lambda c, er: c['answer'] is not None and c['sees'] and not er['scanned'], 'scan', _scanning),
                  (lambda c, er: c['answer'] is not None, 'walk', _fixed)), _nothing),
        'scan': (((lambda c, er: c['scanned'], 'check', _reported),), _halted),
        'check': (((lambda c, er: c['answer'] is not None, 'walk', _fixed),), _nothing),
        'out': (((lambda c, er: c['answer'] is not None, 'door', _ran),), _nothing),
        'door': (((lambda c, er: c['door_out'], 'door', _ended),
                  (lambda c, er: c['corner'] and len(er['way']) > 1, 'door', _passed),
                  (lambda c, er: 'stops' in (c['answer'] or {}), 'door', _ran)), _to_aim),
        'walk': (((lambda c, er: c['corner'], 'corner', _stood), (lambda c, er: c['stops_in'], 'settle', _arrived),
                  (lambda c, er: 'stops' in (c['answer'] or {}), 'walk', _fixed)), _to_aim),
        'corner': (((lambda c, er: c['idle'] and c['still'], 'plan', _cornered),), _nothing),
        'turn': (((lambda c, er: abs(c['err_yaw']) <= modal.LIST_RAD, 'settle', _faced),), _to_face),
        'settle': (((lambda c, er: c['idle'] and c['s_par'] > er['d_stage'] + se2.STOP_M, 'walk', _nothing),
                   (lambda c, er: c['idle'] and c['off'] and not (c['still'] and c['even']), 'settle', _nothing),
                   (lambda c, er: c['idle'] and c['off'] and c['turns'] != 0.0, 'turn', _turning),
                   (lambda c, er: c['idle'] and c['off'] and c['away'], 'done', _stunt),
                   (lambda c, er: c['idle'] and c['off'] and c['facing'] and er['retreats'] < 1, 'clear', _retreat),
                   (lambda c, er: c['idle'] and c['off'], 'turn', _nothing),
                   (lambda c, er: c['idle'] and abs(c['s_perp']) > er['lat_tol'], 'side', _nothing),
                   (lambda c, er: c['idle'] and c['still'] and c['even'] and er['through'], 'out', _out),
                   (lambda c, er: c['idle'] and not er['through'] and c['s_par'] > poise.HALF_M + er['standoff'] + se2.BACK_STOP_M, 'back', _told(back)),
                   (lambda c, er: c['idle'] and c['still'] and not er['through'], 'done', _stunt)), _halted),
        'clear': (((lambda c, er: c['turns'] != 0.0 or c['s_par'] > er['d_stage'] + stride.STRIDE_M, 'settle', _stood),), _nothing),
        'side': (((lambda c, er: er.get('stepped') and c['s_perp_side'] * er.get('side_from', c['s_perp']) <= 0.0, 'settle', _told(STOP)),), _to_line),
        'back': (((lambda c, er: c['s_par'] <= poise.HALF_M + er['standoff'] + se2.BACK_STOP_M, 'done', _told(STOP)),), _to_edge),
        'done': (((lambda c, er: c['idle'] and c['still'] and (abs(c['err_yaw']) > modal.LIST_RAD or abs(c['s_perp']) > er['lat_tol']), 'settle', _nothing), (lambda c, er: c['idle'] and c['still'], 'done', _stunt)), _nothing),
    }


#: What a phase looks for each pass: the look's its verb's thing among hers, and a glimpse of one among
#: her registry's live slots; a scan whether her eyes have looked along her way (`gaze.scanned`); the rest nothing.
_SEEN = {'look': lambda g, e: _seen(g, e['kind'], e.get('at'), e['swept'])}
_GLIMPSE = {'look': lambda g, e: _glimpsed(g, e['kind'], e.get('at'))}
_SCANNED = {'scan': lambda g, e: _parts()[9].scanned(g)}


def _turn_way(g, e, c):
    """The way her turn in place onto the dock's heading from where she stands clears her things, the
    shorter first (`dock.turns`); 0 where neither does."""
    from machine.way import dock
    return dock.turns(g, (c['gx'], c['gz']), c['pyaw'], e['hdd'], math.copysign(1.0, c['err_yaw']))


#: Which way her turn clears her things, by phase: settled still before a turn, and backing off till
#: one does; elsewhere none asked.
_TURNS = {'settle': lambda g, e, c: float(c['idle'] and c['still'] and c['even']) and _turn_way(g, e, c), 'clear': _turn_way}


@functools.cache
def _parts():
    """What a pass of her errand reads, imported at her first errand's pass and not looked up again each."""
    from machine.way import dock, furniture, planner, route
    from machine.senses import edge
    from machine.law import latent, modal, se2, stride
    from machine.eyes import gaze, voxels
    return dock, route, edge, latent, modal, se2, stride, voxels, planner, gaze, furniture


def _watched(g, e):
    """Her way left watched by her planner (`_asked`) once WATCH_S walking, none in flight: replanned
    where her things bar it (`route.barred`) - a new way only where the old stops existing."""
    planner, furniture = _parts()[8], _parts()[10]
    due = e['phase'] in ('walk', 'door') and g['planner'].asked_at is None
    if not due or g['t'] - e.get('checked_at', -math.inf) < planner.WATCH_S:
        return
    e['checked_at'] = g['t']
    q = furniture.where(g)
    curve = [tuple(p) for p in e['curve'][e['k']:, :2]] if len(e['curve']) else []
    pts = [(float(q[0]), float(q[2]))] + curve + [tuple(p) for p in e['way'][bool(curve):]]
    _asked(g, e, 'watch', (e['xd'], e['zd']), way=pts)


def settled(y, fr, a, sway):
    """m, positive where her latent loop stands her still enough for a stunt to begin: both soles
    stand, her capture point `xi = c + v / omega` inside their hull by more than her rhythm sways
    her, her drift `v / omega` within a sole's half-length - numpy's alone on her law's arrays
    (modal's `y`, `fr`, `a`; her latent's `sway`), so compiled (`_settled`)."""
    import numpy as np
    from machine.body import figure
    from machine.law import modal, poise
    rot = fr[modal.ROT].reshape(3, 3)
    c, v = rot @ y[modal.COM], rot @ y[modal.VCOM]
    drift = v[:2] / fr[modal.OMEGA]
    xi = c[:2] + drift
    lo, hi = poise.boxes((y[modal.SOLE].reshape(2, 3) @ rot.T)[:, :2], a[modal.EDGE], y[modal.TILT])
    inside = np.minimum((xi - lo.min(axis=0)).min(), (hi.max(axis=0) - xi).min())
    swayed = np.hypot(sway[modal.DRIVE], sway[modal.LATERAL])
    return np.minimum(inside - swayed, figure.SOLE_HALF - np.hypot(drift[0], drift[1])) * a[modal.STANCE].prod()


def _settled(g):
    """`settled` on `g`'s arrays, compiled at its first read (`bodied.on_args`, `g['settled']`): numpy's
    took 50 us a read, read each pass walking (19-22 us of her errand's 84-97) and seated or lying (5-10)."""
    latent = _parts()[3]
    if g.get('settled') is None:
        from tools.cores import bodied
        g['settled'] = bodied.on_args(settled, 'settled', (0, 1, 2, 3))
    return g['settled'](g['y'], g['fr'], g['a'], g['out'][latent.SWAY])


class _Context(dict):

    """A pass's context for her errand's rows, `idle` (`_settled`) told as a row first reads it - before any word of
    the pass, the same bits: walking, at her door and turning no row reads it, told each pass it was 47 of her
    errand's 100-194 us."""

    def __init__(self, g, fields):
        super().__init__(fields)
        self.g = g

    def __missing__(self, key):
        if key != 'idle':
            raise KeyError(key)
        idle = self['idle'] = bool(_settled(self.g) > 0.0)
        return idle


def step_errand(g, gym, heard_fn, heard_say_fn):
    """Advance an active room errand (`sit_chair`, `lie_bed`, `exit_door`): her place in the dock's
    frame, her stop, her aim - her curve, her next stop, else its normal line -, what the phase looks
    for (`_SEEN`, `_GLIMPSE`), her planner's answer, the row (`_on_rows`), her way watched (`_watched`). Her floor let
    her go (`g['relocalize']`, `machine.eyes.sight.Keep.took`: her light or her room changed under her): the errand
    begun anew, its thing and its dock found from what she sees now."""
    er, relocalize = g.get('_errand'), g.pop('relocalize', False)
    if not er:
        return
    if relocalize and er['phase'] in ('scan', 'check', 'walk', 'corner'):
        start_errand(g, gym, er['kind'], heard_fn, heard_say_fn, er.get('at'))
        return
    dock, route, edge, latent, modal, se2, stride, voxels, _planner, _gaze, furniture = _parts()
    rows, meanwhile = _on_rows()[er['phase']]
    q = furniture.where(g)
    gx, gz = float(q[0]), float(q[2])
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
    rem = max(0.0, e['d_stage'] - 1.5 * stride.STRIDE_M + s_par)
    gap, closing = se2.closing(vx_w, vz_w, stop[0] - gx, stop[1] - gz)
    # her straight leg braked into an intermediate corner as her curve is into its end (`route.rounded`, the
    # wayfinder's centripetal governor): at BRAKE to her CORNER_SPEED a stride and a cell off it, where `corner` stands her;
    # at her stage (`len(e['way']) <= 1`) `corner` and `stops_in` already time her STOP_M run-out from WALK_SPEED
    off = rem if e['phase'] == 'door' else max(0.0, gap - stride.STRIDE_M - voxels.CELL_M)
    walk_pace = (min(se2.WALK_SPEED, math.sqrt(route.CORNER_SPEED ** 2 + 2.0 * route.BRAKE * off))
                 if e['phase'] == 'door' or len(e['way']) > 1 else se2.WALK_SPEED)
    door_push = e['phase'] == 'door' and len(e['way']) <= 1
    way_hd = e['hdd'] if door_push else math.atan2(stop[0] - gx, stop[1] - gz)
    aim = (gx + math.sin(way_hd), gz + math.cos(way_hd)) if door_push and s_par <= 0.0 else stop
    e['k'], (ax, az), along, _there, pace, _bend = (
        route.ahead(e['curve'], e['k'], (gx, gz)) if len(e['curve'])
        else (e['k'], aim, way_hd, pyaw, walk_pace, 0.0))
    # on her curve its stop is what is left of it, her speed along it: by the line to it, round the bed
    # 1 m off and more walked, she was told to stand mid-curve at 0.84 m/s and fell
    gap, closing = ((float(e['curve'][-1, 2] - e['curve'][e['k'], 2]) + math.hypot(float(e['curve'][e['k'], 0]) - gx, float(e['curve'][e['k'], 1]) - gz),
                     max(0.0, vx_w * math.sin(along) + vz_w * math.cos(along))) if len(e['curve']) else (gap, closing))
    ctx = _Context(g, {
        'gx': gx, 'gz': gz, 'pyaw': pyaw, 's_par': s_par, 's_perp': s_perp, 's_perp_stop': s_perp_stop, 's_perp_side': s_perp_side,
        'way_left': -e.get('side_from', s_perp) * (e['tx'] * math.cos(pyaw) - e['tz'] * math.sin(pyaw)),
        'door_out': (s_par < -min(stride.STRIDE_M, e['d_stage'] - 1.5 * stride.STRIDE_M)
                     and e['d_stage'] + s_par <= max(closing * t_stop, 1.5 * stride.STRIDE_M)),
        'stops_in': not e['way'] and (s_par_stop <= e['d_stage'] and abs(s_perp_stop) <= e['lat_tol']
                                      or math.hypot(s_par_stop - e['d_stage'], s_perp_stop) <= stride.STRIDE_M),
        # her run-out her speed's over t_stop - 0.87 m at 0.81 m/s, 1.17 at 0.92 - and a stride at least, her
        # brake's: told 0.31 m off her stage at 0.25 m/s she went on 0.5 m onto the bed; a cell early at a
        # corner, none at her stage: a cell short there she backed it after her turn
        'corner': bool(e['way']) and gap <= max(closing * t_stop, stride.STRIDE_M * float(closing > 0.0))
        + voxels.CELL_M * float(len(e['way']) > 1),
        'err_w': se2.wrap_angle(math.atan2(ax - gx, az - gz) - pyaw), 'off_w': se2.wrap_angle(along - pyaw),
        'aim_m': math.hypot(ax - gx, az - gz), 'pace': pace,
        'err_yaw': se2.wrap_angle(e['hdd'] - pyaw),
        'off': abs(se2.wrap_angle(e['hdd'] - pyaw)) > modal.LIST_RAD,
        'facing': e['nx'] * math.sin(pyaw) + e['nz'] * math.cos(pyaw) < -math.cos(dock.ARRIVALS[1]),
        'away': e['nx'] * math.sin(pyaw) + e['nz'] * math.cos(pyaw) > math.cos(dock.ARRIVALS[1]),
        'still': float(g['st'][latent.NOW][latent.GAIN]) < 1e-6,
        'even': dock.even(g, e),
        'answer': g['planner'].answer(g['t']),
        'seen': _SEEN.get(e['phase'], lambda _g, _e: None)(g, e),
        'glimpse': _GLIMPSE.get(e['phase'], lambda _g, _e: False)(g, e),
        'sees': g.get('keep') is not None, 'scanned': _SCANNED.get(e['phase'], lambda _g, _e: False)(g, e),
    })
    ctx['turns'] = _TURNS.get(e['phase'], lambda _g, _e, _c: 0.0)(g, e, ctx)
    nxt, say = next(((phase, say) for on, phase, say in rows if on(ctx, e)), (e['phase'], meanwhile))
    if nxt != e['phase'] and nxt == 'side':
        e['stepped'], e['side_from'] = False, ctx['s_perp']
    e['phase'] = nxt
    say(g, gym, e, ctx, heard_fn, heard_say_fn)
    _watched(g, e)
