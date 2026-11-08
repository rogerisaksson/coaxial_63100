"""Her gaits as said: a gait entry heard, laid as rows on her table from where she is.

A start from rest or a stand (init_walk, then the gait), a brake into a stand or another gait, a key pressed again
in the brake spliced into the walk at the stand's time, a stop's stand said once she has halted (`halted`), a hop
and the gait resumed after it; split from the worker.

    gaits.heard_say(g, gym, entry)          # the entry said, as rows on her table (`sprung.said`)
    gaits.start_mode(g, by, entry, at)      # a mode's rows from `at`, init_walk first where the gait takes one
    gaits.TURN_ALL                          # the turns, which take a heading
"""
import math
from machine.process import sprung
from machine.texts import ortho

TURN_GAITS = ('turn_left', 'turn_right', 'spin_left', 'spin_right')
TURN_ALL = TURN_GAITS + ('turn_back_left', 'turn_back_right')
#: She has halted after a stop's brake: slower than HALT_M_S along her way with both soles touching (`leap.TOUCH_N`);
#: her stand is said then, or at the brake's doubled horizon unmet. Each sole at a quarter of her weight instead
#: waited out the horizon: stopped, she rests 43 N on one sole and 251 on the other.
HALT_M_S = 0.25


def _sync_nominal_twist(g, name):
    from machine.law import se2
    from machine.senses import touch
    if not g.get('_twist_active') and name != 'hop':
        g['asked'] = se2.NOMINAL_TWIST.get(name, (0.0, 0.0, 0.0))
        g['twist'] = touch.against(g, g['asked'])


def _start_hop(g, by, entry, at):
    from machine.law import latent
    from machine.moves import leap
    wire, names, dur, dt_load_ms = leap.standing_leap(entry[1], g['fr'], float(g.get('_hop_charge', 0.0)))
    g['st'][latent.THETA] = ((0.10 - dt_load_ms * 0.001) % 1.0) * math.tau
    g['_jump_t0'], g['_jump_throw'], g['_jump_until'], g['_leap'] = at, at + dt_load_ms * 0.001, at + dur, False
    sprung.said(g, 'hop', wire, names, at=at, reset=True)
    if 'stand' in by:
        sprung.said(g, *by['stand'], at=at + dur + 0.01, reset=True)


def start_mode(g, by, entry, at):
    from machine.law import se2
    name = g['active_entry'] = entry[0]
    g['_brake_from'], g['_halt_at'] = None, None
    _sync_nominal_twist(g, name)
    if name == 'hop':
        return _start_hop(g, by, entry, at)
    iw = by['init_walk'][1][:ortho.ROW.size] if 'init_walk' in by else None
    skip_iw = (name in TURN_GAITS and g.get('_twist_active')) or name == 'sway' or name in se2.RESET_MODES
    use_iw = name in se2.INIT_GAITS and iw is not None and not skip_iw
    use_sway = name == 'step' and 'sway' in by
    if use_sway:
        sprung.said(g, *by['sway'], at=at)
    if use_iw:
        sprung.said(g, 'init_walk', iw, ('init_walk',), at=at, reset=True)
    dt_lead = 0.80 if use_sway else (se2.init_lead(g) if use_iw else 0.0)
    sprung.said(g, *entry, at=at + dt_lead, reset=(name in se2.RESET_MODES))


def _queue_post_jump(g, by, entry):
    from machine.law import latent, se2
    from machine.moves import leap
    name = entry[0]
    if name == 'hop':
        return leap.update_leap_charge(g, by, se2._WALK_ON_WIRE, float(g.get('_hop_charge', 0.0)))
    if name == g.get('active_entry') and any(w > g['_jump_until'] for w, _ in g['labels']):
        return
    t_res = g['_jump_until'] + 0.02
    g['tab'] = g['tab'][g['tab'][:, latent.AT] <= g['_jump_until'] + 0.01]
    g['labels'] = [(w, n) for w, n in g['labels'] if w <= g['_jump_until'] + 0.01]
    g['active_entry'] = name
    _sync_nominal_twist(g, name)
    leap_brake = name == 'stand' and g.get('_leap') and '_brake_wire' in g and 'stand' in by
    if leap_brake:
        sprung.said(g, 'brake', g['_brake_wire'], ('brake',), at=t_res - 0.01, reset=True)
        return sprung.said(g, *by['stand'], at=t_res + 0.54, reset=True)
    post = (name, se2._WALK_ON_WIRE, (name,)) if name in ('walk', 'jog') else by.get(name)
    if post is not None:
        sprung.said(g, *post, at=t_res, reset=(name == 'stand'))


def _brake_and_switch(g, by, entry, t0, spd, back_now):
    import numpy as np
    from machine.law import latent, se2
    # the gait she brakes from, for a key pressed again in the brake (`heard_say`'s splice)
    g['_brake_from'], g['_halt_at'] = g.get('active_entry'), None
    name = g['active_entry'] = entry[0]
    brake = np.frombuffer(sprung._BRAKE_BACK_WIRE if back_now else sprung._BRAKE_WIRE, latent.WIRE).copy()
    brake['blend'] = round(1e3 * se2.braked(spd))
    sprung.said(g, 'brake', brake.tobytes(), ('brake',), at=t0, reset=True)
    if name == 'stand' or name not in (sprung._FWD_GAITS + sprung._BACK_GAITS):
        ts, tg = se2.stand_lead(spd)
        if name == 'stand':
            # said once she has halted (`halted`), the clock its fallback: on the clock the stand took her at
            # 0.5-0.9 m/s after a stop re-pressed into, the trailing sole bearing 0-40 N, and she went over the
            # leading foot (the stutter's third stop, 5 of 9 starts); the capture point lies past REACH_M above
            # 1.2 m/s, so her brake takes the steps it takes
            g['_halt_at'] = (t0 + ts, t0 + tg)
            return None
        sprung.said(g, *by['stand'], at=t0 + ts, reset=True)
        return start_mode(g, by, entry, at=t0 + tg)
    g['st'][latent.NOW][latent.GAIN] = max(float(g['st'][latent.NOW][latent.GAIN]), 0.04)
    dt_b = se2.brake_lead(spd, float(name in sprung._BACK_GAITS))
    nxt = ('walk', se2._WALK_ON_WIRE, ('walk',)) if name == 'walk' else entry
    return sprung.said(g, *nxt, at=t0 + dt_b, reset=True)


def _splice_fwd(g, entry, t_end, cur_d):
    from machine.law import latent, se2
    name = g['active_entry'] = entry[0]
    g['_brake_from'], g['_halt_at'] = None, None
    t_last = min(t_end, g['t'] + 0.18) if (cur_d == 'init_walk' and name in ('jog', 'turn_left', 'turn_right')) else t_end
    g['tab'] = g['tab'][g['tab'][:, latent.AT] < t_end - 1e-4]
    g['labels'] = [(w, n) for w, n in g['labels'] if w < t_end - 1e-4]
    _sync_nominal_twist(g, name)
    nxt = ('walk', se2._WALK_ON_WIRE, ('walk',)) if name == 'walk' else entry
    sprung.said(g, *nxt, at=t_last)


def heard_say(g, gym, entry):
    from machine.law import latent, modal, se2, stride
    by, name = {e[0]: e for e in gym}, entry[0]
    if g.get('_jump_until', 0.0) > g['t']:
        return _queue_post_jump(g, by, entry)
    if name == g.get('active_entry') and (name == 'stand' or sprung.doing(g) not in ('rest', 'stand')
                                          or bool(len(g['tab']) and (g['tab'][:, latent.AT] >= g['t']).any())):
        return
    striding = (abs(float(g['out'][latent.SWAY][modal.DRIVE])) > stride.STIRS
                or abs(float(g['out'][latent.SWAY][modal.LATERAL])) > 0.08 or g['st'][latent.NOW][latent.GAIN] > 0.02)
    # her gait's direction is the one she was told, not her sway's phase: a spin's read as a backward walk
    back_now = g.get('active_entry') in sprung._BACK_GAITS
    spd, ref = math.hypot(float(g['est']['vel'][0]), float(g['est']['vel'][2])), g['out'][latent.REF]
    t0, cur_d = g['t'] + 0.001, sprung.doing(g)
    off_c = not striding and cur_d not in ('stand', 'rest') and (
        max(abs(float(ref[modal.LIFT])), abs(float(ref[modal.SAGITTAL])), abs(float(ref[modal.LATERAL]))) > 0.15
        or abs(float(ref[modal.AXIAL])) > 0.25 or abs(float(g['loads'][0]) - float(g['loads'][1])) > 90.0)
    future = g['tab'][g['tab'][:, latent.AT] > g['t']]
    # her direction through a brake is the gait she brakes from, not the stand it leads to: a `w` 0.25 s into a
    # stop's brake read `stand`, missed the splice and braked again into a walk laid cold (reset) 0.7 s on at 1 m/s
    # - her sway 0.09, then 0.06 after a second stop, 0.20 still 2.3 s later - and the third stop had nothing to brake
    # with: she stood at 1.3 m/s and fell (stutter 2 of 9 starts)
    braked = g.get('_brake_from') if cur_d == 'brake' else None
    ae_dir = braked if braked in (sprung._FWD_GAITS + sprung._BACK_GAITS) else g.get('active_entry')
    # and the gait queued after the brake (`active_entry` while she brakes): a back turn said in a brake laid for the
    # other back turn was braked again - a brake row reset, the turn 0.9 s later - for a splice (brake, brake, turn
    # for brake, turn on the page's WD, SD, SA)
    queued = g.get('active_entry') if cur_d == 'brake' else None
    same_dir = any((d in sprung._FWD_GAITS and name in sprung._FWD_GAITS) or (d in sprung._BACK_GAITS and name in sprung._BACK_GAITS)
                   for d in (ae_dir, queued))
    # the splice's time: the first row ahead, else the pending stand's earliest (`halted`)
    t_end = float(future[0, latent.AT]) if len(future) else (g['_halt_at'][0] if g.get('_halt_at') else None)
    if (cur_d in ('brake', 'stand', 'init_walk') and same_dir and t_end is not None
            and (cur_d != 'init_walk' or abs(float(g.get('twist', (0.0,))[0])) >= 0.05)):
        return _splice_fwd(g, entry, t_end, cur_d)
    hold = cur_d in se2.HOLD_PHASES and len(g['tab']) > 0
    g['tab'] = g['tab'][g['tab'][:, latent.AT] <= g['t'] + 1.5] if hold else g['tab']
    if not hold:
        sprung._dropped(g)
    t0 = ((float(g['tab'][-1, latent.AT]) if len(g['tab']) else g['t']) + 0.05) if hold else t0
    off_c = off_c or hold
    ae = g.get('active_entry')
    strafe_opp = ({name, ae} == {'strafe_left', 'strafe_right'}
                  or (((name in ('strafe_left', 'strafe_right')) != (ae in ('strafe_left', 'strafe_right'))) and spd > 0.28))
    same = cur_d not in ('brake', 'stand') and not strafe_opp and ((name in sprung._FWD_GAITS and not back_now) or (name in sprung._BACK_GAITS and back_now))
    _sync_nominal_twist(g, name)
    if name in ('parry_shove', 'controlled_fall'):
        g['active_entry'] = name
        return sprung.said(g, *entry, at=t0, reset=True)
    if (striding or g.get('_twist_active')) and name == 'hop' and 'stand' in by:
        return se2.stride_hop(g, by, t0, spd, back_now, sprung._FWD_GAITS, sprung._BACK_GAITS, sprung.said)
    if striding and 'stand' in by and not same:
        return _brake_and_switch(g, by, entry, t0, spd, back_now)
    if striding and same:
        rst = name in ('jog', 'walk') and ae in (('strafe_left', 'strafe_right') + TURN_GAITS)
        nxt = ('walk', se2._WALK_ON_WIRE, ('walk',)) if name == 'walk' else entry
        if ae != name:
            g['active_entry'] = name
            sprung.said(g, *nxt, at=t0, reset=rst)
        return
    if cur_d == 'sway' and name in (('step',) + sprung._FWD_GAITS):
        sprung.said(g, *by['step'], at=t0, reset=True)
        return sprung.said(g, *by[name], at=t0 + 0.90) if (name != 'step' and name in by) else None
    if cur_d in ('over', 'touch') and name != 'one_sole':
        sprung.said(g, 'touch', sprung._TOUCH_WIRE, ('touch',), at=t0)
        sprung.said(g, *by['stand'], at=t0 + 0.80, reset=True)
        return start_mode(g, by, entry, at=t0 + 2.00) if name != 'stand' else None
    if cur_d in se2.RISE_OF and name in (sprung._FWD_GAITS + sprung._BACK_GAITS) and se2.RISE_OF[cur_d] in by:
        rm = se2.RISE_OF[cur_d]
        sprung.said(g, *by[rm], at=t0, reset=True)
        return start_mode(g, by, entry, at=t0 + se2.rise_lead(by[rm][1]))
    if off_c and 'stand' in by and name not in (('stand', 'hop') + tuple(sprung.STUNT_MODES)):
        sprung.said(g, *by['stand'], at=t0, reset=True)
        return start_mode(g, by, entry, at=t0 + (0.55 if cur_d == 'down' else 1.35))
    return start_mode(g, by, entry, at=t0) if not striding else sprung.said(g, *entry, at=t0)


def halted(g, gym):
    """Her stand said once a stop's brake has halted her (`_brake_and_switch`): slower than HALT_M_S along her way
    with both soles touching, or at the brake's doubled horizon unmet; nothing pending, nothing."""
    from machine.moves import leap
    at = g.get('_halt_at')
    if at is None or g['t'] < at[0]:
        return
    spd = math.hypot(float(g['est']['vel'][0]), float(g['est']['vel'][2]))
    borne = min(float(g['loads'][0]), float(g['loads'][1])) > leap.TOUCH_N
    if (spd < HALT_M_S and borne) or g['t'] >= at[1]:
        g['_halt_at'] = None
        by = {e[0]: e for e in gym}
        sprung.said(g, *by['stand'], at=g['t'] + 0.001, reset=True)
