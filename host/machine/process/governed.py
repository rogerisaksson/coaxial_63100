"""The robot's twist governed toward what it is asked, within its clothoid and braking envelope.

    governed.govern_twist(g, gym, dt)       # a pass: the governed twist slewed, the gait it asks heard (`gaits`)
"""
import functools
import math

from machine import libm
from machine.process import gaits, sprung


@functools.cache
def _gov_mods():
    from machine.law import rhythm, se2
    from machine.senses import touch
    from machine.way import cocoon
    return rhythm, se2, touch, cocoon


def _slew(val, goal, step):
    return val + max(-step, min(step, goal - val))


def govern_twist(g, gym, dt):
    """Step the robot's governed twist toward `asked` within its clothoid and braking envelope across
    forward and spin gaits (`se2.SE2_GAITS`), switching gait polytopes without zero-crossing chatter."""
    if not g.get('_twist_active') or g.get('_errand') is not None or g.get('_jump_until', 0.0) > g['t']:
        return
    rhythm, se2, touch, cocoon = _gov_mods()
    if gaits.in_stunt_transient(g):
        return
    rise_ae = g['active_entry'] if g.get('_post_stunt') is not None and sprung.doing(g) not in se2.RISE_OF else None
    if rise_ae is not None:
        g['active_entry'] = g.pop('_post_stunt')
    ae = g.get('active_entry')
    if sprung.doing(g) in ('brake', 'stand', 'init_walk'):
        if rise_ae is not None and not cocoon.clear_of_rise(g):
            g['_post_stunt'], g['active_entry'] = ae, rise_ae
        return
    ax, ay, az = g.get('asked', (0.0, 0.0, 0.0))
    if ae not in se2.SE2_GAITS:
        if g.get('governed') == (0.0, 0.0, 0.0) and max(abs(ax), abs(ay), abs(az)) > 0.01:
            g['governed'] = (ax, ay, az)
            g['twist'] = touch.against(g, g['governed'])
        if rise_ae is not None and not cocoon.clear_of_rise(g):
            g['_post_stunt'], g['active_entry'] = ae, rise_ae
        return
    if ae == 'jog':
        g['_jog_until'] = g['t'] + 0.5 * se2.BRAKE_S
    vx, vy, wz = g.get('governed', (ax, ay, az))
    still_wz = rhythm.R['still_rad_s']
    if abs(ay) > 0.01 or (vx == ax and vy == ay and wz == az and abs(az) <= still_wz
                          and se2.gait_for_twist(vx, vy, wz) == ae):
        if rise_ae is not None and not cocoon.clear_of_rise(g):
            g['_post_stunt'], g['active_entry'] = ae, rise_ae
        return
    prior = max(min(float(g['grip_prior'][0]), float(g['grip_prior'][1])), 1e-6)
    held = g.get('held', g['grip_prior'])
    mu = min(1.0, max(0.25, min(float(held[0]), float(held[1])) / prior))
    a_brake = se2.WALK_SPEED * se2.WALK_HZ * mu
    a_accel, dwz_wind = 2.0 * a_brake, 2.0 * se2.WALK_HZ * se2.YAW_RATE * mu
    dwz_unwind = 2.0 * dwz_wind
    h = float(g['heading'])
    vxp = max(0.0, float(g['est']['vel'][0]) * libm.sin(h) + float(g['est']['vel'][2]) * libm.cos(h))
    fast = abs(az) > still_wz and (vx > se2.MARCH_FROM or ae == 'jog' or g.get('_jog_until', 0.0) > g['t'])
    wz_goal = 0.0 if fast else az
    vx_goal = se2.WALK_SPEED if (fast or (ax > se2.WALK_SPEED and abs(wz) > still_wz)) else ax
    if (vx_goal <= rhythm.R['spin_still_m_s'] and abs(az) > still_wz
            and ae not in ('spin_left', 'spin_right') and vxp > rhythm.R['spin_fast_m_s']):
        vx_goal = rhythm.R['spin_still_m_s'] + rhythm.R['edge_m_s']
    a_vx = (a_accel if ax == 0.0 and abs(az) > still_wz else a_brake) if vx_goal < vx else a_accel
    vx = _slew(vx, vx_goal, a_vx * dt)
    vy = _slew(vy, ay, a_accel * dt)
    if wz * (wz_goal - wz) < 0.0:
        step_0 = min(abs(wz), dwz_unwind * dt)
        rem_dt = max(0.0, dt - step_0 / max(dwz_unwind, 1e-9))
        wz_mid = wz - math.copysign(step_0, wz)
        wz = _slew(wz_mid, wz_goal, dwz_wind * rem_dt) if abs(wz_mid) <= 1e-12 else wz_mid
    else:
        wz = _slew(wz, wz_goal, (dwz_unwind if abs(wz_goal) < abs(wz) else dwz_wind) * dt)
    g['governed'] = (vx, vy, wz)
    g['twist'] = touch.against(g, g['governed'])
    wz_box = (math.copysign(max(abs(wz), still_wz + 1e-4), az)
              if (abs(az) > still_wz and ae in gaits.TURN_GAITS and wz * az >= 0.0) else wz)
    target = ae if (abs(az) > still_wz and ae in gaits.TURN_GAITS and wz * az < 0.0) else se2.gait_for_twist(vx, vy, wz_box)
    if target is not None and target != ae:
        by = {e[0]: e for e in gym}
        if target in by:
            gaits.heard_say(g, gym, by[target])
    if rise_ae is not None and g.get('active_entry') not in se2.STUNT_MODES and not cocoon.clear_of_rise(g):
        g['_post_stunt'], g['active_entry'] = g['active_entry'], rise_ae
