"""Her whole-body ask shaped for a hop's flight, a hit's momentum and a swing kept clear."""
import math

from machine import edge, sprung


def shape_hop_and_support(g, ask, out, cur_doing):
    from machine import flight, latent, modal, se2, wrench
    y, cur_entry = g['y'], g.get('active_entry')
    twist_mode = bool(g.get('_twist_active') or g.get('_twist_stop_until', 0.0) > g['t']
                      or 0.0 < g.get('_jump_until', 0.0) and g['t'] < g['_jump_until'] + 1.5)
    vx, vy_t, wz = g.get('twist', (0.0, 0.0, 0.0))
    tx, ty, _ = (vx, vy_t, wz) if g.get('_twist_active') else se2.NOMINAL_TWIST.get(cur_entry, (0.0, 0.0, 0.0))
    if (cur_doing in ('load', 'throw', 'land') or (cur_doing == 'rest' and g.get('_hop_until', 0.0) > g['t'])) and (
            g.get('_jump_until', 0.0) > g['t'] or cur_entry == 'hop' or g.get('_hop_until', 0.0) > g['t']):
        return flight.shape_hop(g, ask, out, cur_doing, tx, ty, (latent, modal, wrench))
    st_or_p = g['t'] < max(g.get('stumble_until', 0.0), g.get('parry_until', 0.0))
    if max(g['loads']) > 10.0 and (g.get('_upper_hit') or st_or_p):
        ask['kdot'] = -8.0 * y[modal.MOMENT]
    if (twist_mode or st_or_p) and cur_doing not in sprung.STUNT_PHASES and g.get('_jump_until', 0.0) <= g['t'] and ask.get('swing'):
        pyaw = edge.yaw(g['sense'])
        cp, sp, soles = math.cos(pyaw), -math.sin(pyaw), y[modal.SOLE].reshape(2, 3)
        for k, sw in list(ask['swing'].items()):
            other, sign = 1 - k, (1.0 if k == 0 else -1.0)
            p_sw, v_sw = sw[0].copy(), sw[1]
            lt_o = float(soles[other, 0]) * cp + float(soles[other, 2]) * sp
            lt_k = (float(p_sw[0]) + 0.05 * float(v_sw[0])) * cp + (float(p_sw[2]) + 0.05 * float(v_sw[2])) * sp
            dl = sign * (lt_k - lt_o)
            if dl < 0.115:
                adj = (0.115 - dl) * sign
                p_sw[0] += adj * cp
                p_sw[2] += adj * sp
                ask['swing'][k] = (p_sw, sw[1], sw[2], sw[3])
