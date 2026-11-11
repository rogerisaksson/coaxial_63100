"""Her rhythm under a twist told: her gait's drive, turn and arms, and her rhythm's rate.

    rhythm.modulated(st, out, held, prior, state, tw_on, vx, vy, wz, vxp)    # st, out and state in place

Her twist's step (`se2.twisted`) modulates her rhythm so. Her rhythm's numbers are her build's (`[gait.rhythm]`, Froude-similar on another leg, `machine.body.gait`),
laid in NAMES's order in her step's C twin (wbc_twist_t, wbc_twist.c).
"""
import math

from machine.body import gait

#: Her rhythm's numbers in the order wbc_twist_t lays them.
NAMES = ('moving', 'moving_edge', 'gain_on', 'gain_edge', 'rest_lo', 'rest_hi', 'edge_m_s', 'still_rad_s',
         'edge_rad_s', 'rest_drive', 'side_m_s', 'turn_rad_s', 'turn_edge_rad_s', 'spin_rad_s', 'spin_still_m_s',
         'backing_m_s', 'turn_dc', 'turn_dc_side', 'turn_blend', 'turn_stride_blend', 'turning', 'turning_edge',
         'spin_fast_m_s', 'spin_fast_edge_m_s', 'spin_drive', 'spin_drive_s_m', 'spin_drive_from_m_s', 'spin_lift',
         'turn_sway', 'slip_grip', 'slip_grip_edge', 'slip_m_s', 'slip_hz', 'brisk_hz', 'quick_hz', 'run_drive',
         'run_boost', 'run_arms', 'back_drive', 'back_boost', 'side_shift')
R = gait.table('rhythm')


def modulated(st, out, held, prior, state, tw_on, vx, vy, wz, vxp):
    """Her gait's rhythm, drive, turn and arms under the twist told (`vx`, `vy`, `wz`; `vxp` her own
    speed along), `tw_on` while one is: her latent `st` and its `out`, her twist's `state`, her soles'
    grip `held` and her `prior` of it."""
    from machine.law import latent, modal, se2
    r = R
    gain = float(st[latent.NOW][latent.GAIN])
    amp_d = abs(float(st[latent.TO][latent.AMP][modal.DRIVE]))
    w_gait = min(1.0, max(0.0, (max(amp_d, abs(vy), abs(wz)) - r['moving']) / r['moving_edge']))
    w_on = tw_on * min(1.0, max(0.0, (gain - r['gain_on']) / r['gain_edge'])) * w_gait
    w_rest = tw_on * (1.0 - min(1.0, max(0.0, (gain - r['gain_on']) / r['gain_edge']))) * float(r['rest_lo'] < amp_d < r['rest_hi'])
    w_fast_fwd = (min(1.0, max(0.0, (vx - se2.WALK_SPEED) / r['edge_m_s']))
                  * min(1.0, max(0.0, (r['still_rad_s'] - abs(wz)) / r['edge_rad_s']))
                  * min(1.0, max(0.0, (r['edge_m_s'] - abs(vy)) / r['edge_m_s'])))
    st[latent.TO][latent.AMP][modal.DRIVE] += w_rest * w_fast_fwd * max(
        0.0, r['rest_drive'] - float(st[latent.TO][latent.AMP][modal.DRIVE]))
    w_vy = min(1.0, max(0.0, (abs(vy) - r['side_m_s']) / r['edge_m_s']))
    w_wz = min(1.0, max(0.0, (abs(wz) - r['turn_rad_s']) / r['turn_edge_rad_s']))
    w_wz5 = min(1.0, max(0.0, (abs(wz) - r['spin_rad_s']) / r['edge_rad_s']))
    w_spin = w_on * w_wz5 * min(1.0, max(0.0, (r['spin_still_m_s'] - abs(vx)) / r['edge_m_s'])) * (1.0 - w_vy)
    w_stride = w_on * min(1.0, max(0.0, (abs(vxp) - r['spin_still_m_s']) / r['edge_m_s'])) * (1.0 - w_vy)
    wz_way = (1.0 - 2.0 * float(vx < -r['backing_m_s'])) * wz
    ts_raw = w_on * w_wz * (r['turn_dc'] - r['turn_dc_side'] * w_vy) * max(-1.0, min(1.0, wz_way / se2.YAW_RATE))
    tb = r['turn_blend'] + w_stride * (r['turn_stride_blend'] - r['turn_blend'])
    ts = state[se2.TURN_DC] = float(state[se2.TURN_DC]) + tb * (ts_raw - float(state[se2.TURN_DC]))
    w_turn = w_on * max(w_wz, min(1.0, max(0.0, (abs(ts) - r['turning']) / r['turning_edge'])))
    bt_d, w = float(st[latent.THETA] + st[latent.NOW][latent.PHASE][modal.DRIVE]), float(out[latent.TURNS])
    w_vxp = min(1.0, max(0.0, (abs(vxp) - r['spin_fast_m_s']) / r['spin_fast_edge_m_s']))
    drv = (1.0 - w_vxp) * r['spin_drive'] + w_vxp * max(r['spin_drive'], r['spin_drive_s_m'] * (abs(vxp) - r['spin_drive_from_m_s']))
    # told on slower than her walk, past the spot's stillness, her stride is her speed's share of her walk's at her
    # cadence, her drive's sway cut so: kept, told 0.2-0.5 m/s she walked 0.68-0.73 and turned 1.41 m round - her
    # arc is her speed over her turn
    w_short = w_on * (1.0 - min(1.0, max(0.0, (r['spin_still_m_s'] - vx) / r['edge_m_s']))) * max(0.0, 1.0 - vx / se2.WALK_SPEED)
    sw = float(out[latent.SWAY][modal.DRIVE])
    out[latent.REF][modal.DRIVE] -= w_short * sw * math.sin(bt_d)
    out[latent.RATE][modal.DRIVE] -= w_short * sw * w * math.cos(bt_d)
    out[latent.ACC][modal.DRIVE] += w_short * sw * w * w * math.sin(bt_d)
    out[latent.SWAY][modal.DRIVE] -= w_short * sw
    out[latent.SWAY][modal.DRIVE] += w_spin * (drv - float(out[latent.SWAY][modal.DRIVE]))
    out[latent.REF][modal.DRIVE] += w_spin * (drv * math.sin(bt_d) - float(out[latent.REF][modal.DRIVE]))
    out[latent.RATE][modal.DRIVE] += w_spin * (drv * w * math.cos(bt_d) - float(out[latent.RATE][modal.DRIVE]))
    out[latent.ACC][modal.DRIVE] += w_spin * (-drv * w * w * math.sin(bt_d) - float(out[latent.ACC][modal.DRIVE]))
    out[latent.SWAY][modal.LIFT] += w_spin * max(0.0, r['spin_lift'] - float(out[latent.SWAY][modal.LIFT]))
    amp_t = w_vy * abs(ts) + (1.0 - w_vy) * r['turn_sway']
    bt = float(st[latent.THETA] + st[latent.NOW][latent.PHASE][modal.TURN])
    out[latent.SWAY][modal.TURN] += w_turn * (amp_t - float(out[latent.SWAY][modal.TURN]))
    out[latent.REF][modal.TURN] += w_turn * (ts + amp_t * math.sin(bt) - float(out[latent.REF][modal.TURN]))
    out[latent.RATE][modal.TURN] += w_turn * (amp_t * w * math.cos(bt) - float(out[latent.RATE][modal.TURN]))
    out[latent.ACC][modal.TURN] += w_turn * (-amp_t * w * w * math.sin(bt) - float(out[latent.ACC][modal.TURN]))
    w_slip = (min(1.0, max(0.0, (r['slip_grip'] - min(held)) / r['slip_grip_edge']))
              * min(1.0, max(0.0, (min(prior) - r['slip_grip']) / r['slip_grip_edge']))
              * min(1.0, max(0.0, (abs(vx) - r['slip_m_s']) / r['edge_m_s'])))
    w_vx_hi = min(1.0, max(0.0, (abs(vx) - se2.WALK_SPEED) / r['edge_m_s']))
    w_vy_hi = min(1.0, max(0.0, (abs(vy) - se2.STRAFE_SPEED) / r['edge_m_s']))
    w_back_hi = min(1.0, max(0.0, (-vx - se2.BACK_SPEED) / r['edge_m_s'])) * min(1.0, max(0.0, (r['edge_m_s'] - abs(vy)) / r['edge_m_s']))
    w_wz_hi = min(1.0, max(0.0, (abs(wz) - se2.YAW_RATE) / r['edge_rad_s']))
    w_hz = w_on * max(w_slip, w_vx_hi, w_vy_hi, w_back_hi, w_wz5)
    hz_m = w_on * max(r['slip_hz'] * w_slip, r['brisk_hz'] * max(w_spin, w_wz_hi, w_vx_hi), r['quick_hz'] * w_hz)
    st[latent.TO][latent.HZ] = max(float(st[latent.TO][latent.HZ]), hz_m)
    st[latent.NOW][latent.HZ] = max(float(st[latent.NOW][latent.HZ]), hz_m)
    out[latent.TURNS] += w_hz * (math.tau * float(st[latent.NOW][latent.HZ]) - float(out[latent.TURNS]))
    w_boost_fwd = w_hz * w_fast_fwd
    out[latent.SWAY][modal.DRIVE] += w_boost_fwd * (min(r['run_drive'], float(out[latent.SWAY][modal.DRIVE]) * r['run_boost']) - float(out[latent.SWAY][modal.DRIVE]))
    out[latent.SWAY][modal.ARMS] += w_boost_fwd * (min(r['run_arms'], float(out[latent.SWAY][modal.ARMS]) * r['run_boost']) - float(out[latent.SWAY][modal.ARMS]))
    w_boost_back = w_hz * w_back_hi * (1.0 - w_boost_fwd)
    out[latent.SWAY][modal.DRIVE] += w_boost_back * (min(r['back_drive'], float(out[latent.SWAY][modal.DRIVE]) * r['back_boost']) - float(out[latent.SWAY][modal.DRIVE]))
    out[latent.REF][modal.LATERAL] += (w_hz * w_vy_hi) * (math.copysign(r['side_shift'], vy) - float(out[latent.REF][modal.LATERAL]))
