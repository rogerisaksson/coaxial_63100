"""Her leaps on the stack: which soles stand, told from this pass alone.

    ask = leap.step(ask, loads, s, hips)          # between a motion pass's ask and `wbc.step`

The stack lays a force on every sole it is asked to stand on; a floor that is not there gives
none. A sole the ask means to stand stands as it bears TOUCH_N; one that bears nothing stands
lying within TOUCH_M of the floor beside one that bears, or with her centre of mass at rest. One
that does not stand is asked to its landing: under her capture point as she comes down, as far
off it as it is off the others, drawn in to what its leg reaches (`reach.land`), its hip
LAND_DROP_M under the leg's length over it, coming down with her. With no sole standing no turn
is asked, the stack lays no force, and her centre of mass comes down: `wbc.step` and the loop in C
need nothing more. A limb is an index, as `reach.LIMBS`; nothing is kept from one pass to the
next.
"""
import math

from machine import reach
from machine.figure import ANKLE_H, BALL, HEEL

G = 9.81

#: A sole bears at TOUCH_N, lies on the floor within TOUCH_M; she rests under REST_M_S.
#: Near alone, the stack pressed a sole 1 mm over the floor onto it as she left: 648 N on it,
#: 23 deg over. All or none while she rises flickered on one sole: 21 deg over three hops.
TOUCH_N, TOUCH_M, REST_M_S = 15.0, 0.002, 0.1

#: A sole in the air waits for the floor with its hip LAND_DROP_M under the leg's length over
#: it - the knee 25 deg -, no straighter than LAND_SHORT_M short of that length where drawn.
LAND_DROP_M, LAND_SHORT_M = 0.018, 0.004

#: The sole's middle ahead of its ankle, m.
MIDDLE = (BALL - HEEL) / 2.0


def standing(meant, loads, over, rising, borne=0.0):
    """Which soles stand, a bool a sole: `meant` the ask's stance, `loads` the soles' N, `over`
    each sole's height over the floor, `rising` her centre of mass's speed up, `borne` what bears
    her off them, N."""
    rests = (max(loads) >= TOUCH_N) | (borne >= TOUCH_N) | (abs(rising) <= REST_M_S)
    return tuple(bool(m) & bool((n >= TOUCH_N) | ((h <= TOUCH_M) & rests))
                 for m, n, h in zip(meant, loads, over))


def landing(limb, hip, heading, yaw, com, vcom, off, target=(0.0, 0.0), pitch=0.0):
    """(sole's middle, speed, acceleration, quaternion), the stack's swing ask for a sole in the
    air: under her capture point as she lands - from her hip the same whenever that is -, `off`
    it (x, z), drawn in to the leg's reach from `hip`, for `target` (height, distance) and `pitch`."""
    dh, dz = target
    drop = reach.THIGH + reach.SHANK - LAND_DROP_M - dh
    omega = math.sqrt(G / max(0.3, com[1] - (hip[1] - drop - ANKLE_H)))
    c, s = math.cos(yaw), math.sin(yaw)
    want = ((com[0] + vcom[0] / omega + off[0] + (dz - MIDDLE) * s, hip[1] - drop,
             com[2] + vcom[2] / omega + off[1] + (dz - MIDDLE) * c), yaw)
    (ankle, yaw), _m = reach.land(limb, hip, heading, want, LAND_SHORT_M, pitch)
    c, s = math.cos(yaw), math.sin(yaw)
    cy, sy, cp, sp = math.cos(0.5 * yaw), math.sin(0.5 * yaw), math.cos(0.5 * pitch), math.sin(0.5 * pitch)
    return ((ankle[0] + MIDDLE * s, ankle[1] - ANKLE_H, ankle[2] + MIDDLE * c),
            (vcom[0], vcom[1], vcom[2]), (0.0, -G, 0.0),
            (cy * cp, cy * sp, sy * cp, -sy * sp))


def step(ask, loads, s, hips, floor=0.0):
    """`ask` as the stack is to be asked it: the soles it means to stand that do not touch asked
    to their landing; with none standing, no turn. `s` is `wbc.sense`'s, `hips` each limb's hip
    joint, world, `floor` the floor's height under her."""
    meant = ask['stance']
    feet = s['feet']
    over = [float(f['pts'][:, 1].min()) - floor for f in feet]
    stance = standing(meant, loads, over, float(s['vcom'][1]))
    count = sum(bool(m) for m in meant)
    mid = [sum(float(f['sole'][i]) * bool(m) for f, m in zip(feet, meant)) / max(1, count) for i in (0, 2)]
    w, x, y, z = (float(v) for v in s['turns'][0]['quat'])
    heading = math.atan2(2.0 * (x * z + w * y), 1.0 - 2.0 * (x * x + y * y))
    swing = dict(ask.get('swing', {}))
    swing.update({k: landing(k, hips[k], heading, math.atan2(feet[k]['R'][0][2], feet[k]['R'][2][2]),
                             s['com'], s['vcom'], (float(feet[k]['sole'][0]) - mid[0],
                                                   float(feet[k]['sole'][2]) - mid[1]))
                  for k in (0, 1) if meant[k] and not stance[k]})
    # with no sole standing there is nothing to turn her against: asked, even as they were,
    # the stack spun her legs to hold her trunk still - the knees out at 3 rad/s with no
    # torque in them, on their stops as she landed
    return dict(ask, stance=stance, swing=swing, turns=ask['turns'][:2 * any(stance)])


#: Space hold window for progressive jump/leap charge: 5 ms tap (0.0) to 250 ms full hold (1.0).
HOP_MIN_S, HOP_MAX_S = 0.005, 0.250


def hop_charge(dt_s):
    """Map Space hold duration `dt_s` in [HOP_MIN_S, HOP_MAX_S] (5 ms to 250 ms) to [0.0, 1.0]."""
    return max(0.0, min(1.0, (float(dt_s) - HOP_MIN_S) / (HOP_MAX_S - HOP_MIN_S)))


def standing_leap(base_wire, ch=0.0):
    """Construct the 11D modal trajectory table (`latent.WIRE`) for a standing vertical two-footed high jump parameterized by `ch` in [0, 1]."""
    import numpy as np
    from machine import latent, modal, ortho
    ch = max(0.0, min(1.0, float(ch)))
    rb = np.frombuffer(base_wire, latent.WIRE).copy()
    dt_load_ms = int(round(420 + 180 * ch))
    dt_throw_ms = int(round(120 + 55 * ch))
    dt_land_ms = 650
    rb['at'][0], rb['blend'][0] = 0, int(round(0.75 * dt_load_ms))
    rb['at'][1], rb['blend'][1] = dt_load_ms, 0
    rb['at'][2], rb['blend'][2] = dt_load_ms + dt_throw_ms, 80
    if len(rb) > 3:
        rb['at'][3], rb['blend'][3] = dt_load_ms + dt_throw_ms + dt_land_ms, ortho.REST_MS
    p0 = rb['p'][0]
    p0[latent.Z.start + modal.AXIAL] = 0.58 + 0.48 * ch
    p0[latent.Z.start + modal.SAGITTAL] = -0.10
    p0[latent.Z.start + modal.PITCH] = 0.0
    p0[latent.Z.start + modal.ARMS] = -0.40 * ch
    p0[latent.K.start + modal.AXIAL], p0[latent.D.start + modal.AXIAL] = 6000.0, 900.0
    p0[latent.K.start + modal.SAGITTAL], p0[latent.D.start + modal.SAGITTAL] = 1200.0, 400.0
    p0[latent.ZMP], p0[latent.GAIN] = modal.SUPPORT, 0.0
    p1 = rb['p'][1]
    p1[latent.Z.start + modal.AXIAL] = 0.20 - 0.22 * ch
    p1[latent.K.start + modal.AXIAL], p1[latent.D.start + modal.AXIAL] = 18000.0 + 11000.0 * ch, 0.0
    p1[latent.Z.start + modal.SAGITTAL] = 0.0
    p1[latent.K.start + modal.SAGITTAL], p1[latent.D.start + modal.SAGITTAL] = 1200.0, 400.0
    p1[latent.Z.start + modal.PITCH] = 0.0
    p1[latent.K.start + modal.PITCH], p1[latent.D.start + modal.PITCH] = 0.0, 200.0
    p1[latent.Z.start + modal.ARMS], p1[latent.ZMP] = 0.70 * ch, modal.NONE
    p2 = rb['p'][2]
    p2[latent.Z.start + modal.AXIAL], p2[latent.Z.start + modal.SAGITTAL] = 0.36 + 0.49 * ch, 0.0
    p2[latent.Z.start + modal.PITCH], p2[latent.Z.start + modal.ARMS] = 0.0, 0.0
    p2[latent.K.start + modal.AXIAL], p2[latent.D.start + modal.AXIAL] = 3000.0, 600.0
    p2[latent.K.start + modal.SAGITTAL], p2[latent.D.start + modal.SAGITTAL] = 600.0, 300.0
    p2[latent.K.start + modal.PITCH], p2[latent.D.start + modal.PITCH] = 1458.0, 292.0
    p2[latent.K.start + modal.LIFT], p2[latent.D.start + modal.LIFT] = 2400.0 + 3000.0 * ch, 320.0
    p2[latent.ZMP] = modal.SUPPORT
    dur = (dt_load_ms + dt_throw_ms + dt_land_ms) * 0.001
    return rb.tobytes(), ('load', 'throw', 'land', 'rest'), dur, dt_load_ms


def stride_leap(base_wire, walk_wire, ch=0.0, spd=0.69):
    """Construct the 11D modal trajectory table (`latent.WIRE`) for an in-stride forward bound parameterized by `ch` in [0, 1] and approach speed `spd`."""
    import numpy as np
    from machine import latent, modal, ortho
    ch = max(0.0, min(1.0, float(ch)))
    w_fast = max(0.0, min(1.0, (float(spd) - 0.55) / 0.20))
    rb = np.frombuffer(base_wire, latent.WIRE).copy()
    s = 0.06 + 0.28 * ch + 0.08 * w_fast
    u = 0.55 * ch + 0.15 * w_fast
    dt_load_ms = int(round(440 + 140 * ch))
    dt_throw_ms = int(round(146 + 21 * ch))
    dt_land_ms = 400
    rb['at'][0], rb['blend'][0] = 0, int(round(0.75 * dt_load_ms))
    rb['at'][1], rb['blend'][1] = dt_load_ms, 0
    rb['at'][2], rb['blend'][2] = dt_load_ms + dt_throw_ms, 0
    if len(rb) > 3:
        rb['at'][3], rb['blend'][3] = dt_load_ms + dt_throw_ms + dt_land_ms, ortho.REST_MS
    p0 = rb['p'][0]
    p0[latent.Z.start + modal.AXIAL] = 0.66 + 0.24 * s
    p0[latent.Z.start + modal.SAGITTAL] = 0.14 + 0.54 * s
    p0[latent.Z.start + modal.PITCH] = -0.05 + 0.14 * ch
    p0[latent.Z.start + modal.ARMS] = -0.35 - 0.45 * s
    p0[latent.K.start + modal.AXIAL], p0[latent.D.start + modal.AXIAL] = 6000.0, 900.0
    p0[latent.K.start + modal.SAGITTAL], p0[latent.D.start + modal.SAGITTAL] = 1200.0, 400.0
    p0[latent.ZMP], p0[latent.GAIN] = modal.SUPPORT, 0.0
    p1 = rb['p'][1]
    p1[latent.Z.start + modal.AXIAL] = 0.15
    p1[latent.K.start + modal.AXIAL], p1[latent.D.start + modal.AXIAL] = 21600.0 + 3400.0 * s, 0.0
    p1[latent.Z.start + modal.SAGITTAL] = 0.28 + 0.60 * s
    p1[latent.K.start + modal.SAGITTAL], p1[latent.D.start + modal.SAGITTAL] = 1800.0 + 1000.0 * s, 200.0
    p1[latent.Z.start + modal.PITCH] = -0.05 + 0.14 * ch
    p1[latent.K.start + modal.PITCH], p1[latent.D.start + modal.PITCH] = 1458.0, 292.0
    p1[latent.Z.start + modal.ARMS], p1[latent.ZMP] = 0.45 + 0.45 * s, modal.NONE
    p1[latent.AMP.start + modal.DRIVE], p1[latent.AMP.start + modal.LIFT] = 0.64 + 0.24 * u, 1.20 + 0.40 * u
    p1[latent.GAIN] = 0.0
    p2 = rb['p'][2]
    p2[latent.Z.start + modal.AXIAL], p2[latent.Z.start + modal.SAGITTAL] = 0.25, -0.24 - 0.10 * ch
    p2[latent.Z.start + modal.PITCH], p2[latent.Z.start + modal.ARMS] = 0.0, 0.0
    p2[latent.K.start + modal.AXIAL], p2[latent.D.start + modal.AXIAL] = 3000.0, 600.0
    p2[latent.K.start + modal.SAGITTAL], p2[latent.D.start + modal.SAGITTAL] = 600.0, 300.0
    p2[latent.K.start + modal.PITCH], p2[latent.D.start + modal.PITCH] = 1458.0, 292.0
    p2[latent.ZMP], p2[latent.GAIN] = modal.SUPPORT, 0.0
    dur = (dt_load_ms + dt_throw_ms + dt_land_ms) * 0.001
    return rb.tobytes(), ('load', 'throw', 'land', 'rest'), dur, dt_load_ms


def update_leap_charge(g, by, walk_on_wire, ch):
    """Stretch and scale the active leap trajectory in `g['tab']` while in the `load` phase."""
    import numpy as np
    from machine import latent
    t_throw, t0 = g.get('_jump_throw', 0.0), g.get('_jump_t0', 0.0)
    if g['t'] >= t_throw - 0.015 or 'hop' not in by or t0 <= 0.0:
        return
    in_stride = bool(g.get('_leap'))
    spd = float(g.get('_leap_spd', 0.69))
    wire, names, dur, dt_load_ms = (stride_leap(by['hop'][1], walk_on_wire, ch, spd) if in_stride
                                    else standing_leap(by['hop'][1], ch))
    new = latent.table(wire, t0, g['rest'])
    new[0, latent.VALS] = np.where(new[0, latent.SAID] > 0.5, new[0, latent.VALS], g['rest'])
    new[0, latent.SAID] = 1.0
    if g['t'] >= t0 - 1e-4:
        g['st'][latent.TO] = np.where(new[0, latent.SAID] > 0.5, new[0, latent.VALS], g['st'][latent.TO])
        g['st'][latent.BLEND] = new[0, latent.LERP]
        g['st'][latent.THETA] = ((0.10 - (t0 + dt_load_ms * 0.001 - g['t'])) % 1.0) * math.tau
    old_until, new_until = g.get('_jump_until', t0 + dur), t0 + dur
    dt_shift = new_until - old_until
    before = g['tab'][g['tab'][:, latent.AT] < t0 - 1e-4].copy()
    after = g['tab'][g['tab'][:, latent.AT] > old_until + 0.002].copy()
    if len(after):
        after[:, latent.AT] += dt_shift
    g['tab'] = np.concatenate([before, new if g['t'] < t0 - 1e-4 else new[1:], after])
    kept_labels = [(w, n) for w, n in g['labels'] if w < t0 - 1e-4 or (w <= t0 + 1e-4 and g['t'] >= t0 - 1e-4)]
    hop_rows = new if g['t'] < t0 - 1e-4 else new[1:]
    idx0 = 0 if g['t'] < t0 - 1e-4 else 1
    hop_labels = [(float(w), names[i + idx0] if i + idx0 < len(names) else 'rest') for i, w in enumerate(hop_rows[:, latent.AT])]
    after_labels = [(w + dt_shift, n) for w, n in g['labels'] if w > old_until + 0.002]
    g['labels'] = (kept_labels + hop_labels + after_labels)[-16:]
    g['_jump_until'], g['_jump_throw'] = new_until, t0 + dt_load_ms * 0.001


def stride_hop(g, by, t0, spd, back_now, fwd_gaits, back_gaits, said_fn, walk_on_wire):
    """Queue `hop` at `t0` (two-footed gather when striding, ballistic leap, then resume the gait)."""
    import numpy as np
    from machine import latent, ortho
    prev = (g.get('active_entry') if g.get('active_entry') in (fwd_gaits + back_gaits)
            else ('stand' if not g.get('_twist_active') else ('walk_back' if back_now else ('jog' if g.get('asked', (0.0,))[0] > 0.9 else 'walk'))))
    in_fwd = prev in fwd_gaits and not back_now
    moving = in_fwd or prev != 'stand' or spd > 0.15
    w_fast = max(0.0, min(1.0, (float(spd) - 0.55) / 0.20))
    dt_stand = 0.45 + 0.15 * w_fast
    dt_b = (0.85 + 0.20 * w_fast) if moving else 0.0
    if dt_b > 0.0 and '_brake_wire' in g:
        said_fn(g, 'brake', g.get('_brake_back_wire', g['_brake_wire']) if back_now else g['_brake_wire'],
                ('brake',), at=t0, reset=True)
        if 'stand' in by:
            said_fn(g, *by['stand'], at=t0 + dt_stand, reset=True)
    t_hop = t0 + dt_b
    ch = float(g.get('_hop_charge', 0.0))
    g['_leap_spd'] = float(spd)
    wire, names, dur, dt_load_ms = (stride_leap(by['hop'][1], walk_on_wire, ch, spd) if in_fwd
                                    else standing_leap(by['hop'][1], ch))
    if dt_b == 0.0:
        g['st'][latent.THETA] = ((0.10 - dt_load_ms * 0.001) % 1.0) * math.tau
    said_fn(g, 'hop', wire, names, at=t_hop, reset=True)
    t_res = t_hop + dur
    g['_jump_t0'], g['_jump_throw'], g['_jump_until'] = t_hop, t_hop + dt_load_ms * 0.001, t_res
    g['_leap'], g['active_entry'] = in_fwd, prev
    if 'stand' in by:
        said_fn(g, *by['stand'], at=t_res + 0.01, reset=True)
    if prev != 'stand':
        t_go = t_res + 0.70
        iw = by['init_walk'][1][:ortho.ROW.size] if 'init_walk' in by else None
        if in_fwd and iw is not None:
            said_fn(g, 'init_walk', iw, ('init_walk',), at=t_go, reset=True)
            t_gait = t_go + 0.28
        else:
            t_gait = t_go
        if prev == 'walk':
            said_fn(g, prev, walk_on_wire, (prev,), at=t_gait)
        elif prev == 'jog' and 'jog' in by:
            rj = np.frombuffer(by['jog'][1], latent.WIRE)[:1].copy()
            r1 = np.frombuffer(by['jog'][1], latent.WIRE)[1]
            rj['p'][0, ~np.isnan(r1['p'])] = r1['p'][~np.isnan(r1['p'])]
            rj['at'], rj['blend'] = (0,), (160,)
            said_fn(g, 'jog', rj.tobytes(), ('run',), at=t_gait)
        elif prev in by:
            said_fn(g, *by[prev], at=t_gait)



