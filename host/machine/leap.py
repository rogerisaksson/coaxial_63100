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


#: The human's standing jump (`takes.jump(JUMP)`, test_leap holds them to it): the s his crouch
#: takes and his take-off comes at; the crouch he pushes from and his trunk's lean there, axial
#: and pitch; his stand at take-off, axial, and his legs' lengthening there, axial/s.
JUMP = 'standing_jump'
CROUCH_S, TAKEOFF_S = 0.465, 0.745
CROUCH, LEAN, STRAIGHT, LEAVES = 2.18, 1.70, 0.11, 11.7
#: What a push asks of the knees' clamp, as `tools.sim.demand.PAST`: at the limit no margin.
PUSH = 2.0 / 3.0
#: A push's rows this far apart, ms, a sample of its curve each.
PUSH_MS = 20


def _a_z(fr, z):
    """The drives' acceleration of her legs' shortening at `z` (axial/s^2): PUSH of the knees'
    clamp through the knee's lever to the leg's line there, against her weight."""
    from machine import drives, figure, modal
    reach = fr[modal.REACH0] - modal.AXIAL_M * fr[modal.HIPS_BY] * z
    cos_knee = (figure.THIGH ** 2 + figure.SHANK ** 2 - reach ** 2) / (2.0 * figure.THIGH * figure.SHANK)
    lever = figure.THIGH * figure.SHANK * math.sqrt(1.0 - max(-1.0, min(1.0, cos_knee)) ** 2) / reach
    force = 2.0 * PUSH * drives.peak('left_knee') / lever
    return (force / fr[modal.WEIGHT] * G - G) / (modal.AXIAL_M * fr[modal.HIPS_BY])


def pushed(fr, z0):
    """(the push's acceleration, axial a s^2, and its seconds) from the crouch `z0` to STRAIGHT
    at PUSH of the knees' clamp against her weight, the knee's lever to the leg's line at the
    crouch - the longest of the push, where the clamp gives the least - so the drives carry every
    pass of it, the throw's share of it the crouch's share of his (a tap's shallow crouch at
    the whole clamp left at 10.5 axial/s of a hold's 12.3, a hop as high as the jump; the
    clamp's share so, a tap's push fell short of her weight): asked as a spring (18 000) the
    step shrank the throw to a tenth and she rose 0.01 legs on either stack; his push 0.29 m
    in 0.3 s, hers 0.33 s at 472 N."""
    a_z = _a_z(fr, z0) * (z0 - STRAIGHT) / (CROUCH - STRAIGHT)
    return a_z, math.sqrt(2.0 * (z0 - STRAIGHT) / a_z)


def bottom(fr, v_off):
    """The legs' shortening a landing at `v_off` axial/s brakes to from STRAIGHT at the drives'
    acceleration (`_a_z`), CROUCH at most: folded in flight as his, she had 0.07 m to brake 1.6
    m/s in and sank to the floor; from the crouch down, a tap's 0.7 m/s landed in a full squat."""
    import numpy as np
    zs = np.linspace(STRAIGHT, CROUCH, 200)
    v2 = np.concatenate([[0.0], np.cumsum(2.0 * np.array([_a_z(fr, z) for z in zs[1:]]) * (zs[1] - zs[0]))])
    return float(np.interp(v_off * v_off, v2, zs))


def _leap_specs():
    from machine import latent, modal
    z, k, d, amp, zmp, gain = latent.Z.start, latent.K.start, latent.D.start, latent.AMP.start, latent.ZMP, latent.GAIN
    standing = (
        (0, z + modal.AXIAL, 0.58, 0.48, 0.0, 0.0), (0, z + modal.SAGITTAL, -0.10, 0.0, 0.0, 0.0),
        (0, z + modal.PITCH, 0.0, 0.0, 0.0, 0.0), (0, z + modal.ARMS, 0.0, -0.40, 0.0, 0.0),
        (0, k + modal.AXIAL, 6000.0, 0.0, 0.0, 0.0), (0, d + modal.AXIAL, 900.0, 0.0, 0.0, 0.0),
        (0, k + modal.SAGITTAL, 1200.0, 0.0, 0.0, 0.0), (0, d + modal.SAGITTAL, 400.0, 0.0, 0.0, 0.0),
        (0, zmp, modal.SUPPORT, 0.0, 0.0, 0.0), (0, gain, 0.0, 0.0, 0.0, 0.0),
        (1, z + modal.AXIAL, 0.20, -0.22, 0.0, 0.0), (1, k + modal.AXIAL, 2000.0, 0.0, 0.0, 0.0),
        (1, d + modal.AXIAL, 600.0, 0.0, 0.0, 0.0), (1, z + modal.SAGITTAL, 0.0, 0.0, 0.0, 0.0),
        (1, k + modal.SAGITTAL, 1200.0, 0.0, 0.0, 0.0), (1, d + modal.SAGITTAL, 400.0, 0.0, 0.0, 0.0),
        (1, z + modal.PITCH, 0.0, 0.0, 0.0, 0.0), (1, k + modal.PITCH, 0.0, 0.0, 0.0, 0.0),
        (1, d + modal.PITCH, 200.0, 0.0, 0.0, 0.0), (1, z + modal.ARMS, 0.0, 0.70, 0.0, 0.0),
        (1, zmp, modal.NONE, 0.0, 0.0, 0.0),
        (2, z + modal.AXIAL, 0.36, 0.49, 0.0, 0.0), (2, z + modal.SAGITTAL, 0.0, 0.0, 0.0, 0.0),
        (2, z + modal.PITCH, 0.0, 0.0, 0.0, 0.0), (2, z + modal.ARMS, 0.0, 0.0, 0.0, 0.0),
        (2, k + modal.AXIAL, 3000.0, 0.0, 0.0, 0.0), (2, d + modal.AXIAL, 600.0, 0.0, 0.0, 0.0),
        (2, k + modal.SAGITTAL, 600.0, 0.0, 0.0, 0.0), (2, d + modal.SAGITTAL, 300.0, 0.0, 0.0, 0.0),
        (2, k + modal.PITCH, 1458.0, 0.0, 0.0, 0.0), (2, d + modal.PITCH, 292.0, 0.0, 0.0, 0.0),
        (2, k + modal.LIFT, 2400.0, 3000.0, 0.0, 0.0), (2, d + modal.LIFT, 320.0, 0.0, 0.0, 0.0),
        (2, zmp, modal.SUPPORT, 0.0, 0.0, 0.0),
    )
    stride = (
        (0, z + modal.AXIAL, 0.66, 0.0, 0.24, 0.0), (0, z + modal.SAGITTAL, 0.14, 0.0, 0.54, 0.0),
        (0, z + modal.PITCH, -0.05, 0.14, 0.0, 0.0), (0, z + modal.ARMS, -0.35, 0.0, -0.45, 0.0),
        (0, k + modal.AXIAL, 6000.0, 0.0, 0.0, 0.0), (0, d + modal.AXIAL, 900.0, 0.0, 0.0, 0.0),
        (0, k + modal.SAGITTAL, 1200.0, 0.0, 0.0, 0.0), (0, d + modal.SAGITTAL, 400.0, 0.0, 0.0, 0.0),
        (0, zmp, modal.SUPPORT, 0.0, 0.0, 0.0), (0, gain, 0.0, 0.0, 0.0, 0.0),
        (1, z + modal.AXIAL, 0.15, 0.0, 0.0, 0.0), (1, k + modal.AXIAL, 21600.0, 0.0, 3400.0, 0.0),
        (1, d + modal.AXIAL, 0.0, 0.0, 0.0, 0.0), (1, z + modal.SAGITTAL, 0.28, 0.0, 0.60, 0.0),
        (1, k + modal.SAGITTAL, 1800.0, 0.0, 1000.0, 0.0), (1, d + modal.SAGITTAL, 200.0, 0.0, 0.0, 0.0),
        (1, z + modal.PITCH, -0.05, 0.14, 0.0, 0.0), (1, k + modal.PITCH, 1458.0, 0.0, 0.0, 0.0),
        (1, d + modal.PITCH, 292.0, 0.0, 0.0, 0.0), (1, z + modal.ARMS, 0.45, 0.0, 0.45, 0.0),
        (1, zmp, modal.NONE, 0.0, 0.0, 0.0), (1, amp + modal.DRIVE, 0.64, 0.0, 0.0, 0.24),
        (1, amp + modal.LIFT, 1.20, 0.0, 0.0, 0.40), (1, gain, 0.0, 0.0, 0.0, 0.0),
        (2, z + modal.AXIAL, 0.25, 0.0, 0.0, 0.0), (2, z + modal.SAGITTAL, -0.24, -0.10, 0.0, 0.0),
        (2, z + modal.PITCH, 0.0, 0.0, 0.0, 0.0), (2, z + modal.ARMS, 0.0, 0.0, 0.0, 0.0),
        (2, k + modal.AXIAL, 3000.0, 0.0, 0.0, 0.0), (2, d + modal.AXIAL, 600.0, 0.0, 0.0, 0.0),
        (2, k + modal.SAGITTAL, 600.0, 0.0, 0.0, 0.0), (2, d + modal.SAGITTAL, 300.0, 0.0, 0.0, 0.0),
        (2, k + modal.PITCH, 1458.0, 0.0, 0.0, 0.0), (2, d + modal.PITCH, 292.0, 0.0, 0.0, 0.0),
        (2, zmp, modal.SUPPORT, 0.0, 0.0, 0.0), (2, gain, 0.0, 0.0, 0.0, 0.0),
    )
    return standing, stride


def _build_leap(base_wire, times_ms, blends_ms, spec, weights):
    import numpy as np
    from machine import latent, ortho
    rb = np.frombuffer(base_wire, latent.WIRE).copy()
    t0, t1, t2 = times_ms
    rb['at'][:3], rb['blend'][:3] = (0, t0, t0 + t1), blends_ms
    if len(rb) > 3:
        rb['at'][3], rb['blend'][3] = t0 + t1 + t2, ortho.REST_MS
    w0, w1, w2, w3 = weights
    for r, col, a0, a1, a2, a3 in spec:
        rb['p'][r, col] = a0 * w0 + a1 * w1 + a2 * w2 + a3 * w3
    return rb.tobytes(), ('load', 'throw', 'land', 'rest'), (t0 + t1 + t2) * 0.001, t0


def standing_leap(base_wire, fr, ch=0.0):
    """Her standing jump's rows (`latent.WIRE`, the lisp's `hop` their base) by the hold's charge
    `ch` in [0, 1]: the human's own descent (`takes.frames` to CROUCH_S) to his crouch at full
    charge (CROUCH, leant LEAN), shallower by the charge at his pace - told a line to it she fell
    70 ms free, a sole 8 mm up, her parry stepped it and she crouched and pushed on one leg;
    quickened by its depth, a tap's dip of 0.08 m in 0.12 s dropped her 0.2 m off her soles -,
    held a period of the row's spring so the push begins from rest - begun as she still sank,
    the step caught her up at 14 m/s^2 and she led the curve by 0.1 -; the push a curve of rows
    PUSH_MS apart (`pushed`, her frame `fr`) to STRAIGHT; then his frames from his take-off on
    (`takes.wire`), his fold in flight and his squat landing by how fast she leaves against him
    (LEAVES) - told upright in flight, her legs swung 0.35 m ahead with her trunk's spin and she
    fell back -, the landing row's springs on the first; her rest. And their names, the seconds
    to her rest, the ms the push begins at."""
    import numpy as np
    from machine import latent, modal, ortho, takes
    ch = max(0.0, min(1.0, float(ch)))
    z0 = 0.58 + ch * (CROUCH - 0.58)
    a_z, seconds = pushed(fr, z0)
    n = max(2, int(math.ceil(1e3 * seconds / PUSH_MS)))
    push_ms = int(round(1e3 * seconds / n))
    wire, _names, _dur, _t0 = _build_leap(base_wire, (0, (n + 1) * push_ms, 0), (0, push_ms, 0),
                                          _leap_specs()[0], (1.0, ch, 0.0, 0.0))
    rows = np.frombuffer(wire, latent.WIRE).copy()
    z = latent.Z.start
    axial, pitch, sagittal = z + modal.AXIAL, z + modal.PITCH, z + modal.SAGITTAL
    depth = z0 / CROUCH
    t_d, z_d = takes.frames(JUMP, 0.0, CROUCH_S)
    step = t_d[1] - t_d[0]
    load = np.repeat(rows[0:1], len(t_d) + 1)
    for k in range(len(t_d)):
        load['at'][k], load['blend'][k] = int(round(1e3 * t_d[k])), int(round(1e3 * step))
        load['flags'][k] = ortho.CURVE
        load['p'][k, axial], load['p'][k, sagittal] = z_d[k, modal.AXIAL] * depth, z_d[k, modal.SAGITTAL] * depth
        load['p'][k, pitch] = z_d[k, modal.PITCH] * ch
    load['at'][-1], load['blend'][-1], load['flags'][-1] = load['at'][-2] + load['blend'][-2], 0, 0
    load['p'][-1, axial], load['p'][-1, sagittal], load['p'][-1, pitch] = z0, z_d[-1, modal.SAGITTAL] * depth, LEAN * ch
    period_ms = int(round(1e3 * math.tau * math.sqrt(fr[modal.WEIGHT] / G / rows['p'][0, latent.K.start + modal.AXIAL])))
    t0 = int(load['at'][-1]) + period_ms
    throw = np.repeat(rows[1:2], n + 1)
    for k in range(n + 1):
        t = k * push_ms * 1e-3
        throw['at'][k], throw['blend'][k], throw['flags'][k] = t0 + k * push_ms, push_ms, ortho.CURVE
        throw['p'][k, axial] = max(STRAIGHT, z0 - 0.5 * a_z * t * t)
        throw['p'][k, pitch] = LEAN * ch * (1.0 - t / seconds)
        throw['p'][k, sagittal] = z_d[-1, modal.SAGITTAL] * depth
    # her flight at the push's end speed, her legs long and soft (no height spring) and her
    # centre of mass where the crouch had it over her soles, 0.21 m forward over the jump
    # (his 0.45 to 0.04 in his last 30 ms is his heels' rise: told there in a line she left
    # 0.17 m back, told there with its rate dying 0.28 m back; folded in flight as his she
    # landed on her heels 0.17 m behind them), his trunk's fold over it; the absorption a damper told the bottom the
    # drives brake her to (`bottom`): it asks them what her speed is worth and the clamp gives
    # what it can (a parabola from the push's end speed yanked her down at 23 m/s^2 where she
    # met the floor slower); the polygon let go till the bottom; then her rest over his rise's
    # seconds (his own rise, told as rows, carries his rebound - a second squat at 1.3-1.5 s -
    # and she jumped again off the landing)
    v_off = a_z * seconds
    flight_s = 2.0 * v_off * modal.AXIAL_M * fr[modal.HIPS_BY] / G
    fold = min(1.0, v_off / LEAVES)
    t_h, z_h = takes.frames(JUMP, TAKEOFF_S)
    deepest = int(np.argmax(z_h[:, modal.AXIAL]))
    deep = bottom(fr, v_off)
    n_f, n_b = max(1, int(round(1e3 * flight_s / push_ms))), max(2, int(round(1e3 * v_off / a_z / push_ms)))
    t_off = t0 + (n + 1) * push_ms
    sag0, sag_deep = float(z_d[-1, modal.SAGITTAL] * depth), fold * float(z_h[deepest, modal.SAGITTAL])
    flight = np.repeat(rows[2:3], n_f)
    for k in range(n_f):
        flight['at'][k], flight['blend'][k], flight['flags'][k] = t_off + k * push_ms, push_ms, ortho.CURVE
        flight['p'][k, axial], flight['p'][k, sagittal] = STRAIGHT, sag0
        flight['p'][k, pitch] = fold * np.interp(k * push_ms * 1e-3 * LEAVES / v_off, t_h - TAKEOFF_S, z_h[:, modal.PITCH])
        flight['p'][k, latent.ZMP], flight['p'][k, latent.K.start + modal.AXIAL] = modal.NONE, 0.0
    t_land = t_off + n_f * push_ms
    brake = np.repeat(rows[2:3], n_b)
    for k in range(n_b):
        brake['at'][k], brake['blend'][k], brake['flags'][k] = t_land + k * push_ms, push_ms, ortho.CURVE
        brake['p'][k, axial], brake['p'][k, latent.K.start + modal.AXIAL] = deep, 0.0
        brake['p'][k, sagittal] = sag0 + (sag_deep - sag0) * k / max(1, n_b - 1)
        brake['p'][k, pitch], brake['p'][k, latent.ZMP] = fold * z_h[deepest, modal.PITCH], modal.NONE
    rows['at'][3] = t_land + n_b * push_ms
    rows['blend'][3] = int(round(1e3 * (t_h[-1] - t_h[deepest])))
    end_ms = int(rows['at'][3] + rows['blend'][3])
    names = ('load',) * len(load) + ('throw',) * (n + 1) + ('flight',) * n_f + ('land',) * n_b + ('rest',)
    return np.concatenate([load, throw, flight, brake, rows[3:]]).tobytes(), names, end_ms * 1e-3, t0


def stride_leap(base_wire, _walk_wire, ch=0.0, spd=0.69):
    """Construct the 11D modal trajectory table (`latent.WIRE`) for an in-stride forward bound parameterized by `ch` in [0, 1] and approach speed `spd`."""
    ch = max(0.0, min(1.0, float(ch)))
    w_fast = max(0.0, min(1.0, (float(spd) - 0.55) / 0.20))
    dt_load_ms, dt_throw_ms = int(round(440 + 140 * ch)), int(round(146 + 21 * ch))
    return _build_leap(base_wire, (dt_load_ms, dt_throw_ms, 400), (int(round(0.75 * dt_load_ms)), 0, 0),
                       _leap_specs()[1], (1.0, ch, 0.06 + 0.28 * ch + 0.08 * w_fast, 0.55 * ch + 0.15 * w_fast))


def update_leap_charge(g, by, walk_on_wire, ch):
    """Stretch and scale the active leap trajectory in `g['tab']` while in the `load` phase."""
    import numpy as np
    from machine import latent
    t_throw, t0 = g.get('_jump_throw', 0.0), g.get('_jump_t0', 0.0)
    if g['t'] >= t_throw - 0.015 or 'hop' not in by or t0 <= 0.0:
        return
    in_stride, spd = bool(g.get('_leap')), float(g.get('_leap_spd', 0.69))
    wire, names, dur, dt_load_ms = (stride_leap(by['hop'][1], walk_on_wire, ch, spd) if in_stride
                                    else standing_leap(by['hop'][1], g['fr'], ch))
    new = latent.table(wire, t0, g['rest'])
    new[0, latent.VALS] = np.where(new[0, latent.SAID] > 0.5, new[0, latent.VALS], g['rest'])
    new[0, latent.SAID] = 1.0
    started = g['t'] >= t0 - 1e-4
    if started:
        g['st'][latent.TO] = np.where(new[0, latent.SAID] > 0.5, new[0, latent.VALS], g['st'][latent.TO])
        g['st'][latent.BLEND] = new[0, latent.LERP]
        g['st'][latent.THETA] = ((0.10 - (t0 + dt_load_ms * 0.001 - g['t'])) % 1.0) * math.tau
    old_until, new_until = g.get('_jump_until', t0 + dur), t0 + dur
    dt_shift = new_until - old_until
    before = g['tab'][g['tab'][:, latent.AT] < t0 - 1e-4].copy()
    after = g['tab'][g['tab'][:, latent.AT] > old_until + 0.002].copy()
    after[:, latent.AT] += dt_shift
    idx0 = int(started)
    hop_rows = new[idx0:]
    g['tab'] = np.concatenate([before, hop_rows, after])
    kept_labels = [(w, n) for w, n in g['labels'] if w < t0 - 1e-4 or (w <= t0 + 1e-4 and started)]
    hop_labels = [(float(w), names[i + idx0] if i + idx0 < len(names) else 'rest') for i, w in enumerate(hop_rows[:, latent.AT])]
    after_labels = [(w + dt_shift, n) for w, n in g['labels'] if w > old_until + 0.002]
    g['labels'] = (kept_labels + hop_labels + after_labels)[-16:]
    g['_jump_until'], g['_jump_throw'] = new_until, t0 + dt_load_ms * 0.001


def _jog_resume_entry(by):
    import numpy as np
    from machine import latent
    rj = np.frombuffer(by['jog'][1], latent.WIRE)[:1].copy()
    r1 = np.frombuffer(by['jog'][1], latent.WIRE)[1]
    rj['p'][0, ~np.isnan(r1['p'])] = r1['p'][~np.isnan(r1['p'])]
    rj['at'], rj['blend'] = (0,), (160,)
    return ('jog', rj.tobytes(), ('run',))


def _resume_gait(g, by, prev, in_fwd, t_go, walk_on_wire, said_fn):
    from machine import ortho
    if prev == 'stand':
        return
    iw = by['init_walk'][1][:ortho.ROW.size] if (in_fwd and 'init_walk' in by) else None
    if iw is not None:
        said_fn(g, 'init_walk', iw, ('init_walk',), at=t_go, reset=True)
    t_gait = t_go + (0.28 if iw is not None else 0.0)
    entry = (('walk', walk_on_wire, ('walk',)) if prev == 'walk'
             else (_jog_resume_entry(by) if (prev == 'jog' and 'jog' in by) else by.get(prev)))
    if entry is not None:
        said_fn(g, *entry, at=t_gait)


def stride_hop(g, by, t0, spd, back_now, fwd_gaits, back_gaits, said_fn, walk_on_wire):
    """Queue `hop` at `t0` (two-footed gather when striding, ballistic leap, then resume the gait)."""
    from machine import latent
    prev = (g.get('active_entry') if g.get('active_entry') in (fwd_gaits + back_gaits)
            else ('stand' if not g.get('_twist_active') else ('walk_back' if back_now else ('jog' if g.get('asked', (0.0,))[0] > 0.9 else 'walk'))))
    in_fwd = prev in fwd_gaits and not back_now
    moving = in_fwd or prev != 'stand' or spd > 0.15
    w_fast = max(0.0, min(1.0, (float(spd) - 0.55) / 0.20))
    dt_stand, dt_b = 0.45 + 0.15 * w_fast, (0.85 + 0.20 * w_fast) * float(moving)
    if dt_b > 0.0 and '_brake_wire' in g:
        said_fn(g, 'brake', g.get('_brake_back_wire', g['_brake_wire']) if back_now else g['_brake_wire'],
                ('brake',), at=t0, reset=True)
    if dt_b > 0.0 and '_brake_wire' in g and 'stand' in by:
        said_fn(g, *by['stand'], at=t0 + dt_stand, reset=True)
    t_hop, ch = t0 + dt_b, float(g.get('_hop_charge', 0.0))
    g['_leap_spd'] = float(spd)
    wire, names, dur, dt_load_ms = (stride_leap(by['hop'][1], walk_on_wire, ch, spd) if in_fwd
                                    else standing_leap(by['hop'][1], g['fr'], ch))
    if dt_b == 0.0:
        g['st'][latent.THETA] = ((0.10 - dt_load_ms * 0.001) % 1.0) * math.tau
    said_fn(g, 'hop', wire, names, at=t_hop, reset=True)
    t_res = t_hop + dur
    g['_jump_t0'], g['_jump_throw'], g['_jump_until'] = t_hop, t_hop + dt_load_ms * 0.001, t_res
    g['_leap'], g['active_entry'] = in_fwd, prev
    if 'stand' in by:
        said_fn(g, *by['stand'], at=t_res + 0.01, reset=True)
    _resume_gait(g, by, prev, in_fwd, t_res + 0.70, walk_on_wire, said_fn)



