"""Her balance on the stack: the MPC's plan a tick, a step's swing and fold, the stack's ask.

    st = state(s)                             # standing where she stands (`wbc.sense`'s s)
    ask = step(st, s, loads, now)             # the stack's ask (`wbc.step`) this millisecond;
                                              # loads (left, right) the soles' N

The MPC (`machine.mpc`) plans every TICK_S from her capture point: standing on, or a step - its
side, when it lands and where; a sole that bears nothing a while is a step of its own. A step
lifts its sole at the call: up first, its knee folding,
then along a quintic to the landing, re-aimed each tick until FREEZE_S before it is due, lifted
LIFT_M at the middle; down, it stands where it bears LAND_N, past its time descending at
DESCEND_M_S. The centre of mass's acceleration is the LIPM's from
the plan's ZMP, its height held by HEIGHT_KD. World frame: x her left, y up, z ahead.
"""
import math
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np  # noqa: E402

from machine import mpc  # noqa: E402

G = 9.81

#: The MPC's tick, s; a landing re-aimed until FREEZE_S before it is due.
TICK_S, FREEZE_S = 0.02, 0.06

#: A swing's lift at its middle, m; the load a landed sole bears to stand, N; a late sole's
#: descent, m/s.
LIFT_M, LAND_N, DESCEND_M_S = 0.05, 20.0, 0.2

#: A standing sole the stack laid more than 2 LOST_N on that bears under LOST_N for LOST_S stands
#: no more: it swings, put down where the MPC says - counted standing on 0 N for 0.6 s, her rear
#: foot drifted 10 cm, the MPC saw her on both and she fell aside with no step called (100 N
#: from behind, 2026-10-10); by its load alone, a sole the stack unloaded stepped, 126 steps in
#: 24 shoves of 60 N where 2 in 8.
LOST_N, LOST_S = 15.0, 0.05

#: A swinging sole rises alone over LIFT_FIRST of its swing before it travels; it stands where it
#: bears after LANDS_U of it - from 0.5 and 0.15, 0.2 s side steps came down 11 cm short (100 N,
#: 2026-10-10).
LIFT_FIRST, LANDS_U = 0.10, 0.75

#: The centre of mass's height held by (kp 1/s^2, kd 1/s).
HEIGHT_KD = (100.0, 20.0)

#: Walking (`st['walk']`: m/s, s from a landing to the next), both soles bear DS_S after each
#: landing before the next lifts, START_S before the first: lifted at once from standing, her
#: capture point ran 0.19 m out over the standing sole and each step after landed wider.
DS_S, START_S = 0.1, 0.4

#: A swinging knee folds FOLD_DEG over its line from its lift to LANDS_DEG, at the swing's middle:
#: straight, a knee lifts its sole only to second order - asked up 17 m/s^2 the stack gave it
#: 1.5 (80 N from behind, 2026-10-10).
FOLD_DEG, LANDS_DEG = 45.0, 5.0

SIDES = mpc.SIDES


def feet(s):
    """{side: (the sole's middle (x, z), yaw)} from `wbc.sense`'s s."""
    return {side: (f['sole'][[0, 2]].copy(), math.atan2(f['R'][0, 2], f['R'][2, 2]))
            for side, f in zip(SIDES, s['feet'])}


def state(s, knees):
    """Standing where she stands: her height, her posture and turns kept; `knees` each knee's
    index among `s['q']`, left and right (`wbc.Body.knees`)."""
    return {'phase': 'stand', 'height': float(s['com'][1]), 'posture': s['q'].copy(),
            'turns': tuple(t['quat'].copy() for t in s['turns']), 'tick': 0.0,
            'p': s['com'][[0, 2]].copy(), 'steps': 0, 'side': None, 'knees': knees}


def quintic(p0, v0, a0, pf, T, t):
    """(place, speed, acceleration) at `t` of the quintic from (p0, v0, a0) to (pf, 0, 0) in T."""
    d = pf - p0
    c3 = (20.0 * d - 12.0 * v0 * T - 3.0 * a0 * T * T) / (2.0 * T ** 3)
    c4 = (-30.0 * d + 16.0 * v0 * T + 3.0 * a0 * T * T) / (2.0 * T ** 4)
    c5 = (12.0 * d - 6.0 * v0 * T - a0 * T * T) / (2.0 * T ** 5)
    return (p0 + v0 * t + a0 * t * t / 2.0 + c3 * t ** 3 + c4 * t ** 4 + c5 * t ** 5,
            v0 + a0 * t + 3.0 * c3 * t * t + 4.0 * c4 * t ** 3 + 5.0 * c5 * t ** 4,
            a0 + 6.0 * c3 * t + 12.0 * c4 * t * t + 20.0 * c5 * t ** 3)


def _lift(st, now):
    """(height, its speed, its acceleration) of the swinging sole: a quartic bump to LIFT_M."""
    T = st['land_at'] - st['lift_at']
    u = (now - st['lift_at']) / T
    if u >= 1.0:
        return -DESCEND_M_S * (now - st['land_at']), -DESCEND_M_S, 0.0
    return (16.0 * LIFT_M * u * u * (1.0 - u) ** 2,
            32.0 * LIFT_M * u * (1.0 - u) * (1.0 - 2.0 * u) / T,
            32.0 * LIFT_M * (1.0 - 6.0 * u + 6.0 * u * u) / (T * T))


def _fold(st, now):
    """(rad, rad/s, rad/s^2) of the swinging knee: its lift's angle to LANDS_DEG, FOLD_DEG over."""
    T = st['land_at'] - st['lift_at']
    u = min(1.0, (now - st['lift_at']) / T)
    q0, q1, a = st['knee'], math.radians(LANDS_DEG), math.radians(FOLD_DEG)
    return (q0 + (q1 - q0) * u + a * math.sin(math.pi * u),
            ((q1 - q0) + a * math.pi * math.cos(math.pi * u)) / T if u < 1.0 else 0.0,
            -a * math.pi ** 2 * math.sin(math.pi * u) / T ** 2)


def step(st, s, loads, now, dt=0.001, bears=(0.0, 0.0)):
    """The stack's ask this step: the phase moved on, the MPC planned on its tick; `loads` each
    sole's load, N, `bears` what the stack laid on it the step before (`wbc.step`)."""
    k = SIDES.index(st['side']) if st['side'] else None
    if st['phase'] == 'swing':
        sole = s['feet'][k]['sole']
        late = now >= st['land_at']
        if (loads[k] >= LAND_N and now - st['lift_at'] > LANDS_U * (st['land_at'] - st['lift_at'])) \
                or (late and sole[1] < 0.003):
            st.update(phase='stand', side=None, tick=now, stood=now, next=mpc.OTHER[st['side']])
            k = None
    light = st.setdefault('light', [0.0, 0.0])
    for i in (0, 1):
        light[i] = (light[i] + dt if st['phase'] == 'stand' and loads[i] < LOST_N
                    and bears[i] > 2.0 * LOST_N else 0.0)
    c, v = s['com'], s['vcom']
    omega = math.sqrt(G / max(0.3, c[1]))
    if now >= st['tick'] - 1e-9:
        st['tick'] = now + TICK_S
        xi = c[[0, 2]] + v[[0, 2]] / omega
        walk = st.get('walk')
        if st['phase'] == 'stand':
            lost = [i for i in (0, 1) if light[i] >= LOST_S and not light[1 - i]]
            nxt = st.get('next', 'left')
            if walk and st.get('walking') is None:
                st.update(walking=now, stood=now + START_S - DS_S)
            lift_in = st.get('stood', 0.0) + DS_S - now
            if walk and lift_in <= 1e-9:
                out = mpc.plan(xi, omega, feet(s), (nxt, walk[1] - DS_S),
                               (walk[0], walk[1], nxt, 0.0))
            elif walk:
                out = dict(mpc.plan(xi, omega, feet(s), (nxt, lift_in + walk[1] - DS_S),
                                    (walk[0], walk[1], nxt, lift_in)), step=None)
            elif lost:
                out = mpc.plan(xi, omega, feet(s), (SIDES[lost[0]], mpc.SWINGS[0]))
            else:
                out = mpc.plan(xi, omega, feet(s))
            if out['step'] is not None:
                side, due, land = out['step']
                k = SIDES.index(side)
                f = s['feet'][k]
                st.update(phase='swing', side=side, lift_at=now, land_at=now + due, land=land,
                          at=f['sole'][[0, 2]].copy(), speed=np.zeros(2), acc=np.zeros(2),
                          yaw=math.atan2(f['R'][0, 2], f['R'][2, 2]),
                          knee=float(s['q'][st['knees'][k]]), steps=st['steps'] + 1)
                light[:] = [0.0, 0.0]
        else:
            left = max(TICK_S, st['land_at'] - now)
            out = mpc.plan(xi, omega, feet(s), (st['side'], left),
                           (walk[0], walk[1], mpc.OTHER[st['side']], 0.0) if walk else None)
            if st['land_at'] - now > FREEZE_S:
                st['land'] = out['step'][2]
        st['p'], st['captured'] = out['p'], out['captured']
    p = st['p']
    ask = {'stance': (True, True), 'posture': st['posture'], 'turns': st['turns'],
           'com_acc': np.array([omega ** 2 * (c[0] - p[0]),
                                HEIGHT_KD[0] * (st['height'] - c[1]) - HEIGHT_KD[1] * v[1],
                                omega ** 2 * (c[2] - p[1])])}
    if st['phase'] == 'swing':
        ask['stance'] = tuple(i != k for i in range(2))
        T = max(dt, st['land_at'] - now)
        rising = now < st['lift_at'] + LIFT_FIRST * (st['land_at'] - st['lift_at'])
        at, speed, acc = (st['at'], np.zeros(2), np.zeros(2)) if rising or now >= st['land_at']             else quintic(st['at'], st['speed'], st['acc'], np.asarray(st['land'], float), T,
                         min(dt, T))
        st.update(at=at, speed=speed, acc=acc)
        y, vy, ay = _lift(st, now + dt)
        h = st['yaw'] / 2.0
        ask['swing'] = {k: (np.array([at[0], y, at[1]]), np.array([speed[0], vy, speed[1]]),
                            np.array([acc[0], ay, acc[1]]),
                            np.array([math.cos(h), 0.0, math.sin(h), 0.0]))}
        ask['fold'] = {k: _fold(st, now + dt)}
    return ask
