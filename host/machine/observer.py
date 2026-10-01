"""What befell her and how she lies, in words and numbers: what the planner reads (`planner`).

    now = observer.status(bus, world, director)   # {'lying': 'face down', 'pelvis_m': 0.14, ..}
    ok, why = observer.check('knees under', now)  # the step's outcome, as the observer sees it

`lying` from her feet, her head's IMU and her pelvis: on her feet (FEET_SHARE of her weight on
the floor through them, no knee down), 'standing' or 'crouched' by the pelvis's height; her head
upright (UPRIGHT), 'kneeling' or 'sitting'; else 'on her back', 'face down', 'on her left side' or
'on her right side' (her face within SIDE_UP of level). What she touches the floor with and the
drives derated are the rest of it.
"""
import math

from machine import figure

#: Her face this far above or below level, as a sine, is up or down; nearer level, a side. Her
#: head's up this far up is upright.
SIDE_UP, UPRIGHT = 0.5, 0.5
#: Upright, the pelvis's height over the floor, m, above which she stands or crouches - on her
#: feet, FEET_SHARE of her weight on them and no knee down - or kneels. Sat back on her heels,
#: her toes carried 0.65 of her weight (2026-10-01).
STANDS_M, CROUCHES_M, FEET_SHARE = 0.75, 0.25, 0.6
#: Her weight, N.
#: Back on her heels, her centre of mass within OVER_FEET_M of her toes, her head upright: 0.53 m
#: ahead, her head on the floor, passed the pelvis-and-shin check this replaced (2026-10-01).
OVER_FEET_M = 0.15
#: On her knees, the pelvis rolled under LEVEL_DEG: 37-40 degrees onto a side, sitting back
#: rolled her over, 4 kneels of 22 (2026-10-01).
LEVEL_DEG = 20.0
#: Onto her feet, they are flat within FLAT_DEG when the arrival takes her: pitched 22 degrees,
#: the ankles 39 mm over its squat's, its legs reached for the floor and threw her up (2026-10-01).
FLAT_DEG = 10.0
#: ... and still, the pelvis under STILL_M_S: arriving at 0.4 m/s back, she ran out over her heels
#: and its feedback flung her 2.5 m (2026-10-01).
STILL_M_S = 0.1


def _head(bus):
    return figure.quat(*(bus['pelvis.pose.head_q' + a] for a in 'wxyz'))


def _contacts(world):
    """(her soles' load on the floor, N - not her own weight sat back on her heels -, her
    segments touching the floor or what lies on it, by name, sides merged), in one pass."""
    d, m = world.data, world.model
    ours = {m.body(s[0]).id: s[0] for s in figure.SEGMENTS}
    soles = {m.body(side + part).id for side in ('left_', 'right_') for part in ('foot', 'toes')}
    force, load, touched = world._np.zeros(6), 0.0, set()
    for i in range(d.ncon):
        a, b = (m.geom_bodyid[g] for g in (d.contact[i].geom1, d.contact[i].geom2))
        for mine, other in ((a, b), (b, a)):
            if mine in ours and other not in ours:
                touched.add(ours[mine].replace('left_', '').replace('right_', ''))
                if mine in soles:
                    world._mj.mj_contactForce(m, d, i, force)
                    load += force[0]
    return load, sorted(touched)


def on_feet(world):
    """Her soles' load on the floor, N (`_contacts`)."""
    return _contacts(world)[0]


def com_ahead(world):
    """Her centre of mass ahead of her toes along her heading (`_ahead`), m."""
    d, m = world.data, world.model
    ax, az = _ahead(world)
    toes = sum(d.xpos[m.body(s + 'toes').id] for s in ('left_', 'right_')) / 2.0
    com = d.subtree_com[m.body('pelvis').id]
    return float((com[0] - toes[0]) * ax + (com[2] - toes[2]) * az)


def _ahead(world):
    """Her heading on the floor, (x, z): the pelvis's left turned back a quarter, which holds
    however she pitches."""
    d, m = world.data, world.model
    left = d.xmat[m.body('pelvis').id].reshape(3, 3)[:, 0]
    n = math.hypot(left[0], left[2]) or 1.0
    return -left[2] / n, left[0] / n


def feet_pitch(world):
    """Her feet's mean pitch along her heading, deg, as `getup.handed` reads each."""
    d, m = world.data, world.model
    ax, az = _ahead(world)
    out = 0.0
    for side in ('left_', 'right_'):
        up = d.xmat[m.body(side + 'foot').id].reshape(3, 3)[:, 1]
        out += math.degrees(math.atan2(up[0] * ax + up[2] * az, up[1])) / 2.0
    return out


def roll(world):
    """The pelvis's roll, deg: its left side up positive."""
    d, m = world.data, world.model
    up = d.xmat[m.body('pelvis').id].reshape(3, 3)[1, 0]
    return math.degrees(math.asin(max(-1.0, min(1.0, float(up)))))


def lying(bus, world, contacts=None):
    """How she lies or stands, a word or two; `contacts` as `_contacts` read them, if read."""
    head = _head(bus)
    face_up, left_up = head[1][2], head[1][0]
    load, touched = contacts or _contacts(world)
    feet = load / (figure.mass() * 9.81)
    y = bus['pelvis.pose.y']
    if feet > FEET_SHARE and y > CROUCHES_M and not {'shank', 'thigh'} & set(touched):
        return 'standing' if y > STANDS_M else 'crouched'
    if head[1][1] > UPRIGHT:
        return 'kneeling' if y > CROUCHES_M else 'sitting'
    if face_up > SIDE_UP:
        return 'on her back'
    if face_up < -SIDE_UP:
        return 'face down'
    return 'on her left side' if left_up < 0.0 else 'on her right side'


def touching(world):
    """Her segments touching the floor or what lies on it (`_contacts`)."""
    return _contacts(world)[1]


def status(bus, world, director):
    """{name: value}: what felled her, how she lies, what she touches, the rest in numbers."""
    head = _head(bus)
    feet, touched = contacts = _contacts(world)
    return {'cause': director.cause, 'lying': lying(bus, world, contacts),
            'face_up': round(head[1][2], 2),
            'left_up': round(head[1][0], 2), 'head_up': round(head[1][1], 2),
            'pelvis_m': round(bus['pelvis.pose.y'], 2), 'com_ahead_m': round(com_ahead(world), 2),
            'roll_deg': round(roll(world)), 'feet_pitch_deg': round(feet_pitch(world)),
            'speed_m_s': round(math.hypot(bus['pelvis.pose.vx'], bus['pelvis.pose.vz']), 2),
            'touching': touched, 'feet_share': round(feet / (figure.mass() * 9.81), 2),
            'derated': sorted(j for j, n in director.drives.items()
                              if bus.get(n + 'derate', 1.0) < 0.9),
            'tries': director.tries}


#: The steps the observer ends the pass their outcome holds, not at their last keyframe: on
#: her feet, her centre of mass over her toes, the step ran on and sat her down behind them
#: (2026-10-01).
EARLY = ('knees under', 'sit back on heels', 'onto feet')

#: Each step's outcome as the observer judges it at its end: (what it wants, of a status).
EXPECT = {'roll onto front': ('face down', lambda s: s['lying'] == 'face down'),
          'knees under': ('the pelvis up and level on her knees',
                          lambda s: s['pelvis_m'] > 0.2 and abs(s['roll_deg']) < LEVEL_DEG
                          and ('shank' in s['touching'] or 'thigh' in s['touching'])),
          'sit back on heels': ('her head up over her feet',
                                lambda s: abs(s['com_ahead_m']) < OVER_FEET_M
                                and s['head_up'] > UPRIGHT and s['pelvis_m'] > 0.2),
          'onto feet': ('crouched still, flat-footed over her feet',
                        lambda s: s['lying'] in ('crouched', 'standing')
                        and abs(s['com_ahead_m']) < OVER_FEET_M
                        and abs(s['feet_pitch_deg']) < FLAT_DEG and s['speed_m_s'] < STILL_M_S)}


def check(step, now):
    """(ok, why): whether `step` came out as it should, and if not what she is instead."""
    want, ok = EXPECT.get(step, ('', lambda s: True))
    return (True, '') if ok(now) else (False, '%s wanted %s, she is %s (face up %.2f, head up %.2f, '
                                              'pelvis %.2f m rolled %d deg, centre of mass '
                                              '%.2f m ahead of her toes, feet %.2f)' % (
        step, want, now['lying'], now['face_up'], now['head_up'], now['pelvis_m'],
        now['roll_deg'], now['com_ahead_m'], now['feet_share']))
