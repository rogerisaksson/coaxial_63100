"""What befell her and how she lies, in words and numbers: what the planner reads (`planner`).

    now = observer.status(bus, world, director)   # {'lying': 'face down', 'pelvis_m': 0.14, ..}
    ok, why = observer.check('roll onto back', now) # the step's outcome, as the observer sees it

`lying` from her feet, her head's IMU and her pelvis: on her feet (FEET_SHARE of her weight on
the floor through them), 'standing' or 'crouched' by the pelvis's height; her head upright
(UPRIGHT), 'kneeling' or 'sitting'; else 'on her back', 'face down', 'on her left side' or 'on
her right side' (her face within SIDE_UP of level). What she touches the floor with and the
drives derated are the rest of it.
"""
from machine import figure

#: Her face this far above or below level, as a sine, is up or down; nearer level, a side. Her
#: head's up this far up is upright.
SIDE_UP, UPRIGHT = 0.5, 0.5
#: Upright, the pelvis's height over the floor, m, above which she stands or crouches - on her
#: feet, FEET_SHARE of her weight on them - or kneels.
STANDS_M, CROUCHES_M, FEET_SHARE = 0.75, 0.25, 0.6
#: Her weight, N.
WEIGHT_N = figure.MASS_KG * 9.81


def _head(bus):
    return figure.quat(*(bus['pelvis.pose.head_q' + a] for a in 'wxyz'))


def on_feet(world):
    """Her soles' load on the floor, N - not her own weight sat back on her heels."""
    d, m = world.data, world.model
    soles = {m.body(side + part).id for side in ('left_', 'right_') for part in ('foot', 'toes')}
    ours = {m.body(s[0]).id for s in figure.SEGMENTS}
    force, out = world._np.zeros(6), 0.0
    for i in range(d.ncon):
        a, b = (m.geom_bodyid[g] for g in (d.contact[i].geom1, d.contact[i].geom2))
        if (a in soles and b not in ours) or (b in soles and a not in ours):
            world._mj.mj_contactForce(m, d, i, force)
            out += force[0]
    return out


def lying(bus, world):
    """How she lies or stands, a word or two."""
    head = _head(bus)
    face_up, left_up = head[1][2], head[1][0]
    feet = on_feet(world) / WEIGHT_N
    y = bus['pelvis.pose.y']
    if feet > FEET_SHARE and y > CROUCHES_M:
        return 'standing' if y > STANDS_M else 'crouched'
    if head[1][1] > UPRIGHT:
        return 'kneeling' if y > CROUCHES_M else 'sitting'
    if face_up > SIDE_UP:
        return 'on her back'
    if face_up < -SIDE_UP:
        return 'face down'
    return 'on her left side' if left_up < 0.0 else 'on her right side'


def touching(world):
    """Her segments touching the floor or what lies on it, by name, sides merged."""
    d, m = world.data, world.model
    ours = {m.body(s[0]).id: s[0] for s in figure.SEGMENTS}
    out = set()
    for i in range(d.ncon):
        a, b = (m.geom_bodyid[g] for g in (d.contact[i].geom1, d.contact[i].geom2))
        for mine, other in ((a, b), (b, a)):
            if mine in ours and other not in ours:
                out.add(ours[mine].replace('left_', '').replace('right_', ''))
    return sorted(out)


def status(bus, world, director):
    """{name: value}: what felled her, how she lies, what she touches, the rest in numbers."""
    head = _head(bus)
    feet = on_feet(world)
    return {'cause': director.cause, 'lying': lying(bus, world), 'face_up': round(head[1][2], 2),
            'left_up': round(head[1][0], 2), 'head_up': round(head[1][1], 2),
            'pelvis_m': round(bus['pelvis.pose.y'], 2),
            'touching': touching(world), 'feet_share': round(feet / WEIGHT_N, 2),
            'derated': sorted(j for j, n in director.drives.items()
                              if bus.get(n + 'derate', 1.0) < 0.9),
            'tries': director.tries}


#: Each step's outcome as the observer judges it at its end: (what it wants, of a status).
EXPECT = {'roll onto back': ('on her back', lambda s: s['lying'] == 'on her back'),
          'roll onto front': ('face down', lambda s: s['lying'] == 'face down'),
          'sit up': ('sitting', lambda s: s['lying'] in ('sitting', 'kneeling', 'crouched')),
          'knees under': ('the pelvis up on her knees',
                          lambda s: s['pelvis_m'] > 0.2 and 'shank' in s['touching']),
          'sit back on heels': ('the pelvis back on her heels',
                                lambda s: s['pelvis_m'] > 0.3 and 'shank' in s['touching']),
          'onto feet': ('crouched', lambda s: s['lying'] in ('crouched', 'standing'))}


def check(step, now):
    """(ok, why): whether `step` came out as it should, and if not what she is instead."""
    want, ok = EXPECT.get(step, ('', lambda s: True))
    return (True, '') if ok(now) else (False, '%s wanted %s, she is %s (face up %.2f, head up %.2f, '
                                              'pelvis %.2f m, feet %.2f)' % (
        step, want, now['lying'], now['face_up'], now['head_up'], now['pelvis_m'],
        now['feet_share']))
