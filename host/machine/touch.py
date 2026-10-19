"""What her skin and her soles feel: a touch that stops her going, a shove walked out, an edge.

    touch.check_contacts(g, out, doing)     # a pass of her motion, `g` and `out` as `machine.motion`'s
    touch.soles(b, touch, ids, tips, y, fr, stance)   # its arithmetic of her soles
    touch.against(g, twist)                 # a twist less its part toward what she felt

Her law's own reading of the edge's touch (`machine.edge`): no body is named, nothing but forces
on her segments in their own frames, turned by her own model.
"""
import math

from machine import edge, figure

#: Her segments under her knees, a side's: what bears on the floor, or trips over what is on it.
SOLES = ((edge.SKIN.index('left_foot'), edge.SKIN.index('left_toes')),
         (edge.SKIN.index('right_foot'), edge.SKIN.index('right_toes')))
HANDS = tuple(edge.SKIN.index(n) for n in ('left_hand', 'left_fingers', 'right_hand', 'right_fingers'))
ABOVE = [k for k in range(len(edge.SKIN)) if k not in SOLES[0] + SOLES[1] + HANDS]
ARMS = (tuple(edge.SKIN.index(n) for n in ('left_upper_arm', 'left_forearm', 'left_hand', 'left_fingers')),
        tuple(edge.SKIN.index(n) for n in ('right_upper_arm', 'right_forearm', 'right_hand', 'right_fingers')))
CORE = [k for k in ABOVE if k not in ARMS[0] + ARMS[1]]


#: A touch above her soles not along her going - a wall, a tree, a hand that shoves her back or
#: aside - interrupts it: she brakes and stands, steps where her capture point asks, and goes on
#: as she was told RESUME_S after it is gone; one within ALONG_COS of her going she walks out.
#: Walking at 0.69 m/s an 80 N shove felled her 5 ways of 8 going on; stopped 0.1 s after it, 3 -
#: those along her way among them.
RESUME_S, ALONG_COS = 1.5, math.cos(math.radians(50.0))
#: What she felt stays a while: her going toward where it pushed her from is cut for FELT_S after.
FELT_S = 3.0

#: A sole meets an edge EDGE_M and more over the floor her bearing soles stand on: walking a flat
#: floor her swinging soles were touched along it 7.9 mm over it at most - a landing's friction
#: 544 N -, the 35 mm sill's face 14 to 38 mm up. Its lowest point is its toes' tip as much
#: as its corners (TIPS, in the toes' frame): turning, toes scuffing the floor at 0.6 to 6.6 mm
#: read 17 to 35 mm by the corners alone, tripped her on a flat floor.
EDGE_M = 0.01
TIPS = tuple((x, -figure.TOE_RY, figure.TOE_M) for x in (-figure.SOLE_HALF, figure.SOLE_HALF))


def _ids(g):
    """Her segments' bodies in her law's model, `edge.SKIN`'s order, looked up once."""
    ids = g.get('_skin_ids')
    if ids is None:
        ids = g['_skin_ids'] = [g['body'].m.body(n).id for n in edge.SKIN]
    return ids


def _yield_arms(g, out, model, ids, touch, cur_doing):
    """When an arm or hand meets an obstruction opposing her walk (a doorway jamb), bleed its ESO,
    yield its shoulder and elbow backward (`g['posture']`, `g['ref_q']`), narrow her step width,
    and steer her heading slightly clear so she slips past without stopping."""
    import numpy as np
    from machine import latent, modal, se2
    dt = 0.001 * g['every']
    sh_h, ch_h = math.sin(g['heading']), math.cos(g['heading'])
    ay_prev = g.get('_arm_yield', (0.0, 0.0))
    walking = abs(float(out[latent.SWAY][modal.DRIVE])) > 0.06 and cur_doing not in se2.STUNT_PHASES
    ays = []
    for side in (0, 1):
        f_vec = np.sum([model.xmat[ids[k]].reshape(3, 3) @ touch[k] for k in ARMS[side]], axis=0)
        f_max = max(float(np.linalg.norm(touch[k])) for k in ARMS[side])
        along = float(f_vec[0]) * sh_h + float(f_vec[2]) * ch_h
        ob = walking and f_max > edge.TOUCHED_N and along < -12.0
        ays.append(min(1.0, min(1.0, f_max / 42.0) if ob else max(0.0, ay_prev[side] - 2.2 * dt)))
    g['_arm_yield'] = ay_l, ay_r = tuple(ays)
    ay_max = max(ay_l, ay_r)
    if ay_max <= 1e-3:
        if max(ay_prev) > 1e-3:
            g['posture'][[5, 6, 13, 14]] = 0.0
        return 0.0
    out[latent.SWAY][modal.ARMS] *= (1.0 - 0.85 * ay_max)
    out[latent.REF][modal.ARMS] *= (1.0 - 0.85 * ay_max)
    out[latent.RATE][modal.ARMS] *= (1.0 - 0.85 * ay_max)
    out[latent.REF][modal.WIDTH] = -0.35 * ay_max
    for ay, sh_idx, el_idx, sign in ((ay_l, 5, 6, -1.0), (ay_r, 13, 14, 1.0)):
        if ay > 1e-3:
            g['eso'][2][[sh_idx, el_idx]] = 0.0
            g['posture'][sh_idx], g['posture'][el_idx] = -0.55 * ay, 0.30 * ay
            g['ref_q'][sh_idx] = max(-0.75, g['ref_q'][sh_idx] + 0.45 * ay * (-0.55 - g['ref_q'][sh_idx]))
            g['ref_q'][el_idx] = min(0.85, g['ref_q'][el_idx] + 0.45 * ay * (0.30 - g['ref_q'][el_idx]))
            g['ref_v'][[sh_idx, el_idx]] *= (1.0 - 0.5 * ay)
            out[latent.REF][modal.TURN] += -sign * 0.45 * ay
            g['heading'] += sign * 0.75 * ay * dt
    return ay_max


def check_contacts(g, out, cur_doing):
    """A touch on her above her soles halts her drive and brakes her; a sole asked aloft that has
    come clear of everything, then is touched more along the floor than up it EDGE_M over it -
    an edge met, not the floor - raises her swings by how high it met it (`modal.RAISE`).
    Touched aloft at all, her toes leaving a flat floor tripped her 13 times in 40 s."""
    import numpy as np
    from machine import latent, modal, se2
    y, model, ids = g['sense'], g['body'].d, _ids(g)
    touch = y[edge.TOUCH].reshape(-1, 3)
    ay_max = _yield_arms(g, out, model, ids, touch, cur_doing)
    g['_upper_hit'] = bool((np.linalg.norm(touch[CORE], axis=1) > edge.TOUCHED_N).any())
    cx, cz = float(g['y'][modal.COM][0]), float(g['y'][modal.COM][2])
    if g['_upper_hit']:
        f = np.sum([model.xmat[ids[k]].reshape(3, 3) @ touch[k] for k in CORE], axis=0)
        fx, fz = float(f[0]), float(f[2])
        f_mag = math.hypot(fx, fz)
        x0, z0, t0 = g.setdefault('_felt_pos', (cx, cz, g['t']))
        v = g['est']['vel']
        adv = -((cx - x0) * fx + (cz - z0) * fz) / max(f_mag, 1e-9)
        v_against = -(float(v[0]) * fx + float(v[2]) * fz) / max(f_mag, 1e-9)
        yielding = adv > 0.04 or (ay_max > 0.25 and v_against > 0.20) or (g['t'] - t0 < 0.09 and v_against > 0.25)
        if yielding:
            g.pop('_felt', None)
            g['_resume_at'] = None
        else:
            g['_resume_at'], g['_felt'] = g['t'] + RESUME_S, (fx, fz, g['t'])
            way = -1.0 if (cur_doing in se2._BACK_GAITS or g.get('active_entry') in se2._BACK_GAITS
                           or g.get('twist', (0.0, 0.0, 0.0))[0] < -0.01) else 1.0
            gx, gz = way * math.sin(g['heading']), way * math.cos(g['heading'])
            driving = abs(float(out[latent.SWAY][modal.DRIVE])) > 0.06
            opposed = (fx * float(v[0]) + fz * float(v[2]) <= ALONG_COS * f_mag * math.hypot(float(v[0]), float(v[2]))
                       and fx * gx + fz * gz <= ALONG_COS * f_mag)
            if opposed:
                out[latent.SWAY][modal.DRIVE] = min(float(out[latent.SWAY][modal.DRIVE]), 0.04)
                out[latent.REF][modal.DRIVE] = g['a'][modal.GOING] = 0.0
            if opposed and driving and cur_doing not in ('brake', '#201', 'rest') and abs(float(g['st'][latent.NOW][latent.AMP][modal.DRIVE])) > 0.06:
                se2._dropped(g)
                se2.said(g, 'brake', g['_brake_wire'], ('brake',), at=g['t'] + 0.001, reset=True)
                if g['_stand_entry'] is not None:
                    se2.said(g, *g['_stand_entry'], at=g['t'] + 1.20, reset=True)
                g['active_entry'] = '#201'
    else:
        if '_felt' in g and '_felt_pos' in g:
            fx, fz, _at = g['_felt']
            x0, z0 = g['_felt_pos'][:2]
            if (cx - x0) * fx + (cz - z0) * fz < -0.04 * math.hypot(fx, fz):
                g.pop('_felt', None)
        g.pop('_felt_pos', None)
    aloft, clear = g['a'][modal.STANCE] < 0.5, list(g.get('_clear', (False, False)))
    tips = g.get('_tips')
    if tips is None:
        tips = g['_tips'] = np.array(TIPS)
    pushed, sizes, lowest, floors = g['soles'](g['body'], touch, ids, tips, g['y'], g['fr'], g['a'][modal.STANCE])
    for side in range(len(SOLES)):
        felt = max(float(n) for n in sizes[side]) > edge.TOUCHED_N
        up = min(float(g['y'][modal.LOWEST][side]), float(lowest[side])) - float(floors[side])
        met = clear[side] and up > EDGE_M and any(
            n > edge.TOUCHED_N and math.hypot(float(p[0]), float(p[2])) > abs(float(p[1]))
            for p, n in zip(pushed[side], sizes[side]))
        g['a'][modal.RAISE] = max(float(g['a'][modal.RAISE]), met * up)
        clear[side] = bool(aloft[side]) and (clear[side] or not felt)
    g['_clear'] = tuple(clear)


def soles(b, touch, ids, tips, y, fr, stance):
    """`check_contacts`' arithmetic of her soles on her law's body `b` as `wbc.sense` left it: (each
    sole's pushes, world - its foot's and its toes' -, their sizes, N; its toes' lowest tip and the
    floor under it, m) - `touch` the edge's (SKIN, 3), `ids` her segments' bodies, `tips` TIPS."""
    import numpy as np
    from machine import modal
    d = b.d
    pushed = [[d.xmat[ids[k]].reshape(3, 3) @ touch[k] for k in segs] for segs in SOLES]
    sizes = [[np.linalg.norm(p) for p in side] for side in pushed]
    lowest = [(d.xpos[ids[segs[1]]] + tips @ d.xmat[ids[segs[1]]].reshape(3, 3).T)[:, 1].min() for segs in SOLES]
    return pushed, sizes, lowest, modal.under(y, fr, stance)[0]


def against(g, tw):
    """`tw` (body frame) less any axis aimed within ALONG_COS into what she felt within FELT_S:
    a wall walked into, a hand that shoved her - she does not walk on into it."""
    fx, fz, at = g.get('_felt', (0.0, 0.0, -math.inf))
    size = math.hypot(fx, fz)
    if g['t'] - at > FELT_S or size < 1e-9:
        return tw
    nx, nz, sh, ch = fx / size, fz / size, math.sin(g['heading']), math.cos(g['heading'])
    vx = 0.0 if tw[0] * (sh * nx + ch * nz) < -ALONG_COS * abs(tw[0]) else tw[0]
    vy = 0.0 if tw[1] * (ch * nx - sh * nz) < -ALONG_COS * abs(tw[1]) else tw[1]
    return (vx, vy, tw[2])
