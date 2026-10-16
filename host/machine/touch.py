"""What her skin and her soles feel: a touch that stops her going, a shove walked out, a trip.

    touch.check_contacts(g, out, doing)     # a pass of her law, `g` and `out` as `machine.law`'s
    touch.against(g, twist)                 # a twist less its part toward what she felt

Her law's own reading of the edge's touch (`machine.edge`): no body is named, nothing but forces
on her segments in their own frames, turned by her own model.
"""
import math

from machine import edge

#: Her segments under her knees, a side's: what bears on the floor, or trips over what is on it.
SOLES = ((edge.SKIN.index('left_foot'), edge.SKIN.index('left_toes')),
         (edge.SKIN.index('right_foot'), edge.SKIN.index('right_toes')))
ABOVE = [k for k in range(len(edge.SKIN)) if k not in SOLES[0] + SOLES[1]]


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
#: 544 N -, the 35 mm sill's face 14 to 38 mm up.
EDGE_M = 0.01


def check_contacts(g, out, cur_doing):
    """A touch on her above her soles halts her drive and brakes her; a sole asked aloft that has
    come clear of everything, then is touched more along the floor than up it EDGE_M over it -
    an edge met, not the floor - trips her. Touched aloft at all, her toes leaving a flat floor
    tripped her 13 times in 40 s."""
    import numpy as np
    from machine import latent, modal, sprung
    y, model = g['sense'], g['body'].d
    touch = y[edge.TOUCH].reshape(-1, 3)
    g['_upper_hit'] = bool((np.linalg.norm(touch[ABOVE], axis=1) > edge.TOUCHED_N).any())
    if g['_upper_hit']:
        ids = g.setdefault('_skin_ids', [g['body'].m.body(n).id for n in edge.SKIN])
        f = np.sum([model.xmat[ids[k]].reshape(3, 3) @ touch[k] for k in ABOVE], axis=0)
        g['_resume_at'], g['_felt'] = g['t'] + RESUME_S, (float(f[0]), float(f[2]), g['t'])
        v = g['est']['vel']
        opposed = (float(f[0]) * float(v[0]) + float(f[2]) * float(v[2])
                   <= ALONG_COS * math.hypot(float(f[0]), float(f[2])) * math.hypot(float(v[0]), float(v[2])))
        if opposed:
            out[latent.SWAY][modal.DRIVE] = min(float(out[latent.SWAY][modal.DRIVE]), 0.04)
            out[latent.REF][modal.DRIVE] = g['a'][modal.GOING] = 0.0
        if opposed and cur_doing not in ('brake', '#201', 'rest') and abs(float(g['st'][latent.NOW][latent.AMP][modal.DRIVE])) > 0.06:
            sprung._dropped(g)
            sprung.said(g, 'brake', g['_brake_wire'], ('brake',), at=g['t'] + 0.001, reset=True)
            if g['_stand_entry'] is not None:
                sprung.said(g, *g['_stand_entry'], at=g['t'] + 1.20, reset=True)
            g['active_entry'] = '#201'
    aloft, met, clear = g['a'][modal.STANCE] < 0.5, False, list(g.get('_clear', (False, False)))
    for side, segs in enumerate(SOLES):
        pushes = [model.xmat[g['body'].m.body(edge.SKIN[k]).id].reshape(3, 3) @ touch[k] for k in segs]
        felt = max(float(np.linalg.norm(p)) for p in pushes) > edge.TOUCHED_N
        up = float(g['y'][modal.LOWEST][side] - g['fr'][modal.FLOOR])
        met = met or (clear[side] and up > EDGE_M and any(
            np.linalg.norm(p) > edge.TOUCHED_N and math.hypot(float(p[0]), float(p[2])) > abs(float(p[1]))
            for p in pushes))
        clear[side] = bool(aloft[side]) and (clear[side] or not felt)
    g['_clear'] = tuple(clear)
    if met:
        g['stumble_until'], g['a'][modal.GOING] = g['t'] + 0.95, min(float(g['a'][modal.GOING]), 0.25)


def against(g, tw):
    """`tw` (body frame) less its part toward what she felt within FELT_S: a wall walked
    into, a hand that shoved her - she does not walk on into it."""
    fx, fz, at = g.get('_felt', (0.0, 0.0, -math.inf))
    size = math.hypot(fx, fz)
    if g['t'] - at > FELT_S or size < 1e-9:
        return tw
    nx, nz, sh, ch = fx / size, fz / size, math.sin(g['heading']), math.cos(g['heading'])
    wx, wz_ = tw[0] * sh + tw[1] * ch, tw[0] * ch - tw[1] * sh
    into = min(0.0, wx * nx + wz_ * nz)
    wx, wz_ = wx - into * nx, wz_ - into * nz
    return (wx * sh + wz_ * ch, wx * ch - wz_ * sh, tw[2])
