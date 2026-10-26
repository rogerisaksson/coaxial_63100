"""Her doing's flips, as `told.Run` counts them.

Each new doing and how long the last held; a doing come back to after a blip shorter than her
natural time, chatter, captured.
"""


def laid(meta):
    return {'natural_s': meta['natural_s'], 'doing': None, 'since': 0.0, 'before': None}


def step(st, snap):
    doing, t = snap['body']['doing'], float(snap['t'])
    if doing == st['doing']:
        return []
    out = []
    held = t - st['since']
    if st['doing'] is not None:
        out.append({'event': 'phase', 'from': st['doing'], 'to': doing, 'held_s': round(held, 3)})
    if doing == st['before'] and held < st['natural_s']:
        out.append({'event': 'chatter', 'between': [doing, st['doing']], 'blip_s': round(held, 3), 'capture': True})
    st['before'], st['doing'], st['since'] = st['doing'], doing, t
    return out
