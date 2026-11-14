"""What of her strikes what she falls on, how hard and how fast: the world's truth (`struck`).

From her stumble (her surety under where her brace takes her over, `meta['brace']`) or her fall, each of her parts
but her soles' (FEET) told as it first touches what is not hers - the force on it and how fast it closed on it -,
the fall's first captured; each part's hardest kept, and once she is borne off her soles her natural time
(`meta['natural_s']`) her landing told: her parts in the order they struck, each its hardest force and speed.
"""

#: Her soles' bodies: what bears her standing, no landing's.
FEET = ('left_foot', 'left_toes', 'right_foot', 'right_toes')


def laid(meta):
    return {'brace': meta['brace'], 'weight': meta['weight'], 'natural_s': meta['natural_s'], 'falling': False,
            'order': [], 'hardest': {}, 'off_at': None, 'told': False}


def step(st, snap):
    body, t = snap['body'], float(snap['t'])
    falling = body['surety'] < st['brace'] or body['fallen']
    if not falling and not st['falling']:
        return []
    if falling and not st['falling']:
        st.update(order=[], hardest={}, off_at=None, told=False)
    st['falling'] = falling
    out = []
    for part, (force, speed) in snap['ground'].items():
        if part in FEET:
            continue
        was = st['hardest'].get(part)
        if was is None:
            out.append({'event': 'touch', 'part': part, 'force_n': round(force, 1), 'speed_m_s': round(speed, 3),
                        'doing': body['doing'], 'capture': not st['order']})
            st['order'].append(part)
            was = (0.0, 0.0)
        st['hardest'][part] = (max(was[0], force), max(was[1], speed))
    off = sum(body['loads']) < 0.5 * st['weight']
    st['off_at'] = (st['off_at'] if st['off_at'] is not None else t) if off else None
    if off and not st['told'] and t - st['off_at'] >= st['natural_s']:
        st['told'] = True
        out.append({'event': 'landed', 'order': list(st['order']),
                    'hardest': {p: [round(f, 1), round(v, 3)] for p, (f, v) in st['hardest'].items()}})
    return out
