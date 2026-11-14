"""What of its strikes what it falls on, how hard and how fast: the world's truth (`struck`).

From its stumble (its surety under where its brace takes it over, `meta['brace']`) or its fall, each of its parts
but its soles' (FEET) told as it first touches what is not its own - the force on it and how fast it closed on it -,
the fall's first captured; each part's hardest kept, and once it is borne off its soles its natural time
(`meta['natural_s']`) its landing told: its parts in the order they struck, each its hardest force and speed.
"""

#: The robot's soles' bodies: what bears its standing, no landing's.
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
