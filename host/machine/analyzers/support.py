"""What bears her, as `told.Run.borne` and the fall probes read it.

Her soles' share of her weight crossing half and holding there her natural time - on her soles, or
borne by what else she lies or sits on; a landing's bounce about half no change -; a fall,
captured, where she was and what she did.
"""


def laid(meta):
    return {'weight': meta['weight'], 'natural_s': meta['natural_s'], 'soles': None, 'next': None,
            'next_at': 0.0, 'fallen': False}


def step(st, snap):
    body, t = snap['body'], float(snap['t'])
    out = []
    share = sum(body['loads']) / st['weight']
    on = share >= 0.5
    if st['soles'] is None:
        st['soles'] = on
    if on != st['soles'] and on != st['next']:
        st['next'], st['next_at'] = on, t
    if on == st['soles']:
        st['next'] = None
    if st['next'] is not None and t - st['next_at'] >= st['natural_s']:
        out.append({'event': 'borne', 'on_soles': on, 'soles_share': round(share, 3), 'doing': body['doing'],
                    'since': round(st['next_at'], 3)})
        st['soles'], st['next'] = on, None
    if body['fallen'] and not st['fallen']:
        out.append({'event': 'fall', 'doing': body['doing'], 'where': [round(v, 3) for v in body['where']],
                    'soles_share': round(share, 3), 'capture': True})
    st['fallen'] = body['fallen']
    return out
