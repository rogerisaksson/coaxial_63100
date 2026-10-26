"""Her joints' motion a phase, on the peaks held between snapshots.

The spasm probes' and `told.Run.arms`' reduction: at each phase's end its fastest joint and how fast
against its drive's rated speed; a joint past its drive's rating told as it happens, once a joint a
phase, captured - no drive turns it so: it is thrown.
"""


def laid(meta):
    return {'rated': meta['rated'], 'doing': None, 'since': 0.0, 'worst': (0.0, None), 'past': set()}


def _closed(st, t):
    deg_s, joint = st['worst']
    if st['doing'] is None or joint is None:
        return []
    rated = st['rated'].get(joint)
    return [{'event': 'motion', 'phase': st['doing'], 'held_s': round(t - st['since'], 3), 'fastest': joint,
             'deg_s': round(deg_s, 1), 'of_rated': None if rated is None else round(deg_s / rated, 3)}]


def step(st, snap):
    body, t = snap['body'], float(snap['t'])
    out = []
    if body['doing'] != st['doing']:
        out = _closed(st, t)
        st['doing'], st['since'], st['worst'], st['past'] = body['doing'], t, (0.0, None), set()
    for joint, peak in body['peaks'].items():
        st['worst'] = max(st['worst'], (float(peak), joint))
        rated = st['rated'].get(joint)
        if rated is not None and peak > rated and joint not in st['past']:
            st['past'].add(joint)
            out.append({'event': 'past_rated', 'phase': body['doing'], 'joint': joint, 'deg_s': round(peak, 1),
                        'rated': rated, 'capture': True})
    return out
