"""Her joints' motion a phase, on the peaks held between snapshots.

The spasm probes' and `told.Run.arms`' reduction: at each phase's end its fastest joint and how fast
against its drive's rated speed; a joint past its drive's rating told as it happens, once a joint a
phase, captured - no drive turns it so: it is thrown; a driven joint swung faster than her own walk
swings any (WALK_DEG_S), once a joint a phase, captured - a spasm, a jerk: no gait of hers asks it.
Her toes, wrists and grippers no drive turns: a footfall throws her toes 870 deg/s.
"""

#: The fastest her own walk swings a driven joint, deg/s: her knee over 8 s of a flat walk, 464.4, measured
#: (`test_introspect`); a slip's arms 620, an obstacle's 1027, her bed's legs 422.
WALK_DEG_S = 465.0


def laid(meta):
    return {'rated': meta['rated'], 'doing': None, 'since': 0.0, 'worst': (0.0, None), 'past': set(), 'spasmed': set()}


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
        st['doing'], st['since'], st['worst'], st['past'], st['spasmed'] = body['doing'], t, (0.0, None), set(), set()
    for joint, peak in body['peaks'].items():
        # the faster alone: a peak tied with the phase's seed (0.0, None) compared a name with None and threw (fallen still)
        st['worst'] = (float(peak), joint) if float(peak) > st['worst'][0] else st['worst']
        rated = st['rated'].get(joint)
        if rated is not None and peak > rated and joint not in st['past']:
            st['past'].add(joint)
            out.append({'event': 'past_rated', 'phase': body['doing'], 'joint': joint, 'deg_s': round(peak, 1),
                        'rated': rated, 'capture': True})
        if rated is not None and peak > WALK_DEG_S and joint not in st['spasmed']:
            st['spasmed'].add(joint)
            out.append({'event': 'spasm', 'phase': body['doing'], 'joint': joint, 'deg_s': round(peak, 1),
                        'walk_deg_s': WALK_DEG_S, 'capture': True})
    return out
