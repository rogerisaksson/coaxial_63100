"""What she was told and how it went.

A say refused, in its own words; her errand's phases - laid, each phase, over.
"""


def laid(meta):
    return {'refused': None, 'errand': None}


def step(st, snap):
    body = snap['body']
    out = []
    if body['refused'] and body['refused'] != st['refused']:
        out.append({'event': 'refused', 'why': body['refused']})
    er = body['errand']
    was = st['errand']
    if er != was:
        out.append({'event': 'errand', **(er or {'kind': (was or {}).get('kind'), 'phase': 'over'})})
    st['refused'], st['errand'] = body['refused'], er
    return out
