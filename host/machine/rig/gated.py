"""A gate's file read back: its gates and their streams' samples, each gate told short.

    gates = gated.read(path)                      # a file's gates: meta, trailer, each stream's samples
    said = gated.told(gates[0], ('felt.QD',))     # a gate told short, a field's largest entry by name
"""
import json


def read(path):
    """The gates of the file at `path`: [{'meta', 'trailer', 'streams': {name: (times, samples)}}] -
    a vector stream's samples an array (its count, its width), a record stream's a list."""
    import numpy as np
    gates = []
    with open(path, encoding='utf-8') as f:
        for line in f:
            rec = json.loads(line)
            gates.extend([{'meta': rec['gate'], 'trailer': None, 'streams': {}}] if 'gate' in rec else [])
            gates[-1]['trailer'] = rec.get('gated', gates[-1]['trailer'])
            got = gates[-1]['streams'].setdefault(rec['s'], ([], [])) if 's' in rec else None
            got and (got[0].append(rec['t']), got[1].append(rec['v']))
    for g in gates:
        vector = {n for n, m in g['meta']['streams'].items() if m['kind'] == 'vector'}
        g['streams'] = {n: (np.array(ts), np.array(vs, float).reshape(len(ts), -1) if n in vector else vs)
                        for n, (ts, vs) in g['streams'].items()}
    return gates


def told(g, fields=()):
    """A gate read (`read`) told short: its trigger, tag, window, cut and held off; each stream's
    count against the expected; each of `fields` ('stream.FIELD') its largest entry, when, by name."""
    import numpy as np
    m, tr = g['meta'], g['trailer'] or {}
    out = {'n': m['n'], 'name': m['name'], 'tag': m['tag'], 't': m['t'], 'window': m['window'], 'cut': tr.get('cut'),
           'held_off': tr.get('held_off'), 'records': {n: [len(g['streams'].get(n, ((), ()))[0]), d.get('expected')]
                                                       for n, d in m['streams'].items()}}
    for name in fields:
        stream, field = name.split('.', 1)
        if not len(g['streams'].get(stream, ((), ()))[0]):
            out[name] = None
            continue
        d, (ts, vs) = m['streams'][stream], g['streams'][stream]
        f = d['fields'][field]
        block = vs[:, f['at']:f['at'] + f['n']]
        i, j = np.unravel_index(int(np.nanargmax(np.abs(block))), block.shape)
        labels = d['names'].get(f.get('of'), [])
        out[name] = {'value': float(block[i, j]), 't': float(ts[i]),
                     'entry': labels[j // f.get('each', 1)] if labels else int(j)}
    return out
