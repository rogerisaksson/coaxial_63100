"""What the code says flows between the atlas's files: her pass's streams, the wire's commands.

    rows = atlas_flow.flows(atlas)       # [(from node, to node, kind, label)], the atlas's file nodes
    made = atlas_flow.makers(atlas)      # {stream: the atlas's node of the function making it}
    took = atlas_flow.takers(atlas)      # {(kind, stream): [the atlas's nodes of the functions taking it]}

Her pass (machine/rig/streams.py, STREAMS): each stream made by a function, in the order one instant's
records keep - the file making a stream hands on to the file making the next, the stream's name on it -,
and fed back: what the pass ends with (a stream on its end's clock, the world it left) is what her edge
reads the next pass (a stream laid off her edge). The wire (tools/dev/target_map.py, `ops`): a command from
the host's end of the wire to its handler and from the handler to each of the board's functions it
reaches, and fed back, what it read, from the handler to the host's end - the command's code on each.
"""
import ast
import functools
import os

from tools.dev import target_map
from tools.dev.atlas_tree import REPO, TIES

#: The kinds of flow: her pass, the wire, what comes back; where her pass's streams are told.
KINDS, STREAMS = ('pass', 'wire', 'feedback'), os.path.join(REPO, 'host', 'machine', 'rig', 'streams.py')


def streams(path=STREAMS):
    """[(name, 'module:function', whether its clock is the pass's end, whether it is laid off her edge)] of her
    pass's streams in their order, read off STREAMS' rows - Stream(name, by, clock, laid, ...)."""
    with open(path, encoding='utf-8') as f:
        top = ast.parse(f.read())
    out = []
    for node in top.body:
        if isinstance(node, ast.Assign) and any(getattr(t, 'id', '') == 'STREAMS' for t in node.targets):
            for row in getattr(node.value, 'elts', []):
                args = row.args if isinstance(row, ast.Call) else []
                name, by = (getattr(a, 'value', None) for a in (args + [None, None])[:2])
                clock, laid = (args + [None, None, None, None])[2:4]
                end = isinstance(clock, ast.Call) and any(k.arg == 'end' and getattr(k.value, 'value', None) is True
                                                          for k in clock.keywords)
                edge = isinstance(laid, ast.Call) and getattr(laid.func, 'id', '') == '_edge'
                if isinstance(name, str) and isinstance(by, str) and ':' in by:
                    out.append((name, by, end, edge))
    return out


def _file(atlas, dotted):
    """The atlas's node of the host module `dotted`: its file, or its package's __init__.py; -1 for none."""
    base = 'host/' + dotted.replace('.', '/')
    return atlas.at.get(base + '.py', atlas.at.get(base + '/__init__.py', -1))


@functools.lru_cache(maxsize=2)
def makers(atlas):
    """{stream: the atlas's node of the function making it (STREAMS' 'module:function'), -1 where it has
    none}: where each of her pass's words is made, in the tree."""
    out = {}
    for name, by, _end, _edge in streams():
        module, _, function = by.partition(':')
        out[name] = atlas.at.get('host/%s.py::%s' % (module.replace('.', '/'), function), -1)
    return out


@functools.lru_cache(maxsize=2)
def takers(atlas):
    """{(kind, stream): [the atlas's nodes of the functions taking it]} (KINDS, `makers`): in her pass the
    next stream's maker, fed back - a stream on the pass's end's clock - those of the streams laid off her
    edge."""
    rows, made = streams(), makers(atlas)
    out = {(KINDS.index('pass'), name): [made[after]] for (name, *_r), (after, *_s) in zip(rows, rows[1:])}
    edge = [made[name] for name, _by, _end, laid in rows if laid]
    return {**out, **{(KINDS.index('feedback'), name): edge for name, _by, end, _laid in rows if end}}


def flows(atlas):
    """[(from node, to node, kind, label)]: her pass's hand-ons and its loop, the wire's commands and what they
    bring back, between the atlas's files (KINDS), none from a file to itself."""
    passed, wire, back = (KINDS.index(kind) for kind in KINDS)
    made = [(name, _file(atlas, by.partition(':')[0]), end, edge) for name, by, end, edge in streams()]
    out = [(a, b, passed, name) for (name, a, _e, _d), (_n, b, _f, _g) in zip(made, made[1:])
           if a >= 0 and b >= 0 and a != b]
    out += [(a, b, back, name) for name, a, end, _edge in made if end and a >= 0
            for _n, b, _end, edge in made if edge and b >= 0 and a != b]
    host = {}
    for s, d, t in zip(atlas.src, atlas.dst, atlas.tie):
        if TIES[t] == 'wire':
            near, far = (s, d) if atlas.id[s].startswith('host/') else (d, s)
            host[far] = near
    where = target_map.definers()
    for code, _word, rel, _fn, calls in target_map.ops():
        handler = atlas.at.get(rel, -1)
        if handler < 0:
            continue
        out += [(host[handler], handler, wire, code), (handler, host[handler], back, code)] if handler in host else []
        out += [(handler, atlas.at[where[c]], wire, code) for c in calls
                if where.get(c) in atlas.at and atlas.at[where[c]] != handler]
    return out
