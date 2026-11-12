"""What the code says flows between the atlas's files: her pass's streams, the wire's commands.

    rows = atlas_flow.flows(atlas)       # [(from node, to node, kind, label)], the atlas's file nodes

Her pass (machine/rig/streams.py, STREAMS): each stream made by a function, in the order one instant's
records keep - the file making a stream hands on to the file making the next, the stream's name on it.
The wire (tools/dev/target_map.py, `ops`): a command from the host's end of the wire to its handler, and
from the handler to each of the board's functions it reaches, the command's code on each.
"""
import ast
import os

from tools.dev import target_map
from tools.dev.atlas_tree import REPO, TIES

#: The kinds of flow: her pass, the wire; where her pass's streams are told.
KINDS, STREAMS = ('pass', 'wire'), os.path.join(REPO, 'host', 'machine', 'rig', 'streams.py')


def streams(path=STREAMS):
    """[(name, 'module:function')] of her pass's streams in their order, read off STREAMS' rows - a row's
    first word its name, its second what makes it."""
    with open(path, encoding='utf-8') as f:
        top = ast.parse(f.read())
    for node in top.body:
        if isinstance(node, ast.Assign) and any(getattr(t, 'id', '') == 'STREAMS' for t in node.targets):
            rows = [[c.value for c in ast.walk(row) if isinstance(c, ast.Constant) and isinstance(c.value, str)]
                    for row in getattr(node.value, 'elts', [])]
            return [(row[0], row[1]) for row in rows if len(row) > 1 and ':' in row[1]]
    return []


def _file(atlas, dotted):
    """The atlas's node of the host module `dotted`: its file, or its package's __init__.py; -1 for none."""
    base = 'host/' + dotted.replace('.', '/')
    return atlas.at.get(base + '.py', atlas.at.get(base + '/__init__.py', -1))


def flows(atlas):
    """[(from node, to node, kind, label)]: her pass's hand-ons and the wire's commands between the atlas's
    files (KINDS), none from a file to itself."""
    made = [(name, _file(atlas, by.partition(':')[0])) for name, by in streams()]
    out = [(a, b, KINDS.index('pass'), name) for (name, a), (_next, b) in zip(made, made[1:])
           if a >= 0 and b >= 0 and a != b]
    host = {}
    for s, d, t in zip(atlas.src, atlas.dst, atlas.tie):
        if TIES[t] == 'wire':
            near, far = (s, d) if atlas.id[s].startswith('host/') else (d, s)
            host[far] = near
    where, wire = target_map.definers(), KINDS.index('wire')
    for code, _word, rel, _fn, calls in target_map.ops():
        handler = atlas.at.get(rel, -1)
        if handler < 0:
            continue
        out += [(host[handler], handler, wire, code)] if handler in host else []
        out += [(handler, atlas.at[where[c]], wire, code) for c in calls
                if where.get(c) in atlas.at and atlas.at[where[c]] != handler]
    return out
