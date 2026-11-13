"""What the code says flows between the atlas's files: each system's loop, the wire, the calls.

    rows = atlas_flow.flows(atlas)       # [(from node, to node, kind, label)], the atlas's file nodes
    made = atlas_flow.makers(atlas)      # {stream: the atlas's node of the function making it}
    took = atlas_flow.takers(atlas)      # {(kind, stream): [the atlas's nodes of the functions taking it]}
    loops = atlas_flow.ticks(atlas)      # {a craft's tick: its loop's rows}
    asked = atlas_flow.called(atlas, files)  # [(from node, to node, kind, label)], what files call of each other

A system's loop is a pass and what comes back of it. The robot's (machine/rig/streams.py, STREAMS): each
stream made by a function, in the order one instant's records keep - the file making a stream hands on to the
file making the next, the stream's name on it -, and fed back: what the pass ends with (a stream on its end's
clock, the world it left) is what the robot's edge reads the next pass (a stream laid off its edge). A craft's
(`ticks`): the function stepping its law and its air once a tick, its calls in the order they run, and fed
back what its air ends the tick with. The wire (tools/dev/target_map.py, `ops`): a command from the host's
end of the wire to its handler and from the handler to each of the board's functions it reaches, and fed
back, what it read, from the handler to the host's end - the command's code on each. Where no loop is told,
what a system's functions call of each other's carries what it runs (`called`).
"""
import ast
import functools
import os

from tools.dev import target_map
from tools.dev.atlas_tree import REPO, TIES

#: The kinds of flow: a loop's pass, the wire, a call, what comes back - the last; where the robot's streams are
#: told; the names a craft's loop is declared on in its package (`sky, core = Sky(), Core()`), each its class.
KINDS, STREAMS = ('pass', 'wire', 'call', 'feedback'), os.path.join(REPO, 'host', 'machine', 'rig', 'streams.py')
LOOP = {'sky': 'Sky', 'core': 'Core'}


def streams(path=STREAMS):
    """[(name, 'module:function', whether its clock is the pass's end, whether it is laid off its edge)] of the
    robot's streams in their order, read off STREAMS' rows - Stream(name, by, clock, laid, ...)."""
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
    none}: where each of the robot's pass's words is made, in the tree."""
    out = {}
    for name, by, _end, _edge in streams():
        module, _, function = by.partition(':')
        out[name] = atlas.at.get('host/%s.py::%s' % (module.replace('.', '/'), function), -1)
    return out


@functools.lru_cache(maxsize=2)
def takers(atlas):
    """{(kind, stream): [the atlas's nodes of the functions taking it]} (KINDS, `makers`): in the robot's pass
    the next stream's maker, fed back - a stream on the pass's end's clock - those of the streams laid off its
    edge."""
    rows, made = streams(), makers(atlas)
    out = {(KINDS.index('pass'), name): [made[after]] for (name, *_r), (after, *_s) in zip(rows, rows[1:])}
    edge = [made[name] for name, _by, _end, laid in rows if laid]
    return {**out, **{(KINDS.index('feedback'), name): edge for name, _by, end, _laid in rows if end}}


def flows(atlas):
    """[(from node, to node, kind, label)]: the robot's pass's hand-ons and its loop, the wire's commands and what they
    bring back, between the atlas's files (KINDS), none from a file to itself."""
    passed, wire, back = (KINDS.index(kind) for kind in ('pass', 'wire', 'feedback'))
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


def called(atlas, files):
    """[(from node, to node, kind, label)]: what the functions of `files` - the atlas's file nodes - call of each
    other's (the atlas's calls, symbol to symbol), file to file, the function called on each, none from a file to
    itself - kind 'call' (KINDS)."""
    call, out = KINDS.index('call'), set()
    for key, callees in atlas.calls.items():
        a = atlas.at.get(key.partition('::')[0], -1)
        if a in files:
            for k in callees:
                path, _, name = k.partition('::')
                b = atlas.at.get(path, -1)
                if b in files and b != a:
                    out.add((a, b, call, name or path.rpartition('/')[2]))
    return sorted(out)


def _module(dotted):
    """The atlas's path of the module `dotted` names under host/, or None."""
    base = 'host/' + dotted.replace('.', '/')
    return next((p for p in (base + '.py', base + '/__init__.py') if os.path.isfile(os.path.join(REPO, p))), None)


@functools.lru_cache(maxsize=None)
def _named(path):
    """({name: the path of the module it names}, {name: (the path of the module it comes from, its name there)},
    {name: path} of its own top-level defs and classes): what a file's imports and defs bind."""
    with open(os.path.join(REPO, path), encoding='utf-8') as f:
        body = ast.parse(f.read()).body
    package, modules, symbols = path.rpartition('/')[0][5:].replace('/', '.'), {}, {}
    for node in body:
        if isinstance(node, ast.Import):
            modules.update((a.asname, _module(a.name)) for a in node.names if a.asname and _module(a.name))
        elif isinstance(node, ast.ImportFrom) and node.module is not None:
            source = package.rsplit('.', node.level - 1)[0] + '.' + node.module if node.level else node.module
            for a in node.names:
                inner, outer = _module(source + '.' + a.name), _module(source)
                if inner:
                    modules[a.asname or a.name] = inner
                elif outer:
                    symbols[a.asname or a.name] = (outer, a.name)
    return modules, symbols, {n.name: path for n in body if isinstance(n, (ast.FunctionDef, ast.ClassDef))}


def _class(modules, name):
    """The path of the file defining the class `name` one of `modules` ({alias: path}) defines or brings in."""
    for path in modules.values():
        _m, symbols, own = _named(path)
        if name in own:
            return path
        if name in symbols and name in _named(symbols[name][0])[2]:
            return symbols[name][0]
    return None


def _run(node, out):
    """The calls under `node` in the order they run - a receiver's and an argument's before the call they go
    into - into `out`."""
    if isinstance(node, ast.Call):
        _run(node.func.value if isinstance(node.func, ast.Attribute) else node.func, out)
        for arg in node.args + [k.value for k in node.keywords]:
            _run(arg, out)
        out.append(node)
        return
    for child in ast.iter_child_nodes(node):
        _run(child, out)


def _ran(fn, modules, symbols, own):
    """[(path or None, the LOOP name it lands on or None, the name called)]: a function's calls in the order they
    run, each where it lands - its module's file, one of its own module's functions' file, or on a name of LOOP,
    left for the loop's classes to place."""
    calls, out = [], []
    _run(fn, calls)
    for c in calls:
        f = c.func
        if isinstance(f, ast.Attribute) and isinstance(f.value, ast.Name):
            out.append((modules.get(f.value.id), f.value.id if f.value.id in LOOP else None, f.attr))
        elif isinstance(f, ast.Name):
            out.append((own.get(f.id) or symbols.get(f.id, (None, None))[0], None, f.id))
    return out


def ticks(atlas):
    """{tick: [(from node, to node, kind, label)]}: each craft's loop (`_loops`)."""
    return _loops(atlas)[0]


def handing(atlas):
    """{tick: the tick it hands its own sky and core on to} (`_loops`): the one a system runs where it reaches both."""
    return _loops(atlas)[1]


def flying(atlas):
    """{function: [tick]} (`_loops`): the ticks of the module a function gives on - a craft's flight module given
    to its runner, which runs it -: what a system flies where it comes to that function."""
    return _loops(atlas)[2]


def stepped(atlas):
    """{tick: [key]} (`_loops`): where running a tick is reached from - itself and the methods of its loop's
    classes it steps (LOOP), named on a parameter and so beyond its calls (atlas_calls.reach)."""
    return _loops(atlas)[3]


@functools.lru_cache(maxsize=2)
def _loops(atlas):
    """({tick: [(from node, to node, kind, label)]}, {tick: the tick it hands on to}, {function: [tick]}, {tick:
    [key]}): each craft's loop - its tick, a function stepping its law and its air, `core.step` and `sky.step` on the names its
    package declares its loop on (LOOP), or handing its own to such a function -: its calls in the order they run,
    file to file, the name called on each, and what it ends with back to where it began (kind 'feedback'). A call
    lands in its module's file, in one of its own module's functions' file, on a name of LOOP in the file of the
    class its tick's module brings by that name - its own module's where it hands them on -; others are left. A
    function giving a tick's module to a call flies its ticks: the one it is given to runs them, by its name."""
    passed, back, out, hands, flies, steps = KINDS.index('pass'), KINDS.index('feedback'), {}, {}, {}, {}
    paths = sorted(p for p in atlas.at if p.endswith('.py') and '::' not in p and p.startswith('host/'))
    texts = {}
    for path in paths:
        with open(os.path.join(REPO, path), encoding='utf-8') as f:
            texts[path] = f.read()

    def functions(path):
        return [n for n in ast.walk(ast.parse(texts[path])) if isinstance(n, ast.FunctionDef)]

    def rows(ran, loop):
        laid = []
        for where, name_of, name in ran:
            node = atlas.at.get(where or loop.get(name_of) or '', -1)
            if node >= 0 and (not laid or laid[-1][0] != node):
                laid.append((node, name))
        ways, closes = [(a, b, passed, name) for (a, _n), (b, name) in zip(laid, laid[1:])], len(laid) > 1
        return ways + ([(laid[-1][0], laid[0][0], back, laid[-1][1])] if closes and laid[-1][0] != laid[0][0] else [])

    def starts(key, ran, loop):   # the tick, and its loop's classes' methods it steps
        return [key] + sorted({loop[n] + '::' + LOOP[n] + '.' + a for _w, n, a in ran if n and loop.get(n)})
    direct = {}
    for path in (p for p in paths if 'sky.step(' in texts[p] and 'core.step(' in texts[p]):
        modules, symbols, own = _named(path)
        loop = {name: _class(modules, cls) for name, cls in LOOP.items()}
        for fn in functions(path):
            ran = _ran(fn, modules, symbols, own)
            if {('sky', 'step'), ('core', 'step')} <= {(n, a) for _w, n, a in ran if n}:
                key = path + '::' + fn.name
                direct[(path, fn.name)], out[key], steps[key] = ran, rows(ran, loop), starts(key, ran, loop)
    for path in (p for p in paths if any('.%s(' % name in texts[p] for _w, name in direct)):
        modules, symbols, own = _named(path)
        loop = {name: _class(modules, cls) for name, cls in LOOP.items()}
        for fn in functions(path):   # one handing its own sky and core on to a tick: that tick's loop on its own
            names = {a.arg for a in fn.args.args}
            handed = [(modules[c.func.value.id], c.func.attr) for c in ast.walk(fn) if isinstance(c, ast.Call)
                      and isinstance(c.func, ast.Attribute) and isinstance(c.func.value, ast.Name)
                      and (modules.get(c.func.value.id), c.func.attr) in direct]
            if set(LOOP) <= names and handed and (path, fn.name) not in direct:
                key, ran = path + '::' + fn.name, direct[handed[0]]
                out[key], steps[key], hands[key] = rows(ran, loop), starts(key, ran, loop), '::'.join(handed[0])
    held = {}
    for key in out:
        held.setdefault(key.partition('::')[0], []).append(key)
    named = {os.path.basename(m[:-12] if m.endswith('/__init__.py') else m[:-3]) for m in held}
    for path in (p for p in paths if any(name in texts[p] for name in named)):
        modules = _named(path)[0]
        for fn in functions(path):   # a tick's module given on as a value: its ticks run by the one given it
            given = sorted({m for c in ast.walk(fn) if isinstance(c, ast.Call)
                            for a in c.args + [k.value for k in c.keywords] if isinstance(a, ast.Name)
                            for m in (modules.get(a.id),) if isinstance(m, str) and m in held})
            if given:
                flies[path + '::' + fn.name] = [k for m in given for k in held[m]]
    return out, hands, flies, steps
