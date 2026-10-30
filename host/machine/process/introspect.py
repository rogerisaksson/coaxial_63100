"""Her loop's latent, impedance and wrench layers live, for a debugger or the local model to attach to.

    snap = introspect.snapshot(s)                 # a law pass's layers and her body, named, JSON-able
    server = introspect.attach(s, port)            # serve them on 127.0.0.1:port, a JSON line a pass
    rows = introspect.read(port, n=10)             # a client: the next n snapshots
    told = introspect.events(port, seconds=8)       # her analyzers' events over 8 s (`watch`)
    notices = introspect.gated(port, {'tag': 'lie', 'count': 2}, 30)   # a gate armed on them (`gate`)
    python -m machine.process.introspect --attach 8765     # the same from a shell, a JSON line each
    python -m machine.process.introspect --attach 8765 --events   # her analyzers' events alone, a line each
    python -m machine.process.introspect --attach 8765 --gate support.fall --tag fall   # a gate's notices

The layers as her loop leaves them each law pass (`motion.control`): latent - her state's rows and
her latent out (`latent.step`); impedance - the modal ask (`impedance.ask`) and the step's ask it
makes; wrench - the whole-body step's answer (`wrench.step`: tau, qacc, contact forces, ..); observer -
what bears her as her observer splits it, against her skin (`observer.named`). Traced (`calls.Calls`), a
snapshot carries what her law called since the last, a count each. Each
array's fields are named from her code's own layout, generated (`layout`): the run of module-level
slice and int constants ending at a vector's size, laid in rising offsets - none listed by hand.
Attached, a hook on her passes (`worker`'s `s['each']`) publishes a snapshot each law pass to every
client connected, dropped where a client lags, the loop never waiting; none connected, the hook
returns at its first line. A line a client sends asks instead for its analyzers' events, run
beside her loop ({'watch': {'folder'}}), or arms a gate on them ({'gate': spec}, `machine.rig.gate`):
her streams on their own clocks, a trigger's windows before and after it written to a file. Opt in
on the page: `show_humanoid --telemetry PORT`.
"""
import ast
import contextlib
import functools
import importlib
import inspect
import json
import queue
import socket
import socketserver
import sys
import threading
import time
import typing

#: A client's queue of snapshots; past it the oldest is dropped - the loop never waits on a reader.
QUEUED = 64


@functools.lru_cache(maxsize=None)
def layout(name, size, end=None):
    """{field: slice} of module `name`'s vector of `size` (the name of its size constant): the run of
    module-level assignments before the one binding `end` (else `size`), each binding slices or ints
    within the size, in rising offsets - an int a slice of one."""
    module = importlib.import_module(name)
    tree = ast.parse(inspect.getsource(module))
    stmts = [n for n in tree.body if isinstance(n, ast.Assign)]
    bound = lambda n: [t.id for target in n.targets for t in ([target] if isinstance(target, ast.Name) else
                                                                getattr(target, 'elts', [])) if isinstance(t, ast.Name)]
    k = next(i for i, n in enumerate(stmts) if (end or size) in bound(n))
    limit, out, floor = int(getattr(module, size)), {}, None
    for n in reversed(stmts[:k]):
        names = bound(n)
        spans = [sp for sp in (_span(getattr(module, a, None)) for a in names) if sp is not None]
        if not names or len(spans) < len(names) or any(sp.start < 0 or sp.stop > limit for sp in spans):
            break
        if floor is not None and max(sp.stop for sp in spans) > floor:
            break
        floor = min(sp.start for sp in spans)
        out.update(zip(names, spans))
    return dict(sorted(out.items(), key=lambda kv: kv[1].start))


@functools.lru_cache(maxsize=None)
def counted(name, size, end=None):
    """{field: {'of': the tuple of names its entries are ('module.NAME'), 'each': their components}}
    of `layout(name, size, end)`'s fields whose definition counts one tuple of names - `len(X)` in
    it, or a constant counting one, through modules - its width a whole multiple of: by her code's
    own definitions, none guessed from a width. Names-major: an entry's components together."""
    module, out = importlib.import_module(name), {}
    for field, sp in layout(name, size, end).items():
        width = sp.stop - sp.start
        of = ['%s.%s' % k for k in _counts(module, field) if width % len(names_of('%s.%s' % k)) == 0]
        out.update({field: {'of': of[0], 'each': width // len(names_of(of[0]))}} if len(of) == 1 else {})
    return out


def names_of(qualified):
    """The tuple of names 'module.NAME' is."""
    module, _, attr = qualified.rpartition('.')
    return getattr(importlib.import_module(module), attr)


@functools.lru_cache(maxsize=None)
def _bound(name):
    """{name: the expression bound to it} of module `name`'s module-level assignments: a tuple of
    names each its element where the value is a tuple as long, else the whole value."""
    out = {}
    for n in [n for n in ast.parse(inspect.getsource(importlib.import_module(name))).body if isinstance(n, ast.Assign)]:
        for target in n.targets:
            elts = target.elts if isinstance(target, ast.Tuple) else [target]
            values = (n.value.elts if isinstance(n.value, ast.Tuple) and len(n.value.elts) == len(elts)
                      else [n.value] * len(elts))
            out.update({t.id: v for t, v in zip(elts, values) if isinstance(t, ast.Name)})
    return out


def _referred(module, node):
    """(the module, the name) a Name, or a module's Attribute, reads in `module`; else None."""
    inner = (getattr(module, node.value.id, None)
             if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name) else None)
    return (module, node.id) if isinstance(node, ast.Name) else (inner, node.attr) if inspect.ismodule(inner) else None


def _len_of(node):
    """What `node` counts where it is `len(X)`: X; else None."""
    if not (isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == 'len'):
        return None
    return node.args[0] if len(node.args) == 1 else None


def _counts(module, attr, seen=frozenset()):
    """{(module name, name)} of the tuples of names the definition of `attr` in `module` counts:
    `len(X)` in it, or an int it reads whose own definition counts one."""
    seen, out = seen | {(module.__name__, attr)}, set()
    expr = _bound(module.__name__).get(attr)
    alias = None if expr is None else _referred(module, expr)
    if alias is not None and (alias[0].__name__, alias[1]) not in seen:
        return _counts(alias[0], alias[1], seen)
    for node in ast.walk(expr or ast.Pass()):
        counted_arg = _len_of(node)
        counting = counted_arg is not None
        ref = _referred(module, node if counted_arg is None else counted_arg)
        if ref is None:
            continue
        value, key = getattr(ref[0], ref[1], None), (ref[0].__name__, ref[1])
        names = isinstance(value, tuple) and value and all(isinstance(x, str) for x in value)
        out |= {key} if counting and names else set()
        deeper = not counting and isinstance(value, int) and not isinstance(value, bool) and key not in seen
        out |= _counts(ref[0], ref[1], seen) if deeper else set()
    return out


def _span(v):
    """A constant as the slice it names: a slice itself, an int one wide; else None."""
    import numpy as np
    if isinstance(v, slice) and isinstance(v.start, int) and isinstance(v.stop, int):
        return v
    if isinstance(v, (int, np.integer)) and not isinstance(v, bool):
        return slice(int(v), int(v) + 1)
    return None


def _named(vec, fields, of=None):
    """{field: its value, or its values - by name where `of` (`counted`) says what its entries are,
    an entry's components a list} of the array `vec` by `fields`."""
    import numpy as np
    vec, of = np.asarray(vec, float).ravel(), of or {}
    width = lambda sp: sp.stop - sp.start
    by = lambda name, sp: dict(zip(names_of(of[name]['of']), vec[sp].reshape(-1, of[name]['each']).tolist()
                                   if of[name]['each'] > 1 else vec[sp].tolist()))
    return {name: (float(vec[sp.start]) if width(sp) == 1 else by(name, sp) if name in of else vec[sp].tolist())
            for name, sp in fields.items()}


def _plain(v):
    """A value JSON can carry: arrays and numbers as lists and floats, the rest by its string."""
    import numpy as np
    if isinstance(v, np.ndarray):
        return v.tolist()
    if isinstance(v, (np.floating, np.integer, np.bool_)):
        return v.item()
    if isinstance(v, (list, tuple)):
        return [_plain(x) for x in v]
    if isinstance(v, dict):
        return {str(k): _plain(x) for k, x in v.items()}
    return v if isinstance(v, (int, float, str, bool)) or v is None else str(v)


def body(s, peaks=None):
    """Her body as her page shows it, a pass of her session `s`: her joints' angles (deg) and speeds
    (deg/s) by name (`figure.JOINTS`, the world's addresses), their fastest since the last snapshot
    where `peaks` (deg/s) says, else their speeds; her soles' loads (N), her pelvis's place (m), what
    she does, fallen, a say refused, her epoch, her errand's phase."""
    import numpy as np
    from machine.process import sprung
    from machine.body.figure import JOINTS
    g = s['g']
    world, d = g['world'], g['world'].data
    er = g.get('_errand') or {}
    speeds = np.degrees(d.qvel[world.vadr])
    return {'angles': dict(zip(JOINTS, np.degrees(d.qpos[world.qadr]).tolist())),
            'speeds': dict(zip(JOINTS, speeds.tolist())),
            'peaks': dict(zip(JOINTS, (np.abs(speeds) if peaks is None else peaks).tolist())),
            'loads': [float(v) for v in g['loads']], 'where': d.qpos[0:3].tolist(), 'doing': sprung.doing(g),
            'fallen': s.get('fallen_at') is not None, 'refused': s.get('refused'), 'epoch': int(s.get('epoch', 0)),
            'errand': {'kind': er.get('kind'), 'phase': er.get('phase')} if er else None}


def snapshot(s, peaks=None, lazy=False) -> typing.Any:
    """A law pass of her session `s`, named: {'t', 'doing', 'body': `body`, 'latent': {'state': {row:
    {field: ..}}, 'out': {..}}, 'impedance': {'ask': {..}, 'step_ask': {..}}, 'wrench': {..}}; `lazy`,
    its layers named as they are first read (`Snap`)."""
    from machine.process import sprung
    g = s['g']
    snap = Snap(t=float(g['t']), doing=sprung.doing(g), body=body(s, peaks))
    snap.s = s
    return snap if lazy else snap.whole()


def _layers(s):
    """Her law's layers this pass, named: latent, impedance, wrench, observer."""
    import numpy as np
    from machine.senses import observer
    g = s['g']
    row, state, out, ask = (layout('machine.law.latent', 'P', 'NOW'), layout('machine.law.latent', 'STATE'),
                            layout('machine.law.latent', 'OUT'), layout('machine.law.modal', 'ASK'))
    st = np.asarray(g['st'], float).ravel()
    rows = {name: _named(st[sp], row, counted('machine.law.latent', 'P', 'NOW'))
            for name, sp in state.items() if sp.stop - sp.start == row_len(row)}
    rest = {name: v for name, v in _named(st, state, counted('machine.law.latent', 'STATE')).items() if name not in rows}
    return {'latent': {'state': {**rows, **rest}, 'out': _named(g['out'], out, counted('machine.law.latent', 'OUT'))},
            'impedance': {'ask': _named(g['a'], ask, counted('machine.law.modal', 'ASK')), 'step_ask': _plain(g.get('ask', {}))},
            'wrench': _plain(g.get('got', {})), 'observer': observer.named(g)}


class Snap(dict):

    """A snapshot whose layers are named as they are first read (`snap['latent']`), in the pass it
    was taken: an analyzer reading her body alone pays her body's 11 us, not all layers' 134."""

    s: typing.Any = None

    def __missing__(self, key):
        self.update(_layers(self.s) if key in LAYERS else {})
        return dict.__getitem__(self, key)

    def whole(self):
        """Every layer named: a plain dict."""
        return dict(self, **{k: self[k] for k in LAYERS})


#: Her law's layers a snapshot names.
LAYERS = ('latent', 'impedance', 'wrench', 'observer')


def row_len(fields):
    """The length a row of `fields` spans."""
    return max(sp.stop for sp in fields.values())


class _Clients(socketserver.ThreadingTCPServer):
    daemon_threads, allow_reuse_address = True, True

    def __init__(self, address):
        super().__init__(address, _Client)
        self.clients, self.commands, self.lock = [], queue.Queue(), threading.Lock()


class _Client(socketserver.StreamRequestHandler):

    """A client: a snapshot a JSON line each law pass, till a line it sends asks for its analyzers'
    events ({'watch': {'folder'}}) or arms a gate on them ({'gate': spec}, `gate.Scope.arm`)."""

    def handle(self):
        srv = typing.cast(_Clients, self.server)
        c = {'q': queue.Queue(QUEUED), 'watch': None, 'gate': None}
        with srv.lock:
            srv.clients.append(c)
        threading.Thread(target=self._asked, args=(srv, c), daemon=True).start()
        try:
            for line in iter(c['q'].get, None):
                self.wfile.write(line + b'\n')
        except OSError:
            pass
        finally:
            with srv.lock:
                srv.clients.remove(c)
            srv.commands.put((c, None))

    def _asked(self, srv, c):
        """The client's lines, each a command to her loop's hook; their end, its writer's."""
        with contextlib.suppress(OSError, ValueError):
            for line in self.rfile:
                srv.commands.put((c, json.loads(line)))
        _put(c['q'], None)


def _put(q, item):
    """`item` onto a client's queue `q`, its oldest dropped where full: her loop never waits on a reader."""
    while True:
        try:
            return q.put_nowait(item)
        except queue.Full:
            with contextlib.suppress(queue.Empty):
                q.get_nowait()


def _commanded(s, scope, c, command):
    """A client's `command` at a pass of her session `s`: its analyzers' events ({'watch': {'folder'}}),
    a gate armed on them ({'gate': spec}); None - the client gone -, its gate taken off."""
    from machine.process import watch
    from machine.errors import MachineError
    scope.disarm(s, c['gate'])
    c['gate'] = None
    if command is None:
        return
    spec = command.get('gate')
    folder = (command.get('watch') or spec or {}).get('folder')
    c['watch'] = c['watch'] or watch.Watch(folder or watch.FOLDER)
    told = lambda notice: _put(c['q'], json.dumps(_plain(notice)).encode('utf-8'))
    told({'t': float(s['g']['t']), 'analyzer': 'watch', 'event': 'watching', 'folder': c['watch'].folder})
    try:
        c['gate'] = None if spec is None else scope.arm(s, {k: v for k, v in spec.items() if k != 'folder'}, told)
    except MachineError as error:
        told({'t': float(s['g']['t']), 'analyzer': 'gate', 'event': 'refused', 'why': str(error)})


class Tap:

    """Her passes into snapshots, a peak detector between: every pass her joints' fastest held, a
    snapshot each law period of her world's time carrying them since the last - fallen, her law
    stands still and a snapshot a law pass told no fall; one a law pass alone read the tip's fastest
    joint 249.7 deg/s where every pass's 270.7."""

    def __init__(self):
        self.t, self.peaks = -float('inf'), None

    def passed(self, s):
        """A pass of her session `s`: its snapshot (`snapshot`, lazy; the peaks since the last) where a
        law period of her world's time has gone since the last - a new epoch's time from its start -,
        else None."""
        import numpy as np
        from machine.process import worker
        g = s['g']
        world = g['world']
        v = np.abs(world.data.qvel[world.vadr])
        self.peaks = v if self.peaks is None else np.maximum(self.peaks, v)
        t = float(g['t'])
        if self.t <= t < self.t + worker.DT * g['every']:
            return None
        snap, self.peaks, self.t = snapshot(s, np.degrees(self.peaks), lazy=True), None, t
        if 'calls' in s:
            snap['calls'] = s['calls'].take()
        return snap


def attach(s, port, host='127.0.0.1'):
    """Serve her session `s`'s layers on `host:port` (0 a free one; the server's `server_address`
    says which), a hook on her passes (`s['each']`) returning at once with none connected: each
    client a snapshot a JSON line each law pass (`Tap`), or, asked, its analyzers' events (a
    `watch.Watch` beside her loop) and its gate's notices (`gate.Scope`). The server."""
    from machine.rig import gate
    server, tap, scope = _Clients((host, port)), Tap(), gate.Scope()
    threading.Thread(target=server.serve_forever, daemon=True).start()

    def hook(s):
        if not server.clients and server.commands.empty():
            return
        try:
            while not server.commands.empty():
                _commanded(s, scope, *server.commands.get_nowait())
            scope.passed(s)
            snap = tap.passed(s) if server.clients else None
            if snap is not None:
                _served(server, scope, snap)
            scope.closed(s)
        except Exception as exc:
            _faulted(s, server, scope, exc)
    s['each'].append(hook)
    return server


def _faulted(s, server, scope, exc):
    """A fault of her telemetry told to every client watching, its gates dropped unwritten: it never
    stops her loop - a gate's writer started in her page's daemon worker once raised in her pass and
    ended the page."""
    import traceback
    where = traceback.extract_tb(exc.__traceback__)[-1]
    notice = json.dumps({'t': float(s['g']['t']), 'analyzer': 'telemetry', 'event': 'error',
                         'error': '%s: %s' % (type(exc).__name__, exc),
                         'at': '%s:%d' % (where.filename, where.lineno)}).encode('utf-8')
    scope.dropped(s)
    with server.lock:
        for c in server.clients:
            c['gate'] = None
            _put(c['q'], notice)


def _served(server, scope, snap):
    """A snapshot to each client: whole, a JSON line, to one reading the stream; to one watching, its
    analyzers' events on it, its gate's triggers among them."""
    with server.lock:
        clients = list(server.clients)
    line = None
    for c in clients:
        if c['watch'] is None:
            line = line or json.dumps(snap.whole()).encode('utf-8')
            _put(c['q'], line)
            continue
        told = c['watch'].step(snap)
        scope.fired(c['gate'], told)
        for e in told:
            _put(c['q'], json.dumps(_plain(e)).encode('utf-8'))


def read(port, n=1, host='127.0.0.1', timeout=10.0):
    """The next `n` snapshots served on `host:port` (`attach`), parsed."""
    out = []
    with socket.create_connection((host, port), timeout=timeout) as sock:
        f = sock.makefile('rb')
        while len(out) < n:
            line = f.readline()
            if not line:
                break
            out.append(json.loads(line))
    return out


def _session(port, ask, seconds, host='127.0.0.1', until=None):
    """The notices and events served on `host:port` once `ask` is heard, over `seconds` of the wall
    or till `until(event)`; the snapshots before it is heard left out."""
    out, end = [], time.monotonic() + seconds
    with socket.create_connection((host, port), timeout=seconds + 10.0) as sock:
        sock.sendall(json.dumps(ask).encode('utf-8') + b'\n')
        f = sock.makefile('rb')
        while time.monotonic() < end and not (out and until is not None and until(out[-1])):
            sock.settimeout(max(end - time.monotonic(), 0.0))
            try:
                line = f.readline()
            except OSError:
                break
            if not line:
                break
            rec = json.loads(line)
            out.extend([rec] if 'event' in rec else [])
    return out


def events(port, seconds, folder=None, host='127.0.0.1'):
    """The events her analyzers fire (a `watch.Watch` beside her loop, of `folder` else
    machine/analyzers) over `seconds` of the wall: what a reader with little room takes in place of
    the stream."""
    return _session(port, {'watch': {'folder': folder}}, seconds, host)


def gated(port, spec, seconds, host='127.0.0.1'):
    """A gate armed on her analyzers' events (`gate.Scope.arm`: triggers, tag, pre_s, post_s, count,
    streams; folder, the analyzers') over `seconds` of the wall, or till its count is written: its
    notices - armed, each gate written - and the events."""
    return _session(port, {'gate': spec}, seconds, host,
                    until=lambda e: e.get('event') == 'gated' and not e.get('armed'))


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--attach', type=int, required=True, help='the port her page serves on (--telemetry)')
    parser.add_argument('-n', type=int, default=0, help='lines to write, 0 till interrupted')
    parser.add_argument('--events', action='store_true',
                        help='her analyzers\' events alone (machine/analyzers, reloaded as they change)')
    parser.add_argument('--analyzers', help='the analyzers\' folder, else machine/analyzers')
    parser.add_argument('--gate', nargs='*', metavar='TRIGGER',
                        help='a gate armed on her analyzers\' events (`machine.rig.gate`): its triggers '
                             '(\'analyzer.event\'), none those marked capture')
    parser.add_argument('--tag', help='the gate\'s tag')
    parser.add_argument('--pre', type=float, help='s before its trigger, else half her leg\'s swing')
    parser.add_argument('--post', type=float, help='s after it, else half her leg\'s swing')
    parser.add_argument('--count', type=int, help='gates written at most, else unbounded')
    parser.add_argument('--streams', help='its streams, comma-separated (`streams.NAMES`), else all')
    args = parser.parse_args(argv)
    spec = None if args.gate is None else {
        'triggers': args.gate or None, 'tag': args.tag, 'pre_s': args.pre, 'post_s': args.post, 'count': args.count,
        'streams': args.streams.split(',') if args.streams else None, 'folder': args.analyzers}
    ask = {'gate': spec} if spec is not None else {'watch': {'folder': args.analyzers}} if args.events else None
    with socket.create_connection(('127.0.0.1', args.attach)) as sock:
        sock.sendall(b'' if ask is None else json.dumps(ask).encode('utf-8') + b'\n')
        f, k = sock.makefile('rb'), 0
        for line in iter(f.readline, b''):
            if ask is not None and 'event' not in json.loads(line):
                continue
            sys.stdout.write(line.decode('utf-8'))
            sys.stdout.flush()
            k += 1
            if args.n and k >= args.n:
                break
    return 0


if __name__ == '__main__':
    sys.exit(main())
