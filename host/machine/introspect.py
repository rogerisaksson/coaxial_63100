"""Her loop's latent, impedance and wrench layers live, for a debugger or the local model to attach to.

    snap = introspect.snapshot(s)                 # a law pass's layers and her body, named, JSON-able
    server = introspect.attach(s, port)            # serve them on 127.0.0.1:port, a JSON line a pass
    rows = introspect.read(port, n=10)             # a client: the next n snapshots
    told = introspect.events(port, seconds=8)       # her analyzers' events over 8 s (`watch`)
    python -m machine.introspect --attach 8765     # the same from a shell, a JSON line each
    python -m machine.introspect --attach 8765 --events   # her analyzers' events alone, a line each

The layers as her loop leaves them each law pass (`motion.control`): latent - her state's rows and
her latent out (`latent.step`); impedance - the modal ask (`impedance.ask`) and the step's ask it
makes; wrench - the whole-body step's answer (`wrench.step`: tau, qacc, contact forces, ..). Each
array's fields are named from her code's own layout, generated (`layout`): the run of module-level
slice and int constants ending at a vector's size, laid in rising offsets - none listed by hand.
Attached, a hook on her passes (`worker`'s `s['each']`) publishes a snapshot each law pass to every
client connected, dropped where a client lags, the loop never waiting; none connected, the hook
returns at its first line. Opt in on the page: `show_humanoid --telemetry PORT`.
"""
import ast
import functools
import importlib
import inspect
import json
import queue
import socket
import socketserver
import sys
import threading
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


def _span(v):
    """A constant as the slice it names: a slice itself, an int one wide; else None."""
    import numpy as np
    if isinstance(v, slice) and isinstance(v.start, int) and isinstance(v.stop, int):
        return v
    if isinstance(v, (int, np.integer)) and not isinstance(v, bool):
        return slice(int(v), int(v) + 1)
    return None


def _named(vec, fields, labels=None):
    """{field: its value, or its values - by name where `labels` {width: names} names a field's width
    (her modes, `ortho.MODES`; what bears her, `ortho.MANIFOLDS`)} of the array `vec` by `fields`."""
    import numpy as np
    vec, labels = np.asarray(vec, float).ravel(), labels or {}
    width = lambda sp: sp.stop - sp.start
    return {name: (float(vec[sp.start]) if width(sp) == 1 else
                   dict(zip(labels[width(sp)], vec[sp].tolist())) if width(sp) in labels else vec[sp].tolist())
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
    from machine import sprung
    from machine.figure import JOINTS
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


def snapshot(s, peaks=None):
    """A law pass of her session `s`, named: {'t', 'doing', 'body': `body`, 'latent': {'state': {row:
    {field: ..}}, 'out': {..}}, 'impedance': {'ask': {..}, 'step_ask': {..}}, 'wrench': {..}}."""
    import numpy as np
    from machine import ortho, sprung
    g = s['g']
    row, state, out, ask = (layout('machine.latent', 'P', 'NOW'), layout('machine.latent', 'STATE'),
                            layout('machine.latent', 'OUT'), layout('machine.modal', 'ASK'))
    labels = {len(ortho.MODES): ortho.MODES, len(ortho.MANIFOLDS): ortho.MANIFOLDS}
    st = np.asarray(g['st'], float).ravel()
    rows = {name: _named(st[sp], row, labels) for name, sp in state.items() if sp.stop - sp.start == row_len(row)}
    rest = {name: v for name, v in _named(st, state).items() if name not in rows}
    return {'t': float(g['t']), 'doing': sprung.doing(g), 'body': body(s, peaks),
            'latent': {'state': {**rows, **rest}, 'out': _named(g['out'], out, labels)},
            'impedance': {'ask': _named(g['a'], ask), 'step_ask': _plain(g.get('ask', {}))},
            'wrench': _plain(g.get('got', {}))}


def row_len(fields):
    """The length a row of `fields` spans."""
    return max(sp.stop for sp in fields.values())


class _Clients(socketserver.ThreadingTCPServer):
    daemon_threads, allow_reuse_address = True, True

    def __init__(self, address):
        super().__init__(address, _Client)
        self.queues, self.lock = [], threading.Lock()


class _Client(socketserver.StreamRequestHandler):
    def handle(self):
        srv, q = typing.cast(_Clients, self.server), queue.Queue(QUEUED)
        with srv.lock:
            srv.queues.append(q)
        try:
            while True:
                self.wfile.write(q.get() + b'\n')
        except OSError:
            pass
        finally:
            with srv.lock:
                srv.queues.remove(q)


class Tap:

    """Her passes into snapshots, a peak detector between: every pass her joints' fastest held, a
    snapshot each law period of her world's time carrying them since the last - fallen, her law
    stands still and a snapshot a law pass told no fall; one a law pass alone read the tip's fastest
    joint 249.7 deg/s where every pass's 270.7."""

    def __init__(self):
        self.t, self.peaks = -float('inf'), None

    def passed(self, s):
        """A pass of her session `s`: its snapshot (`snapshot`, the peaks since the last) where a law
        period of her world's time has gone since the last - a new epoch's time from its start -,
        else None."""
        import numpy as np
        from machine import worker
        g = s['g']
        world = g['world']
        v = np.abs(world.data.qvel[world.vadr])
        self.peaks = v if self.peaks is None else np.maximum(self.peaks, v)
        t = float(g['t'])
        if self.t <= t < self.t + worker.DT * g['every']:
            return None
        snap, self.peaks, self.t = snapshot(s, np.degrees(self.peaks)), None, t
        return snap


def attach(s, port, host='127.0.0.1'):
    """Serve her session `s`'s snapshots on `host:port` (0 a free one; the server's `server_address`
    says which), a JSON line each law pass to every client (`Tap`) - a hook on her passes
    (`s['each']`) that returns at once with none connected. The server."""
    server, tap = _Clients((host, port)), Tap()
    threading.Thread(target=server.serve_forever, daemon=True).start()

    def hook(s):
        if not server.queues:
            return
        snap = tap.passed(s)
        if snap is None:
            return
        line = json.dumps(snap).encode('utf-8')
        with server.lock:
            for q in server.queues:
                if q.full():
                    q.get_nowait()
                q.put_nowait(line)
    s['each'].append(hook)
    return server


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


def events(port, seconds, folder=None, host='127.0.0.1'):
    """The events her analyzers fire (`watch.Watch`) over `seconds` of the snapshots served on
    `host:port` - what a reader with little room takes in place of the stream."""
    from machine import watch
    w, out = watch.Watch(folder or watch.FOLDER), []
    with socket.create_connection((host, port), timeout=seconds + 10.0) as sock:
        f, t0 = sock.makefile('rb'), None
        for line in iter(f.readline, b''):
            snap = json.loads(line)
            t0 = snap['t'] if t0 is None else t0
            out.extend(w.step(snap))
            if snap['t'] - t0 >= seconds:
                break
    return out


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--attach', type=int, required=True, help='the port her page serves on (--telemetry)')
    parser.add_argument('-n', type=int, default=0, help='lines to write, 0 till interrupted')
    parser.add_argument('--events', action='store_true',
                        help='her analyzers\' events alone (machine/analyzers, reloaded as they change)')
    parser.add_argument('--analyzers', help='the analyzers\' folder, else machine/analyzers')
    args = parser.parse_args(argv)
    from machine import watch
    w = watch.Watch(args.analyzers or watch.FOLDER) if args.events else None
    with socket.create_connection(('127.0.0.1', args.attach)) as sock:
        f, k = sock.makefile('rb'), 0
        for line in iter(f.readline, b''):
            said = [line.decode('utf-8')] if w is None else [json.dumps(e) + '\n' for e in w.step(json.loads(line))]
            for out in said:
                sys.stdout.write(out)
                k += 1
            sys.stdout.flush()
            if args.n and k >= args.n:
                break
    return 0


if __name__ == '__main__':
    sys.exit(main())
