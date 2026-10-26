"""Her telemetry reduced: analyzers firing sparse events on her body's own triggers.

    w = watch.Watch()                       # every analyzer in machine/analyzers, or a folder given
    events = w.step(snap)                   # a snapshot (`introspect.Tap`) in, its events out
    python -m machine.introspect --attach PORT --events    # a page's events, a JSON line each
    python -m machine.introspect --attach PORT --gate      # a gate armed on those marked capture

An analyzer is a file of its folder: `laid(meta) -> state` and `step(state, snap) -> [event, ..]`,
most passes none; `meta` her code's own numbers (`meta`): her joints, each drive's rated speed, her
weight, her natural time. The folder is read again as a file there is saved, added or taken away -
while she runs: a file changed is loaded anew, its state laid again; one removed is dropped; one
that raises is told once and held off till its file changes, the rest firing on. Each event carries
the pass's time and its analyzer's name.

An event an analyzer marks `capture` is a trigger for a gate armed without triggers of its own
(`machine.gate`): her streams on their own clocks over the half swing of her leg before it and after
(`takes.ENTER_S`, pi sqrt(leg/g)) written to a file, the gate armed again.
"""
import glob
import importlib.util
import os
import time

#: Where her analyzers are, one a file.
FOLDER = os.path.join(os.path.dirname(__file__), 'analyzers')
#: How often the folder is read again, s of the wall: a file saved is in within it.
RESCAN_S = 1.0


def meta():
    """Her code's own numbers an analyzer weighs by: her joints (`figure.JOINTS`), each drive's rated
    speed, deg/s (her manifest's), her weight, N, her natural time, s (`takes.TICK_S`: sqrt(leg/g))."""
    from machine import defrobot, takes
    from machine.figure import JOINTS
    r = defrobot.read() or {'drives': {}, 'body': {'weight': float('nan')}}
    return {'joints': list(JOINTS), 'rated': {j: float(r['drives'][j]['speed']) for j in JOINTS if j in r['drives']},
            'weight': float(r['body']['weight']), 'natural_s': float(takes.TICK_S)}


class Watch:

    """The analyzers of `folder` over her snapshots, read again as its files change."""

    def __init__(self, folder=FOLDER, laid_meta=None):
        self.folder, self.meta = folder, laid_meta if laid_meta is not None else meta()
        self.on, self.read_at, self.epoch = {}, -float('inf'), None

    def _load(self, path):
        """(module, its state) of the analyzer file at `path`, laid anew."""
        spec = importlib.util.spec_from_file_location('analyzer_' + os.path.basename(path)[:-3], path)
        if spec is None or spec.loader is None:
            raise ImportError('no analyzer at %s' % path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod, mod.laid(self.meta)

    def scan(self, t):
        """The folder read again: [event] of each analyzer loaded, reloaded, dropped or failing."""
        out, files = [], {os.path.basename(p)[:-3]: p for p in glob.glob(os.path.join(self.folder, '*.py'))
                          if not os.path.basename(p).startswith('_')}
        for name in set(self.on) - set(files):
            del self.on[name]
            out.append({'t': t, 'analyzer': name, 'event': 'dropped'})
        for name, path in files.items():
            stamp = os.path.getmtime(path)
            was = self.on.get(name)
            if was is not None and was['stamp'] == stamp:
                continue
            try:
                mod, state = self._load(path)
                self.on[name] = {'mod': mod, 'state': state, 'stamp': stamp, 'failed': False}
                out.append({'t': t, 'analyzer': name, 'event': 'reloaded' if was else 'loaded'})
            except Exception as exc:
                self.on[name] = {'mod': None, 'state': None, 'stamp': stamp, 'failed': True}
                out.append({'t': t, 'analyzer': name, 'event': 'error', 'error': '%s: %s' % (type(exc).__name__, exc)})
        return out

    def step(self, snap):
        """[event] of every analyzer on the snapshot `snap`, the folder read again where due."""
        t, out = float(snap.get('t', 0.0)), []
        if time.monotonic() - self.read_at >= RESCAN_S:
            self.read_at = time.monotonic()
            out.extend(self.scan(t))
        epoch = snap.get('body', {}).get('epoch')
        if self.epoch is not None and epoch != self.epoch:
            # a new epoch laid: her time from 0 again, every analyzer's state laid anew
            for name, on in self.on.items():
                on['state'] = None if on['failed'] else on['mod'].laid(self.meta)
            out.append({'t': t, 'analyzer': 'watch', 'event': 'relaid', 'epoch': epoch})
        self.epoch = epoch
        fired = []
        for name, on in self.on.items():
            if on['failed']:
                continue
            try:
                fired.extend({'t': t, 'analyzer': name, **event} for event in on['mod'].step(on['state'], snap) or ())
            except Exception as exc:
                on['failed'] = True
                fired.append({'t': t, 'analyzer': name, 'event': 'error', 'error': '%s: %s' % (type(exc).__name__, exc)})
        return out + fired
