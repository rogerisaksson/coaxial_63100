"""Her telemetry reduced: analyzers firing sparse events on her body's own triggers.

    w = watch.Watch()                       # every analyzer in machine/analyzers, or a folder given
    events = w.step(snap)                   # a snapshot (`introspect.Tap`) in, its events out
    python -m machine.introspect --attach PORT --events    # a page's events, a JSON line each

An analyzer is a file of its folder: `laid(meta) -> state` and `step(state, snap) -> [event, ..]`,
most passes none; `meta` her code's own numbers (`meta`): her joints, each drive's rated speed, her
weight, her natural time. The folder is read again as a file there is saved, added or taken away -
while she runs: a file changed is loaded anew, its state laid again; one removed is dropped; one
that raises is told once and held off till its file changes, the rest firing on. Each event carries
the pass's time and its analyzer's name.

An event an analyzer marks `capture` triggers as a scope does: the snapshots of the half swing of
her leg before it (`takes.ENTER_S`, pi sqrt(leg/g)) and of the half swing after written whole to a
file of CAPTURES - every field at the law's rate, the peaks held between -, and a `captured` event
naming it; the same trigger held off till its capture is written, so a storm writes one file.
"""
import collections
import glob
import importlib.util
import json
import os
import time

#: Where her analyzers are, one a file.
FOLDER = os.path.join(os.path.dirname(__file__), 'analyzers')
#: Where a trigger's capture is written, a JSON line a snapshot.
CAPTURES = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'build', 'telemetry')
#: How often the folder is read again, s of the wall: a file saved is in within it.
RESCAN_S = 1.0


def meta():
    """Her code's own numbers an analyzer weighs by: her joints (`figure.JOINTS`), each drive's rated
    speed, deg/s (her manifest's), her weight, N, her natural time, s (`takes.TICK_S`: sqrt(leg/g)),
    a capture's half (`takes.ENTER_S`: half her leg's swing)."""
    from machine import defrobot, takes
    from machine.figure import JOINTS
    r = defrobot.read() or {'drives': {}, 'body': {'weight': float('nan')}}
    return {'joints': list(JOINTS), 'rated': {j: float(r['drives'][j]['speed']) for j in JOINTS if j in r['drives']},
            'weight': float(r['body']['weight']), 'natural_s': float(takes.TICK_S), 'capture_s': float(takes.ENTER_S)}


class Watch:

    """The analyzers of `folder` over her snapshots, read again as its files change; marked events'
    captures written to `captures`."""

    def __init__(self, folder=FOLDER, laid_meta=None, captures=CAPTURES):
        self.folder, self.meta, self.captures = folder, laid_meta if laid_meta is not None else meta(), captures
        self.on, self.read_at, self.epoch = {}, -float('inf'), None
        self.before, self.open = collections.deque(), []

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

    def _captured(self, t, snap, fired):
        """The scope: `snap` into the history and every open capture; a capture begun for each event of
        `fired` marked `capture` whose trigger is not already capturing; [captured event] of each
        capture its half after has filled, its file written."""
        half = self.meta['capture_s']
        self.before.append(snap)
        while self.before and t - float(self.before[0]['t']) > half:
            self.before.popleft()
        for c in self.open:
            c['rows'].append(snap)
        busy = {c['trigger'] for c in self.open}
        for e in fired:
            trigger = (e['analyzer'], e['event'])
            if e.get('capture') and trigger not in busy:
                busy.add(trigger)
                self.open.append({'trigger': trigger, 'at': t, 'rows': list(self.before), 'event': e})
        out, still = [], []
        for c in self.open:
            if 0.0 <= t - c['at'] < half:
                still.append(c)
                continue
            os.makedirs(self.captures, exist_ok=True)
            path = os.path.join(self.captures, '%s_t%09.3f_%s_%s.jsonl' % (
                time.strftime('%Y%m%d_%H%M%S'), c['at'], c['trigger'][0], c['trigger'][1]))
            with open(path, 'w', encoding='utf-8') as f:
                f.write(json.dumps({'trigger': c['event'], 'before_s': half, 'after_s': half}) + '\n')
                f.writelines(json.dumps(r) + '\n' for r in c['rows'])
            out.append({'t': t, 'analyzer': c['trigger'][0], 'event': 'captured', 'of': c['trigger'][1],
                        'at': c['at'], 'file': path, 'snapshots': len(c['rows'])})
        self.open = still
        return out

    def step(self, snap):
        """[event] of every analyzer on the snapshot `snap`, the folder read again where due, the
        scope's captures written as they fill."""
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
        return out + fired + self._captured(t, snap, fired)
