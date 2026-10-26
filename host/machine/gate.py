"""Her streams gated to a file as a scope acquires: a trigger, the windows around it, re-armed.

    scope = gate.Scope()                          # her streams' rings, laid while a gate is armed
    key = scope.arm(s, spec, told)                # a gate: triggers, tag, pre_s, post_s, count, streams
    scope.passed(s); scope.fired(key, events); scope.closed(s)   # each pass; its analyzers' events
    scope.disarm(s, key)                          # taken off, an open window written as it stands
    gates = gate.read(path)                       # a file's gates: meta, trailer, each stream's samples
    python -m machine.gate FILE [--field felt.QD]  # a file's gates told, a JSON line each

Armed, a gate waits for a trigger: an event of its analyzers its spec names ('analyzer.event'),
else one its analyzer marks capture. Fired, it takes its streams (`streams.STREAMS`) over [t -
pre_s, t + post_s] - half her leg's swing each way unless told (`takes.ENTER_S`) -, written once
the post has passed; armed again till `count` gates are written (None, unbounded), a trigger
while its window is open held off and told in its trailer. Its file, one an arming, is gate after
gate: a metadata line {'gate': its trigger's name, tag, event and time, the window, each stream
expected - what makes it and its words, its kind, period, the most it fits and fields -}; its records {'t',
's', 'v'} in time, the streams interleaved - one instant's in her pass's order -, an aperiodic
stream's last before the window first: the value it held; a trailer {'gated': each stream's count, the
triggers held off, a cut}. A vector's 'v' holds its fields at their offsets ('at', 'n'; 'of' the
names its entries are, 'each' their components, names-major); a record's is the record. Her loop
copies a window's samples and hands them on; a process of the scope's, fed through a pipe, encodes
and writes them - encoded beside her loop, a gate's JSON held her passes 0.4 s; handed on, 36 ms -
and ends as its pipe closes. Her page's worker is a daemon: a multiprocessing child it may not start.
"""
import collections
import itertools
import json
import math
import os
import pickle
import queue
import re
import subprocess
import sys
import threading
import time
import typing

#: Where gates are written, a file an arming.
CAPTURES = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'build', 'telemetry')


def _ring(row, lay, span, s):
    """A stream's ring over `span` s of her: a vector's times and samples laid once, as deep as the
    span holds its periods; a record's a deque its samples past the span leave."""
    import numpy as np
    period = None if row.period is None else row.period(s)
    depth = 0 if lay is None else int(math.ceil(span / period)) + 1
    return {'key': None, 'lay': lay, 'span': span, 'period': period, 't': np.full(depth, np.nan),
            'v': np.empty((depth, (lay or {}).get('n', 0))), 'k': 0, 'd': collections.deque()}


def _append(ring, t, record):
    """`record` at `t` onto a record stream's ring; those past its span gone, the last before it kept."""
    d = ring['d']
    d.append((t, record))
    while len(d) > 1 and d[1][0] < t - ring['span']:
        d.popleft()


def _put(ring, t, row, s):
    """This pass's sample of `row` at `t` into its ring."""
    if ring['lay'] is None:
        return _append(ring, t, row.take(s, None, None))
    k = ring['k'] % len(ring['t'])
    row.take(s, ring['lay'], ring['v'][k])
    ring['t'][k], ring['k'] = t, ring['k'] + 1


def _window(ring, lo, hi):
    """(times, samples) of `ring` within [lo, hi] in time: a vector's rows copied; a record's
    records, an aperiodic one's last before `lo` first - the value it held."""
    import numpy as np
    if ring['lay'] is not None:
        idx = np.flatnonzero((ring['t'] >= lo) & (ring['t'] <= hi))
        idx = idx[np.argsort(ring['t'][idx], kind='stable')]
        return ring['t'][idx].tolist(), ring['v'][idx]
    held = [x for x in ring['d'] if x[0] < lo][-1:] if ring['period'] is None else []
    got = held + [x for x in ring['d'] if lo <= x[0] <= hi]
    return [t for t, _r in got], [r for _t, r in got]


def _described(row, lay, s):
    """A stream as a gate's metadata tells it: what makes it, its words, kind, period, fields."""
    from machine import introspect, streams
    period = row.period(s) if row.period is not None else None
    of = {f['of'] for f in (lay or {}).get('fields', {}).values() if 'of' in f}
    names = dict({q: list(introspect.names_of(q)) for q in of if '.' in q}, **(lay or {}).get('names', {}))
    return {'by': row.by, 'says': streams.said(row.by), 'kind': 'record' if lay is None else 'vector',
            'period_s': period, **({} if lay is None else {'n': lay['n'], 'fields': lay['fields'], 'names': names})}


class Scope:

    """Her streams' rings and the gates armed on them, in her loop's thread; a writer's process."""

    def __init__(self):
        self.gates: dict[int, typing.Any] = {}
        self.rings: dict[str, typing.Any] = {}
        self.polled: list[typing.Any] = []
        self.last: typing.Any = None
        self.clock: typing.Any = None
        self.epoch: typing.Any = None
        self.was: typing.Any = None
        self.jobs: typing.Any = None
        self.keys, self.writer, self.pending = itertools.count(), None, {}

    def arm(self, s, spec, told):
        """A gate of `spec` - {triggers, tag, pre_s, post_s, count, streams}, each optional - armed in
        her session `s`; `told(notice)` hears it armed and each gate written. Its key."""
        from machine import streams, takes
        from machine.errors import MachineError
        half = float(takes.ENTER_S)
        spec = typing.cast(typing.Any, dict({'triggers': None, 'tag': None, 'pre_s': half, 'post_s': half, 'count': None,
                                             'streams': None}, **{k: v for k, v in (spec or {}).items() if v is not None}))
        unknown = sorted(set(spec['streams'] or ()) - set(streams.NAMES))
        if unknown:
            raise MachineError('no stream %s; her streams: %s' % (', '.join(unknown), ', '.join(streams.NAMES)))
        key = next(self.keys)
        os.makedirs(CAPTURES, exist_ok=True)
        path = os.path.join(CAPTURES, '%s_%s.jsonl' % (time.strftime('%Y%m%d_%H%M%S'), re.sub(r'\W+', '_', spec['tag'] or 'gate')))
        names = [n for n in streams.NAMES if spec['streams'] is None or n in spec['streams']]
        self.gates[key] = {'spec': spec, 'streams': names, 'told': told, 'path': path, 'armed': True, 'open': None, 'n': 0}
        self._laid(s)
        told({'t': float(s['g']['t']), 'analyzer': 'gate', 'event': 'armed', 'file': path, 'streams': names, **spec})
        return key

    def disarm(self, s, key):
        """The gate `key` taken off, its open window written as it stands; the last gone, the rings."""
        g = self.gates.get(key)
        if g is None:
            return
        if g['open'] is not None:
            self._gated(s, key, 'disarmed')
        del self.gates[key]
        if not self.gates:
            s['heard'], self.was, self.rings, self.polled, self.last = self.was, None, {}, [], None

    def dropped(self, s):
        """Every gate dropped, none written - a fault in her telemetry -; her commands heard as before."""
        s['heard'] = self.was if self.was is not None else s['heard']
        self.gates, self.rings, self.polled, self.was, self.last, self.clock = {}, {}, [], None, None, None

    def _laid(self, s):
        """The rings her gates' streams need, as deep as the longest window and a law pass: those
        laid already kept where deep enough; her commands heard through the scope."""
        from machine import streams
        span = max(g['spec']['pre_s'] + g['spec']['post_s'] for g in self.gates.values()) + streams._law(s)
        wanted = {n for g in self.gates.values() for n in g['streams']}
        old, self.rings = self.rings, {}
        for row in streams.STREAMS:
            keep = row.name in old and old[row.name]['span'] >= span
            self.rings.update({row.name: old[row.name] if keep else _ring(row, row.laid and row.laid(s), span, s)}
                              if row.name in wanted else {})
            self.rings[row.name]['meta'] = _described(row, self.rings[row.name]['lay'], s) if row.name in wanted else None
        for g in self.gates.values():
            g.setdefault('events', dict(_ring(streams.STREAMS[-1], None, span, s),
                                        meta=_described(streams.STREAMS[-1], None, s)))['span'] = span
        if self.was is None:
            self.was, s['heard'] = s['heard'], lambda entry: self._heard(s, entry)
        self.epoch = s['epoch']
        self.polled = [(row, self.rings[row.name]) for row in streams.STREAMS
                       if row.name in self.rings and row.due is not streams._pushed]

    def _heard(self, s, entry):
        """A command heard (epoch, pass, t, command): told on, and into the heard stream at its time."""
        self.was(entry)
        if entry[0] != self.epoch:
            self._rolled(s)
        ring = self.rings.get('heard')
        if ring is not None:
            _append(ring, float(entry[2]), entry[3])

    def _rolled(self, s):
        """A new epoch, her time from its start: each open window written as it stands; the rings anew."""
        for key in [k for k, g in self.gates.items() if g['open'] is not None]:
            self._gated(s, key, 'epoch')
        self.rings, self.last = {}, None
        for g in self.gates.values():
            g.pop('events', None)
        self._laid(s)

    def passed(self, s):
        """A pass of her session `s`: each stream its own clock says has a new sample takes it."""
        if not self.gates:
            return
        from machine import streams
        if s['epoch'] != self.epoch:
            self._rolled(s)
        c = self.clock = streams.clock(s, self.last)
        self.last = streams.seen(s)
        for row, ring in self.polled:
            due = row.due(s, c)
            if due is None or due[0] == ring['key']:
                continue
            ring['key'] = due[0]
            if not c['fresh']:
                _put(ring, due[1], row, s)

    def fired(self, key, events):
        """Its analyzers' `events` to the gate `key`: each into its events stream; the first of its
        triggers while armed opens its window, those while it is open held off."""
        g = self.gates.get(key)
        if g is None:
            return
        for e in events:
            _append(g['events'], float(e['t']), e)
            name, triggers = '%s.%s' % (e.get('analyzer'), e.get('event')), g['spec']['triggers']
            if not (name in triggers if triggers else bool(e.get('capture'))):
                continue
            if g['open'] is not None:
                g['open']['held_off'].append({'name': name, 't': float(e['t'])})
            g['open'] = g['open'] or ({'t': float(e['t']), 'name': name, 'event': e, 'held_off': []} if g['armed'] else None)
            g['armed'] = g['armed'] and g['open'] is None

    def closed(self, s):
        """Each window its post has passed - this pass begun at or after its end - written."""
        if self.clock is None:
            return
        t0 = float(self.clock['t0'])
        for key, g in list(self.gates.items()):
            if g['open'] is not None and t0 >= g['open']['t'] + g['spec']['post_s']:
                self._gated(s, key, None)

    def _gated(self, s, key, cut):
        """The gate `key`'s window as the rings hold it to the writer, the gate armed again while its
        count allows; `cut` why it ends before its post, None where it does not."""
        g = self.gates[key]
        o, sp = g['open'], g['spec']
        lo, hi = o['t'] - sp['pre_s'], o['t'] + sp['post_s']
        rings = dict(self.rings, events=g['events'])
        taken = {n: _window(rings[n], lo, hi) for n in g['streams']}
        g['n'], g['open'] = g['n'] + 1, None
        g['armed'] = sp['count'] is None or g['n'] < sp['count']
        most = lambda p: None if p is None else int(math.floor((hi - lo) / p)) + 1
        streams_meta = {n: dict(rings[n]['meta'], expected=most(rings[n]['period'])) for n in g['streams']}
        meta = {'n': g['n'], 'name': o['name'], 'tag': sp['tag'], 't': o['t'], 'event': o['event'], 'window': [lo, hi],
                'pre_s': sp['pre_s'], 'post_s': sp['post_s'], 'epoch': self.epoch, 'streams': streams_meta}
        trailer = {'n': g['n'], 'name': o['name'], 'tag': sp['tag'], 'held_off': o['held_off'], 'cut': cut,
                   'to': self.clock['t0'] if cut and self.clock else hi, 'armed': g['armed']}
        job = next(self.keys)
        self.pending[job] = (g['told'], {'t': trailer['to'], 'analyzer': 'gate', 'event': 'gated', 'n': meta['n'],
                                         'name': meta['name'], 'tag': meta['tag'], 'at': meta['t'], 'file': g['path'],
                                         'held_off': trailer['held_off'], 'cut': cut, 'armed': g['armed']})
        self._writer().put((job, g['path'], meta, taken, trailer))

    def _writer(self):
        """The writer's queue, its process started where none runs (`python -m machine.gate --writes`):
        a thread pickles each gate into its pipe, another hears each written."""
        if self.writer is None or self.writer.poll() is not None:
            self.writer = subprocess.Popen([sys.executable, '-X', 'utf8', '-m', 'machine.gate', '--writes'],
                                           cwd=os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                           stdin=subprocess.PIPE, stdout=subprocess.PIPE)
            self.jobs = queue.Queue()
            threading.Thread(target=_piped, args=(self.writer, self.jobs), daemon=True).start()
            threading.Thread(target=self._told, args=(self.writer,), daemon=True).start()
        return self.jobs

    def _told(self, writer):
        """Each gate written, told to its client with the records it holds."""
        for line in writer.stdout:
            job, n = json.loads(line)
            told, notice = self.pending.pop(job)
            told(dict(notice, records=n))


def _piped(writer, jobs):
    """Each gate of `jobs` pickled into the writer's pipe."""
    for job in iter(jobs.get, None):
        pickle.dump(job, writer.stdin, protocol=pickle.HIGHEST_PROTOCOL)
        writer.stdin.flush()


def _writes(pipe, done):
    """The writer's process: each gate its parent pipes in written (`written`), its records told
    back a JSON line each; the pipe closed - its parent gone -, it ends."""
    while True:
        try:
            job, path, meta, taken, trailer = pickle.load(pipe)
        except EOFError:
            return 0
        done.write(json.dumps([job, written(path, meta, taken, trailer)]) + '\n')
        done.flush()


def written(path, meta, taken, trailer):
    """A gate's lines appended to `path`: its metadata, its records in time - one instant's in its
    streams' order -, its trailer. How many records."""
    from machine import introspect
    order = list(taken)
    keys = sorted((t, o, i) for o, name in enumerate(order) for i, t in enumerate(taken[name][0]))
    with open(path, 'a', encoding='utf-8') as f:
        f.write(json.dumps({'gate': introspect._plain(meta)}) + '\n')
        for t, o, i in keys:
            v = taken[order[o]][1][i]
            f.write(json.dumps({'t': t, 's': order[o], 'v': v.tolist() if hasattr(v, 'tolist') else introspect._plain(v)}) + '\n')
        f.write(json.dumps({'gated': dict(trailer, records={n: len(taken[n][0]) for n in order})}) + '\n')
    return len(keys)


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
        d, (ts, vs) = m['streams'][stream], g['streams'][stream]
        f = d['fields'][field]
        block = vs[:, f['at']:f['at'] + f['n']]
        i, j = np.unravel_index(int(np.nanargmax(np.abs(block))), block.shape)
        labels = d['names'].get(f.get('of'), [])
        out[name] = {'value': float(block[i, j]), 't': float(ts[i]),
                     'entry': labels[j // f.get('each', 1)] if labels else int(j)}
    return out


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('file', nargs='?', help='a gate file (build/telemetry/*.jsonl)')
    parser.add_argument('--field', action='append', default=[], help="'stream.FIELD': its largest entry each gate")
    parser.add_argument('--writes', action='store_true', help="a scope's writer: gates piped in, written")
    args = parser.parse_args(argv)
    if args.writes:
        return _writes(sys.stdin.buffer, sys.stdout)
    for g in read(args.file):
        sys.stdout.write(json.dumps(told(g, args.field)) + '\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())
