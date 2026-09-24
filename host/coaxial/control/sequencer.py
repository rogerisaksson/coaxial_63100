"""A test stand in miniature: a table of setpoints and jumps, run through a controller.

    seq = Sequencer.read('steps.csv', limits={'iq_ref': {'HH': 1.8}}, init=arm, cleanup=disarm)
    seq = Sequencer.parse(csv_text)       # a model's answer, as it came
    out = seq.run(loop)      # out.status done | tripped | limit; out.rows; out.steps

The table, one row a step (a .csv or an .xlsx's first sheet); every cell a number or blank:
    seconds            the most the step lasts; 0 is a decision, no time passes
    <channel>          set it (blank: holds)          <channel>+    add to it: a counter
    <channel>.H .L     the step ends when reached     <channel>.HH .LL   trip: to cleanup
    label              a row's name                   group    init | cleanup | blank (main)
    then / else        where to go when a level ended the step / when its time did
    goto, times        after the row, to `goto` `times` times (blank 1, inf), then on
A target is a label, or `stop`: the main group is over. Counters, adding and branching on a
level make it a counter machine - any program, as data. `limits` apply to every step but
cleanup; `cycles` repeats the main group; `limit` seconds or `max_steps` rows end a run.
"""
import csv
import io
import math
from collections import namedtuple

from coaxial.errors import RigError

#: The columns that steer rather than set.
STEER = ('seconds', 'label', 'goto', 'times', 'group', 'then', 'else')
LEVELS = ('LL', 'L', 'H', 'HH')
GROUPS = ('init', 'main', 'cleanup')
STOP = 'stop'

#: A finished run: every pass's channels, how it ended, why, and each step it took
#: (row, label, how the step ended, seconds).
Outcome = namedtuple('Outcome', 'rows status reason steps')


class Tripped(RigError):
    """A channel past its HH or LL."""


def _value(cell):
    """A cell as a float, or None when blank; true/false as 1/0."""
    if cell is None or (isinstance(cell, float) and math.isnan(cell)):
        return None
    text = str(cell).strip().lower()
    if text in ('', 'nan'):
        return None
    if text in ('true', 'false'):
        return 1.0 if text == 'true' else 0.0
    return float(text)


def _text(cell):
    text = '' if cell is None else str(cell).strip()
    return None if text.lower() in ('', 'nan') else text


def _crossed(value, level, bound):
    return value >= bound if level in ('H', 'HH') else value <= bound


class Step:

    """One row: how long, what to set and add, when to end or trip, where to go after."""

    def __init__(self, seconds, setpoints=None, adds=None, limits=None, label=None, group='main',
                 then=None, otherwise=None, goto=None, times=1.0, row=0):
        self.seconds = float(seconds)
        self.setpoints, self.adds, self.limits = dict(setpoints or {}), dict(adds or {}), \
            dict(limits or {})
        self.label, self.group, self.row = label, group, row
        self.then, self.otherwise, self.goto, self.times = then, otherwise, goto, times

    @classmethod
    def of(cls, cells, row=0):
        """A table row."""
        if _value(cells.get('seconds')) is None:
            raise RigError('row %d needs seconds: %r' % (row, cells))
        group = _text(cells.get('group')) or 'main'
        if group not in GROUPS:
            raise RigError('row %d: group %s - there are %s' % (row, group, ', '.join(GROUPS)))
        setpoints, adds, limits = {}, {}, {}
        for key, cell in cells.items():
            value = _value(cell) if key not in STEER else None
            if value is None:
                continue
            channel, _, level = key.rpartition('.')
            if level in LEVELS and channel:
                limits.setdefault(channel, {})[level] = value
            elif key.endswith('+'):
                adds[key[:-1]] = value
            else:
                setpoints[key] = value
        times = _value(cells.get('times'))
        return cls(_value(cells['seconds']), setpoints, adds, limits, _text(cells.get('label')),
                   group, _text(cells.get('then')), _text(cells.get('else')),
                   _text(cells.get('goto')), 1.0 if times is None else times, row)

    def targets(self):
        return [t for t in (self.then, self.otherwise, self.goto) if t]

    def ended(self, bus, limits):
        """('trip' | 'exit', why) once a channel crosses a level, else None."""
        merged = {ch: dict(levels) for ch, levels in limits.items()}
        for channel, levels in self.limits.items():
            merged.setdefault(channel, {}).update(levels)
        hits = [(level, channel, bound) for channel, levels in merged.items()
                for level, bound in levels.items()
                if channel in bus and _crossed(bus[channel], level, bound)]
        for level, channel, bound in sorted(hits, key=lambda h: len(h[0]), reverse=True):
            return ('trip' if len(level) == 2 else 'exit',
                    '%s %.4g past %s %.4g' % (channel, bus[channel], level, bound))
        return None

    def __repr__(self):
        jumps = ''.join(' %s %s' % (k, v) for k, v in (
            ('then', self.then), ('else', self.otherwise),
            ('goto', '%s x%g' % (self.goto, self.times) if self.goto else None)) if v)
        return '<%s %s%.3g s%s%s%s%s>' % (
            self.group, '%s: ' % self.label if self.label else '', self.seconds,
            ' set %s' % self.setpoints if self.setpoints else '',
            ' add %s' % self.adds if self.adds else '',
            ' levels %s' % self.limits if self.limits else '', jumps)


class Sequencer:

    """init rows, the main rows with their jumps `cycles` times, cleanup rows - always."""

    def __init__(self, rows, limits=None, cycles=1, limit=None, max_steps=100000, init=None,
                 cleanup=None):
        steps = [r if isinstance(r, Step) else Step.of(r, i) for i, r in enumerate(rows)]
        self.groups = {g: [s for s in steps if s.group == g] for g in GROUPS}
        self.steps = self.groups['main']
        self.limits, self.cycles, self.limit = dict(limits or {}), cycles, limit
        self.max_steps, self.on_init, self.on_cleanup = max_steps, init, cleanup
        self.at, self.taken = 0, {}
        if STOP in self.labels():
            raise RigError('%s is where a jump ends the run, not a label' % STOP)
        missing = sorted({t for s in self.steps for t in s.targets()}
                         - set(self.labels()) - {STOP})
        if missing:
            raise RigError('a jump names no main row: %s; the labels are %s'
                           % (', '.join(missing), ', '.join(self.labels()) or 'none'))

    @classmethod
    def read(cls, path, **kw):
        """A .csv (a header row, then one row a step) or an .xlsx's first sheet."""
        if str(path).lower().endswith(('.xlsx', '.xls')):
            import pandas
            return cls(pandas.read_excel(path).to_dict('records'), **kw)
        with open(path, newline='', encoding='utf-8') as handle:
            return cls.parse(handle.read(), **kw)

    @classmethod
    def parse(cls, text, **kw):
        """The table as csv text: what a model answers with."""
        return cls(list(csv.DictReader(io.StringIO(text.strip()))), **kw)

    def labels(self):
        return {s.label: i for i, s in enumerate(self.steps) if s.label}

    def jump(self, to):
        """The next main row, by label, index or `stop`."""
        if to == STOP:
            self.at = len(self.steps)
        else:
            self.at = self.labels()[to] if isinstance(to, str) else int(to)
        return self.at

    def forward(self, rows=1):
        return self.jump(min(len(self.steps), self.at + rows))

    def back(self, rows=1):
        return self.jump(max(0, self.at - rows))

    def _advance(self, ended):
        """Past the current main row: then, else, goto, or the next."""
        here, step = self.at, self.steps[self.at]
        branch = step.then if ended == 'exit' else step.otherwise
        if branch:
            return self.jump(branch)
        self.at = here + 1
        if step.goto:
            taken = self.taken.get(here, 0)
            if taken < step.times:
                self.taken[here] = taken + 1
                self.jump(step.goto)
            else:
                self.taken[here] = 0
        return self.at

    def _run(self, loop, step, out, watch, trips=True):
        """One step: its setpoints and adds, then the loop until its seconds or a level.
        'exit' when a level ended it, 'time' when its seconds did."""
        why = ['time']
        limits = self.limits if trips else {}

        def check(loop):
            out['rows'].append(loop.read())
            hit = step.ended(loop.bus, limits)
            if watch is not None:
                watch(loop)
            if hit is None:
                return False
            if hit[0] == 'trip' and trips:
                raise Tripped(hit[1])
            why[0] = hit[1]
            return True

        start = loop.bus['t']
        writes = dict(step.setpoints, step=float(step.row),
                      **{k: loop.bus.get(k, 0.0) + v for k, v in step.adds.items()})
        try:
            if step.seconds > 0:
                loop.move(step.seconds, check, **writes)
            else:
                loop.write(**writes)
                check(loop)
        finally:
            out['steps'].append((step.row, step.label, why[0], loop.bus['t'] - start))
        return 'time' if why[0] == 'time' else 'exit'

    def run(self, loop, watch=None):
        """The whole sequence on `loop`; cleanup however it ends."""
        out = {'rows': [], 'steps': []}
        status, reason, spent = 'done', None, 0.0
        try:
            if self.on_init is not None:
                self.on_init(loop)
            for step in self.groups['init']:
                self._run(loop, step, out, watch)
            cycle = 0
            while cycle < self.cycles and status == 'done':
                while self.at < len(self.steps):
                    if self.limit is not None and spent >= self.limit:
                        status, reason = 'limit', '%.3g s spent' % spent
                        break
                    if len(out['steps']) >= self.max_steps:
                        status, reason = 'limit', '%d steps taken' % self.max_steps
                        break
                    before = loop.bus['t']
                    ended = self._run(loop, self.steps[self.at], out, watch)
                    spent += loop.bus['t'] - before
                    self._advance(ended)
                cycle += 1
                self.at, self.taken = 0, {}
        except Tripped as trip:
            status, reason = 'tripped', str(trip)
        finally:
            for step in self.groups['cleanup']:
                self._run(loop, step, out, watch, trips=False)
            if self.on_cleanup is not None:
                self.on_cleanup(loop)
        return Outcome(out['rows'], status, reason, out['steps'])
