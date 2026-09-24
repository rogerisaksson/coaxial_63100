"""A test stand in miniature: a table of setpoints and jumps, run through a controller.

    seq = Sequencer.read('steps.csv', limits={'iq_ref': {'HH': 1.8}}, init=arm, cleanup=disarm)
    seq = Sequencer.parse(text)          # a model's answer as it came: lines or csv
    out = seq.run(loop)                  # checked first; out.status done | tripped | limit
    print(out.summary())                 # a few lines back to the model
    print(prompt(loop))                  # GRAMMAR and the loop's card: all a model needs

A step is a row of a .csv or an .xlsx's first sheet, or a line: `0.5 knee=40 hip=-20`
(the bare number is seconds, the rest name=value, # a comment). Every value a number:
    seconds            the most the step lasts; 0 is a decision, no time passes
    <channel>          set it (blank: holds)          <channel>+    add to it: a counter
    <channel>.H .L     the step ends when reached     <channel>.HH .LL   trip: to cleanup
    label              a row's name                   group    init | cleanup | blank (main)
    then / else        where to go when a level ended the step / when its time did
    goto, times        after the row, to `goto` `times` times (blank 1, inf), then on
A target is a label, or `stop`: the main group is over. Counters, adding and branching on a
level make it a counter machine - any program, as data. `limits` apply to every step but
cleanup; `ranges` {channel: (low, high)} refuse a setpoint outside before anything runs;
`cycles` repeats the main group; `limit` seconds or `max_steps` rows end a run.
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

#: What a model is told, with the loop's card (`prompt`).
GRAMMAR = """One step a line: seconds, then name=value. # starts a comment.
  knee=40            set a target            n+=1         add to a counter
  knee.deg.H=38      end the step once reached (.L: once below)
  label=up           name the line           then=up else=down   go there when a level
  goto=up times=3    after the line, back 3 times             ended the step / when time did
  group=init         runs first; group=cleanup runs last, always
0 seconds: a decision, no time passes. A jump to stop ends the program."""


class Outcome(namedtuple('Outcome', 'rows status reason steps')):

    """A finished run: every pass's channels, how it ended, why, and each step it took
    (row, label, how the step ended, seconds)."""

    def summary(self, *channels, most=12):
        """A few lines: the ending, the steps (at most `most`), `channels` at the end."""
        seconds = sum(s[3] for s in self.steps)
        lines = ['%s after %.2f s, %d steps%s' % (self.status, seconds, len(self.steps),
                                                  ': ' + self.reason if self.reason else '')]
        shown = self.steps if len(self.steps) <= most else \
            self.steps[:most - 2] + [None] + self.steps[-1:]
        for step in shown:
            lines.append('  ...' if step is None else '  row %d%s %.2f s %s' % (
                step[0], ' ' + step[1] if step[1] else '', step[3], step[2]))
        last = self.rows[-1] if self.rows else {}
        if channels:
            lines.append('  end: ' + ', '.join('%s %.4g' % (c, last[c]) for c in channels
                                               if c in last))
        return '\n'.join(lines)


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


def _line(line, row):
    """A line's cells: the bare number is seconds, the rest name=value."""
    cells = {}
    for token in line.split():
        key, sep, value = token.partition('=')
        if sep:
            cells[key] = value
        elif 'seconds' not in cells:
            cells['seconds'] = key
        else:
            raise RigError('row %d: %r is neither seconds nor name=value' % (row, token))
    return cells


def _near(name, names):
    """' - did you mean x?' when one is close."""
    import difflib
    close = difflib.get_close_matches(name, sorted(names), n=1, cutoff=0.6)
    return ' - did you mean %s?' % close[0] if close else ''


def card(loop, units=None, ranges=None):
    """What a program may set and test on `loop`, one line each: the feedback loops'
    targets, their unit and range, and what reads each back."""
    units, ranges = units or {}, ranges or {}
    held = dict(zip(loop.targets(), loop.controlled()))
    groups = {}
    for target in loop.targets():
        back = held.get(target, '-')
        like = '<name>' + back[len(target):] if back.startswith(target) else back
        key = (units.get(target, ''), ranges.get(target), like)
        groups.setdefault(key, []).append(target)
    return '\n'.join('set  %s%s%s  (read back as %s)' % (
        ', '.join(targets), ' ' + unit if unit else '', ' %g..%g' % span if span else '', like)
        for (unit, span, like), targets in groups.items())


def prompt(loop, units=None, ranges=None):
    """GRAMMAR, then the loop's card: what a model is told."""
    return GRAMMAR + '\n\nThis machine:\n' + card(loop, units, ranges)


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
                 cleanup=None, ranges=None):
        steps = [r if isinstance(r, Step) else Step.of(r, i) for i, r in enumerate(rows)]
        self.groups = {g: [s for s in steps if s.group == g] for g in GROUPS}
        self.steps = self.groups['main']
        self.limits, self.cycles, self.limit = dict(limits or {}), cycles, limit
        self.max_steps, self.on_init, self.on_cleanup = max_steps, init, cleanup
        self.ranges = dict(ranges or {})
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
        """Steps as text: lines (`0.5 knee=40`) or csv with a header row."""
        lines = [line.split('#')[0].strip() for line in text.strip().splitlines()]
        lines = [line for line in lines if line]
        if lines and ',' in lines[0] and '=' not in lines[0]:
            return cls(list(csv.DictReader(io.StringIO('\n'.join(lines)))), **kw)
        return cls([_line(line, n) for n, line in enumerate(lines)], **kw)

    def check(self, loop):
        """Every name a step uses, against `loop`: a set that nearly names a loop channel
        (a typo), a test of a channel nothing writes. One line a problem, the close match."""
        known = set(loop.channels()) | loop.reads()
        steps = [s for g in self.groups.values() for s in g]
        sets = {ch for s in steps for ch in list(s.setpoints) + list(s.adds)}
        problems = []
        for step in steps:
            for ch, value in step.setpoints.items():
                low, high = self.ranges.get(ch, (-math.inf, math.inf))
                if not low <= value <= high:
                    problems.append('row %d: %s=%g is outside %g..%g' % (
                        step.row, ch, value, low, high))
            for ch in list(step.setpoints) + list(step.adds):
                near = _near(ch, known)
                if ch not in known and near:
                    problems.append('row %d: nothing reads %s%s' % (step.row, ch, near))
            for ch in step.limits:
                if ch not in known and ch not in sets:
                    problems.append('row %d: no channel %s to test%s' % (
                        step.row, ch, _near(ch, known)))
        return problems

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
        """The whole sequence on `loop`, checked first; cleanup however it ends."""
        problems = self.check(loop)
        if problems:
            raise RigError('the program names what the loop has not:\n' + '\n'.join(problems))
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
