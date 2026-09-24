"""A test stand in miniature: a table of setpoints and jumps, run through a controller.

    seq = Sequencer.read('steps.csv', limits={'iq_ref': {'HH': 1.8}}, init=arm, cleanup=disarm)
    seq = Sequencer.parse(text)          # a model's answer as it came: lines or csv
    out = seq.run(loop)                  # checked first; out.status done | tripped | limit
    print(out.summary())                 # a few lines back to the model
    print(prompt(loop))                  # GRAMMAR and the loop's card: all a model needs

A step is a row of a .csv or an .xlsx's first sheet, or a line: `0.5 knee=40 hip=-20`
(the bare number is seconds, the rest name=value, # a comment). A step sets, then waits:
    seconds            the step time: waited out, a timeout alarm and on - no alarm
                       where the row branches: its timeout is its answer (`else`)
    <channel>          a target: done once it reads back within `band` (blank: holds)
    <channel>+         add to it: a counter
    <channel>.GE .LE   a test: done once >= / <= the value; a row's tests replace
                       its targets' arrival
    wait               at (the default): until its targets and tests hold; time: its
                       seconds whatever, a timed move. Nothing to wait for: its seconds
    band               how near a target counts, its unit (1 % of its range)
    <channel>.L .H     alarm: logged as it comes and as it goes, the run goes on
    <channel>.LL .HH   trip: to cleanup at once. Levels hold from their row on; inf clears
    label              a row's name                   group    init | cleanup | blank (main)
    then / else        where to go when done / when timed out
    goto, times        after the row, to `goto` `times` times (blank 1, inf), then on
0 seconds is a decision: its tests once, then `then` or `else`, no alarm. A target is a
label, or `stop`: the main group is over. Counters, adding and branching on a test make
it a counter machine - any program, as data. Levels, the alarm log and trips are the
`alarms` handler's (`machine.alarms`; `limits` make one): the sequencer calls its hooks
and goes to cleanup when it trips. `ranges` {channel: (low, high)} refuse a setpoint
outside before anything runs and size `band`; `cycles` repeats the main group; `limit`
seconds or `max_steps` rows end a run.
"""
import csv
import io
import math
import re
from collections import namedtuple

from machine.alarms import LEVELS, Alarms, Tripped, crossed
from machine.errors import MachineError

#: The columns that steer rather than set.
STEER = ('seconds', 'label', 'goto', 'times', 'group', 'then', 'else', 'wait', 'band')
TESTS = ('GE', 'LE')
WAITS = ('at', 'time')

#: A target's band where no range sizes it: this share of the target, at least FLOOR.
BAND_SHARE, BAND_FLOOR = 0.01, 1e-3
GROUPS = ('init', 'main', 'cleanup')
STOP = 'stop'

#: What a model is told, with the loop's card (`prompt`).
GRAMMAR = """One step a line: seconds, name=value; # a comment. A csv: names as columns.
  knee=40         a target: the step waits till it reads back (band=, 1 % of range)
  knee.deg.GE=38  waits till >= 38 instead (.LE <=)       n+=1  add to a counter
  seconds         the step time: past it an alarm, and on   wait=time  lasts it all
  knee.deg.H=60   alarm (.L below); .HH .LL trip to cleanup; from their row on
  label=up then=up else=down  go there when done / timed out (then no alarm)
  goto=up times=3  back 3 times    group=init | cleanup, runs always
  run=walk times=4 stride=25  a routine below, its params by name
0 seconds: a decision, no time passes. A jump to stop ends the program."""


class Outcome(namedtuple('Outcome', 'rows status reason steps alarms')):

    """A finished run: every pass's channels, how it ended, why, each step it took (row,
    label, done | time | timeout, seconds) and the alarm log, a line each."""

    def summary(self, *channels, most=12, alarms=4):
        """A few lines: the ending, the steps (at most `most`), the first `alarms`,
        `channels` at the end."""
        seconds = sum(s[3] for s in self.steps)
        lines = ['%s after %.2f s, %d steps%s' % (self.status, seconds, len(self.steps),
                                                  ': ' + self.reason if self.reason else '')]
        shown = self.steps if len(self.steps) <= most else \
            self.steps[:most - 2] + [None] + self.steps[-1:]
        for step in shown:
            lines.append('  ...' if step is None else '  row %d%s %.2f s %s' % (
                step[0], ' ' + step[1] if step[1] else '', step[3], step[2]))
        if self.alarms:
            lines.append('  %d alarm%s%s' % (len(self.alarms), 's' if len(self.alarms) > 1
                                            else '', ': ' + '; '.join(self.alarms[:alarms])
                                            if alarms else ''))
        last = self.rows[-1] if self.rows else {}
        if channels:
            lines.append('  end: ' + ', '.join('%s %.4g' % (c, last[c]) for c in channels
                                               if c in last))
        return '\n'.join(lines)


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
            raise MachineError('row %d: %r is neither seconds nor name=value' % (row, token))
    return cells


_PARAM = re.compile(r'\{(-?)(\w+)(?:\*([0-9.]+))?\}')


def _format(text, params):
    """A routine's text with {p}, {-p} and {p*k} as numbers."""
    def value(m):
        sign, name, factor = m.groups()
        return '%g' % ((-1.0 if sign else 1.0) * params[name] * float(factor or 1.0))
    return _PARAM.sub(value, text)


def _expand(lines, routines, depth=0):
    """Lines with every `run=` line replaced by its routine's lines, `times` over."""
    if depth > 8:
        raise MachineError('routines call each other more than 8 deep')
    out = []
    for n, line in enumerate(lines):
        cells = dict(token.partition('=')[::2] for token in line.split() if '=' in token)
        if 'run' not in cells:
            out.append(line)
            continue
        name = cells.pop('run')
        if name not in routines:
            raise MachineError('line %d: no routine %s%s - there are %s' % (
                n, name, _near(name, routines), ', '.join(routines) or 'none'))
        routine = routines[name]
        times = int(float(cells.pop('times', 1)))
        unknown = sorted(set(cells) - set(routine.defaults))
        if unknown:
            raise MachineError('line %d: %s takes %s, not %s' % (
                n, name, ', '.join(routine.defaults), ', '.join(unknown)))
        params = dict(routine.defaults, **{k: float(v) for k, v in cells.items()})
        body = [x for x in _format(routine.text, params).splitlines() if x.strip()]
        out += _expand(body, routines, depth + 1) * times
    return out


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

    """One row: its time, what it sets, adds and tests, the levels it brings, where next."""

    def __init__(self, seconds, setpoints=None, adds=None, limits=None, label=None, group='main',
                 then=None, otherwise=None, goto=None, times=1.0, row=0, tests=None, wait='at',
                 band=None):
        self.seconds = float(seconds)
        self.setpoints, self.adds, self.limits = dict(setpoints or {}), dict(adds or {}), \
            dict(limits or {})
        self.tests, self.wait, self.band = dict(tests or {}), wait, band
        self.label, self.group, self.row = label, group, row
        self.then, self.otherwise, self.goto, self.times = then, otherwise, goto, times

    @classmethod
    def of(cls, cells, row=0):
        """A table row."""
        if _value(cells.get('seconds')) is None:
            raise MachineError('row %d needs seconds: %r' % (row, cells))
        group = _text(cells.get('group')) or 'main'
        if group not in GROUPS:
            raise MachineError('row %d: group %s - there are %s' % (row, group, ', '.join(GROUPS)))
        wait = _text(cells.get('wait')) or 'at'
        if wait not in WAITS:
            raise MachineError('row %d: wait=%s - it is %s' % (row, wait, ' or '.join(WAITS)))
        setpoints, adds, limits, tests = {}, {}, {}, {}
        for key, cell in cells.items():
            try:
                value = _value(cell) if key not in STEER else None
            except ValueError:
                raise MachineError('row %d: %s=%s is not a number' % (row, key, cell)) from None
            if value is None:
                continue
            channel, _, level = key.rpartition('.')
            if level in LEVELS and channel:
                limits.setdefault(channel, {})[level] = value
            elif level in TESTS and channel:
                tests.setdefault(channel, {})[level] = value
            elif key.endswith('+'):
                adds[key[:-1]] = value
            else:
                setpoints[key] = value
        times, band = _value(cells.get('times')), _value(cells.get('band'))
        if band is not None and not band > 0:
            raise MachineError('row %d: band=%g - a band is above 0' % (row, band))
        return cls(_value(cells['seconds']), setpoints, adds, limits, _text(cells.get('label')),
                   group, _text(cells.get('then')), _text(cells.get('else')),
                   _text(cells.get('goto')), 1.0 if times is None else times, row, tests, wait,
                   band)

    def targets(self):
        return [t for t in (self.then, self.otherwise, self.goto) if t]

    def unmet(self, bus, back, band):
        """What the step still waits for, a phrase each: its tests not holding, or with none,
        its targets not read back within their band (`back` {target: readback},
        `band(target)`)."""
        out = []
        for target, value in ({} if self.tests else self.setpoints).items():
            channel = back.get(target)
            if channel is not None and channel in bus and \
                    abs(bus[channel] - value) > (self.band or band(target, value)):
                out.append('%s %.4g of %.4g' % (channel, bus[channel], value))
        for channel, tests in self.tests.items():
            for test, bound in tests.items():
                if channel not in bus or not crossed(bus[channel], test, bound):
                    out.append('%s %s %s %.4g' % (channel, '%.4g' % bus[channel] if channel in bus
                                                   else 'unread', 'below' if test == 'GE'
                                                   else 'above', bound))
        return out

    def timed(self, back):
        """Whether its seconds end it: wait=time, or nothing to wait for."""
        return self.wait == 'time' or not (self.tests or any(t in back for t in self.setpoints))

    def __repr__(self):
        jumps = ''.join(' %s %s' % (k, v) for k, v in (
            ('then', self.then), ('else', self.otherwise),
            ('goto', '%s x%g' % (self.goto, self.times) if self.goto else None)) if v)
        return '<%s %s%.3g s%s%s%s%s%s%s>' % (
            self.group, '%s: ' % self.label if self.label else '', self.seconds,
            ' wait=time' if self.wait == 'time' else '',
            ' set %s' % self.setpoints if self.setpoints else '',
            ' add %s' % self.adds if self.adds else '',
            ' tests %s' % self.tests if self.tests else '',
            ' levels %s' % self.limits if self.limits else '', jumps)


class Sequencer:

    """init rows, the main rows with their jumps `cycles` times, cleanup rows - always."""

    def __init__(self, rows, limits=None, cycles=1, limit=None, max_steps=100000, init=None,
                 cleanup=None, ranges=None, alarms=None):
        steps = [r if isinstance(r, Step) else Step.of(r, i) for i, r in enumerate(rows)]
        self.groups = {g: [s for s in steps if s.group == g] for g in GROUPS}
        self.steps = self.groups['main']
        self.alarms = alarms if alarms is not None else Alarms(limits)
        self.cycles, self.limit = cycles, limit
        self.max_steps, self.on_init, self.on_cleanup = max_steps, init, cleanup
        self.ranges = dict(ranges or {})
        self.at, self.taken = 0, {}
        if STOP in self.labels():
            raise MachineError('%s is where a jump ends the run, not a label' % STOP)
        missing = sorted({t for s in self.steps for t in s.targets()}
                         - set(self.labels()) - {STOP})
        if missing:
            raise MachineError('a jump names no main row: %s; the labels are %s'
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
    def parse(cls, text, routines=None, **kw):
        """Steps as text: lines (`0.5 knee=40`, `0 run=walk times=2`) or csv with a header."""
        lines = [line.split('#')[0].strip() for line in text.strip().splitlines()]
        lines = [line for line in lines if line]
        if lines and ',' in lines[0] and '=' not in lines[0]:
            return cls(list(csv.DictReader(io.StringIO('\n'.join(lines)))), **kw)
        lines = _expand(lines, routines or {})
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
            for ch in list(step.limits) + list(step.tests):
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
        """Past the current main row: then when done, else when timed out, goto, the next."""
        here, step = self.at, self.steps[self.at]
        branch = step.otherwise if ended == 'timeout' else step.then
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

    def band(self, target, value):
        """How near `target` counts as there: 1 % of its range, else of its value."""
        low, high = self.ranges.get(target, (-math.inf, math.inf))
        if math.isfinite(high - low) and high > low:
            return BAND_SHARE * (high - low)
        return max(BAND_FLOOR, BAND_SHARE * abs(value))

    def _run(self, loop, step, out, watch, trips=True):
        """One step: the handler told, its setpoints and adds, then the loop until it is
        done, timed out, or tripped. 'done', 'time' (its seconds were the plan) or
        'timeout' (the handler told, unless a decision)."""
        self.alarms.step('row %d%s' % (step.row, ' ' + step.label if step.label else ''))
        self.alarms.set(step.limits)
        back = dict(zip(loop.targets(), loop.controlled()))
        timed = step.timed(back)
        left: list = [None]

        def check(loop):
            out['rows'].append(loop.read())
            if watch is not None:
                watch(loop)                   # a stop it asks for counts this pass
            self.alarms.check(loop.bus, trips)
            if timed:
                return False
            left[0] = step.unmet(loop.bus, back, self.band)
            return not left[0]

        start = loop.bus['t']
        writes = dict(step.setpoints, step=float(step.row),
                      **{k: loop.bus.get(k, 0.0) + v for k, v in step.adds.items()})
        ended = 'time'
        try:
            if step.seconds > 0:
                loop.move(step.seconds, check, **writes)
            else:
                loop.write(**writes)
                check(loop)
            if not timed:
                ended = 'done' if left[0] == [] else 'timeout'
            if ended == 'timeout' and step.seconds > 0 and not (step.then or step.otherwise):
                self.alarms.timeout('after %.3g s - %s' % (step.seconds, ', '.join(left[0] or ())))
        finally:
            out['steps'].append((step.row, step.label, ended, loop.bus['t'] - start))
        return ended

    def run(self, loop, watch=None):
        """The whole sequence on `loop`, checked first; cleanup however it ends."""
        self._checked(loop)
        out = {'rows': [], 'steps': []}
        first = self.alarms.begin()
        status, reason = 'done', None
        try:
            if self.on_init is not None:
                self.on_init(loop)
            for step in self.groups['init']:
                self._run(loop, step, out, watch)
            status, reason = self._main(loop, out, watch)
        except Tripped as trip:
            status, reason = 'tripped', str(trip)
        finally:
            for step in self.groups['cleanup']:
                self._run(loop, step, out, watch, trips=False)
            if self.on_cleanup is not None:
                self.on_cleanup(loop)
        return Outcome(out['rows'], status, reason, out['steps'], self.alarms.log[first:])

    def play(self, loop, watch=None, trips=True):
        """The main group only, on a loop already armed: no init or cleanup, the handler's
        levels and log carried on; a trip is raised (Tripped) for whoever armed it."""
        out = {'rows': [], 'steps': []}
        first = len(self.alarms.log)
        status, reason = self._main(loop, out, watch, trips)
        return Outcome(out['rows'], status, reason, out['steps'], self.alarms.log[first:])

    def seconds(self):
        """The main group's step times, each row once: the most it adds to a buffer."""
        return sum(step.seconds for step in self.steps)

    def _checked(self, loop):
        problems = self.check(loop)
        if problems:
            raise MachineError('the program names what the loop has not:\n' + '\n'.join(problems))

    def _main(self, loop, out, watch, trips=True):
        """The main group, `cycles` times, within `limit` and `max_steps`."""
        spent = 0.0
        for _ in range(int(self.cycles) if self.cycles != math.inf else 1 << 30):
            while self.at < len(self.steps):
                if self.limit is not None and spent >= self.limit:
                    return 'limit', '%.3g s spent' % spent
                if len(out['steps']) >= self.max_steps:
                    return 'limit', '%d steps taken' % self.max_steps
                before = loop.bus['t']
                ended = self._run(loop, self.steps[self.at], out, watch, trips)
                spent += loop.bus['t'] - before
                self._advance(ended)
            self.at, self.taken = 0, {}
        return 'done', None
