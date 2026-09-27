#!/usr/bin/env python3
"""The tty's pages on the stand-in and on an emulated board: the stand-in is the ground truth.

    python tools/dev/ab.py                                  # every page but SKIPPED
    python tools/dev/ab.py thermal_observer --port native://?world=bench

Pages on the relay, the longest first, a process and an emulator each (tools.dev.focus,
`--jobs`), each killed at PAGE_S. Each page's own main runs headless (tools/render/page.frame), SIMULATED then `--port`. Every
device read either side takes, at any depth - a method of a board device or of the stand-in's
for it - is recorded by what it is (`thermal.state`, `analog.read(3)`), a field at a time, on
the board's clock. The report, per page: a read of the board's own the emulated side never gave
or gave as None; each shared number whose range over the board seconds both runs cover parts
from the stand-in's by more than `--apart`; each number outside what the physics allows.
"""
import argparse
import importlib
import inspect
import math
import numbers
import pkgutil
import os
import re
import sys

import coaxial.acquire
import coaxial.devices
import coaxial.simulated
from coaxial.comm.hostclock import WALL, clock_of
from coaxial.comm.session import standing
from coaxial.devices.subsystem import Subsystem
from coaxial.rig import Coaxial63100
from machine.roles import Input
from tools.dev.focus import WORKER_GB, Job, relay
from tools.render import page as pages

#: humanoid is the stand-in's alone; chat is the local model's; render has no board.
SKIPPED = ('humanoid', 'chat', 'render')

#: Frames for a board window that holds the page's slowest cycle: the tumble's 25.6 s, the
#: thermal page's 36 s load, the rotor's 16 s demo.
FRAMES = {'orientation': 600, 'thermal_observer': 200, 'rotor_observer': 420}

#: A page's time, both sides, s.
PAGE_S = 600

#: Methods that act, not read.
WRITES = frozenset(('configure', 'write', 'on', 'off', 'hold', 'reset', 'trigger', 'load_cycle',
                    'situation', 'fast_forward', 'save', 'stop', 'start', 'close', 'open'))

#: Fields the stand-in has and a board cannot: its ground truth, a part's identity - the
#: stand-in's says it is invented - a check word, its value's function, and the ISR's cycles,
#: the part's on Renode, none on native, invented on the stand-in.
IGNORED = re.compile(r'thermal\.(identification\.)?truth\..*|imu\.product_id\..*|.*\.crc'
                     r'|drive\.state\.cycles\..*')

#: Counts and clocks from boot: a side that counts where the other stands still is a fault,
#: their values are not compared.
COUNTERS = re.compile(r'.*\.(updates|steps|cycles|periods|trips|errors|now|seconds|cargoes|'
                      r'keepalive|isr_cycles_last|isr_cycles_max|exit_ticks_max)')

#: The DC link's full scale, V (invariant 11).
LINK_FS = 78.15

#: An electrical speed past this share of the no-load speed is lost: the link cannot spin the
#: motor there (terminal.views.rotor.motions.LOST).
LOST = 1.5

TEMPERATURE = r'(ntc|mcu|afe|ambient|ambient_c|expected_ntc|winding_c|nodes\.\w+)'


def _no_load(rec):
    """The electrical no-load speed on the highest link seen, rad/s; None unknown."""
    lam, vdc = rec.get('drive.params.motor_lambda'), rec.get('drive.state.vdc')
    return vdc / (math.sqrt(3.0) * lam) if lam and vdc else None


def _speed(rec):
    w = _no_load(rec)
    return (-LOST * w, LOST * w) if w else None


def _flux(rec):
    lam = rec.get('drive.params.motor_lambda')
    return (0.5 * lam, 2.0 * lam) if lam else None


def _current(rec):
    trip = rec.get('drive.params.drv_i_trip')
    return (-1.5 * trip, 1.5 * trip) if trip else None


#: (field pattern, (low, high) or its function of the run's record[, the read's field that
#: says it is valid]): what the physics allows.
PLAUSIBLE = (
    (r'thermal\.(state|budget|identification)\.%s' % TEMPERATURE, (-40.0, 200.0)),
    (r'thermal\.budget\.derate', (0.0, 1.0)),
    (r'thermal\.budget\.used\.\w+', (0.0, 1.05)),
    (r'(drive\.state|observers\.read)\.omega_hat|observers\.read\.(omega|flux_omega|dual_omega)'
     r'|plant\.read\.omega', _speed),
    (r'observers\.read\.lambda_hat', _flux, 'valid'),
    (r'(drive\.state|plant\.read)\.(id|iq)', _current),
    (r'(drive\.state|plant\.read)\.vdc', (0.0, LINK_FS)),
    (r'imu\.state\.quaternion\.\d', (-1.001, 1.001)),
    (r'angle\.state\.degrees', (0.0, 360.0)),
)


def flat(value, prefix=''):
    """{dotted.key: leaf} of nested dicts and lists."""
    if isinstance(value, dict):
        out = {}
        for key, item in value.items():
            out.update(flat(item, '%s%s.' % (prefix, key)))
        return out
    if isinstance(value, (list, tuple)):
        out = {}
        for i, item in enumerate(value):
            out.update(flat(item, '%s%d.' % (prefix, i)))
        return out
    return {prefix[:-1]: value}


def number(value):
    return isinstance(value, numbers.Real) and not isinstance(value, bool)


#: A quaternion's four, as the IMU's reads carry them.
QUATERNION = frozenset(('i', 'j', 'k', 'real'))


def hemisphere(value):
    """`value` with every quaternion on the real >= 0 side: q and -q are one attitude, and
    a range over either sign says nothing."""
    if isinstance(value, dict):
        if set(value) == QUATERNION and number(value['real']) and value['real'] < 0:
            return {key: -item for key, item in value.items()}
        return {key: hemisphere(item) for key, item in value.items()}
    return value


def argument(value):
    """A call's argument as its read's name carries it: a scalar itself, anything else its
    type - a layout dict differs a field between the sides and would part every read."""
    return repr(value) if isinstance(value, (numbers.Number, str, type(None))) \
        else type(value).__name__


class Tape:
    """What a run's reads gave: {read: [(board s, {field: value})]}."""

    def __init__(self):
        self.reads = {}
        self.stamp = WALL
        self.began = None
        #: What each rig the page opened talks to: live, emulated or simulated.
        self.standing = []
        #: The page's own exit, where it took one.
        self.exited: object = None

    def take(self, read, owner, reply):
        clock = clock_of(owner)
        if clock.virtual:
            self.stamp = clock
        now = self.stamp.now()
        if self.began is None:
            self.began = now
        fields = flat(hemisphere(reply)) if isinstance(reply, (dict, list, tuple)) \
            else {'': reply}
        self.reads.setdefault(read, []).append((now - self.began, fields))

    def span(self):
        return max((rows[-1][0] for rows in self.reads.values()), default=0.0)

    def span_of(self, name, when=None):
        """(low, high) of a number over the reads where `when`, a field of the same read, is
        true - every read without one; None where none gave one."""
        values = []
        for read, rows in self.reads.items():
            if not name.startswith(read + '.'):
                continue
            field = name[len(read) + 1:]
            values += [row[field] for _, row in rows
                       if number(row.get(field)) and (when is None or row.get(when))]
        return (min(values), max(values)) if values else None

    def fields(self, until=None):
        """{read.field: [last, low, high]} over the reads up to `until` board s."""
        out = {}
        for read, rows in self.reads.items():
            for at, fields in rows:
                if until is not None and at > until:
                    break
                for key, value in fields.items():
                    name = read + ('.' + key if key else '')
                    if IGNORED.fullmatch(name):
                        continue
                    was = out.get(name)
                    if number(value) and was is not None and number(was[1]):
                        out[name] = [value, min(was[1], value), max(was[2], value)]
                    else:
                        out[name] = [value] + [value if number(value) else None] * 2
        return out


def _devices():
    """{class: its read's name}: the board's devices and acquisition and the stand-in's, a
    stand-in's by the board's it stands in for."""
    out = {}
    for package in (coaxial.devices, coaxial.acquire, coaxial.simulated):
        for info in pkgutil.walk_packages(package.__path__, package.__name__ + '.'):
            module = importlib.import_module(info.name)
            for name, cls in inspect.getmembers(module, inspect.isclass):
                if cls.__module__ != module.__name__:
                    continue
                if issubclass(cls, (Subsystem, Input)) or name.startswith('Simulated'):
                    out[cls] = re.sub(r'(?<!^)(?=[A-Z])', '_',
                                      name.removeprefix('Simulated')).lower()
    return out


def board_reads():
    """{(device, method)}: what the board's own classes answer - a read only the stand-in has
    is its own working, not a read the emulated side left out."""
    board = set()
    for cls, device in _devices().items():
        if cls.__module__.startswith(('coaxial.devices', 'coaxial.acquire')):
            board |= {(device, name) for name in dir(cls) if not name.startswith('_')}
    return board


def recorded(tape):
    """Every read wrapped to write to `tape`; the originals, to put back."""
    originals = []
    for cls, device in _devices().items():
        for name in dir(cls):
            method = getattr(cls, name)
            if (name.startswith('_') or name in WRITES or name.startswith('set')
                    or not inspect.isfunction(inspect.getattr_static(cls, name))):
                continue
            originals.append((cls, name, device, method, name in vars(cls)))
    for cls, name, device, method, own in originals:
        def wrapper(self, *args, _original=method, _read='%s.%s' % (device, name), **kwargs):
            reply = _original(self, *args, **kwargs)
            # A block of records is a row a record: the desk's meters are the DAQ's.
            rows = ([reply] if isinstance(reply, dict) or number(reply) else
                    [r for r in reply if isinstance(r, dict)]
                    if isinstance(reply, (list, tuple)) else [])
            said = ','.join([argument(a) for a in args]
                            + ['%s=%s' % (k, argument(v)) for k, v in sorted(kwargs.items())])
            for row in rows:
                tape.take(_read + ('(%s)' % said if said else ''), self, row)
            return reply
        setattr(cls, name, wrapper)
    return [(cls, name, method if own else None) for cls, name, _, method, own in originals]


def run(page, port, frames, size):
    """The page's reads over `frames` frames, on the stand-in or on `port`."""
    tape = Tape()
    kept = recorded(tape)
    opened = Coaxial63100.open

    def open_(self):
        got = opened(self)
        tape.standing.append(standing(self.origin))
        return got
    Coaxial63100.open = open_
    try:
        pages.frame(page, *size, frames=frames, port=port)
    except SystemExit as exc:
        tape.exited = exc.code
    finally:
        Coaxial63100.open = opened
        for cls, name, original in kept:
            if original is None:
                delattr(cls, name)
            else:
                setattr(cls, name, original)
    return tape


def apart(truth, other, share):
    """Whether two ranges part by more than `share` of the larger's span or magnitude."""
    scale = max(abs(truth[1]), abs(truth[2]), abs(other[1]), abs(other[2]), 1e-9)
    return (abs(truth[2] - other[2]) > share * scale) or (abs(truth[1] - other[1]) > share * scale)


def implausible(side, tape):
    """(lines, faults): the numbers outside what PLAUSIBLE allows, each over the reads that
    say it is valid."""
    fields = tape.fields()
    rec = {key: value[2] if number(value[2]) else value[0] for key, value in fields.items()}
    lines = []
    for key in sorted(fields):
        for pattern, allowed, *when in PLAUSIBLE:
            if not re.fullmatch(pattern, key):
                continue
            span = tape.span_of(key, when[0] if when else None)
            bounds = allowed(rec) if callable(allowed) else allowed
            if span is None or not bounds:
                break
            low, high = span
            if low < bounds[0] or high > bounds[1]:
                lines.append('IMPLAUS  %-8s %s  %.4g..%.4g, allowed %.4g..%.4g'
                             % (side, key, low, high, bounds[0], bounds[1]))
            break
    return lines, len(lines)


def report(truth, other, share):
    """(lines, faults): the stand-in's reads against the other side's over the board seconds
    both cover, and each side against the physics."""
    window = min(truth.span(), other.span())
    a, b = truth.fields(window), other.fields(window)
    lines, faults = ['\nwindow   %.1f board s (stand-in %.1f, emulated %.1f)'
                     % (window, truth.span(), other.span())], 0
    for side, tape in (('stand-in', truth), ('emulated', other)):
        if tape.exited is not None:
            lines.append('EXITED   %s: the page exited %r' % (side, tape.exited))
            faults += 1
    if other.standing and set(other.standing) != {'emulated'}:
        lines.append('FELLBACK the emulated side ran %s' % ', '.join(sorted(set(other.standing))))
        faults += 1
    board = board_reads()
    for key in sorted(a):
        if key not in b:
            read = re.match(r'(\w+)\.(\w+)', key)
            if read and (read.group(1), read.group(2)) in board:
                lines.append('MISSING  %s (stand-in %r)' % (key, a[key][0]))
                faults += 1
        elif b[key][0] is None and a[key][0] is not None:
            lines.append('NONE     %s (stand-in %r)' % (key, a[key][0]))
            faults += 1
        elif COUNTERS.fullmatch(key):
            if a[key][2] != a[key][1] and b[key][2] == b[key][1]:
                lines.append('STILL    %s  stand-in counts %.4g..%.4g, emulated stays at %.4g'
                             % (key, a[key][1], a[key][2], b[key][1]))
                faults += 1
        elif a[key][1] is not None and b[key][1] is not None and apart(a[key], b[key], share):
            lines.append('APART    %s  stand-in %.4g..%.4g  emulated %.4g..%.4g'
                         % (key, a[key][1], a[key][2], b[key][1], b[key][2]))
            faults += 1
    lines += ['extra    %s' % key for key in sorted(set(b) - set(a))]
    for side, tape in (('stand-in', truth), ('emulated', other)):
        said, count = implausible(side, tape)
        lines += said
        faults += count
    return lines, faults


def compare(page, args):
    """One page in this process against the stand-in: its report printed, its faults."""
    frames = args.frames or FRAMES.get(page, 150)
    truth = run(page, None, frames, args.size)
    other = run(page, args.port, frames, args.size)
    lines, faults = report(truth, other, args.apart)
    for line in lines:
        if args.all or not line.startswith('extra'):
            print(line)
    print('%s: %d faults against the stand-in and the physics, %s\n'
          % (page, faults, args.port), flush=True)
    return faults


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('page', nargs='*',
                        help='of %s; every one but %s by default'
                        % (', '.join(sorted(pages.PAGES)), ' and '.join(SKIPPED)))
    parser.add_argument('--port', default='emulator://')
    parser.add_argument('--frames', type=int, help='every page this many, not FRAMES or 150')
    parser.add_argument('--size', type=int, nargs=2, default=(200, 60), metavar=('WIDTH', 'HEIGHT'))
    parser.add_argument('--apart', type=float, default=0.3,
                        help='a number marked when its range parts by more than this share')
    parser.add_argument('--all', action='store_true', help='the extras too')
    parser.add_argument('--jobs', type=int, help='batons: pages at once, the physical cores')
    args = parser.parse_args(argv)
    unknown = sorted(set(args.page) - set(pages.PAGES))
    if unknown:
        parser.error('no page %s' % ', '.join(unknown))
    chosen = args.page or [p for p in sorted(pages.PAGES) if p not in SKIPPED]
    if len(chosen) == 1:
        return 1 if compare(chosen[0], args) else 0
    # A process a page: a fresh stand-in and a fresh board each, a page's exit its own.
    rest = [a for a in (sys.argv[1:] if argv is None else argv) if a not in chosen]
    failed = []
    jobs = [Job(page, [sys.executable, '-X', 'utf8', os.path.abspath(__file__), page] + rest,
                WORKER_GB, PAGE_S)
            for page in sorted(chosen, key=lambda p: -FRAMES.get(p, 150))]
    for job, out, code, took in relay(jobs, args.jobs):
        page = job.name
        sys.stdout.write(out)
        if code is None:
            print('%s: out of time at %.0f s\n' % (page, PAGE_S))
        if code != 0:
            failed.append(page)
        sys.stdout.flush()
    print('%d of %d pages part from the stand-in or the physics: %s'
          % (len(failed), len(chosen), ', '.join(failed) or 'none'))
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
