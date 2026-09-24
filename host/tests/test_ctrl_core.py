"""The board's parts and feedback (ctrl/), stepped beside host/machine/parts.py."""
import ctypes
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))

from test_modbus_core import Report, build, find_cc          # noqa: E402
from machine import parts                                     # noqa: E402
from machine.controller import Feedback, Loop, Polled         # noqa: E402
from machine.roles import Part                                # noqa: E402

REPO = os.path.dirname(os.path.dirname(HERE))
CTRL = os.path.join(REPO, 'ctrl')
SOURCES = [os.path.join(CTRL, 'test', 'harness.c'), os.path.join(CTRL, 'src', 'ctrl.c')]
DT = 0.002
KT, J, B = 0.07, 2e-5, 1e-5

#: Each kind with parameters away from its defaults, and what drives it.
CASES = {
    'Gain': (parts.Gain(-2.5), 'x'),
    'Slew': (parts.Slew(150.0), 'x'),
    'Wrap': (parts.Wrap(123.4), 'x'),
    'LowPass': (parts.LowPass(0.03), 'x'),
    'SpeedKalman': (parts.SpeedKalman(KT, J, B, 1e4, 25.0), 'estimate'),
    'PI': (parts.PI(4e-3, 0.8, 1.5), 'regulate'),
    'AngleHold': (parts.AngleHold(7.0, 0.3, 2.0, 5.0, 1.5), 'regulate'),
    'Direct': (parts.Direct(2.0), 'regulate'),
    'SpeedPI': (parts.SpeedPI(3.0, 2.0, KT, J, B, 1e-9, 0.10472), 'regulate'),
}


class C:

    """ctrl/ behind its harness."""

    def __init__(self, lib):
        f, p = ctypes.c_float, ctypes.c_void_p
        for name, res, args in (('ctl_kind', ctypes.c_int, [ctypes.c_char_p]),
                                ('ctl_params', ctypes.c_int, [ctypes.c_int]),
                                ('ctl_part', p, [ctypes.c_int, ctypes.POINTER(f)]),
                                ('ctl_feedback', p, []),
                                ('ctl_slot', ctypes.c_int, [p, ctypes.c_int, ctypes.c_int,
                                                            ctypes.POINTER(f)]),
                                ('ctl_free', None, [p]),
                                ('ctl_filter', f, [p, f, f]),
                                ('ctl_estimate', f, [p, f, f, f]),
                                ('ctl_regulate', f, [p, f, f, f, f, ctypes.c_int, ctypes.c_int]),
                                ('ctl_step', f, [p, f, f, f]),
                                ('ctl_runner', p, []),
                                ('ctl_runner_feedback', p, [p]),
                                ('ctl_push', ctypes.c_int, [p, ctypes.c_uint16, f]),
                                ('ctl_free_rows', ctypes.c_int, [p]),
                                ('ctl_seconds', f, [p]),
                                ('ctl_clear', None, [p]),
                                ('ctl_tick', f, [p, f, f]),
                                ('ctl_counts', None, [p, ctypes.POINTER(ctypes.c_uint32)])):
            fn = getattr(lib, name)
            fn.restype, fn.argtypes = res, args
        self.lib = lib

    def kind(self, name):
        return self.lib.ctl_kind(name.encode())

    def part(self, py):
        """The C twin of a machine.parts part, its parameters in PARAMS order."""
        values = [float(getattr(py, n)) for n in py.PARAMS]
        return self.lib.ctl_part(self.kind(type(py).__name__),
                                 (ctypes.c_float * max(1, len(values)))(*values))


def drive(case, rng, n=400):
    """(inputs, python outputs) for one part over `n` steps."""
    py, how = case
    rows = []
    for i in range(n):
        if how == 'x':
            args = {'x': rng.uniform(0.0, 360.0) if isinstance(py, parts.Wrap)
                    else 100.0 * math.sin(i * 0.05) + rng.gauss(0.0, 5.0)}
            rows.append((args, py.step(DT, **args)['y']))
        elif how == 'estimate':
            args = {'measured': 200.0 + rng.gauss(0.0, 20.0), 'command': rng.uniform(-1.0, 1.0)}
            rows.append((args, py.step(DT, **args)['estimate']))
        else:
            args = {'setpoint': 40.0 * math.sin(i * 0.02), 'measured': rng.gauss(0.0, 10.0)}
            rows.append((args, py.step(DT, **args)['command']))
    return rows


def near(c, py, scale):
    return abs(c - py) <= 1e-4 * max(1.0, scale)


def test_every_part_has_its_twin(report, c):
    missing = [n for n in Part.KINDS if not c.kind(n)]
    report.check('every part a loop is wired from has a C twin', not missing, missing)
    wrong = [n for n in Part.KINDS if c.kind(n)
             and c.lib.ctl_params(c.kind(n)) != len(Part.KINDS[n].PARAMS)]
    report.check('and takes the parameters PARAMS names, as many', not wrong, wrong)
    report.check('a kind that is not there is refused', not c.lib.ctl_part(99, None))


def test_each_part_steps_as_the_host(report, c):
    rng = random.Random(7)
    for name, case in CASES.items():
        rows = drive(case, rng)
        part = c.part(case[0])
        scale = max(abs(v) for _, v in rows)
        worst, first = 0.0, None
        for i, (args, want) in enumerate(rows):
            if case[1] == 'x':
                got = c.lib.ctl_filter(part, DT, args['x'])
            elif case[1] == 'estimate':
                got = c.lib.ctl_estimate(part, DT, args['measured'], args['command'])
            else:
                got = c.lib.ctl_regulate(part, DT, args['setpoint'], args['measured'],
                                         0.0, 0, 0)
            worst = max(worst, abs(got - want))
            if first is None and not near(got, want, scale):
                first = (i, got, want)
        c.lib.ctl_free(part)
        report.check('%s: 400 steps as machine.parts steps them' % name, first is None,
                     'worst %.2g of %.3g' % (worst, scale) if first is None else first)


def test_a_feedback_composes_as_the_loop(report, c):
    rng = random.Random(3)
    now = {'sp': 0.0, 'm': 0.0}
    slots = {'prefilter': parts.Slew(400.0), 'measure': parts.Gain(1.0 / 7.0),
             'estimator': parts.SpeedKalman(KT, J, B, 1e4, 25.0),
             'regulator': parts.SpeedPI(3.0, 2.0, KT, J, B)}
    loop = Loop({'src': Polled(lambda: dict(now))}, {})
    loop.add('speed', Feedback(setpoint='src.sp', measured='src.m', **slots))
    f = c.lib.ctl_feedback()
    for i, slot in enumerate(('prefilter', 'measure', 'estimator', 'regulator')):
        py = slots[slot]
        values = [float(getattr(py, n)) for n in py.PARAMS]
        c.lib.ctl_slot(f, i, c.kind(type(py).__name__),
                       (ctypes.c_float * len(values))(*values))
    worst, off = 0.0, None
    for i in range(1000):
        now['sp'] = 200.0 if i > 50 else 0.0
        now['m'] = 7.0 * 150.0 * (1.0 - math.exp(-i / 200.0)) + rng.gauss(0.0, 20.0)
        want = loop.step(DT)['speed/command']
        got = c.lib.ctl_step(f, DT, now['sp'], now['m'])
        worst = max(worst, abs(got - want))
        if off is None and not near(got, want, 2.0):
            off = (i, got, want)
    c.lib.ctl_free(f)
    report.check('slew, gain, Kalman, SpeedPI: the command the Loop makes, pass by pass',
                 off is None, 'worst %.2g A' % worst if off is None else off)


def test_the_c_holds_a_speed(report, c):
    f = c.lib.ctl_feedback()
    for i, (kind, values) in enumerate((('Slew', [400.0]), (None, []), (None, []),
                                        ('SpeedPI', [3.0, 2.0, KT, J, B, 0.0, 1.0]))):
        if kind:
            c.lib.ctl_slot(f, i, c.kind(kind), (ctypes.c_float * len(values))(*values))
    w = 0.0
    for _ in range(int(3.0 / DT)):
        iq = c.lib.ctl_step(f, DT, 200.0, w)
        w += DT * (KT * iq - B * w) / J
    c.lib.ctl_free(f)
    report.check('the C loop holds 200 rad/s on the toy rotor within 2 %',
                 abs(w - 200.0) < 4.0, '%.1f rad/s' % w)


def test_rows_play_in_order(report, c):
    r = c.lib.ctl_runner()
    counts = (ctypes.c_uint32 * 2)()
    for ms, sp in ((100, 10.0), (200, 20.0), (50, -5.0)):
        c.lib.ctl_push(r, ms, sp)
    report.check('rows queued: their seconds', abs(c.lib.ctl_seconds(r) - 0.35) < 1e-4,
                 c.lib.ctl_seconds(r))
    out = [c.lib.ctl_tick(r, 0.001, 0.0) for _ in range(400)]    # no parts: the setpoint out
    report.check('each row held its milliseconds, in order',
                 out[:100] == [10.0] * 100 and out[100:300] == [20.0] * 200
                 and out[300:350] == [-5.0] * 50, (out[99], out[100], out[299], out[300]))
    c.lib.ctl_counts(r, counts)
    report.check('dry, the last setpoint holds; the ticks without a row counted',
                 out[350:] == [-5.0] * 50 and counts[0] == 3 and counts[1] == 50, list(counts))
    took = sum(c.lib.ctl_push(r, 10, 1.0) for _ in range(70))
    report.check('the ring takes 63 rows and refuses the next', took == 63
                 and c.lib.ctl_free_rows(r) == 0, took)
    c.lib.ctl_tick(r, 0.001, 0.0)
    c.lib.ctl_clear(r)
    report.check('clear: nothing queued, the setpoint held',
                 c.lib.ctl_seconds(r) == 0.0 and c.lib.ctl_tick(r, 0.001, 0.0) == 1.0)
    c.lib.ctl_free(r)


def test_rows_drive_a_joint(report, c):
    """A hold on the toy spring: rows of angle, AngleHold's command a spring's zero."""
    r = c.lib.ctl_runner()
    f = c.lib.ctl_runner_feedback(r)
    for i, (kind, values) in enumerate((('Slew', [90.0]), ('Wrap', [0.0]), (None, []),
                                        ('AngleHold', [7.0, 0.0, 0.0, 5.0, 6.0]))):
        if kind:
            c.lib.ctl_slot(f, i, c.kind(kind), (ctypes.c_float * len(values))(*values))
    for ms, deg in ((500, 30.0), (500, -20.0)):
        c.lib.ctl_push(r, ms, deg)
    deg, w, cmds = 0.0, 0.0, []
    for _ in range(1200):
        theta = c.lib.ctl_tick(r, 0.001, deg % 360.0)
        torque = 0.735 * math.sin(theta - math.radians(deg) * 7.0) - 4e-3 * w
        w += 0.001 * torque / 2e-5
        deg += math.degrees(w * 0.001)
        cmds.append(deg)
    report.check('rows of angle: the joint slews to 30, then to -20 and holds',
                 abs(cmds[480] - 30.0) < 1.5 and abs(cmds[-1] + 20.0) < 1.5,
                 '%.1f at 0.48 s, %.1f at 1.2 s' % (cmds[480], cmds[-1]))
    c.lib.ctl_free(r)


ROSTER = (test_every_part_has_its_twin, test_each_part_steps_as_the_host,
          test_a_feedback_composes_as_the_loop, test_the_c_holds_a_speed,
          test_rows_play_in_order, test_rows_drive_a_joint)


def main():
    cc = find_cc()
    if cc is None:
        print('  SKIP  no host C compiler; setup.ps1 installs one')
        print('\n0 passed, 0 failed')
        return 0
    lib_path, warnings = build(cc, SOURCES, [os.path.join(CTRL, 'inc')], name='ctrlcore')
    c = C(ctypes.CDLL(lib_path))
    report = Report()
    report.check('ctrl/ builds warning-free with the firmware flags',
                 not warnings, '; '.join(warnings[:3]))
    for test in ROSTER:
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report, c)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
