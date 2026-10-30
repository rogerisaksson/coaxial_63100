"""What the gynoid's suites share: the report, and two rows compared."""
import math


class Report:
    def __init__(self):
        self.passed = self.failed = 0

    def check(self, name, ok, detail=''):
        self.passed += bool(ok)
        self.failed += (not ok)
        print('  %s  %-58s %s' % ('PASS' if ok else 'FAIL', name, detail))


def same(got, want):
    """Whether two rows agree: their lengths, each value within 1e-6 of its size, NaN with NaN."""
    return len(got) == len(want) and all(
        (math.isnan(a) and math.isnan(b)) or abs(a - b) < 1e-6 * max(1.0, abs(b)) for a, b in zip(got, want))


def suite_read():
    from machine.texts import dsp
    return dsp.read(dsp.SUITE)


def her_frame():
    from machine.process import worker
    return worker.session()['g']['fr']


def near(a, b):
    return math.isclose(a, b, rel_tol=1e-9, abs_tol=1e-9)
