"""What the emulator's suite runs its groups by: its tests in order and timed, a group of one."""
import time


def run(report, tests):
    """`tests` in order, their names first, each one's seconds after it."""
    for test, *given in tests:
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        began = time.monotonic()
        test(report, *given)
        print('  %.0f s' % (time.monotonic() - began))


def one(test):
    """A group of `test` alone."""
    return lambda report, _names=(): run(report, [(test,)])
