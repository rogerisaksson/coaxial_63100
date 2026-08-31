"""Prove the rail reference counting, against the board.

Two ways for a power manager to be wrong, and this checks both:

    HELD WHEN IT SHOULD BE FREE   a leaked hold nobody can see or recover
    FREE WHEN IT SHOULD BE HELD   a rail switched off under a subsystem
                                  that had asked for it

Both have already happened here. The second is why the reference count
exists at all; the first is why every hold but the host's is a lease.

NOT A SUITE. It arms the gate stage to check that an acquire is refused
there, so it belongs beside the board, not in `run_tests.ps1` - which has to
pass on a bench with no board attached.

    python tools/power_check.py
"""
import argparse
import sys
import time

sys.path.insert(0, __file__.rsplit('tools', 1)[0])

from coaxial import Coaxial63100                          # noqa: E402
from coaxial.errors import NoReplyError, RigError         # noqa: E402

#: The lease in the firmware is 3 s. Wait past it, with room for the poll.
LEASE_WAIT_S = 4.5


class Check:

    """Counts, and prints one line per claim."""

    def __init__(self):
        self.bad = 0

    def __call__(self, claim, got, want):
        ok = got == want
        self.bad += not ok
        print('  %-4s %-52s %s' % ('ok' if ok else 'FAIL', claim,
                                   got if ok else '%r, wanted %r' % (got, want)))
        return ok


def afe(rig):
    return rig.board.power.state()['afe']


def quiet(fn, *a, **kw):
    """Run it, swallowing the link's occasional silence (FINDINGS)."""
    for _ in range(8):
        try:
            return fn(*a, **kw)
        except (NoReplyError, RigError):
            time.sleep(0.3)
    return None


def check_host_hold(rig, check):
    print('\nthe host\'s hold: taken by name, and it does NOT expire')
    quiet(rig.board.power.release_all)
    quiet(rig.board.afe.enable)
    st = afe(rig)
    check('host acquire switches the rail on', st['on'], True)
    check('and the mask names the host', st['users'], ['host'])
    check('the host hold carries no lease', st['leased'], [])

    print('  waiting %.1f s - longer than the firmware lease ...' % LEASE_WAIT_S)
    time.sleep(LEASE_WAIT_S)
    st = afe(rig)
    check('still on after the lease would have run out', st['on'], True)
    check('and still the host holding it', st['users'], ['host'])

    quiet(rig.board.afe.disable)
    st = afe(rig)
    check('release switches it off', st['on'], False)
    check('and nobody holds it', st['users'], [])


def check_observer_borrow(rig, check):
    print('\nthe thermal observer: borrows, then gives it back on its own')
    # What it was, so what goes back is what was there rather than a
    # copy of the firmware's default that goes stale when that moves.
    said = quiet(rig.board.thermal.state) or {}
    was = (said.get('sample_every_s', 30.0), said.get('sample_settle_s', 0.5))
    quiet(rig.board.thermal.set_sample, 2.0, 0.3)
    seen_thermal, seen_leased, high, n = False, False, 0, 0

    end = time.time() + 8.0
    while time.time() < end:
        st = quiet(afe, rig)
        if st is None:
            continue
        n += 1
        high += st['on']
        if 'thermal' in st['users']:
            seen_thermal = True
            seen_leased = seen_leased or ('thermal' in st['leased'])
        time.sleep(0.05)

    check('the thermal observer was seen holding the rail', seen_thermal, True)
    check('and its hold carries a lease', seen_leased, True)
    check('it is not held most of the time', high < n * 0.5, True)

    quiet(rig.board.thermal.set_sample, *was)
    time.sleep(1.5)
    check('and it is free again once sampling slows', afe(rig)['on'], False)


def check_armed_refusal(rig, check):
    print('\nwhile the gate stage is armed: an acquire is REFUSED')
    print('  AFE_ON high takes the drivers\' supply away - the gate is inverted')
    quiet(rig.board.afe.disable)
    quiet(rig.gates.arm, bypass_sto=True, ignore_interlock=True)
    try:
        st = afe(rig)
        check('the rail reports itself blocked', st['blocked'], True)
        check('and is off, so the drivers have supply', st['on'], False)

        # The thermal observer is sampling every 5 s; give it several chances.
        quiet(rig.board.thermal.set_sample, 1.0, 0.3)
        high = 0
        end = time.time() + 6.0
        while time.time() < end:
            st = quiet(afe, rig)
            high += bool(st and st['on'])
            time.sleep(0.05)
        check('the thermal observer never got it while armed', high, 0)
    finally:
        quiet(rig.board.thermal.set_sample, 5.0, 0.5)
        quiet(rig.gates.disarm)

    st = afe(rig)
    check('once disarmed it is no longer blocked', st['blocked'], False)


def check_release_all(rig, check):
    print('\nrelease_all: the way out of a leaked hold')
    quiet(rig.board.afe.enable)
    check('held before', afe(rig)['on'], True)
    quiet(rig.board.power.release_all)
    st = afe(rig)
    check('every hold dropped', st['users'], [])
    check('and the rail is off', st['on'], False)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', default='COM4')
    a = p.parse_args()

    check = Check()
    with Coaxial63100(port=a.port, power_afe=False) as rig:
        if not rig.origin.real:
            raise SystemExit('this needs the board - the stand-in switches '
                             'nothing and would prove nothing')
        try:
            check_host_hold(rig, check)
            check_observer_borrow(rig, check)
            check_armed_refusal(rig, check)
            check_release_all(rig, check)
        finally:
            quiet(rig.gates.disarm)
            quiet(rig.board.power.release_all)
            quiet(rig.board.thermal.set_sample, 5.0, 0.5)

    print('\n%s' % ('the rail is held exactly when something holds it'
                    if not check.bad else '%d claim(s) failed' % check.bad))
    return 1 if check.bad else 0


if __name__ == '__main__':
    sys.exit(main())
