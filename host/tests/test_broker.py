#!/usr/bin/env python3
"""coaxial.broker: one process owns the port, everything else asks it.

No board and no serial port. `serve` takes a transport, which is the seam
that makes this testable - a stand-in here answers the same three calls
`Board` makes of a real one, and the broker cannot tell.

There is deliberately no simulated broker. The stand-in board has no port
for two processes to contend over, and it speaks methods rather than frames,
so a broker in front of it would be forwarding nothing.

Run from the host directory:  python tests/test_broker.py
"""
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial import broker                                  # noqa: E402
from coaxial import errors                                  # noqa: E402

#: A TCP port of its own, so a suite run never fights a broker somebody
#: started at the bench - and an address FILE of its own for the same
#: reason. Writing the bench's pointed conformance at a broker that was
#: this suite's, on a port it then asked the wrong address to release.
ADDRESS = ('127.0.0.1', 8791)
broker.WHERE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            '.session.addr.test')


class Report:
    def __init__(self):
        self.passed = self.failed = 0

    def check(self, name, ok, detail=''):
        self.passed += bool(ok)
        self.failed += (not ok)
        print('  %s  %-58s %s' % ('PASS' if ok else 'FAIL', name, detail))


class Fake:

    """A transport that answers without a UART.

    Counts calls and records the order they arrived in, which is what proves
    the lock: the board is one slave on one wire, so two clients must
    interleave whole transactions and never half of one.
    """

    baud = 115200

    def __init__(self, delay=0.0):
        self.delay = delay
        self.calls = []
        self.inside = 0
        self.overlapped = False
        self.closed = False

    def request(self, unit, function, payload=b'', exact_payload=None,
                timeout=None):
        self.inside += 1
        self.overlapped = self.overlapped or self.inside > 1
        try:
            self.calls.append((unit, function, bytes(payload)))
            if self.delay:
                time.sleep(self.delay)
            if function == 0xEE:
                raise errors.ModbusException(unit, function, 4)
            if function == 0xED:
                raise errors.DeviceStateError('the board said no')
            return bytes([unit, function]) + bytes(payload)
        finally:
            self.inside -= 1

    def broadcast(self, function, payload=b'', settle=0.05):
        self.calls.append(('broadcast', function, bytes(payload)))

    def close(self):
        self.closed = True


def served(fake, address=ADDRESS):
    """Run a broker over `fake` in a thread, and hand back a stopper."""
    box = {}

    def run():
        # serve() blocks; the server object is reachable through the module
        # only while it lives, so the thread keeps it and the stopper waits.
        # linger=0: a broker that outlives its test answers the NEXT
        # test's clients through the previous test's transport.
        broker.serve('FAKE', 115200, address, transport=fake, linger=0.0)

    thread = threading.Thread(target=run, daemon=True)
    thread.start()

    # Wait on the ADDRESS FILE, not on a probe connection: serve() writes it
    # before it answers, and a probe that attached and closed would be the
    # last one out and take the broker down before the test began.
    for _ in range(200):                       # up in milliseconds
        if broker.serving():
            break
        time.sleep(0.02)
    box['thread'] = thread
    return box


def stop(address=ADDRESS):
    """Wait until nothing answers, then clear the address file.

    Waited on rather than assumed: a broker takes itself down when its last
    user goes, and the next test starting one while the old socket was still
    bound is a connection refused with no bug behind it. `attach` is safe to
    poll with - a look is not a use, so it cannot keep one alive.
    """
    for _ in range(200):
        probe = broker.attach(address, timeout=0.4)
        if probe is None:
            break
        probe.close()
        time.sleep(0.02)
    try:
        os.remove(broker.WHERE)
    except OSError:
        pass


def test_one_client(report):
    """A request crosses unchanged, and the reply comes back as bytes."""
    stop()
    fake = Fake()
    served(fake)
    try:
        client = broker.attach(ADDRESS)
        report.check('a client attaches', client is not None)
        if client is None:
            return

        got = client.request(1, 0x41, b'\x01\x02')
        report.check('the reply crosses intact', got == b'\x01\x41\x01\x02',
                     got.hex(' '))
        report.check('the request crossed unchanged',
                     fake.calls[-1] == (1, 0x41, b'\x01\x02'),
                     str(fake.calls[-1]))
        report.check('the baud comes from the served transport',
                     client.baud == 115200, str(client.baud))
        report.check('and the port it is serving', client.port == 'FAKE',
                     client.port)

        client.broadcast(0x41, b'\x09')
        report.check('a broadcast crosses and answers nothing',
                     fake.calls[-1] == ('broadcast', 0x41, b'\x09'),
                     str(fake.calls[-1]))
        client.close()
    finally:
        stop()


def test_two_sessions(report):
    """Two clients at once, and never inside the transport together."""
    stop()
    fake = Fake(delay=0.004)
    served(fake)
    try:
        clients = [broker.attach(ADDRESS), broker.attach(ADDRESS)]
        report.check('two clients attach at once', all(clients))
        if not all(clients):
            return

        bad = []

        def hammer(client, tag):
            try:
                for _ in range(25):
                    got = client.request(tag, 0x41, bytes([tag]))
                    if got != bytes([tag, 0x41, tag]):
                        bad.append('%d got %s' % (tag, got.hex()))
            except errors.RigError as exc:
                bad.append('%d: %s' % (tag, exc))

        threads = [threading.Thread(target=hammer, args=(c, n + 1))
                   for n, c in enumerate(clients)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        report.check('50 requests over two clients, none wrong',
                     not bad, '; '.join(bad[:2]))
        report.check('every reply matched the client that asked',
                     len(fake.calls) == 50, '%d calls' % len(fake.calls))
        # The lock is the whole design: one wire, one slave, and two masters
        # inside a transaction is a frame split between them.
        report.check('never two inside the transport at once',
                     not fake.overlapped)
        for client in clients:
            client.close()
    finally:
        stop()


def test_errors_cross_as_themselves(report):
    """A refusal arrives as the class it was raised as, with its sentence."""
    stop()
    fake = Fake()
    served(fake)
    try:
        client = broker.attach(ADDRESS)
        if client is None:
            report.check('a client attaches', False)
            return

        try:
            client.request(1, 0xEE)
            raised = None
        except errors.RigError as exc:
            raised = exc
        report.check('a Modbus exception crosses as one',
                     type(raised) is errors.ModbusException,
                     type(raised).__name__)
        # It formats its message in __init__, so rebuilding from `args` gets
        # `missing 2 required positional arguments` - measured, and it hid a
        # whole run's actual refusal behind a TypeError.
        report.check('with its unit, function and code',
                     getattr(raised, 'code', None) == 4
                     and getattr(raised, 'function', None) == 0xEE,
                     str(raised))

        try:
            client.request(1, 0xED)
            raised = None
        except errors.RigError as exc:
            raised = exc
        report.check('a device refusal crosses as one',
                     type(raised) is errors.DeviceStateError,
                     type(raised).__name__)
        report.check("and carries the board's own sentence",
                     'the board said no' in str(raised), str(raised))
        client.close()
    finally:
        stop()


def test_a_client_never_hands_the_line_back(report):
    """`is_open` is False, so a teardown skips the console handover.

    The port is not a client's to give: handing it to the text console
    would take it from every other client still attached.
    """
    stop()
    fake = Fake()
    served(fake)
    try:
        client = broker.attach(ADDRESS)
        if client is None:
            report.check('a client attaches', False)
            return
        report.check('is_open is False on a broker client',
                     client.is_open is False)
        client.close()
        report.check('closing a client leaves the served transport open',
                     not fake.closed)
    finally:
        stop()


def test_a_stale_address_is_not_a_broker(report):
    """The file outlives a killed process, and reads like a live one."""
    import json

    stop()
    report.check('nothing serving reads as nothing', broker.serving() is None)

    with open(broker.WHERE, 'w', encoding='utf-8') as handle:
        json.dump({'serial': 'COM99', 'pid': 1, 'kind': 'probe',
                   'host': '127.0.0.1', 'tcp': 8792}, handle)
    try:
        report.check('a stale file still names a broker',
                     (broker.serving() or {}).get('serial') == 'COM99')
        report.check('but attaching to it answers None',
                     broker.attach(('127.0.0.1', 8792), timeout=0.5) is None)

        # The hole this closes: `auto` saw the file, committed to a real
        # port, and raised instead of falling back. Checked at the seam
        # rather than through open_session - that one discovers whatever
        # board is actually plugged in, and would start a broker for it.
        from coaxial_mcp.session import _answers
        report.check('and the session layer says it does not answer',
                     _answers({'host': '127.0.0.1', 'tcp': 8792}) is False)
    finally:
        stop()

    # The other half: a broker that IS there answers, and asking does not
    # count as a use - the question must not be what takes it down.
    fake = Fake()
    served(fake)
    try:
        from coaxial_mcp.session import _answers as asks
        report.check('a live broker answers that question',
                     asks({'host': ADDRESS[0], 'tcp': ADDRESS[1]}) is True)
        report.check('and asking did not take it down',
                     broker.attach(ADDRESS, timeout=1.0) is not None)
    finally:
        stop()


def main():
    report = Report()
    for test in (test_one_client, test_two_sessions,
                 test_errors_cross_as_themselves,
                 test_a_client_never_hands_the_line_back,
                 test_a_stale_address_is_not_a_broker):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
