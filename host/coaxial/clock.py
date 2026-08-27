"""The board's cycle counter, tied to this machine's clock.

Every timestamp the board produces is raw CYCCNT - invariant 2's reason, and
it leaves a host holding ticks with no idea what o'clock they are. This maps
them.

Two things have to be got right and only one of them is obvious.

The rate is measured, not assumed. `sysclk_hz` says 475000000 because that
is what the PLL was asked for; the crystal answers to its own tolerance, and
the difference is what accumulates over a capture.

**CYCCNT wraps every 9.04 seconds at 475 MHz.** Any series longer than that
comes back with the timestamps folded, and unwrapping is not optional - a
capture that ran twelve seconds looks like two that ran nine and three, out
of order, unless someone puts them back.

The board keeps no wall clock and is not given one: no RTC, no LSE, so a
time it held would drift against nothing. The host owns the clock.
"""
import socket
import struct
import time

from . import protocol
from .errors import RigError
from .subsystem import Subsystem
from .wire import Reader

TIME_OP_LATCH = 0
TIME_OP_READ = 1

#: CYCCNT is 32 bits and free-running.
WRAP = 1 << 32

#: pool.ntp.org times out on this bench; these two answer and agree.
NTP_SERVER = 'time.google.com'
NTP_EPOCH = 2208988800


def ntp_offset(server=NTP_SERVER, rounds=8, timeout=3.0):
    """How far this machine's clock is from UTC. Seconds, and the trip.

    Positive means this machine is behind. Min-filtered on the round trip,
    the way every NTP client does it: the shortest exchange has the least
    queueing in it, and what is left is half the asymmetry rather than all
    of the delay. Repeatability on this bench is about a millisecond, which
    is what sets `floor_ppm`.

    Needed because this machine is not a reference either. Measured
    2026-08-27 with W32Time having synced six minutes earlier: 947 ms behind
    UTC and losing a further 25 ppm. Windows had declined to step it - the
    offset was just inside the 1 s `MaxAllowedPhaseOffset` - and slewing was
    not keeping up.
    """
    best = None
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(timeout)
    try:
        for _ in range(rounds):
            first = time.time()
            sock.sendto(b'\x1b' + 47 * b'\0', (server, 123))
            data, _ = sock.recvfrom(48)
            last = time.time()
            rx = (struct.unpack('!I', data[32:36])[0]
                  + struct.unpack('!I', data[36:40])[0] / 2 ** 32 - NTP_EPOCH)
            tx = (struct.unpack('!I', data[40:44])[0]
                  + struct.unpack('!I', data[44:48])[0] / 2 ** 32 - NTP_EPOCH)
            trip = (last - first) - (tx - rx)
            if best is None or trip < best[1]:
                best = (((rx - first) + (tx - last)) / 2.0, trip)
    finally:
        sock.close()
    if best is None:
        raise RigError('%s did not answer in %d tries' % (server, rounds))
    return best


class Sync:

    """What a `sync()` worked out: where the counter was, and how fast.

    `reference` is what it ended up tied to, not what was asked for: 'utc'
    when NTP answered, 'pc' when it did not or was not wanted. A capture
    that believes it is on UTC when it is on this machine's wall clock is
    worse than one that knows it is not, so a fallback is recorded in
    `note`, never silent.
    """

    def __init__(self, at_cycles, at_host, hz, spread_us, nominal_hz,
                 reference='pc', pc_ppm=None, floor_ppm=None, note=''):
        self.at_cycles = at_cycles
        self.at_host = at_host
        self.hz = hz
        self.spread_us = spread_us
        self.nominal_hz = nominal_hz
        self.reference = reference
        self.pc_ppm = pc_ppm
        self.floor_ppm = floor_ppm
        self.note = note

    @property
    def error_ppm(self):
        """Measured rate against the rate the PLL was asked for.

        Against `reference`. Believe it only past `floor_ppm`; under that
        the window was too short to tell it from the reference's own noise.
        """
        return (self.hz - self.nominal_hz) / self.nominal_hz * 1e6

    def to_host(self, cycles):
        """One unwrapped cycle count as a `time.time()` value.

        Unwrapped: pass this what `unwrap()` returned, not a raw 32-bit
        stamp, or anything more than 9 seconds from the reference lands in
        the wrong wrap.
        """
        return self.at_host + (cycles - self.at_cycles) / self.hz

    def __repr__(self):
        floor = ('' if self.floor_ppm is None
                 else ', floor %.1f ppm' % self.floor_ppm)
        return ('<Sync %.6f MHz (%+.1f ppm vs %s%s), reference +/- %.0f us%s>'
                % (self.hz / 1e6, self.error_ppm, self.reference, floor,
                   self.spread_us, '; ' + self.note if self.note else ''))


def unwrap(cycles, start=None):
    """A folded sequence of 32-bit counts made monotonic.

    Assumes the samples are in order and no two are more than a wrap apart,
    which for a capture means draining faster than every 9 seconds. Nothing
    here can tell a 10-second gap from a 1-second one, and it does not
    pretend to: it takes the shorter reading, which is the right guess when
    the producer is a ring being drained.
    """
    out = []
    base = 0
    previous = None
    for raw in cycles:
        if previous is not None and raw < previous:
            base += WRAP
        out.append(raw + base)
        previous = raw
    if start is not None and out:
        out = [v - out[0] + start for v in out]
    return out


class Clock(Subsystem):

    """Tie the board's counter to this machine's, and keep the rate."""

    def _op(self, op, payload=b''):
        return self.request(protocol.DEVICE,
                            bytes([protocol.DEVICE_TIME, op]) + bytes(payload))

    def latch(self, settle=0.05):
        """Broadcast a latch: the board takes CYCCNT, and nobody replies.

        Broadcast on purpose. A unicast would put a reply's turnaround
        inside the measurement, and the turnaround is the part that varies.
        """
        self.board.broadcast(protocol.DEVICE,
                             bytes([protocol.DEVICE_TIME, TIME_OP_LATCH]),
                             settle=settle)

    def read_latch(self):
        """What was latched, what the counter says now, and how fast it runs."""
        r = Reader(self._op(TIME_OP_READ))
        return {'seq': r.u32(), 'latched': r.u32(), 'now': r.u32(),
                'sysclk_hz': r.u32()}

    def _bracket(self):
        """One latch, bracketed by this machine's clock.

        `perf_counter` and not `time`: monotonic, and it does not step under
        a capture when something adjusts the wall clock. The settle is left
        out of the bracket and taken afterwards - 50 ms of politeness inside
        the measurement is 50 ms of uncertainty, which is what the first
        version of this measured.
        """
        before = time.perf_counter()
        self.latch(settle=0)
        after = time.perf_counter()
        time.sleep(0.02)
        return (before + after) / 2.0, after - before

    def probe(self, rounds=16):
        """Best-of-N round trips: the board's counter against this clock.

        The other half of the measurement, and the half that can be checked.
        A broadcast latch has no reply, so nothing in it says how long the
        frame took to arrive; a round trip does - `now` in the reply was
        taken between the two host stamps, so the board's clock at the
        midpoint is bracketed by the round trip itself.

        Min-filtered on the round trip: the shortest exchange has the least
        queueing in it.

        **And it loses, by a lot.** Measured against the broadcast bracket
        on the debug probe's VCP: round trip 35 883 us best of twenty, so
        17 941 us one way if it is symmetric, against 5 243 us for the
        bracket. A 16-byte reply is 1.7 ms of line time; the rest is the
        VCP driver's latency timer, and a broadcast never waits for it
        because there is no reply to wait for.

        Kept because it is the measurement that says so, and because a
        segment with a different driver - or a 10 Mbit RS485 one - may well
        answer differently. Compare the two before assuming.
        """
        best = None
        for _ in range(rounds):
            t1 = time.perf_counter()
            got = self.read_latch()
            t4 = time.perf_counter()
            trip = t4 - t1
            if best is None or trip < best[2]:
                best = (got['now'], (t1 + t4) / 2.0, trip)
        return {'cycles': best[0], 'host': best[1], 'round_trip': best[2]}

    def sync(self, seconds=2.0, rounds=8, reference='utc',
             ntp_server=NTP_SERVER):
        """Measure where the counter is and how fast it actually runs.

        seconds    how far apart to put the two ends. The link's latency is
                   a fixed unknown of under a millisecond, so a longer
                   window divides it down: 3 s bounds the rate at parts per
                   thousand, 1000 s resolves about one per million.
        reference  'utc' measures this machine against NTP across the same
                   window and takes both its offset and its rate back out,
                   so the answer is the board against UTC rather than the
                   difference between two unqualified oscillators. 'pc'
                   ties it to this machine's wall clock, whatever that is.
                   When NTP does not answer, 'utc' becomes 'pc' and the
                   Sync says so - it does not fail and it does not pretend.

        Sampled through rather than taken end to end: CYCCNT is 32 bits and
        wraps every 9.04 s at 475 MHz, so a longer window has to have its
        wraps counted. The samples in between exist only to keep the
        unwrapping unambiguous; the rate comes from the two ends, which are
        the brackets worth spending `rounds` on.
        """
        def best_of(n):
            best = None
            for _ in range(n):
                host, width = self._bracket()
                got = self.read_latch()
                if best is None or width < best[2]:
                    best = (got['latched'], host, width)
            return best

        nominal = self.read_latch()['sysclk_hz']
        step = WRAP / nominal / 2.0                  # 4.52 s at 475 MHz
        note = ''
        first_offset = None

        if reference == 'utc':
            try:
                # Short and few: an unreachable server must not cost the
                # caller half a minute of timeouts to find that out.
                first_offset, _ = ntp_offset(ntp_server, rounds=4, timeout=1.0)
            except (RigError, OSError) as why:
                reference, note = 'pc', 'NTP did not answer (%s)' % why

        marks = [best_of(rounds)]
        while True:
            left = seconds - (marks[-1][1] - marks[0][1])
            if left <= 0:
                break
            time.sleep(min(step, left))
            marks.append(best_of(rounds if left <= step else 1))

        cycles = unwrap([m[0] for m in marks])
        elapsed = marks[-1][1] - marks[0][1]
        hz = (cycles[-1] - cycles[0]) / elapsed
        # One tie to the wall clock, taken once. Everything above is
        # perf_counter, which has no epoch of its own.
        at_host = marks[-1][1] + (time.time() - time.perf_counter())
        pc_ppm = None
        floor = max(m[2] for m in (marks[0], marks[-1])) / elapsed * 1e6

        if reference == 'utc':
            last_offset, _ = ntp_offset(ntp_server, rounds=4, timeout=1.0)
            # Positive pc_ppm is this machine falling behind UTC, which
            # means it under-counts: a real second arrives as slightly less
            # than one of its own. Dividing cycles by that short elapsed
            # makes the board look fast by exactly as much, so this comes
            # off. Signed wrong first, and it showed - the board came back
            # +35 ppm where an independent heartbeat measurement had -13.
            pc_ppm = (last_offset - first_offset) / elapsed * 1e6
            hz /= (1.0 + pc_ppm * 1e-6)
            at_host += last_offset
            # NTP repeatability on this bench is about a millisecond at
            # each end, and that, not the bracket, is what bounds the rate.
            floor = 1e-3 / elapsed * 1e6

        return Sync(cycles[-1], at_host, hz, max(marks[0][2], marks[-1][2])
                    * 1e6, nominal, reference, pc_ppm, floor, note)
