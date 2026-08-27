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
import time

from . import protocol
from .subsystem import Subsystem
from .wire import Reader

TIME_OP_LATCH = 0
TIME_OP_READ = 1

#: CYCCNT is 32 bits and free-running.
WRAP = 1 << 32


class Sync:

    """What a `sync()` worked out: where the counter was, and how fast."""

    def __init__(self, at_cycles, at_host, hz, spread_us, nominal_hz):
        self.at_cycles = at_cycles
        self.at_host = at_host
        self.hz = hz
        self.spread_us = spread_us
        self.nominal_hz = nominal_hz

    @property
    def error_ppm(self):
        """Measured rate against the rate the PLL was asked for."""
        return (self.hz - self.nominal_hz) / self.nominal_hz * 1e6

    def to_host(self, cycles):
        """One unwrapped cycle count as a host `time.time()` value.

        Unwrapped: pass this what `unwrap()` returned, not a raw 32-bit
        stamp, or anything more than 9 seconds from the reference lands in
        the wrong wrap.
        """
        return self.at_host + (cycles - self.at_cycles) / self.hz

    def __repr__(self):
        return ('<Sync %.6f MHz (%+.1f ppm), reference +/- %.0f us>'
                % (self.hz / 1e6, self.error_ppm, self.spread_us))


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

    def sync(self, seconds=2.0, rounds=8):
        """Measure where the counter is and how fast it actually runs.

        Two brackets, `seconds` apart, each the best of `rounds` tries. The
        gap is what measures the rate: the link's latency is a fixed unknown
        of well under a millisecond, so spreading the two ends over seconds
        divides it down to nothing. The scatter within a bracket is reported
        as `spread_us` and is what bounds the reference.
        """
        def best_of(n):
            best = None
            for _ in range(n):
                host, width = self._bracket()
                got = self.read_latch()
                if best is None or width < best[2]:
                    best = (got['latched'], host, width)
            return best

        first_cycles, first_host, first_width = best_of(rounds)
        nominal = self.read_latch()['sysclk_hz']
        time.sleep(seconds)
        last_cycles, last_host, last_width = best_of(rounds)
        # One tie to the wall clock, taken once. Everything above is
        # perf_counter, which has no epoch of its own.
        wall_offset = time.time() - time.perf_counter()

        elapsed_host = last_host - first_host
        elapsed_cycles = (last_cycles - first_cycles) % WRAP
        # A gap longer than a wrap cannot be told from a short one, so the
        # caller is told rather than guessed at.
        if elapsed_host * nominal > WRAP:
            raise ValueError('%.1f s is longer than the counter\'s %.2f s '
                             'wrap; sync over a shorter gap'
                             % (elapsed_host, WRAP / nominal))

        hz = elapsed_cycles / elapsed_host
        return Sync(last_cycles, last_host + wall_offset, hz,
                    max(first_width, last_width) * 1e6, nominal)
