"""The board's cycle counter, tied to this machine's clock."""
import socket
import struct
import time

from . import protocol
from .protocol import TimeOp
from .errors import RigError
from .subsystem import Device
from .wire import Reader

#: CYCCNT is 32 bits and free-running.
WRAP = 1 << 32

#: pool.ntp.org times out on this bench; these two answer and agree.
NTP_SERVER = 'time.google.com'
NTP_EPOCH = 2208988800


def ntp_offset(server=NTP_SERVER, rounds=8, timeout=3.0):
    """How far this machine's clock is from UTC. Seconds, and the trip."""
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

    """What a `sync()` worked out: where the counter was, and how fast."""

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
        """Measured rate against the rate the PLL was asked for."""
        return (self.hz - self.nominal_hz) / self.nominal_hz * 1e6

    def to_host(self, cycles):
        """One unwrapped cycle count as a `time.time()` value."""
        return self.at_host + (cycles - self.at_cycles) / self.hz

    def __repr__(self):
        floor = ('' if self.floor_ppm is None
                 else ', floor %.1f ppm' % self.floor_ppm)
        return ('<Sync %.6f MHz (%+.1f ppm vs %s%s), reference +/- %.0f us%s>'
                % (self.hz / 1e6, self.error_ppm, self.reference, floor,
                   self.spread_us, '; ' + self.note if self.note else ''))


def unwrap(cycles, start=None):
    """A folded sequence of 32-bit counts made monotonic."""
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


class Clock(Device, device=protocol.DEVICE_TIME):

    """Tie the board's counter to this machine's, and keep the rate."""

    def latch(self, settle=0.05):
        """Broadcast a latch: the board takes CYCCNT, and nobody replies."""
        self._broadcast(TimeOp.LATCH, settle=settle)

    def read_latch(self):
        """What was latched, what the counter says now, and how fast it runs."""
        r = Reader(self._op(TimeOp.READ))
        return {'seq': r.u32(), 'latched': r.u32(), 'now': r.u32(),
                'sysclk_hz': r.u32()}

    def _bracket(self):
        """One latch, bracketed by this machine's clock."""
        before = time.perf_counter()
        self.latch(settle=0)
        after = time.perf_counter()
        time.sleep(0.02)
        return (before + after) / 2.0, after - before

    def probe(self, rounds=16):
        """Best-of-N round trips: the board's counter against this clock."""
        best = None
        for _ in range(rounds):
            t1 = time.perf_counter()
            got = self.read_latch()
            t4 = time.perf_counter()
            trip = t4 - t1
            if best is None or trip < best[2]:
                best = (got['now'], (t1 + t4) / 2.0, trip)
        if best is None:
            raise RigError('probe() needs at least one round')
        return {'cycles': best[0], 'host': best[1], 'round_trip': best[2]}
    def sync(self, seconds=2.0, rounds=8, reference='utc',
             ntp_server=NTP_SERVER):
        """Measure where the counter is and how fast it actually runs."""
        nominal = self.read_latch()['sysclk_hz']
        step = WRAP / nominal / 2.0                  # 4.52 s at 475 MHz

        first_offset, reference, note = _ntp_or_pc(
            ntp_server, reference, 'NTP did not answer')
        marks = _marks(self, seconds, rounds, step)
        cycles = unwrap([m[0] for m in marks])
        elapsed = marks[-1][1] - marks[0][1]
        hz = (cycles[-1] - cycles[0]) / elapsed
        # One tie to the wall clock, taken once.
        at_host = marks[-1][1] + (time.time() - time.perf_counter())
        floor = max(m[2] for m in (marks[0], marks[-1])) / elapsed * 1e6

        last_offset, reference, said = _ntp_or_pc(
            ntp_server, reference, 'NTP answered at the start and not at the end')
        note = said or note
        hz, at_host, floor, pc_ppm = _against_utc(
            hz, at_host, floor, first_offset, last_offset, elapsed)
        return Sync(cycles[-1], at_host, hz, max(marks[0][2], marks[-1][2])
                    * 1e6, nominal, reference, pc_ppm, floor, note)


# The sync's steps, as functions of the clock they bracket: the stand-in
# borrows `Clock.sync` with itself as the receiver, so nothing sync calls may
# be a method the stand-in would have to carry too.
def _best_bracket(clock, n):
    """The tightest of `n` brackets: (latched cycles, host time, width)."""
    best = None
    for _ in range(n):
        host, width = clock._bracket()
        got = clock.read_latch()
        if best is None or width < best[2]:
            best = (got['latched'], host, width)
    if best is None:
        raise RigError('sync() needs at least one round')
    return best

def _marks(clock, seconds, rounds, step):
    """The two ends bracketed `rounds` times, and one mark every half-wrap
    between them so the unwrapping stays unambiguous.
    """
    marks = [_best_bracket(clock, rounds)]
    while True:
        left = seconds - (marks[-1][1] - marks[0][1])
        if left <= 0:
            return marks
        time.sleep(min(step, left))
        marks.append(_best_bracket(clock, rounds if left <= step else 1))

def _ntp_or_pc(ntp_server, reference, when):
    """NTP's offset when the reference is UTC and the server answers; otherwise
    no offset, the reference fallen back to the PC's clock, and a note
    saying `when` it failed - or nothing to say.
    """
    if reference != 'utc':
        return None, reference, ''
    try:
        offset, _ = ntp_offset(ntp_server, rounds=4, timeout=1.0)
    except (RigError, OSError) as why:
        return None, 'pc', '%s (%s)' % (when, why)
    return offset, reference, ''

def _against_utc(hz, at_host, floor, first_offset, last_offset, elapsed):
    """The rate and the epoch taken against UTC when both ends answered: (hz,
    at_host, floor, pc_ppm).
    """
    if last_offset is None or first_offset is None:
        return hz, at_host, floor, None
    pc_ppm = (last_offset - first_offset) / elapsed * 1e6
    return (hz / (1.0 + pc_ppm * 1e-6), at_host + last_offset,
            1e-3 / elapsed * 1e6, pc_ppm)


