"""How much of the wire a transaction actually uses.

A bitrate is a ceiling nobody reaches. 8N1 puts ten bits on the wire per
byte, so 115200 baud carries 11 520 bytes a second and not one more; what a
round trip costs on top of that is turnaround, driver latency and the gap
that ends a frame, and none of it scales with the payload. This measures the
gap between the two.

    frame_seconds = bytes_on_the_wire * 10 / baud

`efficiency` is that over the measured round trip. A ping is nearly all
overhead by construction and a full block is where the link starts paying
for itself, so the interesting number is not either one alone - it is how
fast the curve climbs between them.

Nothing here judges a link. It reports what it measured against what the
bitrate allows (invariant 10).
"""
import time

from . import protocol

#: 8N1: one start bit, eight data, one stop.
BITS_PER_BYTE = 10

#: Unit id, function code, CRC. Every frame carries these.
FRAME_OVERHEAD = 4


def frame_seconds(byte_count, baud):
    """Line time for `byte_count` bytes at `baud`. The floor, not a guess."""
    return byte_count * BITS_PER_BYTE / float(baud)


class Result:

    """One case: what went on the wire, what it cost, what it could cost."""

    def __init__(self, name, out_bytes, in_bytes, times, baud):
        self.name = name
        self.out_bytes = out_bytes
        self.in_bytes = in_bytes
        self.times = sorted(times)
        self.baud = baud

    @property
    def wire_bytes(self):
        return self.out_bytes + self.in_bytes

    @property
    def best(self):
        return self.times[0]

    @property
    def median(self):
        return self.times[len(self.times) // 2]

    @property
    def theoretical(self):
        """What the bitrate alone would take for the same bytes."""
        return frame_seconds(self.wire_bytes, self.baud)

    @property
    def efficiency(self):
        """Fraction of the round trip that was the wire doing its job."""
        return self.theoretical / self.median

    @property
    def overhead(self):
        """What was not the wire. Flat per transaction, which is the point."""
        return self.median - self.theoretical

    @property
    def payload_bytes_per_second(self):
        """Useful bytes a second, sustained at this size."""
        return (self.out_bytes + self.in_bytes - 2 * FRAME_OVERHEAD) / self.median


def _time(call, rounds):
    times = []
    for _ in range(rounds):
        at = time.perf_counter()
        call()
        times.append(time.perf_counter() - at)
    return times


def ping(board, rounds=40):
    """The smallest transaction there is: an echo carrying nothing.

    Four bytes out, four back, and every millisecond past 0.7 of them at
    115200 is something other than the wire.
    """
    baud = board.baud
    return Result('ping', FRAME_OVERHEAD, FRAME_OVERHEAD,
                  _time(lambda: board.link.echo(b''), rounds), baud)


def echo_block(board, size, rounds=20):
    """`size` bytes out and the same back, which loads both directions."""
    payload = bytes(range(256)) * (size // 256 + 1)
    payload = payload[:size]
    baud = board.baud
    return Result('echo %d B' % size,
                  FRAME_OVERHEAD + size, FRAME_OVERHEAD + size,
                  _time(lambda: board.link.echo(payload), rounds), baud)


def receive_block(board, rounds=20):
    """A small ask for the largest reply the board sends: the ring's burst.

    The download direction on its own. Nothing symmetrical can separate the
    two, and this is the shape a capture actually runs in.
    """
    from .capture import MAX_BURST, RECORD_BYTES

    board.capture.arm(['angle'])
    time.sleep(0.3)
    try:
        held = board.capture.state()['count']
        want = max(1, min(MAX_BURST, held))
        request = FRAME_OVERHEAD + 3
        reply = FRAME_OVERHEAD + 1 + want * RECORD_BYTES
        times = _time(lambda: board.capture.take(want), rounds)
    finally:
        board.capture.arm([])
    return Result('receive %d records' % want, request, reply,
                  times, board.baud)


def run(board, sizes=(0, 16, 64, protocol.MAX_PAYLOAD), rounds=20):
    """Every case, smallest first, so the curve is readable in one glance."""
    results = [ping(board, rounds * 2)]
    results += [echo_block(board, size, rounds) for size in sizes if size]
    results.append(receive_block(board, rounds))
    return results


def table(results):
    """The report, as lines. Bytes, what they must cost, what they did."""
    lines = ['%-20s %5s %7s %8s %8s %7s %9s'
             % ('case', 'wire', 'floor', 'best', 'median', 'of max', 'payload'),
             '-' * 70]
    for r in results:
        lines.append('%-20s %5d %6.2fms %7.2fms %7.2fms %6.1f%% %7.0f B/s'
                     % (r.name, r.wire_bytes, r.theoretical * 1e3,
                        r.best * 1e3, r.median * 1e3, r.efficiency * 100,
                        r.payload_bytes_per_second))
    if results:
        baud = results[0].baud
        lines.append('')
        if any(r.efficiency > 1.0 for r in results):
            # A number from nowhere and one from hardware must not look
            # alike. Nothing can beat its own bitrate, so this is the only
            # thing it can mean.
            lines.append('faster than %d baud allows, which is not a link '
                         'being quick - it is the stand-in, with no wire to '
                         'be slow' % baud)
        else:
            lines.append('%d baud 8N1 carries %d B/s; the flat cost of a '
                         'transaction is %.1f ms'
                         % (baud, baud // BITS_PER_BYTE,
                            min(r.overhead for r in results) * 1e3))
    return lines
