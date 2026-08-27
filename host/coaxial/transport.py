"""Modbus RTU over a serial port: framing, addressing, checksum.

This is the only module that touches pyserial, and the only one that knows a
frame has a CRC. Everything above it works in payloads.

RTU has no length field and no delimiter. A frame ends when the line falls
quiet, which is why a reply is read until a gap rather than to a byte count.
The exception is a reply of known length, which matters after the CONSOLE
command: the board starts printing ASCII the moment it hands the UART back, and
a quiet-time reader would swallow that text into the frame.
"""
import struct
import time
from contextlib import contextmanager

import serial

from .crc import crc16
from .errors import ConnectError, CrcError, FrameError, ModbusException, NoReplyError
from .protocol import BROADCAST, MAX_PAYLOAD


class Transport:
    """One serial port at one bitrate, shared by every unit on it.

    Slaves that share a port and a bitrate share a Transport. A different
    bitrate gets its own, because one UART cannot run two bitrates at once.
    """

    @property
    def interframe_gap(self):
        """Silence before transmitting, from the bitrate rather than a guess.

        Modbus RTU: 3.5 character times, which the specification fixes at
        1.75 ms above 19200 baud. It was a flat 5 ms - three times the
        specification at 115200, and paid before every request.
        """
        return 0.00175 if self.baud > 19200 else 3.5 * 11.0 / self.baud

    QUIET_TIME = 0.008
    """Gap that ends an inbound frame.

    Measured on the debug probe's VCP at 115200, reading greedily with
    `in_waiting`: the largest gap inside a frame was **3.40 ms**, across
    both a 20-byte reply arriving whole in one chunk and a 215-byte one
    arriving in 175. This is twice that. It was 20 ms, six times the margin
    the link needs, and since it is paid at the end of every single
    transaction it was most of the round trip - 46.6 ms became 12.9 ms with
    this and the gap below.

    It cannot go much lower without knowing the reply's length, and the
    board does not send one: nothing in the frame says where it ends, so a
    reader that stops early would hand back a truncated payload that still
    decoded. Stopping on a valid CRC was measured against and rejected - a
    prefix of a 20-byte frame passes the check about once in 4096, which is
    a wrong reading every few minutes rather than an error.
    """

    DEFAULT_TIMEOUT = 0.5

    MAX_FRAME = 4 + MAX_PAYLOAD
    """Unit id, function code, the largest payload and the CRC. Nothing longer
    can be a frame, so a reader holding this many bytes need not wait for a gap
    to know the frame ended."""

    def __init__(self, port, baud):
        self.port = port
        self.baud = baud
        try:
            self.serial = serial.Serial(port, baud, bytesize=8, parity='N',
                                        stopbits=1, timeout=self.QUIET_TIME)
        except (serial.SerialException, ValueError, OSError) as exc:
            raise ConnectError('cannot open %s at %d baud: %s'
                               % (port, baud, exc)) from exc

    def __repr__(self):
        return '<Transport %s@%d>' % (self.port, self.baud)

    # -- pyserial failures, translated ------------------------------------

    @contextmanager
    def _link_errors(self, doing):
        """Turn a pyserial failure into this library's own exception.

        A port that vanishes mid-session - a pulled adapter, a suspended hub -
        fails in these calls rather than at open time. Letting SerialException
        out would force every caller above here to import pyserial to catch it,
        against the one rule the whole library keeps: what leaves this package
        is a coaxial.errors exception.
        """
        try:
            yield
        except (serial.SerialException, OSError) as exc:
            raise ConnectError('%s@%d failed while %s: %s'
                               % (self.port, self.baud, doing, exc)) from exc

    @property
    def is_open(self):
        """Whether the port is still ours. Lets a teardown skip a closed one."""
        return self.serial.is_open

    def close(self):
        with self._link_errors('closing the port'):
            self.serial.close()

    def discard_input(self):
        """Throw away anything already received, before framing starts."""
        with self._link_errors('clearing the input buffer'):
            self.serial.reset_input_buffer()

    # -- the ASCII side of the same wire ----------------------------------

    def write_text(self, text):
        """Send characters to the board's text console.

        The console and the binary protocol share USART3, so this is how the
        link is opened by hand. It is not part of Modbus and does not frame.
        """
        with self._link_errors('writing to the console'):
            self.serial.reset_input_buffer()
            self.serial.write(text.encode())
            self.serial.flush()

    def read_text(self, seconds=1.0):
        """Collect whatever the console prints. For banners and diagnostics."""
        deadline = time.time() + seconds
        chunks = []
        with self._link_errors('reading the console'):
            while time.time() < deadline:
                data = self.serial.read(256)
                if data:
                    chunks.append(data)
        return b''.join(chunks).decode('ascii', 'replace')

    # -- framing -----------------------------------------------------------

    def transmit(self, unit, function, payload=b''):
        frame = bytes([unit, function]) + payload
        frame += struct.pack('<H', crc16(frame))    # low byte first, unlike every
        time.sleep(self.interframe_gap)             # other field in the frame
        with self._link_errors('transmitting'):
            self.serial.reset_input_buffer()
            self.serial.write(frame)
            self.serial.flush()

    def receive(self, exact_payload=None, timeout=None):
        budget = self.DEFAULT_TIMEOUT if timeout is None else timeout
        with self._link_errors('reading a reply'):
            if exact_payload is not None:
                return self._read_exactly(4 + exact_payload, budget)
            return self._read_until_quiet(budget)

    def _first_byte(self, budget):
        """Wait up to `budget` for a reply to start, in QUIET_TIME slices.

        The port's own timeout is never moved. Measured on this VCP,
        assigning `serial.timeout` costs 3.25 ms whatever it is assigned -
        pyserial reconfigures the port, which is a control transfer - and
        the old code paid it three times a transaction. That was 9.75 ms of
        a 46.6 ms round trip, and none of it was the link.
        """
        deadline = time.monotonic() + budget
        while True:
            byte = self.serial.read(1)
            if byte or time.monotonic() >= deadline:
                return byte

    def _read_exactly(self, want, budget):
        """`want` bytes, or whatever arrived before the budget ran out."""
        buffer = self._first_byte(budget)
        if not buffer:
            return buffer
        deadline = time.monotonic() + budget
        while len(buffer) < want and time.monotonic() < deadline:
            chunk = self.serial.read(want - len(buffer))
            if not chunk:
                break
            buffer += chunk
        return buffer

    def _read_until_quiet(self, budget):
        """Wait the budget for the first byte, then read until a gap.

        The two waits differ on purpose. A reply may legitimately be seconds
        late, because a burst blocks the slave for as long as it samples; but
        once the first byte has arrived the rest follow at line rate, so the
        frame ends after QUIET_TIME of silence.

        Whatever is already buffered is taken in one read. A byte at a time
        was measured at 17.8 ms for a 20-byte reply that arrives whole in
        3.3 ms: the cost was one driver round trip per byte, not the link.
        """
        buffer = self._first_byte(budget)
        if not buffer:
            return buffer

        while len(buffer) < self.MAX_FRAME:
            waiting = self.serial.in_waiting
            chunk = self.serial.read(min(waiting, self.MAX_FRAME - len(buffer))
                                     if waiting else 1)
            if not chunk:
                break
            buffer += chunk
        return buffer

    # -- one transaction ---------------------------------------------------

    def request(self, unit, function, payload=b'', exact_payload=None, timeout=None):
        """Send a request and return the reply payload, or raise."""
        self.transmit(unit, function, payload)
        reply = self.receive(exact_payload, timeout)
        return validate(reply, unit, function)

    def broadcast(self, function, payload=b'', settle=0.05):
        """Acted on by every slave, answered by none. Nothing to return.

        The settle is not politeness: there is no reply to synchronise on, so
        without it the next request can arrive before the slaves have acted.
        A caller timing the write itself passes 0 and sleeps afterwards -
        50 ms inside the measurement is 50 ms of uncertainty.
        """
        self.transmit(BROADCAST, function, payload)
        if settle:
            time.sleep(settle)


def validate(reply, unit, function):
    """Check a reply frame and return its payload.

    Raises rather than returning a status, so a caller never has to ask whether
    the bytes it is holding are real.
    """
    if len(reply) < 4:
        raise NoReplyError('unit %d, fc 0x%02X: %s'
                           % (unit, function,
                              reply.hex(' ') if reply else 'silence'))

    if crc16(reply[:-2]) != struct.unpack('<H', reply[-2:])[0]:
        raise CrcError('unit %d: checksum failed on %s' % (unit, reply.hex(' ')))

    if reply[0] != unit:
        raise FrameError('reply came from unit %d, asked unit %d'
                         % (reply[0], unit))

    if reply[1] == (function | 0x80):
        raise ModbusException(unit, function, reply[2])

    if reply[1] != function:
        raise FrameError('reply is fc 0x%02X, asked 0x%02X' % (reply[1], function))

    return reply[2:-2]
