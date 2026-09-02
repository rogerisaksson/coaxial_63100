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
import threading
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
        # ONE TRANSACTION AT A TIME ON THE WIRE. A request is a transmit
        # and then a receive, and two threads interleaving those halves
        # put one thread's reply in the other's hands - or, on RTU,
        # scatter a frame's characters past t1.5 and lose both. Held for
        # the whole exchange and released between them, so a reader
        # thread draining the ring still lets a state() through.
        self._wire = threading.RLock()
        #: When the line last went quiet, so t3.5 is only slept for what is
        #: actually owed.
        self._quiet_since = time.monotonic()
        #: Whether the last exchange ended with a validated reply. False
        #: makes the next transmit purge whatever is left over.
        self._clean = False

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
                                                    # other field in the frame
        # T3.5 IS SILENCE ON THE BUS, NOT A SLEEP TO PERFORM. Decoding the
        # last reply, deciding what to ask next and crossing the broker all
        # happen in that silence, and on a busy reader they already exceed
        # 1.75 ms - sleeping it again is 1.75 ms of every transaction spent
        # proving something that was already true.
        owed = self.interframe_gap - (time.monotonic() - self._quiet_since)
        if owed > 0:
            time.sleep(owed)
        with self._link_errors('transmitting'):
            # ONLY WHEN THE LAST EXCHANGE DID NOT END CLEANLY. A validated
            # reply leaves the buffer empty by construction; purging it
            # anyway is a driver round trip on the critical path. After a
            # timeout or a bad CRC there may well be a stale tail, and that
            # is exactly when this still runs.
            if not self._clean:
                self.serial.reset_input_buffer()
            self._clean = False
            self.serial.write(frame)
            self.serial.flush()

    def receive(self, exact_payload=None, timeout=None, reply_shape=None):
        budget = self.DEFAULT_TIMEOUT if timeout is None else timeout
        with self._link_errors('reading a reply'):
            if exact_payload is not None:
                return self._read_exactly(4 + exact_payload, budget)
            return self._read_until_quiet(budget, reply_shape)

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

    def _read_until_quiet(self, budget, reply_shape=None):
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

        want = self.MAX_FRAME
        while len(buffer) < want:
            if want == self.MAX_FRAME:
                # The length is knowable for some replies as soon as the
                # counted field has arrived, and once it is known the read
                # stops on the last byte instead of on QUIET_TIME of
                # silence after it. That wait is 8 ms of every transaction.
                sized = frame_length(reply_shape, buffer)
                if sized:
                    want = min(sized, self.MAX_FRAME)
                    continue
            waiting = self.serial.in_waiting
            chunk = self.serial.read(min(waiting, want - len(buffer))
                                     if waiting else 1)
            if not chunk:
                break
            buffer += chunk
        return buffer

    # -- one transaction ---------------------------------------------------

    def request(self, unit, function, payload=b'', exact_payload=None,
                timeout=None, reply_shape=None):
        """Send a request and return the reply payload, or raise."""
        with self._wire:
            self.transmit(unit, function, payload)
            reply = self.receive(exact_payload, timeout, reply_shape)
            self._quiet_since = time.monotonic()
        payload = validate(reply, unit, function)
        self._clean = True          # only past validate: a raise is not clean
        return payload

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


#: The shape of every `u8 took` reply: one byte on success, the
#: length-prefixed refusal behind it otherwise. Passed as `reply_shape` so
#: the read stops on the last byte instead of waiting out QUIET_TIME -
#: 8 ms on the write class of transaction, which was most of its 15 ms.
ACK = {'ack': True}


def frame_length(shape, buffer):
    """Whole frame length from `shape` and what has arrived, or 0.

    `shape` is {'at': index of a count byte in the PAYLOAD, 'head': bytes
    before the records, 'stride': one record, 'tail': bytes after them} -
    a dict rather than a callable so it crosses the broker as JSON. 0
    means not knowable yet, and the caller keeps reading until quiet.

    {'ack': True} is the `u8 took` reply: `1` alone, or `0` and the
    board's length-prefixed refusal - knowable either way, unlike the
    general reply (stopping on a valid CRC was measured and rejected: a
    prefix passes about once in 4096, the QUIET_TIME docstring above).

    An exception frame (fc | 0x80) is sized for ANY shape: it is always
    exactly one code byte, and a shaped read of a refused request
    otherwise waited out the quiet time to learn it was refused.
    """
    if not shape or len(buffer) < 2:
        return 0
    if buffer[1] & 0x80:
        return 5                              # unit, fc | 0x80, code, CRC
    if shape.get('ack'):
        if len(buffer) < 3:
            return 0
        if buffer[2]:
            return 5                          # unit, fc, took=1, CRC
        if len(buffer) < 4:
            return 0
        return 2 + 2 + buffer[3] + 2          # took=0, length, string, CRC
    at = 2 + int(shape.get('at', 0))          # past unit and function code
    if len(buffer) <= at:
        return 0
    payload = (int(shape['head']) + buffer[at] * int(shape['stride'])
               + int(shape.get('tail', 0)))
    return 2 + payload + 2                    # unit, fc, payload, CRC


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
