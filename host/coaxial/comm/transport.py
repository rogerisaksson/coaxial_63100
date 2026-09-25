"""Modbus RTU over a serial port: framing, addressing, checksum."""
import importlib
import struct
import threading
import time
from contextlib import contextmanager
from typing import Any

import serial

from coaxial.comm.crc import crc16
from coaxial.comm.protocol import BROADCAST, MAX_PAYLOAD, request_length
from coaxial.errors import ConnectError, CrcError, FrameError, ModbusException, NoReplyError

#: A frame's fixed bytes around the payload: the unit id and function
#: code in front, the CRC behind.
HEAD_BYTES = 2
CRC_BYTES = 2
#: The bit a slave sets on the function code to answer an exception.
EXCEPTION = 0x80

#: URL schemes this checkout's tools serve, by the package pyserial finds their handler in:
#: the firmware's comms/ built for this host, and the image on an emulated MCU.
URL_PACKAGES = {'fakeboard': 'tools.cores', 'emulator': 'tools.emu'}


def url_buses(port):
    """The segments a URL's handler names - its module's `buses(url)` - or None."""
    scheme = str(port).split('://')[0] if '://' in str(port) else None
    if scheme not in URL_PACKAGES:
        return None
    named = getattr(importlib.import_module('%s.protocol_%s' % (URL_PACKAGES[scheme], scheme)),
                    'buses', None)
    return named(port) if named else None


def hand_to_binary(transport, settle=0.5):
    """Hand USART3 from the text console to the binary protocol."""
    transport.write_text('m')
    transport.sleep(settle)
    transport.discard_input()


class Transport:
    """One serial port at one bitrate, shared by every unit on it."""

    #: Only a broker's transport streams records from the ring it holds
    #: (`BrokerTransport.stream`), and only that one has an address to
    #: forward to; a UART has neither, and `connect` opens the binary
    #: link itself on one.
    address = None
    stream = None

    @property
    def interframe_gap(self):
        """Silence before transmitting, from the bitrate rather than a guess.
        """
        return 0.00175 if self.baud > 19200 else 3.5 * 11.0 / self.baud

    QUIET_TIME = 0.008
    """Gap that ends an inbound frame, paid at the end of every transaction.

    Measured on the debug probe's VCP at 115200, reading greedily with
    `in_waiting`: the largest gap inside a frame was 3.40 ms, across
    both a 20-byte reply arriving whole in one chunk and a 215-byte one
    arriving in 175. This is twice that.

    It cannot go much lower without knowing the reply's length, and the
    board does not send one: nothing in the frame says where it ends, so a
    reader that stops early would hand back a truncated payload that still
    decoded. Stopping on a valid CRC was measured against and rejected - a
    prefix of a 20-byte frame passes the check about once in 4096, which is
    a wrong reading every few minutes rather than an error.
    """

    #: The firmware dispatches proven requests on their own CRC (MINOR 9)
    #: - Board.probe() sets this when the version says so. Off, every
    #: transaction pays the spec gap.
    proven_dispatch = False

    DEFAULT_TIMEOUT = 0.5

    _time_scale = 1.0

    MAX_FRAME = HEAD_BYTES + MAX_PAYLOAD + CRC_BYTES
    """Unit id, function code, the largest payload and the CRC. Nothing longer
    can be a frame, so a reader holding this many bytes need not wait for a gap
    to know the frame ended."""

    def __init__(self, port, baud):
        self.port = port
        self.baud = baud
        scheme = str(port).split('://')[0] if '://' in str(port) else None
        if scheme in URL_PACKAGES and URL_PACKAGES[scheme] not in serial.protocol_handler_packages:
            serial.protocol_handler_packages.append(URL_PACKAGES[scheme])
        try:
            # A URL as well as a port name: `loop://`, `socket://`, or one of URL_PACKAGES.
            self.serial: Any = serial.serial_for_url(port, baud, bytesize=8, parity='N',
                                                     stopbits=1, timeout=self.QUIET_TIME)
        except (serial.SerialException, ValueError, OSError) as exc:
            raise ConnectError('cannot open %s at %d baud: %s'
                               % (port, baud, exc)) from exc
        self.time_scale = getattr(self.serial, 'time_scale', 1.0)
        #: What says the time scale now, asked each transaction: an emulator's load.
        self.time_scale_source = getattr(self.serial, 'time_scale_source', None)
        # One transaction at a time on the wire.
        self._wire = threading.RLock()
        #: When the line last went quiet, so t3.5 is only slept for what is
        #: owed.
        self._quiet_since = time.monotonic()
        #: Whether the last exchange ended with a validated reply. False
        #: makes the next transmit purge whatever is left over.
        self._clean = False
        #: Whether the previous request was one the length oracle proves -
        #: with `proven_dispatch`, the next transmit owes no gap for it.
        self._last_proven = False

    def __repr__(self):
        return '<Transport %s@%d>' % (self.port, self.baud)

    @property
    def time_scale(self):
        """Wall seconds a second of the board's: 1 on a real one, what an emulator measures of
        its own (tools/emu). Every wait on the wire is stretched by it."""
        return self._time_scale

    @time_scale.setter
    def time_scale(self, scale):
        self._time_scale = scale
        self.serial.timeout = self.QUIET_TIME * scale

    def sleep(self, seconds):
        """`seconds` of the board's."""
        time.sleep(seconds * self._time_scale)

    def _rescale(self):
        """The time scale as its source has it now, where one does."""
        if self.time_scale_source is not None:
            scale = self.time_scale_source()
            if abs(scale - self._time_scale) > 0.1 * self._time_scale:
                self.time_scale = scale

    # -- pyserial failures, translated ------------------------------------

    @contextmanager
    def _link_errors(self, doing):
        """Turn a pyserial failure into this library's own exception."""
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
        """Send characters to the board's text console."""
        with self._link_errors('writing to the console'):
            self.serial.reset_input_buffer()
            self.serial.write(text.encode())
            self.serial.flush()

    def read_text(self, seconds=1.0):
        """Collect whatever the console prints. For banners and diagnostics."""
        deadline = time.time() + seconds * self._time_scale
        chunks = []
        with self._link_errors('reading the console'):
            while time.time() < deadline:
                data = self.serial.read(256)
                if data:
                    chunks.append(data)
        return b''.join(chunks).decode('ascii', 'replace')

    # -- framing -----------------------------------------------------------

    def _pay_gap(self):
        """What is left of t3.5 since the line went quiet, slept."""
        owed = self.interframe_gap * self._time_scale - (time.monotonic() - self._quiet_since)
        if owed > 0:
            time.sleep(owed)

    def transmit(self, unit, function, payload=b''):
        frame = bytes([unit, function]) + payload
        frame += struct.pack('<H', crc16(frame))    # low byte first, unlike every
                                                    # other field in the frame
        # t3.5 is silence on the bus, not a sleep to perform.
        if not (self.proven_dispatch and self._last_proven):
            self._pay_gap()
        pdu_len = len(frame) - 3
        self._last_proven = (request_length(frame[1:-2]) == pdu_len
                             and pdu_len > 0)
        with self._link_errors('transmitting'):
            # Only when the last exchange did not end cleanly.
            if not self._clean:
                self.serial.reset_input_buffer()
            self._clean = False
            self.serial.write(frame)
            self.serial.flush()

    def receive(self, exact_payload=None, timeout=None, reply_shape=None):
        budget = (self.DEFAULT_TIMEOUT if timeout is None else timeout) * self._time_scale
        with self._link_errors('reading a reply'):
            if exact_payload is not None:
                return self._read_exactly(4 + exact_payload, budget)
            return self._read_until_quiet(budget, reply_shape)

    def _first_byte(self, budget):
        """Wait up to `budget` for a reply to start, in QUIET_TIME slices."""
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
        """Wait the budget for the first byte, then read until a gap."""
        buffer = self._first_byte(budget)
        if not buffer:
            return buffer

        want = self.MAX_FRAME
        while len(buffer) < want:
            # The length is knowable for some replies as soon as the counted
            # field has arrived, and once it is known the read stops on the
            # last byte instead of on QUIET_TIME of silence after it.
            sized = (frame_length(reply_shape, buffer)
                     if want == self.MAX_FRAME else 0)
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
            self._rescale()
            self.transmit(unit, function, payload)
            reply = self.receive(exact_payload, timeout, reply_shape)
            self._quiet_since = time.monotonic()
        payload = validate(reply, unit, function)
        self._clean = True          # only past validate: a raise is not clean
        return payload

    def broadcast(self, function, payload=b'', settle=0.05):
        """Acted on by every slave, answered by none. Nothing to return."""
        self._rescale()
        self.transmit(BROADCAST, function, payload)
        if settle:
            self.sleep(settle)


#: The shape of every `u8 took` reply: one byte on success, the
#: length-prefixed refusal behind it otherwise. Passed as `reply_shape` so
#: the read stops on the last byte instead of waiting out QUIET_TIME -
#: 8 ms on the write class of transaction, which was most of its 15 ms.
ACK = {'ack': True}


def frame_length(shape, buffer):
    """Whole frame length from `shape` and what has arrived, or 0."""
    if not shape or len(buffer) < HEAD_BYTES:
        return 0
    if buffer[1] & EXCEPTION:
        return HEAD_BYTES + 1 + CRC_BYTES       # the exception code alone
    if shape.get('ack'):
        return _ack_length(buffer)
    at = HEAD_BYTES + int(shape.get('at', 0))
    if len(buffer) <= at:
        return 0
    payload = (int(shape['head']) + buffer[at] * int(shape['stride'])
               + int(shape.get('tail', 0)))
    return HEAD_BYTES + payload + CRC_BYTES


def _ack_length(buffer):
    """The `u8 took` reply's length: `1` alone, or `0` and the board's
    length-prefixed refusal - 0 until the byte that settles it is in.
    """
    if len(buffer) < HEAD_BYTES + 1:
        return 0
    if buffer[HEAD_BYTES]:
        return HEAD_BYTES + 1 + CRC_BYTES       # took=1
    if len(buffer) < HEAD_BYTES + 2:
        return 0
    return HEAD_BYTES + 2 + buffer[HEAD_BYTES + 1] + CRC_BYTES


def validate(reply, unit, function):
    """Check a reply frame and return its payload."""
    if len(reply) < 4:
        raise NoReplyError('unit %d, fc 0x%02X: %s'
                           % (unit, function,
                              reply.hex(' ') if reply else 'silence'))

    if crc16(reply[:-2]) != struct.unpack('<H', reply[-2:])[0]:
        raise CrcError('unit %d: checksum failed on %s' % (unit, reply.hex(' ')))

    if reply[0] != unit:
        raise FrameError('reply came from unit %d, asked unit %d'
                         % (reply[0], unit))

    if reply[1] == (function | EXCEPTION):
        raise ModbusException(unit, function, reply[2])

    if reply[1] != function:
        raise FrameError('reply is fc 0x%02X, asked 0x%02X' % (reply[1], function))

    return reply[2:-2]
