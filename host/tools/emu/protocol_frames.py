"""`frames://host:port`: an emulated UART's host socket (tools.emu), each write one frame.

Its length goes ahead of it in two bytes, big-endian, so the line (board/emu/UART_Line.cs) puts
it on the wire whole. As bytes alone a write reached Renode in pieces one to three quanta apart
on CI's runner, and the line broke the frame at the gap - an echo of 50 lost, two framing
errors. Reads are the board's bytes as they come.

    serial.serial_for_url('frames://127.0.0.1:3456', 115200)    # tools.emu in the handlers
"""
import struct
import urllib.parse

from serial.serialutil import SerialException
from serial.urlhandler import protocol_socket


#: An emulated board's clock by its socket's URL, as its emulator registers it on start
#: (tools.emu.emulator): a client of the socket waits by the board's time - its scale, what says
#: it now, its virtual seconds - as the transport does on `emulator://`. A node's reply comes at
#: the line's pace on that time; on the PC's a limb's open gave up on it.
CLOCKS = {}


def frame(data):
    """`data` as one frame on the socket: its length ahead of it, two bytes big-endian."""
    data = bytes(data)
    if len(data) > 0xFFFF:
        raise SerialException('a frame of %d bytes: two length bytes carry 65 535' % len(data))
    return struct.pack('>H', len(data)) + data


class Serial(protocol_socket.Serial):
    """A TCP socket whose writes are frames, on its board's clock: the PC's where no emulator
    registered it."""

    time_scale, time_scale_source, virtual_seconds = 1.0, None, None

    def open(self):
        super().open()
        vars(self).update(CLOCKS.get(self.port, {}))

    def from_url(self, url):
        parts = urllib.parse.urlsplit(url)
        if parts.scheme != 'frames':
            raise SerialException('expected frames://host:port, not %s' % url)
        return super().from_url(urllib.parse.urlunsplit(parts._replace(scheme='socket')))

    def write(self, data):
        data = bytes(data)
        if data:
            super().write(frame(data))
        return len(data)
