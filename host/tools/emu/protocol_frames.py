"""`frames://host:port`: an emulated UART's host socket (tools.emu), each write one frame.

Its length goes ahead of it in two bytes, big-endian, so the line (board/emu/UART_Line.cs) puts
it on the wire whole. As bytes alone a write reached Renode in pieces one to three quanta apart
on CI's runner, and the line broke the frame at the gap - an echo of 50 lost, two framing
errors (2026-09-28). Reads are the board's bytes as they come.

    serial.serial_for_url('frames://127.0.0.1:3456', 115200)    # tools.emu in the handlers
"""
import struct
import urllib.parse

from serial.serialutil import SerialException
from serial.urlhandler import protocol_socket


def frame(data):
    """`data` as one frame on the socket: its length ahead of it, two bytes big-endian."""
    data = bytes(data)
    if len(data) > 0xFFFF:
        raise SerialException('a frame of %d bytes: two length bytes carry 65 535' % len(data))
    return struct.pack('>H', len(data)) + data


class Serial(protocol_socket.Serial):
    """A TCP socket whose writes are frames."""

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
