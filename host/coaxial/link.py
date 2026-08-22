"""The communication link itself: echo and the protocol's own counters."""
from . import protocol
from .errors import FrameError
from .subsystem import Subsystem
from .wire import Reader


class Link(Subsystem):
    """Diagnostics for the wire, as opposed to the board on the end of it."""

    def echo(self, data):
        """Round-trip arbitrary bytes and verify they came back unchanged.

        The first thing a fixture should do. It exercises framing, checksum and
        both codecs while touching no board state at all, so a failure here is
        unambiguously the link.
        """
        payload = data.encode() if isinstance(data, str) else bytes(data)

        if len(payload) > protocol.MAX_PAYLOAD:
            raise ValueError('%d bytes will not fit one frame; the limit is %d'
                             % (len(payload), protocol.MAX_PAYLOAD))

        returned = self.request(protocol.ECHO, payload)

        if returned != payload:
            raise FrameError('echo came back altered: sent %r, got %r'
                             % (payload, returned))

        return data

    def stats(self):
        """Frame counters kept by the slave, named as in the specification."""
        reader = Reader(self.request(protocol.LINK_STATS))
        return {
            'unit_id': reader.u8(),
            't15_ticks': reader.u32(),
            't35_ticks': reader.u32(),
            'bus_message': reader.u32(),
            'bus_comm_error': reader.u32(),
            'server_message': reader.u32(),
            'server_exception': reader.u32(),
            'server_no_response': reader.u32(),
            'char_overrun': reader.u32(),
        }
