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

    def _op(self, op, payload=b''):
        """One 0x6E request for the link device. The device byte lives here."""
        return self.request(protocol.DEVICE,
                            bytes([protocol.DEVICE_LINK, op]) + bytes(payload))

    def loopback(self, port):
        """Have the board send four patterns on `port` and say what returned.

        Not `echo()`, which round-trips a payload through this link: this is
        the board talking to its own receiver, and it is how an RS485 port is
        checked with nothing else on the segment. The transceivers have RE
        tied to GND, so all four patterns must come back on ports 1 and 2;
        on the console port nothing does, and that is correct rather than a
        fault.

        Four bytes go on the bus. Nothing calls it on a timer.
        """
        if port not in protocol.PORTS:
            raise ValueError('port %r is not one of the three' % (port,))

        r = Reader(self._op(protocol.LINK_OP_ECHO, bytes([port])))
        index, rs485, matched, seen = r.u8(), bool(r.u8()), r.u8(), r.u8()

        return {
            'port': index,
            'name': r.string(),
            'rs485': rs485,
            'matched': matched,
            'returned': seen,
            'patterns': [{'sent': p, 'back': bool(matched & (1 << i))}
                         for i, p in enumerate(protocol.ECHO_PATTERNS)],
            'ok': matched == 0x0F if rs485 else matched == 0,
        }

    def port_stats(self, port):
        """One port's framing state and counters.

        `bus_message` counts every frame seen on the segment and
        `server_message` only the ones addressed to this unit. On a multidrop
        bus the difference is the traffic meant for another node, which is
        what says the address filter is working rather than that the wire is
        quiet.
        """
        if port not in protocol.PORTS:
            raise ValueError('port %r is not one of the three' % (port,))

        r = Reader(self._op(protocol.LINK_OP_STATS, bytes([port])))
        got = {
            'port': r.u8(),
            'unit_id': r.u8(),
            'rs485': bool(r.u8()),
            'open': bool(r.u8()),
            'baud': r.u32(),
            't15_ticks': r.u32(),
            't35_ticks': r.u32(),
            'bus_message': r.u32(),
            'bus_comm_error': r.u32(),
            'server_message': r.u32(),
            'server_exception': r.u32(),
            'server_no_response': r.u32(),
            'char_overrun': r.u32(),
            'ring_dropped': r.u32(),
        }
        got['name'] = r.string()
        got['for_others'] = max(0, got['bus_message'] - got['server_message'])
        return got

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
