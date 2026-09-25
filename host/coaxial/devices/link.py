"""The communication link itself: echo and the protocol's own counters."""
from coaxial.comm import protocol
from coaxial.comm.protocol import LinkOp
from coaxial.comm.wire import Reader, pack
from coaxial.devices.subsystem import Device
from coaxial.errors import DeviceStateError, FrameError
from machine.roles import Endpoint

#: Every loopback pattern returned, one bit each.
ALL_PATTERNS = (1 << len(protocol.ECHO_PATTERNS)) - 1


def _port(port):
    """`port` as the board numbers them, or a raise naming the three."""
    if port not in protocol.PORTS:
        raise ValueError('port %r is not one of the three' % (port,))
    return port


class Link(Device, Endpoint, device=protocol.DEVICE_LINK):
    """Diagnostics for the wire, as opposed to the board on the end of it."""

    def echo(self, data):
        """Round-trip arbitrary bytes and verify they came back unchanged."""
        payload = data.encode() if isinstance(data, str) else bytes(data)

        if len(payload) > protocol.MAX_PAYLOAD:
            raise ValueError('%d bytes will not fit one frame; the limit is %d'
                             % (len(payload), protocol.MAX_PAYLOAD))

        returned = self.request(protocol.ECHO, payload)

        if returned != payload:
            raise FrameError('echo came back altered: sent %r, got %r'
                             % (payload, returned))

        return data

    def loopback(self, port):
        """Have the board send four patterns on `port` and say what returned.
        """
        reply = self._op(LinkOp.ECHO, pack(('u8', _port(port))))
        # MINOR 21: the port carrying the request refused in words.
        if reply[:1] == b'\x00' and len(reply) > 1 and len(reply) == 2 + reply[1]:
            raise DeviceStateError(Reader(reply[1:]).string())
        r = Reader(reply)
        index, rs485, matched, seen = r.u8(), bool(r.u8()), r.u8(), r.u8()

        return {
            'port': index,
            'name': r.string(),
            'rs485': rs485,
            'matched': matched,
            'returned': seen,
            'patterns': [{'sent': p, 'back': bool(matched & (1 << i))}
                         for i, p in enumerate(protocol.ECHO_PATTERNS)],
            'ok': matched == (ALL_PATTERNS if rs485 else 0),
        }

    def _port_state(self, port):
        r = Reader(self._op(LinkOp.STATS, pack(('u8', _port(port)))))
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

    def state(self, port=None):
        """The slave's frame counters, named as in the specification; with `port`
        (0 console, 1-2 RS485) that port's framing state and counters."""
        if port is not None:
            return self._port_state(port)
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
