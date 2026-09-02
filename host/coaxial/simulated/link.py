"""The wire that is not there: the link stand-in, the five buses of
the simulated machine, and the broadcast refusal."""
from .. import protocol
from ..errors import DeviceStateError


class SimulatedLink:
    """A stand-in link. It builds no frames: the point is that a missing
    cable is not a failing suite, not that the protocol is exercised."""
    def echo(self, data):
        return data

    def stats(self):
        return self.port_stats(0)

    def loopback(self, port):
        """What a healthy board answers: all four patterns back on the two
        RS485 ports, none on the console port, and the port carrying the
        conversation refused."""
        if port not in protocol.PORTS:
            raise ValueError('port %r is not one of the three' % (port,))
        if port == 0:
            raise DeviceStateError(
                'port 0 carries this conversation; a port cannot check its '
                'own loopback while it is answering on it')

        return {
            'port': port, 'name': protocol.PORTS[port], 'rs485': True,
            'matched': 0x0F, 'returned': 4,
            'patterns': [{'sent': p, 'back': True}
                         for p in protocol.ECHO_PATTERNS],
            'ok': True,
        }

    def port_stats(self, port=0):
        if port not in protocol.PORTS:
            raise ValueError('port %r is not one of the three' % (port,))
        rs485 = port != 0
        return {'port': port, 'name': protocol.PORTS[port], 'rs485': rs485,
                'open': True, 'baud': 115200, 'unit_id': 1,
                't15_ticks': 1750, 't35_ticks': 4083,
                'bus_message': 42, 'bus_comm_error': 0, 'server_message': 42,
                'server_exception': 0, 'server_no_response': 0,
                'char_overrun': 0, 'ring_dropped': 0, 'for_others': 0}


BROADCAST_REFUSAL = ('unit 0 is the broadcast address: every node acts on a '
                     'broadcast and none answers it, so there is nothing to '
                     'read back. Send an order with broadcast(), or select '
                     'one node.')


class _BroadcastRefuses:
    """Every read on unit 0, refused the way the real board refuses it.

    The real Board has one guard, in request(), which every subsystem call
    goes through. The stand-in has no such choke point - its subsystems
    answer directly - so this stands in for all of them at once. Without
    it a broadcast read succeeded here and raised on the board, which is
    the difference test_parity.py exists to catch.
    """

    def __getattr__(self, _name):
        def refuse(*_a, **_k):
            raise DeviceStateError(BROADCAST_REFUSAL)
        return refuse


# Five buses, one per limb plus the axis - shorter runs, a limb's fault
# confined to it, and four segments carrying traffic at once.
#
# The bus says the side, so the unit id says the position down the limb:
# node 2 is the knee on LL and on RL, which is worth more than a unique
# number. Two-letter labels, not emoji - this is a column-aligned table.
SIMULATED_BUSES = {
    # label: (what it serves, {unit: (name, type, where)})
    'LL': ('left leg', {
        1: ('coaxial_63100', 'bldc_inverter', 'left hip'),
        2: ('coaxial_63100', 'bldc_inverter', 'left knee'),
        3: ('coaxial_63020', 'bldc_inverter', 'left ankle'),
        4: ('coaxial_63020', 'bldc_inverter', 'left foot'),
    }),
    'RL': ('right leg', {
        1: ('coaxial_63100', 'bldc_inverter', 'right hip'),
        2: ('coaxial_63100', 'bldc_inverter', 'right knee'),
        3: ('coaxial_63020', 'bldc_inverter', 'right ankle'),
        4: ('coaxial_63020', 'bldc_inverter', 'right foot'),
    }),
    'LA': ('left arm', {
        1: ('coaxial_63100', 'bldc_inverter', 'left shoulder'),
        2: ('coaxial_63020', 'bldc_inverter', 'left elbow'),
        3: ('coaxial_63020', 'bldc_inverter', 'left wrist'),
        4: ('coaxial_63020', 'bldc_inverter', 'left gripper'),
    }),
    'RA': ('right arm', {
        1: ('coaxial_63100', 'bldc_inverter', 'right shoulder'),
        2: ('coaxial_63020', 'bldc_inverter', 'right elbow'),
        3: ('coaxial_63020', 'bldc_inverter', 'right wrist'),
        4: ('coaxial_63020', 'bldc_inverter', 'right gripper'),
    }),
    'AX': ('axis', {
        1: ('coaxial_63100', 'bldc_inverter', 'pelvis'),
        2: ('coaxial_63100', 'bldc_inverter', 'waist'),
        3: ('coaxial_63020', 'bldc_inverter', 'neck'),
        4: ('coaxial_63020', 'bldc_inverter', 'head'),
    }),
}

DEFAULT_BUS = 'AX'


def bus_nodes(label):
    """{unit: (name, type, where)} for one bus, empty for an unknown one."""
    return SIMULATED_BUSES.get(label, ('', {}))[1]
