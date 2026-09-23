"""One lazily-opened board connection - the library's session, which the rig,
the MCP server and the model runner all open through `open_session`.
"""
import collections

from . import broker, ports, protocol
from .board import connect, disconnect, scan
from .errors import RigError
from .simulated import SimulatedSession


# `kind` is the *communication interface type*: how the host reaches the bus,
# which is not the same question as which device is on it.
INTERFACE = {'probe': 'debug probe', 'serial': 'RS485', None: 'simulated'}

Origin = collections.namedtuple(
    'Origin', 'real port baud kind label interface unit')



def _node(unit, where=None):
    """The node half of the prompt tag: which one, or all of them."""
    if unit == protocol.BROADCAST:
        return 'ALL NODES'
    if where:
        return 'node %d %s' % (unit, where)
    return 'node %d' % unit


def _label(real, port, kind, fell_back=False):
    """What the prompt and every suite header say the session is talking to."""
    if not real and fell_back:
        return 'Simulated - nothing answered on %s' % port
    if not real:
        return 'Simulated'
    if kind == 'probe':
        return 'JTAG and %s' % port
    return 'RS485 at %s' % port


def tag(origin, unit=None, where=None):
    """What the prompt shows: the interface, then the node."""
    unit = origin.unit if unit is None else unit
    return '%s, %s' % (origin.label, _node(unit, where))


def _answers(served, unit=1):
    """Whether the BOARD behind the broker answers - not just the broker."""
    # Long enough for the broker's own answer: it gives the board a second and
    # retries once across a console handover before saying no.
    reached = broker.attach((served.get('host', broker.HOST),
                             served.get('tcp', broker.PORT)), timeout=6.0)
    if reached is None:
        return False
    try:
        return reached.answers(unit)
    except (RigError, OSError):
        # OSError too: the socket read times out before the broker gives up on
        # a silent board - it retries the request across a console handover
        # first - and a timeout is a no-answer like any other.
        return False
    finally:
        reached.close()


def board_answers(port=None, baud=115200, unit=1):
    """Whether a board answers anywhere, without opening a session."""
    served = broker.serving()
    if served and _answers(served, unit):
        return True

    found, _kind = ports.discover(port, baud, unit)
    return found is not None


def open_session(port=None, baud=115200, unit=1, simulated=None, only=None):
    """`(session, origin)` - the board, or a stand-in for it."""
    kind = None
    fell_back = False

    # A BROKER IS THE BOARD.
    served = broker.serving() if only is None else None
    if served and simulated is not True and not _answers(served, unit):
        served = None

    if served and simulated is not True:
        held = served.get('kind')
        return (Session(served['serial'], baud, unit),
                Origin(True, served['serial'], baud, held,
                       _label(True, served['serial'], held) + ' - shared',
                       INTERFACE.get(held, held), unit))

    if simulated is None:
        found, kind = ports.discover(port, baud, unit, only=only)
        simulated = found is None
        fell_back = simulated
        port = port if found is None else found
    elif not simulated:
        kind = ports.kind_of(port)

    if simulated:
        return (SimulatedSession(port, baud, unit),
                Origin(False, port, baud, None,
                       _label(False, port, None, fell_back),
                       INTERFACE[None], unit))
    # A BROKER FOR IT, now that the probe has let the port go.
    broker.spawn(port, baud)

    if port is None:
        raise RigError('no port to open - none given and none found')
    return (Session(port, baud, unit),
            Origin(True, port, baud, kind, _label(True, port, kind),
                   INTERFACE.get(kind, kind), unit))


class Session:
    """One transport and the board on it."""

    #: Read by anything that must not mistake a stand-in for a board.
    simulated = False

    def __init__(self, port='COM4', baud=115200, unit=1):
        self.port = port
        self.baud = baud
        self.unit = unit
        self._board = None
        self._info = None

    @property
    def board(self):
        """The connected board, opening the link on first use."""
        if self._board is None:
            self._board = connect([(self.unit, self.baud, self.port)])[0]
        return self._board

    @property
    def attached(self):
        """The board when the link is already open, else None - never opens
        anything, unlike `board`.
        """
        return self._board

    @property
    def bus(self):
        """The segment this session is on: a real bus is a serial segment, and
        its label is its port.
        """
        return self.port

    def info(self, refresh=False):
        """Version, clock and channel table, cached: none of it changes at
        run time, and re-reading it on every call is exactly the waste
        this server exists to avoid.
        """
        if self._info is None or refresh:
            board = self.board
            self._info = (board.version_info or board.probe(),
                          board.system.clock(),
                          board.analog.channels(refresh=refresh))
        return self._info

    def buses(self):
        """[(label, what it serves)] - the segments this host can reach."""
        return [(self.port, 'the attached bus')]

    def scan(self, units=range(1, 17), bus=None):
        """[(unit, version)] for every device answering on this bus."""
        self.close()
        return scan(units, bus or self.port, self.baud)

    def use(self, unit, bus=None):
        """Point this session at another node, and another bus with it."""
        self.close()
        if bus is not None:
            self.port = bus
        self.unit = int(unit)
        return self.unit

    def close(self):
        if self._board is None:
            return
        try:
            disconnect([self._board])
        except RigError:
            pass
        finally:
            self._board = None
            self._info = None

    def reset(self):
        """Drop the connection so the next call reopens it."""
        self.close()
