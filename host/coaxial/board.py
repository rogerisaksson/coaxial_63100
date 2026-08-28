"""The board, as one object with one subsystem per functional area.

    board.system    identity, versions, clock tree, releasing the console
    board.link      echo and the protocol's frame counters
    board.afe       the analog front end switch, which also powers the reference
    board.analog    channels, bursts, temperature, DC bus
    board.gpio      raw pin access for a fixture, behind a gate

The split follows the hardware rather than the protocol: someone reading a test
script should be able to tell which part of the board a line touches without
knowing a single function code.
"""
import time

from .afe import Afe
from .analog import Analog
from .gate_drivers import GateDrivers
from .power import Power
from .thermal_device import Thermal
from .capture import Capture
from .clock import Clock
from .daq import Daq
from .calibration import Calibration
from .errors import (ConnectError, CrcError, DeviceStateError, FrameError,
                     NoReplyError, RigError, UnsupportedProtocolError)
from .gpio import Gpio
from .angle import Angle
from .imu import Imu
from .link import Link
from .protocol import BROADCAST
from .system import System
from .transport import Transport


class Board:
    """One unit on one transport. Every method raises rather than reporting."""

    def __init__(self, transport, unit=1):
        self.transport = transport
        self.unit = unit
        self.version_info = None

        self.system = System(self)
        self.link = Link(self)
        self.afe = Afe(self)
        self.analog = Analog(self)
        self.gpio = Gpio(self)
        self.imu = Imu(self)
        self.angle = Angle(self)
        self.calibration = Calibration(self)
        self.gate_drivers = GateDrivers(self)
        self.thermal = Thermal(self)
        self.power = Power(self)
        self.capture = Capture(self)
        self.clock = Clock(self)
        self.daq = Daq(self)

    @property
    def baud(self):
        """The bitrate this board is reached at. Asked of the board and not
        of the transport, because the stand-in has no transport and a caller
        measuring a link must not have to know which it is holding."""
        return self.transport.baud

    def __repr__(self):
        firmware = (self.version_info or {}).get('firmware', 'unknown fw')
        return '<Board unit=%d %s@%d %s>' % (self.unit, self.transport.port,
                                             self.transport.baud, firmware)

    # -- the single point where a transaction happens ----------------------

    def request(self, function, payload=b'', exact_payload=None, timeout=None):
        if self.unit == BROADCAST:
            # One place, because every read and every read-back write comes
            # through here. Silence from unit 0 is the protocol working, not
            # the board being dead, and a timeout would read as the second.
            from .simulated import BROADCAST_REFUSAL
            raise DeviceStateError(BROADCAST_REFUSAL)
        return self.transport.request(self.unit, function, payload,
                                      exact_payload, timeout)

    def broadcast(self, function, payload=b'', settle=0.05):
        """Acted on by every unit on the wire, answered by none."""
        self.transport.broadcast(function, payload, settle=settle)

    # -- getting the link open and shut ------------------------------------

    def open_binary(self, settle=0.5):
        """Hand USART3 from the text console to the binary protocol.

        The board boots into its console because that is how a human drives it;
        'm' is the console key that gives the line up. Not part of Modbus.
        """
        self.transport.write_text('m')
        time.sleep(settle)
        self.transport.discard_input()

    def close_binary(self):
        """Give the line back to the console."""
        self.system.release_console()

    def probe(self, tries=3):
        """Read and remember the version record. Returns it.

        Retried, and only here. A missed reply is a fact of this link -
        measured, about one transaction in fifty while the board is busy -
        and everywhere else the right answer is to raise, because a caller
        asked for a reading and did not get one. At the identity probe it
        is not: the session has nothing yet, so one silent frame turns a
        working board into "no board", and the demo that hit it had simply
        opened while the previous one was still letting go of the port.

        0x41 is the frozen version record and reading it changes nothing,
        so asking twice is asking the same question again.
        """
        last = None
        for _ in range(max(1, tries)):
            try:
                self.version_info = self.system.version()
                return self.version_info
            except (NoReplyError, CrcError, FrameError) as exc:
                last = exc
        raise last


# Protocol major -> the client class that speaks it. THIS is the lookup: a
# firmware that bumps to major 2 gets a Board subclass on a new line here, and
# every call site stays as it is. Nothing keys off the firmware version, because
# binding a host to firmware numbers means every rebuild breaks the host.
BOARD_CLASSES = {1: Board}


def _build(probe):
    """Probe with the frozen prefix, then instantiate the matching class.

    The link is already open by the time this runs: handing the UART over is a
    precondition for any traffic at all, not part of verification.
    """
    transport, unit = probe.transport, probe.unit
    info = probe.probe()

    board_class = BOARD_CLASSES.get(info['proto_major'])
    if board_class is None:
        raise UnsupportedProtocolError(
            'unit %d on %s@%d speaks protocol %d.%d; this host implements %s'
            % (unit, transport.port, transport.baud,
               info['proto_major'], info['proto_minor'],
               ', '.join(str(major) for major in sorted(BOARD_CLASSES))))

    if board_class is Board:
        return probe

    # A class registered for another major brings its own codec, so let it read
    # the record itself rather than handing it one decoded by this one.
    board = board_class(transport, unit)
    board.probe()
    return board


def _normalise(entry, default_port, default_baud):
    """Accept 1, (1, 19200) or (1, 19200, 'COM7') and return a full triple."""
    if isinstance(entry, int):
        return entry, default_baud, default_port
    if isinstance(entry, (tuple, list)) and len(entry) == 2:
        return entry[0], entry[1], default_port
    if isinstance(entry, (tuple, list)) and len(entry) == 3:
        return entry[0], entry[1], entry[2]
    raise ValueError('bad unit spec %r: expected unit, (unit, baud) or '
                     '(unit, baud, port)' % (entry,))


def scan(units=range(1, 17), port='COM4', baud=115200):
    """Which unit ids answer on this bus, and what each one says it is.

    `[(unit, version_dict)]`, in ascending unit order, skipping silence. One
    transport for the whole sweep - the port cannot be opened twice, and
    reopening it per unit would cost the console handover each time.

    Bounded by default because it is not free: a unit that is not there
    costs the transport's read timeout, so 1..16 is about eight seconds of
    silence in the worst case and 1..247 is two minutes. Widen it when a
    bus is known to be wider.
    """
    wanted = list(units)
    if not wanted:
        return []

    boards = connect([(unit, baud, port) for unit in wanted], verify=False)
    found = []
    try:
        for board in boards:
            try:
                found.append((board.unit, board.probe()))
            except RigError:
                continue            # silence is an answer: nothing is there
    finally:
        disconnect(boards)
    return found


def connect(units, port='COM4', baud=115200, verify=True):
    """Open the links and return one Board per entry, in the order given.

    units   a list of unit ids, or of (unit, baud), or of (unit, baud, port).
            Entries sharing a port and bitrate share one Transport; a different
            bitrate gets its own, because one UART cannot run two at once.

    verify  probe each unit and raise if it does not answer, or speaks a
            protocol major this host has no codec for. On by default: a rig that
            silently proceeds with a dead board produces results that look real.

    There is no partial success. A caller holding the returned list knows every
    board in it answered.
    """
    specs = [_normalise(entry, port, baud) for entry in units]
    transports = {}
    boards = []

    try:
        for unit, unit_baud, unit_port in specs:
            key = (unit_port, unit_baud)
            fresh = key not in transports
            if fresh:
                transports[key] = Transport(unit_port, unit_baud)

            board = Board(transports[key], unit)

            # The board boots into its text console, so the line has to be
            # handed over before any framing - once per port rather than once
            # per unit, and whether or not this call is going to probe. Skipping
            # it would return boards that cannot talk at all.
            if fresh:
                board.open_binary()

            if not verify:
                boards.append(board)
                continue

            try:
                boards.append(_build(board))
            except UnsupportedProtocolError:
                raise
            except RigError as exc:
                raise ConnectError('unit %d on %s@%d did not answer: %s'
                                   % (unit, unit_port, unit_baud, exc)) from exc
    except Exception:
        # No partial success: every transport opened here gets closed, even if
        # one of them refuses, and the original failure is what propagates.
        for transport in transports.values():
            try:
                transport.close()
            except RigError:
                pass
        raise

    return boards


def disconnect(boards):
    """Return every UART to its console and close the ports. Idempotent."""
    seen = set()
    for board in boards:
        if id(board.transport) in seen:
            continue
        seen.add(id(board.transport))
        try:
            try:
                # A port already closed has nothing to hand back, which is what
                # makes a second call to this function a no-op rather than an
                # error from pyserial.
                if board.transport.is_open:
                    board.close_binary()
            finally:
                board.transport.close()
        except RigError:
            # Shutting down. A board that will not answer, or a port that will
            # not close, must not strand the ports of every board after it.
            pass
