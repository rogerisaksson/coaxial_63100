"""The board, as one object with one subsystem per functional area."""

from coaxial.devices.afe import Afe
from coaxial.devices.analog import Analog
from coaxial.devices.gate_drivers import GateDrivers
from coaxial.devices.boot import Boot
from coaxial.devices.power import Power
from coaxial.devices.thermal import Thermal
from coaxial.acquire.capture import Capture
from coaxial.acquire.clock import Clock
from coaxial.acquire.daq import Daq
from coaxial.devices.drive import Drive
from coaxial.kalman.observer import Observer
from coaxial.devices.calibration import Calibration
from coaxial.errors import (ConnectError, CrcError, DeviceStateError, FrameError, NoReplyError,
                            RigError, UnsupportedProtocolError)
from coaxial.devices.gpio import Gpio
from coaxial.devices.angle import Angle
from coaxial.devices.imu import Imu
from coaxial.devices.link import Link
from coaxial.comm.protocol import BROADCAST, PROVEN_DISPATCH_SINCE
from coaxial.devices.subsystem import Subsystem
from coaxial.devices.system import System
from coaxial.comm import broker
from coaxial.comm.transport import Transport, hand_to_binary
from typing import Any
from coaxial.simulated import BROADCAST_REFUSAL
from contextlib import suppress


class Board:
    """One unit on one transport. Every method raises rather than reporting."""

    system: System
    link: Link
    afe: Afe
    analog: Analog
    gpio: Gpio
    imu: Imu
    angle: Angle
    calibration: Calibration
    gate_drivers: GateDrivers
    thermal: Thermal
    power: Power
    boot: Boot
    capture: Capture
    clock: Clock
    daq: Daq
    drive: Drive
    observer: Observer

    #: The rig that opened this board, set by `Coaxial63100.open()` -
    #: `observer.autodetect` drives commissioning steps that are the
    #: rig's, and a subsystem only ever holds the board.
    rig: Any = None

    def __init__(self, transport, unit=1):
        self.transport = transport
        self.unit = unit
        self.version_info = None
        for name, part in self.parts().items():
            setattr(self, name, part(self))

    @classmethod
    def parts(cls):
        """The composition: attribute name -> the subsystem class there."""
        return {name: part for name, part in cls.__annotations__.items()
                if isinstance(part, type) and issubclass(part, Subsystem)}

    @property
    def baud(self):
        """The bitrate this board is reached at."""
        return self.transport.baud

    def __repr__(self):
        firmware = (self.version_info or {}).get('firmware', 'unknown fw')
        return '<Board unit=%d %s@%d %s>' % (self.unit, self.transport.port,
                                             self.transport.baud, firmware)

    # -- the single point where a transaction happens ----------------------

    def request(self, function, payload=b'', exact_payload=None,
                timeout=None, reply_shape=None):
        if self.unit == BROADCAST:
            # One place, because every read and every read-back write comes
            # through here.
            raise DeviceStateError(BROADCAST_REFUSAL)
        return self.transport.request(self.unit, function, payload,
                                      exact_payload, timeout, reply_shape)

    def broadcast(self, function, payload=b'', settle=0.05):
        """Acted on by every unit on the wire, answered by none."""
        self.transport.broadcast(function, payload, settle=settle)

    # -- getting the link open and shut ------------------------------------

    def open_binary(self, settle=0.5):
        """Hand the line from the text console to the binary protocol."""
        hand_to_binary(self.transport, settle)

    def close_binary(self):
        """Give the line back to the console."""
        self.system.release_console()

    def probe(self, tries=3):
        """Read and remember the version record. Returns it."""
        last = None
        for _ in range(max(1, tries)):
            try:
                self.version_info = self.system.version()
                self._dispatch_on_crc(self.version_info)
                return self.version_info
            except (NoReplyError, CrcError, FrameError) as exc:
                last = exc
        if last is None:                   # tries is at least one
            raise NoReplyError('the board never answered')
        raise last

    def _dispatch_on_crc(self, info):
        """MINOR 9: the board dispatches proven requests on their own CRC,
        so the transport can stop paying the pre-TX gap after one.
        """
        major, minor = PROVEN_DISPATCH_SINCE
        proven = (info.get('proto_major') == major
                  and info.get('proto_minor', 0) >= minor)
        if proven:
            self.transport.proven_dispatch = True


# Protocol major -> the client class that speaks it.
#: Major 1 - the six-node thermal vocabulary - has no codec here any more:
#: this Board speaks per-leg nodes, and labelling an old board's six with
#: ten names would be worse than the refusal.
BOARD_CLASSES = {2: Board}


def _build(probe):
    """Probe with the frozen prefix, then instantiate the matching class."""
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
    if not isinstance(entry, (int, tuple, list)):
        raise ValueError('bad unit spec %r: expected unit, (unit, baud) or '
                         '(unit, baud, port)' % (entry,))
    spec = (entry,) if isinstance(entry, int) else tuple(entry)
    if len(spec) not in (1, 2, 3):
        raise ValueError('bad unit spec %r: expected unit, (unit, baud) or '
                         '(unit, baud, port)' % (entry,))
    return spec + (default_baud, default_port)[len(spec) - 1:]


def scan(units=range(1, 17), port='COM4', baud=115200):
    """Which unit ids answer on this bus, and what each one says it is."""
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


def _reach(port, baud):
    """The broker for this port, started if there is not one yet."""
    reached = _attach(port)
    if reached is None and broker.spawn(port, baud):
        reached = _attach(port)
    return Transport(port, baud) if reached is None else reached


def _attach(port):
    """A broker client for `port`, or None if none is serving it."""
    said = broker.serving()
    if not said or said.get('serial') != port:
        return None
    return broker.attach((said.get('host', broker.HOST),
                          said.get('tcp', broker.PORT)))


def _open_one(spec, transports, verify):
    """One entry's Board, on a transport shared by port and bitrate."""
    unit, unit_baud, unit_port = spec
    key = (unit_port, unit_baud)
    fresh = key not in transports
    if fresh:
        transports[key] = _reach(unit_port, unit_baud)
    board = Board(transports[key], unit)
    if fresh and not transports[key].address:
        board.open_binary()
    if not verify:
        return board
    try:
        return _build(board)
    except UnsupportedProtocolError:
        raise
    except RigError as exc:
        raise ConnectError('unit %d on %s@%d did not answer: %s'
                           % (unit, unit_port, unit_baud, exc)) from exc


def connect(units, port='COM4', baud=115200, verify=True):
    """Open the links and return one Board per entry, in the order given."""
    specs = [_normalise(entry, port, baud) for entry in units]
    transports = {}

    try:
        return [_open_one(spec, transports, verify) for spec in specs]
    except Exception:
        # No partial success: every transport opened here gets closed, even if
        # one of them refuses, and the original failure is what propagates.
        for transport in transports.values():
            with suppress(RigError):
                transport.close()
        raise


def _hand_back(board):
    """Return one board's UART to its console and close its port."""
    with suppress(RigError):
        try:
            if board.transport.is_open:
                board.close_binary()
        finally:
            board.transport.close()


def disconnect(boards):
    """Return every UART to its console and close the ports. Idempotent."""
    seen = set()
    for board in boards:
        if id(board.transport) in seen:
            continue
        seen.add(id(board.transport))
        _hand_back(board)
