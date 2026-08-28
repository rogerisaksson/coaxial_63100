"""Host library for the coaxial_63100 test firmware.

    from coaxial import connect, disconnect

    boards = connect([(1, 115200)])

    for board in boards:
        print(board.link.echo('Hello slave!'))
        board.afe.enable()
        print(board.analog.ntc_temperature())
        print(board.analog.dcbus_voltage())

    disconnect(boards)

Layered the same way the firmware is:

    Acquisition          configure, start, acquire, stop     coaxial/acquisition.py
    PolledSensor         a part the board's loop reads    coaxial/sensor.py
    GateControl          the gate-drive ops               coaxial/gates.py
    Board + subsystems   what the hardware can do         coaxial/board.py
    protocol             command codes and versioning     coaxial/protocol.py
    Transport            Modbus RTU framing over serial   coaxial/transport.py
    wire, crc            payload and checksum codecs      coaxial/wire.py

The three interfaces are what the stand-in is held to. Every one of them has a
real implementation and a simulated one, so a name that drifts between them
fails at construction rather than on the first call that reaches for it.
`GateStage` is the odd one out and deliberately concrete: it is the policy for
arming - the checks, the interlock - and there is exactly one of that.

Nothing returns a status code and nothing returns None to mean failure. Every
call either produces its result or raises something from coaxial.errors.
"""
from . import protocol, scaling
from .acquisition import Acquisition
from .board import BOARD_CLASSES, Board, connect, disconnect, scan
from .gates import GateControl, GateStage
from .sensor import PolledSensor
from .rig import Coaxial63100
from .errors import (ConnectError, CrcError, DeviceStateError, FrameError,
                     ModbusException, NoReplyError, PayloadError, RigError,
                     UnsupportedProtocolError)
from .scaling import (DCBUS_ONBOARD, NTC_ONBOARD, PHASE_ONBOARD,
                      DividerParams, NtcParams, ShuntParams)
from .transport import Transport

__all__ = [
    'Coaxial63100',
    'Acquisition', 'PolledSensor', 'GateControl', 'GateStage',
    'connect', 'disconnect', 'scan', 'Board', 'BOARD_CLASSES', 'Transport',
    'NtcParams', 'DividerParams', 'ShuntParams',
    'NTC_ONBOARD', 'DCBUS_ONBOARD', 'PHASE_ONBOARD',
    'RigError', 'ConnectError', 'NoReplyError', 'CrcError', 'FrameError',
    'PayloadError', 'UnsupportedProtocolError', 'DeviceStateError',
    'ModbusException', 'protocol', 'scaling',
]

__version__ = '2.0.0'
