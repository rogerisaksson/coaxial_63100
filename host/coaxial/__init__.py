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

    Board + subsystems   what the hardware can do         coaxial/board.py
    protocol             command codes and versioning     coaxial/protocol.py
    Transport            Modbus RTU framing over serial   coaxial/transport.py
    wire, crc            payload and checksum codecs      coaxial/wire.py

Nothing returns a status code and nothing returns None to mean failure. Every
call either produces its result or raises something from coaxial.errors.
"""
from . import protocol, scaling
from .board import BOARD_CLASSES, Board, connect, disconnect, scan
from .errors import (ConnectError, CrcError, DeviceStateError, FrameError,
                     ModbusException, NoReplyError, PayloadError, RigError,
                     UnsupportedProtocolError)
from .scaling import DCBUS_ONBOARD, NTC_ONBOARD, DividerParams, NtcParams
from .transport import Transport

__all__ = [
    'connect', 'disconnect', 'scan', 'Board', 'BOARD_CLASSES', 'Transport',
    'NtcParams', 'DividerParams', 'NTC_ONBOARD', 'DCBUS_ONBOARD',
    'RigError', 'ConnectError', 'NoReplyError', 'CrcError', 'FrameError',
    'PayloadError', 'UnsupportedProtocolError', 'DeviceStateError',
    'ModbusException', 'protocol', 'scaling',
]

__version__ = '2.0.0'
