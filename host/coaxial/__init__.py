"""Host library for the coaxial_63100 test firmware."""
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
