"""Host library for the coaxial_63100 test firmware."""
from coaxial.comm import protocol
from coaxial.devices import scaling
from coaxial.acquire.acquisition import Acquisition
from coaxial.devices.board import BOARD_CLASSES, Board, connect, disconnect, scan
from coaxial.devices.gates import GateControl, GateStage
from coaxial.devices.sensor import PolledSensor
from coaxial.rig import Coaxial63100
from coaxial.errors import (ConnectError, CrcError, DeviceStateError, FrameError, ModbusException,
                            NoReplyError, PayloadError, RigError, UnsupportedProtocolError)
from coaxial.devices.scaling import (DCBUS_ONBOARD, NTC_ONBOARD, PHASE_ONBOARD, DividerParams,
                                     NtcParams, ShuntParams)
from coaxial.comm.transport import Transport

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
