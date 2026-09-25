"""The Coaxial63100 BLDC inverter: the board over its wire, its stand-in, its procedures."""
from coaxial.comm import protocol
from coaxial.devices import scaling
from coaxial.acquire.acquisition import Acquisition
from coaxial.devices.board import BOARD_CLASSES, Board, connect, disconnect, scan
from coaxial.devices.gates import GateControl, GateStage
from coaxial.devices.sensor import PolledSensor
from coaxial.rig import Coaxial63100
from machine.modes import EMULATED, HARDWARE, SIMULATED, ExecutionMode
from coaxial.errors import (ConnectError, CrcError, DeviceStateError, FrameError, ModbusException,
                            NoReplyError, PayloadError, RigError, UnsupportedProtocolError)
from coaxial.devices.scaling import (DCBUS_ONBOARD, NTC_ONBOARD, PHASE_ONBOARD, DividerParams,
                                     NtcParams, ShuntParams)
from coaxial.comm.transport import Transport

__all__ = [
    'Coaxial63100', 'ExecutionMode', 'HARDWARE', 'SIMULATED', 'EMULATED',
    'Acquisition', 'PolledSensor', 'GateControl', 'GateStage',
    'connect', 'disconnect', 'scan', 'Board', 'BOARD_CLASSES', 'Transport',
    'NtcParams', 'DividerParams', 'ShuntParams',
    'NTC_ONBOARD', 'DCBUS_ONBOARD', 'PHASE_ONBOARD',
    'RigError', 'ConnectError', 'NoReplyError', 'CrcError', 'FrameError',
    'PayloadError', 'UnsupportedProtocolError', 'DeviceStateError',
    'ModbusException', 'protocol', 'scaling',
]
