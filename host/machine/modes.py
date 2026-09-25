"""Where a machine's boards run: the hardware, a stand-in, their images emulated, or none."""
import enum


class ExecutionMode(enum.Enum):

    """HARDWARE: the boards on the port - a stand-in where none answers, if the caller allows
    one; SIMULATED: the stand-in; EMULATED: each board's own image on an emulated MCU;
    VIRTUAL: no board, each actuator where it is told (`machine.virtual`)."""

    HARDWARE = 'hardware'
    SIMULATED = 'simulated'
    EMULATED = 'emulated'
    VIRTUAL = 'virtual'


HARDWARE = ExecutionMode.HARDWARE
SIMULATED = ExecutionMode.SIMULATED
EMULATED = ExecutionMode.EMULATED
VIRTUAL = ExecutionMode.VIRTUAL
