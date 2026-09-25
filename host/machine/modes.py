"""Where a machine's boards run: the hardware, a stand-in for it, or their images emulated."""
import enum


class ExecutionMode(enum.Enum):

    """HARDWARE: the boards on the port - a stand-in where none answers, if the caller allows
    one; SIMULATED: the stand-in; EMULATED: each board's own image on an emulated MCU."""

    HARDWARE = 'hardware'
    SIMULATED = 'simulated'
    EMULATED = 'emulated'


HARDWARE = ExecutionMode.HARDWARE
SIMULATED = ExecutionMode.SIMULATED
EMULATED = ExecutionMode.EMULATED
