"""The firmware's comms/ on this host, over a fake board (board/fake), as a serial port.

    import tools.cores.fakeboard            # registers the scheme
    Coaxial63100(port='fakeboard://', own_image=False).open()

What answers is the firmware's own wire - the Modbus RTU slave, the command tables, every
handler - built with the host's compiler; under it the board API answers neutrally
(board/fake/fake_board.c), the calibration record is the board's own over a RAM sector
(board_cal.c, board/fake/fake_flash.c), the thermal observer too (board_thermal.c), the ports are a byte ring and a clock the
exchange steps (board/fake/fake_uart.c). One fake board a process.
"""
import ctypes
import glob
import os

import serial
from serial.serialutil import SerialBase

from tools import REPO
from tools.cores.build import build, find_cc

#: comms/ less what touches the HAL - the console, the test rig, the UARTs - and the
#: portable cores under it; board/fake and the record for the board.
SOURCES = sorted(p for p in glob.glob(os.path.join(REPO, 'comms', 'src', '*.c'))
                 if os.path.basename(p) not in ('console.c', 'testrig.c', 'dev_uart.c')) + [
    p for core in ('modbus', 'drive', 'thermal', 'daq', 'ctrl', 'filter', 'shtp')
    for p in sorted(glob.glob(os.path.join(REPO, core, 'src', '*.c')))] + sorted(
    glob.glob(os.path.join(REPO, 'board', 'fake', '*.c'))) + [
    os.path.join(REPO, 'board', 'src', name) for name in ('board_cal.c', 'board_thermal.c')]

#: The cores' headers first: comms/inc/board has same-named ones for the board's API,
#: which comms/ includes as "board/<x>.h"; board/fake before board/inc, for its board_hw.h.
INCLUDES = [os.path.join(REPO, part) for part in (
    'ctrl/inc', 'drive/inc', 'thermal/inc', 'daq/inc', 'filter/inc', 'shtp/inc', 'boot/inc',
    'modbus/inc', 'comms/inc', 'comms/inc/board', 'board/fake', 'board/inc')]

#: The longest answer an exchange takes back.
ANSWER = 4096

#: The library a parent process built, loaded by its children as it is: Windows refuses to
#: overwrite a DLL a live process holds.
BUILT = 'COAXIAL_FAKEBOARD'

_LIB = []


def library():
    """The fake board, built on first use."""
    if not _LIB:
        path = os.environ.get(BUILT)
        if not path:
            cc = find_cc()
            if cc is None:
                raise RuntimeError('no C compiler for the fake board')
            path, _warnings = build(cc, SOURCES, INCLUDES, 'fakeboard')
            os.environ[BUILT] = path
        lib = ctypes.CDLL(path)
        lib.fake_exchange.restype = ctypes.c_uint16
        lib.fake_exchange.argtypes = [ctypes.c_char_p, ctypes.c_uint16,
                                      ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint16]
        _LIB.append(lib)
    return _LIB[0]


class Serial(SerialBase):
    """`fakeboard://` as pyserial opens it: what is written is one exchange with the fake
    board, its answer what the next reads take."""

    def open(self):
        self._lib = library()
        self._lib.fake_open()
        self._answer = bytearray()
        self.is_open = True

    def close(self):
        self.is_open = False

    def _reconfigure_port(self, force_update=False):
        pass

    @property
    def in_waiting(self):
        return len(self._answer)

    def read(self, size=1):
        got, self._answer = bytes(self._answer[:size]), self._answer[size:]
        return got

    def write(self, data):
        data = bytes(data)
        out = (ctypes.c_uint8 * ANSWER)()
        n = self._lib.fake_exchange(data, len(data), out, ANSWER)
        self._answer += bytes(out[:n])
        return len(data)

    def reset_input_buffer(self):
        self._answer = bytearray()

    def reset_output_buffer(self):
        pass

    def flush(self):
        pass


if 'tools.cores' not in serial.protocol_handler_packages:
    serial.protocol_handler_packages.append('tools.cores')
