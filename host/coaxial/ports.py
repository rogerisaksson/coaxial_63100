"""Which COM port this board is on - one implementation, called from every
side, so "does this port answer" cannot drift between them:
"""
from contextlib import suppress

import serial
import serial.tools.list_ports

from . import board
from .errors import LINK_FAULTS

# STMicroelectronics.
ST_VID = 0x0483

PROBE = 'probe'      # the debug probe's virtual COM port
SERIAL = 'serial'    # anything else that answers: RS485, on this board

ANSWERED, BUSY, SILENT, ABSENT = 'answered', 'busy', 'silent', 'absent'


def list_ports():
    return [p.device for p in serial.tools.list_ports.comports()]


def kinds():
    """[(device, PROBE|SERIAL)], in the order the OS enumerates them."""
    return [(p.device, PROBE if p.vid == ST_VID else SERIAL)
            for p in serial.tools.list_ports.comports() if p.vid]


def kind_of(device):
    """PROBE or SERIAL for one port, SERIAL if Windows does not list it."""
    for name, kind in kinds():
        if name == device:
            return kind
    return SERIAL


def discover(preferred=None, baud=115200, unit=1, only=None):
    """`(device, kind)` of the first port this board answers on, or `(None,
    None)`.
    """
    listed = kinds()
    if only:
        # "switch to RS485" names the path, not a port.
        listed = [p for p in listed if p[1] == only]
    ordered = ([p for p in listed if p[0] == preferred]
               + [p for p in listed if p[1] == PROBE and p[0] != preferred]
               + [p for p in listed if p[1] == SERIAL and p[0] != preferred])
    for device, kind in ordered:
        if probe(device, baud, unit):
            return device, kind
    return None, None


def probe(candidate, baud=115200, unit=1):
    """True if this board answers on `candidate`."""
    try:
        boards = board.connect([(unit, baud, candidate)])
    except LINK_FAULTS:
        return False
    with suppress(Exception):
        board.disconnect(boards)
    return True


def port_state(candidate, baud=115200, unit=1):
    """Why this port is not answering, not just that it is not."""
    try:
        handle = serial.Serial(candidate, baud, timeout=0.1)
    except serial.SerialException as exc:
        text = str(exc)
        if 'PermissionError' in text or 'Access is denied' in text:
            return BUSY
        return ABSENT
    handle.close()
    return ANSWERED if probe(candidate, baud, unit) else SILENT


def find(preferred=None, baud=115200, unit=1, ports=None):
    """The first port that answers as this board, `preferred` tried first if
    Windows even lists it - or None if nothing did.
    """
    if ports is None:
        ports = list_ports()
    ordered = ([preferred] if preferred in ports else []) + \
        [p for p in ports if p != preferred]
    for candidate in ordered:
        if probe(candidate, baud, unit):
            return candidate
    return None
