"""Which COM port this board is on - one implementation, called from every
side, so "does this port answer" cannot drift between them:

  * `coaxial.session.open_session`, looking for the board before it
    settles for the stand-in;
  * `coaxial.broker`, naming the path a held port is;
  * `coaxial_ollama`'s link_diagnose tool - the model, mid-session, when
    a call has already failed;
  * `tools/find_board.py`, the command line over all of it, which
    `board_chat/ComPort.ps1` calls before a Python session even exists.

A probe goes through `board.connect`, the same round trip a real session
makes, so a wrong port fails here for the reason it would fail inside
dbg.py - not a weaker check that passes here and fails there.
"""
from contextlib import suppress

import serial
import serial.tools.list_ports

from . import board

# STMicroelectronics. Every ST-Link VCP enumerates under this VID - measured
# here, an STLINK-V3SET reports 0483:374F - and it is what lets "which port is
# the debugger" be answered without opening a single one. Find-BoardPort's own
# comment said there was no way to ask; there is, and this is it.
ST_VID = 0x0483

PROBE = 'probe'      # the debug probe's virtual COM port
SERIAL = 'serial'    # anything else that answers: RS485, on this board

ANSWERED, BUSY, SILENT, ABSENT = 'answered', 'busy', 'silent', 'absent'


def list_ports():
    return [p.device for p in serial.tools.list_ports.comports()]


def kinds():
    """[(device, PROBE|SERIAL)], in the order the OS enumerates them.

    USB ports only (`p.vid` set): the debug probe is a USB VCP and RS485
    arrives on a USB dongle, so a port with no USB identity is never this
    board. It is also what keeps discovery bounded on a machine with
    legacy UARTs - a Linux host lists /dev/ttyS* with nothing behind
    them, opening one succeeds, and the close can sit in the driver's
    drain for the better part of a minute. Measured on CI: three suites
    at ~257 s each, every second of it probing motherboard UARTs.
    """
    return [(p.device, PROBE if p.vid == ST_VID else SERIAL)
            for p in serial.tools.list_ports.comports() if p.vid]


def kind_of(device):
    """PROBE or SERIAL for one port, SERIAL if Windows does not list it."""
    for name, kind in kinds():
        if name == device:
            return kind
    return SERIAL


def discover(preferred=None, baud=115200, unit=1, only=None):
    """`(device, kind)` of the first port this board answers on, or
    `(None, None)`.

    Order: `preferred` if Windows lists it, then every debug probe, then
    everything else. `only=PROBE` or `only=SERIAL` narrows it to one
    path. The probe goes first because it is the one that is there by
    definition when somebody is at a bench with a cable in - RS485 is
    the installed drive's path, and trying it first would spend a round
    trip per port on the common case.
    """
    listed = kinds()
    if only:
        # "switch to RS485" names the path, not a port. Without this the
        # probe-first order would answer it with the debug probe, which is
        # the one board the operator just said they did not mean.
        listed = [p for p in listed if p[1] == only]
    ordered = ([p for p in listed if p[0] == preferred]
               + [p for p in listed if p[1] == PROBE and p[0] != preferred]
               + [p for p in listed if p[1] == SERIAL and p[0] != preferred])
    for device, kind in ordered:
        if probe(device, baud, unit):
            return device, kind
    return None, None


def probe(candidate, baud=115200, unit=1):
    """True if this board answers on `candidate`. Opens and closes the link
    each time - the transport's own 0.5s read timeout (coaxial/transport.py)
    is what keeps a silent port from hanging this, not anything here.

    `board.connect` is read at the call, not bound at import: a test in the
    same process patches `coaxial.board.connect` and has it take here.
    """
    try:
        boards = board.connect([(unit, baud, candidate)])
    except Exception:                                    # noqa: BLE001
        return False
    with suppress(Exception):
        board.disconnect(boards)
    return True


def port_state(candidate, baud=115200, unit=1):
    """Why this port is not answering, not just that it is not.

    `probe` returns False for a port another process holds open exactly as it
    does for a board that has stopped talking, and those are different
    problems with different fixes. Measured, and it cost most of a session:
    two `dbg.py` sessions had COM4 open, every probe read "silent", and the
    board was diagnosed as halted, started over SWD and reflashed - none of
    which was the matter with it.

    'busy' is decided on the exception's class name rather than its message:
    Windows localises the text, and the one Measured was Swedish.
    """
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
    """The first port that answers as this board, `preferred` tried first
    if Windows even lists it - or None if nothing did."""
    if ports is None:
        ports = list_ports()
    ordered = ([preferred] if preferred in ports else []) + \
        [p for p in ports if p != preferred]
    for candidate in ordered:
        if probe(candidate, baud, unit):
            return candidate
    return None
