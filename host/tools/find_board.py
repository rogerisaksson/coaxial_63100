#!/usr/bin/env python3
"""Which COM port this board is on - one implementation, called from both sides,
so "does this port answer" cannot drift between them:

  * host/coaxial_ollama/tools.py's link_diagnose tool - the model, mid-
    session, when a call has already failed.
  * board_prompt/ComPort.ps1's Test-BoardPort/Find-BoardPort -
    -AutodetectComport, before a Python session even exists.

It goes through coaxial.connect(), the same round trip a real session makes, so
a wrong port fails here for the reason it would fail inside dbg.py - not a
weaker check that passes here and fails there.

    python tools/find_board.py --list                 # ports Windows sees
    python tools/find_board.py --probe COM4           # does the board answer here?
    python tools/find_board.py --find                 # try every port, print the first that answers
    python tools/find_board.py --find --preferred COM4   # try that one first
    python tools/find_board.py --discover             # probes first; prints "COM4 probe"
    python tools/find_board.py --power                # target voltage over SWD - the ST-Link, not USART3
"""
import argparse
import os
import sys

# host/ on the path: this file's own directory's parent, so it does
# not matter what the working directory is or what any directory
# along the way is called.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def check_power(timeout=15):
    """(voltage_or_None, detail). Whether the ST-Link senses power on the
    target at all - the most fundamental check there is, and one the serial
    side cannot make on its own: a board with no target voltage is never
    going to answer over USART3, whatever the COM port list says. Measured
    live : an unplugged ST-Link cable read `Voltage: 0.00V`
    from STM32_Programmer_CLI, with the serial side reporting nothing more
    specific than silence.

    Runs a bare SWD connect, no flash - `build_and_flash.py`'s own
    toolchain-path resolution finds the programmer, since it is not on PATH
    unless env.ps1 has already run in this shell.

    `mode=HOTPLUG`, never `mode=UR`. Connect-under-reset asserts NRST, and
    this call has a timeout that kills the programmer where it stands: a
    connect killed mid-reset can leave the target held there, and a halted
    core answers nothing on USART3. Measured - a `--power`
    run timed out at 15s, and every serial call afterwards was silent, on
    both the console and raw Modbus, until `-c port=SWD mode=UR --start`
    brought it back. HOTPLUG cannot do that: it never touches reset.
    Diagnosing the link must not be able to break it.

    voltage is None only when the programmer could not be found, or wrote
    no reading before its timeout killed it - not the same as 0.00V, which
    is a real reading that says the target has none.
    """
    import re
    import subprocess

    import build_and_flash

    path = build_and_flash.toolchain_path()
    programmer = build_and_flash.find_programmer(path)
    if programmer is None:
        return None, 'STM32_Programmer_CLI not found - see setup.ps1'

    timed_out = False
    try:
        done = subprocess.run([programmer, '-c', 'port=SWD',
                               'mode=HOTPLUG', '-q'],
                              capture_output=True, text=True,
                              encoding='utf-8', errors='replace',
                              timeout=timeout)
        output = (done.stdout or '') + (done.stderr or '')
    except subprocess.TimeoutExpired as exc:
        # The voltage is printed in the first second; the run then spends
        # the rest on a second connect attempt at 8MHz. Measured with no
        # target: 30.3s total against this 15s budget, and `exc.stdout`
        # already holding `Voltage: 0.00V`. Parsing it is the difference
        # between answering the question this check exists for and
        # reporting "unknown" for the one case it was written to catch.
        timed_out = True
        output = (exc.stdout or '') + (exc.stderr or '')

    match = re.search(r'Voltage\s*:\s*([\d.]+)\s*V', output)
    if match:
        return float(match.group(1)), output.strip()
    if timed_out:
        return None, 'STM32_Programmer_CLI did not answer within %ss' % timeout
    tail = output.strip().splitlines()[-1] if output.strip() else 'no output'
    return None, 'no voltage reading - %s' % tail


# STMicroelectronics. Every ST-Link VCP enumerates under this VID - measured
# here, an STLINK-V3SET reports 0483:374F - and it is what lets "which port is
# the debugger" be answered without opening a single one. Find-BoardPort's own
# comment said there was no way to ask; there is, and this is it.
ST_VID = 0x0483

PROBE = 'probe'      # the debug probe's virtual COM port
SERIAL = 'serial'    # anything else that answers: RS485, on this board


def list_ports():
    import serial.tools.list_ports
    return [p.device for p in serial.tools.list_ports.comports()]


def kinds():
    """[(device, PROBE|SERIAL)], in the order Windows enumerates them."""
    import serial.tools.list_ports
    return [(p.device, PROBE if p.vid == ST_VID else SERIAL)
            for p in serial.tools.list_ports.comports()]


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
    everything else. `only=PROBE` or `only=SERIAL` narrows it to one path. The probe goes first because it is the one that is
    there by definition when somebody is at a bench with a cable in - RS485
    is the installed drive's path, and trying it first would spend a round
    trip per port on the common case.
    """
    listed = kinds()
    if only:
        # "byt till RS485" names the path, not a port. Without this the
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

    Imports coaxial fresh on every call rather than once at module load, on
    purpose: a caller in the same process (tools.py's link_diagnose) can
    patch `coaxial.connect` for a test and have it actually take - `from
    coaxial import connect` at import time would bind this module's own
    name once and never see a patch applied afterward.
    """
    import coaxial

    try:
        boards = coaxial.connect([(unit, baud, candidate)])
    except Exception:                                    # noqa: BLE001
        return False
    try:
        coaxial.disconnect(boards)
    except Exception:                                     # noqa: BLE001
        pass
    return True


ANSWERED, BUSY, SILENT, ABSENT = 'answered', 'busy', 'silent', 'absent'


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
    import serial

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


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--list', action='store_true',
                        help='every COM port Windows currently sees')
    parser.add_argument('--kinds', action='store_true',
                        help='the same, one "PORT kind" per line, without '
                             'opening any of them - the USB VID decides')
    parser.add_argument('--probe', metavar='PORT',
                        help='does the board answer on this one port')
    parser.add_argument('--state', metavar='PORT',
                        help='answered, busy, silent or absent - why it is '
                             'not answering, not just that it is not')
    parser.add_argument('--find', action='store_true',
                        help='try every port, print the first that answers')
    parser.add_argument('--discover', action='store_true',
                        help='the same, debug probes first, printing "PORT '
                             'kind" - kind being probe or serial')
    parser.add_argument('--preferred', metavar='PORT',
                        help='with --find, try this one first')
    parser.add_argument('--power', action='store_true',
                        help='target voltage over SWD, via the ST-Link')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--unit', type=int, default=1)
    args = parser.parse_args(argv)

    if args.power:
        voltage, detail = check_power()
        if voltage is None:
            print('unknown - %s' % detail)
            return 1
        print('%.2fV' % voltage)
        return 0 if voltage > 1.0 else 1
    if args.list:
        print('\n'.join(list_ports()))
        return 0
    if args.kinds:
        print('\n'.join('%s %s' % pair for pair in kinds()))
        return 0
    if args.state:
        state = port_state(args.state, args.baud, args.unit)
        print('%s %s' % (args.state, state))
        return 0 if state == ANSWERED else 1
    if args.probe:
        ok = probe(args.probe, args.baud, args.unit)
        print('%s %s' % (args.probe, 'answered' if ok else 'silent'))
        return 0 if ok else 1
    if args.discover:
        device, kind = discover(args.preferred, args.baud, args.unit)
        if device:
            print('%s %s' % (device, kind))
            return 0
        print('none', file=sys.stderr)
        return 1
    if args.find:
        found = find(args.preferred, args.baud, args.unit)
        if found:
            print(found)
            return 0
        print('none', file=sys.stderr)
        return 1

    parser.print_help()
    return 2


if __name__ == '__main__':
    sys.exit(main())
