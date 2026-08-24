#!/usr/bin/env python3
"""Which COM port this board is actually on - the one implementation both
sides of this project call into, so "does this port answer" cannot drift
between them:

  * host/coaxial_ollama/tools.py's link_diagnose tool - the model, mid-
    session, when a call has already failed.
  * board_prompt/ComPort.ps1's Test-BoardPort/Find-BoardPort -
    -AutodetectComport, before a Python session even exists.

Going through coaxial.connect() is the same Modbus round trip a real session
makes, so a wrong port fails for the same reason it would fail a moment
later inside dbg.py itself - not a weaker, different check that passes here.

    python tools/find_board.py --list                 # ports Windows sees
    python tools/find_board.py --probe COM4           # does the board answer here?
    python tools/find_board.py --find                 # try every port, print the first that answers
    python tools/find_board.py --find --preferred COM4   # try that one first
    python tools/find_board.py --power                # target voltage over SWD - the ST-Link, not USART3
"""
import argparse
import sys

sys.path.insert(0, __file__.rsplit('tools', 1)[0])


def check_power(timeout=15):
    """(voltage_or_None, detail). Whether the ST-Link senses power on the
    target at all - the most fundamental check there is, and one the serial
    side cannot make on its own: a board with no target voltage is never
    going to answer over USART3, whatever the COM port list says. Measured
    live on this bench: an unplugged ST-Link cable read `Voltage: 0.00V`
    from STM32_Programmer_CLI, with the serial side reporting nothing more
    specific than silence.

    Runs a bare SWD connect, no flash - `build_and_flash.py`'s own
    toolchain-path resolution finds the programmer, since it is not on PATH
    unless env.ps1 has already run in this shell.

    voltage is None when the programmer could not be found or did not
    answer in time - not the same as 0.00V, which is a real reading that
    says the target has none.
    """
    import re
    import subprocess

    import build_and_flash

    path = build_and_flash.toolchain_path()
    programmer = build_and_flash.find_programmer(path)
    if programmer is None:
        return None, 'STM32_Programmer_CLI not found - see setup.ps1'

    try:
        done = subprocess.run([programmer, '-c', 'port=SWD', 'mode=UR', '-q'],
                              capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return None, 'STM32_Programmer_CLI did not answer within %ss' % timeout

    output = (done.stdout or '') + (done.stderr or '')
    match = re.search(r'Voltage\s*:\s*([\d.]+)\s*V', output)
    if not match:
        tail = output.strip().splitlines()[-1] if output.strip() else 'no output'
        return None, 'no voltage reading - %s' % tail
    return float(match.group(1)), output.strip()


def list_ports():
    import serial.tools.list_ports
    return [p.device for p in serial.tools.list_ports.comports()]


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
    parser.add_argument('--probe', metavar='PORT',
                        help='does the board answer on this one port')
    parser.add_argument('--find', action='store_true',
                        help='try every port, print the first that answers')
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
    if args.probe:
        ok = probe(args.probe, args.baud, args.unit)
        print('%s %s' % (args.probe, 'answered' if ok else 'silent'))
        return 0 if ok else 1
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
