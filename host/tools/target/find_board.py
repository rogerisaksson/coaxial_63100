#!/usr/bin/env python3
"""Which COM port this board is on, from the command line, and the target voltage over SWD.

The voltage is the ST-Link's to say.

The probe itself is `coaxial.comm.ports`: one implementation, so "does this
port answer" cannot drift between the session opener, the broker, the
model's link_diagnose tool and this script, which board_chat/ComPort.ps1's
Test-BoardPort/Find-BoardPort call before a Python session even exists.
The runner's tool reads `port_state`, `probe` and `list_ports`
through this module, so a test can patch them here.

    python tools/target/find_board.py --list        # ports Windows sees
    python tools/target/find_board.py --probe COM4  # does the board answer here?
    python tools/target/find_board.py --find        # try every port, print the first that answers
    python tools/target/find_board.py --find --preferred COM4  # try that one first
    python tools/target/find_board.py --discover    # probes first; prints "COM4 probe"
    python tools/target/find_board.py --power       # target voltage over SWD (ST-Link), not USART3
"""
import argparse
import os
import re
import subprocess
import sys

# host/ on the path: this file's own directory's parent, so it does not matter
# what the working directory is or what any directory along the way is called.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from coaxial.comm.ports import (ANSWERED, discover, find, kinds, list_ports,  # noqa: E402
                                port_state, probe)

def _text(out):
    """Subprocess output as text, whichever way it came."""
    if isinstance(out, bytes):
        return out.decode('utf-8', errors='replace')
    return out or ''


def check_power(timeout=15):
    """(voltage_or_None, detail)."""

    from tools.target import build_and_flash   # lazy: build_and_flash imports _text from here

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
        # The voltage is printed in the first second; the run then spends the
        # rest on a second connect attempt at 8MHz.
        timed_out = True
        output = _text(exc.stdout) + _text(exc.stderr)
    match = re.search(r'Voltage\s*:\s*([\d.]+)\s*V', output)
    if match:
        return float(match.group(1)), output.strip()
    if timed_out:
        return None, 'STM32_Programmer_CLI did not answer within %ss' % timeout
    tail = output.strip().splitlines()[-1] if output.strip() else 'no output'
    return None, 'no voltage reading - %s' % tail


#: Target voltage over SWD above which the board counts as powered.
POWERED_V = 1.0


def _power_report():
    """The target voltage over SWD, printed; exit 0 when the board is
    powered."""
    voltage, detail = check_power()
    if voltage is None:
        print('unknown - %s' % detail)
        return 1
    print('%.2fV' % voltage)
    return 0 if voltage > POWERED_V else 1


def _reported(found, line):
    """What a search found, printed, exit 0 - or `none` on stderr,
    exit 1."""
    if not found:
        print('none', file=sys.stderr)
        return 1
    print(line)
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
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
        return _power_report()
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
        return _reported(device, '%s %s' % (device, kind))
    if args.find:
        found = find(args.preferred, args.baud, args.unit)
        return _reported(found, found)

    parser.print_help()
    return 2


if __name__ == '__main__':
    sys.exit(main())
