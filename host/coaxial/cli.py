"""Command line front end: python -m coaxial <command>.

One function per command, dispatched from a table. Each takes the board and the
parsed arguments and prints; none of them decides anything, so the library stays
the only place behaviour lives.
"""
import argparse
import sys

from .board import connect, disconnect
from .errors import RigError
from .gpio import reserved_reason


def cmd_version(board, args):
    for key, value in board.system.version().items():
        print('  %-12s %s' % (key, value))


def cmd_clock(board, args):
    for key, value in board.system.clock().items():
        print('  %-14s %s' % (key, value))


def cmd_table(board, args):
    """The board's ADC wiring. Metadata only - 'analog' and 'scan' read it."""
    print('  %-3s %-4s %-4s %-12s %-5s %-9s %-10s' %
          ('idx', 'adc', 'ch', 'pin', 'mode', 'signal', 'unit'))
    for c in board.analog.channels():
        print('  %-3d %-4d %-4d %-12s %-5s %-9s %-10s' % (
            c['index'], c['adc'], c['channel'], c['pin'],
            'diff' if c['differential'] else 'SE', c['signal'] or '-',
            c['unit'] or '-'))


def cmd_afe(board, args):
    action = (args.args[0] if args.args else 'read').lower()
    handler = {'on': board.afe.enable, 'off': board.afe.disable,
               'toggle': board.afe.toggle, 'read': board.afe.state}
    if action not in handler:
        raise SystemExit('afe: expected on, off, toggle or read')
    print('  afe %s -> %s' % (action, handler[action]()))


def cmd_temp(board, args):
    board.afe.enable()
    t = board.analog.ntc_temperature()
    print('  %.3f C   %.0f ohm   spread %.0f mK over %d samples   [%s]'
          % (t['celsius'], t['ohms'], t['spread_millikelvin'], t['samples'],
             t['params']))


def cmd_dcbus(board, args):
    board.afe.enable()
    v = board.analog.dcbus_voltage()
    print('  %.4f V   pin %.4f V   x%.3f   ripple %.1f mV   noise %.1f mV rms'
          % (v['volts'], v['volts_at_pin'], v['scale'],
             v['ripple_volts'] * 1000.0, v['noise_volts_rms'] * 1000.0))


def cmd_analog(board, args):
    board.afe.enable()
    samples = int(args.args[0]) if args.args else 64
    rate = float(args.args[1]) if len(args.args) > 1 else 2000.0
    result = board.analog.read_all(samples, rate)
    print('  %d samples at %.0f Hz actual' % (result['samples'], result['rate_hz']))
    for c in result['channels']:
        print('    %-9s ADC%d IN%-2d %-12s %-4s mean %9.2f  %+8.4f V  '
              'noise %6.2f mV'
              % (c['signal'] or '-', c['adc'], c['channel'], c['pin'],
                 'diff' if c['differential'] else 'SE', c['mean_raw'],
                 c['volts_at_pin'], c['noise_volts_rms'] * 1000.0))


def cmd_scan(board, args):
    # Like the other reading commands: the scan refuses with the front end off,
    # because mid-scale would come back as exactly 25.00 C.
    board.afe.enable()
    for key, value in board.analog.scan().items():
        print('  %-16s %s' % (key, value))


def cmd_echo(board, args):
    message = ' '.join(args.args) or 'Hello slave!'
    print('  %r' % board.link.echo(message))


def cmd_stats(board, args):
    for key, value in board.link.stats().items():
        print('  %-20s %s' % (key, value))


def cmd_pins(board, args):
    """Read a whole port, annotating the pins the firmware will not let go."""
    port = (args.args[0] if args.args else 'E').upper()
    value = board.gpio.port_read(port)
    print('  GPIO%s IDR = 0x%04X' % (port, value))
    for pin in range(15, -1, -1):
        reason = reserved_reason(port, pin)
        print('    P%s%-2d %d%s' % (port, pin, (value >> pin) & 1,
                                    '   reserved: ' + reason if reason else ''))


COMMANDS = {
    'version': cmd_version, 'clock': cmd_clock, 'table': cmd_table,
    'afe': cmd_afe, 'temp': cmd_temp, 'dcbus': cmd_dcbus,
    'analog': cmd_analog, 'scan': cmd_scan, 'echo': cmd_echo,
    'stats': cmd_stats, 'pins': cmd_pins,
}


def main(argv=None):
    parser = argparse.ArgumentParser(prog='python -m coaxial',
                                     description=__doc__.splitlines()[0])
    parser.add_argument('command', choices=sorted(COMMANDS) + ['all'])
    parser.add_argument('args', nargs='*')
    parser.add_argument('--port', default='COM4')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--unit', type=int, default=1)
    parser.add_argument('--keep', action='store_true',
                        help='leave the board in binary mode')
    args = parser.parse_args(argv)

    try:
        boards = connect([(args.unit, args.baud, args.port)])
    except RigError as exc:
        print('connect failed: %s' % exc, file=sys.stderr)
        return 1

    board = boards[0]
    names = sorted(COMMANDS) if args.command == 'all' else [args.command]

    try:
        for name in names:
            print('%s:' % name)
            COMMANDS[name](board, args)
    except RigError as exc:
        print('%s: %s' % (type(exc).__name__, exc), file=sys.stderr)
        return 1
    finally:
        if args.keep:
            boards[0].transport.close()
        else:
            disconnect(boards)

    return 0
