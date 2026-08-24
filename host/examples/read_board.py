#!/usr/bin/env python3
"""Read everything the board will tell you, and judge none of it.

    cd host && python examples/read_board.py

This is what a dumb slave looks like from the outside: it reports, and every
number here is a reading, not a verdict. There are no limits in this file, none
in the library and none in the firmware.

The one thing that does carry pass/fail is the board's self test, and only for
checks the board can settle from its own registers - a locked PLL, a calibration
that ran, a firmware checksum. Those need no external reference, so the board is
entitled to an opinion. Everything else it reports as a value.
"""
import os
import sys

# host/ on the path: this file's own directory's parent. Was '..'
# and '.', which only worked when run from host/ or host/examples -
# `python host/examples/read_board.py` from the repository root
# failed outright.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial import connect, disconnect


def show(board):
    print(board)

    print('\nversion')
    for key, value in board.system.version().items():
        print('  %-14s %s' % (key, value))

    print('\nclock')
    for key, value in board.system.clock().items():
        print('  %-14s %s' % (key, value))

    print('\nself test  (board judges only what it can prove about itself)')
    for check in board.system.self_test():
        marker = {'pass': 'ok  ', 'fail': 'FAIL', 'info': '    '}[check['status']]
        show = check['status'] == 'info' or check['value']
        print('  %s %-20s %s' % (marker, check['name'],
                                 check['value'] if show else ''))

    board.afe.enable()

    print('\nanalog  (raw codes and pin volts; scaling is nominal, not calibrated)')
    reading = board.analog.read_all(nr_of_samples=128, sample_rate=2000.0)
    print('  %d samples at %.0f Hz' % (reading['samples'], reading['rate_hz']))
    for channel in reading['channels']:
        print('    %-9s ADC%d IN%-2d %-12s %-4s mean %9.2f  %+8.4f V  '
              'noise %6.3f mV'
              % (channel['signal'] or '-', channel['adc'], channel['channel'],
                 channel['pin'], 'diff' if channel['differential'] else 'SE',
                 channel['mean_raw'], channel['volts_at_pin'],
                 channel['noise_volts_rms'] * 1000.0))

    print('\nnominal conversions  (nameplate component values, uncalibrated)')
    temperature = board.analog.ntc_temperature()
    print('  ntc      %.3f C   %.0f ohm   [%s]'
          % (temperature['celsius'], temperature['ohms'], temperature['params']))
    link = board.analog.dcbus_voltage()
    print('  dc link  %.4f V   pin %.4f V   x%.3f   [%s]'
          % (link['volts'], link['volts_at_pin'], link['scale'], link['params']))

    print('\nlink')
    for key, value in board.link.stats().items():
        print('  %-20s %s' % (key, value))


def main():
    boards = connect([(1, 115200)])
    try:
        for board in boards:
            show(board)
    finally:
        disconnect(boards)
    return 0


if __name__ == '__main__':
    sys.exit(main())
