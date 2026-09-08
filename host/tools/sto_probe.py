#!/usr/bin/env python3
"""The STO chain's two channels and the pump's pulse, a row a second.

    python tools/sto_probe.py --port COM4 --seconds 60
    python tools/sto_probe.py --simulated

The bench-day measurement of the STO chain in one command (docs/TODO.md):
Cinj, the recovered pilot tone, and Clevel, the integrator, as the
converters read them beside the interlock's want; the gate stage's own
keepalive count, differentiated into the pulses a second the charge pump
is fed, and the worst gap between them; the pilot and level in microvolts
where the board reports them (gate op 0, PROTOCOL), and the two flags the
board sets from its own thresholds. Run it with the tone absent, then
present, then with the circuit changed, and read the columns against the
scope.

Nothing here judges: invariant 10. The `want` column is what
`GateStage.interlock()` asks for, printed beside the reading so the
operator sees both; the `ok` columns are the board's flags, not this
tool's opinion.
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial import Coaxial63100                           # noqa: E402
from coaxial.errors import RigError                        # noqa: E402

#: The gate state's fields that speak of the chain, by the name the board
#: answers with and the name printed. A stand-in or an older firmware
#: leaves some out; a missing one prints as '-'.
FIELDS = (('keepalive', 'keepalive'), ('worst_gap_cycles', 'worst gap'),
          ('pilot_raw', 'pilot raw'), ('pilot_microvolts', 'pilot uV'),
          ('level_raw', 'level raw'), ('level_microvolts', 'level uV'),
          ('pilot_ok', 'pilot ok'), ('level_ok', 'level ok'))


def probe(rig, last=None):
    """One reading of the chain: the interlock's rows, the gate state's
    chain fields, and the keepalive pulses a second since `last`.

    `last` is the previous reading, for the rate; None the first time.
    Pure over the rig: reads, no writes, no verdict.
    """
    now = time.monotonic()
    rows = rig.gates.interlock()
    state = rig.board.gate_drivers.state()
    got = {'at': now,
           'channels': {name: (volts, want, ok) for name, volts, ok, want in rows},
           'fields': {name: state.get(name) for name, _ in FIELDS}}
    rate = None
    if last is not None and got['fields']['keepalive'] is not None \
            and last['fields'].get('keepalive') is not None:
        gap = now - last['at']
        if gap > 0:
            rate = (got['fields']['keepalive'] - last['fields']['keepalive']) / gap
    got['pulses_per_s'] = rate
    return got


def _cell(value, width):
    if value is None:
        text = '-'
    elif isinstance(value, bool):
        text = 'yes' if value else 'no'
    elif isinstance(value, float):
        text = '%.3f' % value
    else:
        text = str(value)
    return text.rjust(width)


def line(got, first=False):
    """One row of the table, and the header above the first."""
    names = [n for n in ('Cinj', 'Clevel') if n in got['channels']]
    head = ['     t/s'] + ['%9s' % n for n in names] + ['     want'] \
        + ['  pulses/s'] + ['%10s' % shown for _, shown in FIELDS]
    volts = [_cell(got['channels'][n][0], 9) for n in names]
    want = _cell(next((got['channels'][n][1] for n in names
                       if got['channels'][n][1] is not None), None), 9)
    cells = ['%8.1f' % got['t']] + volts + [want] \
        + [_cell(got['pulses_per_s'], 10)] \
        + [_cell(got['fields'][name], 10) for name, _ in FIELDS]
    text = ' '.join(cells)
    return (' '.join(head) + '\n' + text) if first else text


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--port', default='COM4')
    parser.add_argument('--simulated', action='store_true',
                        help='the stand-in, without probing for a board')
    parser.add_argument('--seconds', type=float, default=30.0,
                        help='how long to watch')
    parser.add_argument('--every', type=float, default=1.0,
                        help='seconds between rows')
    args = parser.parse_args(argv)

    rig = Coaxial63100(port=args.port, simulated_device=args.simulated,
                       power_afe=True).open()
    try:
        print('%s - %s' % (rig.origin.label, 'live' if rig.origin.real
                           else 'simulated'))
        began = time.monotonic()
        last = None
        first = True
        while True:
            got = probe(rig, last)
            got['t'] = time.monotonic() - began
            print(line(got, first))
            first = False
            last = got
            if got['t'] >= args.seconds:
                break
            time.sleep(args.every)
    except RigError as exc:
        print('the board refused: %s' % exc)
        return 1
    finally:
        rig.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
