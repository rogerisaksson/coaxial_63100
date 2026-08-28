#!/usr/bin/env python3
"""The gate drivers: what the six signals do, and what it costs.

    python tools/show_gate_drivers.py --port COM4

The **gate snapshot** is one IDR read, so the six signals are the same
instant: six asks at 50 kHz can straddle an edge and show a leg with both
FETs on. The **currents** and **DC link** come from the acquisition task's
live accumulator, which carries a count, a lowest and a highest per
channel, so ripple is measured rather than inferred.

    + -     common duty, one step        [ ]     step size
    A       arm / disarm the stage       B       BKIN override
    I       interlock override           R       run and capture
    1 2 3 4 run length 1/10/100/1000 ms  Q / ESC close / menu

**Arming arms a power stage**, and TIM1's 80 ns dead time is the only thing
between the two FETs of a leg - the 2EDL8034 has no interlock of its own.
On this bench board AFE_ON is inverted, so the drivers have supply while it
is off, and with it off the board refuses to convert: switching and
measuring are mutually exclusive here. `--afe` runs it the other way.

Nothing here judges a reading.
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial import Coaxial63100, scaling                   # noqa: E402
from coaxial.errors import RigError                         # noqa: E402
from screen import TO_MENU, Keys, banner, clear, paint, say  # noqa: E402

#: What R runs for, in seconds. Two floors, and the view reports both
#: rather than hiding either: the board takes one conversion per main-loop
#: turn, measured at 521 us for seven channels, and start and stop are a
#: round trip each at about 15 ms - so an ask under about 30 ms is bounded
#: by the link and not by the ask. The stamps say what actually happened.
RUNS = {'1': 0.001, '2': 0.010, '3': 0.100, '4': 1.000}

STEPS = (0.001, 0.005, 0.01, 0.05, 0.10)

#: Legs, in the order the board reports its six gate signals.
LEGS = (('U', 'UH', 'UL'), ('V', 'VH', 'VL'), ('W', 'WH', 'WL'))


def gate_rows(state, width):
    """The six signals as one instant, and where the counter was beside it.

    A leg showing both on would be a shoot-through, and it is drawn as such
    rather than left for the reader to spot in a row of ones and zeros.
    """
    pins = state['pins']
    out = ['  gates          high  low          TIM1->CNT %d of %d'
           % (state['pins_at'], state['period'] - 1)]
    for name, high, low in LEGS:
        both = pins[high] and pins[low]
        out.append('    phase %s      %s     %s      %s'
                   % (name,
                      '1' if pins[high] else '0',
                      '1' if pins[low] else '0',
                      'BOTH ON - shoot through' if both
                      else ('off' if not (pins[high] or pins[low]) else '')))
    return [line[:width] for line in out]


def analog_rows(live, layout, powered, refused, width):
    """Mean and ripple per channel, converted, from the live accumulator.

    Ripple is `highest - lowest` over the accumulator's own window, which is
    every sample the board took rather than the ones that fitted in a
    record. It is a span, not an RMS, and it is named `p-p` for that reason.
    """
    if refused:
        return [line[:width] for line in [
            '  no currents and no DC link: the board refused the task -',
            '  "%s"' % refused,
            '',
            '  AFE_ON powers the converter reference, and on this bench',
            '  board the same pin gated the other way is what gives the',
            '  gate drivers their supply. Switching and measuring are',
            '  mutually exclusive here until that is patched. --afe runs it',
            '  the other way: real currents, unpowered drivers.']]

    if not live or not live.get('mean'):
        return ['  no samples yet'[:width]]

    units = {f['signal']: (f['unit'], f['differential'])
             for f in layout['fields']}
    out = []
    for name in live['mean']:
        unit, differential = units.get(name, (None, False))
        convert = scaling.converter(unit, differential, signal=name)
        mean = convert(live['mean'][name])
        span = convert(live['highest'][name]) - convert(live['lowest'][name])
        out.append('  %-9s %+10.3f %-2s   p-p %8.3f   n %5d'
                   % (name, mean, scaling.UNIT_SYMBOL.get(unit, ''),
                      abs(span), live['count'][name]))
    if not powered:
        out.append('  AFE_ON is off: it powers the ADC reference, so every')
        out.append('  channel above reads mid-scale and none of it is a')
        out.append('  measurement. It is also what gives the drivers supply')
        out.append('  on this bench board.')
    return [line[:width] for line in out]


def capture(rig, seconds, view):
    """Arm nothing, change nothing: run the task for `seconds` and drain it.

    The gate drivers is left exactly as the operator set it. A run that armed the
    stage itself would be a second way to arm one, and there is deliberately
    only one.
    """
    board = rig.board
    board.daq.stop()

    # Unlimited for the burst, which the board allows only because the run
    # is finite: `interval_us` 0 with `records` 0 is the combination that
    # took the link down, and it is the only one refused. A 16 KB buffer
    # holds 512 records at seven channels, so the ask is capped there -
    # asking for more would fill it and start dropping, and a run that
    # drops has a hole nothing in it says the size of.
    holds = max(1, 16384 // max(1, view['layout']['stride']))
    fresh = board.daq.configure(
        [f['signal'] for f in view['layout']['fields']],
        digital=False, accumulate=1, records=holds, interval_us=0)

    board.daq.start()
    started = time.time()
    time.sleep(seconds)
    board.daq.stop()

    view['layout'] = fresh
    records, tries = [], 0
    # Drained after stopping rather than during: a read costs about 15 ms of
    # round trip and a 1 ms run would otherwise be mostly the reading of it.
    while tries < 400:
        batch = board.daq.read(layout=fresh)
        tries += 1
        if not batch:
            break
        records.extend(batch)

    state = board.daq.state()
    view['run'] = {
        'seconds': seconds, 'records': len(records),
        'dropped': state['dropped'], 'produced': state['produced'],
        'wall': time.time() - started,
        'holds': holds,
        'first': records[0] if records else None,
        'last': records[-1] if records else None,
    }
    return records


def run_rows(view, width):
    """What the last run collected. Counts, not conclusions."""
    got = view.get('run')
    if not got:
        return ['  no run yet - 1 2 3 4 pick a length, R runs it'[:width]]

    span = 0.0
    if got['first'] and got['last']:
        span = (got['last']['at'] - got['first']['at']) / 475e6
    rate = got['records'] / span if span else 0.0
    out = ['  last run %.0f ms asked, %.0f ms wall   %d records to the host'
           % (got['seconds'] * 1e3, got['wall'] * 1e3, got['records']),
           '  board produced %d, dropped %d, buffer holds %d   '
           '%.0f records/s while it ran'
           % (got['produced'], got['dropped'], got['holds'], rate)]
    if span:
        out.append('  stamps span %.3f ms, so %.1f us between records'
                   % (span * 1e3,
                      span * 1e6 / max(1, got['records'] - 1)))
    if span > got['seconds'] * 2.0:
        out.append('  the ask is shorter than a round trip. Start and stop '
                   'are 15 ms each,')
        out.append('  so the span is what happened and the ask is what was '
                   'wanted.')
    if got['dropped']:
        out.append('  the buffer filled: %d records were produced with '
                   'nowhere to go, and' % got['dropped'])
        out.append('  the capture has a hole nothing in it says the size of.')
    return [line[:width] for line in out]


def _duty(rig, view, by):
    view['duty'] = min(1.0, max(0.0, view['duty'] + by))
    if rig.gates.armed():
        rig.write(analog={'Phase U': view['duty'],
                          'Phase V': view['duty'],
                          'Phase W': view['duty']})
    return None


def _step(view, by):
    at = STEPS.index(view['step'])
    view['step'] = STEPS[min(max(at + by, 0), len(STEPS) - 1)]
    return None


def _arm(rig, view):
    if rig.gates.armed():
        rig.gates.disarm(keep_bypass=True)
        return 'disarmed'
    rig.gates.arm(ignore_interlock=view['override'])
    return 'armed at zero duty - all three low sides on'


def _bkin(rig):
    want = not rig.board.gate_drivers.state()['break_bypassed']
    rig.board.gate_drivers.bypass_break(want)
    return ('BKIN overridden - the STO break input is disconnected'
            if want else 'BKIN back in circuit')


def _interlock(view):
    view['override'] = not view['override']
    return ('interlock overridden - Cinj and Clevel are not checked'
            if view['override'] else 'interlock back on')


def act(rig, key, view):
    """One keypress. Returns a line to show, or None.

    Every refusal here is the board's own sentence: the host validates only
    what stops a request being formed.
    """
    try:
        if key in ('+', '='):
            return _duty(rig, view, view['step'])
        if key in ('-', '_'):
            return _duty(rig, view, -view['step'])
        if key == ']':
            return _step(view, 1)
        if key == '[':
            return _step(view, -1)
        if key in ('a', 'A'):
            return _arm(rig, view)
        if key in ('b', 'B'):
            return _bkin(rig)
        if key in ('i', 'I'):
            return _interlock(view)
        if key in RUNS:
            view['seconds'] = RUNS[key]
            return 'run length %.0f ms' % (view['seconds'] * 1e3)
        if key in ('r', 'R'):
            capture(rig, view['seconds'], view)
            return 'ran %.0f ms' % (view['seconds'] * 1e3)
        return None
    except RigError as exc:
        return str(exc)


def compose(rig, origin, console, view, layout, width):
    """The whole frame."""
    state = view['gate_drivers']
    lines = [banner(origin, 'gate_drivers', console,
                    'Q closes, ESC for the menu'), '']
    lines.append(' GATEDRIVERS  %-8s  break %-10s  dead time %d = %.1f ns  '
                 'duty %.1f%% step %.1f%%'
                 % ('ARMED' if state['pwm_enabled'] else 'idle',
                    'OVERRIDDEN' if state['break_bypassed']
                    else ('latched' if state['fault'] else 'clear'),
                    state['deadtime'], view['deadtime_ns'],
                    view['duty'] * 100.0, view['step'] * 100.0))
    lines.append(' ' + '-' * max(10, width - 2))
    lines += gate_rows(state, width)
    lines.append(' ' + '-' * max(10, width - 2))
    lines += analog_rows(view.get('live'), layout, state['afe_on'],
                         view.get('refused'), width)
    lines.append(' ' + '-' * max(10, width - 2))
    lines += run_rows(view, width)
    lines += ['', ('  + -  duty   [ ]  step   A  arm   B  BKIN override   '
                   'I  interlock %s   1 2 3 4  ms   R  run'
                   % ('OFF' if view['override'] else 'on'))[:width]]
    if view.get('said'):
        lines += ['', ('  ' + view['said'])[:width]]
    return lines


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--port', default='COM4')
    parser.add_argument('--hz', type=float, default=8.0)
    parser.add_argument('--accumulate', type=int, default=8,
                        help='samples summed per record')
    parser.add_argument('--afe', action='store_true',
                        help='switch AFE_ON on, which makes the currents real '
                             'and - on this bench board, whose gate is '
                             'inverted - takes the supply off the drivers. '
                             'Without it AFE_ON goes off, the drivers have '
                             'power, and the currents are not measurements')
    parser.add_argument('--interlock', action='store_true',
                        help='enforce the arming interlock. Off by default '
                             'because this bench board is unmodified: Cinj '
                             'reads 0.77 V and Clevel 0.06 V against the 3 V '
                             'each wants. I toggles it in the view')
    parser.add_argument('--simulated', action='store_true')
    parser.add_argument('--frames', type=int, default=0)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)

    # power_afe=False so the rig changes nothing on the way in; this view
    # sets it itself, because which way round it goes is the whole question
    # here and leaving it as found makes the run mean different things on
    # different days.
    rig = Coaxial63100(port=args.port, power_afe=False,
                       simulated_device=bool(args.simulated)).open()
    origin, board = rig.origin, rig.board
    was_on = board.afe.is_on()
    if args.afe != was_on:
        board.afe.enable() if args.afe else board.afe.disable()
        time.sleep(0.3)
    say('ok', 'AFE_ON', '%s - %s'
        % ('on' if args.afe else 'off',
           'currents are real, drivers unpowered' if args.afe
           else 'drivers have supply, currents are not measurements'))
    say('ok' if origin.real else 'warn', 'link',
        '%s - %s' % (origin.label, 'live' if origin.real else 'simulated'))

    try:
        state = rig.gates.check()
    except RigError as exc:
        say('fail', 'dead time', str(exc))
        rig.close()
        return 1
    say('ok', 'dead time', 'BDTR DTG %d, and the 2EDL8034 has no interlock '
        'of its own' % state['deadtime'])

    # The board refuses to convert with AFE_ON off, because that pin powers
    # the reference (invariant 9). On this bench board the same pin, gated
    # the other way round, is what gives the drivers supply - so switching
    # and measuring are mutually exclusive here until the patch. The view
    # runs either way and says which half it has.
    layout, refused = rig.configure(accumulate=args.accumulate,
                                    digital=False), None
    try:
        rig.start()
        say('ok', 'task', '%d channels, stride %d'
            % (len(layout['fields']), layout['stride']))
    except RigError as exc:
        refused = str(exc)
        say('warn', 'task', '%s' % refused)

    view = {'duty': 0.0, 'step': 0.01, 'seconds': RUNS['3'], 'said': '',
            'gate_drivers': board.gate_drivers.state(), 'live': None, 'refused': refused,
            'layout': layout, 'override': not args.interlock,
            'deadtime_ns': state['deadtime'] * 1e9
                           / (2.0 * (state['period'] - 1) * 50000.0)}

    console = sys.stdout.isatty()
    if console and os.name == 'nt':
        os.system('')
    shown, leaving, frame = [], None, 0

    try:
        with Keys(console) as keys:
            while True:
                try:
                    view['gate_drivers'] = board.gate_drivers.state()
                    if not refused:
                        view['live'] = rig.latest(block=False)
                except RigError:
                    pass                    # a missed reply is a missed frame

                width = shutil_width()
                lines = compose(rig, origin, console, view, layout, width)
                sys.stdout.write(paint(shown, lines, console))
                sys.stdout.flush()
                shown = lines
                frame += 1
                if args.frames and frame >= args.frames:
                    break

                leaving, _ = keys.poll()
                if leaving:
                    break
                for key in keys.taken():
                    said = act(rig, key, view)
                    if said:
                        view['said'] = said
                time.sleep(1.0 / max(args.hz, 0.5))
    except KeyboardInterrupt:
        pass
    finally:
        clear(console)
        try:
            if not refused:
                rig.stop()
            rig.gates.disarm()
            if board.afe.is_on() != was_on:
                board.afe.enable() if was_on else board.afe.disable()
            say('ok', 'gate_drivers', 'disarmed, BKIN back in circuit, AFE_ON as '
                                'it was found')
        except RigError as exc:
            say('fail', 'putting it back', str(exc))
        rig.close()

    return TO_MENU if leaving == 'menu' else 0


def shutil_width():
    try:
        return max(60, os.get_terminal_size().columns - 2)
    except OSError:
        return 100


if __name__ == '__main__':
    sys.exit(main())
