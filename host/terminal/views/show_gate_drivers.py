#!/usr/bin/env python3
"""The gate drivers: what the six signals do, and what it costs.

    python terminal/views/show_gate_drivers.py --port COM4

The gate snapshot is one IDR read, so the six signals are the same
instant: six asks at 50 kHz can straddle an edge and show a leg with both
FETs on. The currents and DC link come from the acquisition task's
live accumulator, which carries a count, a lowest and a highest per
channel, so ripple is measured rather than inferred.

    + -     common duty, one step        [ ]     step size
    A       arm / disarm the stage       B       BKIN override
    I       interlock override           R       run and capture
    P       one pulse, U against V low   1 2 3 4 run length 1/10/100/1000 ms
    L       the master's pilot on / off  Q / ESC close / menu

Arming arms a power stage, and TIM1's 80 ns dead time is the only thing
between the two FETs of a leg: the 2EDL8034 has no interlock of its own.
The drivers' supply is the STO chain's. On the schematic's board - emulated
or simulated - the master's pilot heard with AFE_ON up releases it, and the
interlock is enforced; L cuts the pilot. The bench board is unmodified (R93
on +5): it supplies them with AFE_ON off, when the board refuses to convert,
so there switching and measuring are mutually exclusive and the interlock is
overridden. `--afe on|off` and `--interlock` override the board's way.

Nothing here judges a reading.
"""
import argparse
import sys
import time
from contextlib import suppress

from rich.text import Text

from coaxial.comm.session import standing
from coaxial.devices import scaling
from coaxial.errors import RigError
from coaxial.simulated.sto import PILOT_VOLTS
from terminal.loader import TO_MENU
from terminal.ui import screen as _screen
from terminal.ui.screen import (PORT, ASH, FPS_CAP, LABEL, SODIUM, closing, demo, open_rig, panel_width,
                                run_view, say, tint, mode_of)
from terminal.ui.stage import hud, panels_of, stage

_screen.CHATTER = False     # the boot bar replaced the scroll

#: What R runs for, in seconds. Two floors, both reported: the board takes
#: one conversion per main-loop turn, measured at 521 us for seven
#: channels, and start and stop are a round trip each at about 15 ms, so an
#: ask under about 30 ms is bounded by the link and not by the ask. The
#: stamps give the span that ran.
RUNS = {'1': 0.001, '2': 0.010, '3': 0.100, '4': 1.000}

STEPS = (0.001, 0.005, 0.01, 0.05, 0.10)

#: Legs, in the order the board reports its six gate signals.
LEGS = (('U', 'UH', 'UL'), ('V', 'VH', 'VL'), ('W', 'WH', 'WL'))


#: What P pulses: U at this duty against V held low, through the load
#: across them (2026-08-30: about 8 ohm, DC link read at 25 V, 3.1 A for
#: the on-time, 60 mA mean). It lasts until the second compare write
#: lands: 15.5 ms measured, ~780 cycles; the protocol has no cycle-counted
#: burst.
PULSE = 0.02


def gate_rows(state, width):
    """The six signals as one instant, and where the counter was beside it."""
    pins = state['pins']
    out = ['  gates - one IDR read, one instant          TIM1->CNT %d of %d'
           % (state['pins_at'], state['period'] - 1)]
    for name, high, low in LEGS:
        both = pins[high] and pins[low]
        # A lamp each, lit sodium and dark ash: which half conducts, at a
        # glance.
        def lamp(on):
            return tint('[#]', SODIUM) if on else tint('[ ]', ASH)
        out.append('    phase %s     H %s  L %s     %s'
                   % (name, lamp(pins[high]), lamp(pins[low]),
                      tint('BOTH ON - shoot through', SODIUM) if both
                      else (tint('off', ASH)
                            if not (pins[high] or pins[low]) else '')))
    return out


def analog_rows(live, layout, powered, refused, width, params=None, bench=False):
    """Mean and ripple per channel, converted, from the live accumulator."""
    if refused:
        why = (['  AFE_ON powers the converter reference, and on this bench',
                '  board the same pin gated the other way is what gives the',
                '  gate drivers their supply. Switching and measuring are',
                '  mutually exclusive here until that is patched. --afe on',
                '  runs it the other way: real currents, unpowered drivers.'] if bench
               else ['  AFE_ON powers the converter reference, and +5 with it: the',
                     '  STO chain\'s pilot detector, so the drivers are unpowered too.'])
        return [line[:width] for line in [
            '  no currents and no DC link: the board refused the task -',
            '  "%s"' % refused, ''] + [tint(line, LABEL) for line in why]]

    if not live or not live.get('mean'):
        return ['  no samples yet'[:width]]

    units = {f['signal']: (f['unit'], f['differential'])
             for f in layout['fields']}
    out = []
    for name in live['mean']:
        unit, differential = units.get(name, (None, False))
        convert = scaling.converter(unit, differential, signal=name,
                                    params=params)
        mean = convert(live['mean'][name])
        span = convert(live['highest'][name]) - convert(live['lowest'][name])
        out.append('  %-9s %+10.3f %-2s   p-p %8.3f   n %5d'
                   % (name, mean, scaling.symbol(unit, name),
                      abs(span), live['count'][name]))
    if not powered:
        out.append('  AFE_ON is off: it powers the ADC reference, so every')
        out.append('  channel above reads mid-scale and none of it is a')
        out.append('  measurement. It is also what gives the drivers supply'
                   if bench else '  measurement, and +5 with it: the STO chain is down.')
        if bench:
            out.append('  on this bench board.')
    return [line[:width] for line in out]


def sto_rows(state):
    """The STO chain as the board reads it: the recovered pilot and the pump's level against
    the interlock's, PE15, the drivers' supply."""
    rows = [('pilot', '%.2f V' % (state['pilot_microvolts'] / 1e6)),
            ('level', '%.2f V' % (state['level_microvolts'] / 1e6))]
    if state.get('nfault') is not None:
        rows += [('FAULTOUT', Text.from_ansi(tint('high', SODIUM) if state['nfault']
                                             else tint('low', ASH))),
                 ('+15V7', 'unread - AFE_ON off' if not state['afe_on']
                  else 'unread - the drive holds the converters' if state['vgate_mv'] is None
                  else '%.1f V' % (state['vgate_mv'] / 1e3))]
    return rows


def capture(rig, seconds, view):
    """Arm nothing, change nothing: run the task for `seconds` and drain it."""
    board = rig.board
    board.daq.stop()

    # Unlimited for the burst, which the board allows only because the run is
    # finite: `interval_us` 0 with `records` 0 is the combination that took the
    # link down, and it is the only one refused.
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
        batch = board.daq.acquire(layout=fresh)
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
    if rig.gates.is_on():
        rig.write(analog={'Phase U': view['duty'],
                          'Phase V': view['duty'],
                          'Phase W': view['duty']})
    return None


def _step(view, by):
    at = STEPS.index(view['step'])
    view['step'] = STEPS[min(max(at + by, 0), len(STEPS) - 1)]
    return None


def _arm(rig, view):
    if rig.gates.is_on():
        rig.gates.off(keep_bypass=True)
        return 'disarmed'
    rig.gates.on(ignore_interlock=view['override'])
    return 'armed at zero duty - all three low sides on'


def _bkin(rig):
    want = not rig.board.gate_drivers.state()['break_bypassed']
    rig.board.gate_drivers.configure(bypass_break=want)
    return ('BKIN overridden - the STO break input is disconnected'
            if want else 'BKIN back in circuit')


def _interlock(view):
    view['override'] = not view['override']
    return ('interlock overridden - Cinj and Clevel are not checked'
            if view['override'] else 'interlock back on')


def _pilot(rig, view):
    """The master's pilot cut or restored: an emulated or simulated bus's; a live one's is its
    master's own."""
    if not demo(rig.origin):
        return 'the pilot is the bus master\'s: nothing here sends one'
    view['pilot'] = not view['pilot']
    rig.pilot(PILOT_VOLTS if view['pilot'] else 0.0)
    return ('the master\'s pilot on - the STO chain releases in ~2 ms' if view['pilot']
            else 'the master\'s pilot cut - the STO chain trips, the break latches')


def _pulse(rig, view):
    """U at PULSE against V low, W low, for two writes, then every leg back
    to the common duty.
    """
    state = rig.board.gate_drivers.state()
    if not state['pwm_enabled']:
        return 'arm first - A - then P pulses'
    # Raw compare writes off one state read: rig.write()'s own arm check is a
    # 31 ms read the pulse would be spent waiting for.
    period = state['period'] - 1
    back = int(view['duty'] * period)
    rig.board.gate_drivers.write((int(PULSE * period), 0, 0))
    began = time.perf_counter()
    rig.board.gate_drivers.write((back, back, back))
    held = time.perf_counter() - began
    state = rig.board.gate_drivers.state()
    return ('pulsed U %.0f %% against V low: %.0f ms, ~%d cycles - break %s,'
            ' %d overruns'
            % (PULSE * 100.0, held * 1e3, int(held * 50000),
               'latched' if state['fault'] else 'clear', state['overruns']))


def act(rig, key, view):
    """One keypress."""
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
        if key in ('p', 'P'):
            return _pulse(rig, view)
        if key in ('l', 'L'):
            return _pilot(rig, view)
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
    """One frame on the stage: stage state, gates and currents as boxes."""


    state = view['gate_drivers']
    stage_box = hud('STAGE', [
        Text.from_ansi(tint('ARMED', SODIUM) if state['pwm_enabled']
                       else tint('idle', ASH)),
        ('break', 'OVERRIDDEN' if state['break_bypassed']
         else ('latched' if state['fault'] else 'clear')),
        ('dead time', '%d = %.1f ns' % (state['deadtime'],
                                        view['deadtime_ns'])),
        ('duty', '%.1f %%   step %.1f %%'
         % (view['duty'] * 100.0, view['step'] * 100.0))] + sto_rows(state))
    gates_box = hud('GATES', gate_rows(state, width))
    currents = hud('CURRENTS',
                   analog_rows(view.get('live'), layout, state['afe_on'],
                               view.get('refused'), width, view['scaling'], view['bench']))
    run_box = hud('RUN', run_rows(view, width))

    keys = [('+ -', 'DUTY'), ('[ ]', 'STEP'), ('A', 'ARM'), ('B', 'BKIN'),
            ('I', 'INTERLOCK %s' % ('OFF' if view['override'] else 'ON')),
            ('P', 'PULSE U-V'), ('L', 'PILOT %s' % ('ON' if view['pilot'] else 'OFF')),
            ('1-4', 'MS'), ('R', 'RUN'), ('Q', 'EXIT'), ('ESC', 'MENU')]
    if view.get('said'):
        keys.append(('', view['said']))
    return panels_of(console, origin, 'GATE DRIVERS',
                     [[stage_box, gates_box], [currents, run_box]], keys)


def parse_args(argv):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--port', default=PORT)
    parser.add_argument('--hz', type=float, default=FPS_CAP)
    parser.add_argument('--accumulate', type=int, default=8,
                        help='samples summed per record')
    parser.add_argument('--afe', choices=('on', 'off'), default=None,
                        help='AFE_ON, the board\'s way by default: on where the board '
                             'follows the schematic (emulated, simulated) - the STO '
                             'chain\'s pilot detector runs off it; off on the bench, '
                             'whose unmodified gate supplies the drivers only then')
    parser.add_argument('--interlock', action='store_true',
                        help='enforce the arming interlock on the bench too, whose '
                             'unmodified chain reads Cinj 0.77 V and Clevel 0.06 V; '
                             'enforced by default elsewhere. I toggles it in the view')
    parser.add_argument('--simulated', action='store_true')
    parser.add_argument('--frames', type=int, default=0)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)

    # power_afe=False so the rig changes nothing on the way in; this view sets
    # AFE_ON itself: its polarity decides what the run measures, and left as
    # found it would differ from day to day.
    rig = open_rig('LINKING GATE DRIVERS', port=args.port, power_afe=False,
                   execution_mode=mode_of(args))
    if rig is None:
        return 1
    origin, board = rig.origin, rig.board
    # The bench board is unmodified (R93 on +5); an emulated or simulated one is the schematic's.
    bench = not demo(origin)
    afe = (args.afe == 'on') if args.afe is not None else not bench
    was_on = board.afe.is_on()
    if afe != was_on:
        board.afe.write(afe)
        time.sleep(0.3)
    say('ok', 'AFE_ON', '%s - %s'
        % ('on' if afe else 'off',
           ('currents are real, drivers unpowered' if afe
            else 'drivers have supply, currents are not measurements') if bench
           else ('the STO chain\'s pilot detector up, currents real' if afe
                 else 'the STO chain down: drivers unpowered, currents not measurements')))
    say('ok' if origin.real else 'warn', 'link',
        '%s - %s' % (origin.label, standing(origin)))

    try:
        state = rig.gates.check()
    except RigError as exc:
        say('fail', 'dead time', str(exc))
        rig.close()
        return 1
    say('ok', 'dead time', 'BDTR DTG %d, and the 2EDL8034 has no interlock '
        'of its own' % state['deadtime'])

    # The board refuses to convert with AFE_ON off, because that pin powers the
    # reference (invariant 9).
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
            'layout': layout, 'override': bench and not args.interlock,
            'bench': bench, 'pilot': True,
            'deadtime_ns': state['deadtime'] * 1e9
                           / (2.0 * (state['period'] - 1) * 50000.0),
            'scaling': board.analog.scaling()}


    board_view = stage()
    terminal = board_view.is_terminal
    leaving = None

    def draw():
        with suppress(RigError):
            view['gate_drivers'] = board.gate_drivers.state()
            if not refused:
                view['live'] = rig.latest(block=False)
        return compose(rig, origin, board_view, view, layout, panel_width())

    def on_input(typed, _moved):
        for key in typed:
            view['said'] = act(rig, key, view) or view['said']

    try:
        leaving = run_view(board_view, terminal, 1.0 / max(args.hz, 0.5),
                           args.frames, draw, on_input)
    finally:
        done = []
        try:
            if not refused:
                rig.stop()
                done.append(('acquisition', 'task stopped'))
            rig.gates.off()
            done.append(('gate stage', 'disarmed, MOE clear'))
            done.append(('BKIN', 'back in circuit'))
            if not view['pilot']:
                rig.pilot(PILOT_VOLTS)
                done.append(('pilot', 'the master\'s back on'))
            if board.afe.is_on() != was_on:
                board.afe.write(was_on)
            done.append(('AFE_ON', 'back the way it was found'))
        except RigError as exc:
            done.append(('putting it back', 'FAILED: %s' % exc))
        rig.close()
        sys.stdout.write('\n')
        closing(done, terminal, 0)

    return TO_MENU if leaving == 'menu' else 0


if __name__ == '__main__':
    sys.exit(main())
