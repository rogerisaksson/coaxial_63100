"""The board's thermal observer, live: its three thermometers and the nodes it estimates.

The estimates come off the wire (0x6E device 8, the firmware's observer at
10 Hz), never recomputed here. The AFE is put back as found: on, it takes the
gate drivers' supply away; off, there is no NTC and the observer runs open on
power and time. `--switch` drives the load from inside this view - a real
board's with the break in circuit - and the gates go down in the same
`finally` that restores the screen.
"""
import argparse
import math
import sys
import time

from rich.text import Text

from coaxial import Coaxial63100
from coaxial.comm.session import standing
from coaxial.draw import cross_section, gauges
from coaxial.draw.thermalmap import CELL_ASPECT, SCALE_LINES, render
from coaxial.errors import NoReplyError, RigError
from coaxial.model import thermal
from coaxial.model.thermal import IDENT_MARGIN_FLOOR
from terminal.loader import TO_MENU
from terminal.ui import aspect as _aspect, screen as _screen
from terminal.ui.demo import cycle_motor, stop_motor
from terminal.ui.screen import (PORT, FPS_CAP, Feed, closing, mode_of, run_view, say, stamp_crosses,
                                visible)
from terminal.ui.stage import boot, frame_of, hud, stage
from terminal.views.thermal.boxes import room_hint, status_boxes

_screen.CHATTER = False     # the boot bar replaced the scroll

#: Rows the stage puts round the map: the title band, the viewport's two
#: edges, the crosses' gutter row.
HEAD_LINES = 4

#: Below the scale: the keys (TRAILING covers the blank above them).
FOOT_LINES = 1

#: Blank lines between the scale and the keys. Zero since 2026-08-30:
#: the row went to the board, which was asked a size up.
TRAILING = 0

#: Under the board: a blank and the evidence bar - `evidence_rows`.
#: Counted in the reserve, or the board is drawn two rows too tall and
#: its bottom edge goes under them.
GAUGE_LINES = 2
GAUGE_CELLS = 20


#: The page's load cycle, thermal s: 2 min at 30 A a phase rms, 4 idle - 36 wall s at HASTE
#: (bench 2026-09-06: "pulse a bit faster"): the stand-in's own `load_cycle`, an emulated
#: board's demo motor.
PAGE_CYCLE_ON_S, PAGE_CYCLE_OFF_S = 120.0, 240.0
PAGE_CYCLE_AMPS = 30.0 * math.sqrt(2.0)

#: The status panel's field width, map margin included. Fixed, so the map
#: never breathes when a number changes length.
PANEL_W = 42

#: How often the page asks for the identification, seconds. It moves once
#: a sample - every thirty seconds on the board.
IDENT_EVERY_S = 5.0


def evidence_class(level):
    """The bar's colour by how much of its span the model has earned:
    red below a third, yellow to two thirds, green above - red to
    yellow to green as it fills, the bench's ramp for it."""
    if level < 1.0 / 3.0:
        return cross_section.SOA_TRIP
    return cross_section.SOA_WARN if level < 2.0 / 3.0 else cross_section.SOA_OK


def evidence_rows(ident, colour=True):
    """The two rows under the board: a blank, then TH OBS and a bar of the
    span the model has earned - empty at the floor, full at the whole
    span, one minus the doubt - red to yellow to green as it fills.
    """
    if not ident:
        return ['', '   TH OBS -']
    floor = ident.get('margin_floor')
    floor = IDENT_MARGIN_FLOOR if floor is None else floor
    span = 1.0 - floor
    level = (ident['margin'] - floor) / span if span > 0.0 else 1.0
    level = max(0.0, min(1.0, level))
    cls = evidence_class(level)
    bar = gauges.bar(level, GAUGE_CELLS, cls=cls, colour=colour)
    # The label in the leaders' grey, constant; only the bar changes colour
    # (bench 2026-09-06).
    label = ('\x1b[38;5;%dmTH OBS\x1b[0m' % cross_section.LEADER_GREY) if colour \
        else 'TH OBS'
    return ['', '   %s %s' % (label, bar)]


def picture(state, console, reserve, aspect=CELL_ASPECT):
    """The board and its scale."""
    nodes = state['nodes']
    board_c = nodes.get('board')
    if board_c is None:
        return ['  the board sent no board node - device 8 is out of step']

    # Every node: `board` is the centre patch, and stripped of it - the bulk board's cut,
    # 2026-08-28 - the map's middle was its neighbours' mean, 9.6 K off (2026-10-05). The
    # leading blank moves the board one row down the frame - asked 2026-08-30, and counted
    # in the caller's reserve.
    return [''] + render(nodes, board_c=board_c, colour=console,
                         margin=PANEL_W, reserve=reserve,
                         trailing=TRAILING, aspect=aspect).split('\n')


#: PE15 up behind AFE_ON before the latch is cleared, s (tools/bench/switch.py).
BREAK_SETTLE_S = 0.4


def switch_on(rig, origin, load):
    """--switch: the stage armed, `load`'s duty on its legs. A real board's drivers have supply
    with AFE_ON off (R93 unmodified): off, PE15 up behind it, the latch cleared, armed with
    the break in circuit (docs/HARDWARE.md, 2026-10-05) - refused, the board's words are the
    RigError's. A demo board's are the schematic's, off the STO chain whose pilot detector
    runs off AFE_ON, and no pilot is laid: on, the chain bypassed."""
    bench = not _screen.demo(origin)
    if bench:
        rig.board.afe.off()
        time.sleep(BREAK_SETTLE_S)
        rig.gates.clear()
        rig.gates.on(bypass_sto=False, ignore_interlock=True)
    else:
        rig.board.afe.on()
        rig.gates.on(bypass_sto=True, ignore_interlock=True)
    rig.write(analog=load)
    say('warn', 'switching', '%s at %.0f %% - %s' % (
        '+'.join(name.split()[-1] for name in load), 100.0 * next(iter(load.values())),
        'AFE off, the break in circuit' if bench else 'AFE on, STO bypassed'))


def put_back(rig, load):
    """Undo what --switch armed, step by step, and say what each did."""
    done = []
    for name, what, undo in (
            ('duty', 'three legs to zero',
             lambda: rig.write(analog=dict.fromkeys(load, 0.0))),
            ('gate stage', 'disarmed, MOE clear', rig.gates.off)) if load is not None else ():
        try:
            undo()
            done.append((name, what))
        except (NoReplyError, RigError) as exc:
            done.append((name, 'FAILED: %s' % exc))
    return done


def afe_back(rig, was_on):
    """AFE_ON as the page found it - after the stage is down: the board refuses the rail
    under an armed one. What it did, for the closing list."""
    try:
        if rig.board.afe.is_on() != was_on:
            rig.board.afe.write(was_on)
        return [('AFE_ON', 'as found, %s' % ('on' if was_on else 'off'))]
    except (NoReplyError, RigError) as exc:
        return [('AFE_ON', 'FAILED: %s' % exc)]


def demo_load(rig, origin):
    """The page's load, laid on a demo board: the stand-in's own, the emulated MCU's
    demo motor; the per-frame step to run, or None."""
    motor = None
    if _screen.demo(origin) and not origin.real:
        # The stand-in: its own load, two model minutes at 30 A on all three legs and
        # four cooling - a turning motor's. The demo motor's held vector in its place
        # (569ae47) carried the current one leg at a time, 70 thermal s each at
        # 0.14 Hz, and the heat walked U, V, W round the board (2026-09-28).
        rig.thermal.load_cycle(on_s=PAGE_CYCLE_ON_S, off_s=PAGE_CYCLE_OFF_S)
        say('ok', 'load', '30 A a phase for %.0f s, %.0f s off'
            % (PAGE_CYCLE_ON_S / thermal.HASTE, PAGE_CYCLE_OFF_S / thermal.HASTE))
    elif _screen.demo(origin):
        # The emulated MCU: its thermometers read with the AFE on, and nothing to gate.
        rig.board.afe.on()
        say('ok', 'AFE_ON', 'on - the emulated board, its thermometers read')
        # A sample every 2 s of its time: at 30 the first came half a minute in, the
        # observer open loop till then (2026-09-28).
        rig.board.thermal.configure(sample_every_s=2.0)
        # The load: the demo motor's current through the legs, the regions pulsing on the
        # map and the bar under the board with cooldowns to rise on.
        on_s, off_s = PAGE_CYCLE_ON_S / thermal.HASTE, PAGE_CYCLE_OFF_S / thermal.HASTE
        motor = cycle_motor(rig, origin, on_s, off_s, PAGE_CYCLE_AMPS)
        say('ok', 'load', 'the demo motor, %.0f A for %.0f s, %.0f s off'
            % (PAGE_CYCLE_AMPS, on_s, off_s))
    else:
        say('ok', 'AFE_ON', 'left exactly as found - it gates the drivers')
    return motor


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', default=PORT)
    p.add_argument('--simulated', action='store_true')
    p.add_argument('--hz', type=float, default=FPS_CAP)
    p.add_argument('--frames', type=int, default=0,
                   help='stop after this many; 0 = until Q, ESC or Ctrl+C')
    p.add_argument('--switch', type=float, metavar='DUTY',
                   help='arm the gate drivers at this duty (0-1) and hold it '
                        'while drawing, so the zones have something to move')
    p.add_argument('-P', '--phases', default='U,V,W')
    p.add_argument('--cell-aspect', type=float, default=None,
                   help='how tall a character cell is against its width; '
                        'measured off the terminal when not given')
    a = p.parse_args()

    # power_afe=False: the AFE stays as found.
    with (boot('LINKING OBSERVER') as ready,
          Coaxial63100(port=a.port, execution_mode=mode_of(a), power_afe=False) as rig):
        ready()
        origin = rig.origin
        # On a demo board, however reached: the stand-in or an emulated board's world. Keyed
        # on the flag, the page ran without its load (bench 2026-09-06).
        if _screen.demo(origin):
            # The tour (temperate, cold, toasty, repeating), moved on once the identification
            # has earned the room.
            rig.thermal.situation('tour')
        say('ok' if origin.real else 'warn', 'link',
            '%s - %s' % (origin.label, standing(origin)))
        was_on = rig.board.afe.is_on()
        motor = demo_load(rig, origin)
        say('wait', 'drawing', 'Q closes it, ESC goes back to the menu')

        legs = [x.strip().upper() for x in a.phases.split(',')]
        if a.switch is not None and not set(legs) <= set('UVW'):
            p.error('--phases names legs of U, V, W: %s' % a.phases)
        load = None if a.switch is None else {'Phase ' + leg: a.switch for leg in legs}

        board_view = stage()
        console = board_view.is_terminal
        # The field's row aspect, half the character's, keeps the map round;
        # the character's is asked of the terminal, not assumed.
        aspect = _aspect.aspect_of(a.cell_aspect)

        period = 1.0 / max(a.hz, 0.2)
        # Everything in the frame that is not picture, so `render` can size the
        # board to what is left.
        reserve = (HEAD_LINES + 1 + SCALE_LINES + TRAILING + FOOT_LINES
                   + GAUGE_LINES)
        last = {'body': ['  waiting for device 8'], 'boxes': [], 'ident': None,
                'ident_at': float('-inf'), 'hint': None, 'quiet': None}
        leaving = None

        def sample():
            # A quiet link keeps the last good picture (FINDINGS): a blank
            # board each time made the view unreadable - and says why it stands. On the
            # feed's thread: the link need not hold a frame.
            try:
                if motor is not None:
                    motor()
                got = rig.board.thermal.state()
                # The identification moves once a sample, every thirty seconds
                # on the board: one round trip every few seconds is plenty, and
                # one a frame was a fifth of the frame. Stamped before it is asked: a
                # refusal is asked again in its period, not every sample.
                if time.monotonic() - last['ident_at'] > IDENT_EVERY_S:
                    last['ident_at'] = time.monotonic()
                    last['ident'] = rig.board.thermal.identification()
                    last['afe'] = rig.board.afe.state()['on']
                    # The hint with its hysteresis: what was shown stands until
                    # the room is well past a threshold.
                    last['hint'] = room_hint(last['ident'], last['hint'])
                elif last['ident'] and 'truth' in last['ident']:
                    # The stand-in's truth with every state - its own, no round trip: the
                    # load's row was 5 s old in a 12 s phase.
                    last['ident']['truth'] = rig.board.thermal.truth()
                last['boxes'] = status_boxes(got, rig.board.thermal.budget(),
                                             aspect, ident=last['ident'],
                                             hint=last['hint'],
                                             afe=last.get('afe'))
                last['body'] = picture(got, console, reserve,
                                       aspect[0] / 2.0)
                last['quiet'] = None
            except (NoReplyError, RigError) as exc:
                last['quiet'] = last['quiet'] or (time.monotonic(), str(exc))

        feed = Feed(sample, period=1.0 / FPS_CAP).start()

        def draw():
            # Three cells of pad and eight of field: six and twelve read as
            # dead air around the board.
            body = last['body']
            field = max((visible(l) for l in body), default=0) + 8
            art = stamp_crosses(['   ' + l for l in body], field)
            # The evidence bar under the board, after the crosses are stamped
            # so nothing lands on it.
            art += evidence_rows(last['ident'], colour=console)
            boxes = last['boxes']
            if last['quiet']:
                since, why = last['quiet']
                boxes = [hud('LINK', [Text('quiet %.0f s' % (time.monotonic() - since),
                                           style='alarm'), Text(why)])] + boxes
            return frame_of(board_view, origin, 'THERMAL OBSERVER',
                            '\n'.join(art), boxes,
                            (('Q', 'EXIT'), ('ESC', 'MENU')))

        done = []
        try:
            if load is not None:
                switch_on(rig, origin, load)
            leaving = run_view(board_view, console, period, a.frames, draw)
        except RigError as exc:
            done.append(('switching', 'FAILED: %s' % exc))
        finally:
            feed.stop()
            done += (put_back(rig, load) + (stop_motor(rig) if motor is not None else [])
                     + afe_back(rig, was_on))
            sys.stdout.write('\n')
            closing(done, console, 0)

    return TO_MENU if leaving == 'menu' else 0


if __name__ == '__main__':
    sys.exit(main())
