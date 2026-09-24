"""The board's thermal observer, live: one measurement and five estimates.

The estimates come off the wire (0x6E device 8, the firmware's observer at
10 Hz), never recomputed here. The AFE stays as found: on, it takes the gate
drivers' supply away; off, there is no NTC and the observer runs open on power
and time (`open for`). `--switch` drives the load from inside this view - the
port is exclusive, so `switch.py` cannot run beside it - and the gates go down
in the same `finally` that restores the screen.
"""
import argparse
import sys
import time
from contextlib import suppress

from rich.text import Text

from coaxial import Coaxial63100
from coaxial.draw import cross_section, gauges
from coaxial.draw.thermalmap import CELL_ASPECT, MARKS, SCALE_LINES, render
from coaxial.errors import NoReplyError, RigError
from coaxial.kalman import thermal_ident
from coaxial.model.thermal import ALL_NODES, IDENT_MARGIN_FLOOR, pretty
from terminal.loader import TO_MENU
from terminal.ui import aspect as _aspect, screen as _screen
from terminal.ui.screen import closing, run_view, say, stamp_crosses, visible
from terminal.ui.stage import boot, frame_of, hud, stage

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


#: The page's load cycle, model s: 2 min at 30 A, 4 idle - 36 wall s at HASTE
#: (bench 2026-09-06: "pulse a bit faster"). Measured in a box: driver U
#: 71-109 C, CONVERGING by minute 4, margin 0.98 by 18; STABLE wants the
#: walk's longer cooldowns.
PAGE_CYCLE_ON_S, PAGE_CYCLE_OFF_S = 120.0, 240.0

#: The room hint above the board, on the estimated ambient (op 10): cold
#: under 5 C, hot from 35, 'unsure' while the filtered innovation is 0.3 K or
#: more (UNCERTAIN's ratio on this board's 0.1). Emoji on the bench's word
#: (2026-09-06, after braille pictograms), two cells at the terminal's size:
#: no escape scales a glyph (DECDHL is undone in Windows Terminal and
#: xterm.js).
ROOM_COLD_C, ROOM_HOT_C = 5.0, 35.0
ROOM_UNSURE_K = 0.3
#: Hysteresis, so a boundary room keeps its word (bench 2026-09-06): a held
#: word stands until the room is this far past its threshold, and 'unsure'
#: until the innovation is under ROOM_SURE_K.
ROOM_HYSTERESIS_K = 2.0
ROOM_SURE_K = 0.2
#: Each hint is two single-code-point emoji with a space between (bench):
#: the snowflake and thermometer asked for are narrow characters with a
#: variation selector, counted one cell and drawn two, and the SENSE frame
#: shifted (2026-09-06).
ROOM_HINTS = {'cold': '🧊 🥶', 'mild': '🍃 😌', 'hot': '🔥 🥵',
              'unsure': '🤒 🤔'}

def room_hint(ident, held=None):
    """Which hint the estimate gets - 'unsure' while the innovation is
    large, else 'cold', 'mild', 'hot' on the room - or nothing before the
    board has answered op 10 with a room (MINOR 15).
    """
    room = (ident or {}).get('ambient')
    if room is None:
        return ''
    innovation = ident.get('innovation_k', 0.0)
    if innovation >= (ROOM_SURE_K if held == 'unsure' else ROOM_UNSURE_K):
        return 'unsure'
    band = ROOM_HYSTERESIS_K
    if held == 'cold' and room < ROOM_COLD_C + band:
        return 'cold'
    if held == 'hot' and room >= ROOM_HOT_C - band:
        return 'hot'
    if held == 'mild' and ROOM_COLD_C - band <= room < ROOM_HOT_C + band:
        return 'mild'
    if room < ROOM_COLD_C:
        return 'cold'
    return 'mild' if room < ROOM_HOT_C else 'hot'


#: The status panel's field width, map margin included. Fixed, so the map
#: never breathes when a number changes length.
PANEL_W = 42

#: The soak bar's width in cells: the spend against the worst node's
#: ceiling, the one level on the page that is not a temperature.
SOAK_CELLS = 16


def soak(budget):
    """The HEADROOM box's row: the spend as a solid bar in the margin's
    colour with an orange tip, the figure beside it.
    """

    used = budget['worst']
    line = gauges.bar(used, SOAK_CELLS,
                      cls=gauges.margin_class(used, budget['tripped']))
    return [('soak', Text.from_ansi('%s  %3.0f %%' % (line, 100.0 * used)))]


#: How the identification's state is worn in SENSE: a chip in the colour
#: the policy deserves - the margin it trims the ceilings by is beside it.
IDENT_STYLE = {'STABLE': 'chip.live', 'CONVERGING': 'chip.sim',
               'UNCERTAIN': 'alarm'}

#: How often the page asks for the identification, seconds. It moves once
#: a sample - every thirty seconds on the board.
IDENT_EVERY_S = 5.0


def ident_rows(ident, hint=None):
    """The identification in SENSE, one fact a row: the state as a chip, the
    margin the envelope acts on and the floor it rose from, each online
    scale with its sigma, the room, and on the stand-in the truth and the
    load on it.
    """

    state = ident['state']
    rows: list = [('model', Text(' %s ' % state, IDENT_STYLE.get(state, 'value')))]
    # The margin, the floor and the innovation are HEADROOM's rows
    # (`envelope_rows`).
    scales, sigma = ident['scales'], ident['sigma']
    for name in ident['online']:
        if name in scales:
            rows.append(('cap' if name == 'capacity' else name,
                         '%.2f ±%.2f' % (scales[name], sigma[name])))
    # The room, identified beside the scales (MINOR 15): the board has no
    # ambient sensor, so its rise is measured against this. Its hint, the
    # bench's emoji pair, sits here rather than over the board (2026-09-06).
    if ident.get('ambient') is not None:
        kind = hint if hint is not None else room_hint(ident)
        rows.append(('room', Text('%.1f ±%.1f C   %s' % (
            ident['ambient'], ident.get('ambient_sigma', 0.0),
            ROOM_HINTS.get(kind, '')))))
    # The stand-in only: the situation its board is in and the scales that
    # make it, beside what the observer found. A board has no truth to tell;
    # the rows are absent.
    truth = ident.get('truth')
    if not truth:
        return rows
    rows.append(('sim', '%s  %.0f min' % (truth['situation'],
                                           truth['since_s'] / 60.0)))
    rows.append(('', 'air %.2f  cap %.2f  room %.0f C'
                 % (truth['air'], truth['capacity'],
                    truth.get('ambient', 25.0))))
    if truth.get('load_a') is not None:
        rows.append(('load', '%.0f A on' % truth['load_a']
                     if truth['load_a'] > 0.0 else 'idle'))
    return rows


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


#: The thermometers' floor the board judges its innovation against -
#: `THERMAL_IDENT_NOISE_K`, not on the wire; the same 0.1 K the stand-in
#: is told.
IDENT_NOISE_K = 0.1


def envelope_rows(ident):
    """HEADROOM's rows for the identification: the margin the envelope keeps
    of every span now, the floor it rose from, the innovation that moves
    it, and which term holds the margin down - the innovation, or the air
    path's, the capacity's or the room's sigma - `none` when the span is
    earned.
    """

    if not ident:
        return []
    cap = ident.get('trip_cap')
    held = cap is not None and cap < 1.0 and abs(ident['margin'] - cap) < 1e-6
    rows = [('margin', '%.2f%s' % (ident['margin'],
                                   '  the trip cap' if held else ''))]
    if ident.get('margin_floor') is not None:
        rows.append(('floor', '%.2f' % ident['margin_floor']))
    rows.append(('innovation', '%.2f K' % ident.get('innovation_k', 0.0)))
    sigma = ident.get('sigma') or {}
    if sigma and ident.get('ambient_sigma') is not None:
        terms = thermal_ident.doubt_terms(
            ident.get('innovation_k', 0.0),
            [sigma.get(name, 0.0) for name in thermal_ident.SCALES]
            + [ident['ambient_sigma']], IDENT_NOISE_K)
        name, worst = max(terms.items(), key=lambda kv: kv[1])
        rows.append(('doubt', 'none' if worst < 0.005
                     else '%s %.2f' % (name, worst)))
    return rows


#: The map's marks in words, a box under SENSE (bench 2026-09-06).
MAP_WORDS = {'U': 'FETs, shunts', 'V': 'FETs, shunts', 'W': 'FETs, shunts',
             'REG': 'regulators', 'MCU': 'the STM32', 'HS': 'hot swap',
             'AFE': 'front end', 'NTC': 'thermistor, by the bore'}


def map_rows():
    """One row a mark: the label, its references off the pick and place,
    and what they are - the panel's width, so a family of six is its
    first and last and the count."""
    rows = []
    for label, refs, _where, _margin in MARKS:
        named = (' '.join(refs) if len(refs) <= 5
                 else '%s..%s (%d)' % (refs[0], refs[-1], len(refs)))
        rows.append((label, '%s - %s' % (named, MAP_WORDS.get(label, ''))))
    return rows


def status_boxes(state, budget, aspect=None, ident=None, hint=None):
    """The thermal observer's numbers as instrument boxes, every one the
    board's - and, given `(aspect, how)`, the one number that is the
    terminal's: how tall its cell was measured, or assumed, to be.
    """

    age = state.get('seen_s_ago')
    every = state.get('sample_every_s') or 0.0
    fresh = state['ntc'] is not None and (
        age is None or every <= 0.0 or age <= 2.0 * every)

    if state['ntc'] is None:
        sense: list = [Text('AFE off - open loop', style='value')]
    elif not fresh:
        sense = [('NTC', '%.1f C' % state['ntc']),
                 Text('%.0f s old - open loop' % age, style='value')]
    else:
        sense = [('NTC', '%.1f C' % state['ntc']),
                 ('err', '%+.2f K' % state['error'])]
    # One fact a row (bench 2026-09-06): `sample every 30 s - last 0 s ago`
    # as one row was cropped at the panel's edge.
    sense += [('open', '%d s  %s' % (state['seconds'],
                                     'settled' if state['settled']
                                     else 'settling')),
              ('sample', '%.0f s' % every),
              ('last', '%.0f s ago' % age if age is not None else '-')]
    if ident is not None:
        sense += ident_rows(ident, hint)
    if aspect is not None:
        sense.append(('cell', '%.2f tall %s' % aspect))

    boxes = [hud('SENSE', sense), hud('MAP', map_rows())]
    if budget is not None:
        left = budget['seconds_to_limit']
        state_text = ('TRIPPED' if budget['tripped']
                      else 'THROTTLING' if budget['throttling'] else 'ok')
        boxes.append(hud('HEADROOM', soak(budget) + [
            ('worst', Text.assemble(
                (pretty(budget['worst_node']), 'name'), '   ',
                (state_text, 'value' if state_text != 'ok' else 'label'))),
            ('to limit', ('%.0f s' % left) if left is not None
             else 'not heating')] + envelope_rows(ident)))
    # Every node the thermal observer estimates, by name, plus the dies
    # (measured) and the ambient it infers.
    nodes = state.get('nodes') or {}
    rows = [(pretty(name), '%.1f C' % nodes[name])
            for name in ALL_NODES if name in nodes]
    rows += [('ambient', '%.1f C' % (state.get('ambient') or 0.0)),
             ('mcu die', '%.1f C' % state['mcu']
              if state.get('mcu') is not None else '-'),
             ('a1335 die', '%.1f C' % state['afe']
              if state.get('afe') is not None else '-')]
    boxes.append(hud('LEVELS', rows))
    boxes.append(hud('TUBES  %.0f to %.0f C' % (gauges.TEMP_FLOOR_C,
                                                gauges.TEMP_SCALE_C),
                     [Text.from_ansi(line) for line in tubes(state, budget)]))
    return boxes


#: What each tube is called under itself, two letters at the tubes'
#: pitch of three: the leg for the drivers and the phases, the part for
#: the rest.
SHORT_NODE = {'driver_u': 'dU', 'driver_v': 'dV', 'driver_w': 'dW',
              'phase_u': 'pU', 'phase_v': 'pV', 'phase_w': 'pW',
              'mcu': 'MC', 'regulators': 'RG', 'afe': 'AF', 'board': 'PB',
              'hotswap': 'HS', 'patch_u': 'lU', 'patch_v': 'lV',
              'patch_w': 'lW', 'patch_left': 'lL', 'patch_bottom': 'lB',
              'patch_right': 'lR', 'winding': 'WI', 'stator': 'ST',
              'rotor': 'RO'}
TUBE_ROWS = 8

#: Two rows of tubes since the graph: the parts, then the laminate's
#: patches and the motor - twenty tubes at a pitch of three would be
#: sixty columns in a forty-column box.
TUBE_ROWS_OF = (('driver_u', 'driver_v', 'driver_w', 'phase_u', 'phase_v',
                 'phase_w', 'mcu', 'regulators', 'afe', 'hotswap'),
                ('board', 'patch_u', 'patch_v', 'patch_w', 'patch_left',
                 'patch_bottom', 'patch_right', 'winding', 'stator',
                 'rotor'))


def tubes(state, budget):
    """The ten nodes and the thermistor as thermometers - the motor page's
    own, on this page too.
    """
    nodes = state.get('nodes') or {}
    used = (budget or {}).get('used') or {}
    tripped = bool((budget or {}).get('tripped'))
    lines = []
    for index, row in enumerate(TUBE_ROWS_OF):
        entries, labels = [], []
        for name in row:
            if name in nodes:
                entries.append((gauges.temp_share(nodes[name]),
                                gauges.margin_class(used.get(name, 0.0),
                                                    tripped)))
                labels.append(SHORT_NODE.get(name, name[:2]))
        if index == 0 and state.get('ntc') is not None:
            entries += [None, (gauges.temp_share(state['ntc']),
                               gauges.thermometer_class(state['ntc']))]
            labels += ['', 'NTC']
        if entries:
            lines += gauges.tubes(entries, TUBE_ROWS, labels, pitch=3)
    return lines


def picture(state, console, reserve, aspect=CELL_ASPECT):
    """The board and its scale."""
    nodes = state['nodes']
    board_c = nodes.get('board')
    if board_c is None:
        return ['  the board sent no board node - device 8 is out of step']

    zones = {k: v for k, v in nodes.items() if k != 'board'}
    # The leading blank moves the board one row down the frame - asked
    # 2026-08-30, and counted in the caller's reserve.
    return [''] + render(zones, board_c=board_c, colour=console,
                         margin=PANEL_W, reserve=reserve,
                         trailing=TRAILING, aspect=aspect).split('\n')


def put_back(rig, load):
    """Undo what the run armed, step by step, and say what each did."""
    if load is None:
        return [('AFE_ON', 'untouched - this run only watched'),
                ('gate stage', 'untouched, nothing was armed')]
    done = []
    for name, what, undo in (
            ('duty', 'three legs to zero',
             lambda: rig.write(analog=dict.fromkeys(load, 0.0))),
            ('gate stage', 'disarmed, MOE clear', rig.gates.off)):
        try:
            undo()
            done.append((name, what))
        except (NoReplyError, RigError) as exc:
            done.append((name, 'FAILED: %s' % exc))
    done.append(('AFE_ON', 'back the way it was found'))
    return done


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', default='COM4')
    p.add_argument('--simulated', action='store_true')
    p.add_argument('--hz', type=float, default=2.0)
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
          Coaxial63100(port=a.port, simulated=a.simulated, power_afe=False) as rig):
        ready()
        origin = rig.origin
        # On the stand-in, however reached: `--simulated`, or a bench with no
        # cable where the rig falls back to it. Keyed on the flag, the page ran
        # without its load (bench 2026-09-06).
        if not origin.real:
            # Ground truth on the tour (temperate, cold, toasty, repeating),
            # moved on once the identification has earned the room: the
            # innovation swings and settles before it matters on a board.
            rig.thermal.situation('tour')
            # A load on it, two model minutes at 30 A and four cooling: the
            # regions pulse on the map and the bar under the board has
            # cooldowns to rise on.
            rig.thermal.load_cycle(on_s=PAGE_CYCLE_ON_S,
                                   off_s=PAGE_CYCLE_OFF_S)
        say('ok' if origin.real else 'warn', 'link',
            '%s - %s' % (origin.label, 'live' if origin.real else 'simulated'))
        say('ok', 'AFE_ON', 'left exactly as found - it gates the drivers')
        say('wait', 'drawing', 'Q closes it, ESC goes back to the menu')

        load = None
        if a.switch is not None:
            legs = [x.strip().upper() for x in a.phases.split(',')]
            # AFE off first, then arm.
            rig.board.afe.off()
            rig.gates.on(bypass_sto=True, ignore_interlock=True)
            load = {'Phase ' + leg: a.switch for leg in legs}
            rig.write(analog=load)
            say('warn', 'switching', '%s at %.0f %% - AFE off, STO bypassed'
                % ('+'.join(legs), a.switch * 100))

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
        last = {'body': ['  waiting for device 8'], 'boxes': [],
                'ident': None, 'ident_at': 0.0, 'hint': None}
        leaving = None

        def draw():
            # A quiet link keeps the last good picture (FINDINGS): a blank
            # board each time made the view unreadable.
            with suppress(NoReplyError, RigError):
                got = rig.board.thermal.state()
                # The identification moves once a sample, every thirty seconds
                # on the board: one round trip every few seconds is plenty, and
                # one a frame was a fifth of the frame.
                if time.time() - last['ident_at'] > IDENT_EVERY_S:
                    last['ident'] = rig.board.thermal.identification()
                    last['ident_at'] = time.time()
                    # The hint with its hysteresis: what was shown stands until
                    # the room is well past a threshold.
                    last['hint'] = room_hint(last['ident'], last['hint'])
                last['boxes'] = status_boxes(got, rig.board.thermal.budget(),
                                             aspect, ident=last['ident'],
                                             hint=last['hint'])
                last['body'] = picture(got, console, reserve,
                                       aspect[0] / 2.0)
            # Three cells of pad and eight of field: six and twelve read as
            # dead air around the board.
            body = last['body']
            field = max((visible(l) for l in body), default=0) + 8
            art = stamp_crosses(['   ' + l for l in body], field)
            # The evidence bar under the board, after the crosses are stamped
            # so nothing lands on it.
            art += evidence_rows(last['ident'], colour=console)
            return frame_of(board_view, origin, 'THERMAL OBSERVER',
                            '\n'.join(art), last['boxes'],
                            (('Q', 'EXIT'), ('ESC', 'MENU')))

        try:
            leaving = run_view(board_view, console, period, a.frames, draw)
        finally:
            done = put_back(rig, load)
            sys.stdout.write('\n')
            closing(done, console, 0)

    return TO_MENU if leaving == 'menu' else 0


if __name__ == '__main__':
    sys.exit(main())
