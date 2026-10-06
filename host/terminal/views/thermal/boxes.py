"""THERMAL OBSERVER's side column: SENSE, MAP, HEADROOM, LEVELS and TUBES, every number the board's.

    boxes = status_boxes(state, budget, aspect, ident=ident, hint=hint, afe=afe)
    hint = room_hint(ident, held)            # the room's word, with its hysteresis
"""
from rich.text import Text

from coaxial.devices.thermal import at_trip_cap
from coaxial.draw import gauges
from coaxial.draw.thermalmap import MARKS
from coaxial.kalman import thermal_ident
from coaxial.model.thermal import ALL_NODES, pretty
from terminal.ui.stage import hud

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


def ident_rows(ident, hint=None):
    """The identification in SENSE, one fact a row: the state as a chip, each
    online scale with its sigma, the room, and on the stand-in the truth and
    the load on it.
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
    rows.append(('', 'air %.2f cap %.2f room %.0f C'
                 % (truth['air'], truth['capacity'],
                    truth.get('ambient', 25.0))))
    # What the cycle asks and what the envelope lets through of it: `0 A on` stood over a
    # bridge switching, throttled to nothing (2026-10-05).
    if truth.get('load_a') is not None:
        asked = truth.get('asked_a')
        asked = truth['load_a'] if asked is None else asked
        rows.append(('load', '%.0f of %.0f A rms' % (truth['load_a'], asked)
                     if asked > 0.0 else 'idle'))
    return rows


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
    held = at_trip_cap(ident['margin'], ident.get('trip_cap'))
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


def status_boxes(state, budget, aspect=None, ident=None, hint=None, afe=None):
    """The thermal observer's numbers as instrument boxes, every one the
    board's - and, given `(aspect, how)`, the one number that is the
    terminal's: how tall its cell was measured, or assumed, to be.

    `afe` is whether AFE_ON stands, where the page has read it: with no
    reading yet the box said the AFE was off while it was on, the observer
    simply not having reached its first sample 30 s in (2026-09-28).
    """

    age = state.get('seen_s_ago')
    every = state.get('sample_every_s') or 0.0
    fresh = state['ntc'] is not None and (
        age is None or every <= 0.0 or age <= 2.0 * every)

    if state['ntc'] is None:
        waiting = every > 0.0 and (state.get('seconds') or 0) < every
        if afe is False:
            why = 'AFE off - open loop'
        elif waiting:
            why = 'open loop, sample in %d s' % (
                every - (state.get('seconds') or 0))
        else:
            why = 'no reading - open loop'
        sense: list = [Text(why, style='value')]
    elif not fresh:
        sense = [('NTC', '%.1f C' % state['ntc']),
                 Text('%.0f s old - open loop' % age, style='value')]
    else:
        # No `err` row: the model now against a reading a sample old read +9 K on a model
        # 0.1 K out (2026-10-05); the miss as judged is HEADROOM's innovation.
        sense = [('NTC', '%.1f C' % state['ntc'])]
    # One fact a row (bench 2026-09-06): `sample every 30 s - last 0 s ago`
    # as one row was cropped at the panel's edge. `run`: how long the observer has, with a
    # thermometer or without; `last` a dash until one has answered.
    answered = any(state.get(name) is not None for name in ('ntc', 'mcu', 'afe'))
    sense += [('run', '%d s  %s' % (state['seconds'],
                                    'settled' if state['settled']
                                    else 'settling')),
              ('sample', '%.0f s' % every),
              ('last', '%.0f s ago' % age if age is not None and answered else '-')]
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
             else 'at the limit' if budget['worst'] >= 1.0
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
    """Every node and the thermistor as thermometers - the motor page's
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
