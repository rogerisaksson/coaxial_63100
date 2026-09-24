"""The session dash's panels: analog, thermal, bridges, DIO, IMU, angle, acquisition."""
import time

from rich.text import Text

from coaxial.devices import scaling
from coaxial.draw import desk, gauges
from coaxial.model import thermal
from terminal.screen import ASH, LABEL, SODIUM, gauge, hud, panels_of, steady, tint
from terminal.views.session.state import ACTIVITIES


#: Samples per channel in the dash's analog read. 64 costs 88 ms of round
#: trip against a 500 ms frame; 16 costs 30 and the dash is a glance, not a
#: measurement - the meter bridge view is the one that averages.
ADC_SAMPLES = 16

#: Cells in a budget bar. Ten reads as a percentage without being counted.
BAR = 10

#: Below this the A1335 has no magnet in front of it and the angle is noise.
#: The same number dial.py uses, and it is the host's judgement, not the
#: board's - the board reports the gauss and says nothing about it.
WEAK_GAUSS = 30


def snapshot(session):
    """One round of reads, each guarded."""
    rig = session.rig
    board = rig.board
    got = {
        'afe': steady(board.afe.state),
        'gates': steady(rig.gates.state),
        'thermal': steady(board.thermal.state),
        'budget': steady(board.thermal.budget),
        'imu': steady(board.imu.state),
        'angle': steady(board.angle.state),
    }
    # Only while the rail is up.
    afe_on = bool(got['afe'] and got['afe']['on'])
    got['analog'] = (steady(board.analog.read_all,
                            nr_of_samples=ADC_SAMPLES) if afe_on else None)
    got['daq'] = steady(board.daq.state)
    got['ring'] = steady(board.capture.state)
    got['scaling'] = session.scaling
    got['field'] = session.field
    return got


def block(title, rows):
    """One dashboard instrument - the stage's hud, ANSI rows carried as-is."""
    return hud(title, [row.strip() if isinstance(row, str) else row
                       for row in rows])


def adc_block(got):
    """Every analog channel, cooked by the board's own record."""
    table = got.get('analog')
    afe = got.get('afe')
    if table is None and afe is not None and not afe['on']:
        # Not a fault: the session leaves the rail down to save power and heat,
        # and the thermal observer borrows it on its own schedule.
        return block('ANALOG', [
            tint('  AFE_ON down - no reference', LABEL),
            tint('  thermal observer borrows it for samples', LABEL)])
    if table is None:
        return block('ANALOG', ['  did not answer'])

    params = got.get('scaling')
    rows = []
    for r in table['channels']:
        to = scaling.converter(r.get('unit'), r['differential'],
                               signal=r.get('signal'), params=params)
        # THE METER BRIDGE'S GAUGE BESIDE THE NUMBER: where the reading sits in
        # the converter's own range, a bipolar channel about its centre - the
        # same instrument that page draws, at the width the thermal box below
        # draws its levels.
        share = desk.fraction(r)
        rows.append('  %-8s %9.3f %-2s %s'
                    % (r['signal'][:8], to(r['mean_raw']),
                       scaling.symbol(r.get('unit'), r.get('signal')),
                       gauges.gauge((share + 1.0) / 2.0 if r['differential']
                                    else share, BAR,
                                    centre=0.5 if r['differential']
                                    else None)))
    return block('ANALOG', rows)


def thermal_block(got):
    """Each node as a fraction of its own ceiling."""
    spend = got.get('budget')
    if spend is None:
        return block('THERMAL', ['  the thermal observer did not answer'])

    # EVERY NODE, in the firmware's own order, so a leg keeps its row whether it
    # is heating or not - sorted-and-cut dropped driver W the moment two other
    # nodes warmed, on the dashboard that exists to show one leg heating alone.
    used = spend['used']
    nodes = [n for n in thermal.ALL_NODES if n in used]
    width = max((len(thermal.pretty(n)) for n in nodes), default=0)
    rows = ['  %-*s %s %3.0f %%' % (width, thermal.pretty(n), gauge(used[n], BAR),
                                    100.0 * used[n]) for n in nodes]
    return block('THERMAL', rows)


def bridges_block(got):
    """The six gate signals, by leg, and what the stage is doing."""
    state = got.get('gates')
    if state is None:
        return block('HALF-BRIDGES', ['  the stage did not answer'])

    pins = state.get('pins') or {}
    # `requested_ticks` is a CCR count, so the duty is it over the period.
    span = float(max(1, state.get('period', 1) - 1))
    rows = []
    for leg, ticks in zip(('U', 'V', 'W'),
                          state.get('requested_ticks', (0, 0, 0))):
        rows.append('  %s   H %s  L %s   %3.0f %%'
                    % (leg,
                       'ON ' if pins.get(leg + 'H') else 'off',
                       'ON ' if pins.get(leg + 'L') else 'off',
                       100.0 * ticks / span))
    rows.append('  MOE %-4s  break %s'
                % ('set' if state['pwm_enabled'] else 'clear',
                   'LATCHED' if state['fault'] else 'clear'))
    rows.append('  dead time %d ns' % state['deadtime_ns'])
    return block('HALF-BRIDGES', rows)


def dio_block(got):
    """The three pins the board drives, and the break it only reads."""
    afe = got.get('afe')
    state = got.get('gates')
    if afe is None:
        return block('DIO', ['  did not answer'])

    rows = ['  AFE_ON  %-4s %s'
            % ('high' if afe['on'] else 'low',
               ', '.join(afe['users']) or 'nobody holds it')]
    if state is not None:
        rows.append('  PE15    %-4s break, read only'
                    % ('high' if not afe['pe15'] else 'low'))
        rows.append('  KEEPALIVE  %d edges' % state['keepalive'])
    return block('DIO', rows)


def imu_block(got):
    """The BNO085: is it reporting, and how fast."""
    state = got.get('imu')
    if state is None:
        return block('IMU', ['  did not answer'])

    q = state.get('quaternion')
    asked = state.get('feature') or {}
    rows = ['  loop %-8s  %d reports' % (state['loop'], state['updates']),
            '  report %s' % _report(asked),
            '  errors %-6d %s' % (state['errors'],
                                  state.get('last_fault', ''))]
    if q:
        rows.append('  q %+.3f %+.3f %+.3f %+.3f'
                    % (q['i'], q['j'], q['k'], q['real']))
    else:
        rows.append('  no quaternion yet')
    return block('IMU', rows)


def angle_block(got):
    """The A1335, and whether there is a magnet to measure."""
    state = got.get('angle')
    if state is None:
        return block('ANGLE', ['  did not answer'])

    rows = ['  loop %-8s  %d readings' % (state['loop'], state['updates']),
            '  errors %-6d %s' % (state['errors'], state['error'])]
    field = got.get('field')
    if field is not None and field < WEAK_GAUSS:
        rows.append('  %d gauss - no magnet, so' % field)
        rows.append('  the angle is noise')
    elif state.get('degrees') is None:
        # The loop reads a register the host asked it to, and only ANG decodes
        # to an angle.
        rows.append('  reading %s, not ANG' % state.get('register_name', '?'))
    else:
        rows.append('  %7.2f deg' % state['degrees'])
    return block('ANGLE', rows)


def _interval(us):
    if not us:
        return 'nothing asked for'
    return '%d ms' % (us // 1000) if us >= 1000 else '%d us' % us


def _report(asked):
    """What the part was asked to send, or that nothing was."""
    if not asked.get('interval_us'):
        return 'none asked for'
    return '%d @ %s' % (asked.get('report_id', 0),
                        _interval(asked['interval_us']))


def _degc(value):
    """A thermometer that did not answer says so rather than reading 0."""
    return '--' if value is None else '%.1f' % value


def acquisition_block(got):
    """The DAQ task and the capture ring - the capture view, as one box."""
    daq, ring = got.get('daq'), got.get('ring')
    if daq is None and ring is None:
        return block('ACQUISITION', ['  did not answer'])

    rows = []
    if daq is not None:
        rows.append('task %s  acc %d' % (daq.get('clock', '?'),
                                         daq.get('accumulate', 0)))
        rows.append('buffered %d  dropped %d' % (daq.get('available', 0),
                                                 daq.get('dropped', 0)))
    if ring is not None:
        rows.append('ring %s' % (', '.join(ring.get('sources', ()))
                                 or 'not armed'))
        rows.append('held %d of %d  thin %d'
                    % (ring.get('count', 0), ring.get('depth', 0),
                       ring.get('thinned', 0)))
    return block('ACQUISITION', rows)


def dash(session, got):
    """One line over the six blocks: what the system is doing right now."""
    afe, gates = got.get('afe'), got.get('gates')
    spend, therm = got.get('budget'), got.get('thermal')

    bits = []
    if afe is not None:
        # Sodium marks power flowing; off is just the street.
        bits.append('AFE %s' % (tint('on', SODIUM) if afe['on']
                                else tint('off', ASH)))
    if gates is not None:
        bits.append('gates %s' % (tint('SWITCHING', SODIUM)
                                  if gates['pwm_enabled']
                                  else tint('idle', ASH)))
    if spend is not None:
        bits.append('worst %s %.0f %%'
                    % (thermal.pretty(spend['worst_node']),
                       100.0 * spend['worst']))
    if therm is not None:
        # All three thermometers here rather than in the thermal column: they
        # are the widest line there is, and a column sized by them squeezed the
        # two beside it.
        bits.append('NTC %s  A1335 die %s  MCU die %s C'
                    % (_degc(therm['ntc']), _degc(therm['afe']),
                       _degc(therm['mcu'])))
    if session.running:
        bits.append('running: %s' % ', '.join(sorted(session.running)))
    if session.plan is not None:
        bits.append(session.plan.caption(time.time()))
    return '  ' + '   '.join(bits)


def frame(session, console, note):
    """The dashboard on the stage: dash strip, six instruments, key bar."""


    got = snapshot(session)
    rows = [[Text.from_ansi(dash(session, got))],
            [adc_block(got), thermal_block(got), bridges_block(got)],
            [dio_block(got), imu_block(got), angle_block(got),
             acquisition_block(got)]]
    keys = [(a.key.upper(), a.name.upper()
             + (' [ON]' if a.name in session.running else ''))
            for a in ACTIVITIES]
    keys += [('A', 'AFE'), ('+ -', 'DUTY %.0f%%' % (100 * session.duty)),
             ('Q', 'EXIT'), ('ESC', 'MENU')]
    if note:
        keys.append(('', note))
    return panels_of(console, session.rig.origin, 'SESSION', rows,
                     keys)
