"""Acquisition: the converters into records, a frame, and a live plot.

The DAQ session, the pandas run and the live plot, as one paper."""
from .parts import code, md, section

TITLE = 'Acquisition'
SUBTITLE = ('The converters into records: a task configured and clocked, its '
            'records read in a loop, scaled into a frame, and drawn live over '
            'the switches.')
ABSTRACT = (
    'The board records sums of converter codes at a rate the task asks for '
    'and the loop manages, and a host reads them as `Record`s over one link. '
    'This notebook runs the whole path on the stand-in, and on a board with '
    'the knob flipped: the catalogue of what can be recorded, the rail that '
    'makes a reading a measurement, the clock tied to the host, a task of two '
    'then five channels, the record read back as sums, counts and means, a '
    'run scaled into a pandas frame through the calibration record, and a '
    'two-second window of phase currents and gate duties redrawn as they '
    'arrive. What it measures: the record period the loop managed against '
    'the rate asked for, the spread of `dt`, what the ring holds, and how far '
    'behind a reader is allowed to fall. What a reader takes to the bench: '
    'a record is a sum and a count, `dt` is measured and not configured, and '
    'a code column beside a scaled one is what arrived beside what it means.')

SECTIONS = [
    section(
        'What the board can record',
        md('`daq.catalogue()` is the board\'s own list, each row saying its '
           'kind and whether `configure()` may ask for it. AFE_ON powers the '
           'ADC reference: with it off every channel reads exact mid-scale '
           'and the NTC exactly 25.00 C (invariant 9), so `enable()` takes '
           'this session\'s reference on the rail before anything is '
           'believed, and `close()` releases it.'),
        code('''daq = device.daq
daq.open()
for row in daq.catalogue():
    print('%-16s %-8s %-4s %-10s selectable=%s'
          % (row['name'], row['kind'], row['direction'], row['unit'],
             row['selectable']))
daq.enable()
print(device.afe.state())'''),
    ),
    section(
        'The clock',
        md('The board counts cycles, not time. `set_time_from_pc` ties the '
           'counter to the host\'s clock; `reference=\'utc\'` measures that '
           'clock against NTP over the same window and takes out its offset '
           'and its rate, since a host clock is not a reference either.'),
        code('''sync = device.set_time_from_pc(reference='pc')
print(sync)'''),
    ),
    section(
        'A task, read in a loop',
        md('`start()` puts a reader thread on the link, and it is the only '
           'thing that touches the transport while it lives. Every `read(-1)` '
           'answers its own backlog: the first record blocks, the rest come '
           'with it.'),
        code('''import time

layout = daq.configure('phaseU', 'NTC', sample_rate=50)
print(daq.channel_names())
print(layout)
daq.start()
records = []
for _ in range(5):
    records.extend(daq.read(-1))
    time.sleep(0.2)
daq.stop()
for r in records[:8]:
    print('%.3f  dt %-7s %s' % (r.start_time, '%.4f' % r.dt if r.dt is not None else 'none',
                                 [(s.name, round(s.value, 1)) for s in r.samples]))
print('records:', len(records))
shape = daq.state()
held = daq.buffered
print(shape)
print(held)'''),
        md('A `Record` is a dict underneath: `r[\'NTC\']` is the SUM over '
           '`r.count` readings, `r.value(\'NTC\')` that channel\'s mean and '
           '`r.sample(\'NTC\')` the struct behind it. `daq.series` and '
           '`daq.columns` are the two helpers around a whole run.'),
        code('''r = records[0]
print('sum   ', r['NTC'])
print('count ', r.count, ' (r["samples"] is the same number:', r['samples'], ')')
print('mean  ', r.value('NTC'))
print('sample', r.sample('NTC'))
print('names ', r.channel_name)
print('pins  ', r.digital)
cols = daq.columns(records)
print(sorted(cols))
ntc = daq.series(records, 'NTC')
seconds = daq.series(records, 'time')
print('%.1f s of NTC, first %.1f last %.1f' % (seconds[-1] - seconds[0], ntc[0], ntc[-1]))'''),
    ),
    section(
        'A run into a frame, scaled by the record',
        md('`frame(scaled=True)` adds one column per channel in real units '
           'beside the codes, through the board\'s own converters and the '
           'channel trims in its calibration record (invariant 7). `stored` '
           'says whether that record was ever written or is the schematic\'s '
           'arithmetic; an uncalibrated board answers an empty record and '
           'every converter falls back to the compiled-in constant, which is '
           'what `name` says.'),
        code('''daq.configure('phaseU', 'phaseV', 'phaseW', 'DC bus', 'NTC', sample_rate=100)
daq.start()
run = daq.read(300)
daq.stop()
print(len(run), 'records;', daq.channel_names(run[0]))
cal = device.calibration.read()
print('record stored:', cal['stored'], ' version:', cal['version'],
      ' params held:', len(cal['params']))
for name, params in sorted(device.analog.scaling().items()):
    print('%-8s %s' % (name, params.name))
print('channel trims:', [(c['index'], c['offset_raw'], c['gain_ppm'])
                         for c in cal['channels'][:3]])'''),
        code('''df = daq.frame(run, index='elapsed', scaled=True)
df.head()'''),
        code('''df.describe().round(3)'''),
        code('''from coaxial.draw.figures import figure, show

units = [c for c in df.columns if c.endswith('(A)')]
shown = units + ['DC bus (V)', 'NTC (C)']
fig, panels = figure(rows=len(shown), sharex=True)
for panel, column in zip(panels, shown):
    panel.plot(df.index, df[column])
    panel.set_ylabel(column)
panels[-1].set_xlabel('s')
show(fig)'''),
        md('The codes stay in the frame under the board\'s own channel names, '
           'so a tare or a span can be checked against what arrived.'),
        code('''print(df[['Phase U', 'Phase U (A)', 'NTC', 'NTC (C)']].iloc[:3])'''),
    ),
    section(
        'Currents over the switches, live',
        md('The pins ride the same records as the analog fields, so every '
           'point on both is one window. The phase sense is zeroed with the '
           'stage down - a tare stores what the channels read now as their '
           'zero - and then the drive holds a current vector turning at 3.5 Hz '
           'electrical on the model: three currents 120 degrees apart, and '
           'the six gates modulating about half. Fifty records a second is '
           'fourteen points per electrical turn. `frames()` yields the last '
           '`window` seconds each time records arrive, indexed on seconds '
           'before now.'),
        code('''import math

daq.configure('phaseU', 'phaseV', 'phaseW', digital=True, sample_rate=50)
print(daq.channel_names())
print(device.calibration.tare('phaseU', 'phaseV', 'phaseW'))
drive = device.drive
drive.source('model')
device.gates.arm(bypass_sto=True, ignore_interlock=True)
drive.setpoint(id_ref=4.0, iq_ref=0.0, theta=0.0, omega_target=2 * math.pi * 3.5)
drive.mode('hold')'''),
        code('''from IPython.display import clear_output

daq.start()
frames = 0
for window in daq.frames(window=2.0, buffer=6.0, seconds=6.0, scaled=True):
    frames += 1
    clear_output(wait=True)
    fig, (top, bottom) = figure(rows=2, sharex=True)
    for column in (c for c in window.columns if c.endswith('(A)')):
        top.plot(window.index, window[column], label=column)
    top.set_ylabel('A')
    top.legend(loc='upper left')
    for column in (c for c in window.columns if c.startswith('TIM1_CH')):
        bottom.plot(window.index, window[column])
    bottom.set_ylabel('gate duty')
    bottom.set_ylim(-0.05, 1.05)
    bottom.set_xlabel('s before now')
    show(fig)
live = daq.buffered
daq.stop()
drive.off()
device.gates.disarm()
drive.source('adc')
whole = daq.history(scaled=True)
print(frames, 'frames drawn;', live)
print(len(whole), 'records held,', round(-whole.index.min(), 2), 's back')'''),
    ),
]

CONCLUSIONS = [
    code('''spans = [r.dt for r in records if r.dt]
counts = [r.count for r in records]
codes = [c for c in df.columns if not c.endswith(')')]
scaled = [c for c in df.columns if c.endswith(')')]
print('1. records          %d at 50/s asked, %.2f s of covered time' % (len(records), sum(spans)))
print('2. record period    %.4f s mean = %.1f /s; dt spread %.4f to %.4f s'
      % (sum(spans) / len(spans), len(spans) / sum(spans), min(spans), max(spans)))
print('3. readings summed  %d to %d per record' % (min(counts), max(counts)))
print('4. stride           %d bytes, %d analog fields; the ring holds %d records'
      % (shape['stride'], shape['fields'], shape['capacity']))
print('5. dropped          %d, host queue peak %d' % (shape['dropped'], held['peak']))
print('6. frame            %d records, %d code columns, %d scaled columns'
      % (len(df), len(codes), len(scaled)))
for name in ('Phase U', 'DC bus', 'NTC'):
    unit = [c for c in scaled if c.startswith(name)][0]
    span = df[unit].max() - df[unit].min()
    print('   %-8s codes %8.1f +/- %6.1f   %-12s %8.3f +/- %.3f  span %.3f'
          % (name, df[name].mean(), df[name].std(), unit, df[unit].mean(),
             df[unit].std(), span))
print('7. live             %d frames in 6 s = %.1f /s; %d records %.2f s deep; '
      'reader %d reads, %.1f records/s; queue peak %d, dropped %d, board backlog %s'
      % (frames, frames / 6.0, len(whole), -whole.index.min(), live['reads'],
         live['rate'], live['peak'], live['dropped'], live['backlog']))'''),
    md('`dt` is measured, not configured: it is the gap to the next record\'s '
       'timestamp within the block that carried it, because what the task was '
       'asked for and what the loop managed are different numbers - which is '
       'why the board sends a count with every sum. It is per block because '
       'the stamps are raw CYCCNT and that counter wraps every 9.04 s at '
       '475 MHz; the acquisition unwraps each block it receives, so an index '
       'that runs longer than that stays monotonic. `dropped` is what the '
       'ring had no room for; a reader thread that keeps up leaves it at '
       'zero.\n\n'
       'A code column is what arrived; the column beside it is what it means. '
       'The conversion is the board\'s own (invariant 7): the scaling lives '
       'in the calibration record, so `frame(scaled=True)` asks '
       '`board.analog.scaling()` rather than holding a constant, then applies '
       'that channel\'s offset and gain trim. Both columns stay, because what '
       'arrived and what it means are two things. With an empty record every '
       'converter falls back to the compiled-in constant and says so in '
       '`name`. `calibration.span(index, reference)` writes a gain trim '
       'against an instrument, taking the reference in the channel\'s own '
       'unit - mA for a phase, mV for the DC link; the DC link\'s stands at '
       '-32 418 ppm (FINDINGS). That divider is 49.9k/2.2k: 78.15 V full '
       'scale on a 63 V rating, 24 % of headroom so an over-rating transient '
       'is recorded rather than clipped (invariant 11).\n\n'
       'Live, the reader thread is the only thing on the transport, so a '
       'redraw in the loop never sits between two round trips, and every '
       'read answers its own backlog in the same transaction. `frames()` '
       'yields what is on screen and keeps `buffer` seconds behind it as '
       'records, so nothing is concatenated and nothing grows; the index is '
       'seconds before now, newest at 0, so the axis stands still while the '
       'data moves through it. A ring is finite: a reader that falls far '
       'enough behind for the writer to lap it loses records and is told how '
       'many in `buffered[\'lost\']` - a terminal that stopped drawing for '
       'six seconds once overflowed a 16 K ring, 334 records (FINDINGS).'),
]

BENCH = (
    'Flip `SIMULATED` and name the port. Run the first three sections before '
    'trusting a number: the catalogue says what this firmware records, '
    '`afe.state()` says the reference rail is up, and the sync says how far '
    'the host clock sat from NTP. Then compare conclusion 2 against the rate '
    'asked for - the loop\'s own period is what the board managed, and a '
    'board under a switching run manages less. Before a scaled frame means '
    'amperes, read `stored` and `version` in section 5: an empty record is '
    'the schematic\'s arithmetic, and `calibration.span` against a meter is '
    'what turns it into a measurement. The stand-in cannot show a lapped '
    'ring or a reader falling behind a real link; `buffered[\'lost\']` on a '
    'board is where that shows.')

REFERENCES = [
    ('host/coaxial/rig.py', 'the front door: `daq`, `set_time_from_pc`, `frame`, `frames`, `history`'),
    ('host/coaxial/acquire/record.py', 'a `Record`: the sum, the count, the mean, the struct behind a sample'),
    ('host/coaxial/simulated/daq.py', 'the stand-in this ran on, paced to real time like a board'),
    ('daq/src/daq.c', 'the engine on the board: the ring, the window, the ladder'),
    ('docs/PROTOCOL.md', 'device 6, the task and the record on the wire'),
    ('host/tests/test_daq_api.py', 'the front door pinned: naming, reading, the record shape, the buffers'),
]

