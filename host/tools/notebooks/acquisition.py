"""Acquisition: the converters into records, a frame, and a live plot."""
from .parts import code, md, section

TITLE = 'Acquisition'
SUMMARY = 'A task configured, read in a loop, scaled into a frame, drawn live.'

SECTIONS = [
    section(
        'What the board can record',
        md("`catalogue()` is the board's list. AFE_ON off reads mid-scale and 25.00 C "
           '(invariant 9): `enable()` takes the rail, `device.close()` gives it back.'),
        code('''daq = device.daq
for row in daq.catalogue():
    print('%-16s %-8s %-4s %-10s selectable=%s'
          % (row['name'], row['kind'], row['direction'], row['unit'],
             row['selectable']))
daq.enable()
print(device.afe.state())'''),
    ),
    section(
        'The clock',
        md('The board counts cycles: `set_time_from_pc` ties them to the host clock, '
           "`reference='utc'` to NTP."),
        code('''sync = device.set_time_from_pc(reference='pc')
print(sync)'''),
    ),
    section(
        'A task, read in a loop',
        md('`start()` puts a reader thread on the link; `read(-1)` is its backlog.'),
        code('''import time

layout = daq.configure('phaseU', 'NTC', sample_rate=50)
print(daq.channel_names())
print(layout)
daq.start()
records = []
for _ in range(5):
    records.extend(daq.read(-1))
    time.sleep(0.2)
held = daq.buffered
daq.stop()
for r in records[:8]:
    print('%.3f  dt %-7s %s' % (r.start_time, '%.4f' % r.dt if r.dt is not None else 'none',
                                 [(s.name, round(s.value, 1)) for s in r.samples]))
print('records:', len(records))
shape = daq.state()
print(shape)
print(held)'''),
        md("A `Record`: `r['NTC']` the sum over `r.count`, `r.value('NTC')` the mean."),
        code('''r = records[0]
print('sum   ', r['NTC'])
print('count ', r.count)
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
        md('`frame(scaled=True)`: a column in units beside each code, through the '
           "calibration record (invariant 7). `stored` False: the schematic's arithmetic. "
           "Untared, a phase's offset reads as amps until section 6's tare."),
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
print(df[['Phase U', 'Phase U (A)', 'NTC', 'NTC (C)']].iloc[:3])'''),
        code('''from coaxial.draw.figures import figure, show

units = [c for c in df.columns if c.endswith('(A)')]
shown = units + ['DC bus (V)', 'NTC (C)']
fig, panels = figure(rows=len(shown), sharex=True)
for panel, column in zip(panels, shown):
    panel.plot(df.index, df[column])
    panel.set_ylabel(column)
panels[-1].set_xlabel('s')
show(fig)'''),
    ),
    section(
        'Currents over the switches, live',
        md('Tare with the stage off, in RAM (`save=False`: no sector erase), then 4 A turning '
           'at 3.5 Hz electrical on the model: 50 '
           'records/s is 14 points a turn. `frames()` yields the last `window` s.'),
        code('''import math

daq.configure('phaseU', 'phaseV', 'phaseW', digital=True, sample_rate=50)
print(daq.channel_names())
print(device.calibration.tare('phaseU', 'phaseV', 'phaseW', save=False))
drive = device.drive
drive.configure(source='model')
device.gates.on(bypass_sto=True, ignore_interlock=True)
drive.write(id_ref=4.0, iq_ref=0.0, theta=0.0, omega_target=2 * math.pi * 3.5)
print('holding:', drive.hold())'''),
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
device.gates.off()
drive.configure(source='adc')
whole = daq.history(scaled=True)
print(frames, 'frames drawn;', live)
print(len(whole), 'records held,', round(-whole.index.min(), 2), 's back')'''),
    ),
]

RESULTS = [
    code('''spans = [r.dt for r in records if r.dt]
counts = [r.count for r in records]
codes = run[0].channel_name
scaled = [c for c in df.columns if c.endswith(')')]
print('1. records          %d at 50/s asked, %.2f s of covered time' % (len(records), sum(spans)))
print('2. record period    %.4f s mean = %.1f /s; dt spread %.4f to %.4f s'
      % (sum(spans) / len(spans), len(spans) / sum(spans), min(spans), max(spans)))
print('3. readings summed  %d to %d per record' % (min(counts), max(counts)))
print('4. stride           %d bytes, %d analog fields; the ring holds %d records'
      % (shape['stride'], shape['fields'], shape['capacity']))
print('5. dropped          %d, host queue peak %d' % (shape['dropped'], held['peak']))
print('6. frame            %d records, %d code columns, %d scaled, of %d'
      % (len(df), len(codes), len(scaled), len(df.columns)))
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
    md('- `dt` is measured: the gap to the next stamp in its block (CYCCNT wraps every 9.04 '
       "s).\n- `dropped`: what the ring had no room for; `buffered['lost']`: what a lapped "
       'reader lost (334 records, 16 K ring, 6 s stall - FINDINGS).'),
]

BENCH = ('`stored` True once `calibration.save()` commits a record. The DC link spanned '
         'against a DMM: -32 418 ppm (2026-08-30), on 49.9k/2.2k = 78.15 V full scale '
         '(invariant 11).')

REFERENCES = [
    ('host/coaxial/rig.py', 'the front door: `daq`, `set_time_from_pc`'),
    ('host/coaxial/acquire/task.py', '`configure`, `columns`, `frame`, `frames`, `history`'),
    ('host/coaxial/acquire/record.py', 'a `Record`: the sum, the count, the mean, the struct behind a sample'),
    ('host/coaxial/simulated/acquire/', 'the stand-in this ran on, paced to real time like a board'),
    ('daq/src/daq.c', 'the engine on the board: the ring, the window, the ladder'),
    ('docs/PROTOCOL.md', 'device 6, the task and the record on the wire'),
    ('host/tests/test_daq_api.py', 'the front door pinned: naming, reading, the record shape, the buffers'),
]

