#!/usr/bin/env python3
"""Write notebook_examples/*.ipynb, and optionally execute them.

The notebooks are checked in WITH their outputs so they read without a
kernel, which makes them artefacts rather than sources: editing the
JSON by hand is how a cell's code and its printed output part company.
This file is the source. Every notebook is a list of cells here, the
markdown beside the code it explains, and `--execute` runs them so what
is checked in is what the code actually printed.

    python tools/make_notebooks.py                  # write them
    python tools/make_notebooks.py --execute        # write and run
    python tools/make_notebooks.py --execute daq_session foc_montecarlo

Executing needs a kernel and the library: `jupyter`, `nbclient`,
`pandas` and `matplotlib`. Writing needs none of them. The notebooks
run against the stand-in, so no board is needed either - the knob at
the top of each is what a reader flips at the bench.

test_structure.py parses the code cells of every notebook as one module
(`notebook_source`), so a rename that leaves a dead call in a cell
fails there. What it cannot check is that the outputs match the code,
which is what --execute is for.
"""
import argparse
import io
import json
import os
import sys

#: notebook_examples/ sits beside host/, not under it: this file is
#: host/tools/make_notebooks.py, so the repository is two levels up.
OUT_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__)))),
    'notebook_examples')

KNOB = """SIMULATED = True          # False, and PORT, at the bench
PORT = 'COM4'"""

OPEN = """from coaxial import Coaxial63100

device = Coaxial63100(port=PORT, simulated_device=SIMULATED).open()
print(device)"""


def md(text):
    return ('markdown', text)


def code(text):
    return ('code', text)


NOTEBOOKS = {}

# ---------------------------------------------------------------- daq_session
NOTEBOOKS['daq_session'] = [
    md("# DAQ session\n\nConnect, configure, set the clock, acquire in a loop."),
    code(KNOB),
    code(OPEN),
    code("""daq = device.daq
daq.open()
for row in daq.catalogue():
    print('%-16s %-8s %-4s %-10s selectable=%s'
          % (row['name'], row['kind'], row['direction'], row['unit'],
             row['selectable']))"""),
    md("AFE_ON powers the ADC reference: with it off every channel reads exact "
       "mid-scale and the NTC exactly 25.00 C (invariant 9). `enable()` takes "
       "this session's reference on the rail; `close()` releases it."),
    code("""daq.enable()
print(device.afe.state())"""),
    md("The board counts cycles, not time. `set_time_from_pc` ties the counter "
       "to the host's clock; `reference='utc'` measures that clock against NTP "
       "over the same window and takes out its offset and its rate, since a "
       "host clock is not a reference either."),
    code("""sync = device.set_time_from_pc(reference='pc')
print(sync)"""),
    code("""layout = daq.configure('phaseU', 'NTC', sample_rate=50)
print(daq.channel_names())
print(layout)"""),
    md("`start()` puts a reader thread on the link. Every `read(-1)` answers "
       "its own backlog: the first record blocks, the rest come with it."),
    code("""import time

daq.start()
records = []
for _ in range(5):
    records.extend(daq.read(-1))
    time.sleep(0.2)
daq.stop()
for r in records[:8]:
    print('%.3f  dt %.4f  %s' % (r.start_time, r.dt,
                                 [(s.name, round(s.value, 1)) for s in r.samples]))
print('records:', len(records))
print(daq.state())
print(daq.buffered)"""),
    md("A `Record` is a dict underneath: `r['NTC']` is the SUM over `r.count` "
       "readings, `r.value('NTC')` that channel's mean and `r.sample('NTC')` "
       "the struct behind it. `daq.series` and `daq.columns` are the two "
       "helpers around a whole run."),
    code("""r = records[0]
print('sum   ', r['NTC'])
print('count ', r.count, ' (r["samples"] is the same number:', r['samples'], ')')
print('mean  ', r.value('NTC'))
print('sample', r.sample('NTC'))
print('names ', r.channel_name)
print('pins  ', r.digital)"""),
    code("""cols = daq.columns(records)
print(sorted(cols))
ntc = daq.series(records, 'NTC')
seconds = daq.series(records, 'time')
print('%.1f s of NTC, first %.1f last %.1f' % (seconds[-1] - seconds[0], ntc[0], ntc[-1]))"""),
    code("""shape = daq.state()
held = daq.buffered
device.close()
print(device)"""),
    md("## Conclusions"),
    code("""spans = [r.dt for r in records if r.dt]
counts = [r.count for r in records]
print('records          %d, %.2f s of covered time' % (len(records), sum(spans)))
print('record period    %.4f s mean = %.1f /s (asked for 50)'
      % (sum(spans) / len(spans), len(spans) / sum(spans)))
print('dt spread        %.4f to %.4f s' % (min(spans), max(spans)))
print('readings summed  %d to %d per record' % (min(counts), max(counts)))
print('stride           %d bytes, %d analog fields' % (shape['stride'], shape['fields']))
print('ring holds       %d records at this stride' % shape['capacity'])
print('dropped          %d, host queue peak %d' % (shape['dropped'], held['peak']))"""),
    md("`dt` is measured, not configured: it is the gap to the next record's "
       "timestamp within the block that carried it, because what the task was "
       "asked for and what the loop managed are different numbers - which is "
       "why the board sends a count with every sum. It is per block because "
       "the stamps are raw CYCCNT and that counter wraps every 9.04 s at "
       "475 MHz; `_timed` unwraps each block it receives.\n\n"
       "`dropped` is what the ring had no room for; a reader thread that "
       "keeps up leaves it at zero."),
]

# ------------------------------------------------------- gate_drivers_session
NOTEBOOKS['gate_drivers_session'] = [
    md("# Gate drivers session\n\nDead time, arm, duty, the gate snapshot, a burst."),
    code(KNOB),
    code(OPEN),
    md("`device.gates` is the arming policy, one of it. `check()` re-reads "
       "BDTR DTG and refuses a stage with no dead time; the 2EDL8034 has no "
       "interlock of its own."),
    code("""stage = device.gates
print('dead time:', stage.dead_time())
print('check:', {k: stage.check()[k] for k in ('deadtime', 'deadtime_ns', 'period', 'gate_shorts')})
for name, volts, ok, want in stage.interlock():
    print('%-8s %-8s ok=%s want=%s' % (name, volts, ok, want))"""),
    md("The unmodified bench board reads Cinj 0.77 V and Clevel 0.06 V against "
       "3 V each, and its break input is latched: "
       "`bypass_sto=True, ignore_interlock=True` is what arms it. Both are "
       "decisions, which is why neither is silent."),
    code("""armed = stage.arm(bypass_sto=True, ignore_interlock=True)
print('pwm_enabled', armed['pwm_enabled'], 'fault', armed['fault'],
      'break_bypassed', armed['break_bypassed'])"""),
    md("A duty is ticks against `period`. With a period count the update ISR "
       "zeroes the compares after exactly that many periods: 500 is 10.000 ms "
       "at 50 kHz, where a link-timed hold was 93 to 108 ms (MINOR 8)."),
    code("""gd = device.gate_drivers
period = gd.state()['period']
tenth = (period - 1) // 10
gd.duty((tenth, 0, 0), periods=500)
snap = gd.state()
for key in ('period', 'deadtime', 'deadtime_ns', 'duty', 'requested_ticks',
            'pins', 'pins_at', 'periods_left', 'updates', 'overruns',
            'keepalive', 'worst_gap_cycles', 'gate_shorts', 'break_bypassed',
            'dcbus_raw', 'ntc_raw'):
    print('%-16s %s' % (key, snap.get(key, 'not in this reply')))"""),
    md("The gate snapshot is one IDR load with TIM1->CNT beside it: six "
       "separate reads at 50 kHz could straddle an edge and show a leg with "
       "both FETs on, the one state dead time exists to prevent. Repeated "
       "reads land at different counts, so they walk the period."),
    code("""import time

time.sleep(0.05)
print('periods_left after the hold:', gd.state()['periods_left'])
gd.duty((tenth, tenth, tenth))
snapshots = [gd.state() for _ in range(8)]
print()
print('  CNT   UL UH   VL VH   WL WH   both on?')
for s in snapshots:
    p = s['pins']
    both = [leg for leg in ('U', 'V', 'W') if p[leg + 'L'] and p[leg + 'H']]
    print('%5d %5d%3d%5d%3d%5d%3d   %s'
          % (s['pins_at'], p['UL'], p['UH'], p['VL'], p['VH'], p['WL'], p['WH'],
             ','.join(both) if both else 'no'))"""),
    md("A burst: every sweep the loop manages becomes a record until the ring "
       "is full, the pins riding the same records as the currents."),
    code("""import matplotlib.pyplot as plt

daq = device.daq
daq.enable()
burst = daq.capture('phaseU', 'phaseV', 'phaseW', records=300)
print(len(burst), 'records, dropped', daq.state()['dropped'])
df = daq.frame(burst, index='elapsed', scaled=True)
fig, (top, bottom) = plt.subplots(2, 1, sharex=True, figsize=(9, 5))
df[[c for c in df.columns if c.endswith('(A)')]].plot(ax=top)
top.set_ylabel('A')
df[[c for c in df.columns if c.startswith('TIM1_CH')]].plot(ax=bottom, legend=False)
bottom.set_ylabel('gate duty')
bottom.set_xlabel('s')
plt.show()"""),
    code("""gd.duty((0, 0, 0))
print(stage.disarm()['pwm_enabled'])
device.close()"""),
    md("## Conclusions"),
    code("""ticks = snap['duty'][0]
period = snap['period']
print('period           %d ticks = %.1f kHz centre-aligned' % (period, 237.5e6 / (2.0 * (period - 1)) / 1e3))
print('dead time        DTG %d = %d ns' % (snap['deadtime'], snap['deadtime_ns']))
print('duty asked       %d of %d ticks = %.1f %%' % (ticks, period - 1, 100.0 * ticks / (period - 1)))
print('counted hold     500 periods = %.3f ms' % (500.0 / 50e3 * 1e3))
print('gate shorts      %s' % (snap['gate_shorts'] or 'none'))
print('overruns         %d' % snap['overruns'])
print('keepalive        %d edges, worst gap %d cycles = %.1f us'
      % (snap['keepalive'], snap['worst_gap_cycles'], snap['worst_gap_cycles'] / 475.0))
walked = sorted(s['pins_at'] for s in snapshots)
both_on = sum(1 for s in snapshots for leg in ('U', 'V', 'W')
              if s['pins'][leg + 'L'] and s['pins'][leg + 'H'])
print('snapshots        %d reads, CNT %d to %d of %d'
      % (len(snapshots), walked[0], walked[-1], period))
print('both FETs on     %d of %d legs sampled' % (both_on, 3 * len(snapshots)))
print('burst            %d records, %d analog columns, %d pin columns'
      % (len(df), len([c for c in df.columns if c.endswith('(A)')]),
         len([c for c in df.columns if c.startswith('TIM1_CH')])))"""),
    md("A duty is ticks against `period`, and the compare write itself lands "
       "in about 15 ms - some 800 PWM cycles. That is why op 2 takes a period "
       "count: the update ISR zeroes the compares when it reaches zero, so "
       "500 periods is 10.000 ms exactly, where a link-timed hold measured "
       "93 to 108 ms at the FETs (FINDINGS).\n\n"
       "`gate_shorts` is measured by the board, which drives one gate pin and "
       "watches the other sink through its own pull-down; it reads no legs "
       "while armed, because the probe needs the pins. The pins and the "
       "currents ride the same records, so every point on both is one window."),
]

# ------------------------------------------------------------- shared_session
NOTEBOOKS['shared_session'] = [
    md("# Shared session\n\nTwo sessions on one port, and who else is attached."),
    code(KNOB),
    md("The first session on a port spawns a broker (`tools/session.py`) and "
       "every later one attaches to it on loopback port 8763; the broker owns "
       "the port and answers the board's 10 s deadman every 3 s for an "
       "attached client."),
    code("""from coaxial import Coaxial63100, broker

print('serving:', broker.serving())
print('clients before:', broker.clients())"""),
    code("""first = Coaxial63100(port=PORT, simulated_device=SIMULATED).open()
second = Coaxial63100(port=PORT, simulated_device=SIMULATED).open()
print(first)
print(second)
print('clients now:', broker.clients())"""),
    md("`device.link` is the port policy this session was opened with; the "
       "board's own link subsystem is `device.board.link`."),
    code("""print(first.origin.interface, '|', first.system.version()['description'])
print('opened with link=%r on %r' % (first.link, first.port))
print(second.board.link.stats())"""),
    md("Closing one session leaves the other's board alone: the stage is "
       "disarmed on the way out only by the session that armed it, or when "
       "nobody else is left."),
    md("A cooked reading claims a physical quantity, so it refuses with the "
       "front end off: mid-scale would put the NTC at exactly 25.00 C and the "
       "DC link at a plausible number that is not a measurement (invariant 9). "
       "`daq.enable()` takes a reference this session's `close()` releases."),
    code("""from coaxial.errors import DeviceStateError

try:
    print(first.analog.scan())
except DeviceStateError as exc:
    print('refused:', exc)

daq = first.daq
daq.enable()
print(first.analog.scan())"""),
    code("""second.close()
print('clients after one close:', broker.clients())
first.close()
print('clients after both:', broker.clients())"""),
    md("## Conclusions"),
    code("""for name, session in (('first', first), ('second', second)):
    print('%-7s %-22s simulated=%s' % (name, session.origin.label, session.simulated))
print('broker serving:', broker.serving())
print('port asked for: %r  link policy: %r' % (first.port, first.link))"""),
    md("On a real port the first session spawns a broker in its own process "
       "(`tools/session.py`) and every later one attaches over loopback; the "
       "broker owns the port, hands the console over once, and answers the "
       "board's 10 s deadman every 3 s for an attached client, so a session "
       "thinking between turns keeps its rail claims and its armed stage. "
       "Opening through a live broker is 0.05 s against 5.85 s starting one, "
       "which is why it lingers 45 s after the last client (FINDINGS).\n\n"
       "The stage is the board's, not a session's: `close()` disarms what "
       "this session armed, and otherwise only when nobody else is left - "
       "three switching runs once ended the moment a second session asked the "
       "board an unrelated question.\n\n"
       "`broker.clients()` is None when nothing is serving the port: asking "
       "is not using, so the count it reports attaches, asks and closes, and "
       "does not include itself."),
]

# ---------------------------------------------------------------- imu_session
NOTEBOOKS['imu_session'] = [
    md("# IMU session\n\nThe BNO085, and the three things it refuses over."),
    code(KNOB),
    code(OPEN),
    md("The part is powered by AFE_ON. Off, it still answers reads, resets and "
       "advertises while acting on no write, so the front end goes up first."),
    code("""from coaxial.errors import RigError

imu = device.imu
try:
    with imu.configuring():
        print(imu.product_id())
except RigError as exc:
    print('refused with the AFE off:', exc)
device.afe.enable()
print(device.afe.state())"""),
    md("Every operation that drives SPI2 - `feature`, `product_id`, `reset`, "
       "`probe`, `write` - needs the poll loop held; the board refuses them "
       "while it runs, because both would be masters on one bus. "
       "`configuring()` holds and resumes."),
    code("""try:
    print(imu.product_id())
except RigError as exc:
    print('refused while the loop runs:', exc)

with imu.configuring():
    print(imu.product_id())
    print(imu.pins())
    print('wake test:', imu.wake_test())"""),
    md("Set Feature: report 0x05 is the rotation vector, the interval in "
       "microseconds, 0 disables it. A write into a part still announcing "
       "itself after a reset is a write nobody acts on, which is why the "
       "firmware drains first."),
    code("""import time

ROTATION_VECTOR = 0x05
with imu.configuring():
    imu.feature(ROTATION_VECTOR, 20000)
time.sleep(0.3)
for _ in range(5):
    st = imu.state()
    q = st['quaternion']
    print(st['loop'], st['updates'], st['feature'],
          None if q is None else {k: round(v, 3) for k, v in q.items()})
    time.sleep(0.1)"""),
    md("The three vectors ride the same reply since MINOR 6, each with its own "
       "`have`; a feature nobody enabled is None, not zero."),
    code("""st = imu.state()
for name in ('accelerometer', 'gyroscope', 'magnetometer'):
    print('%-14s %s' % (name, st.get(name)))"""),
    code("""with imu.configuring():
    imu.feature(ROTATION_VECTOR, 0)
device.close()"""),
    md("## Conclusions"),
    code("""print('loop            %s' % st['loop'])
print('updates         %d monotonic, cargoes %d, errors %d'
      % (st['updates'], st['cargoes'], st['errors']))
print('feature asked   report 0x%02X at %d us, pending %s'
      % (st['feature']['report_id'], st['feature']['interval_us'],
         st['feature']['pending']))
print('last fault      %s (id %d)' % (st['last_fault'], st['last_fault_id']))
print('quaternion      %s' % st['quaternion'])
for name in ('accelerometer', 'gyroscope', 'magnetometer'):
    print('%-15s %s' % (name, 'not enabled' if st.get(name) is None else st[name]['unit']))"""),
    md("The three refusals:\n\n"
       "1. **AFE_ON low.** The rail powers the part, not just the front end. "
       "Unpowered it still drives MISO, resets and advertises - a valid "
       "276-byte advertisement reads back - while acting on no write, so "
       "every symptom presents as SPI. `Board_ImuInit` refuses while PB2 is "
       "low.\n"
       "2. **The poll loop running.** Both driving SPI2 is two masters on one "
       "bus; `configuring()` holds and resumes.\n"
       "3. **A part mid-sentence.** H_INTN stays asserted until everything "
       "queued is collected, so a write on top of a reset's three "
       "announcements loses both messages - SERVER DEVICE FAILURE. The "
       "firmware drains first, three empty reads a couple of milliseconds "
       "apart being quiet.\n\n"
       "`updates` is monotonic, so the same reading read twice is telling. "
       "`error` is the last poll's and clears on the next good read; "
       "`last_fault` is what a host polling at 5 Hz would never see."),
]

# ----------------------------------------------------------------- daq_pandas
NOTEBOOKS['daq_pandas'] = [
    md("# DAQ into pandas\n\nA run into a DataFrame, scaled by the board's record."),
    code(KNOB),
    code(OPEN),
    code("""daq = device.daq
daq.open()
daq.enable()
device.set_time_from_pc(reference='pc')
daq.configure('phaseU', 'phaseV', 'phaseW', 'DC bus', 'NTC', sample_rate=100)
daq.start()
records = daq.read(300)
daq.stop()
print(len(records), 'records;', daq.channel_names(records[0]))"""),
    md("`scaled=True` adds one column per channel in real units beside the "
       "codes, through the board's own converters and the channel trims in "
       "its calibration record (invariant 7). `stored` says whether that "
       "record was ever written or is the schematic's arithmetic; an "
       "uncalibrated board answers an empty record and every converter falls "
       "back to the compiled-in constant, which is what `name` says."),
    code("""cal = device.calibration.read()
print('record stored:', cal['stored'], ' version:', cal['version'],
      ' params held:', len(cal['params']))
for name, params in sorted(device.analog.scaling().items()):
    print('%-8s %s' % (name, params.name))
print('channel trims:', [(c['index'], c['offset_raw'], c['gain_ppm'])
                         for c in cal['channels'][:3]])"""),
    code("""df = daq.frame(records, index='elapsed', scaled=True)
df.head()"""),
    code("""df.describe().round(3)"""),
    code("""import matplotlib.pyplot as plt

units = [c for c in df.columns if c.endswith('(A)')]
axes = df[units + ['DC bus (V)', 'NTC (C)']].plot(subplots=True, figsize=(9, 8), legend=True)
axes[-1].set_xlabel('s')
plt.show()"""),
    md("The codes stay in the frame under the board's own channel names, so "
       "a tare or a span can be checked against what arrived."),
    code("""print(df[['Phase U', 'Phase U (A)', 'NTC', 'NTC (C)']].iloc[:3])
device.close()"""),
    md("## Conclusions"),
    code("""codes = [c for c in df.columns if not c.endswith(')')]
scaled = [c for c in df.columns if c.endswith(')')]
print('%d records, %d code columns, %d scaled columns' % (len(df), len(codes), len(scaled)))
for name in ('Phase U', 'DC bus', 'NTC'):
    unit = [c for c in scaled if c.startswith(name)][0]
    span = df[unit].max() - df[unit].min()
    print('%-8s codes %8.1f +/- %6.1f   %-12s %8.3f +/- %.3f  span %.3f'
          % (name, df[name].mean(), df[name].std(), unit, df[unit].mean(),
             df[unit].std(), span))"""),
    md("A code column is what arrived; the column beside it is what it means. "
       "The conversion is the board's own (invariant 7): the scaling lives in "
       "the calibration record, so `frame(scaled=True)` asks "
       "`board.analog.scaling()` rather than holding a constant, then applies "
       "that channel's offset and gain trim. Both columns stay, because what "
       "arrived and what it means are two things.\n\n"
       "With an empty record every converter falls back to the compiled-in "
       "constant and says so in `name`. `calibration.span(index, reference)` "
       "writes a gain trim against an instrument, taking the reference in the "
       "channel's own unit - mA for a phase, mV for the DC link. The DC "
       "link's stands at -32 418 ppm (FINDINGS).\n\n"
       "That divider is 49.9k/2.2k: 78.15 V full scale on a 63 V rating, 24 % "
       "of headroom so an over-rating transient is recorded rather than "
       "clipped (invariant 11)."),
]

# -------------------------------------------------------------- daq_live_plot
NOTEBOOKS['daq_live_plot'] = [
    md("# DAQ live plot\n\nCurrents over the switches, one time base, live."),
    code(KNOB),
    code(OPEN),
    md("The pins ride the same records as the analog fields, so every point "
       "on both is one window. `frames()` yields the last `window` seconds "
       "each time records arrive, indexed on seconds before now."),
    code("""daq = device.daq
daq.open()
daq.enable()
device.set_time_from_pc(reference='pc')
daq.configure('phaseU', 'phaseV', 'phaseW', digital=True, sample_rate=50)
print(daq.channel_names())"""),
    code("""import matplotlib.pyplot as plt
from IPython.display import clear_output

daq.start()
frames = 0
for df in daq.frames(window=2.0, buffer=6.0, seconds=6.0, scaled=True):
    frames += 1
    clear_output(wait=True)
    fig, (top, bottom) = plt.subplots(2, 1, sharex=True, figsize=(9, 5))
    df[[c for c in df.columns if c.endswith('(A)')]].plot(ax=top)
    top.set_ylabel('A')
    df[[c for c in df.columns if c.startswith('TIM1_CH')]].plot(ax=bottom, legend=False)
    bottom.set_ylabel('gate duty')
    bottom.set_xlabel('s before now')
    plt.show()
daq.stop()
print(frames, 'frames drawn;', daq.buffered)"""),
    md("`history()` is the buffer behind the window, already a frame."),
    code("""whole = daq.history(scaled=True)
print(len(whole), 'records held,', round(-whole.index.min(), 2), 's back')
device.close()"""),
    md("## Conclusions"),
    code("""held = daq.buffered
print('frames drawn     %d in 6 s = %.1f /s' % (frames, frames / 6.0))
print('records          %d in the buffer, %.2f s deep' % (len(whole), -whole.index.min()))
print('reader           %d reads, %d records, %.1f records/s'
      % (held['reads'], held['records'], held['rate']))
print('host queue       %d waiting, peak %d, dropped %d'
      % (held['host'], held['peak'], held['dropped']))
print('board backlog    %s at the last read' % held['backlog'])
print('columns          %s' % ', '.join(c for c in whole.columns if c.endswith('(A)')))"""),
    md("`start()` puts a reader thread on the link and it is the only thing "
       "that touches the transport while it lives, so a `print` or a redraw "
       "in this loop never sits between two round trips. Every read answers "
       "its own backlog in the same transaction, so pacing costs no extra "
       "round trip.\n\n"
       "What `frames()` yields is what is on screen, and `buffer` seconds are "
       "kept behind it - the buffer is records, so nothing is concatenated "
       "and nothing grows, and a plot that forgets to trim cannot become the "
       "bottleneck that fills the ring. The index is seconds before now, "
       "newest at 0, so the axis stands still while the data moves through "
       "it.\n\n"
       "A ring is finite: a reader that falls far enough behind for the "
       "writer to lap it loses records and is told how many in "
       "`buffered['lost']`. A terminal that stopped drawing for six seconds "
       "once overflowed a 16 K ring - 334 records (FINDINGS)."),
]

# -------------------------------------------------------------- angle_session
NOTEBOOKS['angle_session'] = [
    md("# Angle session\n\nThe A1335's registers, and whether there is a magnet."),
    code(KNOB),
    code(OPEN),
    md("The A1335 sits on SPI4 behind AFE_ON. The poll loop reads one register "
       "into shared memory; `state()` reads that record and touches no SPI."),
    code("""device.afe.enable()
angle = device.angle
st = angle.state()
print({k: st[k] for k in ('loop', 'updates', 'register_name', 'value', 'degrees', 'crc')})
print(angle.clock())
print(angle.poll_register())"""),
    md("Registers, from the reference implementation rather than the datasheet "
       "in this tree: ANG 0x20, STA 0x22, ERR 0x24, XERR 0x26, TSEN 0x28, "
       "FIELD 0x2A. A read is two frames; the CRC is reported, not checked. "
       "Direct reads need the loop held."),
    code("""from coaxial.angle import degrees, kelvin, gauss

with angle.configuring():
    for reg in (0x20, 0x22, 0x24, 0x26, 0x28, 0x2A):
        got = angle.read(reg)
        print('%-5s 0x%04X  crc %d' % (got['register_name'], got['value'], got['crc']))
    ang = angle.read(0x20)['value']
    tsen = angle.read(0x28)['value']
    field = angle.read(0x2A)['value']
print('angle  %.2f deg' % degrees(ang))
print('die    %.1f K' % kelvin(tsen))
print('field  %.0f G' % gauss(field))"""),
    md("FIELD reads about 2 G with no magnet on the real board; 300 to 1000 G "
       "is the recommended range. The stand-in reports a magnet in place."),
    code("""import time

turning = []
for _ in range(6):
    st = angle.state()
    turning.append((time.monotonic(), st['degrees'], st['updates']))
    print('%8.2f deg  updates %d' % (st['degrees'], st['updates']))
    time.sleep(0.25)
device.close()"""),
    md("## Conclusions"),
    code("""span = turning[-1][0] - turning[0][0]
moved = turning[-1][1] - turning[0][1]
print('shaft            %.2f deg over %.2f s' % (moved, span))
print('updates          %d in that window = %.0f /s'
      % (turning[-1][2] - turning[0][2], (turning[-1][2] - turning[0][2]) / span))
print('ANG  0x%04X      low 12 bits x 360/4096 = %.2f deg' % (ang, degrees(ang)))
print('TSEN 0x%04X      eighths of a kelvin    = %.1f K = %.1f C'
      % (tsen, kelvin(tsen), kelvin(tsen) - 273.15))
print('FIELD 0x%04X     %.0f gauss' % (field, gauss(field)))
print('CRC              reported, not checked')"""),
    md("Every read is two frames: the address arrives on MOSI bits 17..12 "
       "while MISO has already shifted out bits 19..16, so the answer cannot "
       "be to the frame carrying the address - asking TSEN, FIELD, TSEN in "
       "turn returned the previous register's value every time. The first "
       "frame posts the address, the second clocks the answer out.\n\n"
       "The CRC is reported and not checked: the datasheet in this tree gives "
       "the field's width and not its polynomial, and checking against a "
       "guessed one would reject good readings. The register map came from a "
       "reference implementation rather than that datasheet, which is why the "
       "polled register is settable without a rebuild.\n\n"
       "FIELD says whether there is a magnet: the real board reads about "
       "2 gauss with none, and 300 to 1000 is the recommended range. TSEN is "
       "the part's own die, not the board - it quantises at 0.125 K and is "
       "reset every time AFE_ON breaks; measured 2026-08-28 it fell 1.88 K "
       "during a run that warmed the board, which is why the NTC is the "
       "thermal observer's reference and this is not."),
]

# ------------------------------------------------------------- thermal_budget
NOTEBOOKS['thermal_budget'] = [
    md("# Thermal budget\n\nThe SOA budget, and a burst planned against it."),
    code(KNOB),
    code(OPEN),
    md("The board never calls a reading good: it reports the margin and acts "
       "at a ceiling by dropping MOE, the same path the break uses. The "
       "ceilings live in the calibration record (invariant 10)."),
    code("""thermal = device.thermal
st = thermal.state()
print('NTC', st['ntc'], ' ambient', st['ambient'], ' settled', st['settled'],
      ' every', st['sample_every_s'], 's')
for node, celsius in st['nodes'].items():
    print('%-12s %6.2f C' % (node, celsius))"""),
    md("`used` is a fraction, 0 at ambient and 1 at the node's ceiling: a "
       "temperature cannot say how close a part is without its limit beside "
       "it, so the board sends the fraction and keeps degrees on `state()`. "
       "The ceilings themselves are the record's - an uncalibrated board "
       "holds none, and a node with no ceiling is not judged."),
    code("""from coaxial.thermal import ALL_NODES

cal = device.calibration.read()
budget = thermal.budget()
limits = dict(zip(ALL_NODES, cal['soa_limit_c']))
print('ceilings in the record:', len(limits), ' throttle at', cal['soa_throttle_at'])
for node in ALL_NODES:
    print('%-12s %6.2f C  used %5.1f %%  limit %s'
          % (node, st['nodes'][node], 100.0 * budget['used'][node],
             '%.1f C' % limits[node] if node in limits else 'none in the record'))
print('worst', budget['worst_node'], ' seconds_to_limit', budget['seconds_to_limit'],
      ' throttling', budget['throttling'], ' tripped', budget['tripped'],
      ' trips', budget['trips'])"""),
    md("A burst planned against the network in `coaxial.thermal`: a node's "
       "rise over the board is `P * to_board`, reached on its own time "
       "constant `capacity * to_board`, while the board itself rises on 6.8 "
       "minutes. With a ceiling in the record the same arithmetic gives the "
       "seconds to it."),
    code("""import math
from coaxial.thermal import CFG, tau_minutes

def rise_after(node, watts, seconds):
    tau = CFG['capacity'][node] * CFG['to_board'][node]
    return watts * CFG['to_board'][node] * (1.0 - math.exp(-seconds / tau))

def seconds_to(node, watts, now_c, ceiling_c):
    tau = CFG['capacity'][node] * CFG['to_board'][node]
    top = now_c + watts * CFG['to_board'][node]
    if top <= ceiling_c:
        return None
    return -tau * math.log(1.0 - (ceiling_c - now_c) / (top - now_c))

print('board tau %.1f min' % tau_minutes())
for node in ('phase_u', 'driver_u', 'mcu'):
    tau = CFG['capacity'][node] * CFG['to_board'][node]
    print('%-9s %5.1f K/W, tau %.2f s' % (node, CFG['to_board'][node], tau))
    for watts in (5.0, 15.0, 35.0):
        line = ('%6.1f W: +%5.1f K after 100 ms, +%5.1f K steady'
                % (watts, rise_after(node, watts, 0.1), watts * CFG['to_board'][node]))
        if node in limits and cal['soa_throttle_at']:
            ceiling = limits[node] * cal['soa_throttle_at']
            t = seconds_to(node, watts, st['nodes'][node], ceiling)
            line += ('  throttle point %.1f C: %s'
                     % (ceiling, 'never' if t is None else '%.2f s' % t))
        print('   ' + line)"""),
    md("`set_sample` is how often the observer borrows AFE_ON for an NTC "
       "reading when nothing else holds the rail."),
    code("""print(thermal.set_sample(30.0, settle_s=0.5))
print(thermal.state()['sample_every_s'])
device.close()"""),
    md("## Conclusions"),
    code("""print('measured         NTC %s C' % st['ntc'])
print('model says       NTC %.2f C, error %s' % (st['expected_ntc'], st['error']))
print('ambient          %.1f C, settled %s, %d integration steps'
      % (st['ambient'], st['settled'], st['steps']))
print('other dies       afe %s C, mcu %s C, seen %.1f s ago'
      % (st['afe'], st['mcu'], st['seen_s_ago']))
print('worst node       %s at %.1f %% of its ceiling'
      % (budget['worst_node'], 100.0 * budget['worst']))
print('acting           throttling %s, tripped %s, trips %d'
      % (budget['throttling'], budget['tripped'], budget['trips']))"""),
    md("One measurement and nine estimates. The NTC is the only thermometer "
       "that sees the power stage, and `error` - the model's expected NTC "
       "minus the measured one - is the only number that says whether the "
       "parameters hold.\n\n"
       "`ntc` is None while AFE_ON is low, which is when the drivers have "
       "supply and switching is possible: the sensor and the drivers share "
       "one switch. The model then runs open on power and time.\n\n"
       "This is the narrow exception to invariant 10. The board never calls a "
       "reading good - it reports the margin and *acts*, dropping MOE at a "
       "ceiling by the path the break uses. The ceilings live in the "
       "calibration record, a limit it was given rather than invented, and a "
       "node with none is not judged.\n\n"
       "`set_sample` is how often the observer borrows AFE_ON when nothing "
       "else holds the rail; while another subsystem holds it the NTC is read "
       "every step, and an acquire is refused while the stage is armed."),
]

# -------------------------------------------------------------- thermal_model
NOTEBOOKS['thermal_model'] = [
    md("# Thermal model\n\nThe node network in Python, and how it was fitted. No board."),
    code(KNOB),
    md("`coaxial.thermal` carries the same network the firmware runs: ten "
       "nodes, driver and phase per leg, mcu, regulators, afe, and the board "
       "to ambient. Only `board_to_ambient` and `board_capacity` have a clean "
       "measurement behind them."),
    code("""from coaxial import thermal

print('ambient %.1f C' % thermal.AMBIENT)
print('board_to_ambient %.2f K/W, board_capacity %.0f J/K, tau %.1f min'
      % (thermal.CFG['board_to_ambient'], thermal.CFG['board_capacity'],
         thermal.tau_minutes()))
for node in thermal.NODES:
    print('%-12s to_board %5.1f K/W  capacity %.3f J/K'
          % (node, thermal.CFG['to_board'][node], thermal.CFG['capacity'][node]))"""),
    md("The NTC sits in the drivers' hot spot: an offset over the board taken "
       "in the passive state, and a coupling to the drivers' rise solved from "
       "the switching state."),
    code("""print(thermal.MEASURED)
print('NTC_OFFSET %.2f K' % thermal.NTC_OFFSET)
print('NTC_SEES_DRIVERS %.3f' % thermal.NTC_SEES_DRIVERS)
print('driver rise while switching %.1f K' % thermal.DRIVER_RISE_SWITCHING)
for state in thermal.STATES:
    print('%-8s %s' % (state, thermal.STATE_IS[state]))"""),
    code("""steady = thermal.steady(thermal.POWER_SWITCHING)
print('power while switching: %.2f W' % sum(thermal.POWER_SWITCHING.values()))
for node in thermal.ALL_NODES:
    print('%-12s %6.2f C' % (node, steady[node]))
print('NTC expected %.2f C' % thermal.expected_ntc(steady['board'], steady['driver_v'] - steady['board']))
for minutes in (5, 10, 25):
    print('%2d min: %.0f %% of the way to equilibrium' % (minutes, 100 * thermal.settled_fraction(minutes)))"""),
    md("`calibrate` is the fit itself: `to_board = (T_zone - T_reference) / "
       "P_zone`, one division per node, the camera's surface temperature at "
       "each source against a reference patch of soldermask at the same "
       "moment. Feed it a zone and the power in that zone and it returns "
       "that zone's spreading resistance."),
    code("""camera = {'mcu': 40.0 + 20.0, 'regulators': 40.0 + 10.1}
for node, k_per_w in sorted(thermal.calibrate(camera, board_c=40.0).items()):
    print('%-12s %6.1f K/W from the camera, %5.1f in the model'
          % (node, k_per_w, thermal.CFG['to_board'][node]))"""),
    code("""from coaxial import thermalmap

print(thermalmap.render({n: steady[n] for n in thermal.NODES}, steady['board'],
                        cells=60, colour=False, title='steady state, switching'))"""),
    md("## Conclusions"),
    code("""print('measured, against the supply and the camera:')
for name in ('board_to_ambient', 'board_capacity'):
    print('   %-20s %.2f' % (name, thermal.CFG[name]))
print()
print('the drivers, and the chain the NTC compensation hangs on:')
watts = sum(thermal.POWER_SWITCHING[n] for n in thermal.DRIVERS)
lumped = thermal.DRIVER_RISE_SWITCHING / watts
print('   %-20s %.2f W over the three legs' % ('driver power', watts))
print('   %-20s %.1f K/W lumped, %.1f per leg'
      % ('to_board', lumped, thermal.CFG['to_board']['driver_u']))
print('   %-20s %.2f W x %.1f K/W = %.1f K'
      % ('node rise switching', watts, lumped, thermal.DRIVER_RISE_SWITCHING))
print('   %-20s %.1f - %.1f - %.1f = %.1f K over the node'
      % ('NTC while switching', thermal.MEASURED['switching']['ntc'],
         thermal.MEASURED['switching']['board'], thermal.NTC_OFFSET,
         thermal.MEASURED['switching']['ntc'] - thermal.MEASURED['switching']['board']
         - thermal.NTC_OFFSET))
print('   %-20s %.3f of that rise'
      % ('NTC_SEES_DRIVERS', thermal.NTC_SEES_DRIVERS))"""),
    md("No least squares: `to_board = (T_zone - T_reference) / P_zone`, one "
       "division per node, T from a camera against a dead patch of "
       "soldermask - not the NTC, which sits in the drivers' hot spot. A "
       "spreading resistance in the laminate is a few K/W; tens means the "
       "power or the reference surface is wrong.\n\n"
       "Each state adds one power term to the one before, so the differences "
       "isolate a subsystem no single state can. Each was held 25 minutes, "
       "3.7 times the board's constant.\n\n"
       "**The camera saw one bridge zone**, so it constrains the three legs "
       "together. Per leg is three times the lumped 15.2 K/W and a third of "
       "the capacity: the three in parallel are what was measured, while one "
       "leg alone rises three times as far and three times as fast.\n\n"
       "The NTC coupling is above 1 because it sits closer to the heat than "
       "the point its node stands for; capping it at 1.0 cost 5.6 K in the "
       "switching state. The whole calibration was taken **dry**, and at "
       "100 A the shunt alone makes 35 W against the dry budget's 1.2 W."),
    md("## The envelope\n\n"
       "Four states the board actually sits in, from the same network: "
       "quiet, switching dry, switching under current, and cooling. The "
       "housekeeping - MCU and regulators - is there in all of them; the "
       "drivers' share appears with the PWM, and conduction with the "
       "current."),
    code("""import math
from coaxial import inverter

R_PHASE = inverter.RDS_ON + inverter.SHUNT
KT = 0.0435                          # N.m/A, coaxial.motor.KT_NM_PER_AMP

def split(iq=0.0, switching=True):
    \"\"\"Power per node: housekeeping, the drivers' share, conduction.\"\"\"
    out = dict(thermal.POWER_SWITCHING)
    if not switching:
        for n in thermal.DRIVERS:
            out[n] = 0.0
    rms = iq / math.sqrt(2.0)
    for n in thermal.PHASES:
        out[n] = rms * rms * R_PHASE
    return out

CEILING_C = 125.0 * 0.85             # the record's throttle point
STATES = (('quiet, no PWM', 0.0, False),
          ('switching, no current', 0.0, True),
          ('switching, 10 A of iq', 10.0, True),
          ('switching, 20 A of iq', 20.0, True),
          ('switching, 60 A of iq', 60.0, True))

print('%-24s %6s %8s %10s %11s   %s'
      % ('', 'W', 'board C', 'worst C', 'which', 'holdable'))
for name, iq, on in STATES:
    power = split(iq, on)
    at = thermal.steady(power)
    worst = max(thermal.NODES, key=lambda n: at[n])
    holds = at[worst] <= CEILING_C
    print('%-24s %6.2f %8.1f %10.1f %11s   %s'
          % (name, sum(power.values()), at['board'], at[worst],
             thermal.pretty(worst),
             'yes' if holds else 'NO - a burst, timed below'))"""),
    md("Where the worst node's equilibrium reaches the throttle point is the "
       "current the board can hold for ever; everything above it is timed."),
    code("""rms = thermal.continuous_amps(R_PHASE, CEILING_C)
iq_cont = rms * math.sqrt(2.0)
at = thermal.steady(thermal.phase_power(rms, R_PHASE))
print('continuous: %.1f A rms a phase = %.1f A of iq = %.2f N.m'
      % (rms, iq_cont, KT * iq_cont))
print('   worst node %.1f C against the %.1f C throttle point, board %.1f C'
      % (max(at[n] for n in thermal.NODES), CEILING_C, at['board']))"""),
    md("Equilibrium is where a holdable state ends up. The board's own "
       "constant decides how long that takes, and a node reaches its rise "
       "over the board far sooner:"),
    code("""tau_board = thermal.tau_minutes()
print('board       %.0f J/K over %.2f K/W = %.1f min'
      % (thermal.CFG['board_capacity'], thermal.CFG['board_to_ambient'], tau_board))
for node in ('driver_u', 'phase_u', 'mcu'):
    tau = thermal.CFG['capacity'][node] * thermal.CFG['to_board'][node]
    print('%-11s %.2f J/K over %.1f K/W = %.1f s'
          % (thermal.pretty(node), thermal.CFG['capacity'][node],
             thermal.CFG['to_board'][node], tau))
print()
for minutes in (1, 5, 10, 25):
    print('%2d min: %.0f %% of the way to equilibrium'
          % (minutes, 100 * thermal.settled_fraction(minutes)))"""),
    md("A burst is over long before either constant matters: the phase node "
       "climbs at `P / capacity` from wherever it started, and what stops it "
       "is the ceiling in the calibration record, throttled at 85 %."),
    code("""capacity = thermal.CFG['capacity']['phase_u']

print('  iq A   W a phase   K/s      s from ambient   s from a warm board (60 C)')
for iq in (20.0, 40.0, 60.0, 100.0):
    p = (iq / math.sqrt(2.0)) ** 2 * R_PHASE
    slope = p / capacity
    print('%7.0f %11.1f %7.1f %16.2f %27.2f'
          % (iq, p, slope, (CEILING_C - thermal.AMBIENT) / slope,
             (CEILING_C - 60.0) / slope))"""),
    md("And cooling: with the current off the node dumps into the board on "
       "its own constant, and the board loses what it has to ambient on 6.8 "
       "minutes. The node is cold in seconds; the board is what a second "
       "burst has to wait for."),
    code("""def cools_in(rise_k, tau_s, to_k=1.0):
    \"\"\"Seconds for an exponential fall from rise_k down to to_k.\"\"\"
    return tau_s * math.log(rise_k / to_k) if rise_k > to_k else 0.0

tau_node = capacity * thermal.CFG['to_board']['phase_u']
print('phase node, after a 60 A burst to the ceiling:')
rise = CEILING_C - thermal.AMBIENT
for target in (20.0, 5.0, 1.0):
    print('   to +%4.0f K over the board: %5.1f s' % (target, cools_in(rise, tau_node, target)))
print()
print('board, after holding a state it can hold:')
for name, iq, on in STATES:
    at = thermal.steady(split(iq, on))
    if max(at[n] for n in thermal.NODES) > CEILING_C:
        continue
    board_rise = at['board'] - thermal.AMBIENT
    print('   %-24s +%5.1f K, back to +1 K in %.0f min'
          % (name, board_rise, cools_in(board_rise, tau_board * 60.0) / 60.0))"""),
    md("The two constants are what the whole envelope rests on. A burst "
       "lives on the node's 18 seconds and is bounded by the record's "
       "ceiling; the duty cycle a bench can hold lives on the board's 6.8 "
       "minutes, which is also what a second burst waits for. Between them "
       "the observer runs at 100 ms steps and samples the NTC every 30 s, "
       "which is fast against the board and slow against a burst - so the "
       "board's estimate is anchored and the node's is open-loop over the "
       "burst, by construction.\n\n"
       "The continuous number falls out of the first table: at the current "
       "where the worst node's equilibrium reaches the throttle point, the "
       "state can be held for ever, and everything above it is timed."),
]

# ----------------------------------------------------------- loss_calculation
NOTEBOOKS['loss_calculation'] = [
    md("# Loss calculation\n\nSwitching loss from the SPICE models, no board."),
    code(KNOB),
    md("`coaxial.inverter` carries the stage's constants: FSW, the dead time, "
       "the FET's junction law (CJO, M, VJ from the LTSpice model), the power "
       "loop inductance, the shunt and the sense chain. None is a measurement "
       "on this board unless its comment says which."),
    code("""from coaxial import inverter

print('FSW %.0f Hz  TS %.1f us  T_DEAD %.1f ns  T_DEAD_SIM %.1f ns  T_MIN_PULSE %.0f ns'
      % (inverter.FSW, inverter.TS * 1e6, inverter.T_DEAD * 1e9,
         inverter.T_DEAD_SIM * 1e9, inverter.T_MIN_PULSE * 1e9))
print('RDS_ON %.1f mohm  SHUNT %.1f mohm  L_LOOP %.1f nH  Q_RING %.1f (assumed)'
      % (inverter.RDS_ON * 1e3, inverter.SHUNT * 1e3, inverter.L_LOOP * 1e9, inverter.Q_RING))
print('CJO %.1f nF  M %.2f  VJ %.2f' % (inverter.CJO * 1e9, inverter.M, inverter.VJ))
print('AFE %.3f V/A, delay %.0f ns, %.1f mA per count, noise %s A rms measured'
      % (inverter.AFE_V_PER_A, inverter.AFE_DELAY * 1e9, inverter.AFE_A_PER_COUNT * 1e3,
         inverter.NOISE_A))"""),
    md("Coss and Qoss over the link sweep, and the stored energy "
       "`E_oss = integral of v C(v) dv`: the capacitive part of a hard "
       "switched edge, dissipated once per turn-on per switch."),
    code("""import numpy as np
import matplotlib.pyplot as plt

VDC = (23.0, 33.0, 43.0, 53.0, 63.0)

def e_oss(v, steps=2000):
    grid = np.linspace(0.0, v, steps)
    return np.trapezoid(grid * inverter.coss(grid), grid)

print(' vdc   Coss nF   Qoss nC   E_oss uJ   P_coss W/switch')
for vdc in VDC:
    e = e_oss(vdc)
    print('%5.0f   %7.2f   %7.1f   %8.3f   %8.3f'
          % (vdc, inverter.coss(vdc) * 1e9, inverter.qoss(vdc) * 1e9, e * 1e6, e * inverter.FSW))
volts = np.linspace(1.0, 70.0, 200)
fig, ax = plt.subplots(figsize=(8, 4))
ax.plot(volts, inverter.coss(volts) * 1e9)
ax.set_xlabel('V'); ax.set_ylabel('Coss nF'); ax.grid(True)
plt.show()"""),
    md("The switch-node ring, the blanking margin left for the current "
       "sample, and the dead-time voltage error with its knee: what the "
       "firmware's compensation table is built from."),
    code("""print(' vdc   f_ring MHz   tau ns   settle ns   Z ohm   blanking ns   V_dt V   knee A')
for vdc in VDC:
    r = inverter.ring(vdc)
    print('%5.0f   %9.1f   %6.2f   %8.1f   %6.2f   %10.1f   %6.3f   %6.2f'
          % (vdc, r['f_hz'] / 1e6, r['tau_s'] * 1e9, r['settle_s'] * 1e9, r['z_ohm'],
             inverter.blanking(vdc) * 1e9, inverter.dead_time_volts(vdc), inverter.knee_amps(vdc)))
step, table = inverter.dt_table(43.0)
print('43 V table: step %.2f A, volts %s' % (step, [round(v, 3) for v in table]))"""),
    md("Conduction: `I^2 (RDS_ON + SHUNT)` per phase, RDS_ON at 25 C. The "
       "thermal network's measured switching figure is 1.20 W for three legs "
       "at 50 % on 24.6 V (`coaxial.thermal.POWER_SWITCHING`)."),
    code("""from coaxial import thermal

print('measured switching power, three legs 50 %% at 24.6 V: %.2f W'
      % sum(thermal.POWER_SWITCHING.values()))
print('  I A   conduction W/phase')
for amps in (1.0, 5.0, 20.0, 50.0, 100.0):
    print('%5.0f   %8.2f' % (amps, amps * amps * (inverter.RDS_ON + inverter.SHUNT)))"""),
    md("## Conclusions"),
    code("""print('MEASURED on this board:')
print('   NOISE_A            %s A rms, the phase noise floor' % (inverter.NOISE_A,))
print('   T_DEAD             %.1f ns, DTG 8, trimmed against the supply OCP' % (inverter.T_DEAD * 1e9))
print('TRACED from the schematic:')
print('   SHUNT              %.1f mohm, two 7 mohm in parallel' % (inverter.SHUNT * 1e3))
print('   AFE_V_PER_A        %.3f V/A, 4.5455 V/V x the shunt' % inverter.AFE_V_PER_A)
print('FROM THE PART AND THE SIMULATION:')
print('   CJO / M / VJ       %.1f nF / %.2f / %.2f, the VDMOS junction law'
      % (inverter.CJO * 1e9, inverter.M, inverter.VJ))
print('   RDS_ON             %.1f mohm' % (inverter.RDS_ON * 1e3))
print('   AFE_DELAY          %.0f ns, from the AFE simulation' % (inverter.AFE_DELAY * 1e9))
print('ASSUMED:')
print('   Q_RING             %.1f, the ring damping' % inverter.Q_RING)
print('   L_LOOP             %.1f nH, 0.25 nH/mm over the layout' % (inverter.L_LOOP * 1e9))
print()
print('blanking margin at 63 V: %.1f ns' % (inverter.blanking(63.0) * 1e9))"""),
    md("`ring()` gives the switch node's frequency, its decay and its "
       "impedance from L_LOOP against both FETs' Coss. `blanking()` rests on "
       "the settling time and is what says whether the current sample lands "
       "on settled current: a quarter of the modulator's unused window, minus "
       "the ring and the sense chain's delay.\n\n"
       "`knee_amps` is the current that just slews the switch node across the "
       "link inside the dead time, `2 Qoss / t_dead`: below it the output "
       "charge soft-switches the error away, which is the tanh knee the "
       "firmware's compensation table and the motor model share. `dt_table` "
       "samples that curve every half knee - eight points, held past the "
       "last - and those are record ids 34 to 42.\n\n"
       "The thermal calibration's own figure sits beside these: 1.20 W for "
       "three legs at 50 % on 24.6 V, of which roughly half fell on the "
       "supply corner - gate charge comes out of the +15V7 buck - and half on "
       "the bridge."),
]

# ----------------------------------------------------- rotor_observer_session
NOTEBOOKS['rotor_observer_session'] = [
    md("# Rotor observer session\n\nThe rotor observer on the board's own PMSM model."),
    code(KNOB),
    code(OPEN),
    md("Device 10 has two sample sources: the converters, and a motor model "
       "the firmware steps in the same interrupt. On the model the law needs "
       "no reference and no stage, and the rotor's true angle is known, so "
       "the observer can be watched against it."),
    code("""drive = device.drive
drive.source('model')
print(drive.model_param(j=2e-5, b=1e-5, load=0.0, noise=0.0))
print(drive.model())
print({k: drive.params()[k] for k in ('motor_r_uohm', 'motor_ld_nh', 'motor_lq_nh',
                                       'motor_lambda_uvs', 'motor_pole_pairs')})"""),
    code("""import time

drive.setpoint(id_ref=0.0, iq_ref=0.05, theta=0.0, omega_target=0.0)
drive.mode('sensorless')
rows = []
t0 = time.monotonic()
while time.monotonic() - t0 < 4.0:
    m = drive.model()
    rows.append((time.monotonic() - t0, m['theta'], m['theta_hat'], m['error'],
                 m['omega'], m['omega_hat']))
    time.sleep(0.05)
state = drive.state()
drive.off()
print({k: state[k] for k in ('mode', 'fault', 'omega_hat', 'iq', 'vq', 'periods',
                             'isr_cycles_max', 'exit_ticks_max', 'cycles')})"""),
    code("""import matplotlib.pyplot as plt

t = [r[0] for r in rows]
fig, axes = plt.subplots(3, 1, sharex=True, figsize=(9, 7))
axes[0].plot(t, [r[1] for r in rows], label='theta (model)')
axes[0].plot(t, [r[2] for r in rows], '.', label='theta_hat')
axes[0].set_ylabel('rad'); axes[0].legend()
axes[1].plot(t, [r[3] for r in rows]); axes[1].set_ylabel('error rad')
axes[2].plot(t, [r[4] for r in rows], label='omega')
axes[2].plot(t, [r[5] for r in rows], label='omega_hat')
axes[2].set_ylabel('rad/s el'); axes[2].set_xlabel('s'); axes[2].legend()
plt.show()"""),
    md("The window since the last take: means and deviations per field, the "
       "innovation's autocorrelation for the whiteness test, the peak current."),
    code("""w = drive.window()
print('n', w['n'], 'i_peak', w['i_peak'])
for name, f in w['fields'].items():
    print('%-4s n %-7s mean %s sd %s' % (name, f['n'], f['mean'], f['sd']))
print('rho', [round(r, 4) for r in w['rho']])
drive.model_reset()
drive.source('adc')
device.close()"""),
    md("## Conclusions"),
    code("""import math

errors = [abs(r[3]) for r in rows]
rms = math.sqrt(sum(e * e for e in errors) / len(errors))
print('rotor            %.1f to %.1f rad/s electrical'
      % (min(r[4] for r in rows), max(r[4] for r in rows)))
print('observer error   %.4f rad rms, worst %.4f rad (%.2f deg)'
      % (rms, max(errors), math.degrees(max(errors))))
print('period           %.1f us, %d cycles a period at %.0f MHz'
      % (state['ts'] * 1e6, state['isr_cycles_max'], 475.0))
print('exit             %d TIM1 ticks past the trigger, of %d in a period'
      % (state['exit_ticks_max'], 2 * 2375))
print('virtual step     sample %d, law %d, advance %d cycles'
      % (state['cycles']['sample'], state['cycles']['step'], state['cycles']['advance']))
print('window           %d periods, i_peak %.3f A' % (w['n'], w['i_peak']))"""),
    md("Device 10 has two sample sources. On the converters the law reads the "
       "injected triple; on the model it reads a PMSM the firmware steps in "
       "the same interrupt, so it runs with the AFE off, no stage, and a "
       "rotor whose true angle is known - the only way the observer's error "
       "can be measured at all.\n\n"
       "`theta_hat` and the model's `theta` ride one reply: two requests are "
       "15 ms apart, six radians at 440 rad/s, so an error across two round "
       "trips would be the link's.\n\n"
       "At -O0 with the caches off a step was 10 040 cycles, 21 us against a "
       "20 us period, and the interrupt outgrew it. With the instruction "
       "cache on and -O2 it is 6 756, and the board steps at 2 922 cycles a "
       "period with the drivers unpowered (FINDINGS, *The caches were off*).\n\n"
       "`rho` is the innovation's autocorrelation: a residual that is not "
       "white is a model that is wrong, and `ljung_box` judges it."),
]

# ------------------------------------------------------------ propeller_sweep
NOTEBOOKS['propeller_sweep'] = [
    md("# Propeller sweep\n\nThe 5230SL and its propeller against Hobbywing's stand. No board."),
    code(KNOB),
    md("`coaxial.motor` holds the Hobbywing Platinum 5230SL 190KV as the "
       "manufacturer's sheet gives it - poles and friction from the sheet, "
       "R, Ld, Lq and J estimated from the size class - and the APC20x10E "
       "fitted over the sheet's own 22-row thrust stand at 37 V."),
    code("""from coaxial.motor import PLATINUM_5230SL, APC20x10E, APC20X10E_CURVE, RATINGS, KT_NM_PER_AMP

print(PLATINUM_5230SL)
print(PLATINUM_5230SL.source)
print(APC20x10E, '-', APC20x10E.source)
print({k: RATINGS[k] for k in ('slots_poles', 'kv', 'i_max', 'p_max', 'source')})
print('Kt %.4f N.m/A' % KT_NM_PER_AMP)"""),
    md("The chain: a raised cosine to 6717 rpm and back, a speed loop, a "
       "current loop, the machine behind the propeller, at the stand's 37 V."),
    code("""import math
from coaxial.loop import Ramp, SpeedLoop, CurrentLoop, Machine
from coaxial import inverter

TWO_PI = 2.0 * math.pi
motor = PLATINUM_5230SL
top = 6717.0 * TWO_PI / 60.0
chain = (Ramp(top, rise=2.0)
         >> SpeedLoop(hz=4.0, limit=RATINGS['i_max'], motor=motor, load=APC20x10E)
         >> CurrentLoop(hz=800.0, motor=motor, vdc=37.0)
         >> Machine(motor, vdc=37.0, load=APC20x10E, noise=0.0, sub=4))
dt = 4.0 * inverter.TS
run = chain.run(seconds=4.5, dt=dt, every=25)
rpm = run['w'] * 60.0 / TWO_PI
print('reached %.0f rpm, iq peak %.1f A, v_sat %.0f %% of the run'
      % (rpm.max(), abs(run['iq']).max(), 100.0 * run['v_sat'].mean()))"""),
    code("""import numpy as np
import matplotlib.pyplot as plt

torque = APC20x10E.k * run['w'] ** 2
electrical = 1.5 * (run['vd'] * run['id'] + run['vq'] * run['iq'])
stand = np.array(APC20X10E_CURVE)
fig, (a, b) = plt.subplots(1, 2, figsize=(11, 4))
a.plot(rpm, torque, label='model, k w^2')
a.plot(stand[:, 0], stand[:, 1], 'o', label='Hobbywing stand')
a.set_xlabel('rpm'); a.set_ylabel('N.m'); a.legend(); a.grid(True)
b.plot(rpm, electrical, label='model, 1.5 (vd id + vq iq)')
b.plot(stand[:, 0], stand[:, 2], 'o', label='stand input W')
b.set_xlabel('rpm'); b.set_ylabel('W'); b.legend(); b.grid(True)
plt.show()"""),
    code("""print(' rpm    stand N.m   model N.m   stand W   model W')
for row_rpm, row_torque, row_watts in APC20X10E_CURVE[::4]:
    w = row_rpm * TWO_PI / 60.0
    near = np.argmin(np.abs(rpm - row_rpm))
    print('%5d   %8.2f   %8.2f   %8.0f   %8.0f'
          % (row_rpm, row_torque, APC20x10E.k * w * w, row_watts, electrical[near]))"""),
    md("## Conclusions"),
    code("""worst = 0.0
for row_rpm, row_torque, _watts in APC20X10E_CURVE:
    w = row_rpm * TWO_PI / 60.0
    worst = max(worst, abs(APC20x10E.k * w * w / row_torque - 1.0))
print('propeller fit    k %.3e N.m/(rad/s)^2, worst point %.1f %% off a pure square'
      % (APC20x10E.k, 100.0 * worst))
top_row = APC20X10E_CURVE[-1]
iq_at_top = top_row[1] / KT_NM_PER_AMP
print('at %d rpm         %.2f N.m needs %.1f A of q current at Kt %.4f'
      % (top_row[0], top_row[1], iq_at_top, KT_NM_PER_AMP))
print('the board         %.0f A rated; the motor %.1f A burst'
      % (100.0, RATINGS['i_max']))
print('lambda            %.5f Wb from %d KV at %d pole pairs'
      % (motor.lam, RATINGS['kv'], motor.poles))
print('Kt = 1.5 P lambda %.4f N.m/A' % (1.5 * motor.poles * motor.lam))"""),
    md("The sheet pins the **product** `Kt = 1.5 P lambda`, not the pole "
       "count: lambda from a KV goes as 1/P, so the P cancels and iq is the "
       "same whether the machine has 5 pole pairs or 14. No torque "
       "measurement gives P; it came from the winding, 24N28P being 28 poles "
       "and 14 pairs, and the observer needs it on its own because "
       "electrical speed is P times mechanical.\n\n"
       "`b` is arithmetic off the sheet: 3.0 A at 44.4 V spins it at 8436 "
       "rpm, so 133 W against 883 rad/s is 0.151 N.m of drag. R, Ld, Lq and "
       "J come from the size class; `auto_tune.ipynb` replaces them.\n\n"
       "Two ratings bound the drive: the motor's 112.5 A burst sits above the "
       "board's 100 A, so the **inverter** is the limit, and a 12S pack at "
       "full charge is 50.4 V against a 63 V rating and 78.15 V of divider "
       "scale.\n\n"
       "The model meets the stand at the stand's rpm because `k` was fitted "
       "over those 22 rows; the curve is kept whole in `APC20X10E_CURVE` so "
       "the fit can be re-done."),
]

# ----------------------------------------------------------------- speed_loop
NOTEBOOKS['speed_loop'] = [
    md("# Speed loop\n\n`coaxial.loop`'s chain, identified back out of its own run. No board."),
    code(KNOB),
    md("Blocks on one bus: a ramp, a d-axis probe (torque-free, and the one "
       "thing that lets Ld out of a fit), the speed loop, the current loop, "
       "the machine. `run` records every slot; `identify` hands the run to "
       "`coaxial.sysid` and gets the constants back with their uncertainty."),
    code("""import math
from coaxial.loop import Ramp, Probe, SpeedLoop, CurrentLoop, Machine, identify
from coaxial.motor import BENCH_MOTOR
from coaxial import inverter

TWO_PI = 2.0 * math.pi
motor = BENCH_MOTOR
print(motor)
vdc = 24.0
chain = (Ramp(top=300.0 * TWO_PI, rise=0.6)
         >> Probe(amps=1.0, hz=40.0)
         >> SpeedLoop(hz=8.0, limit=6.0, motor=motor)
         >> CurrentLoop(hz=1000.0, motor=motor, vdc=vdc)
         >> Machine(motor, vdc=vdc, noise=0.02, sub=4))
run = chain.run(seconds=1.4, dt=2.0 * inverter.TS)
print('samples', len(run['t']), ' top %.0f rad/s mech' % run['w'].max())"""),
    code("""import matplotlib.pyplot as plt

fig, axes = plt.subplots(3, 1, sharex=True, figsize=(9, 7))
axes[0].plot(run['t'], run['w_ref'], label='w_ref')
axes[0].plot(run['t'], run['w'], label='w'); axes[0].set_ylabel('rad/s'); axes[0].legend()
axes[1].plot(run['t'], run['id'], label='id'); axes[1].plot(run['t'], run['iq'], label='iq')
axes[1].set_ylabel('A'); axes[1].legend()
axes[2].plot(run['t'], run['vd'], label='vd'); axes[2].plot(run['t'], run['vq'], label='vq')
axes[2].set_ylabel('V'); axes[2].set_xlabel('s'); axes[2].legend()
plt.show()"""),
    code("""fit, got = identify(run, motor.poles)
print(fit)
print(fit.source)
print('condition %.2e  residual %.4f V' % (got['condition'], got['residual_v']))
for name, truth in (('r', motor.r), ('ld', motor.ld), ('lq', motor.lq), ('lam', motor.lam)):
    print('%-4s truth %.5g  fit %.5g  %+.1f %%  uncertainty %.1f %%  trusted %s'
          % (name, truth, got[name], 100.0 * (got[name] / truth - 1.0),
             100.0 * got['uncertainty'][name], got['trusted'][name]))"""),
    md("Without the probe the inductance column is R's: the same run, "
       "identified with `did/dt` unexcited."),
    code("""still = (Ramp(top=300.0 * TWO_PI, rise=0.6)
         >> SpeedLoop(hz=8.0, limit=6.0, motor=motor)
         >> CurrentLoop(hz=1000.0, motor=motor, vdc=vdc)
         >> Machine(motor, vdc=vdc, noise=0.02, sub=4))
run2 = still.run(seconds=1.4, dt=2.0 * inverter.TS)
_, got2 = identify(run2, motor.poles)
for name in ('r', 'ld', 'lq', 'lam'):
    print('%-4s fit %.5g  uncertainty %.1f %%  trusted %s'
          % (name, got2[name], 100.0 * got2['uncertainty'][name], got2['trusted'][name]))"""),
    md("## Conclusions"),
    code("""print('%-6s %-12s %-12s %-12s %-12s' % ('', 'truth', 'with probe', 'no probe', 'probe cost'))
for name, truth in (('r', motor.r), ('ld', motor.ld), ('lq', motor.lq), ('lam', motor.lam)):
    print('%-6s %-12.5g %-12.5g %-12.5g %+.1f %% -> %+.1f %%'
          % (name, truth, got[name], got2[name],
             100.0 * (got[name] / truth - 1.0), 100.0 * (got2[name] / truth - 1.0)))
print()
print('condition   with probe %.2e, without %.2e' % (got['condition'], got2['condition']))
print('trusted     with probe %s' % [n for n in ('r', 'ld', 'lq', 'lam') if got['trusted'][n]])
print('            without    %s' % [n for n in ('r', 'ld', 'lq', 'lam') if got2['trusted'][n]])"""),
    md("Two equations a sample, stacked; the columns are R, Ld, Lq and "
       "lambda, and each needs its own excitation. Without `did/dt` the "
       "inductance column is R's, which is why the probe is in the chain: "
       "torque-free, so the speed loop never sees it, and the one thing that "
       "lets Ld out of a fit.\n\n"
       "The per-parameter standard error says which column the run excited; "
       "one number for the whole fit does not. A V/f ramp once identified R "
       "to 0.4 %, Ld to 1.4 %, lambda to 0.1 % and **Lq to minus 73** - iq "
       "barely moved, so `Lq did/dt` had no excitation and `omega Lq iq` went "
       "collinear with lambda. The global condition was 6.5e-2 and said "
       "nothing.\n\n"
       "Two alignment facts sit behind the numbers, both found by the fit "
       "reading wrong. The bus is published before the period advances, so "
       "the loops run one period behind the machine as the firmware's "
       "pipeline does; publishing after read r at +17 % and Lq at -4 %. And "
       "the vector is aimed half a period of angle ahead, being held in the "
       "stator frame while the rotor turns through it; without that r came "
       "out at -218 % of itself at 9000 rad/s."),
]

# ------------------------------------------------------------- foc_montecarlo
_FOC_INTRO = [
    md("# FOC Monte Carlo, and two sensorless observers\n\n"
       "Three parts, in this order:\n\n"
       "1. **Sliding Mode Observer (SMO)** and **Flux Linkage Observer** - "
       "the two back-EMF observers in `coaxial.sensorless`, each simulated "
       "over drawn plants, then the hybrid that blends them and what the "
       "flux observer's magnitude measures about the machine.\n"
       "2. **The firmware's own control law**, searched over the 23-63 V "
       "link sweep with `tools/montecarlo.py`.\n"
       "3. **The envelope** both land in: speed, torque, the thermal "
       "ceiling, and reference values for an outrunner or another "
       "low-saliency machine.\n\n"
       "No board."),
    code(KNOB),
    md("`tools/montecarlo.py` runs the firmware's own C - the current loop, "
       "the injection demodulator, the rotor observer, the dead-time table - "
       "through `test_drive_core.py`'s bench against `drive_model.c`, one "
       "process per core. A plant is drawn around the 5230SL that the "
       "controller was not told about; the cost is "
       "`sigma_theta + speed_err + 10 trip`."),
    code("""import os
import sys

sys.path.insert(0, os.path.join('..', 'host', 'tools'))
import montecarlo as mc

print('link sweep', mc.VDC_SWEEP)
print('knobs', {k: v[:2] for k, v in mc.KNOBS.items()})
print('I_MAX %.0f  I_TRIP %.0f  I_H_MAX %.0f  TOP %.2f  LOST %.2f rad'
      % (mc.I_MAX, mc.I_TRIP, mc.I_H_MAX, mc.TOP, mc.LOST))
plant = mc.draw(1, 43.0)
print({k: round(v, 6) for k, v in plant.items()})"""),
]

_FOC_SEARCH = [
    md("## Part 2: the firmware's law over the link sweep\n\n"
       "A small search here so the notebook executes in minutes; the tool's "
       "defaults are 48 candidates, 16 draws and 24 refinements per link "
       "voltage."),
    code("""import time

VDCS = (23.0, 43.0, 63.0)
t0 = time.perf_counter()
with mc.pool() as p:
    best, runs = mc.search(p, vdcs=VDCS, candidates_n=6, draws=2, refine=3)
    checked = mc.verify(p, best, draws=4)
print('%d runs in %.0f s' % (len(runs) + len(checked), time.perf_counter() - t0))
best[['vdc', 'mean', 'p90', 'robust'] + list(mc.KNOBS)].round(4)"""),
    code("""import matplotlib.pyplot as plt

fig, (a, b) = plt.subplots(1, 2, figsize=(11, 4))
for vdc in VDCS:
    sub = runs[runs.vdc == vdc]
    a.scatter([vdc] * len(sub), sub.cost, s=8, alpha=0.5)
a.set_xlabel('V link'); a.set_ylabel('cost per run'); a.set_yscale('log'); a.grid(True)
b.plot(best.vdc, best['mean'], 'o-', label='mean'); b.plot(best.vdc, best.p90, 's-', label='p90')
b.set_xlabel('V link'); b.set_ylabel('cost, best tune'); b.legend(); b.grid(True)
plt.show()"""),
    md("The sensorless floor: the verification runs the best tune with the "
       "injection off through the descent and reports the speed the back-EMF "
       "alone loses the rotor at."),
    code("""bemf = checked[checked.bemf_only]
floor = bemf.groupby('vdc')['min_rpm'].agg(['mean', 'max', 'count'])
print(floor.round(0))
print('trips with injection: %d of %d' % (int(checked[~checked.bemf_only].trip.sum()),
                                          int((~checked.bemf_only).sum())))
print('sigma_theta rms, injection on, per link:')
print(checked[~checked.bemf_only].groupby('vdc')['sigma_theta'].mean().round(4))"""),
    md("## What the search found"),
    code("""import math

print('runs             %d in the search, %d in the verification' % (len(runs), len(checked)))
print('draws per point  %d plants, each drawn around the 5230SL' % 2)
print('cost             sigma_theta + speed_err + 10 x trip')
for vdc in VDCS:
    row = best[best.vdc == vdc].iloc[0]
    sub = checked[(checked.vdc == vdc) & (~checked.bemf_only)]
    print('%4.0f V  robust %.3f  bw_i %6.0f Hz  f_pll %5.0f Hz  v_inj %.3f  n_inj %d'
          % (vdc, row.robust, row.bw_i, row.f_pll, row.v_inj, int(row.n_inj)))
    print('        verified sigma_theta %.4f rad (%.2f deg), trips %d of %d'
          % (sub.sigma_theta.mean(), math.degrees(sub.sigma_theta.mean()),
             int(sub.trip.sum()), len(sub)))"""),
    md("The C is the firmware's own - current loop, demodulator, observer, "
       "dead-time table - built by `test_drive_core.py`'s harness with the "
       "host compiler and driven through ctypes against `drive_model.c`. The "
       "speed loop over it is `coaxial.loop`.\n\n"
       "**The plant is never what the controller was told**, which is what "
       "makes the cost a robustness figure: copper to 125 C on R, a quarter "
       "either way on L, saliency from barely there to 1.5, the dead time "
       "either side of the commissioned one, the AFE at its measured floor, "
       "the rotor inside the injection's pull-in.\n\n"
       "A run is a lock from a random error, a raised cosine to half the "
       "link's no-load speed, a hold, a descent. The verification repeats it "
       "with the injection off through the descent, and `min_rpm` is where "
       "the back-EMF alone lost the rotor - the sensorless floor."),
    md("## Expected performance"),
    code("""import math
from coaxial import inverter
from coaxial.motor import PLATINUM_5230SL, APC20x10E, RATINGS, KT_NM_PER_AMP

TWO_PI = 2.0 * math.pi
motor = PLATINUM_5230SL
kt = 1.5 * motor.poles * motor.lam
print('Kt               %.4f N.m/A  (the sheet: %.4f)' % (kt, KT_NM_PER_AMP))
print('board rating     %.0f A instantaneous, %.0f V link' % (100.0, 63.0))
print('motor rating     %.1f A burst for %.0f s, %.0f W' % (RATINGS['i_max'], RATINGS['t_i_max'], RATINGS['p_max']))
print('the limit        %s' % ('the inverter' if 100.0 < RATINGS['i_max'] else 'the motor'))
print()
print('with an APC20x10E on the shaft, the operating point:')
print('link   no-load rpm   held rpm   N.m    shaft kW   iq A   phase A rms   limited by')
for vdc in (23.0, 33.0, 43.0, 53.0, 63.0):
    w_e = inverter.V_FRAC * vdc / math.sqrt(3.0) / motor.lam
    no_load = w_e / motor.poles
    # Where k w^2 meets the torque the current ceiling makes, or the link's
    # own speed ceiling - whichever comes first.
    by_current = math.sqrt(kt * 100.0 / APC20x10E.k)
    wm = min(by_current, no_load)
    torque = APC20x10E.k * wm * wm
    iq = torque / kt
    print('%4.0f V %10.0f %11.0f %7.2f %10.2f %7.1f %12.1f   %s'
          % (vdc, no_load * 60.0 / TWO_PI, wm * 60.0 / TWO_PI, torque,
             torque * wm / 1000.0, iq, iq / math.sqrt(2.0),
             'the link' if wm < by_current else 'the 100 A rating'))"""),
    md("The no-load speed is what the modulator can hold against the "
       "back-EMF: `V_FRAC Vdc/sqrt(3) / lambda`, with V_FRAC 0.95 of the link "
       "the vector may use. The held speed is where the propeller's `k w^2` "
       "meets the torque the current ceiling makes, or that no-load speed - "
       "whichever comes first, which is what the last column names. Torque "
       "and top speed do not coexist: the shaft power column is at the "
       "operating point, not the product of the two ceilings."),
    code("""from coaxial import sensorless

print('what the search chose, and what it implies:')
for vdc in VDCS:
    row = best[best.vdc == vdc].iloc[0]
    sub = checked[(checked.vdc == vdc) & (~checked.bemf_only)]
    bemf_rpm = floor.loc[vdc, 'mean'] if vdc in floor.index else float('nan')
    w_e = inverter.V_FRAC * vdc / math.sqrt(3.0) / motor.lam
    top_rpm = w_e / motor.poles * 60.0 / TWO_PI
    f_inj = 50e3 / (2.0 * int(row.n_inj))
    print()
    print('%4.0f V' % vdc)
    print('   current loop   %.0f Hz of the %.0f Hz two periods of delay allow'
          % (row.bw_i, 50e3 * 0.05))
    print('   observer       %.0f Hz, zeta %.2f' % (row.f_pll, row.zeta))
    print('   injection      %.0f Hz at %.1f %% of Vdc/sqrt3, %d period(s)'
          % (f_inj, 100.0 * row.v_inj, int(row.n_inj)))
    print('   blend          %.0f to %.0f rad/s electrical = %.0f to %.0f rpm'
          % (row.w_lo, row.w_lo * row.w_ratio,
             row.w_lo / motor.poles * 60.0 / TWO_PI,
             row.w_lo * row.w_ratio / motor.poles * 60.0 / TWO_PI))
    print('   held           sigma_theta %.2f deg electrical, %.3f deg mechanical'
          % (math.degrees(sub.sigma_theta.mean()),
             math.degrees(sub.sigma_theta.mean()) / motor.poles))
    print('   back-EMF alone loses the rotor at %.0f rpm = %.1f %% of the %.0f rpm this link holds'
          % (bemf_rpm, 100.0 * bemf_rpm / top_rpm, top_rpm))"""),
    md("The current loop's ceiling is a twentieth of the sampling rate, two "
       "periods of pipeline delay wanting the phase margin, and the injection "
       "stays eight times above whatever it ended up at or the two fight. The "
       "observer's bandwidth is no knob: `kalman_gains` iterates the Riccati "
       "recursion to its fixed point, so the measured noise sets it and "
       "quieter shunts give a faster observer.\n\n"
       "`w_lo .. w_hi` is where the estimate hands over from injection to "
       "back-EMF, and `min_rpm` is the other side of that number.\n\n"
       "The floor is the AFE's: the demodulated angle error is `sigma_i` "
       "over the demodulator's gain, and that gain is "
       "`V_inj Ts (Lq - Ld) / (2 Ld Lq)`. The 5230SL's saliency of about 1.3 "
       "is what the zero-speed method rests on; less, and `decide` picks I/f "
       "at its 10 dB threshold."),
]

_FOC_OBSERVERS = [
    md("## Part 1: two back-EMF observers\n\n"
       "The firmware holds the rotor at rest by injecting and demodulating, "
       "and blends to back-EMF above `w_lo`. The **Sliding Mode Observer "
       "(SMO)** and the **Flux Linkage Observer** are the two ways to do "
       "that upper half. Both live in `coaxial.sensorless`, both are fed by "
       "nothing but the phase voltages and currents, and both are run here "
       "over plants drawn with the same tolerances the search uses in part "
       "2.\n\n"
       "This first sweep runs them side by side; the two sections after it "
       "take each one on its own."),
    code("""import math
from coaxial import inverter, sensorless
from coaxial.loop import CurrentLoop, Machine, Signals
from coaxial.motor import PLATINUM_5230SL, Parameters

TWO_PI = 2.0 * math.pi
motor = PLATINUM_5230SL
SPEEDS = (20.0, 50.0, 100.0, 200.0, 500.0, 1000.0, 2000.0)
PLANTS = 5
#: Where the blend hands over from sliding mode to flux linkage, rad/s
#: electrical - a band about the crossover, not a threshold.
BLEND_BAND = (400.0, 1400.0)

def observe(plant, w_e, seconds=0.4):
    \"\"\"Both observers against a rotor held at w_e, error in degrees rms.\"\"\"
    fitted = Parameters(name='drawn', r=plant['r'], ld=plant['ld'],
                        lq=plant['lq'], lam=plant['lambda'],
                        poles=int(plant['pole_pairs']), measured=False)
    dt = 2.0 * inverter.TS
    loop = CurrentLoop(hz=800.0, motor=fitted, vdc=plant['vdc'])
    machine = Machine(fitted, vdc=plant['vdc'], noise=plant['noise'], sub=4,
                      locked=True)
    machine.motor.omega = w_e
    smo = sensorless.SlidingModeObserver(
        fitted.r, fitted.ld, k=1.5 * fitted.lam * abs(w_e) + 1.0)
    flux = sensorless.FluxObserver(fitted.r, fitted.ld, wc=20.0)
    s = Signals()
    s.iq_ref = 2.0
    got = {'smo': [], 'flux': [], 'gap': [], 'blend': []}
    # Each observer's own residual - no truth in either. The sliding-mode
    # observer is only estimating anything while its model current is on
    # the measured one; the flux observer's rotor flux should have the
    # magnitude lambda, and does not when the integrator has drifted.
    res = {'smo': [], 'flux': []}
    rows = []
    for step in range(int(seconds / dt)):
        s.t = step * dt
        loop(s, dt)
        machine(s, dt)
        c, sn = math.cos(s.theta), math.sin(s.theta)
        va, vb = s.vd * c - s.vq * sn, s.vd * sn + s.vq * c
        ia, ib = s.id * c - s.iq * sn, s.id * sn + s.iq * c
        th_s = smo.update(va, vb, ia, ib, dt)
        th_f = flux.update(va, vb, ia, ib, dt)
        if s.t > 0.5 * seconds:
            got['smo'].append(sensorless._wrap(th_s - s.theta))
            got['flux'].append(sensorless._wrap(th_f - s.theta))
            got['gap'].append(sensorless._wrap(th_s - th_f))
            res['smo'].append(math.hypot(smo.i_alpha - ia, smo.i_beta - ib)
                              / max(plant['noise'], 1e-3))
            psi = math.hypot(flux.psi_alpha - fitted.ld * ia,
                             flux.psi_beta - fitted.ld * ib)
            res['flux'].append(abs(psi - fitted.lam) / fitted.lam)
            rows.append((th_s, th_f, s.theta, abs(smo.omega)))
    # The blend, ramped over BLEND_BAND on the observer's own speed and
    # applied to the unit vectors - an angle is not a quantity you
    # average. All sliding mode below the band, all flux above it.
    lo, hi = BLEND_BAND
    share = 0.0
    for th_s, th_f, truth, w in rows:
        f = min(1.0, max(0.0, (w - lo) / (hi - lo)))
        x = (1.0 - f) * math.cos(th_s) + f * math.cos(th_f)
        y = (1.0 - f) * math.sin(th_s) + f * math.sin(th_f)
        got['blend'].append(sensorless._wrap(math.atan2(y, x) - truth))
        share += (1.0 - f) / len(rows)
    out = {k: math.degrees(math.sqrt(sum(e * e for e in v) / len(v)))
           for k, v in got.items()}
    out['share'] = share
    out['res_smo'] = sum(res['smo']) / len(res['smo'])
    out['res_flux'] = sum(res['flux']) / len(res['flux'])
    return out

plants = [mc.draw(90 + i, 43.0) for i in range(PLANTS)]
print('  rad/s el    rpm      SMO deg rms       flux deg rms     apart   blend')
print('                     median  worst     median  worst       deg  median')
table = []
for w_e in SPEEDS:
    rows = [observe(p, w_e) for p in plants]
    def column(key):
        return sorted(r[key] for r in rows)
    smo, flux = column('smo'), column('flux')
    gap, blend, share = column('gap'), column('blend'), column('share')
    table.append((w_e, smo, flux, gap, blend, share,
                  column('res_smo'), column('res_flux')))
    print('%10.0f %6.0f %11.1f %6.1f %10.1f %6.1f %9.1f %7.1f'
          % (w_e, w_e / motor.poles * 60.0 / TWO_PI,
             smo[len(smo) // 2], smo[-1], flux[len(flux) // 2], flux[-1],
             gap[len(gap) // 2], blend[len(blend) // 2]))"""),
    md("A torque command at an angle error of `eps` delivers `cos(eps)` of "
       "itself: 20 degrees electrical costs 6 %, 40 degrees costs 23 %. "
       "Taking 20 degrees as the line each has to stay inside:"),
    code("""CRITERION_DEG = 20.0
COLUMN = {'smo': 1, 'flux': 2, 'blend': 4}

def band_of(which):
    \"\"\"(lowest, highest) swept speed whose worst plant is inside it.\"\"\"
    inside = [row[0] for row in table if row[COLUMN[which]][-1] <= CRITERION_DEG]
    return (min(inside), max(inside)) if inside else (None, None)

def rpm(w_e):
    return w_e / motor.poles * 60.0 / TWO_PI

for which, name in (('smo', 'sliding mode'), ('flux', 'flux linkage'),
                    ('blend', 'the two blended')):
    lo, hi = band_of(which)
    print('%-14s inside %.0f deg from %5.0f to %5.0f rad/s = %4.0f to %5.0f rpm'
          % (name, CRITERION_DEG, lo, hi, rpm(lo), rpm(hi)))"""),
    md("## Sliding Mode Observer (SMO)\n\n"
       "The stator's current equation run as a model, with a switching term "
       "driving the model's current onto the measured one. Once it slides, "
       "that term is the only thing the model was missing, which is the "
       "back-EMF; low-passing it is the estimate, and "
       "`e = lambda w (-sin, cos)` gives the angle.\n\n"
       "`k` has to exceed the back-EMF the machine can make, so it is sized "
       "from `lambda w` rather than tuned. It degrades at **both** ends and "
       "for different reasons: low down the back-EMF is small against `R i` "
       "and the boundary layer, so the error grows as the signal shrinks; "
       "high up the lag `atan(w/wc)` becomes a large correction - 76 degrees "
       "at 2000 rad/s with `wc` 500 - and a large correction rests heavily "
       "on the speed estimate under it.\n\n"
       "It needs R and L and is the least sensitive of the two to them: the "
       "switching term absorbs what the model gets wrong, which is the point "
       "of it."),
    code("""print('sliding mode, deg rms, over %d drawn plants' % PLANTS)
print('  rad/s el    rpm    median   worst   current residual, in sigma_i')
for row in table:
    w_e, smo, res = row[0], row[1], row[6]
    print('%10.0f %6.0f %9.1f %7.1f %19.2f'
          % (w_e, rpm(w_e), smo[len(smo) // 2], smo[-1], res[len(res) // 2]))
lo, hi = band_of('smo')
print()
print('inside %.0f deg from %.0f rpm to %.0f rpm' % (CRITERION_DEG, rpm(lo), rpm(hi)))"""),
    md("## Flux Linkage Observer\n\n"
       "`psi = integral of (v - R i)` is the stator's flux linkage and "
       "`psi - L i` is the rotor's, whose angle is the rotor's. Nothing is "
       "differentiated and nothing switches, so it is quiet where it "
       "works.\n\n"
       "The integrator is the whole difficulty. A pure one walks away on any "
       "offset in v, in the current, or in R, so this leaks at `wc` - which "
       "costs exactly what it saves: the estimate comes out "
       "`sqrt(1 + (wc/w)^2)` short and `atan(wc/w)` late, and the correction "
       "is only as good as the speed it rests on. At `w = wc` that "
       "correction is 45 degrees, and below it the observer has nothing.\n\n"
       "Above the corner it is the better of the two by a growing margin: no "
       "lag worth compensating, and **lambda falls out of it**. The rotor "
       "flux's magnitude should be lambda, so the observer can check itself "
       "without a reference - which the sliding-mode observer cannot do from "
       "its angle alone."),
    code("""print('flux linkage, deg rms, over %d drawn plants' % PLANTS)
print('  rad/s el    rpm    median   worst   wc/w correction   |psi| off lambda')
for row in table:
    w_e, flux, res = row[0], row[2], row[7]
    print('%10.0f %6.0f %9.1f %7.1f %13.1f deg %15.1f %%'
          % (w_e, rpm(w_e), flux[len(flux) // 2], flux[-1],
             math.degrees(math.atan2(20.0, w_e)), 100.0 * res[len(res) // 2]))
lo, hi = band_of('flux')
print()
print('inside %.0f deg from %.0f rpm to %.0f rpm' % (CRITERION_DEG, rpm(lo), rpm(hi)))"""),
    md("## Running both, and the hybrid\n\n"
       "They fail at opposite ends, so there are three ways to use the pair, "
       "in rising order of what they ask for.\n\n"
       "**Switch** on speed, with hysteresis. The cheapest and the only one "
       "needing no extra state - but a threshold where both are marginal is "
       "a threshold the estimate chatters across, and the estimate is what "
       "the commutation rests on.\n\n"
       "**Blend** over a band on the observer's own speed, the shape the "
       "firmware already uses between injection and back-EMF over "
       "`w_lo .. w_hi`. The two disagree by a few degrees in the overlap, "
       "and a step of that size in the commutation angle is a step in "
       "torque, so a ramped weight removes it for the cost of a multiply. "
       "The weight is applied to the unit vectors, not to the angles.\n\n"
       "**Weight on each observer's own residual.** Neither needs truth to "
       "say how it is doing: the sliding-mode observer is estimating nothing "
       "unless its model current is on the measured one, and the flux "
       "observer's rotor flux should have magnitude lambda. Both columns are "
       "printed in the sections above."),
    code("""print('the blend, %.0f to %.0f rad/s = %.0f to %.0f rpm'
      % (BLEND_BAND[0], BLEND_BAND[1], rpm(BLEND_BAND[0]), rpm(BLEND_BAND[1])))
print('  rad/s el    rpm   SMO share   blend deg rms   best single   gained')
for row in table:
    w_e, smo, flux, blend, share = row[0], row[1], row[2], row[4], row[5]
    single = min(smo[len(smo) // 2], flux[len(flux) // 2])
    got = blend[len(blend) // 2]
    print('%10.0f %6.0f %10.2f %14.1f %13.1f %8.1f %%'
          % (w_e, rpm(w_e), share[len(share) // 2], got, single,
             100.0 * (single - got) / single))
print()
print('cost, per step, against the drive step\\'s own %d cycles:' % 2921)
print('   sliding mode   2 integrators, 2 saturations, 2 low-passes, one atan2')
print('   flux linkage   2 integrators, one atan2, a sqrt and an atan')
print('   the blend      one ramp, two multiplies, one atan2')
print('   headroom       %d cycles a period at %.0f kHz'
      % (2 * 2375 - 2921, 1e-3 / inverter.TS))"""),
    md("The blend is not a compromise between the two - in the overlap it "
       "beats both, because the two errors are not the same error. One is a "
       "lag correction resting on a speed estimate; the other is an "
       "integrator's leak. Averaging two partly independent errors is worth "
       "more than picking the better one, and the `gained` column is what "
       "that is worth here.\n\n"
       "Weighting on the residuals instead was tried and **does not work as "
       "written**: the sliding-mode observer's current residual is dominated "
       "by the AFE's own noise, which its low-pass rejects but the residual "
       "still shows, so referring that residual to an angle error "
       "over-states it by orders of magnitude and the weight collapses onto "
       "the flux observer at every speed. A residual that maps to angle "
       "error needs the filter's rejection in it, which is more than a "
       "normalisation.\n\n"
       "What the residuals are good for as they stand is validity, not "
       "weight. The current residual says whether the sliding-mode observer "
       "is on its surface at all, and `|psi|` against lambda says whether "
       "the flux integrator has drifted - two conditions a speed threshold "
       "cannot see, and the same two a supervisor would trip on."),
    md("## Maximising what the machine tells you\n\n"
       "The angle is not all that is in those residuals. The flux observer's "
       "rotor flux has a **magnitude** as well as a direction, and that "
       "magnitude is lambda - the one constant that says what the magnets "
       "are doing. Nothing else on this board can see them: the NTC is on "
       "the PCB, and the rotor is on the other side of an air gap.\n\n"
       "So the observer that commutates the machine is also measuring it, "
       "for the cost of one `hypot` a step. The question is what corrupts "
       "that measurement, and the answer is R: the flux integrates "
       "`v - R i`, so an error in R lands in the flux, and it lands hardest "
       "where `R i` is a large share of `v` - at low speed."),
    code("""def identify_lambda(plant, w_e, r_error=0.0, seconds=0.4):
    \"\"\"What the flux observer's magnitude says lambda is, with R off by
    `r_error` as a fraction. Returns the estimate over the truth.\"\"\"
    fitted = Parameters(name='drawn', r=plant['r'] * (1.0 + r_error),
                        ld=plant['ld'], lq=plant['lq'], lam=plant['lambda'],
                        poles=int(plant['pole_pairs']), measured=False)
    truth = Parameters(name='truth', r=plant['r'], ld=plant['ld'],
                       lq=plant['lq'], lam=plant['lambda'],
                       poles=int(plant['pole_pairs']), measured=False)
    dt = 2.0 * inverter.TS
    loop = CurrentLoop(hz=800.0, motor=fitted, vdc=plant['vdc'])
    machine = Machine(truth, vdc=plant['vdc'], noise=plant['noise'], sub=4,
                      locked=True)
    machine.motor.omega = w_e
    flux = sensorless.FluxObserver(fitted.r, fitted.ld, wc=20.0)
    s = Signals()
    s.iq_ref = 2.0
    seen = []
    for step in range(int(seconds / dt)):
        s.t = step * dt
        loop(s, dt)
        machine(s, dt)
        c, sn = math.cos(s.theta), math.sin(s.theta)
        va, vb = s.vd * c - s.vq * sn, s.vd * sn + s.vq * c
        ia, ib = s.id * c - s.iq * sn, s.id * sn + s.iq * c
        flux.update(va, vb, ia, ib, dt)
        if s.t > 0.5 * seconds:
            seen.append(math.hypot(flux.psi_alpha - fitted.ld * ia,
                                   flux.psi_beta - fitted.ld * ib))
    return (sum(seen) / len(seen)) / truth.lam

plant = plants[0]
print('lambda recovered from |psi_r|, as a fraction of the truth')
print('  rad/s el    rpm    R exact   R +30 %%   R -30 %%')
for w_e in SPEEDS:
    row = [identify_lambda(plant, w_e, e) for e in (0.0, 0.3, -0.3)]
    print('%10.0f %6.0f %10.3f %9.3f %9.3f' % (w_e, rpm(w_e), row[0], row[1], row[2]))"""),
    md("Two things fall out of that table, and together they are the "
       "procedure.\n\n"
       "**Lambda is recoverable at speed and only at speed.** High up, "
       "`v` is dominated by the back-EMF and a 30 % error in R barely moves "
       "the magnitude; low down, `R i` is most of `v` and the same error "
       "swamps it. So the magnet's state is measured where the machine is "
       "already turning - which is where it matters, since that is where "
       "the flux observer is the one commutating.\n\n"
       "**And R is recoverable once lambda is known.** Run it the other way: "
       "at low speed, with lambda fixed at what the high-speed measurement "
       "said, the magnitude error is a function of R alone. The same "
       "observer, the same `hypot`, run at two speeds, separates the two "
       "constants that the thermal model most wants - because R is the "
       "winding's temperature and lambda is the magnets'.\n\n"
       "That is the information the machine gives up for free, and it is "
       "worth naming what it buys: the board's thermal observer has one "
       "measurement, the NTC, which sits on the PCB in the drivers' hot "
       "spot. It estimates the phase node and it cannot see the motor at "
       "all. A winding resistance and a magnet flux tracked online are two "
       "more anchors, on the other side of the gap, from an observer the "
       "drive is running anyway."),
]

_FOC_ENVELOPE = [
    md("## Part 3: speed and torque against the tolerances\n\n"
       "Every number below moves with lambda, and the plants are drawn with "
       "lambda at +/- 10 %: speed goes as `1/lambda` and torque as `lambda`, "
       "so the two ends of the tolerance are the two ends of the envelope."),
    code("""LAMBDA_SPREAD = (0.9, 1.1)          # what mc.draw draws over
I_RATING = 100.0                    # the board, instantaneous

print('link   no-load rpm            torque at 100 A       shaft kW at that torque')
print('         min    max            min     max            min     max')
for vdc in (23.0, 43.0, 63.0):
    speeds, torques, powers = [], [], []
    for f in LAMBDA_SPREAD:
        lam = motor.lam * f
        w_e = inverter.V_FRAC * vdc / math.sqrt(3.0) / lam
        kt = 1.5 * motor.poles * lam
        speeds.append(w_e / motor.poles * 60.0 / TWO_PI)
        torques.append(kt * I_RATING)
        # At the propeller's operating point, not at both ceilings at once.
        wm = min(math.sqrt(kt * I_RATING / APC20x10E.k), w_e / motor.poles)
        powers.append(APC20x10E.k * wm * wm * wm / 1000.0)
    print('%4.0f V %7.0f %6.0f %14.2f %7.2f %14.2f %7.2f'
          % (vdc, min(speeds), max(speeds), min(torques), max(torques),
             min(powers), max(powers)))"""),
    code("""from coaxial import thermal

R_PHASE = inverter.RDS_ON + inverter.SHUNT
CEILING_C = 125.0 * 0.85            # the record's throttle point
AMBIENT_C = thermal.AMBIENT
capacity = thermal.CFG['capacity']['phase_u']

# The same definition thermal_model.ipynb uses: the worst node's
# equilibrium against the ceiling, housekeeping and drivers included.
i_rms = thermal.continuous_amps(R_PHASE, CEILING_C)
iq_cont = i_rms * math.sqrt(2.0)
kt = 1.5 * motor.poles * motor.lam
print('continuous, worst node at the %.1f C throttle point:' % CEILING_C)
print('   %.1f A rms a phase = %.1f A of iq = %.2f N.m'
      % (i_rms, iq_cont, kt * iq_cont))
print()
print('burst from ambient, the node on its own capacity (%.2f J/K):' % capacity)
print('   iq A   A rms   W a phase   K/s      s to the ceiling')
for iq in (20.0, 40.0, 60.0, 100.0):
    rms = iq / math.sqrt(2.0)
    p = rms * rms * R_PHASE
    print('%7.0f %7.1f %11.1f %7.1f %17.2f'
          % (iq, rms, p, p / capacity, (CEILING_C - AMBIENT_C) / (p / capacity)))"""),
    md("The rate at the start of a burst is `P / capacity` and nothing else: "
       "the node holds a third of a joule per kelvin, so it moves in seconds "
       "while the board under it moves in 6.8 minutes. That separation is why "
       "a burst is planned against `seconds_to_limit` rather than a steady "
       "state, and why the envelope throttles at 85 % of the ceiling rather "
       "than waiting for it.\n\n"
       "The network was fitted **dry** - nothing on the phases, nothing "
       "through the hot swap - so the phase node's `to_board` is the first "
       "number to re-fit with current flowing: `(T_zone - T_board) / P` off a "
       "camera, into `thermal.set_node`. Everything in this section moves "
       "with it."),
    md("## The envelope"),
    code("""def band(f):
    \"\"\"(min, max) of f over the lambda tolerance.\"\"\"
    got = [f(motor.lam * s) for s in LAMBDA_SPREAD]
    return min(got), max(got)

top = band(lambda lam: inverter.V_FRAC * 63.0 / math.sqrt(3.0) / lam
           / motor.poles * 60.0 / TWO_PI)
low = band(lambda lam: inverter.V_FRAC * 23.0 / math.sqrt(3.0) / lam
           / motor.poles * 60.0 / TWO_PI)
peak = band(lambda lam: 1.5 * motor.poles * lam * I_RATING)
cont = band(lambda lam: 1.5 * motor.poles * lam * iq_cont)
smo_rpm = band_of('smo')[0]
flux_rpm = band_of('flux')[0]
bemf = floor.loc[43.0, 'mean'] if 43.0 in floor.index else float('nan')

print('%-34s %s' % ('MAXIMUM SPEED, no load',
                    '%.0f to %.0f rpm at 63 V' % top))
print('%-34s %s' % ('', '%.0f to %.0f rpm at 23 V' % low))
print('%-34s %s' % ('MINIMUM SPEED, injection', '0 rpm - it holds at rest'))
print('%-34s %.0f rpm' % ('   back-EMF alone, firmware', bemf))
print('%-34s %.0f rpm' % ('   sliding mode, 20 deg', smo_rpm / motor.poles * 60.0 / TWO_PI))
print('%-34s %.0f rpm' % ('   flux linkage, 20 deg', flux_rpm / motor.poles * 60.0 / TWO_PI))
print('%-34s %s' % ('PEAK TORQUE, 100 A instantaneous',
                    '%.2f to %.2f N.m' % peak))
print('%-34s %s' % ('CONTINUOUS TORQUE, thermal',
                    '%.2f to %.2f N.m (%.1f A of iq)' % (cont[0], cont[1], iq_cont)))
print('%-34s %.2f s at 100 A, %.2f s at 60 A'
      % ('BURST TO THE THROTTLE POINT',
         (CEILING_C - AMBIENT_C) / ((100.0 / math.sqrt(2.0)) ** 2 * R_PHASE / capacity),
         (CEILING_C - AMBIENT_C) / ((60.0 / math.sqrt(2.0)) ** 2 * R_PHASE / capacity)))
print('%-34s %.1f to %.1f deg electrical'
      % ('ANGLE ERROR, injection held',
         math.degrees(checked[~checked.bemf_only].sigma_theta.min()),
         math.degrees(checked[~checked.bemf_only].sigma_theta.max())))"""),
    md("Speed goes as `1/lambda` and torque as `lambda`, so the +/- 10 % the "
       "plants are drawn over puts a 22 % spread on both. The link sets the "
       "speed and the board's 100 A sets the torque; neither is the motor, "
       "whose 112.5 A burst sits above the rating.\n\n"
       "**Continuous torque is a fifth of peak**, and that is the thermal "
       "network rather than the stage. Everything above it is a burst "
       "measured in seconds, which is what `thermal_budget.ipynb` plans "
       "against `seconds_to_limit`.\n\n"
       "**The floor is the injection's**, not an observer's: the injection "
       "holds at rest and the back-EMF observers do not."),
    md("## Reference values for a low-saliency machine\n\n"
       "An outrunner is the case this has to survive. Saliency is what the "
       "injection lives on - the demodulator's gain goes as `Lq - Ld` - and "
       "an outrunner's magnets sit on the rotor surface, so there is little "
       "of it. The plants here are drawn from 1.05 to 1.5, which spans the "
       "case where injection works and the case where it does not."),
    code("""from coaxial import sensorless

k = {'ld': motor.ld, 'lq': motor.lq, 'sigma_i': max(inverter.NOISE_A),
     'vdc': 43.0, 'r': motor.r, 'i_max': I_RATING}
loop_hz = sensorless.current_loop(k['r'], k['ld'], 1.0 / inverter.TS,
                                  k['sigma_i'], k['vdc'])['bw_hz']
print('             ---- 5 A of HF headroom ----   ---- 1 A ----')
print('saliency   SNR dB   method    i_h peak A   SNR dB   method')
for ratio in (1.02, 1.05, 1.15, 1.32, 1.5):
    lq = motor.ld * ratio
    row = [ratio]
    for cap in (5.0, 1.0):
        choice = sensorless.choose_injection(
            motor.ld, lq, k['sigma_i'], 1.0 / inverter.TS, 50.0, k['vdc'],
            i_h_max=cap, bw_i_hz=loop_hz)
        row.append(choice)
    wide, tight = row[1], row[2]
    print('%8.2f %8.1f   %-9s %10.2f %8.1f   %-9s'
          % (ratio, wide['snr_db'], sensorless.decide(wide['snr_db']),
             wide['i_h_peak'],
             tight['snr_db'], sensorless.decide(tight['snr_db'])))
print()
print('the 5230SL sits at %.2f, sigma_i %.2f A rms' % (motor.saliency, k['sigma_i']))"""),
    md("At this AFE's noise floor the injection clears the 10 dB threshold "
       "all the way down to 1.02, so the answer is not that a low-saliency "
       "machine cannot be held at rest. What falls is the exchange rate: the "
       "same angle information costs 0.39 A of HF current at 1.5 saliency "
       "and 2.26 A at 1.05, and that current is loss and acoustic noise in "
       "the machine for no torque at all. Cap the HF current and the method "
       "changes on its own - the right-hand columns are the same machines "
       "with 1 A of headroom instead of 5.\n\n"
       "So `i_h_max` is the knob a low-saliency machine is set up around, "
       "and where it forces `if_start` the drive ramps open-loop on current "
       "until the back-EMF is readable, with a saturation pulse to settle "
       "which end of the axis the magnet is. That makes the back-EMF "
       "observers the whole strategy above the ramp, and their low-speed "
       "limit the binding one:"),
    code("""hand_over = band_of('smo')[0]
second = band_of('flux')[0]
print('CONTROL STRATEGY, low saliency')
print('   below the hand-over      I/f ramp on current, saturation pulse for polarity')
print('   hand-over at             %.0f rad/s = %.0f rpm, the sliding-mode floor'
      % (hand_over, rpm(hand_over)))
print('   w_lo .. w_hi             %.0f .. %.0f rad/s, blended not switched'
      % (hand_over, 2.0 * hand_over))
print('   second hand-over at      %.0f rad/s = %.0f rpm to flux linkage,'
      % (second, rpm(second)))
print('                            weighted on the two residuals, not on speed')
print('   current loop             %.0f Hz, a twentieth of %.0f kHz sampling'
      % (loop_hz, 1e-3 / inverter.TS))
print('   injection                clears 10 dB to saliency 1.02 here; the')
print('                            cost is HF current, so i_h_max is the knob')
print()
print('THERMAL OBSERVER')
print('   node ceilings            125 C the FETs and the MCU, 105 C the board')
print('   throttle at              85 %% = %.1f C, and it acts by dropping MOE' % CEILING_C)
print('   continuous               %.1f A rms a phase = %.2f N.m'
      % (i_rms, kt * iq_cont))
print('   burst                    timed, not held: %.1f s at 100 A from ambient'
      % ((CEILING_C - AMBIENT_C) / ((100.0 / math.sqrt(2.0)) ** 2 * R_PHASE / capacity)))
print('   sample the NTC every     30 s, against a board constant of %.1f min'
      % thermal.tau_minutes())
print('   re-fit first             the phase node to_board, with current flowing')"""),
    md("The **hand-over speed** is where the sliding-mode observer's error "
       "crosses what the torque can carry. It moves with lambda and with the "
       "AFE's floor, not with saliency, so it is the same number whether the "
       "injection is used or not - and on a machine run without it, that "
       "speed is the lowest the drive can hold at all.\n\n"
       "The thermal numbers are the board's and travel with any motor bolted "
       "to it. The exception is the continuous rating: it is conduction, so "
       "it moves with `r_phase` and with whatever the phase node's spreading "
       "resistance turns out to be once it has been re-fit with current "
       "flowing - which is the one measurement that would move every number "
       "in this section."),
    md("## Conclusions\n\n"
       "The angle is covered end to end, and by three different mechanisms "
       "rather than one: saliency at rest, a switching term through the "
       "middle, an integrator at the top. None of the three covers the range "
       "alone, and the hand-overs between them are where the design work "
       "is - which is why the weight should come from what each estimator "
       "says about itself rather than from a speed threshold someone "
       "picked.\n\n"
       "The envelope is set by the board, not the machine: 100 A "
       "instantaneous against the motor's 112.5, and a continuous rating a "
       "fifth of that which is thermal and nothing else. Peak torque is "
       "3.92 to 4.79 N.m over the lambda tolerance and continuous is 0.74 to "
       "0.90, so the useful question for a mission is never the peak but how "
       "long it may be held - 1.3 s at 100 A from ambient, and less from a "
       "warm board.\n\n"
       "The one measurement that would move most of this is the phase node's "
       "spreading resistance with current flowing. It was fitted dry, it "
       "sets the continuous rating and every burst time, and the flux "
       "observer's lambda is the other anchor a thermal model of the whole "
       "machine would want."),
]

#: Observers first, with their simulations and what they mean, then the
#: search that tunes the firmware's own law, then the envelope both land
#: in. The observer sweep defines `rpm` and `band_of`, which the
#: reference values at the end read.
NOTEBOOKS['foc_montecarlo'] = (_FOC_INTRO + _FOC_OBSERVERS + _FOC_SEARCH
                               + _FOC_ENVELOPE)

# ------------------------------------------------------------------ auto_tune
NOTEBOOKS['auto_tune'] = [
    md("# Commissioning and tuning\n\n"
       "The drive runs on constants held in the calibration record - motor, "
       "loop gains, observer gains, injection, dead-time table - and an "
       "uncommissioned board holds placeholders there. This measures them "
       "for one machine, searches a tune against the firmware's own law, "
       "writes both back and verifies."),
    code(KNOB),
    code(OPEN),
    md("`Commissioning` is twelve steps, each needing what the last "
       "measured:\n\n"
       "1. **afe_noise** - `sigma_i` per channel, gates off then on the zero "
       "vector; the difference is switch pickup.\n"
       "2. **sample_point_scan** - CCR5 across the period, keeping the least "
       "phase variance.\n"
       "3. **offsets** - each phase's code at zero current.\n"
       "4. **gain_mismatch** - the three shunt chains against each other.\n"
       "5. **sign_check** - which way a positive d current reads.\n"
       "6. **deadtime** - R, the dead-time volts and the knee, off a current "
       "sweep.\n"
       "7. **l_map** - Ld and Lq against d bias, and the saliency.\n"
       "8. **flux** - lambda, from a spin under I/f.\n"
       "9. **budget** - injection frequency and amplitude for the best SNR.\n"
       "10. **gains** - the loop PI, Kalman gains from the measured noise, "
       "the back-EMF crossover.\n"
       "11. **decide** - injection above the threshold, else I/f.\n"
       "12. **verify** - run it and judge the innovation.\n\n"
       "`arm` is what `gates.arm()` is called with where a step needs the "
       "stage; without it the switching steps refuse. `run()` puts the stage "
       "down whatever happens."),
    code("""from coaxial.commission import Commissioning

c = Commissioning(device, arm=dict(bypass_sto=True, ignore_interlock=True),
                  log=print, rated_rpm=3000.0)
report = c.run()
print(report['line'])"""),
    code("""r = report['results']
print('steps that ran:', ', '.join(sorted(r)))
print()
print('sigma_i      %.4f A, ENOB %.1f' % (r['afe']['sigma_i'], r['afe']['enob']))
print('sample point CCR5 %d of %d' % (r['sample_point']['best'], r['sample_point']['period']))
print('decision     %s' % r['decision']['method'])
print('verify       %s' % {k: r['verify'][k] for k in
                           ('method', 'sigma_theta_deg', 'omega_hat', 'fault')})"""),
    md("The four constants into a `Parameters`: R from the dead-time sweep, "
       "Ld and Lq from the map, lambda from the flux step, pole pairs from "
       "the record - no torque measurement recovers those. A step that saw "
       "no current reports `measured: False` and the record's value stands "
       "in. `measured` and `source` travel with the set."),
    code("""from coaxial.motor import Parameters

p = device.drive.params()

def got(step, key, fallback):
    block = r.get(step) or {}
    return block[key] if block.get('measured') is not False and key in block else fallback

identified = Parameters(
    name='commissioned',
    r=got('deadtime', 'r', p['motor_r_uohm']),
    ld=got('l_map', 'ld', p['motor_ld_nh']),
    lq=got('l_map', 'lq', p['motor_lq_nh']),
    lam=got('flux', 'lambda', p['motor_lambda_uvs']),
    poles=int(p['motor_pole_pairs']),
    sat=0.3, i_sat=4.0, measured=True, source='commissioning on this rig')
print(identified)
vdc = device.drive.state()['vdc']
print('link %.2f V' % vdc)"""),
    md("`gains` already wrote a tune in closed form. This searches for one "
       "instead: `run_job` takes the motor and the board's limits, so every "
       "candidate is scored against the firmware's C driving plants drawn "
       "around **this** machine - R up to a hot winding, L a quarter either "
       "way, the dead time either side of the commissioned one, the rotor "
       "anywhere.\n\n"
       "Cost is `sigma_theta + speed_err + 10 x trip`; `robust` is its mean "
       "plus its 90th percentile, so a tune that occasionally loses the "
       "rotor scores worse than one that is merely average. Six candidates "
       "and three draws here, against the tool's 48 and 16."),
    code("""import os
import sys

sys.path.insert(0, os.path.join('..', 'host', 'tools'))
import montecarlo as mc

fields = {k: getattr(identified, k) for k in ('name', 'r', 'ld', 'lq', 'lam', 'poles',
                                             'j', 'b', 'sat', 'i_sat', 'measured', 'source')}
i_max, i_trip = p['drv_i_max_ma'], p['drv_i_trip_ma']
knobs = mc.candidates(6, seed=3)
jobs = [{'vdc': vdc, 'knobs': k, 'seed': 1000 * i + s, 'motor': fields,
         'i_max': i_max, 'i_trip': i_trip, 'i_h_max': 1.0, 'k_prop': 0.0}
        for i, k in enumerate(knobs) for s in range(3)]
with mc.pool() as pool:
    runs = mc.sweep(pool, jobs)
score = mc.score(runs)
best = score.loc[score.robust.idxmin()]
print(score[['robust', 'mean', 'p90'] + list(mc.KNOBS)].round(4).sort_values('robust').head())"""),
    md("`design` turns the winning knobs into the firmware's parameters - kp "
       "and ki from the loop bandwidth, l1 and l2 from the PLL's, the "
       "injection volts and demodulator gain, the blend band. `set_params` "
       "writes them into the record in SI and reloads; `verify` runs the "
       "drive under them and judges the innovation."),
    code("""tune = mc.design({k: best[k] for k in mc.KNOBS}, vdc, identified, i_max, i_trip, 1.0)
written = device.drive.set_params(
    motor_r_uohm=identified.r, motor_ld_nh=identified.ld, motor_lq_nh=identified.lq,
    motor_lambda_uvs=identified.lam,
    drv_kp_mv_per_a=tune['kp'], drv_ki_v_per_as=tune['ki'],
    drv_l1_milli=tune['l1'], drv_l2_milli=tune['l2'],
    drv_inj_mv=tune['inj_volts'], drv_inj_periods=tune['inj_periods'],
    drv_eps_gain_ua_per_rad=tune['eps_gain'],
    drv_w_lo_mrad_s=tune['w_lo'], drv_w_hi_mrad_s=tune['w_hi'])
for name, value in written.items():
    print('%-24s %s' % (name, value))
check = c.verify(iq=0.5, seconds=1.0)
print({k: check[k] for k in ('method', 'sigma_theta_deg', 'ljung_box', 'omega_hat', 'fault')})"""),
    md("`device.calibration.save()` is what keeps the record across a reset - "
       "the drive reloads its parameters from it at boot, so a board runs the "
       "same tune after a power cycle that it ran before."),
    code("""print('saved:', device.calibration.save())
print('the drive reads them back through the record:')
for name, value in sorted(device.drive.params().items()):
    print('   %-26s %s' % (name, value))
device.close()"""),
    md("## Conclusions"),
    code("""def step(number, name, got, line):
    if isinstance(got, dict) and got.get('measured') is False:
        print('%-2s %-13s not measured - %s' % (number, name, got.get('why', 'no current')))
    else:
        print('%-2s %-13s %s' % (number, name, line()))

step(1, 'AFE', r['afe'], lambda: 'sigma_i %.4f A, ENOB %.1f, ISR %.1f us'
     % (r['afe']['sigma_i'], r['afe']['enob'], r['afe']['latency']['isr_cost_us']))
step(2, 'sample point', r['sample_point'], lambda: 'CCR5 %d of %d (was %d)'
     % (r['sample_point']['best'], r['sample_point']['period'], r['sample_point']['was']))
step(3, 'offsets', r['offsets'], lambda: ' '.join(
    '%s %+d%s' % (n[-1], v['offset_raw'], ' SUSPECT' if v['suspect'] else '')
    for n, v in r['offsets'].items()))
step(4, 'sign', r['sign'], lambda: 'sign %+d, id %.3f A'
     % (r['sign']['sign'], r['sign']['id']))
step(5, 'dead time', r['deadtime'], lambda: 'R %.4f ohm, V_dt %.3f V, knee %.2f A'
     % (r['deadtime']['r'], r['deadtime']['v_dt'], r['deadtime']['i_knee']))
step(6, 'L map', r['l_map'], lambda: 'Ld %.1f uH, Lq %.1f uH, dL/L %.3f'
     % (r['l_map']['ld'] * 1e6, r['l_map']['lq'] * 1e6, r['l_map']['dl_over_l']))
step(7, 'flux', r['flux'], lambda: 'lambda %.5f V.s, load angle %.2f rad'
     % (r['flux']['lambda'], r['flux']['load_angle']))
step(8, 'budget', r['budget'], lambda: 'f_inj %.0f Hz, V %.2f, SNR %.1f dB, limited by %s'
     % (r['budget']['choice']['f_inj_hz'], r['budget']['choice']['v_inj'],
        r['budget']['choice']['snr_db'], r['budget']['choice']['limited_by']))
step(9, 'gains', r['gains'], lambda: 'iloop %.0f Hz, PLL %.0f Hz, crossover %.0f rpm'
     % (r['gains']['loop']['bw_hz'], (r['gains']['kalman'] or {}).get('wn_hz', 0.0),
        r['gains']['crossover']['rpm']))
step(10, 'decision', r['decision'], lambda: '%s (SNR %.1f dB against %.0f)'
     % (r['decision']['method'], r['decision']['snr_db'], r['decision']['threshold_db']))
step(11, 'verify', check, lambda: 'sigma_theta %.2f deg, innovation %s, fault %s'
     % (check['sigma_theta_deg'],
        'white' if check['ljung_box']['white'] else 'NOT white', check['fault']))
print()
print('tune written for %.1f V:' % vdc)
for name, value in sorted(written.items()):
    print('   %-26s %s' % (name, value))"""),
    md("Three steps decide rather than measure.\n\n"
       "**The sample point** needs the stage switching: with nothing on the "
       "gates the scan is a walk through noise, which picked 990 of 2376 - "
       "mid-period, where the pickup is worst.\n\n"
       "**An offset past `limit_codes`** is reported, not applied. A phase "
       "reading -52 A with nothing connected is a fault in that chain; "
       "storing it as the zero hides it.\n\n"
       "**The method** needs saliency - the demodulator's gain goes as "
       "`Lq - Ld` - so a machine with little of it lands on I/f. The "
       "polarity comes from a saturation pulse either way: injection locks "
       "the d **axis**, and only saturation says which end is the magnet.\n\n"
       "The record now holds what the drive runs on after the next reset, "
       "which is the one place any of it lives (invariant 7)."),
]

# ------------------------------------------------------------- position_servo
NOTEBOOKS['position_servo'] = [
    md("# Position servo\n\nThe PMSM as stepper and servo, ring and sag measured."),
    code(KNOB),
    code(OPEN),
    md("HOLD holds a current vector at a commanded angle and the rotor is "
       "dragged along by the load-angle spring, `amps kt sin(delta)`: a "
       "microstepper. Open loop, so an overload slips poles silently. The "
       "servo is the same move with the slip measured out over the A1335.\n\n"
       "On the `model` source the shaft sensor follows the rotor the drive "
       "torques, so the whole loop closes."),
    code("""device.drive.source('model')
device.drive.model_param(j=2e-5, b=1e-5, load=0.0)
stage = device.gates.arm(bypass_sto=True, ignore_interlock=True)
print('armed', stage['pwm_enabled'])
print('pole pairs', device.drive.params()['motor_pole_pairs'])"""),
    code("""import time

def trace(seconds, every=0.02):
    rows = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        rows.append((time.monotonic() - t0, device.angle.state()['degrees']))
        time.sleep(every)
    return rows

with device.motion.stepper(amps=2.0, deg_s=90.0) as m:
    print('detented; command at %.2f deg' % m.position)
    m.to(90.0)
    ring = trace(0.6)
    print('command %.1f deg, shaft %.1f deg' % (m.position, ring[-1][1] - ring[0][1] + 0.0))
    m.step(10)
    print('ten full steps: command %.1f deg' % m.position)"""),
    md("The ring after a move: the shaft sampled at 50 Hz while the spring "
       "settles. Mechanical degrees, from where the block began."),
    code("""import matplotlib.pyplot as plt

fig, ax = plt.subplots(figsize=(8, 3.5))
ax.plot([r[0] for r in ring], [r[1] for r in ring], '.-')
ax.set_xlabel('s after the move'); ax.set_ylabel('shaft deg'); ax.grid(True)
plt.show()"""),
    md("The servo corrects what the load stole: move, settle, measure, "
       "correct, until inside `tol`. A load on the model makes the spring "
       "sag; `error` is target minus shaft from the last correction."),
    code("""with device.motion.servo(amps=2.0, settle=0.3) as s:
    got = s.to(45.0, tol=0.5)
    print('no load:   shaft %.2f deg, error %.2f' % (got, s.error))
    device.drive.model_param(load=0.02)
    time.sleep(0.4)
    sag = s.to(45.0, tol=0.5)
    print('0.02 N.m:  shaft %.2f deg, error %.2f' % (sag, s.error))
    device.drive.model_param(load=0.0)
print(device.gates.disarm()['pwm_enabled'])
device.drive.source('adc')
device.close()"""),
    md("## Conclusions"),
    code("""import math

peak = max(r[1] for r in ring)
floor = min(r[1] for r in ring)
print('ring after a 90 deg move   %.2f deg peak to peak over %.2f s'
      % (peak - floor, ring[-1][0]))
print('sampled at                 %.0f Hz' % (len(ring) / ring[-1][0]))
print('servo, no load             error %.2f deg' % (45.0 - got))
print('servo, 0.02 N.m            error %.2f deg' % (45.0 - sag))
print('holding torque             %.1f A x Kt' % 2.0)"""),
    md("Two verbs, one loop: `_slew_to` walks the command at `pitch` degrees "
       "a write, so the spring never spans more than a few degrees at once, "
       "and the stepper's move and the servo's correction are that same "
       "code.\n\n"
       "`_energize` ramps the current instead of snapping it on: full current "
       "onto an unknown rotor is a yank of up to half a pole that an "
       "underdamped rotor rides through, pole after pole. Grown slowly it "
       "detents into the nearest pole, and that is where angles count from.\n\n"
       "The servo closes **once per move**. The link corrects at tens of "
       "hertz and the load-angle spring rings at tens of hertz, so a per-pass "
       "loop samples its own resonance aliased and pumps it - six clean "
       "passes wound the rotor through a pole slip into a freewheel. Slew, "
       "let the ring die, read, correct.\n\n"
       "Every reading is a **mean** of nine: the detent rings, and a single "
       "read froze up to the ring's amplitude into the frame for the life of "
       "the block. `to()` raises after `tries` corrections still outside "
       "`tol` - a stalled arm is a fact, not a return code (invariant 8)."),
]

# ---------------------------------------------------- position_and_sensorless
NOTEBOOKS['position_and_sensorless'] = [
    md("# Position and sensorless\n\nObserver vs shaft sensor, one rotor two answers."),
    code(KNOB),
    code(OPEN),
    md("The rotor observer estimates the electrical angle from the currents; the "
       "A1335 reads the shaft. Under the `velocity` verb - sensorless, "
       "`coaxial.loop`'s speed loop over `omega_hat` at link rate - both are "
       "sampled once a pass."),
    code("""import math

drive = device.drive
drive.source('model')
drive.model_param(j=2e-5, b=1e-5, load=0.0)
device.gates.arm(bypass_sto=True, ignore_interlock=True)
poles = int(drive.params()['motor_pole_pairs'])
rows = []

def watch(v):
    m = drive.model()
    shaft = device.angle.state()['degrees']
    rows.append((len(rows) * v.pause, math.degrees(m['theta']) / poles,
                 math.degrees(m['theta_hat']) / poles, shaft, m['error'],
                 m['omega'] / poles * 60.0 / (2.0 * math.pi), v.rpm_now))

with device.motion.velocity(amps=1.0, hz=3.0) as v:
    v.rpm(600.0, seconds=3.0, watch=watch)
    print('settled at %.0f rpm' % v.rpm_now)
    v.stop(seconds=1.5)
device.gates.disarm()
drive.source('adc')
print(len(rows), 'passes')"""),
    code("""import matplotlib.pyplot as plt

t = [r[0] for r in rows]
fig, axes = plt.subplots(3, 1, sharex=True, figsize=(9, 8))
axes[0].plot(t, [r[1] % 360 for r in rows], label='rotor, mech deg (model)')
axes[0].plot(t, [r[2] % 360 for r in rows], '.', label='observer / poles')
axes[0].plot(t, [r[3] for r in rows], label='shaft sensor')
axes[0].set_ylabel('deg'); axes[0].legend()
axes[1].plot(t, [math.degrees(r[4]) for r in rows]); axes[1].set_ylabel('observer error, el deg')
axes[2].plot(t, [r[5] for r in rows], label='rotor rpm')
axes[2].plot(t, [r[6] for r in rows], label='observer rpm')
axes[2].set_ylabel('rpm'); axes[2].set_xlabel('s'); axes[2].legend()
plt.show()
device.close()"""),
    md("## Conclusions"),
    code("""import math

errors = [abs(r[4]) for r in rows]
rms = math.sqrt(sum(e * e for e in errors) / len(errors))
top = max(r[5] for r in rows)
print('passes           %d at %.0f Hz' % (len(rows), len(rows) / rows[-1][0]))
print('top speed        %.0f rpm mechanical, %.0f rad/s electrical'
      % (top, top * 2.0 * math.pi / 60.0 * poles))
print('observer error   %.4f rad rms, worst %.4f rad (%.2f deg electrical)'
      % (rms, max(errors), math.degrees(max(errors))))
print('                 %.3f deg mechanical at %d pole pairs'
      % (math.degrees(max(errors)) / poles, poles))
print('shaft vs rotor   %.2f deg apart at the end'
      % abs((rows[-1][3] % 360.0) - (rows[-1][1] % 360.0)))"""),
    md("The observer estimates the **electrical** angle from the currents; "
       "the A1335 reads the **mechanical** shaft. They differ by the pole "
       "pairs, and an electrical degree of observer error is a fourteenth of "
       "a mechanical degree on the 5230SL - which is why an error that looks "
       "large in the estimate can be small at the shaft, and why the pole "
       "count has to come from the winding rather than from any torque "
       "measurement.\n\n"
       "The shaft sensor is absolute over one turn and needs a magnet; the "
       "observer needs neither, and holds down to the speed the back-EMF "
       "stops being readable against the dead-time residual - which is what "
       "`sensorless.crossover` computes and `foc_montecarlo.ipynb` measures. "
       "Below it the injection carries the estimate, and only saturation says "
       "which end of the d axis is the magnet.\n\n"
       "One state read a pass carries both `omega_hat` and the fault: a trip "
       "here is a runaway or an overcurrent, the one place stopping the loop "
       "matters most, and taking it off the same reply costs no extra round "
       "trip."),
]

# ---------------------------------------------------------------- app_quad_esc
WATCH_LOAD = """import math

TWO_PI = 2.0 * math.pi

def prop_load(k):
    def watch(v):
        wm = drive.model()['omega'] / poles
        drive.model_param(load=k * wm * abs(wm))
        log.append((len(log) * v.pause, v.bus.w_ref * 60.0 / TWO_PI, v.rpm_now,
                    v.bus.iq_ref))
    return watch"""

NOTEBOOKS['app_quad_esc'] = [
    md("# Quad ESC lane\n\nOne motor of a quadrotor: a throttle lane under "
       "`coaxial.motion.velocity`, the propeller law fed to the model's load."),
    code(KNOB),
    code(OPEN),
    md("The drive commutates itself at 50 kHz from the record's tune; the "
       "lane reads `omega_hat` and writes `iq_ref` at link rate. A write "
       "lands in about 7 ms, so nothing here runs faster than a few tens of "
       "hertz - a quad's rate loop belongs where 50 kHz lives, in `drive/`, "
       "and what runs here is what a flight controller would do with this "
       "board on the other end of a wire.\n\n"
       "`load_k` is the loop's knowledge of the propeller, the law its "
       "feedforward leans on; the drag itself is a torque on the machine, so "
       "the watch feeds `k w^2` to the model each pass."),
    code("""drive = device.drive
drive.source('model')
drive.model_param(j=2e-5, b=1e-5, load=0.0)
device.gates.arm(bypass_sto=True, ignore_interlock=True)
poles = int(drive.params()['motor_pole_pairs'])
log = []"""),
    code(WATCH_LOAD),
    code("""K_PROP = 2e-8
with device.motion.velocity(amps=2.0, hz=3.0, load_k=K_PROP) as lane:
    for rpm, hold in ((1500.0, 2.0), (3000.0, 2.0), (2000.0, 1.5), (3500.0, 2.0)):
        got = lane.rpm(rpm, seconds=hold, watch=prop_load(K_PROP))
        print('asked %5.0f  settled %5.0f rpm' % (rpm, got))
    lane.stop(seconds=1.0)
device.gates.disarm()
drive.model_param(load=0.0)
drive.source('adc')"""),
    code("""import matplotlib.pyplot as plt

t = [r[0] for r in log]
fig, (a, b) = plt.subplots(2, 1, sharex=True, figsize=(9, 6))
a.plot(t, [r[1] for r in log], label='w_ref'); a.plot(t, [r[2] for r in log], label='rpm')
a.set_ylabel('rpm'); a.legend()
b.plot(t, [r[3] for r in log]); b.set_ylabel('iq_ref A'); b.set_xlabel('s')
plt.show()
device.close()"""),
    md("## Conclusions"),
    code("""rate = len(log) / log[-1][0]
print('passes           %d at %.0f Hz, %.0f ms a pass' % (len(log), rate, 1000.0 / rate))
print('iq_ref           %.2f A peak, %.2f A mean'
      % (max(abs(r[3]) for r in log), sum(r[3] for r in log) / len(log)))
for name, target in (('1500', 1500.0), ('3000', 3000.0), ('2000', 2000.0), ('3500', 3500.0)):
    settled = [r[2] for r in log if abs(r[1] - target) < 1.0]
    if settled:
        held = settled[len(settled) // 2:]
        print('at %-5s rpm      held %.0f, spread %.0f rpm over %d passes'
              % (name, sum(held) / len(held), max(held) - min(held), len(held)))"""),
    md("The reference slews rather than steps - by default it reaches the "
       "target in a third of the block - so the current stays a control "
       "action instead of a step into the clamp. The loop's PI cancels the "
       "mechanical pole: kp is `w0 J / kt`, and the plant about the reference "
       "is `J s + b + 2 k |w_ref|`, because a propeller linearises to twice "
       "its slope. The feedforward carries the acceleration and the standing "
       "drag, and the integrator holds on the current clamp and on the inner "
       "loop's `v_sat` - past either, error is not information.\n\n"
       "`j` and `b` default to the smallest plausible machine on purpose: an "
       "overstated `j` scales kp by the same factor, and the discrete loop "
       "flips sign and doubles - measured, `j` five times the plant took "
       "+900 rpm asked to -1552 delivered. Understating only makes a big "
       "machine sluggish. `speed_loop.ipynb` identifies the real pair."),
]

# ------------------------------------------------------------- app_fixed_wing
NOTEBOOKS['app_fixed_wing'] = [
    md("# Fixed wing cruise\n\nA cruise held under `coaxial.motion.velocity`, "
       "a gust on the load, and the loop's answer."),
    code(KNOB),
    code(OPEN),
    code("""drive = device.drive
drive.source('model')
drive.model_param(j=2e-5, b=1e-5, load=0.0)
device.gates.arm(bypass_sto=True, ignore_interlock=True)
poles = int(drive.params()['motor_pole_pairs'])
log = []"""),
    code(WATCH_LOAD),
    md("Cruise at one speed; halfway through the load steps up for a second "
       "and back - the gust - while the reference stands still."),
    code("""import time

K_CRUISE = 2e-8
gust_at = None

def gusty(k):
    inner = prop_load(k)
    def watch(v):
        global gust_at
        now = time.monotonic()
        if gust_at is None:
            gust_at = now
        factor = 2.5 if 2.0 < now - gust_at < 3.0 else 1.0
        inner(v)
        wm = drive.model()['omega'] / poles
        drive.model_param(load=factor * k * wm * abs(wm))
    return watch

with device.motion.velocity(amps=2.0, hz=2.0, load_k=K_CRUISE) as cruise:
    cruise.rpm(2500.0, seconds=1.5, watch=prop_load(K_CRUISE))
    cruise.rpm(2500.0, seconds=5.0, watch=gusty(K_CRUISE))
    print('after the gust: %.0f rpm' % cruise.rpm_now)
    cruise.stop(seconds=1.0)
device.gates.disarm()
drive.model_param(load=0.0)
drive.source('adc')"""),
    code("""import matplotlib.pyplot as plt

t = [r[0] for r in log]
fig, (a, b) = plt.subplots(2, 1, sharex=True, figsize=(9, 6))
a.plot(t, [r[1] for r in log], label='w_ref'); a.plot(t, [r[2] for r in log], label='rpm')
a.set_ylabel('rpm'); a.legend()
b.plot(t, [r[3] for r in log]); b.set_ylabel('iq_ref A'); b.set_xlabel('s')
plt.show()
device.close()"""),
    md("## Conclusions"),
    code("""settled = [r for r in log if r[0] > 1.0]
cruise = [r for r in settled if r[0] < 3.0]
gust = [r for r in settled if 3.5 < r[0] < 4.5]
after = [r for r in settled if r[0] > 5.0]
for name, rows_ in (('cruise', cruise), ('gust', gust), ('recovered', after)):
    if rows_:
        speed = [r[2] for r in rows_]
        current = [r[3] for r in rows_]
        print('%-10s rpm %6.0f +/- %4.0f    iq_ref %5.2f A'
              % (name, sum(speed) / len(speed), max(speed) - min(speed),
                 sum(current) / len(current)))
print('reference stood still at %.0f rpm throughout' % log[-1][1])"""),
    md("The reference never moved: the speed error is what the loop sees, and "
       "the current is what it answers with. A load that grows with the "
       "square of speed cannot be carried by `b`, a linear drag - that "
       "reaches the same speed at a torque which is wrong everywhere except "
       "the one point it was fitted at, which is why `Motor` carries `k_load` "
       "separately.\n\n"
       "The feedforward already knows the standing drag `k w |w|`, so the "
       "integrator only has to make up the difference the gust adds, and it "
       "gives it back when the gust passes. The clamp is `limit`, the amps "
       "the verb was opened with."),
]

# -------------------------------------------------------------- app_robot_arm
NOTEBOOKS['app_robot_arm'] = [
    md("# Two-joint arm\n\nTwo boards, two joints, `coaxial.motion.servo` on each."),
    code(KNOB),
    md("One board per joint: unit 1 the shoulder, unit 2 the elbow. On a bus "
       "they share the segment; on the stand-in each unit is its own board."),
    code("""from coaxial import Coaxial63100

shoulder = Coaxial63100(port=PORT, unit=1, simulated_device=SIMULATED).open()
elbow = Coaxial63100(port=PORT, unit=2, simulated_device=SIMULATED).open()
for joint in (shoulder, elbow):
    joint.drive.source('model')
    joint.drive.model_param(j=2e-5, b=1e-5, load=0.0)
    joint.gates.arm(bypass_sto=True, ignore_interlock=True)
print(shoulder, elbow)"""),
    md("A move is a pair of targets; each servo slews, settles, reads its "
       "shaft and corrects what the load stole. The elbow carries a standing "
       "load, which is what a link hanging off it is."),
    code("""elbow.drive.model_param(load=0.01)
POSES = ((30.0, 60.0), (60.0, 20.0), (0.0, 0.0))
reached = []
with shoulder.motion.servo(amps=2.0) as s, elbow.motion.servo(amps=2.0) as e:
    for a, b in POSES:
        got_a = s.to(a, tol=0.5)
        got_b = e.to(b, tol=0.5)
        reached.append((a, b, got_a, s.error, got_b, e.error))
        print('pose (%5.1f, %5.1f)  shoulder %6.2f err %5.2f  elbow %6.2f err %5.2f'
              % (a, b, got_a, s.error, got_b, e.error))
for joint in (shoulder, elbow):
    joint.gates.disarm()
    joint.drive.model_param(load=0.0)
    joint.drive.source('adc')
    joint.close()"""),
    md("## Conclusions"),
    code("""print('%-16s %-22s %-22s' % ('pose', 'shoulder', 'elbow (0.01 N.m)'))
for a, b, got_a, err_a, got_b, err_b in reached:
    print('(%5.1f, %5.1f)   %7.2f deg err %5.2f   %7.2f deg err %5.2f'
          % (a, b, got_a, err_a, got_b, err_b))
print()
print('worst shoulder error %.2f deg, worst elbow error %.2f deg'
      % (max(abs(r[3]) for r in reached), max(abs(r[5]) for r in reached)))
print('tolerance asked for  0.50 deg, up to 4 corrections a move')"""),
    md("Two boards, two unit ids, one segment. A Modbus RTU frame carries the "
       "unit id first and every node on the wire sees every frame; "
       "`bus_message` counts what passed and `server_message` only what was "
       "addressed here, so the difference is the traffic meant for the other "
       "joint. Unit 0 is broadcast - every node acts, none answers, and reads "
       "are refused.\n\n"
       "Each joint holds its own current vector, so nothing about the pair is "
       "coupled through the drive: what couples them is the arm, and the "
       "elbow's standing load is what its own servo corrects out. The "
       "correction is what the load stole - a spring wound by holding torque, "
       "or poles slipped outright."),
]

# --------------------------------------------------------- app_precision_servo
NOTEBOOKS['app_precision_servo'] = [
    md("# Precision hold\n\nA servo holding one angle against a load step, the "
       "shaft watched through it."),
    code(KNOB),
    code(OPEN),
    code("""import time

drive = device.drive
drive.source('model')
drive.model_param(j=2e-5, b=1e-5, load=0.0)
device.gates.arm(bypass_sto=True, ignore_interlock=True)

def watch_shaft(seconds, every=0.02):
    rows = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        rows.append((time.monotonic() - t0, device.angle.state()['degrees']))
        time.sleep(every)
    return rows"""),
    md("Hold at 30 degrees. A load step on the model pulls the shaft off the "
       "command by the spring's sag; the next correction takes it back."),
    code("""with device.motion.servo(amps=3.0, settle=0.3) as hold:
    zero = hold.to(30.0, tol=0.25)
    print('held at %.2f deg, error %.2f' % (zero, hold.error))
    before = watch_shaft(0.5)
    drive.model_param(load=0.03)
    during = watch_shaft(0.8)
    corrected = hold.to(30.0, tol=0.25)
    print('after the load step: shaft %.2f deg, error %.2f' % (corrected, hold.error))
    after = watch_shaft(0.5)
    drive.model_param(load=0.0)
device.gates.disarm()
drive.source('adc')"""),
    code("""import matplotlib.pyplot as plt

fig, ax = plt.subplots(figsize=(9, 3.5))
t = 0.0
for rows, label in ((before, 'held'), (during, 'load step'), (after, 'corrected')):
    ax.plot([t + r[0] for r in rows], [r[1] for r in rows], '.-', label=label)
    t += rows[-1][0]
ax.set_xlabel('s'); ax.set_ylabel('shaft deg'); ax.legend(); ax.grid(True)
plt.show()
device.close()"""),
    md("## Conclusions"),
    code("""held = sum(r[1] for r in before) / len(before)
pulled = sum(r[1] for r in during[len(during) // 2:]) / (len(during) - len(during) // 2)
back = sum(r[1] for r in after) / len(after)
print('held at          %.2f deg (asked 30.00)' % held)
print('under 0.03 N.m   %.2f deg, sag %.2f deg' % (pulled, pulled - held))
print('after correcting %.2f deg, residual %.2f deg' % (back, back - 30.0))
print('sampled at       %.0f Hz through each phase' % (len(before) / before[-1][0]))
print('holding current  3.0 A, tolerance 0.25 deg')"""),
    md("The sag is the load-angle spring at work: HOLD commutates on the "
       "**commanded** angle, and the rotor sits wherever `amps kt sin(delta)` "
       "balances the load. That angle is the error, and it is a fact about "
       "the torque - a stiffer hold is more current, not a tighter loop.\n\n"
       "The correction moves the command by the measured error, so the spring "
       "is re-centred rather than fought. Nothing here runs a position loop "
       "per pass: the ring is at the same frequency the link corrects at, and "
       "closing per pass pumps it.\n\n"
       "A shaft that stays outside `tol` after `tries` corrections raises "
       "with what it saw - the load is past the holding torque, or there is "
       "no magnet in front of the sensor."),
]


def cell(kind, text, ident):
    """One notebook cell, in nbformat 4's shape.

    `ident` is the cell id nbformat wants; it is derived from the
    notebook's name and the cell's position so a rewrite of one cell
    does not renumber the rest of the file.
    """
    lines = text.split('\n')
    source = [line + '\n' for line in lines[:-1]] + [lines[-1]]
    if kind == 'markdown':
        return {'cell_type': 'markdown', 'id': ident, 'metadata': {},
                'source': source}
    return {'cell_type': 'code', 'id': ident, 'metadata': {}, 'source': source,
            'outputs': [], 'execution_count': None}


def notebook(name, cells):
    """One notebook as the dict nbformat writes."""
    return {
        'cells': [cell(kind, text, '%s-%02d' % (name.replace('_', '-'), i))
                  for i, (kind, text) in enumerate(cells)],
        'metadata': {
            'kernelspec': {'display_name': 'Python 3', 'language': 'python',
                           'name': 'python3'},
            'language_info': {'name': 'python'},
        },
        'nbformat': 4, 'nbformat_minor': 5,
    }


def write(name, out_dir):
    """Write one notebook, outputs empty. Returns the path."""
    path = os.path.join(out_dir, name + '.ipynb')
    with io.open(path, 'w', encoding='utf-8', newline='\n') as handle:
        json.dump(notebook(name, NOTEBOOKS[name]), handle, indent=1,
                  ensure_ascii=False)
        handle.write('\n')
    return path


def execute(path, out_dir, timeout=1800):
    """Run a notebook in place. Returns the first error, or None.

    `allow_errors` so a failing cell does not stop the rest: the whole
    run is more use than the first traceback, and the outputs of the
    cells that did work are what say where it went wrong.
    """
    import nbformat
    from nbclient import NotebookClient

    book = nbformat.read(path, as_version=4)
    NotebookClient(book, timeout=timeout, kernel_name='python3',
                   resources={'metadata': {'path': out_dir}},
                   allow_errors=True).execute()
    nbformat.write(book, path)
    for number, one in enumerate(book.cells):
        for output in one.get('outputs', []):
            if output.get('output_type') == 'error':
                return 'cell %d: %s: %s' % (number, output.get('ename'),
                                            output.get('evalue'))
    return None


def main(argv=None):
    """Write the notebooks named on the command line, or all of them."""
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('names', nargs='*', help='notebooks; default all')
    parser.add_argument('--execute', action='store_true',
                        help='run each one and keep its outputs')
    parser.add_argument('--out', default=OUT_DIR)
    args = parser.parse_args(argv)

    unknown = [n for n in args.names if n not in NOTEBOOKS]
    if unknown:
        parser.error('no such notebook: %s. There are %d: %s'
                     % (', '.join(unknown), len(NOTEBOOKS),
                        ', '.join(sorted(NOTEBOOKS))))
    wanted = args.names or sorted(NOTEBOOKS)
    if not os.path.isdir(args.out):
        os.makedirs(args.out)

    failed = []
    for name in wanted:
        path = write(name, args.out)
        if not args.execute:
            print('wrote %s' % os.path.basename(path))
            continue
        sys.stdout.write('%-30s ' % name)
        sys.stdout.flush()
        error = execute(path, args.out)
        print('FAILED  %s' % error if error else 'ok')
        if error:
            failed.append(name)

    print('%d notebook%s%s' % (len(wanted), '' if len(wanted) == 1 else 's',
                               ', %d failed' % len(failed) if failed else ''))
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
