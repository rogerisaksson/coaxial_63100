# %% [markdown]
# # coaxial 63100 - a DAQ session
# `# %%` cells: opens as a notebook, runs as a script.
# `SIMULATED = True` needs no cable; every value is then invented.
#
# The lifecycle is one vocabulary and reads in order:
#
#     configure -> start -> acquire ... -> stop
#
# and it lives behind `device.daq`, the acquisition FRONT DOOR: the same
# vocabulary the device owns - it stops before reconfiguring, keeps the
# layout, and puts real times and the sample count on every record - as a
# handle you can bind before open() and open through, so a script reads
# subsystem-first the way the sensor examples do. The raw ops sit at
# `device.board.daq`, the way `stage.control` sits under the arming policy.
#
# `write` is not part of the lifecycle. It sets an output pin and has
# nothing to do with the acquisition, which is why it sits after `start`
# here rather than in the middle of the sequence pretending to belong.

# %%
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'host'))          # the library lives here

from coaxial import Coaxial63100, scaling

SIMULATED = False
BLOCKS = 20

device = Coaxial63100(port='COM4', baud=115200,
                      simulated_device=SIMULATED, power_afe=True)
daq = device.daq
daq.open()
print(device)
print(daq)

# %% [markdown]
# ## Clock
# The board counts CPU cycles and nothing else. This puts a real time on
# every record - UTC, not this PC's idea of it, which was 947 ms out.

# %%
print(device.set_time_from_pc())

# %% [markdown]
# ## Configure, then start
# `sample_rate=100` asks for 100 records a second and lets the BOARD do
# the averaging: the converter runs flat out and each record carries the
# sum of everything the window held, with `samples` as the divisor.
# `accumulate=N` is the other way - N samples close a record instead.

# %% [markdown]
# `catalogue()` is the board's own list of everything it can put in a
# record - the analog channels, the sampled pins, and the sensor fields it
# does not carry yet, each row saying which it is.

# %%
for row in daq.catalogue():
    print('  %-16s %-8s %-10s %s'
          % (row['name'], row['kind'], row['unit'], row['selectable']))

# %% [markdown]
# Names go in as arguments or as a list, in any spelling: `phaseU`,
# `Phase U` and `phase_u` are one channel. Naming a pin turns the whole
# group on, because the board puts every sampled pin in a record or none.

# %%
daq.configure('phaseU', 'NTC', 'AFE_ON', sample_rate=100)
print('recording:', daq.channel_names())
daq.start()

# %% [markdown]
# ## While it runs
# Both of these are answered by the board mid-task. `channels` and `outputs`
# are asked, never hardcoded; `write` drives a pin and leaves the acquisition
# alone. Analog would be a PWM duty - there is no DAC - and it arms the gate
# drivers, so it is not in this example.

# %%
print('in :', daq.channels())
print('out:', daq.outputs())
print(device.write(digital={'UART5_TERM': False}))

# %% [markdown]
# ## Read
# `read(-1)` blocks until there is something and then takes everything the
# board has; `read(n)` waits for n records. A reader thread has been on the
# link since `start()`, so the loop body below costs the link nothing.
#
# A record is an object AND the mapping it came from. `sample.raw` is the
# SUM the board sent, `sample.value` is that sum over `record.count` - the
# mean of the window. `dt` is measured, from the gap to the next record.
#
# The task buffers converter codes and scales nothing, so the code is what
# arrives. `scaling()` is the board's own record - the thermistor it was
# told it has, not one written down here (invariant 7).

# %%
ntc = device.analog.scaling()['ntc']
print('scaling from:', ntc.name)

for n in range(1, BLOCKS + 1):
    values = daq.read(-1)
    r = values[-1]
    code = dict(zip(r.channel_name, (s.value for s in r.samples)))['NTC']
    print('%2d  %s  dt %6.4f  %d records  NTC %7.1f = %5.2f C  AFE %s'
          % (n, time.strftime('%H:%M:%S', time.localtime(r.start_time)),
             r.dt or 0.0, len(values), code, ntc.celsius(code),
             '%3.0f%%' % (100.0 * r.digital['AFE_ON'])))

# %% [markdown]
# ## The same run as columns
# A record is a struct and a run is an array of them, which is the shape
# the link delivers. Anything that plots or fits wants one array per
# channel, and `columns()` is that flip.

# %%
cols = daq.columns(daq.read(-1))
for name in daq.channel_names():
    print('  %-10s %s' % (name, [round(v, 1) for v in cols[name][:6]]))
print('  %-10s %s' % ('dt', [round(v or 0, 6) for v in cols['dt'][:6]]))

# %% [markdown]
# ## Or the running average, which widens instead of dropping
# The board accumulates and hands over the window, so a slow link loses
# resolution rather than records. It stops widening at 32767 samples: the sum
# is a signed 32-bit total and a single-ended code reaches 65535, so one more
# would wrap it and a wrapped sum divided by its count is not a mean.

# %%
live = daq.latest()
params = device.analog.scaling()
units = {f['signal']: f for f in daq.layout['fields']}

for name in live['mean']:
    f = units[name]
    to = scaling.converter(f['unit'], f['differential'], signal=name,
                           params=params)
    print('%-8s %9.1f codes = %9.3f %-2s  over %4d'
          % (name, live['mean'][name], to(live['mean'][name]),
             scaling.symbol(f['unit'], name), live['count'][name]))

# %%
daq.stop()
print(daq.state())
device.close()
