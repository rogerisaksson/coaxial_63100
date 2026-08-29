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
# `accumulate=8` sums 8 samples per record - averaging that loses nothing.
# `rate_hz=None` lets the board pick what the link carries.

# %%
daq.configure(['Phase U', 'NTC'], rate_hz=None, accumulate=8, digital=True)
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
# ## Acquire, in a loop
# A channel's value is the SUM of `samples` readings. Divide for the mean.
# `acquire()` drains what has arrived since the last call; the buffer drops
# when full, so a slow reader loses records rather than falling behind.
#
# The task buffers converter codes and scales nothing, so the code is what
# arrives. `scaling()` is the board's own record - the thermistor it was
# told it has, not one written down here (invariant 7).

# %%
ntc = device.analog.scaling()['ntc']
print('scaling from:', ntc.name)

for n in range(1, BLOCKS + 1):
    block = daq.acquire()
    if not block:
        continue
    r = block[-1]
    code = r['NTC'] / r['samples']
    print('%2d  %s  NTC %7.1f = %5.2f C  AFE %s'
          % (n, time.strftime('%H:%M:%S', time.localtime(r['time'])),
             code, ntc.celsius(code), r['digital']['AFE_ON']))

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
