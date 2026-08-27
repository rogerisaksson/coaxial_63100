# %% [markdown]
# # A DAQ session on the coaxial 63100
#
# Connect, set the clock, configure the acquisition, run the bridge, then
# read N blocks of measurements.
#
# The `# %%` markers make this a notebook in VS Code and in Jupyter, and it
# is still an ordinary Python file you can run with `python daq_session.py`.
# Either way it does the same thing in the same order.
#
# Set `SIMULATED = True` and it needs no board at all. Every value is then
# invented, and it says so on every line - a number from nowhere and a
# number from hardware must never look alike.

# %%
import time

from coaxial_63100 import Coaxial63100

SIMULATED = False        # True needs no cable
PORT = 'COM4'            # Windows; on Linux something like /dev/ttyACM0
BAUD = 115200            # the debug probe's virtual COM port
BLOCKS = 20              # how many blocks of measurements to read

# Off, and read the cell about it before turning it on. This is the only
# switch here that puts energy anywhere.
RUN_BRIDGE = False

daq = Coaxial63100(port=PORT, baud=BAUD, simulated_device=SIMULATED)
daq.open()
print(daq)

# %% [markdown]
# ## What is on this board?
#
# Ask it. The channel list is not written down here, so a board that grows
# a channel shows it without this file being edited.

# %%
print('channels:', daq.channels())

# %% [markdown]
# ## Set the clock from this PC
#
# The board has no clock of its own, so every timestamp it produces is a
# raw CPU cycle count. This measures where that counter is and how fast it
# actually runs. After it, every record gets a real time on it.

# %%
sync = daq.set_time_from_pc(seconds=3.0)
print(sync)
print('the counter runs %+.1f ppm off its nominal %d Hz'
      % (sync.error_ppm, sync.nominal_hz))

# %% [markdown]
# ## Configure the acquisition
#
# `accumulate=8` sums eight samples into every record. That is how you
# average without throwing anything away: the record carries the sum and
# the count, and the reader divides.
#
# `rate_hz=None` lets the board decide, from its own record size and the
# speed of the link it is answering on. That is the safe default - ask for
# more than the link can carry and records get dropped.

# %%
layout = daq.configure_daq(
    channels=['Phase U', 'Phase V', 'Phase W', 'NTC', 'DC bus'],
    rate_hz=None,
    accumulate=8,
    digital=True,
)

print('record is %d bytes:' % layout['stride'])
for field in layout['fields']:
    print('   %-8s %s' % (field['signal'], field['unit'] or 'raw code'))
print('   digital pins:', ', '.join(p['signal'] for p in layout['pins']))

# %% [markdown]
# ## Optional: run the bridge
#
# **This is not part of a measurement session and it is off by default.**
# Everything above and below works with the bridge cold.
#
# `bypass_sto=True` disconnects the Safe Torque Off break input, which is
# the interlock that stops the gates moving. The argument for it being safe
# was that the STO chain gates the gate drivers' own supply, so the outputs
# toggle into an unpowered stage - and on the bench board that argument did
# not hold: 25 % duty on all three phases tripped the hot-swap's
# over-current protection and took the board down. Equal duty puts no
# voltage between the legs, so it was not phase current; something else
# drew it, and it is not understood yet.
#
# So: leave this off unless you know what is on the other end of those six
# pins, and have a supply you are willing to trip.

# %%
if RUN_BRIDGE:
    bridge = daq.configure_pwm(duty=0.25, bypass_sto=True)
    print('PWM %d Hz, dead time %d ticks, duty %s'
          % (475e6 / 2 / (bridge['period'] - 1) / 2,
             bridge['deadtime'], bridge['duty']))
else:
    print('bridge left cold - set RUN_BRIDGE to change that, and read the '
          'cell above first')

# %% [markdown]
# ## The measurement loop
#
# `daq.blocks(N)` yields N non-empty blocks. Each block is a list of
# records, and each record is a plain dict.

# %%
daq.start()

first = last = None
for number, block in enumerate(daq.blocks(BLOCKS), start=1):
    record = block[-1]
    if first is None:
        first = record['time']
    last = record['time']

    # A channel's value is the SUM of `samples` readings, not one reading.
    # Divide to get the mean - that is what averaging on the board buys you.
    n = record['samples']
    print('block %2d  %2d records  %s  U %+7.1f  V %+7.1f  NTC %7.1f  %s'
          % (number, len(block),
             time.strftime('%H:%M:%S', time.localtime(record['time'])),
             record['Phase U'] / n, record['Phase V'] / n,
             record['NTC'] / n,
             'AFE on' if record['digital']['AFE_ON'] else 'AFE OFF'))

daq.stop()
print('\n%d blocks over %.3f s' % (BLOCKS, last - first))

# %% [markdown]
# Those are means. Each record holds the **sum** of `accumulate` readings
# and `record['samples']` says how many, so dividing is the whole of it.
# The sum is what the board sends because it keeps the bits an average
# would throw away.

# %% [markdown]
# ## How did it go?
#
# `dropped` is the board saying the link could not keep up. If it climbs,
# raise `accumulate` - that reduces the number of records without losing
# any samples, and it happens on the board before a byte is sent.

# %%
state = daq.status()
print('produced %d, dropped %d, buffered %d'
      % (state['produced'], state['dropped'], state['available']))

# %% [markdown]
# ## The other way to read
#
# `latest()` takes a running average instead of draining a buffer. It
# cannot drop: a slow link widens its window rather than losing samples, so
# over a bad connection this is the one to use. Each channel carries its
# own count, because they are not sampled at the same instant.

# %%
daq.start()
time.sleep(0.5)
live = daq.latest()
for name, mean in live['mean'].items():
    print('   %-8s mean %10.1f over %4d samples'
          % (name, mean, live['count'][name]))
daq.stop()

# %% [markdown]
# ## Put it all back
#
# Gates down, break input restored, port released.

# %%
if RUN_BRIDGE:
    daq.stop_pwm()
daq.close()
print('closed')
