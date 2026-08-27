# %% [markdown]
# # coaxial 63100 - a DAQ session
# `# %%` cells: opens as a notebook, runs as a script.
# `SIMULATED = True` needs no cable; every value is then invented.

# %%
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'host'))          # the library lives here

from coaxial import Coaxial63100

SIMULATED = False
BLOCKS = 20

daq = Coaxial63100(port='COM4', baud=115200, simulated_device=SIMULATED)
daq.open()
print(daq)

# %% [markdown]
# ## What it has. Asked, not hardcoded.

# %%
print('in :', daq.channels())
print('out:', daq.outputs())

# %% [markdown]
# ## Clock
# The board counts CPU cycles and nothing else. This puts a real time on
# every record - UTC, not this PC's idea of it, which was 947 ms out.

# %%
print(daq.set_time_from_pc())

# %% [markdown]
# ## Configure
# `accumulate=8` sums 8 samples per record - averaging that loses nothing.
# `rate_hz=None` lets the board pick what the link carries.

# %%
daq.configure_daq(['Phase U', 'NTC'], rate_hz=None, accumulate=8,
                  digital=True)

# %% [markdown]
# ## Write
# Digital by name. Analog would be a PWM duty - there is no DAC - and it
# arms the gate drivers, so it is not in this example.

# %%
print(daq.daq_write(digital={'UART5_TERM': False}))

# %% [markdown]
# ## Read N blocks
# A channel's value is the SUM of `samples` readings. Divide for the mean.

# %%
daq.start()
for n, block in enumerate(daq.blocks(BLOCKS), 1):
    r = block[-1]
    print('%2d  %.3f  NTC %8.1f  AFE %s'
          % (n, r['time'], r['NTC'] / r['samples'], r['digital']['AFE_ON']))
daq.stop()

# %% [markdown]
# ## Or: the running average, which cannot overflow
# `read()` drains a buffer that drops when full. This widens its window
# instead, so it is the one for a slow link.

# %%
daq.start()
live = daq.latest()
for name in live['mean']:
    print('%-8s %9.1f  over %4d  [%d..%d]'
          % (name, live['mean'][name], live['count'][name],
             live['lowest'][name], live['highest'][name]))
daq.stop()

# %%
print(daq.status())
daq.close()
