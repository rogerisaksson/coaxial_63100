# %% [markdown]
# # coaxial 63100 - the shaft angle, and whether it means anything
# `# %%` cells: opens as a notebook, runs as a script.
# `SIMULATED = True` needs no cable; every value is then invented.
#
# The A1335 sits under the shaft and reports an angle whatever is in front of
# it. **A reading is only a measurement if there is a magnet**, and the part
# says so in a different register from the one carrying the angle - so a
# script that reads only ANG gets a plausible number out of noise.
#
# The conversions live in `coaxial.angle` and nowhere else: `counts`,
# `degrees`, `kelvin`, `gauss`. They were spelled out at six call sites once,
# and two of them re-derived degrees and kelvin with their own literals.
#
# **This device has no start and no stop.** The board polls SPI4 as fast as
# the loop allows, from boot, and there is no interval to set - so acquire is
# the whole lifecycle and `state()` is all of it. Registers are reachable, but
# only inside `hold()`, which is the loop standing aside rather than stopping.

# %%
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'host'))          # the library lives here

from coaxial import Coaxial63100, angle

SIMULATED = False
REG = {'ANG': 0x20, 'STA': 0x22, 'ERR': 0x24, 'TSEN': 0x28, 'FIELD': 0x2A}

device = Coaxial63100(port='COM4', simulated_device=SIMULATED, power_afe=True)
sensor = device.angle            # the A1335 behind SPI4
sensor.open()
print(device)
print(sensor)

for _ in range(40):
    if sensor.state()['loop'] == 'running':
        break
    time.sleep(0.3)
print('loop:', sensor.state()['loop'])

# %% [markdown]
# ## The registers, raw and decoded
# Inside `hold()`: the poll loop owns SPI4, and a register read while it runs
# is two masters on one bus. Same rule as the IMU, same refusal.

# %%
sensor.hold()
try:
    said = dict((name, sensor.read(addr)['value'])
                for name, addr in REG.items())
finally:
    sensor.resume()

print('%-6s %6s  %s' % ('reg', 'raw', 'means'))
print('%-6s %6d  %.2f deg' % ('ANG', said['ANG'], angle.degrees(said['ANG'])))
print('%-6s %6d  %.1f K' % ('TSEN', said['TSEN'], angle.kelvin(said['TSEN'])))
print('%-6s %6d  %d gauss' % ('FIELD', said['FIELD'], angle.gauss(said['FIELD'])))
print('%-6s %6d  %d counts' % ('STA', said['STA'], angle.counts(said['STA'])))

# %% [markdown]
# ## Is there a magnet
# The datasheet asks for 300 to 1000 gauss. Below a few tens there is nothing
# to measure and the low twelve bits of ANG are noise - which is a plausible
# angle, not an obviously broken one, and that is what makes it worth
# checking before believing anything.

# %%
field = angle.gauss(said['FIELD'])
enough = field >= 30
print('%d gauss - %s' % (field, 'a magnet' if enough
                         else 'nothing in front of the sensor'))

# %% [markdown]
# ## Watch it sit still
# The honest test, and it needs no magnet to be informative: a stationary
# shaft should read the same angle twice. Spread over a few seconds is the
# measurement that says whether the number is one.

# %%
seen = []
for _ in range(12):
    seen.append(sensor.state()['degrees'])
    time.sleep(0.25)

print('%.2f to %.2f deg, spread %.2f, sd %.2f'
      % (min(seen), max(seen), max(seen) - min(seen),
         statistics.pstdev(seen)))
print('a still shaft reading that far apart is noise, not motion'
      if max(seen) - min(seen) > 5 else 'steady enough to be a reading')

# %% [markdown]
# ## How fast the loop runs
# The board polls SPI4 as fast as it allows, so this is the main loop's own
# rate - it falls the moment anything in that loop starts blocking, which is
# what `test_bench.py` guards against.

# %%
first = sensor.state()['updates']
started = time.time()
time.sleep(3)
print('%.0f readings a second' % ((sensor.state()['updates'] - first)
                                  / (time.time() - started)))

device.close()
print('registers untouched - this example only read')
