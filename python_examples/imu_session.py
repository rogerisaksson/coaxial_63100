# %% [markdown]
# # coaxial 63100 - the IMU, and the three things it refuses over
# `# %%` cells: opens as a notebook, runs as a script.
# `SIMULATED = True` needs no cable; every value is then invented.
#
# The BNO085 is the part on this board with the most ways to look broken
# while working. Each of these cost hours once and is one line here:
#
#   1. AFE_ON powers it. With the rail down it answers reads, resets and
#      advertises normally and acts on no write at all - so the fault
#      presents as SPI, and a day went into SPI before the supply was
#      checked.
#   2. The poll loop owns SPI2. A bus call while it runs is two masters on
#      one bus and is refused - `hold()`, the call, `resume()`.
#   3. The part forgets its Set Feature on every reset, and a reset is what
#      `resume()` does. The board re-applies it, but only once the part has
#      gone quiet: a write into the middle of the advertisement is accepted
#      and discarded, and then nothing streams for ever.

# %%
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'host'))          # the library lives here

from coaxial import Coaxial63100

SIMULATED = False
ROTATION_VECTOR = 0x05
INTERVAL_US = 20000                                # 50 Hz

rig = Coaxial63100(port='COM4', simulated_device=SIMULATED, power_afe=True)
rig.open()
imu = rig.board.imu
print(rig)

# %% [markdown]
# ## Wait for the loop
# The board brings the part up itself, and that takes a reset and an
# advertisement. `loop` says where it is; anything before `running` is not a
# fault.

# %%
for _ in range(40):
    if imu.state()['loop'] == 'running':
        break
    time.sleep(0.3)
print('loop:', imu.state()['loop'])

# %% [markdown]
# ## Ask it who it is
# Inside `hold()`, because this is a bus call. The refusal without it names
# the reason and what to do, which is what a refusal is for.

# %%
imu.hold()
try:
    print('product id:', imu.product_id())
    imu.feature(ROTATION_VECTOR, INTERVAL_US)
finally:
    imu.resume()

# %% [markdown]
# ## Read the record, not the bus
# `state()` and `latest()` read what the poll loop already collected, so they
# need no hold at all. That is what the shared record is for: one master on
# the bus, any number of readers above it.

# %%
time.sleep(2)
for _ in range(5):
    said = imu.state()
    q = said['quaternion']
    print('%6d reports  %4d errors  last %-24s  q %s'
          % (said['updates'], said['errors'], said['last_fault'],
             'none yet' if not q else
             '%+.3f %+.3f %+.3f %+.3f' % (q['i'], q['j'], q['k'], q['real'])))
    time.sleep(1)

# %% [markdown]
# ## How fast is it going
# Measured rather than asked for: the part adopts what it can, and says so
# through a Get Feature Response. 400 Hz is its ceiling for this report and
# the transfer is what decides whether you get it.

# %%
first = imu.state()['updates']
started = time.time()
time.sleep(4)
print('%.1f reports a second' % ((imu.state()['updates'] - first)
                                 / (time.time() - started)))

# %% [markdown]
# ## Put it back
# Disabling the report is an interval of zero. Leaving it streaming costs the
# main loop for whoever opens the board next.

# %%
imu.hold()
try:
    imu.feature(ROTATION_VECTOR, 0)
finally:
    imu.resume()
rig.close()
print('rotation vector disabled, supply back the way it was found')
