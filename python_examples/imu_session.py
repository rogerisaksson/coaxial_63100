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
#
# Same lifecycle as `daq_session`, in the part's own words: a **Set Feature**
# is start, `state()` is acquire, and an interval of **zero** is stop. What
# differs is who runs the loop - the board does, so acquire reads a record
# rather than a converter.

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

device = Coaxial63100(port='COM4', simulated_device=SIMULATED, power_afe=True)
device.open()
imu = device.imu                 # the BNO085 behind SPI2
print(device)

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
# ## The board itself, turning
# Same shape as the thermal picture, and for the same reason: a quaternion is
# four numbers nobody reads as an attitude. This is the board's own CAD
# export - `render/models`, decimated once to 12 % of its 419,338 triangles
# and cached - rasterised the way three.js's AsciiEffect does it, flat
# shading and one point light.
#
# It is the model that turns, not the camera. The camera sits still at 55
# degrees up, which is where the parts stand up: measured over the same mesh,
# at 90 the board is two characters wide and flat by geometry.

# %%
from coaxial import orientation                                # noqa: E402


def attitude():
    """The quaternion as a tuple, or level if the part has not reported."""
    q = imu.state()['quaternion']
    return (q['i'], q['j'], q['k'], q['real']) if q else (0.0, 0.0, 0.0, 1.0)


for _ in range(3):
    print(orientation.picture(attitude(), width=54, height=22))
    time.sleep(1)

# %% [markdown]
# ## The same mesh, fewer steps of ink
# `ascii3d.render` takes the ramp, so the cartoon look costs no new code: ten
# characters is a photograph of the surface, four is a drawing of the parts.
# Shorter posterises the shading, which is what makes an edge read as an edge
# rather than as one more shade of the face beside it.

# %%
CARTOON = ' .+#'                                    # four steps, space darkest
print(orientation.render(attitude(), width=54, height=22, ramp=CARTOON))

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
device.close()
print('rotation vector disabled, supply back the way it was found')
