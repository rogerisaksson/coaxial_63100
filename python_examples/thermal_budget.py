# %% [markdown]
# # coaxial 63100 - the thermal budget
# `# %%` cells: opens as a notebook, runs as a script.
#
# A burst does not ask how hot the board is. It asks **how long it may stay
# at this power** - and that is a different number, one the observer can only
# give because it dead-reckons from power and time between NTC samples.
#
# The board holds the ceilings; it does not invent them. They arrive the same
# way the thermistor constants do, and the board reports the margin rather
# than a verdict. What it *does* act on is a trip: at a limit it drops MOE and
# every gate goes to its idle level in hardware.
#
# **The observer needs no start either.** It runs on the board from boot and
# integrates whether anybody is reading - `set_sample()` is its configure,
# `state()` and `budget()` are acquire, and there is nothing to stop. That is
# what lets it answer for a burst that has already finished.

# %%
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'host'))

from coaxial import Coaxial63100

SIMULATED = False
DUTY = 0.50

device = Coaxial63100(port='COM4', power_afe=False,
                      simulated_device=SIMULATED)
observer = device.thermal        # the model runs on the board, not here
observer.open()
print(device)
print(observer)

# %% [markdown]
# ## What the ceilings are, and which of them is measured
# Two come from datasheets already recorded in this tree. The rest are
# estimates and are marked as such in `thermal_soa_defaults` - a limit nobody
# measured must never look measured.

# %%
budget = observer.budget()
for name, used in sorted(budget['used'].items(), key=lambda kv: -kv[1]):
    print('  %-11s %5.1f %% of its budget' % (name, 100.0 * used))

# %% [markdown]
# ## The one line that matters
# `worst` is the node with least room. `seconds_to_limit` is the dead
# reckoning at the power going in right now - `None` means it is not heading
# for a limit at all, which is a different answer from "a long time".

# %%
print('worst : %s at %.1f %%' % (budget['worst_node'],
                                 100.0 * budget['worst']))
print('left  : %s' % (('%.1f s' % budget['seconds_to_limit'])
                      if budget['seconds_to_limit'] is not None
                      else 'not heating'))

# %% [markdown]
# ## A burst, planned rather than hoped
# AFE_ON off FIRST: the gate is inverted, so the drivers only have supply
# while the analog front end does not. Arming with it on gives six switching
# inputs and nothing behind them.

# %%
device.afe.disable()
device.gates.arm(bypass_sto=True, ignore_interlock=True)
load = {'Phase %s' % leg: DUTY for leg in ('U', 'V', 'W')}
device.write(analog=load)

try:
    for _ in range(20):
        b = observer.budget()
        left = b['seconds_to_limit']
        print('%5.1f %%  %-11s  %s%s'
              % (100.0 * b['worst'], b['worst_node'],
                 ('%.1f s left' % left) if left is not None else 'steady',
                 '  THROTTLING' if b['throttling'] else ''))
        if b['tripped']:
            print('the board stopped the stage itself')
            break
        time.sleep(3.0)
finally:
    # One try per step: a failed first undo must not skip the disarm.
    for undo in (lambda: device.write(analog=dict.fromkeys(load, 0.0)),
                 device.gates.disarm):
        try:
            undo()
        except Exception as exc:            # noqa: BLE001 - report, then go on
            print('undo: %s' % exc)

# %% [markdown]
# ## Moving a ceiling
# Zero disables a node's limit, which is what a node with no measurement
# behind it deserves - better than a guess that reads like a specification.

# %%
print(observer.set_limit('board', 105.0, throttle_at=0.85))
device.close()
