# %% [markdown]
# # coaxial 63100 - the gate drivers
# `# %%` cells: opens as a notebook, runs as a script.
# **This arms a power stage.** The 2EDL8034 has no interlock; TIM1's dead
# time is all there is between the two FETs of a leg.

# %%
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'host'))          # the library lives here

from coaxial import Coaxial63100, scaling

SIMULATED = False
DUTY = 0.25

daq = Coaxial63100(port='COM4', simulated_device=SIMULATED, power_afe=False)
daq.open()
print(daq)

# %% [markdown]
# ## Dead time, before anything else
# Refuses at zero. Read from BDTR every time, not remembered.

# %%
state = daq.gates.check()
print('DTG %d, period %d ticks' % (state['deadtime'], state['period']))

# %% [markdown]
# ## Which half of the board you get
# AFE_ON powers the ADC reference. On this bench board the same pin, gated
# the other way, is what gives the drivers supply - so with the drivers
# powered the board refuses to convert, and there are no currents.

# %%
daq.write(digital={'AFE_ON': True})            # currents real, no drive
time.sleep(0.3)
daq.configure(accumulate=8, digital=False)
daq.start()

# %% [markdown]
# ## Arm, then set a duty
# Arming is a call. A duty write is refused until it has been made.

# %%
# ignore_interlock because this bench board is not modified: the schematic
# wants Cinj and Clevel both above 3 V first and they read 0.79 and 0.08.
daq.gates.arm(bypass_sto=True, ignore_interlock=True)
print('armed:', daq.gates.armed())
daq.write(analog={'Phase U': DUTY, 'Phase V': DUTY, 'Phase W': DUTY})

# %% [markdown]
# ## The six gate signals, as one instant
# One IDR read on the board. Six asks would be six instants.

# %%
snap = daq.board.gate_drivers.state()
print('CNT %d of %d' % (snap['pins_at'], snap['period'] - 1))
for leg in ('U', 'V', 'W'):
    high, low = snap['pins'][leg + 'H'], snap['pins'][leg + 'L']
    print('  %s  high %d  low %d%s'
          % (leg, high, low, '   BOTH ON' if high and low else ''))

# %% [markdown]
# ## Current, DC link and the two supply senses, with ripple
# The live accumulator carries a count, a lowest and a highest per channel,
# so ripple is measured rather than inferred from one sample.
#
# The phase readings here are OFFSET, not current: nothing is armed and no
# current flows, so what they show is where each channel sits with zero
# through the shunt. Zeroing them is `calibration.zero(index)`, against an
# instrument - invariant 7. The ripple beside them is real either way.

# %%
live = daq.latest()
params = daq.board.analog.scaling()      # the board's record, not this file's
units = {f['signal']: (f['unit'], f['differential'])
         for f in daq.layout['fields']}
for name in ('Phase U', 'Phase V', 'Phase W', 'DC bus', '+5V', 'Vgate'):
    unit, diff = units[name]
    to = scaling.converter(unit, diff, signal=name,   # three mV channels,
                           params=params)             # three dividers
    print('%-8s %+9.3f %-2s  p-p %7.3f  over %d'
          % (name, to(live['mean'][name]), scaling.UNIT_SYMBOL.get(unit, ''),
             abs(to(live['highest'][name]) - to(live['lowest'][name])),
             live['count'][name]))

# %% [markdown]
# ## A timed burst
# Unlimited rate, finite record count - the board allows that combination
# because the run ends. Start and stop are a round trip each, about 15 ms,
# so an ask under that is bounded by the link and the stamps say so.

# %%
daq.stop()
daq.board.daq.configure([f['signal'] for f in daq.layout['fields']],
                        digital=False, accumulate=1, records=512,
                        interval_us=0)
daq.start()
time.sleep(0.100)
daq.stop()

got = []                                   # read() is one reply, not the lot
while True:
    batch = daq.board.daq.acquire(layout=daq.layout)
    if not batch:
        break
    got.extend(batch)

span = (got[-1]['at'] - got[0]['at']) / 475e6 if len(got) > 1 else 0.0
print('%d records over %.3f ms, %.0f us apart, %d dropped'
      % (len(got), span * 1e3, span * 1e6 / max(1, len(got) - 1),
         daq.state()['dropped']))

# %%
daq.gates.disarm()
daq.close()
