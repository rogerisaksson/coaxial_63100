# %% [markdown]
# # coaxial 63100 - two sessions, one board
# `# %%` cells: opens as a notebook, runs as a script.
# `SIMULATED = True` needs no cable; every value is then invented.
#
# The board is one slave on one wire, so two masters split a frame. A broker
# owns the port and forwards Modbus requests unchanged - unit, function,
# payload - and serialises them. What it removes is the exclusive OWNERSHIP,
# not the exclusivity of the wire.
#
# NOBODY STARTS IT. The first session spawns one for the port it found, and
# it takes itself down when its last client goes. This example does not
# mention it until it asks how many are attached.

# %%
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'host'))          # the library lives here

from coaxial import Coaxial63100, broker

SIMULATED = False
PORT = 'COM4'

first = Coaxial63100(port=PORT, simulated_device=SIMULATED)
first.open()
print(first)

# %% [markdown]
# ## Who else is here
# `broker.clients()` counts the sessions using the port. Asking is not using:
# this attaches and closes, and does not count itself - otherwise the last
# one out would be whoever wondered whether anybody was in.

# %%
print('serving:', (broker.serving() or {}).get('serial', 'nothing'))
print('sessions:', broker.clients())

# %% [markdown]
# ## A second session, in its own process
# It opens the same port. Without the broker this is `could not open port`;
# with it, both talk. Nothing here says which - that is the point.

# %%
SECOND = '''
import sys
sys.path.insert(0, %r)
from coaxial import Coaxial63100
with Coaxial63100(port=%r) as rig:
    print('   second session sees firmware', rig.board.system.version()['firmware'])
    import time
    time.sleep(6)
''' % (sys.path[0], PORT)

other = subprocess.Popen([sys.executable, '-c', SECOND])
time.sleep(3)
print('sessions now:', broker.clients())

# %% [markdown]
# ## Both read the same board
# Whatever one session does to the board, the other sees. A reading that
# moved because somebody else armed the stage should not be a mystery, which
# is why the views put the count in their banner.

# %%
print('first  session sees firmware',
      first.board.system.version()['firmware'])
print('and a dead time of %d ns' % first.gates.state()['deadtime_ns'])

other.wait()
time.sleep(1)
print('sessions after it left:', broker.clients())

# %% [markdown]
# ## Leaving
# The last one out takes the broker down, so the port is free for anything
# that wants it raw - the conformance suite sends deliberately malformed
# frames, which is the one thing a broker cannot forward.

# %%
first.close()
time.sleep(1)
print('serving after the last session left:',
      (broker.serving() or {}).get('serial', 'nothing'))
