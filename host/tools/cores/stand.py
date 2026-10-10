"""The stand-in's heat through ctypes: world/src/world_stand.c's slots, its row and its views named.

The board's own observer (thermal/src/thermal_run.c) beside the truth the emulated worlds run
(world/src/world_heat.c), a slot a stand-in board: what coaxial.simulated.thermal steps and
reads. One library a process, built once for its sources' bytes (tools.cores.build.cached).
"""
import ctypes
import functools
import os

from tools import REPO
from tools.cores.build import cached

SOURCES = ([os.path.join(REPO, 'world', 'src', name) for name in ('world_heat.c', 'world_stand.c')]
           + [os.path.join(REPO, 'thermal', 'src', name)
              for name in ('thermal.c', 'thermal_ident.c', 'thermal_app.c', 'thermal_run.c')])
INCLUDES = [os.path.join(REPO, part) for part in ('world/inc', 'thermal/inc')]

#: The slots a process holds (WORLD_STANDS).
SLOTS = 64

#: world_stand_run's row (WORLD_STAND_IN), in order.
ROW = ('afe_on', 'switching', 'duty_u', 'duty_v', 'duty_w', 'amps_u', 'amps_v', 'amps_w',
       'link_volts', 'link_amps', 'speed_rpm', 't_dead_s', 'meter_locked', 'acting')

#: The views (world_stand.h's enum) and how many floats the longest fills.
STATE, BUDGET, IDENT, NETWORK, TRUTH = range(5)
VIEW_FLOATS = 256


@functools.lru_cache(maxsize=None)
def library():
    """The stand-in's C, built and typed once a process."""
    lib = ctypes.CDLL(cached(SOURCES, INCLUDES, 'stand')[0])
    i, f, u, fp = ctypes.c_int, ctypes.c_float, ctypes.c_uint32, ctypes.POINTER(ctypes.c_float)
    lib.world_stand_init.argtypes = [i, i, f, f, f, f, f, u]
    lib.world_stand_envelope.argtypes = [i, fp, u, f, f, f]
    lib.world_stand_room.argtypes = [i, f, f, f]
    lib.world_stand_application.argtypes = [i, i]
    lib.world_stand_network.argtypes = [i, i, i, f, f]
    lib.world_stand_network.restype = ctypes.c_bool
    lib.world_stand_identify.argtypes = [i]
    lib.world_stand_every.argtypes = [i, u]
    lib.world_stand_wep.argtypes = [i, u]
    lib.world_stand_airspeed.argtypes = [i, f, u, f]
    lib.world_stand_cycle.argtypes = [i, f, f, f]
    lib.world_stand_run.argtypes = [i, fp, f, fp]
    lib.world_stand_place.argtypes = [i, i, f]
    lib.world_stand_phase_r.argtypes = [i, f]
    lib.world_stand_settle.argtypes = [i, fp, f]
    lib.world_stand_read.argtypes = [i, i, fp]
    lib.world_stand_read.restype = i
    return lib


def floats(values):
    """A C float array of `values`."""
    return (ctypes.c_float * len(values))(*values)


def view(slot, what):
    """View `what` of `slot`, its floats."""
    out = (ctypes.c_float * VIEW_FLOATS)()
    n = library().world_stand_read(slot, what, out)
    return list(out[:n])
