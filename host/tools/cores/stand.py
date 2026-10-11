"""The stand-in's heat and drive through ctypes: world/src's slots, their rows and views named.

The board's own observer (thermal/src/thermal_run.c) beside the truth the emulated worlds run
(world/src/world_heat.c), and its own law (drive/src) on the motor it models
(world/src/world_drive.c), a slot a stand-in board: what coaxial.simulated steps and reads. One
library a process, built once for its sources' bytes (tools.cores.build.cached).
"""
import ctypes
import functools
import os

from tools import REPO
from tools.cores import drive
from tools.cores.build import cached

SOURCES = ([os.path.join(REPO, 'world', 'src', name)
            for name in ('world_heat.c', 'world_stand.c', 'world_drive.c')]
           + [os.path.join(REPO, 'thermal', 'src', name)
              for name in ('thermal.c', 'thermal_ident.c', 'thermal_app.c', 'thermal_run.c')]
           + [os.path.join(REPO, 'ctrl', 'src', name) for name in ('ctrl.c', 'ctrl_loop.c')]
           + drive.SOURCES)
INCLUDES = [os.path.join(REPO, part)
            for part in ('world/inc', 'thermal/inc', 'drive/inc', 'ctrl/inc')]

#: The harness marks its own for export, and a PE linker then exports nothing else unasked.
EXPORT = ['-Wl,--export-all-symbols'] if os.name == 'nt' else []

#: The slots a process holds (WORLD_STANDS).
SLOTS = 64

#: world_stand_run's row (WORLD_STAND_IN), in order.
ROW = ('afe_on', 'switching', 'duty_u', 'duty_v', 'duty_w', 'amps_u', 'amps_v', 'amps_w',
       'link_volts', 'link_amps', 'speed_rpm', 't_dead_s', 'meter_locked', 'acting',
       'clamp_amps', 'sq_u', 'sq_v', 'sq_w')

#: The views (world_stand.h's enum) and how many floats the longest fills.
STATE, BUDGET, IDENT, NETWORK, TRUTH = range(5)
VIEW_FLOATS = 256

#: world_drive_read's row (WORLD_DRIVE_ROW), in order.
DRIVE_ROW = ('periods', 'trips', 'injecting', 'driven', 'duty_u', 'duty_v', 'duty_w',
             'amps_u', 'amps_v', 'amps_w', 'shaft')

#: world_drive_window's row (WORLD_DRIVE_WINDOW_ROW), in order.
WINDOW_ROW = ('sq_u', 'sq_v', 'sq_w', 'link_amps', 'periods')

#: world_drive_loop_read's row (WORLD_DRIVE_LOOP_ROW), in order: ctrl_loop_state_t's.
LOOP_ROW = ('running', 'playing', 'measured', 'command', 'hz', 'rows', 'free', 'queued_s',
            'played', 'idle', 'blind', 'setpoint', 'ref', 'value', 'estimate', 'out')


@functools.lru_cache(maxsize=None)
def library():
    """The stand-in's C, built and typed once a process."""
    lib = ctypes.CDLL(cached(SOURCES, INCLUDES, 'stand', EXPORT)[0])
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
    lib.world_drive_init.argtypes = [i, f]
    lib.world_drive_law.argtypes = [i]
    lib.world_drive_law.restype = ctypes.c_void_p
    lib.world_drive_rest.argtypes = [i]
    lib.world_drive_run.argtypes = [i, u, ctypes.c_bool, ctypes.c_bool]
    lib.world_drive_run.restype = u
    lib.world_drive_read.argtypes = [i, ctypes.POINTER(ctypes.c_double)]
    lib.world_drive_read.restype = i
    lib.world_drive_window.argtypes = [i, ctypes.POINTER(ctypes.c_double)]
    lib.world_drive_window.restype = i
    lib.world_drive_loop.argtypes = [i]
    lib.world_drive_loop.restype = ctypes.c_void_p
    lib.world_drive_loop_read.argtypes = [i, ctypes.POINTER(ctypes.c_double)]
    lib.world_drive_loop_read.restype = i
    p, u8, u16 = ctypes.c_void_p, ctypes.c_uint8, ctypes.c_uint16
    lib.ctrl_loop_slot.argtypes = [p, u8, u8, fp, u8]
    lib.ctrl_loop_wire.argtypes = [p, f, u8, u8, u16]
    lib.ctrl_loop_rows.argtypes = [p, ctypes.POINTER(u16), fp, u8]
    lib.ctrl_loop_run.argtypes = [p, ctypes.c_bool]
    for call in (lib.ctrl_loop_slot, lib.ctrl_loop_wire, lib.ctrl_loop_rows, lib.ctrl_loop_run):
        call.restype = ctypes.c_char_p
    lib.ctrl_rows_clear.argtypes = [p]
    return lib


def floats(values):
    """A C float array of `values`."""
    return (ctypes.c_float * len(values))(*values)


def view(slot, what):
    """View `what` of `slot`, its floats."""
    out = (ctypes.c_float * VIEW_FLOATS)()
    n = library().world_stand_read(slot, what, out)
    return list(out[:n])


def law(slot, ts):
    """Slot `slot`'s drive fresh at a period of `ts` s, its law named (tools.cores.drive)."""
    lib = library()
    lib.world_drive_init(slot, ts)
    return drive.Drive(lib, ts, at=lib.world_drive_law(slot))


def row(slot):
    """Slot `slot`'s drive row, named (DRIVE_ROW)."""
    out = (ctypes.c_double * len(DRIVE_ROW))()
    library().world_drive_read(slot, out)
    return dict(zip(DRIVE_ROW, out))


def window(slot):
    """Slot `slot`'s drive's window, named (WINDOW_ROW), taken."""
    out = (ctypes.c_double * len(WINDOW_ROW))()
    library().world_drive_window(slot, out)
    return dict(zip(WINDOW_ROW, out))


def loop(slot):
    """Slot `slot`'s loop as it stands, named (LOOP_ROW)."""
    out = (ctypes.c_double * len(LOOP_ROW))()
    library().world_drive_loop_read(slot, out)
    return dict(zip(LOOP_ROW, out))
