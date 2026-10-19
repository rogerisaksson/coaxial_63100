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
            for name in ('world_heat.c', 'world_stand.c', 'world_drive.c', 'world_sto.c')]
           + [os.path.join(REPO, 'thermal', 'src', name)
              for name in ('thermal.c', 'thermal_ident.c', 'thermal_app.c', 'thermal_run.c')]
           + [os.path.join(REPO, 'ctrl', 'src', name) for name in ('ctrl.c', 'ctrl_loop.c')]
           + [os.path.join(REPO, 'daq', 'src', 'daq.c'),
              os.path.join(REPO, 'filter', 'src', 'filter.c')]
           + drive.SOURCES)
INCLUDES = [os.path.join(REPO, part)
            for part in ('world/inc', 'thermal/inc', 'drive/inc', 'ctrl/inc',
                         'daq/inc', 'filter/inc')]

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
    lib.world_drive_pass.argtypes = [i, u, ctypes.c_bool, ctypes.c_bool, f,
                                    ctypes.POINTER(ctypes.c_double)]
    lib.world_drive_pass.restype = u
    lib.world_drive_read.argtypes = [i, ctypes.POINTER(ctypes.c_double)]
    lib.world_drive_read.restype = i
    lib.world_drive_window.argtypes = [i, ctypes.POINTER(ctypes.c_double)]
    lib.world_drive_window.restype = i
    lib.world_drive_loop.argtypes = [i]
    lib.world_drive_loop.restype = ctypes.c_void_p
    lib.world_drive_loop_read.argtypes = [i, ctypes.POINTER(ctypes.c_double)]
    lib.world_drive_loop_read.restype = i
    p, u8, u16, i16, i32 = (ctypes.c_void_p, ctypes.c_uint8, ctypes.c_uint16,
                            ctypes.c_int16, ctypes.c_int32)
    lib.ctrl_loop_slot.argtypes = [p, u8, u8, fp, u8]
    lib.ctrl_loop_wire.argtypes = [p, f, u8, u8, u16]
    lib.ctrl_loop_rows.argtypes = [p, ctypes.POINTER(u16), fp, u8]
    lib.ctrl_loop_run.argtypes = [p, ctypes.c_bool]
    for call in (lib.ctrl_loop_slot, lib.ctrl_loop_wire, lib.ctrl_loop_rows, lib.ctrl_loop_run):
        call.restype = ctypes.c_char_p
    lib.ctrl_rows_clear.argtypes = [p]
    lib.stand_sto_bytes.restype = u
    lib.stand_sto_init.argtypes = [p]
    lib.stand_sto_step.argtypes = [p, f, fp, i, fp]
    lib.stand_daq_bytes.restype = u
    lib.stand_daq_init.argtypes = [p, ctypes.POINTER(u8), u]
    lib.stand_daq_begin.argtypes = [p, ctypes.POINTER(u8), u8, u16, u16, u, u8, u16, u, i]
    lib.stand_daq_start.argtypes = [p, u]
    lib.stand_daq_start.restype = i
    lib.stand_daq_stop.argtypes = [p]
    lib.stand_daq_set_filter.argtypes = [p, fp, u8, u16]
    lib.stand_daq_set_filter.restype = ctypes.c_char_p
    lib.stand_daq_set_rung.argtypes = [p, u8, u16, fp, u8, u16]
    lib.stand_daq_set_rung.restype = ctypes.c_char_p
    lib.stand_daq_set_tone.argtypes = [p, u, u, i32, i32, u8, u, u]
    lib.stand_daq_set_tone.restype = ctypes.c_char_p
    lib.stand_daq_tone_on.argtypes = [p]
    lib.stand_daq_tone_on.restype = i
    lib.stand_daq_tone_cycles.argtypes = [p]
    lib.stand_daq_tone_cycles.restype = u
    lib.stand_daq_tone_poll.argtypes = [p, u, u]
    lib.stand_daq_triggers_per_record.argtypes = [p]
    lib.stand_daq_triggers_per_record.restype = u
    lib.stand_daq_window.argtypes = [p, ctypes.POINTER(i32), u16, u,
                                     ctypes.POINTER(u8), ctypes.POINTER(i16)]
    lib.stand_daq_take.argtypes = [p, ctypes.POINTER(u8), u16]
    lib.stand_daq_take.restype = u16
    lib.stand_daq_live_insert.argtypes = [p, u8, i32, u, u]
    lib.stand_daq_take_live.argtypes = [p, ctypes.POINTER(u), ctypes.POINTER(i32)]
    lib.stand_daq_take_live.restype = i
    lib.stand_daq_state.argtypes = [p, ctypes.POINTER(u)]
    return lib


def _biquad_floats(sections):
    """Flat (b0, b1, b2, a1, a2) c_float array for `sections`, or None."""
    flat = [float(c) for s in sections
            for c in ((s['b0'], s['b1'], s['b2'], s['a1'], s['a2'])
                      if isinstance(s, dict) else s)]
    return (ctypes.c_float * len(flat))(*flat) if flat else None


class StoCore:
    """One stand-in board's STO chain (world/src/world_sto.c)."""

    def __init__(self):
        lib = self._lib = library()
        self._mem = (ctypes.c_uint8 * lib.stand_sto_bytes())()
        self._in = (ctypes.c_float * 5)()
        self._out = (ctypes.c_float * 4)()
        lib.stand_sto_init(self._mem)

    def step(self, dt, pilot, link_volts, rail5, pumping):
        """(cinj, clevel, faultout, vgate) after `dt` s."""
        self._in[0], self._in[1], self._in[2] = pilot
        self._in[3] = link_volts
        self._in[4] = 1.0 if rail5 else 0.0
        self._lib.stand_sto_step(self._mem, dt, self._in, 1 if pumping else 0, self._out)
        return self._out[0], self._out[1], bool(self._out[2]), self._out[3]


class DaqCore:
    """One stand-in board's acquisition engine (daq/src/daq.c, filter/src/filter.c)."""

    def __init__(self, ring_bytes):
        lib = self._lib = library()
        self._mem = (ctypes.c_uint8 * lib.stand_daq_bytes())()
        self._ring = (ctypes.c_uint8 * ring_bytes)()
        self._sums = (ctypes.c_int32 * 10)()
        self._duties = (ctypes.c_uint8 * 16)()
        self._sensors = (ctypes.c_int16 * 20)()
        self._state = (ctypes.c_uint32 * 15)()
        self._meta = (ctypes.c_uint32 * 3)()
        self._slots = (ctypes.c_int32 * 40)()
        lib.stand_daq_init(self._mem, self._ring, ring_bytes)

    def begin(self, order, accumulate, decimate, interval_cycles, pins, sensors, records, adapt):
        arr = (ctypes.c_uint8 * len(order))(*order)
        self._lib.stand_daq_begin(self._mem, arr, len(order), int(accumulate), int(decimate),
                                  int(interval_cycles), int(pins), int(sensors), int(records),
                                  1 if adapt else 0)

    def start(self, now):
        return bool(self._lib.stand_daq_start(self._mem, int(now)))

    def stop(self):
        self._lib.stand_daq_stop(self._mem)

    def set_filter(self, sections, decimate):
        why = self._lib.stand_daq_set_filter(self._mem, _biquad_floats(sections),
                                             len(sections), int(decimate))
        return why.decode() if why else None

    def set_rung(self, rung, boxcar, sections, decimate):
        why = self._lib.stand_daq_set_rung(self._mem, int(rung), int(boxcar),
                                           _biquad_floats(sections), len(sections), int(decimate))
        return why.decode() if why else None

    def set_tone(self, hz, rate_hz, amplitude, offset, kind, cycles_per_second, now):
        why = self._lib.stand_daq_set_tone(self._mem, int(hz), int(rate_hz), int(amplitude),
                                           int(offset), int(kind), int(cycles_per_second),
                                           int(now))
        return why.decode() if why else None

    def tone_on(self):
        return bool(self._lib.stand_daq_tone_on(self._mem))

    def tone_cycles(self):
        return int(self._lib.stand_daq_tone_cycles(self._mem))

    def tone_poll(self, now, burst):
        self._lib.stand_daq_tone_poll(self._mem, int(now), int(burst))

    def triggers_per_record(self):
        return int(self._lib.stand_daq_triggers_per_record(self._mem))

    def push_window(self, sums, count, at, duties=None, sensors=None):
        for i, v in enumerate(sums):
            self._sums[i] = int(v)
        if duties is not None:
            for i, v in enumerate(duties):
                self._duties[i] = int(v)
        if sensors is not None:
            for b, words in sensors.items():
                for w, val in enumerate(words):
                    self._sensors[b * 4 + w] = int(val)
        self._lib.stand_daq_window(self._mem, self._sums, int(count), int(at),
                                   self._duties if duties is not None else None,
                                   self._sensors if sensors is not None else None)

    def take(self, max_records, stride):
        buf = (ctypes.c_uint8 * (max_records * stride))()
        got = self._lib.stand_daq_take(self._mem, buf, int(max_records))
        return bytes(buf[:got * stride])

    def live_insert(self, field, value, at, digital):
        self._lib.stand_daq_live_insert(self._mem, int(field), int(value), int(at), int(digital))

    def take_live(self):
        fresh = bool(self._lib.stand_daq_take_live(self._mem, self._meta, self._slots))
        slots = [(self._slots[f * 4], self._slots[f * 4 + 1],
                  self._slots[f * 4 + 2], self._slots[f * 4 + 3]) for f in range(10)]
        return fresh, self._meta[0], self._meta[1], self._meta[2], slots

    def state(self):
        self._lib.stand_daq_state(self._mem, self._state)
        s = self._state
        return {'running': bool(s[0]), 'done': bool(s[1]), 'stride': int(s[2]),
                'fields': int(s[3]), 'available': int(s[4]), 'produced': int(s[5]),
                'dropped': int(s[6]), 'capacity': int(s[7]), 'worst': int(s[8]),
                'rung': int(s[9]), 'rungs': int(s[10]), 'rung_changes': int(s[11]),
                'accumulate': int(s[12]), 'sections': int(s[13]), 'chain_decimate': int(s[14])}


_VIEW_BUF = (ctypes.c_float * VIEW_FLOATS)()
_ROW_BUF = (ctypes.c_double * len(DRIVE_ROW))()
_WIN_BUF = (ctypes.c_double * len(WINDOW_ROW))()
_LOOP_BUF = (ctypes.c_double * len(LOOP_ROW))()


def floats(values):
    """A C float array of `values`."""
    return (ctypes.c_float * len(values))(*values)


def view(slot, what):
    """View `what` of `slot`, its floats."""
    n = library().world_stand_read(slot, what, _VIEW_BUF)
    return _VIEW_BUF[:n]


def law(slot, ts):
    """Slot `slot`'s drive fresh at a period of `ts` s, its law named (tools.cores.drive)."""
    lib = library()
    lib.world_drive_init(slot, ts)
    return drive.Drive(lib, ts, at=lib.world_drive_law(slot))


def row(slot):
    """Slot `slot`'s drive row, named (DRIVE_ROW)."""
    library().world_drive_read(slot, _ROW_BUF)
    return dict(zip(DRIVE_ROW, _ROW_BUF))


def window(slot):
    """Slot `slot`'s drive's window, named (WINDOW_ROW), taken."""
    library().world_drive_window(slot, _WIN_BUF)
    return dict(zip(WINDOW_ROW, _WIN_BUF))


def loop(slot):
    """Slot `slot`'s loop as it stands, named (LOOP_ROW)."""
    library().world_drive_loop_read(slot, _LOOP_BUF)
    return dict(zip(LOOP_ROW, _LOOP_BUF))

