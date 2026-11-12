"""Her drives' pass whole in C (wbc_pass_tick): her surety, the step, its clamps, her WEP in one call.

    g['tick'] = tick.laid(g, hop_k, wep_s, wep_fill)    # on her law's laid g: its drives and surety field
    motion.drives(g, dt, g['tick'][:2])                 # a pass, her rest told it (`se2.limp`)

`machine.law.motion.drives_twin`'s arithmetic, Python's floats and numpy's arrays operation for operation (test_tick):
its python was 32 us a 1 kHz pass, 24 of them around its two C calls.
"""
import ctypes
import functools

import numpy as np

from tools.cores import wbc

_d, _vp = ctypes.c_double, ctypes.c_void_p


class Tick(ctypes.Structure):

    """wbc_tick_t: her drives' pass, pointing at her law's arrays, its drives' step and its surety field."""

    _fields_ = [('drives', _vp), ('imp', _vp), ('field', _vp), ('rng', _vp), ('kept', _vp), ('limit', _vp),
                ('hop_k', _d), ('wep_s', _d), ('wep_fill', _d), ('surety', _d), ('left', _d), ('vec', _d * 5)]


@functools.lru_cache(maxsize=None)
def _typed():
    """The pass typed, wbc_tick_t's size the C's."""
    lib = wbc.library()
    lib.wbh_tick_size.restype = ctypes.c_int
    if lib.wbh_tick_size() != ctypes.sizeof(Tick):
        raise ValueError('wbc_tick_t is %d bytes, its mirror %d' % (lib.wbh_tick_size(), ctypes.sizeof(Tick)))
    lib.wbh_pass_tick.argtypes, lib.wbh_pass_tick.restype = [_vp, _d, _d, _d, _d], None
    return lib.wbh_pass_tick


def laid(g, hop_k, wep_s, wep_fill):
    """(its address, the pass, the struct, its surety's vec viewed) on her law's laid `g`: its drives' step
    (`g['drives']`), surety field (`g['surety_field']`), setpoints' ranges, the step's clamps and those told."""
    p, kept, _step = g['drives']
    imp, field, _s = g['surety_field'].laid
    t = Tick(drives=ctypes.addressof(p), imp=imp, field=field, rng=wbc.pointer(g['rng']),
             kept=wbc.pointer(kept['limit']), limit=wbc.pointer(g['cmd']['limit']), hop_k=hop_k, wep_s=wep_s,
             wep_fill=wep_fill)
    t._kept = (p, kept, g['surety_field'], g['rng'], g['cmd']['limit'])
    return ctypes.addressof(t), _typed(), t, np.ctypeslib.as_array(t.vec)

