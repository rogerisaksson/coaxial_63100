"""wbc_gmo.c through ctypes: her momentum observer's struct mirrored, its calls typed, its envelope."""
import ctypes
import os
import re

from tools import REPO
from tools.cores.wbc import DRIVEN, N, _define

_d, _i, _vp = ctypes.c_double, ctypes.c_int, ctypes.c_void_p


def _gmo_define(name):
    with open(os.path.join(REPO, 'wbc', 'inc', 'wbc_gmo.h'), encoding='utf-8') as f:
        found = re.search(r'#define %s +(\d+)' % name, f.read())
    if not found:
        raise RuntimeError('%s is not defined in wbc_gmo.h' % name)
    return int(found.group(1))


#: The observer's sizes, as wbc_gmo.h defines them; her chain's body in doubles (wbc_body_t).
GMO_P, GMO_E, GMO_PUSH, GMO_C, GMO_PAIRS, GMO_T, GMO_S, GMO_M = (_gmo_define(n) for n in (
    'WBC_GMO_POINTS', 'WBC_GMO_EDGES', 'WBC_GMO_PUSHES', 'WBC_GMO_CAPSULES', 'WBC_GMO_PAIRS', 'WBC_GMO_TOUCH',
    'WBC_GMO_SHARES', 'WBC_GMO_MANIFOLDS'))
GMO_F = GMO_PUSH + N - 6 + GMO_T


def horizon():
    """Her pairs' envelope, m (`horizon`): the preload distance of working_envelope_scoper.lisp."""
    from machine.texts import kernel
    return kernel.said('evaluate-operating-point-horizon', 'manifold-preload-distance-mm') * 1e-3
_LINKS = _define('WBC_LINKS')
BODY_DOUBLES = (2 * 12 * _LINKS + (N - 6) + 2 * 3 * _LINKS + N + 3 * 6 * _LINKS + 10 * _LINKS + 6 * _LINKS + _LINKS
                + 3 * _LINKS + N * N + N)


#: A row of the split's matrices, padded to whole 64-byte lines (WBC_GMO_LD).
GMO_LD = (GMO_F + 7) // 8 * 8
_FIELDS = [('a', (_d * GMO_LD) * N), ('h', (_d * GMO_LD) * GMO_F), ('k', (_d * GMO_LD) * GMO_F), ('ag', (_d * GMO_LD) * N),
           ('w', _d * GMO_LD), ('alpha', _d), ('slip', _d), ('eps', _d), ('weight', _d), ('upright', _d), ('reach', _d),
           ('still', _d), ('slide', _d * 3), ('horizon', _d), ('loss', _d * N), ('driven_at', _i * DRIVEN),
           ('driven_joint', _i * DRIVEN), ('link', _i * GMO_P), ('local', (_d * 3) * GMO_P),
           ('edge', ((_d * 3) * GMO_E) * GMO_P), ('edges', _i * GMO_P), ('col', _i * GMO_P), ('whole', _i), ('pushes', _i),
           ('gated', _i * GMO_P), ('world', _i * GMO_P), ('soled', _i * GMO_P), ('stop_lo', _d * (N - 6)),
           ('stop_hi', _d * (N - 6)), ('stopped', _i * (N - 6)), ('capsules', _i), ('capsule_link', _i * GMO_C),
           ('capsule_r', _d * GMO_C), ('capsule_a', (_d * 3) * GMO_C), ('capsule_b', (_d * 3) * GMO_C), ('pairs', _i),
           ('pair', (_i * 2) * GMO_PAIRS), ('listed', _i), ('list', _i * GMO_PAIRS), ('staged', ((_d * 3) * 2) * GMO_C),
           ('share_point', _i * GMO_S), ('share_manifold', _i * GMO_S), ('share_link', _i * GMO_S),
           ('share_axis', (_d * 3) * GMO_S), ('tau', _d * DRIVEN), ('acc', _d * 3), ('span', _d), ('primed', _d),
           ('u', _d * N), ('r', _d * N), ('f', _d * GMO_F), ('force', _d * GMO_M), ('at', (_d * 3) * GMO_P),
           ('touched', _i * GMO_T), ('past', _i * (N - 6)), ('stagings', _i), ('prior', _d * GMO_P),
           ('speed', _d * GMO_P), ('expected', _d * GMO_F), ('rounds', _i), ('lives', _i), ('live', _i * GMO_F),
           ('spare', _i), ('body', _d * BODY_DOUBLES)]
#: The members whose offsets the C says (wbh_gmo_offsets), in its order.
OFFSETS = ('a h k ag w alpha loss driven_at local edges whole pushes soled stop_lo stopped capsule_r pairs list staged '
           'share_point share_axis f at touched stagings prior rounds live spare body').split()


class _Laid(ctypes.Structure):
    _fields_ = _FIELDS


class Gmo(ctypes.Structure):

    """wbc_gmo_t: her momentum observer (`machine.senses.observer`), laid and its state - its hot block first, the
    struct on whole 64-byte lines (alignas(64)): the tail C pads it to (`made` lays it on a line)."""

    _fields_ = _FIELDS + [('tail', ctypes.c_char * (-ctypes.sizeof(_Laid) % 64))]


def made(**fields):
    """A Gmo on a 64-byte line, `fields` set: ctypes' own allocation keeps 16 bytes' alignment, not 64."""
    room = bytearray(ctypes.sizeof(Gmo) + 64)
    at = ctypes.addressof(ctypes.c_char.from_buffer(room))
    o = Gmo.from_buffer(room, -at % 64)
    for name, value in fields.items():
        setattr(o, name, value)
    return o


def typed(lib):
    """`lib`'s observer calls typed, its struct's size held against the mirror's."""
    lib.wbh_gmo_size.restype = _i
    if lib.wbh_gmo_size() != ctypes.sizeof(Gmo):
        raise ValueError('wbc_gmo_t is %d bytes, its mirror %d' % (lib.wbh_gmo_size(), ctypes.sizeof(Gmo)))
    said = (ctypes.c_longlong * len(OFFSETS))()
    lib.wbh_gmo_offsets(said)
    apart = [name for name, at in zip(OFFSETS, said) if getattr(Gmo, name).offset != at]
    if apart:
        raise ValueError('wbc_gmo_t lays %s elsewhere than its mirror' % ', '.join(apart))
    lib.wbh_gmo_felt.argtypes, lib.wbh_gmo_felt.restype = [ctypes.POINTER(Gmo), _vp, _vp, _vp, _d], None
    lib.wbh_gmo_step.argtypes, lib.wbh_gmo_step.restype = [ctypes.POINTER(Gmo), _vp, _vp, _vp, _d], None
