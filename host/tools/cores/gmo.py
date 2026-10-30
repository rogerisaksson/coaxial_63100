"""wbc_gmo.c through ctypes: her momentum observer's struct mirrored, its calls typed."""
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
_LINKS = _define('WBC_LINKS')
BODY_DOUBLES = (2 * 12 * _LINKS + (N - 6) + 2 * 3 * _LINKS + N + 3 * 6 * _LINKS + 10 * _LINKS + 6 * _LINKS + _LINKS
                + 3 * _LINKS + N * N + N)


class Gmo(ctypes.Structure):

    """wbc_gmo_t: her momentum observer (`machine.senses.observer`), laid and its state."""

    _fields_ = [('alpha', _d), ('slip', _d), ('eps', _d), ('weight', _d), ('upright', _d), ('reach', _d), ('still', _d), ('slide', _d * 3), ('loss', _d * N), ('driven_at', _i * DRIVEN),
                ('driven_joint', _i * DRIVEN), ('link', _i * GMO_P), ('local', (_d * 3) * GMO_P),
                ('edge', ((_d * 3) * GMO_E) * GMO_P), ('edges', _i * GMO_P), ('col', _i * GMO_P), ('pushes', _i), ('gated', _i * GMO_P), ('world', _i * GMO_P), ('soled', _i * GMO_P),
                ('stop_lo', _d * (N - 6)), ('stop_hi', _d * (N - 6)), ('stopped', _i * (N - 6)), ('capsules', _i),
                ('capsule_link', _i * GMO_C), ('capsule_r', _d * GMO_C), ('capsule_a', (_d * 3) * GMO_C),
                ('capsule_b', (_d * 3) * GMO_C), ('pairs', _i), ('pair', (_i * 2) * GMO_PAIRS),
                ('share_point', _i * GMO_S), ('share_manifold', _i * GMO_S),
                ('share_link', _i * GMO_S), ('share_axis', (_d * 3) * GMO_S), ('tau', _d * DRIVEN), ('acc', _d * 3),
                ('span', _d), ('primed', _d), ('u', _d * N), ('r', _d * N), ('f', _d * GMO_F), ('force', _d * GMO_M),
                ('at', (_d * 3) * GMO_P), ('touched', _i * GMO_T), ('past', _i * (N - 6)), ('prior', _d * GMO_P), ('speed', _d * GMO_P), ('expected', _d * GMO_F), ('rounds', _i),
                ('body', _d * BODY_DOUBLES), ('a', (_d * GMO_F) * N), ('h', (_d * GMO_F) * GMO_F),
                ('k', (_d * GMO_F) * GMO_F)]


def typed(lib):
    """`lib`'s observer calls typed, its struct's size held against the mirror's."""
    lib.wbh_gmo_size.restype = _i
    if lib.wbh_gmo_size() != ctypes.sizeof(Gmo):
        raise ValueError('wbc_gmo_t is %d bytes, its mirror %d' % (lib.wbh_gmo_size(), ctypes.sizeof(Gmo)))
    lib.wbh_gmo_felt.argtypes, lib.wbh_gmo_felt.restype = [ctypes.POINTER(Gmo), _vp, _vp, _vp, _d], None
    lib.wbh_gmo_step.argtypes, lib.wbh_gmo_step.restype = [ctypes.POINTER(Gmo), _vp, _vp, _vp, _d], None
