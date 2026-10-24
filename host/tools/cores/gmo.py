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
GMO_P, GMO_F, GMO_S, GMO_M = (_gmo_define(n) for n in ('WBC_GMO_POINTS', 'WBC_GMO_F', 'WBC_GMO_SHARES',
                                                       'WBC_GMO_MANIFOLDS'))
_LINKS = _define('WBC_LINKS')
BODY_DOUBLES = (2 * 12 * _LINKS + (N - 6) + 2 * 3 * _LINKS + N + 3 * 6 * _LINKS + 10 * _LINKS + 6 * _LINKS + _LINKS
                + 3 * _LINKS + N * N + N)


class Gmo(ctypes.Structure):

    """wbc_gmo_t: her momentum observer (`machine.observer`), laid and its state."""

    _fields_ = [('rounds', _i), ('alpha', _d), ('slip', _d), ('eps', _d), ('loss', _d * N), ('driven_at', _i * DRIVEN),
                ('driven_joint', _i * DRIVEN), ('link', _i * GMO_P), ('local', (_d * 3) * GMO_P), ('normal', (_d * 3) * GMO_P), ('sole', _i * GMO_P),
                ('col', _i * GMO_P), ('share_point', _i * GMO_S), ('share_manifold', _i * GMO_S),
                ('share_link', _i * GMO_S), ('share_axis', (_d * 3) * GMO_S), ('tau', _d * DRIVEN), ('acc', _d * 3),
                ('span', _d), ('primed', _d), ('u', _d * N), ('r', _d * N), ('f', _d * GMO_F), ('force', _d * GMO_M),
                ('body', _d * BODY_DOUBLES), ('a', (_d * GMO_F) * N), ('h', (_d * GMO_F) * GMO_F),
                ('k', (_d * GMO_F) * GMO_F)]


def typed(lib):
    """`lib`'s observer calls typed, its struct's size held against the mirror's."""
    lib.wbh_gmo_size.restype = _i
    if lib.wbh_gmo_size() != ctypes.sizeof(Gmo):
        raise ValueError('wbc_gmo_t is %d bytes, its mirror %d' % (lib.wbh_gmo_size(), ctypes.sizeof(Gmo)))
    lib.wbh_gmo_felt.argtypes, lib.wbh_gmo_felt.restype = [ctypes.POINTER(Gmo), _vp, _vp, _vp, _d], None
    lib.wbh_gmo_step.argtypes, lib.wbh_gmo_step.restype = [ctypes.POINTER(Gmo), _vp, _vp, _vp, _d], None
