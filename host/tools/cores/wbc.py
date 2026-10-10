"""wbc/ through ctypes: her poses, Jacobians, mass matrix and bias on the static model.

The chain the firmware runs, on the arrays `tools.cores.model` writes; one library a process.
"""
import ctypes
import functools
import os
import re

import numpy as np

from tools import REPO
from tools.cores.build import cached
from tools.cores.model import HEADER, rotation

SOURCES = [os.path.join(REPO, 'wbc', 'test', 'harness.c'),
           os.path.join(REPO, 'wbc', 'src', 'wbc_body.c'),
           os.path.join(REPO, 'wbc', 'src', 'wbc_stack.c'),
           os.path.join(REPO, 'wbc', 'src', 'wbc_model.c')]
INCLUDES = [os.path.join(REPO, 'wbc', 'inc')]
#: The firmware compiles the chain at -O2 (CMakeLists.txt).
FLAGS = ('-O2',)


def _define(name):
    with open(HEADER, encoding='utf-8') as f:
        found = re.search(r'#define %s (\d+)' % name, f.read())
    if not found:
        raise RuntimeError('%s is not defined in %s' % (name, HEADER))
    return int(found.group(1))


#: The model's sizes, as wbc_model.h defines them; the levels, as wbc.h does.
N, DRIVEN, LEVELS = _define('WBC_N'), _define('WBC_DRIVEN'), 4
_d, _i = ctypes.c_double, ctypes.c_int


class Ask(ctypes.Structure):

    """wbc_ask_t: what the loop is asked, R^k."""

    _fields_ = [('stance', _i * 2), ('com_acc', _d * 3), ('turn', (_d * 4) * 2),
                ('swing', _i * 2), ('swing_at', (_d * 3) * 2), ('swing_speed', (_d * 3) * 2),
                ('swing_acc', (_d * 3) * 2), ('swing_quat', (_d * 4) * 2), ('fold', _i * 2),
                ('fold_knee', _d * 2), ('fold_rate', _d * 2), ('fold_acc', _d * 2),
                ('posture', _d * DRIVEN), ('derate', _d * DRIVEN), ('wep', _i),
                ('kdot_given', _i), ('kdot', _d * 3)]


class Out(ctypes.Structure):

    """wbc_out_t: what the loop answers."""

    _fields_ = [('tau', _d * DRIVEN), ('udot', _d * N), ('wrench', (_d * 6) * 2),
                ('bears', _d * 2), ('alpha', _d * LEVELS), ('over', _i * DRIVEN),
                ('load', _d * DRIVEN)]


def ask_of(ask):
    """An `Ask` from `machine.wbc.step`'s dict: stance, com_acc, turns, swing, fold, posture,
    derate, wep, kdot."""
    a = Ask()
    for k in (0, 1):
        a.stance[k] = int(bool(ask['stance'][k]))
        a.turn[k][:] = [float(x) for x in ask['turns'][k]]
    a.com_acc[:] = [float(x) for x in ask['com_acc']]
    for k, (at, speed, acc, quat) in ask.get('swing', {}).items():
        a.swing[k] = 1
        a.swing_at[k][:] = [float(x) for x in at]
        a.swing_speed[k][:] = [float(x) for x in speed]
        a.swing_acc[k][:] = [float(x) for x in acc]
        a.swing_quat[k][:] = [float(x) for x in quat]
    for k, (knee, rate, acc) in ask.get('fold', {}).items():
        a.fold[k], a.fold_knee[k], a.fold_rate[k], a.fold_acc[k] = 1, knee, rate, acc
    a.posture[:] = [float(x) for x in ask['posture']]
    a.derate[:] = list(np.broadcast_to(np.asarray(ask.get('derate', 1.0), float), (DRIVEN,)))
    a.wep = int(bool(ask.get('wep', False)))
    if ask.get('kdot') is not None:
        a.kdot_given = 1
        a.kdot[:] = [float(x) for x in ask['kdot']]
    return a


def typed(lib):
    """`lib`'s harness calls typed; `lib` back."""
    d, i = ctypes.c_double, ctypes.c_int
    dp = ctypes.POINTER(d)
    lib.wbh_links.restype = i
    lib.wbh_n.restype = i
    lib.wbh_name.restype = ctypes.c_char_p
    lib.wbh_name.argtypes = [i]
    lib.wbh_parent.restype = i
    lib.wbh_parent.argtypes = [i]
    lib.wbh_pose.argtypes = [dp, dp, dp]
    lib.wbh_mass.argtypes = [dp]
    lib.wbh_bias.argtypes = [dp, dp]
    lib.wbh_point.argtypes = [i, dp, dp]
    lib.wbh_jacobian.argtypes = [i, dp, dp]
    lib.wbh_com.argtypes = [dp, dp]
    lib.wbh_drift.argtypes = [i, dp, dp]
    lib.wbh_momentum.argtypes = [dp, dp]
    lib.wbh_seconds.restype = d
    lib.wbh_seconds.argtypes = [dp, dp, dp, i, i, i]
    lib.wbh_stack.argtypes = [dp, dp, dp, ctypes.POINTER(Ask), ctypes.POINTER(Out)]
    lib.wbh_stack_seconds.restype = d
    lib.wbh_stack_seconds.argtypes = [dp, dp, dp, ctypes.POINTER(Ask), i]
    return lib


@functools.lru_cache(maxsize=None)
def library():
    """The core built once a process for its sources' bytes."""
    return typed(ctypes.CDLL(cached(SOURCES, INCLUDES, 'wbc', FLAGS)[0]))


def _arr(a):
    return np.ascontiguousarray(a, dtype=np.float64)


def _ptr(a):
    return a.ctypes.data_as(ctypes.POINTER(ctypes.c_double))


class Core:

    """The chain behind its harness: one body, posed, then asked."""

    def __init__(self, lib=None):
        self.lib = lib or library()
        self.links, self.n = int(self.lib.wbh_links()), int(self.lib.wbh_n())
        self.names = [self.lib.wbh_name(k).decode() for k in range(self.links)]
        self.parent = [int(self.lib.wbh_parent(k)) for k in range(self.links)]

    def pose(self, base, q):
        """Every link's frame in the world, (links, 12): R by rows, then p. `base` the pelvis's
        the same way, `q` the hinges' rad."""
        base, q, out = _arr(base), _arr(q), np.zeros((self.links, 12))
        self.lib.wbh_pose(_ptr(base), _ptr(q), _ptr(out))
        return out

    def mass(self):
        """The mass matrix at the pose, (n, n)."""
        out = np.zeros((self.n, self.n))
        self.lib.wbh_mass(_ptr(out))
        return out

    def bias(self, u):
        """C u + g at the pose, (n,); u the pelvis's twist (w, v) in its frame, then rad/s."""
        u, out = _arr(u), np.zeros(self.n)
        self.lib.wbh_bias(_ptr(u), _ptr(out))
        return out

    def point(self, link, r):
        """A point r of `link` in the world."""
        r, out = _arr(r), np.zeros(3)
        self.lib.wbh_point(link, _ptr(r), _ptr(out))
        return out

    def jacobian(self, link, r):
        """The point's Jacobian over u in the world, (6, n): rows w, then v."""
        r, out = _arr(r), np.zeros((6, self.n))
        self.lib.wbh_jacobian(link, _ptr(r), _ptr(out))
        return out

    def com(self):
        """Her centre of mass in the world and its Jacobian over u, ((3,), (3, n))."""
        com, j = np.zeros(3), np.zeros((3, self.n))
        self.lib.wbh_com(_ptr(com), _ptr(j))
        return com, j

    def drift(self, link, r):
        """The point's J-dot u in the world, (6,): rows w, then v; after `bias`."""
        r, out = _arr(r), np.zeros(6)
        self.lib.wbh_drift(link, _ptr(r), _ptr(out))
        return out

    def momentum(self, com):
        """Her angular momentum about `com`, world, (3,); after `bias`."""
        com, out = _arr(com), np.zeros(3)
        self.lib.wbh_momentum(_ptr(com), _ptr(out))
        return out

    def seconds(self, base, q, u, a, b, reps=1000):
        """Seconds a step on this host: the pose, M, the bias, links a's and b's Jacobians."""
        base, q, u = _arr(base), _arr(q), _arr(u)
        return float(self.lib.wbh_seconds(_ptr(base), _ptr(q), _ptr(u), a, b, reps))

    def stack(self, base, q, u, ask):
        """One tick of the loop for `ask` (`machine.wbc.step`'s dict): {tau, udot, wrench, bears,
        alpha, over, load}, udot in u's order."""
        base, q, u, out = _arr(base), _arr(q), _arr(u), Out()
        self.lib.wbh_stack(_ptr(base), _ptr(q), _ptr(u), ctypes.byref(ask_of(ask)), ctypes.byref(out))
        return {'tau': np.array(out.tau), 'udot': np.array(out.udot),
                'wrench': np.array([list(w) for w in out.wrench]), 'bears': list(out.bears),
                'alpha': list(out.alpha), 'over': np.array(out.over, bool),
                'load': np.array(out.load)}

    def stack_seconds(self, base, q, u, ask, reps=200):
        """Seconds a tick of the loop on this host."""
        base, q, u = _arr(base), _arr(q), _arr(u)
        return float(self.lib.wbh_stack_seconds(_ptr(base), _ptr(q), _ptr(u),
                                                ctypes.byref(ask_of(ask)), reps))


def state_of(b):
    """(base, q, u, T) of her `wbc.Body` as its MjData stands: the pelvis's frame, the hinges'
    rad, u with the pelvis's twist in its own frame; T taking u to MuJoCo's qvel."""
    qpos, qvel, n = b.d.qpos, b.d.qvel, b.n
    R = rotation(qpos[3:7])
    T = np.eye(n)
    T[:3, :3], T[:3, 3:6], T[3:6, :3], T[3:6, 3:6] = 0.0, R, np.eye(3), 0.0
    return np.r_[R.ravel(), qpos[:3]], qpos[7:7 + n - 6].copy(), np.linalg.solve(T, qvel[:n]), T


def stack_step(b, s, ask, core=None):
    """`machine.wbc.step`'s answer from the loop in C, on her `wbc.Body` as `sense` left it:
    {tau, qacc (MuJoCo's qvel order), bears, cop, over, slack, held, alpha, wrench}."""
    core = core or Core()
    base, q, u, T = state_of(b)
    out = core.stack(base, q, u, ask)
    dT_u = np.zeros(b.n)
    dT_u[:3] = T[:3, 3:6] @ np.cross(u[:3], u[3:6])
    cop = []
    for k in (0, 1):
        if ask['stance'][k]:
            m, f = out['wrench'][k][:3], out['wrench'][k][3:]
            cop.append(np.array([m[2] / f[1], -m[0] / f[1]]) if f[1] > 1e-6 else None)
    return {'tau': out['tau'], 'qacc': T @ out['udot'] + dT_u, 'bears': out['bears'], 'cop': cop,
            'over': out['over'], 'slack': [], 'held': (), 'alpha': out['alpha'],
            'wrench': out['wrench'], 'load': out['load']}
