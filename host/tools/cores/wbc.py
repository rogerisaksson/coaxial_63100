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
from machine import ortho
from machine.wbc import MU
from tools.cores.model import HEADER, rotation

SOURCES = ([os.path.join(REPO, 'wbc', 'test', 'harness.c')]
           + [os.path.join(REPO, 'wbc', 'src', 'wbc_%s.c' % n) for n in (
               'body stack solve wrench cone shrunk model pass imp stride swing floor surety touch twist wait brace blas law step rounds sil').split()]
           + [os.path.join(REPO, 'modbus', 'src', 'modbus_%s.c' % n) for n in ('crc rtu slave').split()])
INCLUDES = [os.path.join(REPO, 'wbc', 'inc'), os.path.join(REPO, 'modbus', 'inc')]
#: The firmware compiles the chain at -O2 (CMakeLists.txt); sin and cos called apart, as numpy calls
#: them: gcc's sincos for the pair is glibc's own path at 0.855 < |x| < 2.43, its sin's last bit not sin's.
FLAGS = ('-O2', '-fno-builtin-sin', '-fno-builtin-cos')


def _define(name):
    with open(HEADER, encoding='utf-8') as f:
        found = re.search(r'#define %s (\d+)' % name, f.read())
    if not found:
        raise RuntimeError('%s is not defined in %s' % (name, HEADER))
    return int(found.group(1))


#: The model's sizes, as wbc_model.h defines them.
N, DRIVEN = _define('WBC_N'), _define('WBC_DRIVEN')
_d, _i = ctypes.c_double, ctypes.c_int


class Ask(ctypes.Structure):

    """wbc_ask_t: what the loop is asked, R^k."""

    _fields_ = [('stance', _i * 2), ('com_acc', _d * 3), ('turn', (_d * 4) * 2),
                ('swing', _i * 2), ('swing_at', (_d * 3) * 2), ('swing_speed', (_d * 3) * 2),
                ('swing_acc', (_d * 3) * 2), ('swing_quat', (_d * 4) * 2), ('fold', _i * 2),
                ('fold_knee', _d * 2), ('fold_rate', _d * 2), ('fold_acc', _d * 2), ('knee_rate', _d * 2),
                ('posture', _d * DRIVEN), ('derate', _d * DRIVEN), ('wep', _i),
                ('kdot_given', _i), ('kdot', _d * 3), ('settle', (_d * 6) * 2), ('edge', _d * 2),
                ('strides', _d), ('mu', _d * 2)]


class Out(ctypes.Structure):

    """wbc_out_t: what the loop answers."""

    _fields_ = [('tau', _d * DRIVEN), ('udot', _d * N), ('wrench', (_d * 6) * 2),
                ('bears', _d * 2), ('held', _i), ('passes', _i), ('guarded', _i * 5),
                ('ids', _i * 32), ('alpha', _d * 3), ('residual', _d), ('over', _i * DRIVEN),
                ('load', _d * DRIVEN), ('jeff', _d * DRIVEN), ('force', (_d * 12) * 2),
                ('stuck', _i), ('clipped', _d * 2)]


class OrthoRow(ctypes.Structure):

    """ortho_row_t: a row on the wire (machine.ortho.ROW)."""

    _fields_ = [('id', ctypes.c_uint32), ('at', ctypes.c_uint32), ('blend', ctypes.c_uint32),
                ('flags', ctypes.c_uint32), ('p', ctypes.c_float * ortho.P)]


_vp = ctypes.c_void_p


class Drives(ctypes.Structure):

    """wbc_drives_t: her law's drive-side step, pointing at her law's arrays (`machine.law.laid`)."""

    _fields_ = [('n', _i), ('q_at', _i), ('qd_at', _i), ('qm_at', _i), ('knee', _i * 2), ('cervical', _i * 2),
                ('knee_nm', _d), ('knee_soft', _d), ('leak_s', _d), ('wep_hold_s', _d),
                ('joints', _vp), ('act', _vp), ('y', _vp), ('qacc', _vp), ('tau', _vp), ('over', _vp),
                ('jeff', _vp), ('clamp', _vp), ('wep', _vp), ('wep_lim', _vp), ('clamp_j', _vp), ('q_lo', _vp),
                ('q_hi', _vp), ('eso', _vp), ('ref_q', _vp), ('ref_v', _vp), ('hold', _vp), ('sb', _vp),
                ('feed', _vp), ('limit', _vp)]


class World(ctypes.Structure):

    """wbc_world_t: her world's pass, pointing at the world's arrays and MuJoCo's (`physics.World`)."""

    _fields_ = [('n', _i), ('npairs', _i), ('pelvis', _i), ('wound', _d),
                ('qadr', _vp), ('vadr', _vp), ('pairs', _vp), ('qpos', _vp), ('qvel', _vp), ('qacc', _vp),
                ('xmat', _vp), ('ctrl', _vp), ('target', _vp), ('rate', _vp), ('feed', _vp), ('gains', _vp),
                ('limit', _vp), ('kt', _vp), ('shorted', _vp), ('cut', _vp), ('delta', _vp), ('zero', _vp),
                ('armature', _vp), ('motor', _vp), ('geared', _vp),
                ('effort', _d), ('work', _d), ('brake', _d), ('loss', _vp), ('blas', _vp), ('heat', _d)]


class Contacts(ctypes.Structure):

    """wbc_contacts_t: her soles' and skin's contacts read, pointing at the edge's bins (`machine.edge.laid`)."""

    _fields_ = [('nbody', _i), ('nskin', _i), ('ninsoles', _i), ('nfeet', _i), ('stride', _i), ('off_geom', _i),
                ('off_frame', _i), ('off_dim', _i), ('off_efc', _i), ('middle_m', _d),
                ('gb', _vp), ('hers', _vp), ('sole_side', _vp), ('insole', _vp), ('foot', _vp), ('skin', _vp),
                ('xmat', _vp), ('xpos', _vp), ('loads', _vp), ('touch', _vp), ('pressed', _vp), ('at', _vp),
                ('cells', _vp), ('felt_n', _vp), ('felt_at', _vp)]


class Heat(ctypes.Structure):

    """wbc_heat_t: her drives' heat, pointing at `machine.heat.Heat`'s arrays."""

    _fields_ = [('n', _i), ('at', _d), ('haste', _d), ('ambient_c', _d), ('rds_ohm', _d), ('rds_alpha', _d),
                ('rds_beta', _d), ('shunt_ohm', _d), ('switching_w', _d), ('into_k_w', _d), ('rth_jc', _d),
                ('lookahead_s', _d), ('throttle_at', _d), ('recover_per_s', _d), ('trip_margin', _d),
                ('trip_recover_per_s', _d), ('tj_max_c', _d),
                ('kt', _vp), ('scale', _vp), ('ratio_k', _vp), ('cloth_air', _vp), ('r', _vp), ('g_ls', _vp),
                ('g_ws', _vp), ('lam_k_w', _vp), ('win_k_w', _vp), ('capacity', _vp), ('ceiling', _vp),
                ('house_w', _vp), ('active', _vp), ('t', _vp), ('sq', _vp), ('krpm', _vp), ('limb_rad_s', _vp),
                ('derate', _vp), ('spent', _vp), ('die', _vp), ('cap', _vp), ('cap_at', _vp), ('ticks', _vp),
                ('worst', _vp), ('trips', _vp), ('gates', _vp), ('shorted', _vp)]


def _data_at():
    """Where in an ndarray's object its data pointer is, bytes: numpy's PyArrayObject, after the head."""
    probes = [np.empty(k + 1) for k in range(3)]
    for at in range(8, 80, 8):
        if all(ctypes.c_void_p.from_address(id(a) + at).value == a.ctypes.data for a in probes):
            return at
    raise RuntimeError("numpy's array head not found")


#: Where an ndarray's data pointer lies in its object (`address`).
_AT = _data_at()
_DTYPES = {np.float64: np.dtype(np.float64), np.int32: np.dtype(np.int32), np.bool_: np.dtype(np.bool_)}


def plain(a):
    """`a`, a ctypes field's numpy view, in numpy's own dtype: as_array's is equal but another
    instance, and numpy copies an array of it where one of its own is asked."""
    return a.view(np.dtype(a.dtype.char))


def address(a):
    """The address of an ndarray's first element, read off its object."""
    return ctypes.c_void_p.from_address(id(a) + _AT).value or 0


def pointer(a, dtype: type = np.float64):
    """`a`'s address for a struct's pointer, `a` contiguous and of `dtype` - a copy would be
    pointed at and written in vain."""
    want = _DTYPES.get(dtype) or np.dtype(dtype)
    if not a.flags.c_contiguous or (a.dtype is not want and a.dtype != want):
        raise ValueError('a laid array is contiguous %s, not %s %s' % (want.name, a.dtype.name,
                                                                        'contiguous' if a.flags.c_contiguous
                                                                        else 'strided'))
    return address(a)


def ask_of(ask, a=None):
    """An `Ask` from `machine.wbc.step`'s dict: stance, com_acc, turns - none asked, all zeros
    -, swing, fold, posture, derate, wep, kdot; `machine.wrench.step`'s settle, edge, strides and mu."""
    if a is None:
        a = Ask()
    else:
        ctypes.memset(ctypes.byref(a), 0, ctypes.sizeof(a))
    st = ask['stance']
    a.stance[0], a.stance[1] = int(bool(st[0])), int(bool(st[1]))
    for k, turn in enumerate(ask['turns']):
        a.turn[k][:] = turn
    a.com_acc[:] = ask['com_acc']
    swing = ask.get('swing')
    if swing:
        for k, (at, speed, acc, quat) in swing.items():
            a.swing[k] = 1
            a.swing_at[k][:] = at
            a.swing_speed[k][:] = speed
            a.swing_acc[k][:] = acc
            a.swing_quat[k][:] = quat
    fold = ask.get('fold')
    if fold:
        for k, (knee, rate, acc) in fold.items():
            a.fold[k], a.fold_knee[k], a.fold_rate[k], a.fold_acc[k] = 1, knee, rate, acc
    a.knee_rate[:] = ask.get('knee_rate', (0.0, 0.0))
    np.ctypeslib.as_array(a.posture)[:] = ask['posture']
    np.ctypeslib.as_array(a.derate)[:] = ask.get('derate', 1.0)
    a.wep = int(bool(ask.get('wep', False)))
    kdot = ask.get('kdot')
    if kdot is not None:
        a.kdot_given = 1
        a.kdot[:] = kdot
    settle = ask.get('settle')
    if settle is not None:
        for k, row in enumerate(settle):
            a.settle[k][:] = row
    edge = ask.get('edge')
    if edge is not None:
        a.edge[0], a.edge[1] = float(edge[0]), float(edge[1])
    a.strides = float(ask.get('strides', 0.0))
    mu = ask.get('mu')
    if mu is None:
        a.mu[0] = a.mu[1] = MU
    else:
        a.mu[0], a.mu[1] = float(mu[0]), float(mu[1])
    return a


def typed(lib):
    """`lib`'s harness calls typed; `lib` back."""
    d, i = ctypes.c_double, ctypes.c_int
    dp = ctypes.POINTER(d)
    u32p = ctypes.POINTER(ctypes.c_uint32)
    fp = ctypes.POINTER(ctypes.c_float)
    lib.wbh_world_size.restype = i
    if lib.wbh_world_size() != ctypes.sizeof(World):
        raise ValueError('wbc_world_t is %d bytes, its mirror %d' % (lib.wbh_world_size(), ctypes.sizeof(World)))
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
    lib.wbh_stack_init.argtypes = []
    lib.wbh_stack.argtypes = [dp, dp, dp, ctypes.POINTER(Ask), ctypes.POINTER(Out)]
    lib.wbh_stack_seconds.restype = d
    lib.wbh_stack_seconds.argtypes = [dp, dp, dp, ctypes.POINTER(Ask), i]
    lib.wbh_wrench_init.argtypes = []
    lib.wbh_wrench.argtypes = [dp, dp, dp, ctypes.POINTER(Ask), ctypes.POINTER(Out)]
    lib.wbh_wrench_seconds.restype = d
    lib.wbh_wrench_seconds.argtypes = [dp, dp, dp, ctypes.POINTER(Ask), i]
    lib.wbh_ortho_row_size.restype = i
    lib.wbh_ortho_row_size.argtypes = []
    lib.wbh_ortho_row.argtypes = [ctypes.c_void_p, u32p, fp]
    lib.wbh_pass_drives.argtypes = [ctypes.POINTER(Drives), d, d, d, d, i, i]
    lib.wbh_pass_ctrl.argtypes = [ctypes.POINTER(World), d, _vp, _vp]
    lib.wbh_pass_motor.argtypes = [ctypes.POINTER(World), d]
    lib.wbh_pass_metered.argtypes = [ctypes.POINTER(World), _vp, d]
    lib.wbh_pass_felt.argtypes = [ctypes.POINTER(World), _vp, i, i, i, i, i, i]
    lib.wbh_pass_contacts.argtypes = [ctypes.POINTER(Contacts), i, _vp, _vp]
    lib.wbh_pass_sensed.argtypes = [ctypes.POINTER(Contacts), _vp, i, i, i, i, i, i]
    lib.wbh_pass_heat_load.argtypes = [ctypes.POINTER(Heat), _vp, _vp]
    lib.wbh_pass_heat.argtypes = [ctypes.POINTER(Heat), d, _vp, _vp, i]
    lib.wbh_pass_heat_peak.argtypes = [ctypes.POINTER(Heat), _vp]
    lib.wbh_pass_drawn.argtypes = [ctypes.POINTER(World), _vp, _vp, _vp, _vp, _vp, _vp, d, d]
    lib.wbh_pass_drawn.restype = d
    lib.wbh_pass_hem.argtypes = [i, d] + [_vp] * 9
    lib.wbh_pass_paired.argtypes, lib.wbh_pass_paired.restype = [_vp, ctypes.c_ssize_t, i, _vp, _vp], i
    lib.wbh_pass_averaged.argtypes = [ctypes.POINTER(World), d, _vp, _vp]
    lib.wbh_pass_hem.restype = None
    lib.wbh_sil_init.argtypes = [ctypes.POINTER(World), i, _vp, _vp]
    lib.wbh_sil_transceive.argtypes = [ctypes.POINTER(World), i, ctypes.c_char_p, i]
    lib.wbh_sil_transceive.restype = i
    lib.wbh_sil_drive.argtypes = [ctypes.POINTER(World), i, _vp, _vp, _vp, _vp, _vp]
    lib.wbh_sil_stats.argtypes = [i, _vp, _vp, _vp, _vp]
    for name in ('wbh_pass_drives', 'wbh_pass_ctrl', 'wbh_pass_motor', 'wbh_pass_metered', 'wbh_pass_averaged', 'wbh_pass_felt',
                 'wbh_pass_contacts', 'wbh_pass_sensed', 'wbh_pass_heat_load', 'wbh_pass_heat',
                 'wbh_pass_heat_peak', 'wbh_sil_init', 'wbh_sil_drive', 'wbh_sil_stats'):
        getattr(lib, name).restype = None
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
        self.lib.wbh_stack_init()
        self.lib.wbh_wrench_init()
        self.links, self.n = int(self.lib.wbh_links()), int(self.lib.wbh_n())
        self.names = [self.lib.wbh_name(k).decode() for k in range(self.links)]
        self.parent = [int(self.lib.wbh_parent(k)) for k in range(self.links)]
        self._ask, self._out = Ask(), Out()
        #: The step's inputs laid once (`wrench_laid`): the pelvis's frame, the hinges, u.
        self._base, self._q, self._u = np.empty(12), np.empty(self.n - 6), np.empty(self.n)
        self._pb, self._pq, self._pu = _ptr(self._base), _ptr(self._q), _ptr(self._u)

    def reset(self):
        """The loop's memory cleared: the loads WEP is granted on, the warm set."""
        self.lib.wbh_stack_init()

    def ortho_row_size(self):
        """sizeof(ortho_row_t), bytes."""
        return int(self.lib.wbh_ortho_row_size())

    def ortho_row(self, wire):
        """(id, at, blend, flags, p) unpacked by `ortho_row_t` in C from `wire`."""
        head = np.zeros(4, dtype=np.uint32)
        p = np.zeros(ortho.P, dtype=np.float32)
        self.lib.wbh_ortho_row(
            ctypes.c_char_p(bytes(wire)),
            head.ctypes.data_as(ctypes.POINTER(ctypes.c_uint32)),
            p.ctypes.data_as(ctypes.POINTER(ctypes.c_float)))
        return int(head[0]), int(head[1]), int(head[2]), int(head[3]), p

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
        """Her angular momentum about `com`, world, (3,); after `mass` and `bias`."""
        com, out = _arr(com), np.zeros(3)
        self.lib.wbh_momentum(_ptr(com), _ptr(out))
        return out

    def seconds(self, base, q, u, a, b, reps=1000):
        """Seconds a step on this host: the pose, M, the bias, links a's and b's Jacobians."""
        base, q, u = _arr(base), _arr(q), _arr(u)
        return float(self.lib.wbh_seconds(_ptr(base), _ptr(q), _ptr(u), a, b, reps))

    def stack(self, base, q, u, ask):
        """One tick of the loop for `ask` (`machine.wbc.step`'s dict): {tau, udot, wrench, bears,
        held, passes, over, load, jeff}, udot in u's order."""
        base, q, u, out = _arr(base), _arr(q), _arr(u), self._out
        self.lib.wbh_stack(_ptr(base), _ptr(q), _ptr(u), ctypes.byref(ask_of(ask, self._ask)), ctypes.byref(out))
        return {'tau': np.ctypeslib.as_array(out.tau).copy(),
                'udot': np.ctypeslib.as_array(out.udot).copy(),
                'wrench': np.ctypeslib.as_array(out.wrench).copy(),
                'bears': [float(out.bears[0]), float(out.bears[1])],
                'held': int(out.held), 'passes': int(out.passes), 'guarded': list(out.guarded),
                'ids': [i for i in out.ids if i >= 0], 'residual': float(out.residual),
                'alpha': list(out.alpha),
                'over': np.ctypeslib.as_array(out.over).astype(bool),
                'load': np.ctypeslib.as_array(out.load).copy(),
                'jeff': np.ctypeslib.as_array(out.jeff).copy(),
                'force': np.ctypeslib.as_array(out.force).reshape(2, 4, 3).copy(),
                'stuck': int(out.stuck), 'clipped': list(out.clipped)}

    def wrench(self, base, q, u, ask):
        """One tick of the step without an active set (wbc_wrench.c) for `ask`
        (`machine.wrench.step`'s dict): {tau, udot, wrench, bears, over, load, scale}."""
        self._base[:], self._q[:], self._u[:] = base, q, u
        return self.wrench_laid(ask)

    def wrench_laid(self, ask):
        """The tick on `_base`, `_q` and `_u` as the caller filled them."""
        out = self._out
        self.lib.wbh_wrench(self._pb, self._pq, self._pu, ctypes.byref(ask_of(ask, self._ask)), ctypes.byref(out))
        return {'tau': np.ctypeslib.as_array(out.tau).copy(),
                'udot': np.ctypeslib.as_array(out.udot).copy(),
                'wrench': np.ctypeslib.as_array(out.wrench).copy(),
                'bears': [float(out.bears[0]), float(out.bears[1])],
                'over': np.ctypeslib.as_array(out.over).astype(bool),
                'load': np.ctypeslib.as_array(out.load).copy(),
                'scale': float(out.alpha[0])}

    def wrench_seconds(self, base, q, u, ask, reps=2000):
        """Seconds a tick of it on this host."""
        base, q, u = _arr(base), _arr(q), _arr(u)
        return float(self.lib.wbh_wrench_seconds(_ptr(base), _ptr(q), _ptr(u),
                                                 ctypes.byref(ask_of(ask, self._ask)), reps))

    def stack_seconds(self, base, q, u, ask, reps=200):
        """Seconds a tick of the loop on this host."""
        base, q, u = _arr(base), _arr(q), _arr(u)
        return float(self.lib.wbh_stack_seconds(_ptr(base), _ptr(q), _ptr(u),
                                                ctypes.byref(ask_of(ask, self._ask)), reps))


def state_of(b):
    """(base, q, u, T) of her `wbc.Body` as its MjData stands: the pelvis's frame, the hinges'
    rad, u with the pelvis's twist in its own frame; T taking u to MuJoCo's qvel."""
    qpos, qvel, n = b.d.qpos, b.d.qvel, b.n
    R = rotation(qpos[3:7])
    T = np.eye(n)
    T[:3, :3], T[:3, 3:6], T[3:6, :3], T[3:6, 3:6] = 0.0, R, np.eye(3), 0.0
    u, base = np.empty(n), np.empty(12)
    u[:3], u[3:6], u[6:] = qvel[3:6], R.T @ qvel[:3], qvel[6:n]
    base[:9], base[9:] = R.ravel(), qpos[:3]
    return base, qpos[7:7 + n - 6].copy(), u, T


@functools.lru_cache(maxsize=None)
def shared():
    """The one `Core` a process shares: a `Core()` clears the loop's memory - the loads WEP is
    granted on, the warm set - so a core a tick had neither."""
    return Core()


def stack_step(b, s, ask, core=None):
    """`machine.wbc.step`'s answer from the loop in C, on her `wbc.Body` as `sense` left it:
    {tau, qacc (MuJoCo's qvel order), bears, cop, over, slack, held, passes, wrench, load,
    jeff}; `held` the rows held at a bound this tick, `passes` the solves after the first."""
    core = core or shared()
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
            'over': out['over'], 'slack': [], 'held': out['held'], 'passes': out['passes'],
            'guarded': out['guarded'], 'ids': out['ids'], 'residual': out['residual'],
            'alpha': out['alpha'], 'force': out['force'], 'stuck': out['stuck'],
            'clipped': out['clipped'],
            'wrench': out['wrench'], 'load': out['load']}


def wrench_step(b, s, ask, core=None):
    """`machine.wrench.step`'s answer from wbc_wrench.c, on her `wbc.Body` as `sense` left it:
    {tau, qacc (MuJoCo's qvel order), bears, cop, over, slack, held, scale, borne}."""
    core = core or shared()
    qpos, qvel, n = b.d.qpos, b.d.qvel, b.n
    R = rotation(qpos[3:7])
    base, u = core._base, core._u
    base[:9], base[9:] = R.ravel(), qpos[:3]
    u[:3], u[3:6], u[6:] = qvel[3:6], R.T @ qvel[:3], qvel[6:n]
    core._q[:] = qpos[7:7 + n - 6]
    out = core.wrench_laid(ask)
    udot = out['udot']
    qacc = udot.copy()
    w0, w1, w2, v0, v1, v2 = u[:6].tolist()
    qacc[:3] = R @ (udot[3:6] + np.array([w1 * v2 - w2 * v1, w2 * v0 - w0 * v2, w0 * v1 - w1 * v0]))
    qacc[3:6] = udot[:3]
    cop = []
    for k in (0, 1):
        if ask['stance'][k]:
            ml, fy = s['feet'][k]['R'].T @ out['wrench'][k][:3], float(out['wrench'][k][4])
            cop.append(np.array([ml[2], -ml[0]]) / fy if fy > 1e-6 else None)
    b.load = out['load']
    return {'tau': out['tau'], 'qacc': qacc, 'bears': out['bears'], 'cop': cop,
            'over': out['over'], 'slack': [], 'held': (), 'scale': out['scale'],
            'borne': np.zeros((0, 3))}

