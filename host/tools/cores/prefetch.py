"""Tiered L1, L2 and L3 cache prefetch and working-set loader (`prefetch.c`) for cores and engrams.

Stages the robot's real-time control hierarchy across hardware caches in bottom-up order (`L3 -> L2 -> L1`)
so inner-loop lines land last in L1 without self-eviction:
  - L1 (`PREFETCHT0` / `PRFM PLDL1KEEP`): 1 kHz beat, ESO/drive tick, active move/engram table, and sense/latent vectors.
  - L2 (`PREFETCHT1` / `PRFM PLDL2KEEP`): law-pass `bodied` microkernels, active hot-swapped C cores, and solver header.
  - L3 (`PREFETCHT2` / `PRFM PLDL3KEEP` + OS working-set pin): warm engram bank, shaped hulls, solver tail, and `.text`.
"""
from collections import namedtuple
import ctypes
import functools
import os
import time

import numpy as np

from tools.cores import build

SOURCE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'prefetch.c')
BUDGET_SHARE = 0.75
CODE_WINDOW_B = 4096
OPENBLAS_CAP_B = 2 * 1024 * 1024
STEP_WORK_L2_B = 96 * 1024
GMO_LAW_L2_B = 64 * 1024
PART_WORK_L2_B = 64 * 1024

CacheInfo = namedtuple('CacheInfo', ('line', 'l1d', 'l1i', 'l2', 'l3'))
_PINNED_TEXT: dict[int, tuple[int, int]] = {}
_KEEP_LIBS: list = []


@functools.lru_cache(maxsize=1)
def _lib():
    """Compile and load `prefetch.c` once per process with native instruction-set flags."""
    path, _warn = build.cached([SOURCE], [], 'prefetch', ('-O3',) + build.native())
    lib = ctypes.CDLL(path)
    u64p = ctypes.POINTER(ctypes.c_uint64)
    i32p = ctypes.POINTER(ctypes.c_int32)
    lib.pf_caches.argtypes, lib.pf_caches.restype = [u64p], None
    lib.pf_pin_code.argtypes, lib.pf_pin_code.restype = [ctypes.c_uint64, u64p], ctypes.c_int
    for fn in (lib.pf_prefetch_all, lib.pf_prefetch_hot, lib.pf_prefetch_l1):
        fn.argtypes = [u64p, u64p, i32p, ctypes.c_uint64]
        fn.restype = ctypes.c_uint64
    return lib


@functools.lru_cache(maxsize=1)
def caches() -> CacheInfo:
    """Return live host CPU cache geometry `CacheInfo(line, l1d, l1i, l2, l3)` in bytes."""
    out = (ctypes.c_uint64 * 5)()
    _lib().pf_caches(out)
    return CacheInfo(*(int(v) for v in out))


def _pin_symbol(sym_addr: int, cap_b: int = 0) -> tuple[int, int]:
    """Pin the shared library containing `sym_addr` in OS RAM once and return its `.text` `(addr, size)`."""
    if not sym_addr:
        return 0, 0
    for base, size in _PINNED_TEXT.values():
        if base <= sym_addr < base + size:
            return base, (min(size, cap_b) if cap_b else size)
    span = (ctypes.c_uint64 * 2)()
    _lib().pf_pin_code(ctypes.c_uint64(sym_addr), span)
    base, size = int(span[0]), int(span[1])
    if base and size:
        _PINNED_TEXT[base] = (base, size)
        return base, (min(size, cap_b) if cap_b else size)
    return 0, 0


def _fn_addr(cfn) -> int:
    """Extract raw integer entrypoint address from a `ctypes` function pointer, or 0."""
    if cfn is None:
        return 0
    try:
        return int(ctypes.cast(cfn, ctypes.c_void_p).value or 0)
    except (TypeError, ctypes.ArgumentError):
        return 0


def _arr_span(arr) -> tuple[int, int]:
    """Return `(data_addr, nbytes)` for a contiguous NumPy array, else `(0, 0)`."""
    if isinstance(arr, np.ndarray) and arr.size > 0 and arr.flags.c_contiguous:
        return int(arr.ctypes.data), int(arr.nbytes)
    return 0, 0


def _struct_span(obj) -> tuple[int, int]:
    """Return `(addressof, sizeof)` for a `ctypes.Structure` or `ctypes.Array`, else `(0, 0)`."""
    if isinstance(obj, (ctypes.Structure, ctypes.Array)):
        return int(ctypes.addressof(obj)), int(ctypes.sizeof(obj))
    return 0, 0


def _probe_for_kept(g, hook: str):
    """Build lightweight live-shape arguments for `kept.taken` without `deepcopy` or execution."""
    from machine.law import modal
    from machine.senses import edge
    body, world = g['body'], g['world']
    dt = 0.001 * float(g.get('every', 1))
    sits = g['situations']
    table = {
        'estimated': lambda: (g['est'], body, g['sense'], dt),
        'clear': lambda: (body, world.data.qpos[:body.m.nq]),
        'weighed': lambda: (g['observer'], g['told'], float(g['fr'][modal.WEIGHT]), dt),
        'outside': lambda: (g['grip'], sits['grips']),
        'sloped': lambda: (g['grip'], sits['grips']),
        'readings': lambda: (g['y'], g['fr'], g['a'], g['grip'],
                             g['sense'][edge.TOUCH].reshape(-1, 3), float(g['fr'][modal.WEIGHT])),
        'read': lambda: (sits, g['into']),
        'modes': lambda: (g['y'], g['fr'], g['out']),
        'pitched': lambda: (g['y'], g['fr']),
        'bounds': lambda: (g['s']['v'][body.act], float(g['sense'][edge.LINK][0])),
        'gripped': lambda: (g['out'], float(g.get('held', 0.8)), g['a'][modal.STANCE],
                            sits['caps'], g['into'], sits['floors'], float(sits['period']), g['eased'], dt),
        'sured': lambda: (g['a'], g['posture'], np.zeros_like(g['posture']), g['brace'], 0.8, 0.2, 0.0, 0.0),
        'released': lambda: (g['release'], g['weighed_w'], g['a'], g['distal'], g['distal_side']),
        'poses': lambda: (g['a'], g['y'], g['fr'], g['posture'], g['bend'], g['arms'],
                          g['reaches'], g['folds'], body.knees, g['weighed_w'], 0.0),
        'modal_sensed': lambda: (g['s'], list(g['loads']), g['y']),
    }
    fn = table.get(hook)
    return fn() if fn is not None else None


def _eager_bind_kept(g) -> int:
    """Bind already-cached `kept` libraries onto unbound `bodied.Compiled` hooks before tick 0."""
    from tools.cores import bodied, kept
    kept.settle()
    bound_n = 0
    for hook, part in list(g.items()):
        if not isinstance(part, bodied.Compiled) or part.c is not None or part.looked or not kept.ready(part):
            continue
        probe = _probe_for_kept(g, hook)
        if probe is None:
            continue
        got = kept.taken(part, probe, bodied.Bound, waited=True)
        if isinstance(got, tuple):
            prog, info, specs, c = got
            c.seen[:] = [None] * len(c.seen)
            part.made = (prog, info, None, specs)
            part.c = c
            part.looked = True
            bound_n += 1
    return bound_n


def _collect_l1(g, add_span):
    """Collect 1 kHz inner-loop structs, active move/engram tables, and vectors into L1."""
    beat = g.get('beat')
    if isinstance(beat, tuple) and len(beat) >= 4:
        add_span(1, 0, *_struct_span(beat[3]))
    drives = g.get('drives')
    if isinstance(drives, tuple) and len(drives) >= 2:
        add_span(1, 0, *_struct_span(drives[0]))
        kept = drives[1]
        for k in ('joints', 'act', 'clamp_j', 'sea'):
            add_span(1, 0, *_arr_span(kept.get(k)))
        for k in ('sb', 'feed', 'limit', 'qacc', 'tau', 'jeff', 'over', 'release'):
            add_span(1, 1, *_arr_span(kept.get(k)))
    for k in ('rng', 'wep_lim', 'q_lo', 'q_hi'):
        add_span(1, 0, *_arr_span(g.get(k)))
    world = g.get('world')
    if world is not None:
        add_span(1, 1, *_struct_span(getattr(world, 'passing', None)))
        add_span(1, 1, *_arr_span(getattr(world, 'motor', None)))
        link = getattr(world, 'link', None)
        if isinstance(link, dict):
            add_span(1, 1, *_arr_span(link.get('v')))
    tick = g.get('tick')
    if isinstance(tick, tuple) and len(tick) >= 4:
        add_span(1, 1, *_struct_span(tick[2]))
        add_span(1, 1, *_arr_span(tick[3]))
    obs = (g.get('observer') or {}).get('c')
    if isinstance(obs, ctypes.Structure):
        base = ctypes.addressof(obs)
        cls = type(obs)
        if hasattr(cls, 'box_k') and hasattr(cls, 'link'):
            b0, b1 = getattr(cls, 'box_k').offset, getattr(cls, 'link').offset
            add_span(1, 1, base + b0, max(0, b1 - b0))
        if hasattr(cls, 'tau') and hasattr(cls, 'u'):
            t0, t1 = getattr(cls, 'tau').offset, getattr(cls, 'u').offset
            add_span(1, 1, base + t0, max(0, t1 - t0))
    inekf = g.get('inekf') or {}
    if isinstance(inekf, dict):
        add_span(1, 1, *_arr_span(inekf.get('imu')))
        add_span(1, 1, *_arr_span(inekf.get('span')))
    cmd = g.get('cmd') or {}
    for arr in cmd.values():
        add_span(1, 1, *_arr_span(arr))
    bus = g.get('bus')
    if isinstance(bus, tuple) and len(bus) >= 2:
        add_span(1, 1, *_struct_span(bus[1]))
    for k in ('sense', 'ref_q', 'ref_v', 'eso', 'hold', 'st', 'out', 'a', 'y', 'fr'):
        add_span(1, 1, *_arr_span(g.get(k)))
    played = g.get('played')
    if isinstance(played, dict):
        mv = played.get('move')
        if mv is not None and hasattr(mv, '_laid'):
            laid_mv = mv._laid()
            if isinstance(laid_mv, tuple) and len(laid_mv) >= 2:
                add_span(1, 0, *_struct_span(laid_mv[1]))
        for k in ('kp', 'kd', 'joints'):
            add_span(1, 0, *_arr_span(played.get(k)))
        for k in ('state', 'cmd', 'pose', 'target', 'feed', 'limit'):
            add_span(1, 1, *_arr_span(played.get(k)))


def _collect_l2_l3(g, add_span, code_syms: list[tuple[int, int]]):
    """Collect law-pass solvers, `bodied`/hot-swap C cores, and warm `bank` engrams into L2 and L3."""
    from tools.cores import bodied
    solves = g.get('solves') or ()
    active_idx = int(bool(g.get('chained', 0)))
    for idx, fn in enumerate(solves):
        args = getattr(fn, 'args', ())
        if not args or not isinstance(args[0], ctypes.Structure):
            continue
        k = args[0]
        Step = type(k)
        base, total = ctypes.addressof(k), ctypes.sizeof(k)
        w_off = getattr(Step, 'work').offset
        w_len = ctypes.sizeof(k.work)
        tail_off = w_off + w_len
        tier_hdr = 2 if idx == active_idx else 3
        add_span(tier_hdr, 1, base, w_off)
        add_span(tier_hdr, 1, base + tail_off, max(0, total - tail_off))
        if idx == active_idx:
            hot_w = min(w_len, STEP_WORK_L2_B)
            add_span(2, 1, base + w_off, hot_w)
            if w_len > hot_w:
                add_span(3, 1, base + w_off + hot_w, w_len - hot_w)
        else:
            add_span(3, 1, base + w_off, w_len)
        if len(args) >= 2:
            code_syms.append((_fn_addr(args[1]), 0))

    obs = (g.get('observer') or {}).get('c')
    if isinstance(obs, ctypes.Structure):
        base, total = ctypes.addressof(obs), ctypes.sizeof(obs)
        l2_len = min(total, GMO_LAW_L2_B)
        add_span(2, 1, base, l2_len)
        if total > l2_len:
            add_span(3, 1, base + l2_len, total - l2_len)

    for k in ('brace', 'bend', 'arms', 'reaches', 'folds'):
        add_span(2, 0, *_arr_span(g.get(k)))
    sits = g.get('situations') or {}
    if isinstance(sits, dict):
        for k in ('caps', 'floors', 'grips'):
            add_span(2, 0, *_arr_span(sits.get(k)))
    bus = g.get('bus')
    if isinstance(bus, tuple) and bus:
        add_span(2, 0, *_struct_span(bus[0]))
    for fn_key in ('impedance', 'latent'):
        fn = g.get(fn_key)
        args = getattr(fn, 'args', ())
        if args:
            st = args[0][0] if isinstance(args[0], tuple) and args[0] else args[0]
            add_span(2, 0, *_struct_span(st))

    for k in ('posture', 'grip', 'grip_seen', 'grip_settled', 'grip_prior', 'into', 'eased',
              'touched', 'twisted', 'words', 'tab'):
        add_span(2, 1, *_arr_span(g.get(k)))
    world = g.get('world')
    d = getattr(world, 'data', None)
    if d is not None:
        for attr in ('qpos', 'qvel', 'qacc', 'ctrl', 'qfrc_bias', 'xpos', 'xmat', 'qM'):
            add_span(2, 1, *_arr_span(getattr(d, attr, None)))

    for val in g.values():
        if isinstance(val, bodied.Compiled) and val.c is not None:
            c = val.c
            code_syms.append((_fn_addr(c.cfn), 0))
            add_span(2, 1, *_arr_span(c.words))
            add_span(2, 1, *_arr_span(c._sbuf))
            for blk in c.blocks:
                add_span(2, 1, *_arr_span(blk))
            w_addr, w_len = _arr_span(c.work)
            if w_addr:
                hot_w = min(w_len, PART_WORK_L2_B)
                add_span(2, 1, w_addr, hot_w)
                if w_len > hot_w:
                    add_span(3, 1, w_addr + hot_w, w_len - hot_w)
        elif isinstance(val, bodied.Shaped):
            active_part = val.part
            for part in val.values():
                if part.c is None:
                    continue
                c = part.c
                tier = 2 if part is active_part else 3
                code_syms.append((_fn_addr(c.cfn), 0))
                add_span(tier, 1, *_arr_span(c.words))
                for blk in c.blocks:
                    add_span(tier, 1, *_arr_span(blk))
                add_span(tier, 1, *_arr_span(c.work))
        elif hasattr(val, 'cfn') and hasattr(val, 'lib_path'):
            code_syms.append((_fn_addr(getattr(val, 'cfn', None)), 0))

    played = g.get('played')
    bank = g.get('bank') or {}
    slots = list((bank.get('moves') or {}).values())
    slots.extend(entry[1] for entry in (bank.get('engrams') or {}).values()
                 if isinstance(entry, tuple) and len(entry) >= 2 and isinstance(entry[1], dict))
    for slot in slots:
        if slot is played or not isinstance(slot, dict):
            continue
        mv = slot.get('move')
        if mv is not None and hasattr(mv, '_laid'):
            laid_mv = mv._laid()
            if isinstance(laid_mv, tuple) and len(laid_mv) >= 2:
                add_span(3, 0, *_struct_span(laid_mv[1]))
        for k in ('kp', 'kd', 'joints'):
            add_span(3, 0, *_arr_span(slot.get(k)))
        for k in ('state', 'cmd', 'pose', 'target', 'feed', 'limit'):
            add_span(3, 1, *_arr_span(slot.get(k)))
    for tab in (bank.get('tables') or {}).values():
        add_span(3, 0, *_arr_span(tab))


@functools.lru_cache(maxsize=1)
def _quick_init_addr() -> int:
    """Resolve `PyInit__quick` once per process without repeated `CDLL` loads."""
    from tools.cores import quick
    qmod = quick.module()
    if qmod is None or not getattr(qmod, '__file__', None):
        return 0
    try:
        qlib = ctypes.CDLL(qmod.__file__)
        _KEEP_LIBS.append(qlib)
        return _fn_addr(getattr(qlib, 'PyInit__quick', None))
    except OSError:
        return 0


def _collect_code(g, add_span, extra_syms: list[tuple[int, int]]) -> int:
    """Pin shared-library `.text` segments into OS RAM and register hot entrypoints in L2 and segments in L3."""
    from tools.cores import motion as c_motion, wbc
    lib = wbc.library()
    syms: list[tuple[int, int]] = [
        (_fn_addr(getattr(lib, 'wbh_beat_passed', None)), 0),
        (_fn_addr(getattr(lib, 'wbh_pass_drives', None)), 0),
        (_fn_addr(getattr(lib, 'wbh_step', None)), 0),
    ]
    blas = c_motion.numpy_blas()
    syms.append((int(blas.gemm or 0), OPENBLAS_CAP_B))
    q_addr = _quick_init_addr()
    if q_addr:
        syms.append((q_addr, 0))
    syms.extend(extra_syms)
    seen_bases: set[int] = set()
    seen_hot: set[int] = set()
    for sym, cap_b in syms:
        if not sym:
            continue
        base, size = _pin_symbol(sym, cap_b)
        if base and base not in seen_bases:
            seen_bases.add(base)
            add_span(3, 0, base, size)
        hot_page = sym & ~63
        if hot_page not in seen_hot and (not base or base <= sym < base + size):
            seen_hot.add(hot_page)
            lo = max(base, hot_page) if base else hot_page
            ln = min(CODE_WINDOW_B, (base + size - lo) if base else CODE_WINDOW_B)
            if ln > 0:
                add_span(2, 0, lo, ln)
    return len(seen_bases)


class Loader:

    """Compiled SoA L1/L2/L3 cache-prefetch plan bound to a live control session `g`."""

    def __init__(self, g, *, eager: bool = True):
        self.info = caches()
        self.bound_parts = 0
        self.pinned_libs = 0
        self.tier_bytes = (0, 0, 0)
        self.tier_spans = (0, 0, 0)
        self.lines_all = 0
        self.lines_hot = 0
        self.lines_l1 = 0
        self.addrs = np.zeros(1, dtype=np.uint64)
        self.lens = np.zeros(1, dtype=np.uint64)
        self.cuts = np.zeros(6, dtype=np.int32)
        self._ap = self.addrs.ctypes.data_as(ctypes.POINTER(ctypes.c_uint64))
        self._lp = self.lens.ctypes.data_as(ctypes.POINTER(ctypes.c_uint64))
        self._cp = self.cuts.ctypes.data_as(ctypes.POINTER(ctypes.c_int32))
        self.rebuild(g, eager=eager)

    def rebuild(self, g, *, eager: bool = False):
        """Rebuild tiered `(L3, L2, L1)` SoA span tables from `g` and prefetch all three tiers."""
        if eager:
            self.bound_parts += _eager_bind_kept(g)
        line, l1d, _l1i, l2, l3 = self.info
        caps = {1: int(l1d * BUDGET_SHARE), 2: int(l2 * BUDGET_SHARE), 3: int(l3 * BUDGET_SHARE)}
        used = {1: 0, 2: 0, 3: 0}
        seen: set[tuple[int, int]] = set()
        buckets: dict[tuple[int, int], list[tuple[int, int]]] = {
            (3, 0): [], (3, 1): [], (2, 0): [], (2, 1): [], (1, 0): [], (1, 1): [],
        }

        def add_span(tier: int, rw: int, addr: int, nbytes: int):
            if addr <= 0 or nbytes <= 0:
                return
            aligned = addr & ~(line - 1)
            span_b = ((addr + nbytes - aligned + line - 1) // line) * line
            key = (aligned, span_b)
            if key in seen:
                return
            t = tier
            while t < 3 and used[t] + span_b > caps[t]:
                t += 1
            if used[t] + span_b > caps[t]:
                rem = max(0, (caps[t] - used[t]) // line * line)
                if rem <= 0:
                    return
                span_b = rem
            seen.add((aligned, span_b))
            used[t] += span_b
            buckets[(t, rw)].append((aligned, span_b))

        code_syms: list[tuple[int, int]] = []
        _collect_l1(g, add_span)
        _collect_l2_l3(g, add_span, code_syms)
        self.pinned_libs = _collect_code(g, add_span, code_syms)

        order = ((3, 0), (3, 1), (2, 0), (2, 1), (1, 0), (1, 1))
        flat_a: list[int] = []
        flat_l: list[int] = []
        cuts: list[int] = []
        for key in order:
            for a, ln in buckets[key]:
                flat_a.append(a)
                flat_l.append(ln)
            cuts.append(len(flat_a))
        self.addrs = np.array(flat_a or [0], dtype=np.uint64)
        self.lens = np.array(flat_l or [0], dtype=np.uint64)
        self.cuts = np.array(cuts if flat_a else [0] * 6, dtype=np.int32)
        self._ap = self.addrs.ctypes.data_as(ctypes.POINTER(ctypes.c_uint64))
        self._lp = self.lens.ctypes.data_as(ctypes.POINTER(ctypes.c_uint64))
        self._cp = self.cuts.ctypes.data_as(ctypes.POINTER(ctypes.c_int32))
        self.tier_bytes = (used[1], used[2], used[3])
        self.tier_spans = (
            len(buckets[(1, 0)]) + len(buckets[(1, 1)]),
            len(buckets[(2, 0)]) + len(buckets[(2, 1)]),
            len(buckets[(3, 0)]) + len(buckets[(3, 1)]),
        )
        self.all()
        return self

    def all(self) -> int:
        """Execute bottom-up `L3 -> L2 -> L1` hardware prefetch across all staged spans."""
        self.lines_all = int(_lib().pf_prefetch_all(self._ap, self._lp, self._cp, self.info.line))
        self.lines_hot = int(self.cuts[5] - self.cuts[1]) and int(
            (self.tier_bytes[0] + self.tier_bytes[1]) // self.info.line)
        self.lines_l1 = int(self.tier_bytes[0] // self.info.line)
        return self.lines_all

    def hot(self) -> int:
        """Execute `L2 -> L1` hardware prefetch for law and 1 kHz inner-loop working sets."""
        self.lines_hot = int(_lib().pf_prefetch_hot(self._ap, self._lp, self._cp, self.info.line))
        return self.lines_hot

    def l1(self) -> int:
        """Execute `L1` hardware prefetch for the 1 kHz inner-loop working set alone."""
        self.lines_l1 = int(_lib().pf_prefetch_l1(self._ap, self._lp, self._cp, self.info.line))
        return self.lines_l1

    def stats(self) -> dict:
        """Return measured cache capacities, staged byte/span counts, and prefetch latency."""
        t0 = time.perf_counter()
        hot_lines = self.hot()
        hot_us = round(1e6 * (time.perf_counter() - t0), 2)
        t1 = time.perf_counter()
        l1_lines = self.l1()
        l1_us = round(1e6 * (time.perf_counter() - t1), 2)
        return {
            'caches': self.info._asdict(),
            'l1_bytes': self.tier_bytes[0], 'l2_bytes': self.tier_bytes[1], 'l3_bytes': self.tier_bytes[2],
            'l1_spans': self.tier_spans[0], 'l2_spans': self.tier_spans[1], 'l3_spans': self.tier_spans[2],
            'pinned_libs': self.pinned_libs, 'bound_parts': self.bound_parts,
            'lines_all': self.lines_all, 'lines_hot': hot_lines, 'lines_l1': l1_lines,
            'hot_us': hot_us, 'l1_us': l1_us,
        }


def stage(g, *, eager: bool = True) -> Loader:
    """Build or rebuild `g['_prefetch']` and prefetch `L3 -> L2 -> L1`."""
    loader = g.get('_prefetch')
    if isinstance(loader, Loader):
        return loader.rebuild(g, eager=eager)
    loader = g['_prefetch'] = Loader(g, eager=eager)
    return loader


def refresh(g) -> Loader | None:
    """Refresh `g['_prefetch']` spans after an engram, move, or core transition."""
    loader = g.get('_prefetch')
    return loader.rebuild(g, eager=False) if isinstance(loader, Loader) else None


def hot(g) -> int:
    """Prefetch `L2 -> L1` on `g['_prefetch']` if staged."""
    loader = g.get('_prefetch')
    return loader.hot() if isinstance(loader, Loader) else 0


def l1(g) -> int:
    """Prefetch `L1` on `g['_prefetch']` if staged."""
    loader = g.get('_prefetch')
    return loader.l1() if isinstance(loader, Loader) else 0
