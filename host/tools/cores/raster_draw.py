"""SIMD C11 bindings for ascii3d, cross_section, dial, thermalmap, and lit splat (`raster_draw.c`)."""
import ctypes
import functools
import math
import threading

import numpy as np

from machine.rig import ansi
from tools.cores import raster

_draw_tls = threading.local()
_MESH_CACHE: dict = {}
_TINT_CACHE: dict = {}
_SHADE_LUT: dict = {}
_THERMAL_CACHE: dict = {}
_THERMAL_CONST: list = [None]
_LIT_CONST: list = [None]


@functools.lru_cache(maxsize=1)
def _lib():
    """Bind `raster_draw.c` symbols on the shared `raster` library."""
    lib = raster.ensure()
    if lib is None:
        return None
    i32, vp = ctypes.c_int, ctypes.c_void_p
    lib.rs_ascii3d_raster.argtypes = [
        i32, vp, i32, vp, vp, vp, vp, i32, i32, i32, i32, vp,
        vp, vp, vp, vp, vp, vp, vp, vp, vp,
    ]
    lib.rs_ascii3d_raster.restype = None
    lib.rs_ascii3d_resolve.argtypes = [
        vp, vp, vp, i32, i32, i32, i32, i32, vp, i32, i32, i32, i32, i32,
        i32, vp, i32, vp, vp, vp, vp, vp, vp, vp,
    ]
    lib.rs_ascii3d_resolve.restype = None
    lib.rs_csec_seat.argtypes = [i32, i32, vp, vp, vp, vp, vp, vp]
    lib.rs_csec_seat.restype = i32
    lib.rs_csec_body.argtypes = [
        i32, vp, vp, vp, vp, vp, i32, i32, vp, i32, i32, i32,
        vp, vp, vp, vp, vp, vp,
    ]
    lib.rs_csec_body.restype = None
    lib.rs_dial_arrays.argtypes = [i32, i32, vp, vp, vp, vp, vp, vp, vp, vp, vp, vp, vp, vp]
    lib.rs_dial_arrays.restype = None
    lib.rs_dial_frame.argtypes = [
        i32, i32, i32, vp, vp, vp, vp, vp, vp, vp, vp, vp, vp,
        i32, ctypes.c_double, ctypes.c_double, ctypes.c_double,
        i32, ctypes.c_double, ctypes.c_double, ctypes.c_double,
        vp, vp, vp, vp, vp, vp,
    ]
    lib.rs_dial_frame.restype = None
    lib.rs_thermal_braille.argtypes = [i32, i32, vp, vp, vp, vp, vp, vp, i32, vp, vp]
    lib.rs_thermal_braille.restype = None
    lib.rs_lit_shade_win.argtypes = [i32, vp, vp, vp, vp, vp, vp, vp, vp, vp, vp]
    lib.rs_lit_shade_win.restype = None
    return lib


def _model_arrays(model):
    pos, idx, nrm = model
    key = (id(pos), id(idx), id(nrm))
    hit = _MESH_CACHE.get(key)
    if hit is None or hit[0] is not pos or hit[1] is not idx or hit[2] is not nrm:
        if len(_MESH_CACHE) >= 16:
            _MESH_CACHE.clear()
        hit = _MESH_CACHE[key] = (
            pos, idx, nrm,
            np.ascontiguousarray(pos, dtype=np.float64),
            np.ascontiguousarray(idx, dtype=np.int32),
            np.ascontiguousarray(nrm, dtype=np.float64),
        )
    return hit[3], hit[4], hit[5]


def _tints_array(tints):
    if tints is None:
        return None
    hit = _TINT_CACHE.get(id(tints))
    if hit is None or hit[0] is not tints:
        if len(_TINT_CACHE) >= 16:
            _TINT_CACHE.clear()
        hit = _TINT_CACHE[id(tints)] = (tints, np.ascontiguousarray(tints, dtype=np.int32))
    return hit[1]


def ascii3d_raster(model, matrix, distance, scale, cx, cy, cols, top, bottom, lamp, cull,
                   tints=None, who=None):
    """Run `ascii3d.rasterise` in C, returning `(depth_arr, value_arr, who_arr)` or `None`."""
    lib = _lib()
    if lib is None:
        return None
    pos_c, idx_c, nrm_c = _model_arrays(model)
    n_verts, n_tris = len(pos_c) // 3, len(idx_c) // 3
    v_hit = getattr(_draw_tls, 'a3_v', None)
    if v_hit is None or v_hit[0] < n_verts:
        v_hit = (n_verts, tuple(np.empty(max(1, n_verts), dtype=np.float64) for _ in range(6)))
        _draw_tls.a3_v = v_hit
    v_sc = v_hit[1]
    deep = max(0, (bottom - top) * cols)
    depth = np.empty(deep, dtype=np.float64)
    value = np.empty(deep, dtype=np.float64)
    need_who = who is not None or tints is not None
    who_c = np.empty(deep, dtype=np.int32) if need_who else None
    tints_c = _tints_array(tints) if need_who and tints is not None else None
    m9 = np.ascontiguousarray(matrix, dtype=np.float64).ravel()
    cam8 = np.array([distance, scale, cx, cy, lamp[0], lamp[1], lamp[2], 0.0], dtype=np.float64)
    lib.rs_ascii3d_raster(
        n_verts, pos_c.ctypes.data, n_tris, idx_c.ctypes.data, nrm_c.ctypes.data,
        m9.ctypes.data, cam8.ctypes.data, int(cols), int(top), int(bottom), int(bool(cull)),
        0 if tints_c is None else tints_c.ctypes.data,
        v_sc[0].ctypes.data, v_sc[1].ctypes.data, v_sc[2].ctypes.data,
        v_sc[3].ctypes.data, v_sc[4].ctypes.data, v_sc[5].ctypes.data,
        depth.ctypes.data, value.ctypes.data,
        0 if who_c is None or who is None else who_c.ctypes.data,
    )
    if isinstance(who, list) and who_c is not None:
        who[:] = who_c.tolist()
    return depth, value, (who_c if who is not None else None)


def _shades_lut(shades):
    if shades is None:
        return None
    hit = _SHADE_LUT.get(id(shades))
    if hit is None or hit[0] is not shades:
        if len(_SHADE_LUT) >= 16:
            _SHADE_LUT.clear()
        lut = np.repeat(np.arange(256, dtype=np.int32)[:, None], 3, axis=1)
        for k, triple in shades.items():
            if 0 <= k < 256:
                lut[k] = triple
        hit = _SHADE_LUT[id(shades)] = (shades, np.ascontiguousarray(lut.ravel(), dtype=np.int32))
    return hit[1]


def ascii3d_resolve(depth, value, width, height, cols, cell_rows, supersample, ramp, invert,
                    ink=None, who=None, ink_colour=None, shades=None, wire=False):
    """Run `ascii3d.resolve` in C, returning formatted multi-line text or `None`."""
    lib = _lib()
    if lib is None or height <= 0 or width <= 0:
        return None
    depth_c = np.ascontiguousarray(depth, dtype=np.float64)
    value_c = np.ascontiguousarray(value, dtype=np.float64)
    who_c = None if who is None else np.ascontiguousarray(who, dtype=np.int32)
    ncells = width * height
    r_key = (ncells, height)
    r_hit = getattr(_draw_tls, 'a3_r', None)
    if r_hit is None or r_hit[0] != r_key:
        r_hit = (
            r_key,
            np.empty(ncells, dtype=np.float64),
            np.empty(ncells, dtype=np.float64),
            np.empty(ncells, dtype=np.int32),
            np.empty(ncells, dtype=np.int32),
            np.empty(ncells, dtype=np.int32),
            np.empty(ncells, dtype=np.int32),
            np.empty(height, dtype=np.int32),
        )
        _draw_tls.a3_r = r_hit
    sc = r_hit[1:]
    ramp_c = np.array([ord(ch) for ch in ramp], dtype=np.int32)
    lut_c = _shades_lut(shades)
    lib.rs_ascii3d_resolve(
        depth_c.ctypes.data, value_c.ctypes.data, 0 if who_c is None else who_c.ctypes.data,
        int(width), int(height), int(cols), int(cell_rows), int(supersample),
        ramp_c.ctypes.data, len(ramp_c), int(bool(invert)),
        int(ink is not None), ord(ink) if ink is not None else 0,
        -1 if ink_colour is None else int(ink_colour),
        int(lut_c is not None), 0 if lut_c is None else lut_c.ctypes.data, int(bool(wire)),
        sc[0].ctypes.data, sc[1].ctypes.data, 0 if who_c is None else sc[2].ctypes.data,
        sc[3].ctypes.data, sc[4].ctypes.data, sc[5].ctypes.data, sc[6].ctypes.data,
    )
    text_all = sc[4].view(f'=U{ncells}')[0]
    lens = sc[6].tolist()
    if who_c is None:
        return '\n'.join(text_all[r * width:r * width + lens[r]] for r in range(height))
    tones_all = sc[5].tolist()
    lines = []
    for r in range(height):
        n = lens[r]
        if n == 0:
            lines.append('')
            continue
        row = r * width
        t_row = [None if v < 0 else v for v in tones_all[row:row + n]]
        lines.append(ansi.run_row(text_all[row:row + n], t_row))
    return '\n'.join(lines)


def csec_seat(width: int, height: int, seat):
    """Build `cross_section._seat_arrayed` dict in C, or return `None`."""
    lib = _lib()
    if lib is None:
        return None
    r = seat.radii
    g11 = np.array([seat.cx, seat.cy, seat.stretch, r.can, r.magnet_out, r.magnet_in,
                    r.tooth_out, r.tooth_in, r.bore, r.can_inner, r.line], dtype=np.float64)
    ncap = max(1, width * 2 * height * 4)
    kind = np.empty((ncap, 4), dtype=np.int64)
    a = np.empty((ncap, 4), dtype=np.float64)
    b = np.empty((ncap, 4), dtype=np.float64)
    xs = np.empty(ncap, dtype=np.int32)
    ys = np.empty(ncap, dtype=np.int32)
    m = lib.rs_csec_seat(int(width), int(height), g11.ctypes.data, kind.ctypes.data,
                         a.ctypes.data, b.ctypes.data, xs.ctypes.data, ys.ctypes.data)
    xc, yc = xs[:m].copy(), ys[:m].copy()
    return {'kind': kind[:m].copy(), 'a': a[:m].copy(), 'b': b[:m].copy(),
            'x': xc.tolist(), 'y': yc.tolist(), '_xc': xc, '_yc': yc}


def csec_body(frame, s, r, rotor_deg: float, slots: int, poles: int, drive, blur: float) -> bool:
    """Run `cross_section._body` in C into `frame`, returning `True` on success."""
    lib = _lib()
    if lib is None:
        return False
    width, height = int(frame.width), int(frame.height)
    kind = np.ascontiguousarray(s['kind'], dtype=np.int64)
    a = np.ascontiguousarray(s['a'], dtype=np.float64)
    b = np.ascontiguousarray(s['b'], dtype=np.float64)
    xc = s.get('_xc')
    yc = s.get('_yc')
    if xc is None or yc is None:
        xc = np.asarray(s['x'], dtype=np.int32)
        yc = np.asarray(s['y'], dtype=np.int32)
    n_dots, ncells = len(kind), width * height
    key = (width, height, n_dots)
    cs_hit = getattr(_draw_tls, 'csec', None)
    if cs_hit is None or cs_hit[0] != key:
        cs_hit = (
            key,
            np.empty(width * 2 * height * 4, dtype=np.uint8),
            np.empty(max(1, n_dots), dtype=np.int32),
            np.empty(max(1, n_dots), dtype=np.int32),
            np.empty((ncells, 9), dtype=np.int32),
            np.empty((height, width), dtype=np.int32),
            np.empty((height, width), dtype=np.int32),
        )
        _draw_tls.csec = cs_hit
    sc = cs_hit[1:]
    coast = int(drive is not None and max(abs(d) for d in drive) < 0.02)
    pars8 = np.array([r.tooth_in, r.tooth_out, r.magnet_in, r.magnet_out, r.line,
                      math.radians(rotor_deg), float(blur), math.tau], dtype=np.float64)
    lib.rs_csec_body(
        n_dots, kind.ctypes.data, a.ctypes.data, b.ctypes.data, xc.ctypes.data, yc.ctypes.data,
        width, height, pars8.ctypes.data, int(slots), int(poles), coast,
        sc[0].ctypes.data, sc[1].ctypes.data, sc[2].ctypes.data, sc[3].ctypes.data,
        sc[4].ctypes.data, sc[5].ctypes.data,
    )
    frame.dots = sc[4].tolist()
    frame.owner = sc[5].tolist()
    lit_cells = np.flatnonzero(sc[4].ravel()).tolist()
    t9 = sc[3]
    tally = frame.tally
    for cell_i in lit_cells:
        row, col = divmod(cell_i, width)
        vals = t9[cell_i].tolist()
        tally[row][col] = {cls: cnt for cls, cnt in enumerate(vals) if cnt}
    return True


def dial_arrays(width: int, height: int, aspect: float):
    """Build `dial._arrayed` dict in C, or return `None`."""
    lib = _lib()
    if lib is None:
        return None
    cx, cy = width * 2 / 2.0 - 0.5, height * 4 / 2.0 - 0.5
    rim = min(width * 2, height * 4) / 2.0 - 7.0 - 4.0 - 1.0
    line = max(0.8, min(1.0, rim * 0.020))
    stretch = aspect / 4.0 * 2.0
    pars12 = np.array([cx, cy, rim, line, stretch, math.tau, math.degrees(1.0), math.radians(1.0),
                       math.radians(30), math.radians(6), math.radians(3), 0.0], dtype=np.float64)
    dcap = max(1, width * 2 * height * 4)
    scap = dcap * 4
    dot = np.empty(scap, dtype=np.int64)
    dx = np.empty(scap, dtype=np.float64)
    dy = np.empty(scap, dtype=np.float64)
    phi = np.empty(scap, dtype=np.float64)
    base = np.empty(scap, dtype=np.int64)
    needle = np.empty(scap, dtype=np.uint8)
    sweep = np.empty(scap, dtype=np.uint8)
    seg = np.empty(scap, dtype=np.int64)
    cell = np.empty(dcap, dtype=np.int64)
    bit = np.empty(dcap, dtype=np.int64)
    counts = np.zeros(2, dtype=np.int64)
    lib.rs_dial_arrays(
        int(width), int(height), pars12.ctypes.data,
        dot.ctypes.data, dx.ctypes.data, dy.ctypes.data, phi.ctypes.data,
        base.ctypes.data, needle.ctypes.data, sweep.ctypes.data, seg.ctypes.data,
        cell.ctypes.data, bit.ctypes.data, counts.ctypes.data,
    )
    nd, ns = int(counts[0]), int(counts[1])
    return {'dot': dot[:ns].copy(), 'dx': dx[:ns].copy(), 'dy': dy[:ns].copy(),
            'phi': phi[:ns].copy(), 'base': base[:ns].copy(),
            'needle': needle[:ns].view(bool).copy(), 'sweep': sweep[:ns].view(bool).copy(),
            'seg': seg[:ns].copy(), 'cell': cell[:nd].copy(), 'bit': bit[:nd].copy()}


def dial_frame(a, geom, span, needle, lit, glow, width: int, height: int):
    """Run `dial._raster` sample classification and dot scatter in C, or return `None`."""
    lib = _lib()
    if lib is None:
        return None
    n_samples, n_dots, ncells = len(a['dot']), len(a['cell']), width * height
    key = (n_dots, height, width)
    dl_hit = getattr(_draw_tls, 'dial_f', None)
    if dl_hit is None or dl_hit[0] != key:
        dl_hit = (
            key,
            np.empty(max(1, n_dots), dtype=np.int32),
            np.empty(max(1, n_dots), dtype=np.int64),
            np.empty((height, width), dtype=np.int64),
            np.empty((height, width), dtype=np.int64),
        )
        _draw_tls.dial_f = dl_hit
    sc = dl_hit[1:]
    seg_lut = None
    if geom.subs:
        flat = []
        for which in range(len(geom.subs)):
            on = lit[which] if which < len(lit) else None
            flat += [on[k] if on and k < len(on) else 6 for k in range(24)]
        seg_lut = np.array(flat, dtype=np.int64)
    glow_c = None if glow is None else np.ascontiguousarray(glow, dtype=np.float64)
    nc, ns_v = (needle[0], needle[1]) if needle is not None else (0.0, 0.0)
    lib.rs_dial_frame(
        n_samples, n_dots, ncells,
        a['dot'].ctypes.data, a['dx'].ctypes.data, a['dy'].ctypes.data, a['phi'].ctypes.data,
        a['base'].ctypes.data, a['needle'].view(np.uint8).ctypes.data,
        a['sweep'].view(np.uint8).ctypes.data, a['seg'].ctypes.data,
        a['cell'].ctypes.data, a['bit'].ctypes.data,
        int(needle is not None), float(nc), float(ns_v), float(geom.needle),
        int(span is not None), float(span or 0.0), math.radians(70.0), math.tau,
        0 if glow_c is None else glow_c.ctypes.data,
        0 if seg_lut is None else seg_lut.ctypes.data,
        sc[0].ctypes.data, sc[1].ctypes.data, sc[2].ctypes.data, sc[3].ctypes.data,
    )
    return sc[2].tolist(), sc[3].tolist()


def stash_thermal_grid(rows, temps, on):
    """Remember the latest `(temps, on)` NumPy arrays behind `thermalmap._grid`'s list output."""
    _draw_tls.grid_stash = (
        rows,
        np.ascontiguousarray(temps, dtype=np.float64),
        np.ascontiguousarray(on, dtype=np.uint8),
    )


def thermal_braille_rows(grid, mask, labels, noise):
    """Run `thermalmap._braille_rows` in C + `ansi.run_row`, or return `None`."""
    lib = _lib()
    if lib is None or not grid or not grid[0]:
        return None
    down, cells = len(grid), len(grid[0])
    stash = getattr(_draw_tls, 'grid_stash', None)
    if stash is not None and stash[0] is grid:
        temps_c, on_c = stash[1], stash[2]
    else:
        temps_c = np.array([[0.0 if v is None else float(v) for v in r] for r in grid],
                           dtype=np.float64)
        on_c = np.array([[0 if v is None else 1 for v in r] for r in grid], dtype=np.uint8)
    m_hit = _THERMAL_CACHE.get(id(mask))
    if m_hit is None or m_hit[0] is not mask:
        if len(_THERMAL_CACHE) >= 16:
            _THERMAL_CACHE.clear()
        m_hit = _THERMAL_CACHE[id(mask)] = (mask, np.asarray(mask, dtype=np.uint8, order='C'))
    c_const = _THERMAL_CONST[0]
    if c_const is None or c_const[0] is not noise:
        stops_c = np.array([s[0] for s in ansi.THERMAL_STOPS], dtype=np.float64)
        stops_rgb = np.array([ch for _t, rgb in ansi.THERMAL_STOPS for ch in rgb], dtype=np.int32)
        noise_c = np.asarray(noise, dtype=np.uint16, order='C')
        c_const = _THERMAL_CONST[0] = (noise, stops_c, stops_rgb, noise_c)
    rows_n = down // 2
    out_codes = np.empty(rows_n * cells, dtype=np.int32)
    out_rgb = np.empty(rows_n * cells, dtype=np.int32)
    lib.rs_thermal_braille(
        down, cells, temps_c.ctypes.data, on_c.ctypes.data, m_hit[1].ctypes.data,
        c_const[3].ctypes.data, c_const[1].ctypes.data, c_const[2].ctypes.data,
        len(c_const[1]), out_codes.ctypes.data, out_rgb.ctypes.data,
    )
    mark_ink = (60, 175, 185)
    text_all = list(out_codes.view(f'=U{rows_n * cells}')[0])
    rgb_list = out_rgb.tolist()
    tones_all = [None if v < 0 else ((v >> 16) & 255, (v >> 8) & 255, v & 255) for v in rgb_list]
    for (r, c), ch in labels.items():
        if 0 <= r < rows_n and 0 <= c < cells:
            idx = r * cells + c
            text_all[idx] = ch
            tones_all[idx] = mark_ink
    return [ansi.run_row(text_all[r * cells:(r + 1) * cells],
                         tones_all[r * cells:(r + 1) * cells]) for r in range(rows_n)]


def splat_lit(arrays, m, cam, centre, under_flag: int, layer_m: float):
    """Run `c_raster.splat_bins` + C11 `rs_lit_shade_win` matching `lit.splat`, or `None`."""
    lib = _lib()
    if lib is None:
        return None
    positions, normals, materials, uv = arrays
    bins = raster.splat_bins(positions, materials, m, cam, centre, under_flag, layer_m)
    if bins is None:
        return None
    depth, win = bins
    height, width = depth.shape
    c_lit = _LIT_CONST[0]
    if c_lit is None:
        from coaxial.graphics import lit as lit_mod
        _np, key3, fill3, half3, pal, *_ = lit_mod._const()
        c_lit = _LIT_CONST[0] = (
            np.ascontiguousarray(key3, dtype=np.float64),
            np.ascontiguousarray(fill3, dtype=np.float64),
            np.ascontiguousarray(half3, dtype=np.float64),
            np.ascontiguousarray(pal, dtype=np.float64).ravel(),
        )
    nrm_c = np.ascontiguousarray(normals, dtype=np.float64)
    mat_c = np.ascontiguousarray(materials, dtype=np.uint32)
    uv_c = np.ascontiguousarray(uv, dtype=np.float64)
    m9 = np.ascontiguousarray(m, dtype=np.float64).ravel()
    rgb = np.empty((height, width, 4), dtype=np.uint8)
    lib.rs_lit_shade_win(
        height * width, win.ctypes.data, nrm_c.ctypes.data, mat_c.ctypes.data, uv_c.ctypes.data,
        m9.ctypes.data, c_lit[0].ctypes.data, c_lit[1].ctypes.data, c_lit[2].ctypes.data,
        c_lit[3].ctypes.data, rgb.ctypes.data,
    )
    return depth, rgb
