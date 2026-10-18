"""Columnar CSV and telemetry tables on RAPIDS (`cudf` / `cupy`) or `pandas` / `numpy`.

When RAPIDS (`cudf`) and `cupy` are installed on the machine's GPU (and `COAXIAL_GPU` is not
`'0'`), CSV ingestion and vector reductions run on device; otherwise `pandas`'s C engine and
`numpy` run the identical columnar operations without row-by-row Python loops.
"""
import csv
import importlib
import math
import os
from typing import Any

from coaxial.model.blocks import numpy as np

_MODS: dict[str, Any] = {}


def _gpu_enabled(gpu=None):
    if gpu is not None:
        return bool(gpu)
    return os.environ.get('COAXIAL_GPU', '1').strip().lower() not in ('0', 'false', 'no', 'cpu')


def xp(gpu=None):
    """`cupy` when installed and GPU is enabled, otherwise `numpy`."""
    if _gpu_enabled(gpu):
        if 'cp' not in _MODS:
            try:
                cp = importlib.import_module('cupy')
                cp.cuda.Device(0).attributes
                _MODS['cp'] = cp
            except Exception:
                _MODS['cp'] = None
        if _MODS['cp'] is not None:
            return _MODS['cp']
    return np


def df_lib(gpu=None):
    """`cudf` (RAPIDS) when installed and GPU is enabled, otherwise `pandas`."""
    if _gpu_enabled(gpu):
        if 'cudf' not in _MODS:
            try:
                _MODS['cudf'] = importlib.import_module('cudf')
            except Exception:
                _MODS['cudf'] = None
        if _MODS['cudf'] is not None:
            return _MODS['cudf']
    if 'pd' not in _MODS:
        try:
            _MODS['pd'] = importlib.import_module('pandas')
        except Exception:
            _MODS['pd'] = None
    return _MODS['pd']


def to_numpy(series_or_array):
    """Host `numpy.ndarray` from a `cudf` Series, `pandas` Series, `cupy` array, or sequence."""
    if hasattr(series_or_array, 'to_numpy'):
        return series_or_array.to_numpy()
    if hasattr(series_or_array, 'get'):
        return series_or_array.get()
    return np.asarray(series_or_array)


def read_frame(path, usecols=None, dtype=None, keep_default_na=True, gpu=None):
    """Read `path` into a `cudf.DataFrame` (RAPIDS) or `pandas.DataFrame` (C engine)."""
    lib = df_lib(gpu)
    if lib is None:
        raise RuntimeError('tabular.read_frame requires cudf or pandas')
    kw: dict[str, Any] = {'keep_default_na': keep_default_na}
    if usecols is not None:
        kw['usecols'] = list(usecols)
    if dtype is not None:
        kw['dtype'] = dtype
    return lib.read_csv(path, **kw)


def numeric_columns(path, names=None, defaults=None, gpu=None):
    """{column: float64 ndarray} from `path` via `read_frame`, filling `defaults` when absent."""
    lib = df_lib(gpu)
    defaults = defaults or {}
    if lib is not None:
        frame = read_frame(path, gpu=gpu)
        present = set(frame.columns)
        wanted = [c for c in (names if names is not None else frame.columns) if c in present]
        out = {}
        n = len(frame)
        for col in wanted:
            try:
                out[col] = np.asarray(to_numpy(frame[col]), dtype=np.float64)
            except (ValueError, TypeError):
                continue
        for col, val in defaults.items():
            if col not in out:
                out[col] = np.full(n, float(val), dtype=np.float64)
        return out
    with open(path, encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return {}
    wanted = [c for c in (names if names is not None else rows[0].keys()) if c in rows[0]]
    out = {}
    for col in wanted:
        try:
            out[col] = np.array([float(r[col]) for r in rows], dtype=np.float64)
        except (ValueError, TypeError):
            continue
    for col, val in defaults.items():
        if col not in out:
            out[col] = np.full(len(rows), float(val), dtype=np.float64)
    return out


def read_rows(path, gpu=None):
    """List of `{col: str}` row dicts parsed by the C/GPU CSV engine (`cudf` or `pandas`)."""
    lib = df_lib(gpu)
    if lib is not None:
        frame = read_frame(path, dtype=str, keep_default_na=False, gpu=gpu)
        if hasattr(frame, 'to_pandas'):
            frame = frame.to_pandas()
        return frame.to_dict('records')
    with open(path, encoding='utf-8') as f:
        return list(csv.DictReader(f))


def fit_line(t, a, gpu=None):
    """(mean, p2p, slope_per_min, sd, detrended_sd) via vectorized dot products on `xp(gpu)`."""
    mod = xp(gpu)
    tv = mod.asarray(t, dtype=mod.float64)
    av = mod.asarray(a, dtype=mod.float64)
    n = int(tv.size)
    m = float(av.mean())
    tb = float(tv.mean())
    dt = tv - tb
    da = av - m
    sxx = float(dt @ dt)
    slope = float(dt @ da) / sxx if sxx > 0.0 else 0.0
    res = da - slope * dt
    sd = math.sqrt(float(da @ da) / (n - 1))
    rsd = math.sqrt(float(res @ res) / (n - 1))
    p2p = float(av.max() - av.min())
    return m, p2p, slope * 60000.0, sd, rsd

