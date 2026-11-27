"""Compile and dry-run validate LLM-generated C11 microkernels, Python hooks, and Lisp engrams."""
import ctypes
import hashlib
import json
import math
import os
import re
import time
from typing import Any

from machine import libm
from machine.errors import MachineError
from machine.process import engrams
from machine.process.engram_catalog import Engram

_CORE_CACHE: dict[str, Any] = {}


def _safe_ident(name: str) -> str:
    """Sanitize `name` into a valid C and file identifier fragment."""
    cleaned = re.sub(r'[^0-9a-zA-Z_]+', '_', str(name).strip()).strip('_')
    if not cleaned:
        raise MachineError('core or engram name must contain ASCII letters or digits')
    return ('u_' + cleaned) if cleaned[0].isdigit() else cleaned


def _tables_from_defmove(name: str, lisp_text: str):
    """Compile a Lisp `(defmove ...)` source string into `machine.moves.sensed.Tables`."""
    import numpy as np
    from machine.moves import sensed as py_sensed
    from machine.texts import dsp, patch
    env = dict(dsp.read(dsp.SUITE))
    defined = None
    for form in patch.forms(lisp_text):
        if form[0] not in dsp.DEFINE:
            raise MachineError('(%s ..): unsupported form in move source' % form[0])
        got = dsp.DEFINE[form[0]](env, form)
        env.update(got)
        if form[0] == 'defmove':
            defined = form[1]
    move_name = defined or name
    if move_name not in env:
        raise MachineError('no defmove %r found in Lisp source' % (move_name,))
    told = dsp.value([move_name], env)
    phases = [(p['name'].lstrip(':'), py_sensed._ENDED[p['ends']], p['next'].lstrip(':'),
               p.get('again', p['next']).lstrip(':'), float(bool(p.get('counts', False))),
               py_sensed._goals_of(p)) for p in told['phases']]
    names, ends, nexts, agains, counts, goals = zip(*phases)
    said = told.get('constants', {})
    stroked = agains != nexts or any(counts)
    unsaid = sorted(({'tol'} | {n for e in ends for n in py_sensed.READS[e]}
                     | ({'near', 'strokes'} if stroked else set())) - set(said))
    if unsaid:
        raise MachineError('%s: move ends read %s, left unsaid' % (move_name, ', '.join(unsaid)))
    goal, rates, how = (np.zeros((len(phases), len(py_sensed.PARTS))) for _ in range(3))
    for k, part_goals in enumerate(goals):
        for part, (deg, rate, *how_) in part_goals.items():
            i = py_sensed.PARTS.index(part)
            goal[k, i], rates[k, i], how[k, i] = float(deg), float(rate), float((how_ or [py_sensed.RATE])[0])
    after, again = (np.array([float(names.index(n)) for n in ns]) for ns in (nexts, agains))
    return py_sensed.Tables(
        names, np.array(ends), after, again, np.array(counts),
        np.ascontiguousarray(goal, float), np.ascontiguousarray(rates, float),
        np.ascontiguousarray(how, float), names.index('done'),
        tuple(float(said.get(n, 0.0)) for n in py_sensed.NUMBERS), bool(told.get('holds', True)))


def _engram_from_row(row: dict[str, Any]) -> Engram:
    """Normalize an LLM engram dict (supporting raw Lisp `defmove` or `ortho` text) into `Engram`."""
    from machine.texts import ortho
    name, slot, kind = str(row['name']), str(row['slot']), str(row['kind'])
    task = str(row.get('task', slot))
    spec = dict(row.get('spec', {}))
    if kind == 'sensed' and 'lisp' in spec:
        spec = dict(spec, tables=_tables_from_defmove(name, str(spec['lisp'])))
    elif kind == 'wire' and 'lisp' in spec:
        tables, phases = ortho.compiled(str(spec['lisp']), texts=spec.get('texts'))
        beh = spec.get('behavior')
        if beh is None:
            beh = next(iter(phases)) if phases else ''
        if beh not in tables or not tables[beh]:
            raise MachineError('wire engram %s: Lisp source produced no rows for %r' % (name, beh))
        names = tuple(spec.get('names', (phases[beh] + ('rest',)) if beh in phases else (slot,)))
        wire = engrams._scale_wire(tables[beh], spec.get('tempo_scale', 1.0),
                                   spec.get('voice_scales'), spec.get('voice_offsets'))
        spec = {'wire': wire, 'names': names}
    return Engram(name, slot, kind, task, row.get('stunt_ref'), row.get('description', ''), spec)


class CompiledCCore:

    """Callable wrapper around a host-compiled C11 shared-library microkernel."""

    def __init__(self, name: str, hook: str | None, lib_path: str, entry: str,
                 arg_specs: tuple[str, ...], returns: Any, warnings: list[str], spec: dict[str, Any]):
        import numpy as np
        self.name, self.hook, self.lib_path, self.entry = name, hook, lib_path, entry
        self.arg_specs, self.returns, self.warnings, self.spec = arg_specs, returns, warnings, dict(spec)
        self.lib = ctypes.CDLL(lib_path)
        try:
            self.cfn = getattr(self.lib, entry)
        except AttributeError as exc:
            raise MachineError('C core %s: symbol %r missing in %s' % (name, entry, lib_path)) from exc
        c_args: list[Any] = []
        for token in arg_specs:
            if token in ('f64*', 'ptr', 'i32*'):
                c_args.append(ctypes.c_void_p)
            elif token in ('f64', 'double'):
                c_args.append(ctypes.c_double)
            elif token in ('i32', 'int') or token.startswith(('len:', 'size:')):
                c_args.append(ctypes.c_int)
            else:
                raise MachineError('C core %s: unsupported arg descriptor %r' % (name, token))
        self._out_specs: tuple[tuple[tuple[int, ...], Any], ...] = ()
        if returns in (None, 'none') or (isinstance(returns, str) and returns.startswith(('arg:', 'inplace:'))):
            self.cfn.restype = None
        elif returns in ('f64', 'double'):
            self.cfn.restype = ctypes.c_double
        elif returns in ('i32', 'int'):
            self.cfn.restype = ctypes.c_int
        elif isinstance(returns, dict) and 'shape' in returns:
            self._out_specs = ((tuple(int(d) for d in returns['shape']), np.dtype(returns.get('dtype', 'float64'))),)
            c_args.append(ctypes.c_void_p)
            self.cfn.restype = None
        elif isinstance(returns, (list, tuple)):
            self._out_specs = tuple((tuple(int(d) for d in r['shape']), np.dtype(r.get('dtype', 'float64')))
                                    for r in returns)
            c_args.extend([ctypes.c_void_p] * len(self._out_specs))
            self.cfn.restype = None
        else:
            raise MachineError('C core %s: unsupported return descriptor %r' % (name, returns))
        self.cfn.argtypes = c_args

    def __call__(self, *args):
        import numpy as np
        c_vals, keep_alive, writeback = [], [], []
        pos = 0
        for token in self.arg_specs:
            if token.startswith('len:'):
                c_vals.append(int(len(args[int(token.split(':', 1)[1])])))
                continue
            if token.startswith('size:'):
                c_vals.append(int(np.asarray(args[int(token.split(':', 1)[1])]).size))
                continue
            val = args[pos]
            pos += 1
            if token in ('f64*', 'ptr', 'i32*'):
                dt = np.int32 if token == 'i32*' else np.float64
                arr = val if (isinstance(val, np.ndarray) and val.dtype == dt
                              and val.flags.c_contiguous) else np.ascontiguousarray(val, dtype=dt)
                if arr is not val and isinstance(val, np.ndarray) and val.flags.writeable:
                    writeback.append((val, arr))
                keep_alive.append(arr)
                c_vals.append(arr.ctypes.data)
            elif token in ('f64', 'double'):
                c_vals.append(float(val))
            else:
                c_vals.append(int(val))
        outs = [np.empty(shape, dtype=dt) for shape, dt in self._out_specs]
        for out in outs:
            c_vals.append(out.ctypes.data)
        res = self.cfn(*c_vals)
        for orig, copy_arr in writeback:
            np.copyto(orig, copy_arr)
        if self._out_specs:
            return outs[0] if len(outs) == 1 else tuple(outs)
        if isinstance(self.returns, str) and self.returns.startswith(('arg:', 'inplace:')):
            return args[int(self.returns.split(':', 1)[1])]
        if self.returns in ('f64', 'double'):
            return float(res)
        if self.returns in ('i32', 'int'):
            return int(res)
        return None


def _compile_c_core(row: dict[str, Any]) -> dict[str, Any]:
    """Compile a C11 core specification via `tools.cores.build.cached` with full host SIMD flags."""
    from tools.cores import build, wbc_sources
    name = str(row['name'])
    safe = _safe_ident(name)
    source = str(row['source'])
    if not source.isascii():
        raise MachineError('C core %s: source must be pure ASCII' % name)
    entry = str(row.get('entry', safe))
    arg_specs = tuple(str(a) for a in row.get('args', ()))
    returns = row.get('returns', 'none')
    os.makedirs(build.OUT, exist_ok=True)
    digest = hashlib.sha1(source.encode('utf-8')).hexdigest()[:12]
    src_path = os.path.join(build.OUT, 'hs_%s_%s.c' % (safe, digest))
    if not os.path.exists(src_path):
        tmp_path = '%s.%d.tmp' % (src_path, os.getpid())
        with open(tmp_path, 'w', encoding='utf-8') as f:
            f.write(source)
        os.replace(tmp_path, src_path)
    try:
        lib_path, warnings = build.cached([src_path], wbc_sources.INCLUDES, 'hs_%s_%s' % (safe, digest),
                                          extra=wbc_sources.FLAGS)
    except RuntimeError as exc:
        raise MachineError('C core %s failed compilation: %s' % (name, exc)) from exc
    if row.get('strict_warnings', False) and warnings:
        raise MachineError('C core %s emitted compiler warnings: %s' % (name, '; '.join(warnings)))
    core = CompiledCCore(name, row.get('hook'), lib_path, entry, arg_specs, returns, warnings, row)
    record = {'type': 'core', 'kind': 'c', 'name': name, 'hook': row.get('hook'),
              'callable': core, 'lib_path': lib_path, 'warnings': warnings, 'compiled_c': True, 'spec': dict(row)}
    _CORE_CACHE[name] = record
    return record


def _eval_python_fn(name: str, source: str, entry: str):
    """Evaluate isolated Python kernel source and extract callable `entry`."""
    import numpy as np
    if not source.isascii():
        raise MachineError('Python core %s: source must be pure ASCII' % name)
    ns: dict[str, Any] = {'np': np, 'math': math, 'libm': libm}
    try:
        exec(compile(source, '<hotswap_%s>' % _safe_ident(name), 'exec'), ns)
    except Exception as exc:
        raise MachineError('Python core %s syntax/eval error: %s' % (name, exc)) from exc
    fn = ns.get(entry)
    if not callable(fn):
        raise MachineError('Python core %s: callable %r missing in source' % (name, entry))
    return fn


def _clone_one(val):
    """Clone NumPy arrays, dicts, lists, and tuples while preserving opaque C handles."""
    import numpy as np
    if isinstance(val, np.ndarray):
        return val.copy()
    if isinstance(val, dict):
        return {k: _clone_one(v) for k, v in val.items()}
    if isinstance(val, (list, tuple)):
        return type(val)(_clone_one(v) for v in val)
    return val


def _clone_args(args):
    """Deep-copy NumPy arrays, dicts, and lists in `args` so dry-run probes never mutate `g`."""
    return tuple(_clone_one(a) for a in args)


def _hook_probe_args(g, hook: str):
    """Build isolated dry-run arguments from live `g` buffers for standard `motion.HOOKS`."""
    import numpy as np
    from machine.law import modal
    from machine.senses import edge
    dt, sits, body = 0.001 * float(g['every']), g['situations'], g['body']
    probes = {
        'released': lambda: (g['release'], g['weighed_w'], g['a'], g['distal'], g['distal_side']),
        'outside': lambda: (g['grip'], sits['grips']),
        'sloped': lambda: (g['grip'], sits['grips']),
        'bounds': lambda: (g['s']['v'][body.act], float(g['sense'][edge.LINK][0])),
        'pitched': lambda: (g['y'], g['fr']),
        'sured': lambda: (g['a'], g['posture'], np.zeros_like(g['posture']), g['brace'], 0.8, 0.2, 0.0, 0.0),
        'weighed': lambda: (g['observer'], g['told'], float(g['fr'][modal.WEIGHT]), dt),
        'modes': lambda: (g['y'], g['fr'], g['out']),
        'modal_sensed': lambda: (g['s'], list(g['loads']), g['y']),
        'latent': lambda: (g['st'], g['tab'], float(g['t']), dt, g['out']),
        'couple': lambda: (g['s']['q'][g['joints']], g['s']['v'][body.act],
                           g['ref_q'], g['ref_v'], g['eso'][2], g['out']),
        'impedance': lambda: (g['out'], g['y'], g['fr'], g['a']),
        'surety_field': lambda: (float(g['surety']), dt, math.nan, float(g['t'])),
        'poses': lambda: (g['a'], g['y'], g['fr'], g['posture'], g['bend'], g['arms'],
                          g['reaches'], g['folds'], body.knees, g['weighed_w'], 0.0),
        'gripped': lambda: (g['out'], float(g.get('held', 0.8)), g['a'][modal.STANCE],
                            sits['caps'], g['into'], sits['floors'], float(sits['period']), g['eased'], dt),
        'readings': lambda: (g['y'], g['fr'], g['a'], g['grip'],
                             g['sense'][edge.TOUCH].reshape(-1, 3), float(g['fr'][modal.WEIGHT])),
        'read': lambda: (sits, np.zeros(len(sits['names']))),
    }
    fn = probes.get(hook)
    return _clone_args(fn()) if fn is not None else None


def _bind_python_core(record: dict[str, Any], g=None, probe_args=None):
    """Bind or compile a Python core to native C11 via `bodied` when target metadata is known."""
    from tools.cores import bodied
    row = record['spec']
    fn = record['raw_fn']
    digest = hashlib.sha1(str(row.get('source', '')).encode('utf-8')).hexdigest()[:10]
    cname = 'hs_%s_%s' % (_safe_ident(record['name']), digest)
    hook = record['hook']
    if not row.get('compile_c', True):
        record['callable'], record['compiled_c'] = fn, False
        return record
    bk = (g or {}).get('bank') or {}
    orig = bk.get('stock_cores', {}).get(hook, (g or {}).get(hook)) if hook else None
    part: Any = None
    if 'inputs' in row:
        inputs = tuple(int(i) for i in row['inputs'])
        if row.get('shapes', False):
            part = bodied.on_shapes(fn, cname, inputs, body=row.get('body'))
        elif row.get('body') is not None:
            part = bodied.on_body(fn, cname, inputs, body=int(row['body']))
        else:
            part = bodied.on_args(fn, cname, inputs)
    elif isinstance(orig, bodied.Compiled):
        part = bodied.Compiled(fn, cname, orig.inputs, orig.body, orig.fresh, orig.seed, orig.sources)
    elif isinstance(orig, bodied.Shaped):
        _fn, _nm, inputs, body, fresh, seed, sources = orig.at
        part = bodied.on_shapes(fn, cname, inputs, fresh, seed, sources, body)
    if part is None:
        if row.get('require_c', False):
            raise MachineError('Python core %s requires `inputs` or a `bodied` hook to compile to C' % record['name'])
        record['callable'], record['compiled_c'] = fn, False
        return record
    if probe_args is not None:
        require_c = bool(row.get('require_c', True))
        try:
            part.wait(*_clone_args(probe_args))
            record['compiled_c'] = True
        except Exception as exc:
            if require_c:
                raise MachineError('Python core %s failed C trace/parity hold: %s' % (record['name'], exc)) from exc
            record['compiled_c'] = False
    record['callable'] = part
    return record


def _compile_python_core(row: dict[str, Any], g=None) -> dict[str, Any]:
    """Compile a Python/NumPy core specification and optionally trace/build C11 immediately."""
    name, hook = str(row['name']), row.get('hook')
    entry = str(row.get('entry') or hook or name)
    fn = _eval_python_fn(name, str(row['source']), entry)
    record: dict[str, Any] = {
        'type': 'core', 'kind': 'python', 'name': name, 'hook': hook, 'raw_fn': fn,
        'callable': fn, 'lib_path': None, 'warnings': [], 'compiled_c': False, 'spec': dict(row),
    }
    probe = row.get('sample_args')
    if probe is None and g is not None and hook:
        probe = _hook_probe_args(g, hook)
    if g is not None or 'inputs' in row:
        _bind_python_core(record, g=g, probe_args=probe)
    _CORE_CACHE[name] = record
    return record


def normalize_specs(source: Any) -> list[dict[str, Any]]:
    """Normalize a JSON file path, JSON string, dict, or list into ordered unit spec dicts."""
    if isinstance(source, str):
        stripped = source.strip()
        if stripped.startswith(('{', '[')):
            raw = json.loads(stripped)
        else:
            with open(source, encoding='utf-8') as f:
                raw = json.load(f)
    else:
        raw = source
    if isinstance(raw, Engram):
        return [{'type': 'engram', 'engram': raw}]
    if isinstance(raw, dict):
        if 'items' in raw or 'cores' in raw or 'engrams' in raw:
            out: list[dict[str, Any]] = []
            for item in raw.get('items', ()):
                out.extend(normalize_specs(item))
            for core in raw.get('cores', ()):
                out.extend(normalize_specs(dict(core, type='core')))
            for eng in raw.get('engrams', ()):
                out.append({'type': 'engram', 'engram': eng} if isinstance(eng, Engram) else dict(eng, type='engram'))
            return out
        utype = raw.get('type') or ('engram' if 'slot' in raw else 'core')
        return [dict(raw, type=utype)]
    if isinstance(raw, (list, tuple)):
        out = []
        for item in raw:
            out.extend(normalize_specs(item))
        return out
    raise MachineError('unsupported hotswap spec %r' % (type(source).__name__,))


def compile_unit(spec: Any, g=None) -> dict[str, Any]:
    """Compile one LLM-generated core or engram spec and return its structured build record."""
    items = normalize_specs(spec)
    if len(items) != 1:
        raise MachineError('compile_unit expects exactly 1 unit, got %d' % len(items))
    row = items[0]
    t0 = time.perf_counter()
    if row['type'] == 'engram':
        eng = row['engram'] if isinstance(row.get('engram'), Engram) else _engram_from_row(row)
        engrams.register([eng])
        try:
            payload = engrams.compile_engram(eng)
        except Exception as exc:
            raise MachineError('engram %s failed compilation: %s' % (eng.name, exc)) from exc
        return {
            'type': 'engram', 'kind': eng.kind, 'name': eng.name, 'slot': eng.slot,
            'keep': str(row.get('keep', '')), 'engram': eng, 'payload': payload,
            'compiled_c': eng.kind == 'sensed', 'warnings': [],
            'compile_ms': round(1e3 * (time.perf_counter() - t0), 3),
        }
    kind = str(row.get('kind', 'c')).lower()
    if kind == 'c':
        rec = _compile_c_core(row)
    elif kind == 'python':
        rec = _compile_python_core(row, g=g)
    else:
        raise MachineError('unknown core kind %r for %s' % (kind, row.get('name')))
    rec['compile_ms'] = round(1e3 * (time.perf_counter() - t0), 3)
    return rec


def compile_batch(specs: Any, g=None) -> list[dict[str, Any]]:
    """Compile a batch of cores and/or engrams, returning compiled records in order."""
    return [compile_unit(item, g=g) for item in normalize_specs(specs)]


def _check_finite_match(want, got, label: str):
    """Verify `got` matches `want` in type/shape and contains no newly introduced NaN/Inf values."""
    import numpy as np
    if want is None:
        if got is not None:
            raise MachineError('%s: expected None return, got %s' % (label, type(got).__name__))
        return
    if isinstance(want, (float, int, np.floating, np.integer)):
        if not isinstance(got, (float, int, np.floating, np.integer)):
            raise MachineError('%s: expected scalar return, got %s' % (label, type(got).__name__))
        if math.isfinite(float(want)) and not math.isfinite(float(got)):
            raise MachineError('%s: non-finite scalar output %r' % (label, got))
        return
    if isinstance(want, np.ndarray):
        if not isinstance(got, np.ndarray) or got.shape != want.shape:
            raise MachineError('%s: expected ndarray%s, got %s' % (
                label, want.shape, getattr(got, 'shape', type(got).__name__)))
        bad = ~np.isfinite(got) & np.isfinite(want)
        if np.any(bad):
            raise MachineError('%s: output array contains %d non-finite entries' % (label, int(np.sum(bad))))
        return
    if isinstance(want, tuple):
        if not isinstance(got, tuple) or len(got) != len(want):
            raise MachineError('%s: expected %d-tuple return' % (label, len(want)))
        for idx, (w_i, g_i) in enumerate(zip(want, got)):
            _check_finite_match(w_i, g_i, '%s[%d]' % (label, idx))
        return
    if isinstance(want, dict):
        if not isinstance(got, dict) or set(want) != set(got):
            raise MachineError('%s: dict keys mismatch' % label)
        for key in want:
            _check_finite_match(want[key], got[key], '%s.%s' % (label, key))


def _ensure_core_bank(g):
    """Ensure `g['bank']` holds `stock_cores` and `active_cores` dicts alongside engram state."""
    bk = engrams._ensure_bank(g)
    bk.setdefault('stock_cores', {})
    bk.setdefault('active_cores', {})
    return bk


def _validate_and_swap_core(g, rec: dict[str, Any]) -> dict[str, Any]:
    """Dry-run validate compiled core `rec` on copies of `g` buffers, then swap onto `g[hook]`."""
    import numpy as np
    hook = rec.get('hook')
    if not hook:
        raise MachineError('core %s has no `hook` target to swap onto `g`' % rec['name'])
    if hook not in g:
        raise MachineError('core %s targets unknown hook %r on `g`' % (rec['name'], hook))
    bk = _ensure_core_bank(g)
    stock_fn = bk['stock_cores'].get(hook, g[hook])
    probe = rec['spec'].get('sample_args')
    if probe is None:
        probe = _hook_probe_args(g, hook)
    if rec['kind'] == 'python':
        _bind_python_core(rec, g=g, probe_args=probe)
    cand = rec['callable']
    if probe is not None:
        ref_args, cand_args = _clone_args(probe), _clone_args(probe)
        want = stock_fn(*ref_args)
        try:
            got = cand(*cand_args)
        except Exception as exc:
            raise MachineError('core %s dry-run failed on %s: %s' % (rec['name'], hook, exc)) from exc
        _check_finite_match(want, got, '%s(%s)' % (rec['name'], hook))
        for idx, (r_a, c_a) in enumerate(zip(ref_args, cand_args)):
            if isinstance(r_a, np.ndarray) and isinstance(c_a, np.ndarray):
                _check_finite_match(r_a, c_a, '%s(%s).arg[%d]' % (rec['name'], hook, idx))
    if hook not in bk['stock_cores']:
        bk['stock_cores'][hook] = g[hook]
    g[hook] = cand
    bk['active_cores'][hook] = rec['name']
    from tools.cores import prefetch
    prefetch.refresh(g)
    return {
        'type': 'core', 'name': rec['name'], 'hook': hook, 'kind': rec['kind'],
        'compiled_c': bool(rec.get('compiled_c')), 'warnings': list(rec.get('warnings', ())),
    }
