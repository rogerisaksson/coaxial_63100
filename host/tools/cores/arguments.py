"""A compiled part's arguments as its tracer reads them and its hold copies them (`bodied`).

    got = arguments.tree_in(prog, k, (), v, specs, True)   # argument k as traced: an extern, a dict lazy, else pinned
    arguments.rebound(prog, got, specs)                    # a dict entry rebound written back into the extern it was
    ours = arguments.copy_in(v, rng, True)                 # each array an extern copied, or drawn afresh from rng
    arrays = arguments.leaves_in(trees)                    # the arrays `copy_in` copies, in order
"""
import numpy as np

from tools.cores import trace


def tree_in(prog, k, path, v, specs, top):
    """Argument `k`'s `v` at `path` as traced: an array or a float (or at the top a number) an extern
    over a copy of it in C's order, as `bodied.Bound.leaf` lays every array its C reads - traced in its own order, an
    F-ordered array's views were baked transposed and read wrong, held apart and left numpy's (the robot's gate's
    leave, ten or eleven points under it as it lay, 93 us a law pass); a dict lazy (`Lazy`), a list of arrays its
    own of each, its length pinned; anything else itself, pinned (`specs`)."""
    if isinstance(v, dict):
        return Lazy(prog, k, path, v, specs)
    if isinstance(v, (list, tuple)) and any(isinstance(x, (dict, list, tuple, np.ndarray)) for x in v):
        specs.append(('length', k, path, len(v)))
        return type(v)(tree_in(prog, k, path + (i,), x, specs, False) for i, x in enumerate(v))
    if ((isinstance(v, np.ndarray) and v.dtype.kind in 'fb') or isinstance(v, (float, np.floating))
            or (top and np.asarray(v).dtype.kind in 'fbiu')):
        name = 'a%d%s' % (k, ''.join('/%s' % p for p in path))
        traced = prog.extern(name, np.array(v, dtype=float, order='C'))
        specs.append(('array', k, path, name, traced))
        return traced
    specs.append(('pin', k, path, v))
    return v


class Lazy(dict):

    """A dict argument as traced: an entry made what `tree_in` makes it as the code reads it, what
    it finds there or not pinned; read whole, every entry made and its keys pinned."""

    def __init__(self, prog, k, path, raw, specs):
        super().__init__()
        self.at = (prog, k, path, raw, specs)

    def __getitem__(self, key):
        if not dict.__contains__(self, key):
            prog, k, path, raw, specs = self.at
            dict.__setitem__(self, key, tree_in(prog, k, path + (key,), raw[key], specs, False))
        return dict.__getitem__(self, key)

    def __contains__(self, key):
        _prog, k, path, raw, specs = self.at
        there = dict.__contains__(self, key) or key in raw
        specs.append(('has', k, path, key, there))
        return there

    def get(self, key, default=None):
        return self[key] if key in self else default

    def _whole(self):
        _prog, k, path, raw, specs = self.at
        specs.append(('keys', k, path, tuple(raw)))
        for key in raw:
            self[key]

    def keys(self):
        self._whole()
        return dict.keys(self)

    def values(self):
        self._whole()
        return dict.values(self)

    def items(self):
        self._whole()
        return dict.items(self)

    def __iter__(self):
        self._whole()
        return dict.__iter__(self)

    def __len__(self):
        self._whole()
        return dict.__len__(self)

    def copy(self):
        self._whole()
        return dict(dict.items(self))


def rebound(prog, args, specs):
    """A dict entry fn rebound to an array of its own written back into the extern it was."""
    for spec in [s for s in specs if s[0] == 'array' and s[2]]:
        _kind, k, path, name, extern = spec
        node = args[k]
        for p in path[:-1]:
            node = node[p]
        now = dict.__getitem__(node, path[-1]) if isinstance(node, Lazy) else node[path[-1]]
        if now is extern:
            continue
        if not isinstance(now, trace.Traced) or now.shape != extern.shape:
            raise trace.Untraced('%s rebound to a %s' % (name, type(now).__name__))
        prog.add('ew', extern.ref(), ('positive', ('ref', now.ref())))


def copy_in(v, rng, top):
    """`v` with each array `tree_in` makes an extern copied (or drawn afresh from `rng`), the rest itself."""
    if isinstance(v, dict):
        return {key: copy_in(x, rng, False) for key, x in v.items()}
    if isinstance(v, (list, tuple)) and any(isinstance(x, (dict, list, tuple, np.ndarray)) for x in v):
        return type(v)(copy_in(x, rng, False) for x in v)
    if ((isinstance(v, np.ndarray) and v.dtype.kind in 'fb') or isinstance(v, (float, np.floating))
            or (top and np.asarray(v).dtype.kind in 'fbiu')):
        a = np.array(v)
        if rng is not None:
            a = rng.normal(size=a.shape) > 0.0 if a.dtype == bool else rng.normal(size=a.shape).astype(a.dtype)
        return a if isinstance(v, np.ndarray) or a.ndim else type(v)(a[()]) if isinstance(v, float) else a[()]
    return v


def leaves_in(trees):
    """The arrays `copy_in` copies, in order."""
    out = []
    for v in trees:
        if isinstance(v, dict):
            out += leaves_in(list(v.values()))
        elif isinstance(v, (list, tuple)) and any(isinstance(x, (dict, list, tuple, np.ndarray)) for x in v):
            out += leaves_in(list(v))
        elif isinstance(v, np.ndarray):
            out.append(v)
    return out
