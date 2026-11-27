"""A pure function's results kept pickled by their key in memory and under `.cache/compiled/texts`."""
import hashlib
import os
import pickle
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
CACHE = os.path.join(REPO, '.cache', 'compiled', 'texts')
_FILE_DIGESTS: dict[str, tuple[tuple[int, int], bytes]] = {}


def loaded(kept, key, fn, size=8):
    """`fn(key)` as `kept` ({key: its pickle}, the latest `size`, the latest last) holds it, else
    made and kept: each call its own copy - a pickle's load -; a result of no pickle made each call."""
    blob = kept.pop(key, None)
    if blob is None:
        got = fn(key)
        try:
            kept[key] = pickle.dumps(got, pickle.HIGHEST_PROTOCOL)
        except (pickle.PicklingError, TypeError, AttributeError):
            pass
        while len(kept) > size:
            del kept[next(iter(kept))]
        return got
    kept[key] = blob
    return pickle.loads(blob)


def _file_digest(path: str) -> bytes:
    """Return SHA-256 digest of `path` cached in-process by `(st_mtime_ns, st_size)`."""
    ap = os.path.abspath(path)
    st = os.stat(ap)
    sig = (st.st_mtime_ns, st.st_size)
    hit = _FILE_DIGESTS.get(ap)
    if hit is None or hit[0] != sig:
        with open(ap, 'rb') as f:
            _FILE_DIGESTS[ap] = (sig, hashlib.sha256(f.read()).digest())
    return _FILE_DIGESTS[ap][1]


def _atomic_write(prefix: str, target: str, data: bytes) -> None:
    os.makedirs(CACHE, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=CACHE, prefix=prefix + '.', suffix='.tmp')
    try:
        with os.fdopen(fd, 'wb') as f:
            f.write(data)
        os.replace(tmp, target)
    except OSError:
        try:
            os.unlink(tmp)
        except OSError:
            pass


def _prune_stats(prefix: str, limit: int = 32) -> None:
    entries = []
    try:
        with os.scandir(CACHE) as it:
            for e in it:
                if e.name.startswith(prefix) and e.name.endswith('.idx'):
                    try:
                        entries.append((e.stat().st_mtime_ns, e.path))
                    except OSError:
                        continue
    except OSError:
        return
    for _mt, p in sorted(entries)[:-limit]:
        try:
            os.unlink(p)
        except OSError:
            pass


def disk(kept, tag: str, key, files, extra, fn, size: int = 8):
    """`loaded` backed by `.cache/compiled/texts/<tag>_<sha256>.pkl` keyed by `files` and `extra`."""
    blob = kept.pop(key, None)
    if blob is not None:
        kept[key] = blob
        return pickle.loads(blob)
    meta = (sys.version_info[:3], pickle.HIGHEST_PROTOCOL, extra)
    try:
        paths = sorted({os.path.abspath(f) for f in files if f})
        stats = [(p, st.st_mtime_ns, getattr(st, 'st_ctime_ns', 0), st.st_size)
                 for p in paths for st in (os.stat(p),)]
    except OSError:
        return loaded(kept, key, fn, size)
    stat_sig = hashlib.sha256(repr((meta, stats)).encode('utf-8')).hexdigest()[:16]
    idx_prefix = tag + '_stat_'
    idx_path = os.path.join(CACHE, '%s%s.idx' % (idx_prefix, stat_sig))
    digest_hex = None
    if os.path.isfile(idx_path):
        try:
            with open(idx_path, 'rb') as f:
                raw = f.read().decode('ascii').strip()
            if len(raw) == 16:
                digest_hex = raw
        except (OSError, UnicodeDecodeError):
            digest_hex = None
    wrote_idx = False
    if digest_hex is None:
        h = hashlib.sha256()
        h.update(repr(meta).encode('utf-8'))
        try:
            for p in paths:
                h.update(os.path.basename(p).encode('utf-8') + b'\0' + _file_digest(p))
        except OSError:
            return loaded(kept, key, fn, size)
        digest_hex = h.hexdigest()[:16]
        wrote_idx = True
    target = os.path.join(CACHE, '%s_%s.pkl' % (tag, digest_hex))
    if os.path.isfile(target):
        try:
            with open(target, 'rb') as f:
                blob = f.read()
            got = pickle.loads(blob)
            kept[key] = blob
            while len(kept) > size:
                del kept[next(iter(kept))]
            if wrote_idx:
                try:
                    _atomic_write(idx_prefix, idx_path, digest_hex.encode('ascii'))
                    _prune_stats(idx_prefix)
                except OSError:
                    pass
            return got
        except (OSError, EOFError, pickle.UnpicklingError, AttributeError, ValueError, ImportError):
            blob = None
    got = fn(key)
    try:
        blob = pickle.dumps(got, pickle.HIGHEST_PROTOCOL)
    except (pickle.PicklingError, TypeError, AttributeError):
        return got
    kept[key] = blob
    while len(kept) > size:
        del kept[next(iter(kept))]
    try:
        _atomic_write(tag, target, blob)
        _atomic_write(idx_prefix, idx_path, digest_hex.encode('ascii'))
        _prune_stats(idx_prefix)
    except OSError:
        pass
    return got


_ENTRIES: dict = {}


def gym_entries(name: str, default_name: str, compile_fn, extra_files=()):
    """Return cached `[(label, wire, names)]` for `name` from memory or `.cache/compiled/texts`."""
    from machine.body import figure, gait, mechatronics
    from machine.moves import clocked
    from machine.texts import acts, curves, dsp, kernel, maneuver, ortho, patch
    extras = tuple(extra_files)
    key = (name, mechatronics.ROBOT, figure.SCALE, gait.SIMILAR, extras)
    hit = _ENTRIES.get(key)
    if hit is not None:
        return list(pickle.loads(hit))
    lisp_path = os.path.join(os.path.dirname(ortho._FILE), name + '.lisp')
    try:
        inc = tuple(ortho.sources(lisp_path))
    except (SyntaxError, OSError):
        return list(compile_fn(name))
    motion_lisp = os.path.join(mechatronics.HUMANOID, 'motion.lisp')
    extra = (tuple(dsp.SUITE) + tuple(dsp.KERNEL) + ((motion_lisp,) if os.path.isfile(motion_lisp) else ())
             + (dsp.__file__, kernel.__file__, patch.__file__,
                maneuver.__file__, acts.__file__, curves.__file__, clocked.__file__)
             if name == default_name else ())
    files = inc + extra + extras + mechatronics.body_sources(mechatronics.ROBOT) + (ortho.__file__,)
    return list(disk(_ENTRIES, 'entries', key, files, key, lambda k: compile_fn(k[0])))
