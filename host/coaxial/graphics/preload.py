"""The preload: decimates, the outline's exact index and loops, and the stereotypes, in one pickle.

Built once by a process behind the front page and kept under the user's
local application data, so a view opens in a tenth of a second instead of
parsing the STL and decimating it six times.
"""
import hashlib
import os
import pickle
import shutil
import sys
import time

from ..memory import physical

#: Free memory under which the background build is not started, and
#: free disk under which the pickle is not written.
RAM_FLOOR = 1 << 30
DISK_FLOOR = 100 << 20
FILE = 'preload.pickle'


def cache_dir():
    """The preload's directory: under LOCALAPPDATA on Windows, under
    ~/.cache elsewhere - never in the tree."""
    base = os.environ.get('LOCALAPPDATA') or os.path.join(
        os.path.expanduser('~'), '.cache')
    return os.path.join(base, 'coaxial_63100', 'preload')


def stamp(path):
    """What a pickle must match to be trusted: the model's absolute path,
    size and mtime, and the first twelve hex digits of a hash of the
    mesh, solids, creases and stereotype sources - a decimate or a fit
    that changed in the code makes the file stale.
    """
    from . import creases, mesh, solids, stereotype
    digest = hashlib.sha1()
    for module in (mesh, solids, creases, stereotype):
        with open(module.__file__, 'rb') as source:
            digest.update(source.read())
    st = os.stat(path)
    return (os.path.abspath(path), st.st_size, int(st.st_mtime),
            digest.hexdigest()[:12])


def room(where=None):
    """(free memory, free disk) in bytes for the cache directory, None
    where unknown."""
    where = where or cache_dir()
    probe = where
    while not os.path.isdir(probe) and os.path.dirname(probe) != probe:
        probe = os.path.dirname(probe)
    try:
        disk = shutil.disk_usage(probe).free
    except OSError:
        disk = None
    return physical()[1], disk


def refusal(where=None):
    """Why the preload will not build here, in words - or None when the
    machine has the room."""
    ram, disk = room(where)
    if ram is not None and ram < RAM_FLOOR:
        return 'skipped: %.1f gb of memory free, under the %.0f gb floor' % (
            ram / 2**30, RAM_FLOOR / 2**30)
    if disk is not None and disk < DISK_FLOOR:
        return 'skipped: %.0f mb of disk free, under the %.0f mb floor' % (
            disk / 2**20, DISK_FLOOR / 2**20)
    return None


def build(path, progress=None, where=None):
    """Decimate, index and fit `path`, write the pickle atomically, and
    return its size in bytes.
    """
    from . import creases, solids, stereotype
    say = progress or (lambda label: None)
    lods = {}
    for _zoom, divisions in solids.LODS:
        lods[divisions] = solids._decimated(path, divisions)
        say('decimate grid %d: %d triangles' % (divisions, len(lods[divisions][1]) // 3))
    exact, loops = creases._outline_source()
    say('outline edges: %d loops' % len(loops))
    prims = stereotype._stereotypes()
    say('stereotypes: %d primitives' % len(prims))
    bundle = {'stamp': stamp(path), 'lods': lods, 'exact': (exact, loops),
              'prims': prims, 'built': time.time()}
    return save(bundle, where)


def save(bundle, where=None):
    """Write `bundle` as the pickle, through a temporary name so a reader
    never sees half a file.
    """
    where = where or cache_dir()
    os.makedirs(where, exist_ok=True)
    final = os.path.join(where, FILE)
    tmp = final + '.%d.tmp' % os.getpid()
    with open(tmp, 'wb') as out:
        pickle.dump(bundle, out, protocol=pickle.HIGHEST_PROTOCOL)
    os.replace(tmp, final)
    return os.path.getsize(final)


def load(path, where=None):
    """The bundle for `path` when the pickle is there and its stamp
    matches, else None - never an exception: a torn or foreign file is
    just no preload."""
    final = os.path.join(where or cache_dir(), FILE)
    try:
        with open(final, 'rb') as source:
            bundle = pickle.load(source)
        if bundle.get('stamp') != stamp(path):
            return None
        return bundle
    except (OSError, EOFError, pickle.UnpicklingError, AttributeError,
            ValueError, TypeError, KeyError):
        return None


def main(argv=None):
    """Build the preload for the board's model, a line per step on
    stdout for the front page to print; exit 0 either way."""
    from .. import orientation
    path = orientation.MODEL
    why = refusal()
    if why is not None:
        print(why, flush=True)
        return 0
    size = build(path, progress=lambda label: print(label, flush=True))
    print('written: %.1f mb' % (size / 2**20), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
