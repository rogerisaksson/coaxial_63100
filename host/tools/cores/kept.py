"""The robot's compiled parts held once, taken by a part laid anew in this process or a later one.

    kept.warm(part)                                 # a part laid (`bodied.Compiled`): its kind's libraries kept loading
    kept.store(part, prog, info, specs, lib, cfn)   # a part held: its program and its build, here and on disk
    got = kept.taken(part, args, Bound)             # (program, info, specs, its Bound on `args`), None where none fits

Each laying of the robot's world made its law's parts anew, and each traced and held its program again: 33
traces and 31 holds, 0.23 s of the first second of every errand after the first, and of each process's first. A
program held fits another part of its function, its name, its inputs and its body where that part's arguments pass
its checks (`bodied.Bound.fits`); the robot's body's constants of the same bits (`same`) are taken as another
laying's body's own objects, that body bound anew. Kept on disk beside the libraries (KEPT) under the stamp of
every source a trace reads and the host it was held on (`stamp`): one of them changed, none is taken; a nested
function's, never - its cells were its process's.
"""
import contextlib
import copy
import functools
import hashlib
import os
import pickle
import sys

import numpy as np

from tools import REPO
from tools.cores import build

#: Where held programs are kept between processes: a file a kind, beside the libraries they are built in.
KEPT = os.path.join(build.OUT, 'kept')
#: The folders whose sources a trace reads - the robot's machine, these cores, the texts its constants are of.
SOURCES = tuple(os.path.join(REPO, 'host', *part) for part in (('machine',), ('tools', 'cores'), ('lisp',)))
#: What a kept file may fail with as it is read - torn, of another numpy's or another code's - read as none.
TORN = (OSError, EOFError, pickle.UnpicklingError, AttributeError, ImportError, TypeError, ValueError, KeyError,
        IndexError)
#: The parts held in this process, by function, name, inputs and body: [(program, info, specs, library, function)],
#: a kept one's library still loading [(program, info, specs, its build's job)].
_HELD: dict = {}
#: The kinds whose kept file was read in this process.
_READ: set = set()
#: What `taken` tells a part not waiting while a kept library of its kind still loads: it asks again at its next call,
#: numpy's meanwhile - waited for, the robot's loop stood 6 to 22 ms at a part laid anew on a shape it met first (its
#: legs' leave at each new count of what bears it, seated), a library its build had lost a compile's whole.
LOADING = object()


def _key(part):
    return part.fn, part.name, tuple(part.inputs), part.body


@functools.cache
def stamp():
    """Every source a trace reads - its path, size and time written -, its build (`COAXIAL_ROBOT`) and the host it is
    held on: its Python, numpy, MuJoCo and the instruction sets its libraries are built for (`build._host`), one
    digest; once a process."""
    from importlib import metadata
    from machine.body import mechatronics
    out = hashlib.sha1()
    for root in SOURCES:
        _stamped(root, out)
    out.update(('%s %s %s %s %s' % (sys.version, np.__version__, metadata.version('mujoco'), build._host(),
                                    mechatronics.ROBOT)).encode())
    return out.hexdigest()


def _stamped(at, out):
    """Folder `at`'s sources stamped into `out` by their directory entries' own sizes and times - a stat of each
    path its own was 131 ms of the robot's laying."""
    try:
        with os.scandir(at) as listed:
            found = sorted(listed, key=lambda e: e.name)
    except OSError:
        return
    for e in found:
        if e.is_dir(follow_symlinks=False):
            if e.name != '__pycache__':
                _stamped(e.path, out)
        elif e.name.endswith(('.py', '.lisp', '.c', '.h', '.toml')):
            st = e.stat(follow_symlinks=False)
            out.update(('%s %d %d\n' % (e.path[len(REPO):], st.st_size, st.st_mtime_ns)).encode())


def _file(part):
    """The kept file of `part`'s kind and build, None for a nested function's or a lambda's."""
    from machine.body import mechatronics
    fn = part.fn
    if '<' in getattr(fn, '__qualname__', '<'):
        return None
    kind = '%s.%s|%s|%s|%s|%s' % (fn.__module__, fn.__qualname__, part.name, tuple(part.inputs), part.body is not None,
                                  mechatronics.ROBOT)
    return os.path.join(KEPT, hashlib.sha1(kind.encode()).hexdigest()[:20] + '.pkl')


def _kept(path):
    """The programs kept in `path` under this stamp, [] where none or another's."""
    try:
        with open(path, 'rb') as f:
            got = pickle.load(f)
    except TORN:
        return []
    return list(got['programs']) if isinstance(got, dict) and got.get('stamp') == stamp() else []


def warm(part):
    """`part`'s kind's kept programs, not yet read here, read and their libraries loading in the builds' thread
    behind the robot's laying: by its first pass they are in memory."""
    from tools.cores import held
    key, path = _key(part), _file(part)
    if path is None or key in _READ or _HELD.get(key):
        return
    _READ.add(key)
    for prog, info, specs, source, matmul in _kept(path):
        _HELD.setdefault(key, []).append((prog, info, specs, held.building(source, part.name, matmul, info['solves'])))


def store(part, prog, info, specs, lib, cfn):
    """Part `part`'s program and its build held: here, and on disk under this stamp - written whole, then put in
    place, as another process may write it too; a source kept already, not again."""
    from tools.cores import emit, held
    _HELD.setdefault(_key(part), []).append((prog, info, specs, lib, cfn))
    path = _file(part)
    if path is None:
        return
    source, programs = emit.c(prog, part.name)[0], _kept(path)
    if any(p[3] == source for p in programs):
        return
    programs.append((prog, info, specs, source, held.matmuls(prog)))
    whole = '%s.%d' % (path, os.getpid())
    try:
        os.makedirs(KEPT, exist_ok=True)
        with open(whole, 'wb') as f:
            pickle.dump({'stamp': stamp(), 'programs': programs}, f, protocol=pickle.HIGHEST_PROTOCOL)
        os.replace(whole, path)
    except (OSError, TypeError, AttributeError, pickle.PicklingError):
        # a constant pickle cannot carry (a module the robot's body keeps): held in this process alone
        with contextlib.suppress(OSError):
            os.remove(whole)


def taken(part, args, bound, waited=True):
    """(program, info, specs, a `bound` - `bodied.Bound` - of its build on `args`) of a program held for `part`'s
    kind that `args` fit - its body's constants rebound to `args`' body where they are of the same bits, a kept one's
    library waited for where it still loads, or LOADING where not `waited` -; None where none fits."""
    warm(part)
    held_here = _HELD.get(_key(part), [])
    if not waited and any(len(entry) == 4 and not entry[3]['done'].is_set() for entry in held_here):
        return LOADING
    for k, entry in enumerate(held_here):
        if len(entry) == 4:
            prog, info, specs, job = entry
            job['done'].wait()
            if 'why' in job:
                continue
            entry = held_here[k] = (prog, info, specs) + tuple(job['lib'])
        prog, info, specs, lib, cfn = entry
        if part.body is not None:
            b = args[part.body]
            if not all(same(getattr(b, name, None), v) for name, v in prog.consts.items()):
                continue
            prog = copy.copy(prog)
            prog.consts = {name: getattr(b, name) for name in prog.consts}
        c = bound(lib, cfn, prog, info, specs, part.body, part.sources)
        if part.body is not None:
            try:
                c.bind(args[part.body])
            except TypeError:
                # a body of other shapes (`trace.Untraced`: a 46-dof body's mass matrix, the program a 47's): not it
                continue
        if c.fits(args) is not None:
            return prog, info, specs, c
    return None


def same(a, b):
    """Whether two constants of a trace are the same: one object, arrays of one type, shape and bits, numbers or
    words of one type and value, sequences of one type whose items are."""
    if a is b:
        return True
    if isinstance(a, np.ndarray) and isinstance(b, np.ndarray):
        return a.dtype == b.dtype and a.shape == b.shape and a.tobytes() == b.tobytes()
    if type(a) is not type(b):
        return False
    if isinstance(a, (list, tuple)):
        return len(a) == len(b) and all(same(x, y) for x, y in zip(a, b))
    return isinstance(a, (int, float, str, bool)) and a == b
