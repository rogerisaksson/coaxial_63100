"""A portable C core built with the host's compiler into a shared library."""
import glob
import hashlib
import os
import platform
import re
import subprocess
import time
from contextlib import suppress
from shutil import which

from tools import REPO

OUT = os.path.join(REPO, 'build', 'hosttest')

# The warnings the firmware build puts on the portable cores.
FLAGS = ['-std=c11', '-O1', '-Wall', '-Wextra', '-Wconversion', '-Wshadow']

#: A world run here: -O3; on x86-64 SSE4.1's roundss for floorf and nearbyintf, exact; on Windows
#: ucrt's fminf and fmaxf inline (world_libm.h). The stand-in's drive period 448 -> 277 ns, its
#: state the same bits at -O0 to -O3; her twenty native boards 2.03 -> 1.81 CPU s a wall s.
FAST = (['-O3', '-fno-math-errno']
        + (['-msse4.1'] if platform.machine().lower() in ('amd64', 'x86_64') else [])
        + (['-include', os.path.join(REPO, 'world', 'inc', 'world_libm.h')] if os.name == 'nt' else []))

#: With COAXIAL_GCOV set (tools/dev/cover.py), built for gcov: unoptimised, so a
#: line is a line, the counts beside the library.
GCOV = ['--coverage', '-O0']


def find_cc():
    """A host C compiler, or None. PATH first, then where winget puts one."""
    found = which('gcc') or which('clang') or which('cc')
    if found:
        return found
    packages = os.path.join(os.environ.get('LOCALAPPDATA', ''),
                            'Microsoft', 'WinGet', 'Packages')
    if not os.path.isdir(packages):
        return None
    for root, _dirs, files in os.walk(packages):
        if 'gcc.exe' in files and root.endswith(os.path.join('mingw64', 'bin')):
            return os.path.join(root, 'gcc.exe')
    return None


def mujoco_headers():
    """('-isystem', the directory MuJoCo's headers are included from), beside its package - wbc_world.c reads its data
    through them -, found without importing it; () where it is not installed."""
    import importlib.util
    spec = importlib.util.find_spec('mujoco')
    roots = list(spec.submodule_search_locations or ()) if spec is not None else []
    found = sorted(h for r in roots for h in glob.glob(os.path.join(r, 'include', '**', 'mujoco.h'), recursive=True))
    return ('-isystem', os.path.dirname(os.path.dirname(found[0]))) if found else ()


def build(cc, sources, includes, name, extra=()):
    """(path, warnings) for a shared library, built fresh every run; `extra` flags after the
    firmware's."""
    os.makedirs(OUT, exist_ok=True)
    lib = os.path.join(OUT, name + ('.dll' if os.name == 'nt' else '.so'))
    flags = [] if os.name == 'nt' else ['-fPIC', '-pthread']    # a .so on Linux; mingw warns on it
    for path in includes:
        flags += ['-I', path]
    if os.environ.get('COAXIAL_GCOV'):
        flags += GCOV
    done = subprocess.run([cc, '-shared', '-o', lib] + FLAGS + list(extra) +
                          sources + flags,
                          capture_output=True, text=True, encoding='utf-8',
                          errors='replace')
    if done.returncode != 0:
        raise RuntimeError(done.stderr.strip()[-2000:])
    return lib, [l for l in (done.stderr or '').splitlines() if 'warning:' in l]


def native():
    """The compiler's flags for this host's own instruction sets, nothing contracted: -march=native on x86,
    -mcpu=native on ARM (clang's -march takes no native there), none on another processor; -ffp-contract=off with
    them - gcc's ISO C contracts nothing, clang's does within an expression, and a fused multiply-add is not
    numpy's bits."""
    flag = {'amd64': '-march=native', 'x86_64': '-march=native', 'arm64': '-mcpu=native',
            'aarch64': '-mcpu=native'}.get(platform.machine().lower())
    return ('-ffp-contract=off',) + ((flag,) if flag else ())


def _host():
    """This host's processor by the instruction sets it has, as numpy probes them: a library built for them
    (`native`) is no other processor's, its digest this too."""
    import numpy as np
    sets = getattr(getattr(getattr(np, '_core', None), '_multiarray_umath', None), '__cpu_features__', {})
    return ','.join(sorted(k for k, on in sets.items() if on))


def headers(includes):
    """Every header in `includes`, their bytes in their names' order: the last part of a library's digest
    (`cached`)."""
    out = []
    for path in sorted(h for d in includes for h in glob.glob(os.path.join(d, '*.h'))):
        with open(path, 'rb') as f:
            out.append(f.read())
    return b''.join(out)


def cached(sources, includes, name, extra=(), heads=None):
    """(path, warnings) of a library built once for these sources', their headers' and the
    flags' bytes, shared by every process that asks - a stand-in opened in each of a gate's
    suites built it once -, a compiler sought only where none is built yet; built by one process
    at a time, the others waiting for it (`_claimed`). `heads` its headers' bytes where a round of
    builds read them once (`headers`): read for each of her law's 33 parts, 23 files a part in a
    thread beside her loop, they held her builds 0.63 s of her first second."""
    digest = hashlib.sha1(' '.join(FLAGS + list(extra)).encode())
    digest.update(str(bool(os.environ.get('COAXIAL_GCOV'))).encode())
    digest.update((_host() if any(flag.endswith('=native') for flag in extra) else '').encode())
    for path in sources:
        with open(path, 'rb') as f:
            digest.update(f.read())
    digest.update(headers(includes) if heads is None else heads)
    lib = os.path.join(OUT, '%s_%s%s' % (name, digest.hexdigest()[:12],
                                          '.dll' if os.name == 'nt' else '.so'))
    if os.path.exists(lib) or not _claimed(lib):
        return lib, []
    try:
        cc = find_cc()
        if cc is None:
            raise RuntimeError('%s is C built on this host, and there is no C compiler here '
                               '(setup.ps1 installs one)' % name)
        made, warnings = build(cc, sources, includes, '%s_%d' % (name, os.getpid()), extra)
        try:
            os.replace(made, lib)
        except OSError:
            return made, warnings           # another process's is there, loaded
        _pruned(lib, name)
        return lib, warnings
    finally:
        _unmarked(lib + '.building')


def _pruned(lib, name):
    """`name`'s other builds gone once `lib` is laid: each change of its sources built one and kept it - 11 369 over
    1757 cores, 1.2 GB, one core 1015 times -, and a build its process could not lay (`cached`) after it went; one a
    process holds stays, as Windows removes no open file."""
    from tools.dev.focus import _alive
    own = re.compile(re.escape(name) + r'_(?:([0-9a-f]{12})|(\d+))\.(?:dll|so)$')
    for other in glob.glob(os.path.join(OUT, glob.escape(name) + '_*')):
        built = own.match(os.path.basename(other))
        if other != lib and built and (built.group(1) or not _alive(int(built.group(2)))):
            with suppress(OSError):
                os.remove(other)


def _unmarked(marker):
    """`marker` gone, or left where a waiting process holds it open to read its pid: Windows
    removes no open file (WinError 32 crashed a suite as its library came). One left beside its
    library is read by none; one left by a failed build goes with its process (`_claimed`)."""
    with suppress(OSError):
        os.remove(marker)


def _claimed(lib):
    """Whether this process is to build `lib`: its marker made here; else another's built it -
    waited for while that process lives - and False."""
    from tools.dev.focus import _alive
    os.makedirs(OUT, exist_ok=True)
    marker = lib + '.building'
    while True:
        try:
            fd = os.open(marker, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
        except FileExistsError:
            try:
                with open(marker, encoding='utf-8') as f:
                    pid = int(f.read() or 0)
            except (OSError, ValueError):
                pid = 0
            if os.path.exists(lib):
                return False
            if pid and not _alive(pid):
                _unmarked(marker)
            time.sleep(0.2)
            continue
        with os.fdopen(fd, 'w', encoding='utf-8') as f:
            f.write(str(os.getpid()))
        if os.path.exists(lib):
            _unmarked(marker)
            return False
        return True


# setup.ps1's probe, before host/ is installed: winget's gcc is on PATH in new shells only.
if __name__ == '__main__':
    print(find_cc() or '')
