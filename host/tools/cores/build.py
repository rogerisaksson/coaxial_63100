"""A portable C core built with the host's compiler into a shared library."""
import os
import subprocess
from shutil import which

from tools import REPO

OUT = os.path.join(REPO, 'build', 'hosttest')

# The warnings the firmware build puts on the portable cores.
FLAGS = ['-std=c11', '-O1', '-Wall', '-Wextra', '-Wconversion', '-Wshadow']

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


def build(cc, sources, includes, name, extra=()):
    """(path, warnings) for a shared library, built fresh every run; `extra` flags after the
    firmware's."""
    os.makedirs(OUT, exist_ok=True)
    lib = os.path.join(OUT, name + ('.dll' if os.name == 'nt' else '.so'))
    flags = [] if os.name == 'nt' else ['-fPIC']    # a .so on Linux; mingw warns on it
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


# setup.ps1's probe, before host/ is installed: winget's gcc is on PATH in new shells only.
if __name__ == '__main__':
    print(find_cc() or '')
