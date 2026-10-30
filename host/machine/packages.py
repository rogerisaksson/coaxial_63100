"""The ortho-DSP's lisp modules on disk: a folder's files, each after those its package uses."""
import glob
import os

from machine import mechatronics, ortho, patch
from machine.errors import MachineError

#: The packages of the language itself a module may use.
LANGUAGE = (':cl',)


def ordered(paths):
    """`paths`, each module after the modules its package uses (`defpackage`'s :use)."""
    found, out = {}, []
    for path in paths:
        with open(path, encoding='utf-8') as f:
            head = next(form for form in patch.forms(f.read()) if form[0] == 'defpackage')
        found[head[1]] = (path, [u for clause in head[2:] if clause[0] == ':use' for u in clause[1:] if u not in LANGUAGE])

    def after(name, seen=()):
        if name in seen or name not in found:
            raise MachineError('%s: %s' % (name, 'a package using itself' if name in seen else 'no module of the suite'))
        path, uses = found[name]
        for used in uses:
            after(used, seen + (name,))
        out.extend([path] * (path not in out))

    for name in sorted(found):
        after(name)
    return out


def modules(folder):
    """`folder`'s modules under her lisp (`mechatronics.HUMANOID`), each after those it uses."""
    return ordered(sorted(ortho.key(p) for p in glob.glob(os.path.join(mechatronics.HUMANOID, folder, '*.lisp'))))
