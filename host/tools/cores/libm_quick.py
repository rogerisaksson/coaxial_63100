"""The robot's own libm as an extension of this interpreter (libm_quick.c): a float or an array a call.

    q = libm_quick.module()            # the extension, or None where it cannot be built here
    q.sin(0.5), q.sin(a)               # wbc_libm's bits: a float inlined, an array laid its run - a few a call each
    q.sincos(a), q.arctan2(y, x)       # sin and cos of one reduction; two floats or two arrays of a shape
    q.laid(other)                      # other(name, *args): Python's way with anything else it is given

A call of Python's math is 0.09 us and numpy's on an array of three 0.5; the robot's libm through ctypes was 7.7 us a
call and its numpy twin 81 - neither a way to call it from the robot's loop. Here a float is a call inlined in C
(wbc_sin, ...) and an array of float64 in C's order its run in wbc_libm.c, as cheap as math's and numpy's; what is
neither - an int, a numpy scalar, an array strided or of another type, two arguments to broadcast - goes to the
callable `laid` holds (`machine.libm`'s: laid as C takes it, then back here). Built once with the host's compiler
for its instruction sets, nothing contracted, as the robot's cores are (`build.native`); where none can be built its
callers keep the numpy twin.
"""
import functools
import importlib.machinery
import importlib.util
import os
import sys
import sysconfig

import numpy as np

from tools import REPO
from tools.cores import build, quick

#: The extension's sources: its calls beside this, the arrays' runs the robot's cores' own.
SOURCES = (os.path.join(os.path.dirname(os.path.abspath(__file__)), 'libm_quick.c'),
           os.path.join(REPO, 'wbc', 'src', 'wbc_libm.c'))
#: The functions it has: of one argument, sin and cos together, of two.
ONE = ('sin', 'cos', 'tan', 'exp', 'expm1', 'log', 'arctan', 'arcsin', 'arccos', 'tanh')
PAIR, TWO = ('sincos',), ('arctan2', 'hypot')


@functools.lru_cache(maxsize=None)
def module():
    """The extension, built (numpy's version in its flags: a build its headers') and loaded once a process; None
    where it cannot be built here."""
    try:
        extra = ('-O2', *build.native(), '-DLIBM_QUICK_NUMPY=%s' % np.__version__, *quick.interpreter())
        includes = [sysconfig.get_paths()['include'], np.get_include(), os.path.join(REPO, 'wbc', 'inc')]
        path = build.cached(list(SOURCES), includes, 'libm_quick', extra)[0]
        loader = importlib.machinery.ExtensionFileLoader('_libm_quick', path)
        spec = importlib.util.spec_from_loader('_libm_quick', loader)
        if spec is None:
            raise ImportError('no spec for its build')
        made = importlib.util.module_from_spec(spec)
        loader.exec_module(made)
        return made
    except (RuntimeError, OSError, ImportError, AttributeError) as e:
        print('libm stays its numpy twin: %s' % str(e)[:300], file=sys.stderr)
        return None
