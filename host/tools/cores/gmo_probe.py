"""The robot's observer's C (wbc_gmo.c) timed function by function, in a build of its own.

gmo_probe.c beside this includes the observer's sources (`OBSERVER`) whole and unchanged, built apart from the
robot's library with -finstrument-functions over them alone - their calls out to the chain's dynamics, a Jacobian,
the split's solves and a Givens rotation's hypot wrapped there.

    probe = gmo_probe.Probe()              # built once for its sources' bytes, typed, its clock's rate read
    probe.step(o, base, q, u, dt)          # wbh_gmo_step's step on an observer (`gmo.Gmo`): the same bits
    calls, own, inclusive = probe.read()   # each function's calls and seconds since the last read

A timed call's hooks are measured (`Probe.hooks`): what a call costs its caller is taken off the caller's own
seconds, what it costs within itself off the call's, so a function's seconds are its code's.
"""
import ctypes
import functools
import glob
import os
import time

import numpy as np

from tools import REPO
from tools.cores import build, gmo
from tools.cores.model import built
from tools.cores.wbc_sources import FLAGS, INCLUDES, SOURCES

#: The probe's source; the observer's it includes.
SOURCE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'gmo_probe.c')
OBSERVER = tuple(os.path.join(REPO, 'wbc', 'src', name) for name in ('wbc_gmo.c', 'wbc_gmo_factor.c'))
#: wbc_gmo.c's functions too small to time - a call's hooks would cost more than the call -, their seconds their
#: callers'.
UNTIMED = ('turned', 'dot', 'clip01', 'passed', 'along', 'pushed', 'closest', 'own', 'added', 'lessened',
           'wbc_gmo_added', 'wbc_gmo_lessened')
#: The rows probe_gmo_read lays: each slot's calls, ticks with its callees, ticks alone, calls it made, calls beneath.
ROWS = 5
#: The empty calls the hooks are measured on.
EMPTIES = 200000


def _untimed_files(sources, includes):
    """The files whose functions the probe's build leaves untimed, by name - gcc matches a part of a function's file's
    path: every source but the probe, every header of the robot's and the compiler's own ('include')."""
    names = {os.path.basename(p) for p in sources if os.path.normcase(p) != os.path.normcase(SOURCE)}
    names |= {os.path.basename(h) for d in includes for h in glob.glob(os.path.join(d, '*.h'))}
    return sorted(names) + ['include']


@functools.cache
def library():
    """The probe built once for its sources', wbc_gmo.c's and the headers' bytes (`build.cached`), loaded: the
    robot's library's sources and flags (`tools.cores.wbc.library`), gmo_probe.c in wbc_gmo.c's place."""
    included = {os.path.normcase(p) for p in OBSERVER}
    sources = [SOURCE] + [p for p in built(SOURCES) if os.path.normcase(p) not in included]
    includes = list(INCLUDES) + [os.path.dirname(OBSERVER[0])]
    extra = list(FLAGS) + ['-finstrument-functions',
                           '-finstrument-functions-exclude-file-list=' + ','.join(_untimed_files(sources, includes)),
                           '-finstrument-functions-exclude-function-list=' + ','.join(UNTIMED)]
    heads = build.headers(build.beside(sources, includes))
    for path in OBSERVER:
        with open(path, 'rb') as f:
            heads += f.read()
    path, _warnings = build.cached(sources, includes, 'gmo_probe', extra, heads)
    return ctypes.CDLL(path)


class Probe:

    """The probe typed (`library`), its slots named, its clock's ticks a second and its hooks' cost measured."""

    def __init__(self):
        lib = self.lib = library()
        lib.probe_gmo_size.restype = ctypes.c_int
        if lib.probe_gmo_size() != ctypes.sizeof(gmo.Gmo):
            raise ValueError('the probe lays wbc_gmo_t in %d bytes, its mirror %d' % (lib.probe_gmo_size(),
                                                                                     ctypes.sizeof(gmo.Gmo)))
        names = (ctypes.c_char_p * 64)()
        lib.probe_gmo_fns.argtypes, lib.probe_gmo_fns.restype = [ctypes.c_void_p], ctypes.c_int
        self.names = [names[k].decode() for k in range(lib.probe_gmo_fns(ctypes.addressof(names)))]
        self._out = np.zeros((ROWS, len(self.names)), np.int64)
        lib.probe_gmo_read.argtypes, lib.probe_gmo_read.restype = [ctypes.c_void_p], None
        lib.probe_gmo_step.argtypes = [ctypes.POINTER(gmo.Gmo)] + [ctypes.c_void_p] * 3 + [ctypes.c_double]
        lib.probe_gmo_step.restype = None
        lib.probe_gmo_now.restype = ctypes.c_ulonglong
        lib.probe_gmo_hooks.argtypes, lib.probe_gmo_hooks.restype = [ctypes.c_int], None
        self.rate = self._rate()
        self.caller, self.within = self.hooks()

    def _rate(self, wait=0.25):
        """The probe's clock's ticks a second, against this process's."""
        n0, t0 = self.lib.probe_gmo_now(), time.perf_counter()
        time.sleep(wait)
        n1, t1 = self.lib.probe_gmo_now(), time.perf_counter()
        return (n1 - n0) / (t1 - t0)

    def _raw(self):
        self.lib.probe_gmo_read(self._out.ctypes.data)
        return self._out.copy()

    def hooks(self, n=EMPTIES):
        """(ticks a timed call's hooks cost its caller, ticks they cost within the call): the least of five rounds
        of `n` empty calls beneath one."""
        empty, empties = self.names.index('probe_empty'), self.names.index('probe_empties')
        rounds = []
        for _ in range(5):
            self._raw()
            self.lib.probe_gmo_hooks(n)
            raw = self._raw()
            rounds.append((raw[2, empties] / n, raw[1, empty] / n))
        return min(r[0] for r in rounds), min(r[1] for r in rounds)

    def step(self, o, base, q, u, dt):
        """wbh_gmo_step's step on the observer `o` (`gmo.Gmo`), at the robot's frame `base` (12 doubles), hinges
        `q` and velocities `u` (their addresses), `dt` s on."""
        self.lib.probe_gmo_step(o, base, q, u, dt)

    def read(self):
        """(calls, own s, inclusive s) of each slot (`names`) since the last read, the hooks' cost taken off."""
        calls, inclusive, own, kids, below = self._raw().astype(float)
        own = (own - self.caller * kids - self.within * calls) / self.rate
        inclusive = (inclusive - (self.caller + self.within) * below - self.within * calls) / self.rate
        return calls, own, inclusive
