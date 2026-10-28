"""mppi/ through ctypes on the CPU, through NVRTC on a CUDA card: one source for both.

    mppi.library().mppi_plane_costs(...)      # the CPU's, a library a process
    mppi.kernel(cupy)(grid, block, args)      # the card's, a thread a rollout

The firmware builds none of it (CMakeLists.txt); `machine.aircraft.se3mpc` drives it.
"""
import ctypes
import functools
import os

from tools import REPO
from tools.cores.aero import BUILT
from tools.cores.build import cached

SOURCES = [os.path.join(REPO, 'mppi', 'src', 'mppi_plane.c')]
INCLUDES = [os.path.join(REPO, 'mppi', 'inc')]
#: mppi_plane_costs's loop on the card. The source's text goes to NVRTC whole: cupy keys its
#: cache on the text, so an `#include` of the file would run a stale build after an edit.
GRID = r'''
extern "C" __global__ void mppi_plane_grid(const double *x0, const double *u, const double *ref,
    const double *k, const double *c, int nc, int ns, const double *r, int nr, const double *weight,
    const double *u0, int n, int h, double dt, double w_p, double w_v, const double *wind, double *cost)
{
    const int i = (int)(blockDim.x * blockIdx.x + threadIdx.x);
    if (i < n)
        cost[i] = mppi_plane_rollout(x0, u + (long)i * h * 4, ref, k, c, nc, ns, r, nr, weight, u0, h, dt,
                                     w_p, w_v, wind);
}
'''


@functools.lru_cache(maxsize=1)
def library():
    """mppi_plane.c built for this host's CPU, as the aero core is built."""
    lib = ctypes.CDLL(cached(SOURCES, INCLUDES, 'mppi', list(BUILT))[0])
    d, i, f = ctypes.POINTER(ctypes.c_double), ctypes.c_int, ctypes.c_double
    lib.mppi_plane_costs.argtypes = [d, d, d, d, d, i, i, d, i, d, d, i, i, f, f, f, d, d]
    lib.mppi_plane_costs.restype = None
    return lib


def kernel(cupy):
    """mppi_plane.c built for `cupy`'s card: its rollout a __device__ function under `GRID`."""
    text = ['#define MPPI_FN __device__\n']
    for path in (os.path.join(INCLUDES[0], 'mppi.h'), SOURCES[0]):
        with open(path, encoding='utf-8') as f:
            text.append(f.read())
    return cupy.RawKernel(''.join(text) + GRID, 'mppi_plane_grid')
