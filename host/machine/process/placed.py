"""Her processes placed a core apart: one hardware thread of each physical core, the first left out.

Two of her processes on one core's two threads ran at about half pace each: her vision apart on her law's
sibling thread slowed her law's pass 0.41 ms, on a core of its own 0.04 ms (her Threadripper's 32 cores).
Her worker, her vision, planner and camera's map, her page's drawing and a craft's flight each place itself
at its start; the first core is the system's interrupts'. Windows says what a core is; elsewhere nothing.
Placed under a benchmark's relay (BENCH set, `tools.dev.focus.relay`), each also runs above normal priority: her
pace measured beside a peer's suites flickered 8-17 % between runs of the same tree.

    placed.placed()      # this process: True where it now runs on one thread of each core but the first
    placed.raised()      # this process above normal priority where BENCH is set: True where it now does
    placed.cores()       # [each physical core's logical processors, a mask] where Windows tells them, else []
"""
import ctypes
import os
import subprocess
import sys

#: GetLogicalProcessorInformation's relation of a physical core's logical processors.
_CORE = 0
#: Set (by a benchmark's relay, inherited by every process of hers it starts), her processes run above normal priority:
#: Windows' ABOVE_NORMAL class - over the host's other loads, short of HIGH, which a baton a core could starve the
#: desktop with.
BENCH = 'COAXIAL_BENCH'


class _Info(ctypes.Structure):
    _fields_ = [('mask', ctypes.c_size_t), ('relation', ctypes.c_int), ('union', ctypes.c_ulonglong * 2)]


def cores():
    """Each physical core's logical processors as a mask (Windows' GetLogicalProcessorInformation), in its
    processor group; [] on another system or where it is refused."""
    if sys.platform != 'win32':
        return []
    from ctypes import wintypes
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    size = wintypes.DWORD(0)
    kernel.GetLogicalProcessorInformation(None, ctypes.byref(size))
    infos = (_Info * (size.value // ctypes.sizeof(_Info)))()
    if not kernel.GetLogicalProcessorInformation(infos, ctypes.byref(size)):
        return []
    return [info.mask for info in infos if info.relation == _CORE]


def raised():
    """This process above normal priority where BENCH is set (a benchmark's relay): her measurements less at the
    mercy of the host's other loads. False where unset or on another system; results untouched - only how soon."""
    if not os.environ.get(BENCH) or sys.platform != 'win32':
        return False
    from ctypes import wintypes
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    kernel.GetCurrentProcess.restype = wintypes.HANDLE
    kernel.SetPriorityClass.argtypes = [wintypes.HANDLE, wintypes.DWORD]
    return bool(kernel.SetPriorityClass(kernel.GetCurrentProcess(), getattr(subprocess, 'ABOVE_NORMAL_PRIORITY_CLASS')))


def placed():
    """This process on the lowest logical processor of each physical core but the first: none of her processes
    on another's sibling thread, the scheduler free among the rest - and above normal priority where a benchmark
    asks (`raised`). False where it cannot be placed (one core, another system); results are untouched either way -
    only where her processes run."""
    raised()
    masks = sorted(cores())
    if len(masks) < 2:
        return False
    from ctypes import wintypes
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    kernel.GetCurrentProcess.restype = wintypes.HANDLE
    kernel.SetProcessAffinityMask.argtypes = [wintypes.HANDLE, ctypes.c_size_t]
    return bool(kernel.SetProcessAffinityMask(kernel.GetCurrentProcess(), sum(m & -m for m in masks[1:])))
