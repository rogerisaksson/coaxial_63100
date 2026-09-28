"""A batch below the pages: this process and every process it starts at below-normal priority.

A terminal page the user runs keeps the CPU it wants while suites and searches run on what is
left (2026-09-28).

    from tools.dev import background; background.lower()
"""
import os
import sys

#: Windows' BELOW_NORMAL_PRIORITY_CLASS: a child process takes its parent's when it is this or
#: idle, so a pool's workers and their buses follow; POSIX's niceness for the same.
BELOW_NORMAL, NICE = 0x4000, 10


def lower():
    """This process below the pages; the processes it starts inherit it."""
    if sys.platform == 'win32':
        import ctypes
        kernel = getattr(ctypes, 'windll').kernel32
        # The pseudo-handle is a pointer's width: taken as ctypes' default int it came out 32 bits
        # wide and the call failed.
        kernel.GetCurrentProcess.restype = ctypes.c_void_p
        kernel.SetPriorityClass.argtypes = (ctypes.c_void_p, ctypes.c_uint32)
        kernel.SetPriorityClass(kernel.GetCurrentProcess(), BELOW_NORMAL)
    else:
        os.nice(NICE)
