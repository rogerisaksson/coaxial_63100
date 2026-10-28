"""A batch below the pages: below-normal priority, off the cores a page keeps.

This process and every process it starts, held so on Windows by a job: Renode raises itself above
normal and, inheriting below normal, ran there over the page.

A terminal page the user runs keeps the CPU it wants while suites and searches run on what is
left.

    from tools.dev import background; background.lower()
    background.cores()                       # the physical cores the batch is left
"""
import ctypes
import os
import sys

#: Windows' BELOW_NORMAL_PRIORITY_CLASS: a child process takes its parent's when it is this or
#: idle, so a pool's workers and their buses follow; POSIX's niceness for the same.
BELOW_NORMAL, NICE = 0x4000, 10
#: A page's own processes - its draw, its drawing ahead (`terminal.ui.ahead`), her worker, her camera -,
#: a physical core each the batch leaves whole, both its threads: under the offline gate below normal the
#: HUMANOID page's frame took 41.3 ms to its 7.3 alone, 28.3 with these kept; the rest is the memory bus,
#: shared at any priority - eight streaming numpy loads alone took it to 39.3. Kept only where the host has
#: twice those besides (CI's runner has 2).
PAGE_CORES = 4
#: A job's limits (JOB_OBJECT_LIMIT_AFFINITY, _PRIORITY_CLASS) and its basic limits' information class.
LIMIT_AFFINITY, LIMIT_PRIORITY, BASIC_LIMITS = 0x10, 0x20, 2


class _Limits(ctypes.Structure):
    """JOBOBJECT_BASIC_LIMIT_INFORMATION."""
    _fields_ = [('user_time', ctypes.c_int64), ('job_time', ctypes.c_int64), ('flags', ctypes.c_uint32),
                ('min_set', ctypes.c_size_t), ('max_set', ctypes.c_size_t), ('processes', ctypes.c_uint32),
                ('affinity', ctypes.c_size_t), ('priority', ctypes.c_uint32), ('scheduling', ctypes.c_uint32)]


def kept():
    """The physical cores the batch leaves the pages: PAGE_CORES on a host of three times those, else none."""
    from tools.dev.focus import physical_cores
    return PAGE_CORES if physical_cores() >= 3 * PAGE_CORES else 0


def cores():
    """The physical cores the batch is left (`kept`'s off the machine's)."""
    from tools.dev.focus import physical_cores
    return physical_cores() - kept()


def lower():
    """This process below the pages and off the cores they keep (`kept`): the first's threads, a core's
    numbered beside each other; the processes it starts held to both."""
    from tools.dev.focus import physical_cores
    threads = os.cpu_count() or 1
    spare = kept() * max(1, threads // physical_cores())
    if sys.platform == 'win32':
        kernel = getattr(ctypes, 'windll').kernel32
        # The pseudo-handle is a pointer's width: taken as ctypes' default int it came out 32 bits
        # wide and the call failed.
        kernel.GetCurrentProcess.restype = kernel.CreateJobObjectW.restype = ctypes.c_void_p
        kernel.SetPriorityClass.argtypes = (ctypes.c_void_p, ctypes.c_uint32)
        kernel.SetInformationJobObject.argtypes = (ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p, ctypes.c_uint32)
        kernel.AssignProcessToJobObject.argtypes = (ctypes.c_void_p, ctypes.c_void_p)
        kernel.SetPriorityClass(kernel.GetCurrentProcess(), BELOW_NORMAL)
        limits = _Limits(flags=LIMIT_PRIORITY | (LIMIT_AFFINITY if spare else 0), priority=BELOW_NORMAL,
                         affinity=((1 << threads) - 1) & ~((1 << spare) - 1))
        job = kernel.CreateJobObjectW(None, None)
        kernel.SetInformationJobObject(job, BASIC_LIMITS, ctypes.byref(limits), ctypes.sizeof(limits))
        kernel.AssignProcessToJobObject(job, kernel.GetCurrentProcess())
    else:
        os.nice(NICE)
        if spare:
            os.sched_setaffinity(0, set(range(spare, threads)))
