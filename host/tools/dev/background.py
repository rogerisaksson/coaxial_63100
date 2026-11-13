"""A batch below the pages: below-normal priority, off the cores a page keeps.

This process and every process it starts, held so on Windows by a job: Renode raises itself above
normal and, inheriting below normal, ran there over the page.

A terminal page the user runs keeps the CPU it wants while suites and searches run on what is
left. That on a desk, where its user is; a CI runner (`runner`) is nobody's: its batch has the
machine - nothing kept, the priority it was started at, the width that does most work as its run
measures it (`widths`, `machine_detector.host`).

    from tools.dev import background; background.lower()
    background.cores()                       # the physical cores the batch is left
    background.widths()                      # the jobs at once it may run: (28,) here, (2, 4) on a runner
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
#: shared at any priority - eight streaming numpy loads alone took it to 39.3. Of a desk's cores a third at
#: most (`kept`): the batch has two of three; a runner keeps none.
PAGE_CORES = 4
#: What a CI service sets true in the environment of its jobs.
RUNNER = 'CI'
#: A job's limits (JOB_OBJECT_LIMIT_AFFINITY, _PRIORITY_CLASS) and its basic limits' information class.
LIMIT_AFFINITY, LIMIT_PRIORITY, BASIC_LIMITS = 0x10, 0x20, 2


class _Limits(ctypes.Structure):
    """JOBOBJECT_BASIC_LIMIT_INFORMATION."""
    _fields_ = [('user_time', ctypes.c_int64), ('job_time', ctypes.c_int64), ('flags', ctypes.c_uint32),
                ('min_set', ctypes.c_size_t), ('max_set', ctypes.c_size_t), ('processes', ctypes.c_uint32),
                ('affinity', ctypes.c_size_t), ('priority', ctypes.c_uint32), ('scheduling', ctypes.c_uint32)]


def runner(environ=None):
    """Whether this host is a CI runner - nobody's desk, the batch's alone: RUNNER true in the environment."""
    return (os.environ if environ is None else environ).get(RUNNER, '').lower() in ('1', 'true')


def kept(cores=None, on_runner=None):
    """The physical cores the batch leaves the pages: of a desk's a third, PAGE_CORES at most - one of four,
    two of eight, four of twelve or more; of a runner's none. This host's unless told."""
    from tools.dev.focus import physical_cores
    cores = physical_cores() if cores is None else cores
    return 0 if (runner() if on_runner is None else on_runner) else min(PAGE_CORES, cores // 3)


def widths(cores=None, threads=None, on_runner=None):
    """The jobs at once a batch may run here, the narrowest first: a desk's cores less those kept; a
    runner's cores and, where it has more threads than cores, its every thread - which does more work a
    run measures as it starts (`machine_detector.host`). This host's unless told - a container's its own."""
    from tools.dev import focus
    cores = focus.physical_cores() if cores is None else cores
    threads = focus.threads() if threads is None else threads
    on_runner = runner() if on_runner is None else on_runner
    batch = cores - kept(cores, on_runner)
    return (batch, threads) if on_runner and threads > cores else (batch,)


def cores():
    """The physical cores the batch is left (`kept`'s off the machine's)."""
    from tools.dev.focus import physical_cores
    return physical_cores() - kept()


def lower():
    """This process below the pages and off the cores they keep (`kept`): the first's threads, a core's
    numbered beside each other; the processes it starts held to both. A runner's batch is left as it was
    started: nobody's pages are there."""
    from tools.dev.focus import physical_cores
    if runner():
        return
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
