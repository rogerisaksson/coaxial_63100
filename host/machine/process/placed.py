"""Her processes placed by her host's own topology: a core apart, her loop's on a cache of its own.

Where each of her processes runs, never what it tells; read off the host at each start, nothing of any one machine
written here. Windows tells its cores, their classes and their caches (GetLogicalProcessorInformationEx), Linux its
sysfs, on any processor of theirs; macOS binds no thread to a core, there her loop's thread is told the user's own
(its quality of service: its fast cores, not throttled); another system is left as it is. A host of two cores or fewer,
of one cache for all its cores, of a cache a core, of fast and slow cores or of several memory nodes is placed by the
same lines.

A core apart: each process on one logical processor of each physical core but the first (the system's
interrupts'). Two of her processes on one core's two threads ran at about half pace each: her vision apart on her
law's sibling thread slowed her law's pass 0.41 ms, on a core of its own 0.04 ms.
Her loop's on a cache of its own (`placed(True)`): the cores that share a last-level cache are a group; her
loop's process takes one for itself among her loops on the host (`_claim`) - one of its fastest cores' where they
differ -, its thread held to the group's first fast core where the system holds a thread apart from its process
(Windows), the thread of her C's beside it (`steps.ahead`) on its neighbours; her other processes leave every group
so taken, or its first core alone where that would leave them none (`left`). A group lies within one memory node
and memory is the node's that first touches it, so her loop's is its own. Moved among all her cores her loop ran
each at a cold cache, her camera's map beside it on its cache: on a host of 32 cores, four to a cache, her bed
errand's pace 1.28 -> 1.70, her door's 1.68 -> 1.94, a fifth apart between runs before and steady after.
Not throttled: Windows' power throttling of a process whose window is not the user's is told off for each.
Above normal priority under a benchmark's relay (BENCH set, `tools.dev.focus.relay`): her pace measured beside a
peer's suites flickered 8-17 % between runs of the same tree.

    placed.placed()      # this process: True where it now runs a core apart, off her loops' caches
    placed.placed(True)  # her loop's: on a cache group of its own where one is free, its thread on its first core
    placed.placed(cache=True)  # her camera's, her vision's: a group of its own, its threads free, kept by later calls
    placed.raised()      # this process above normal priority where BENCH is set: True where it now does
    placed.cores()       # [each physical core's logical processors, a mask] where the host tells them, else []
    placed.groups()      # [each last-level cache's logical processors, a mask], else []
    placed.topology()    # ([(class, a core's mask)], [a last-level cache's mask]) as the host tells them
    placed.now()         # this process's logical processors as the system holds it now, a mask
    placed.chosen(caches, own, free, fast)   # her loop's processors of the group it takes, 0 where none fits
    placed.left(caches, own, taken)          # another process's: `own` off the groups taken
"""
import ctypes
import os
import subprocess
import sys

#: GetLogicalProcessorInformationEx's relations: a physical core's logical processors, a cache's; all of them.
_CORE, _CACHE, _ALL = 0, 2, 0xffff
#: Windows' process information class of its power throttling and its execution speed's bit; its word for a mutex
#: there already; the right to wait on one, enough to open it.
_THROTTLING, _SPEED, _EXISTS, _SYNCHRONIZE = 4, 0x1, 183, 0x00100000
#: macOS' quality of service of the user's own interaction (QOS_CLASS_USER_INTERACTIVE).
_INTERACTIVE = 0x21
#: Linux's processors, as its sysfs tells them.
SYSFS = '/sys/devices/system/cpu'
#: Linux's control groups from a root: a process's cpuset - a container's processors - told in them.
CGROUPS = ('sys', 'fs', 'cgroup')
#: Set (by a benchmark's relay, inherited by every process of hers it starts), her processes run above normal priority:
#: Windows' ABOVE_NORMAL class - over the host's other loads, short of HIGH, which a baton a core could starve the
#: desktop with.
BENCH = 'COAXIAL_BENCH'
#: Her loop's cache group once taken: what holds it while this process lives, its processors of hers.
_MINE: list = []


class _Throttling(ctypes.Structure):
    _fields_ = [('version', ctypes.c_ulong), ('control', ctypes.c_ulong), ('state', ctypes.c_ulong)]


class _Number(ctypes.Structure):
    _fields_ = [('group', ctypes.c_ushort), ('number', ctypes.c_ubyte), ('reserved', ctypes.c_ubyte)]


def _kernel():
    """Windows' kernel32, typed for what is asked of it here."""
    from ctypes import wintypes
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    for fn in (kernel.GetCurrentProcess, kernel.GetCurrentThread, kernel.CreateMutexW, kernel.OpenMutexW):
        fn.restype = wintypes.HANDLE
    kernel.CreateMutexW.argtypes = [ctypes.c_void_p, wintypes.BOOL, wintypes.LPCWSTR]
    kernel.OpenMutexW.argtypes = [wintypes.DWORD, wintypes.BOOL, wintypes.LPCWSTR]
    kernel.CloseHandle.argtypes = [wintypes.HANDLE]
    kernel.SetPriorityClass.argtypes = [wintypes.HANDLE, wintypes.DWORD]
    kernel.SetProcessAffinityMask.argtypes = [wintypes.HANDLE, ctypes.c_size_t]
    kernel.SetThreadAffinityMask.argtypes = [wintypes.HANDLE, ctypes.c_size_t]
    kernel.GetProcessAffinityMask.argtypes = [wintypes.HANDLE, ctypes.c_void_p, ctypes.c_void_p]
    kernel.SetProcessInformation.argtypes = [wintypes.HANDLE, ctypes.c_int, ctypes.c_void_p, wintypes.DWORD]
    return kernel


def _windows():
    """([(class, each physical core's logical processors)], [each last-level cache's]) of Windows'
    GetLogicalProcessorInformationEx in this thread's processor group - a core's class its efficiency class, the
    higher the faster -; ([], []) where it is refused."""
    import struct
    from ctypes import wintypes
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    size, here = wintypes.DWORD(0), _Number()
    try:
        kernel.GetLogicalProcessorInformationEx(_ALL, None, ctypes.byref(size))
        raw = ctypes.create_string_buffer(size.value)
        if not size.value or not kernel.GetLogicalProcessorInformationEx(_ALL, raw, ctypes.byref(size)):
            return [], []
        kernel.GetCurrentProcessorNumberEx(ctypes.byref(here))
    except AttributeError:
        return [], []
    found, caches, at = [], [], 0
    while at < size.value:
        relation, length = struct.unpack_from('<II', raw, at)
        # a core's record: its flags and class, 20 bytes, its count of groups, its group's mask; a cache's: its
        # level first, its group's mask after its 32 bytes
        if relation == _CORE and struct.unpack_from('<H', raw, at + 40)[0] == here.group:
            found.append((raw.raw[at + 9], struct.unpack_from('<Q', raw, at + 32)[0]))
        elif relation == _CACHE and struct.unpack_from('<H', raw, at + 48)[0] == here.group:
            caches.append((raw.raw[at + 8], struct.unpack_from('<Q', raw, at + 40)[0]))
        at += length
    last = max((level for level, _mask in caches), default=0)
    return sorted(found, key=lambda core: core[1]), sorted({mask for level, mask in caches if level == last})


def listed(text):
    """A sysfs list of processors ('0-3,8,10-11') as a mask."""
    mask = 0
    for part in text.strip().split(','):
        lo, _dash, hi = part.partition('-')
        for cpu in range(int(lo), int(hi or lo) + 1):
            mask |= 1 << cpu
    return mask


def _told(*path):
    """A sysfs file's number, 0 where it is not there or tells none."""
    try:
        with open(os.path.join(*path), encoding='ascii') as f:
            return int(f.read())
    except (OSError, ValueError):
        return 0


def sysfs(root=SYSFS):
    """([(class, each physical core's logical processors)], [each last-level cache's]) as Linux's sysfs under `root`
    tells them: a processor's thread siblings, its class its capacity where the kernel tells one (a core of two
    sizes') else its highest clock, and of its caches those of the highest level; ([], []) where it tells none."""
    import glob
    found, caches = {}, set()
    try:
        for cpu in glob.glob(os.path.join(root, 'cpu[0-9]*')):
            with open(os.path.join(cpu, 'topology', 'thread_siblings_list'), encoding='ascii') as f:
                mask = listed(f.read())
            klass = _told(cpu, 'cpu_capacity') or _told(cpu, 'cpufreq', 'cpuinfo_max_freq')
            found[mask] = max(found.get(mask, 0), klass)
            for index in glob.glob(os.path.join(cpu, 'cache', 'index[0-9]*')):
                with open(os.path.join(index, 'shared_cpu_list'), encoding='ascii') as f:
                    caches.add((_told(index, 'level'), listed(f.read())))
    except (OSError, ValueError):
        return [], []
    last = max((level for level, _mask in caches), default=0)
    return sorted((klass, mask) for mask, klass in found.items()), sorted(mask for level, mask in caches if level == last)


def given(root=os.sep):
    """The logical processors this process's control group is given, a mask: Linux's cpuset - a container's (its
    --cpuset-cpus), not its host's - of its group or the nearest above it that tells one, cgroup v2's or v1's, under
    `root`; 0 where none is told (another system, no cpuset)."""
    try:
        with open(os.path.join(root, 'proc', 'self', 'cgroup'), encoding='ascii') as f:
            groups = [line.split(':', 2) for line in f.read().splitlines() if line.count(':') >= 2]
    except OSError:
        return 0
    for _number, controllers, path in groups:
        if controllers and 'cpuset' not in controllers.split(','):
            continue
        base = os.path.join(root, *CGROUPS, *(('cpuset',) if controllers else ()))
        name = 'cpuset.effective_cpus' if controllers else 'cpuset.cpus.effective'
        parts = [part for part in path.split('/') if part]
        for depth in range(len(parts), -1, -1):
            try:
                with open(os.path.join(base, *parts[:depth], name), encoding='ascii') as f:
                    text = f.read().strip()
            except OSError:
                continue
            if text:
                return listed(text)
    return 0


def held(found, caches, mask):
    """(cores, caches) as a host tells them (`topology`) held to the processors `mask` holds - a control group's
    (`given`) -: each by its processors there, those with none there left out; as they are where `mask` is 0."""
    if not mask:
        return found, caches
    return ([(klass, core & mask) for klass, core in found if core & mask],
            sorted({cache & mask for cache in caches if cache & mask}))


def topology():
    """([(class, each physical core's logical processors, a mask)], [each last-level cache's]) as this host tells
    them - Windows, Linux held to the processors its control group is given (`held`: a container's) -; ([], []) on
    another system."""
    if sys.platform == 'win32':
        return _windows()
    return held(*sysfs(), given()) if sys.platform.startswith('linux') else ([], [])


def cores():
    """Each physical core's logical processors as a mask; [] on another system or where it is refused."""
    return [mask for _class, mask in topology()[0]]


def groups():
    """Each last-level cache's logical processors as a mask - the cores that share it -; [] likewise."""
    return topology()[1]


def apart(found):
    """(her processors, the fast of them) of the cores `found` [(class, mask)]: one logical processor of each
    physical core but the first, and those of them on cores of the highest class - all, where every core is one."""
    kept = sorted(found, key=lambda core: core[1])[1:]
    best = max((klass for klass, _mask in kept), default=0)
    return sum(m & -m for _k, m in kept), sum(m & -m for k, m in kept if k == best)


def chosen(caches, own, free, fast=0):
    """Her loop's processors of the cache group it takes: of `caches` (masks) one that `free` (a group -> bool,
    asked once each till one is) says is none's and that holds two of her processors `own` at least - her loop's
    thread and the one beside it -, those with a `fast` processor before the rest, the last first: the first's
    first core is the system's. 0 where none fits: a cache a core, every group taken."""
    for group in sorted(caches, key=lambda g: (bool(g & fast), g), reverse=True):
        mine = group & own
        if mine & (mine - 1) and free(group):
            return mine
    return 0


def left(caches, own, taken):
    """Another process's processors: `own` less every cache group of `caches` that `taken` (a group -> bool) says
    one of her loops holds; less those loops' first cores alone where that leaves none - a host of one cache for all
    its cores -; `own` where that leaves none either."""
    held = [group for group in caches if taken(group)]
    firsts = sum((group & own) & -(group & own) for group in held)
    return own & ~sum(held) or own & ~firsts or own


def _claim(group, keep):
    """Whether cache `group` is none of her loops' on this host - a named mutex there (Windows), a locked file
    (elsewhere) -, taken and kept while this process lives where `keep`, else only asked."""
    name = 'coaxial-loop-%x' % group
    if sys.platform == 'win32':
        kernel = _kernel()
        held = (kernel.CreateMutexW(None, False, 'Local\\' + name) if keep
                else kernel.OpenMutexW(_SYNCHRONIZE, False, 'Local\\' + name))
        free = bool(held) and ctypes.get_last_error() != _EXISTS if keep else not held
        if held and not (keep and free):
            kernel.CloseHandle(held)
        if keep and free:
            _MINE.append(held)
        return free
    import fcntl
    import tempfile
    try:
        fd = os.open(os.path.join(tempfile.gettempdir(), name + '.lock'), os.O_CREAT | os.O_RDWR, 0o666)
    except OSError:
        return False
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        os.close(fd)
        return False
    if keep:
        _MINE.append(fd)
    else:
        os.close(fd)
    return True


def _given():
    """Her loop's cache group given back: taken where this process cannot be held to it - a job's own affinity, a
    test's runner's -, it is none's again."""
    if sys.platform == 'win32':
        _kernel().CloseHandle(_MINE[0])
    else:
        os.close(_MINE[0])
    _MINE.clear()


def _bound(mask, thread):
    """This process on the processors of `mask`, this thread on `thread`'s where given: whether it now is, as the
    system tells it after - a job's own affinity (a test's runner's) takes the call and keeps its own. Windows
    holds a thread apart from its process; Linux's later threads are their maker's, so there this thread stays its
    process's - held to one core, every thread it made would share it."""
    if sys.platform == 'win32':
        kernel = _kernel()
        kernel.SetProcessAffinityMask(kernel.GetCurrentProcess(), mask)
        if now() == mask and thread:
            kernel.SetThreadAffinityMask(kernel.GetCurrentThread(), thread)
        return now() == mask
    try:
        os.sched_setaffinity(0, [cpu for cpu in range(mask.bit_length()) if mask >> cpu & 1])
    except (AttributeError, OSError):
        return False
    return now() == mask


def now():
    """This process's logical processors as the system holds it now, a mask; 0 where it tells none."""
    if sys.platform == 'win32':
        kernel, mine, every = _kernel(), ctypes.c_size_t(0), ctypes.c_size_t(0)
        kernel.GetProcessAffinityMask(kernel.GetCurrentProcess(), ctypes.byref(mine), ctypes.byref(every))
        return mine.value
    try:
        return sum(1 << cpu for cpu in os.sched_getaffinity(0))
    except (AttributeError, OSError):
        return 0


def _keen():
    """This thread the user's own on macOS (its quality of service user-interactive): on its fast cores, not
    throttled - all a thread is told there. Whether it now is."""
    try:
        return ctypes.CDLL(None).pthread_set_qos_class_self_np(_INTERACTIVE, 0) == 0
    except (AttributeError, OSError):
        return False


def raised():
    """This process above normal priority where BENCH is set (a benchmark's relay): her measurements less at the
    mercy of the host's other loads. False where unset or on another system; results untouched - only how soon."""
    if not os.environ.get(BENCH) or sys.platform != 'win32':
        return False
    kernel = _kernel()
    return bool(kernel.SetPriorityClass(kernel.GetCurrentProcess(), getattr(subprocess, 'ABOVE_NORMAL_PRIORITY_CLASS')))


def placed(loop=False, host=None, cache=False):
    """This process on the lowest logical processor of each physical core but the first: none of her processes
    on another's sibling thread, the scheduler free among the rest - above normal priority where a benchmark asks
    (`raised`), its speed not throttled as a background window's (Windows). Her `loop`'s process on those of a cache
    group it takes for itself (`chosen`), this thread - her loop's - held to its first fast one (`_bound`), or told
    the user's own where no thread is bound (`_keen`, macOS); a `cache` process (a working set of its own: her
    camera's, her vision's) on a group of its own alike, its threads free among its cores - held to one, a peer's
    load on it held her eyes and her errands' pace fell 6-10 %; another process off every group so taken (`left`).
    False where it cannot be placed (two cores or fewer, another system); results are untouched either way - only where
    her processes run."""
    if host is None:
        raised()
        if sys.platform == 'darwin':
            return loop and _keen()
    found, caches = topology() if host is None else host
    if len(found) < 3:
        return False
    if sys.platform == 'win32':
        kernel, state = _kernel(), _Throttling(1, _SPEED, 0)
        kernel.SetProcessInformation(kernel.GetCurrentProcess(), _THROTTLING, ctypes.byref(state), ctypes.sizeof(state))
    own, fast = apart(found)
    if (loop or cache) and len(_MINE) < 2:
        mine = chosen(caches, own, lambda group: _claim(group, True), fast)
        if mine:
            _MINE.append(mine)
    # a group once taken is kept: an aside's own call after it (`aside.serve`) binds it there again
    mine = _MINE[1] if len(_MINE) > 1 else 0
    first = (mine & fast or mine) & -(mine & fast or mine) if loop else 0
    held = _bound(mine or left(caches, own, lambda group: not _claim(group, False)), first)
    if mine and not held:
        _given()
    return held
