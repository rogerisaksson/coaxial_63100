"""The robot's processes placed by its host's topology: a core apart, its loop's on a cache of its own.

Read off the host at each start: Windows via GetLogicalProcessorInformationEx, Linux via sysfs and cgroup cpusets,
macOS via QOS_CLASS_USER_INTERACTIVE. Each helper process claims a physical core (`core`) off processor 0 and binds a
single thread (`apart`) without SMT sibling contention; a terminal UX (`placed(ui=True)`) pins its render thread to a
claimed core on Windows while keeping its process mask across one thread per non-loop core so background loaders and
child pools spread cleanly. Each real-time loop (`placed(True)`) claims a last-level cache group (`chosen`), pinning
its main thread to the group's first fast core while helper C threads run on the group's remaining cores.

    placed.placed()            # helper process: 1 thread on a claimed physical core, off loop caches
    placed.placed(True)        # real-time loop: dedicated LLC group, main thread on its first fast core
    placed.placed(cache=True)  # camera / vision: dedicated LLC group, threads free within it
    placed.placed(ui=True)     # terminal UX: UI thread on a claimed core, process mask 1 thread/core off loop caches
    placed.free_processors()   # 1 thread per free physical core off processor 0, loop caches, and claimed cores
    mask, held = placed.claim()
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
#: Set (by a benchmark's relay, inherited by every process it starts), each runs above normal priority:
#: Windows' ABOVE_NORMAL class - over the host's other loads, short of HIGH, which a baton a core could starve
#: the desktop with.
BENCH = 'COAXIAL_BENCH'
#: Set by who starts the robot's processes (a desk's batch, `tools.dev.background.lower`): the processors they are
#: given, a hex mask, placed within as a container's cpuset (`given`); unset, all.
GIVEN = 'COAXIAL_GIVEN'
#: Set (not '0'), processes are left where the system schedules them, none bound: a run's `--float`.
FLOAT = 'COAXIAL_FLOAT'
#: Windows' rights on another process enough to set its processors and read them back.
_SET_INFORMATION, _QUERY_LIMITED = 0x0200, 0x1000
#: The loop's cache group once taken: what holds it while this process lives, its own processors.
_MINE: list = []
#: This process's core once taken (`core`): what holds it while this process lives, its logical processors.
_CORED: list = []


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
    kernel.OpenProcess.restype = wintypes.HANDLE
    kernel.OpenProcess.argtypes = [wintypes.DWORD, wintypes.BOOL, wintypes.DWORD]
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


def told(environ=None):
    """The logical processors who started this process gives it (GIVEN), a mask; 0 where none is
    told."""
    try:
        return int((os.environ if environ is None else environ).get(GIVEN, '0'), 16)
    except ValueError:
        return 0


def topology():
    """([(class, each physical core's logical processors, a mask)], [each last-level cache's]) as this host tells
    them - Windows, Linux held to the processors its control group is given (`held`: a container's) -, held to
    those who started it gives the robot (`told`: a desk's batch's); ([], []) on another system."""
    if sys.platform == 'win32':
        return held(*_windows(), told())
    return held(*held(*sysfs(), given()), told()) if sys.platform.startswith('linux') else ([], [])


def cores():
    """Each physical core's logical processors as a mask; [] on another system or where it is refused."""
    return [mask for _class, mask in topology()[0]]


def groups():
    """Each last-level cache's logical processors as a mask - the cores that share it -; [] likewise."""
    return topology()[1]


def apart(found):
    """(the robot's processors, the fast of them) of the cores `found` [(class, mask)]: one logical processor of
    each physical core but the system's first (processor 0's, its interrupts'), and those of them on cores of the
    highest class - all, where every core is one."""
    kept = [core for core in sorted(found, key=lambda core: core[1]) if not core[1] & 1]
    best = max((klass for klass, _mask in kept), default=0)
    return sum(m & -m for _k, m in kept), sum(m & -m for k, m in kept if k == best)


def chosen(caches, own, free, fast=0):
    """The robot's loop's processors of the cache group it takes: of `caches` (masks) one that `free` (a group
    -> bool, asked once each till one is) says is none's and that holds two of its processors `own` at
    least - its loop's thread and the one beside it -, those with a `fast` processor before the rest,
    the last first: the first's first core is the system's. 0 where none fits: a cache a core, every group
    taken."""
    for group in sorted(caches, key=lambda g: (bool(g & fast), g), reverse=True):
        mine = group & own
        if mine & (mine - 1) and free(group):
            return mine
    return 0


def core(found, caches, own, free):
    """A process's core of its own: of `found`'s [(class, mask)] with a processor in `own`, the fast first, each of
    `caches`' first core before any's second, the last cache first, the first `free` (mask -> bool) takes; 0, none."""
    mine = [(klass, mask) for klass, mask in found if mask & own]
    ranked = []
    for klass, mask in mine:
        group = next((g for g in caches if g & mask), 0)
        rank = sum(1 for _k, other in mine if other < mask and other & group)
        ranked.append((-klass, rank, -group, mask))
    return next((mask for *_key, mask in sorted(ranked) if free(mask)), 0)


def left(caches, own, taken):
    """Another process's processors: `own` less every cache group of `caches` that `taken` (a group -> bool) says
    one of the robot's loops holds; less those loops' first cores alone where that leaves none - a host of one cache
    for all its cores -; `own` where that leaves none either."""
    held = [group for group in caches if taken(group)]
    firsts = sum((group & own) & -(group & own) for group in held)
    return own & ~sum(held) or own & ~firsts or own


def _locked(name):
    """A descriptor holding lock file `name` (not Windows); None where another holds it or it cannot open."""
    if sys.platform == 'win32':
        return None
    import fcntl
    import tempfile
    try:
        fd = os.open(os.path.join(tempfile.gettempdir(), name + '.lock'), os.O_CREAT | os.O_RDWR, 0o666)
    except OSError:
        return None
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        os.close(fd)
        return None
    return fd


def _take(mask, kind='loop'):
    """What holds `mask` (a cache group's, or a core's: `kind`) for this process where none of the robot's own on the
    host does: a named mutex (Windows), a locked file (elsewhere), till it ends or `free`; None where another holds
    it."""
    name = 'coaxial-%s-%x' % (kind, mask)
    if sys.platform != 'win32':
        return _locked(name)
    kernel = _kernel()
    held = kernel.CreateMutexW(None, False, 'Local\\' + name)
    if held and ctypes.get_last_error() != _EXISTS:
        return held
    if held:
        kernel.CloseHandle(held)
    return None


def _taken(mask, kind='loop'):
    """Whether one of the robot's own holds `mask` (`_take`'s), only asked; held where it cannot be told."""
    name = 'coaxial-%s-%x' % (kind, mask)
    if sys.platform != 'win32':
        fd = _locked(name)
        if fd is not None:
            os.close(fd)
        return fd is None
    kernel = _kernel()
    held = kernel.OpenMutexW(_SYNCHRONIZE, False, 'Local\\' + name)
    if held:
        kernel.CloseHandle(held)
    return bool(held)


def free(held):
    """What `_take` or `claim` took let go; None, nothing."""
    if held is None:
        return
    if sys.platform == 'win32':
        _kernel().CloseHandle(held)
    else:
        os.close(held)


def _group(group, found):
    """Whether the loop takes `group`, none holding it or a core of it (`found`'s): kept in `_MINE`."""
    if any(_taken(mask, 'core') for _klass, mask in found if mask & group):
        return False
    held = _take(group)
    if held is not None:
        _MINE.append(held)
    return held is not None


def _cored(mask):
    """Whether this process takes core `mask`: kept in `_CORED`."""
    held = _take(mask, 'core')
    if held is not None:
        _CORED.append(held)
    return held is not None


def _given():
    """The loop's cache group given back: taken where this process cannot be held to it - a job's own
    affinity, a test's runner's -, it is none's again."""
    free(_MINE[0])
    _MINE.clear()


def _ungiven():
    """This process's core given back, as `_given` its group."""
    free(_CORED[0])
    _CORED.clear()


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
    """This process above normal priority where BENCH is set (a benchmark's relay): measurements less at the
    mercy of the host's other loads. False where unset or on another system; results untouched - only how
    soon."""
    if not os.environ.get(BENCH) or sys.platform != 'win32':
        return False
    kernel = _kernel()
    return bool(kernel.SetPriorityClass(kernel.GetCurrentProcess(), getattr(subprocess, 'ABOVE_NORMAL_PRIORITY_CLASS')))


def placed(loop=False, host=None, cache=False, ui=False):
    """Place this process on its host's topology (`loop` for a real-time loop's LLC group, `cache` for camera/vision,
    `ui` for a terminal UX whose Windows render thread takes a dedicated core while child/loader threads span `rest`)."""
    if host is None:
        raised()
        if sys.platform == 'darwin':
            return (loop or ui) and _keen()
    found, caches = topology() if host is None else host
    if len(found) < 3:
        return False
    if sys.platform == 'win32':
        kernel, state = _kernel(), _Throttling(1, _SPEED, 0)
        kernel.SetProcessInformation(kernel.GetCurrentProcess(), _THROTTLING, ctypes.byref(state), ctypes.sizeof(state))
    if os.environ.get(FLOAT, '0') != '0':
        return False
    own, fast = apart(found)
    if (loop or cache) and len(_MINE) < 2:
        mine = chosen(caches, own, lambda group: _group(group, found), fast)
        if mine:
            _MINE.append(mine)
            if _CORED:
                _ungiven()
    mine = _MINE[1] if len(_MINE) > 1 else 0
    rest = left(caches, own, _taken)
    if not mine and len(_CORED) < 2:
        cored = core(found, caches, rest, _cored)
        if cored:
            _CORED.append(cored)
    cored = 0 if mine else (_CORED[1] if len(_CORED) > 1 else 0)
    one = (cored & own) or (cored & -cored)
    win_ui = ui and not (loop or cache) and sys.platform == 'win32'
    to = (rest | one) if win_ui else (
        mine or one or rest & ~sum(mask for _klass, mask in found if _taken(mask, 'core')) or rest)
    first = ((to & fast or to) & -(to & fast or to) if (mine or cored) else 0) if loop else (one if win_ui else 0)
    held = _bound(to, first)
    if mine and not held:
        _given()
    if cored and not held:
        _ungiven()
    return held


def free_processors(host=None):
    """One logical processor per free physical core (no SMT sibling sharing, off processor 0, off active loop cache
    groups and claimed cores such as a terminal page's), falling back to non-loop or host processors on small hosts."""
    found, caches = topology() if host is None else host
    if not found:
        return 0
    own, _fast = apart(found)
    rest = left(caches, own, _taken) if own else 0
    free = rest & ~sum(mask for _klass, mask in found if _taken(mask, 'core'))
    return free or rest or own or sum(mask for _klass, mask in found)


def claim(host=None):
    """(A core's processors, what holds it) for a process this one starts (a batch's job, `tools.dev.focus.run`),
    taken as `placed` takes one (`core`), held till `free`; (0, None) where none is free or none placed."""
    found, caches = topology() if host is None else host
    if len(found) < 3 or os.environ.get(FLOAT, '0') != '0':
        return 0, None
    held = [None]

    def take(mask):
        held[0] = _take(mask, 'core')
        return held[0] is not None
    mask = core(found, caches, left(caches, apart(found)[0], _taken), take)
    return (mask, held[0]) if mask else (0, None)


def bind(pid, mask):
    """Process `pid` (this one's child) on processors `mask`: whether it now is - Linux's main thread, its later
    threads its own."""
    if not mask:
        return False
    if sys.platform != 'win32':
        try:
            os.sched_setaffinity(pid, [cpu for cpu in range(mask.bit_length()) if mask >> cpu & 1])
        except (AttributeError, OSError):
            return False
        return True
    kernel, mine, every = _kernel(), ctypes.c_size_t(0), ctypes.c_size_t(0)
    handle = kernel.OpenProcess(_SET_INFORMATION | _QUERY_LIMITED, False, pid)
    if not handle:
        return False
    try:
        kernel.SetProcessAffinityMask(handle, mask)
        kernel.GetProcessAffinityMask(handle, ctypes.byref(mine), ctypes.byref(every))
    finally:
        kernel.CloseHandle(handle)
    return mine.value == mask
