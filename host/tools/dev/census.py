"""The machine's census: its cores and threads, its free commit, who lives, a tree's processes.

    census.physical_cores(), census.threads()     # the machine's - a container's, its own
    census.free_commit_gb()                       # the commit charge still free (Windows)
    census.console()                              # whether this process's console answers (Windows)
    census.alive(pid)                             # whether the process lives
    taken = census.Snapshot()                     # every process and its parent, read once
    taken.tree(pids, own)                         # the live processes in the trees `pids` root, less `own`'s
    taken.private_gb(taken.tree([pid]))           # what a tree holds, GB

The relay and the machine's ledger ask it (tools.dev.focus, tools.dev.ledger), and so do the batch's widths and
the host's pace (tools.dev.background, tools.dev.machine_detector).
"""
import collections
import ctypes
import functools
import os
import subprocess
from contextlib import suppress


@functools.lru_cache(maxsize=None)
def counted():
    """(physical cores, logical processors) of those Linux gives this process's control group - a container's, not
    its host's: its cores as sysfs pairs their threads (`placed.given`, `placed.topology`) -; None where none is told:
    another system, no cpuset."""
    from machine.process import placed
    mask = placed.given()
    found = placed.topology()[0] if mask else []
    return (len(found), bin(mask).count('1')) if found else None


@functools.lru_cache(maxsize=None)
def threads():
    """The machine's logical processors: its control group's where Linux tells them (`counted`), else the system's
    count."""
    told = counted()
    return told[1] if told else (os.cpu_count() or 1)


@functools.lru_cache(maxsize=None)
def physical_cores():
    """The machine's physical cores: its control group's where Linux tells them (`counted`), else psutil's count
    where it is installed, Windows' own (Win32_Processor), else the logical count."""
    told = counted()
    if told:
        return told[0]
    try:
        import psutil
        return psutil.cpu_count(logical=False) or os.cpu_count() or 1
    except ImportError:
        pass
    if os.name == 'nt':
        with suppress(OSError, ValueError, subprocess.TimeoutExpired):
            said = subprocess.run(
                ['powershell', '-NoProfile', '-Command',
                 '(Get-CimInstance Win32_Processor | Measure-Object -Property NumberOfCores '
                 '-Sum).Sum'], capture_output=True, text=True, timeout=30).stdout
            return max(1, int(said.strip()))
    return os.cpu_count() or 1


def free_commit_gb():
    """The commit charge still free, GB; None off Windows."""
    if os.name != 'nt':
        return None

    class Status(ctypes.Structure):
        _fields_ = [('length', ctypes.c_uint32), ('load', ctypes.c_uint32)] + [
            (name, ctypes.c_uint64) for name in ('phys', 'phys_free', 'commit', 'commit_free',
                                                 'virtual', 'virtual_free', 'extended')]
    status = Status(length=ctypes.sizeof(Status))
    if not ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(status)):
        return None
    return status.commit_free / 2 ** 30


def console():
    """Whether this process has a console that answers (Windows': its code page told): begun in one, a process
    lives on when the console's host is killed - with who started the process -, and each process started in that
    console after ends at once, 0xC0000142. False with none, and off Windows."""
    return os.name == 'nt' and bool(ctypes.windll.kernel32.GetConsoleCP())


def alive(pid):
    """Whether process `pid` lives."""
    try:
        import psutil
        return psutil.pid_exists(pid)
    except ImportError:
        if os.name != 'nt':
            with suppress(OSError):
                os.kill(pid, 0)
                return True
            return False
        handle = ctypes.windll.kernel32.OpenProcess(0x1000, False, pid)
        if not handle:
            return False
        code = ctypes.c_uint32()
        ctypes.windll.kernel32.GetExitCodeProcess(handle, ctypes.byref(code))
        ctypes.windll.kernel32.CloseHandle(handle)
        return code.value == 259


def alive_since(pid, at):
    """Whether process `pid` lives and is the one that lived at `at` (a wall time): a pid the system gave again to a
    process started after `at` is another's - a build's marker whose writer was killed named a pid Windows had
    given to msedge.exe, SystemSettings.exe and EpicWebHelper.exe, and every process asking for those libraries
    waited on it for good."""
    if not alive(pid):
        return False
    try:
        import psutil
    except ImportError:
        return True
    try:
        return psutil.Process(pid).create_time() <= at
    except psutil.Error:
        return False


def _parents():
    """{pid: its parent's} of every process on the machine."""
    if os.name != 'nt':
        out = {}
        for name in filter(str.isdigit, os.listdir('/proc')):
            with suppress(OSError, ValueError, IndexError), open('/proc/%s/stat' % name, encoding='utf-8') as f:
                out[int(name)] = int(f.read().rsplit(')', 1)[1].split()[1])
        return out

    class Entry(ctypes.Structure):
        _fields_ = [('size', ctypes.c_ulong), ('usage', ctypes.c_ulong), ('pid', ctypes.c_ulong), ('heap', ctypes.c_size_t),
                    ('module', ctypes.c_ulong), ('threads', ctypes.c_ulong), ('parent', ctypes.c_ulong),
                    ('base', ctypes.c_long), ('flags', ctypes.c_ulong), ('exe', ctypes.c_char * 260)]
    k = ctypes.windll.kernel32
    k.CreateToolhelp32Snapshot.restype = ctypes.c_void_p
    snap, e, out = ctypes.c_void_p(k.CreateToolhelp32Snapshot(2, 0)), Entry(), {}
    e.size = ctypes.sizeof(Entry)
    ok = k.Process32First(snap, ctypes.byref(e))
    while ok:
        out[e.pid] = e.parent
        ok = k.Process32Next(snap, ctypes.byref(e))
    k.CloseHandle(snap)
    return out


class Snapshot:
    """Every process on the machine and its parent, read once: an ask of the ledger reads every job's tree off the
    one (`focus.tally`). Read a booking - psutil's children, a scan of every process each - an ask took 15 ms a
    job booked, 467 ms with 28, under the ledger's lock, and the relays' batons waited on it in a convoy."""

    #: How many this process has read: a test counts an ask's.
    count = 0

    def __init__(self):
        Snapshot.count += 1
        self.parents = _parents()
        self.kids = collections.defaultdict(list)
        for pid, parent in self.parents.items():
            self.kids[parent].append(pid)

    def tree(self, pids, own=()):
        """The live processes in the trees `pids` root: a job's children - the robot's planner, an emulator's
        Renode - are cores it holds too; counted by its own process alone, a batch of 28 batons held 282. Less
        those of `own` and what they start, counted by the cores booked for them: counted twice, two groups of
        two cores no longer fit a runner's four threads."""
        todo, seen = [pid for pid in pids if pid in self.parents], set(own)
        while todo:
            pid = todo.pop()
            todo += self.kids[pid] * (pid not in seen)
            seen.add(pid)
        return seen - set(own)

    def private_gb(self, pids):
        """The private bytes the processes `pids` hold, GB; None unread (no psutil)."""
        try:
            import psutil
        except ImportError:
            return None
        held = 0
        for pid in pids:
            with suppress(psutil.Error):
                info = psutil.Process(pid).memory_info()
                held += getattr(info, 'private', info.rss)
        return held / 2 ** 30
