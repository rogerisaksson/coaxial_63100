"""Physical memory, total and free, off the OS: one call for the preload and the model picker."""
import ctypes
import sys


class _Status(ctypes.Structure):
    """MEMORYSTATUSEX, what GlobalMemoryStatusEx fills."""
    _fields_ = [('dwLength', ctypes.c_ulong),
                ('dwMemoryLoad', ctypes.c_ulong),
                ('ullTotalPhys', ctypes.c_ulonglong),
                ('ullAvailPhys', ctypes.c_ulonglong),
                ('ullTotalPageFile', ctypes.c_ulonglong),
                ('ullAvailPageFile', ctypes.c_ulonglong),
                ('ullTotalVirtual', ctypes.c_ulonglong),
                ('ullAvailVirtual', ctypes.c_ulonglong),
                ('ullAvailExtendedVirtual', ctypes.c_ulonglong)]


def physical():
    """(total, free) bytes, or (None, None) where the platform does not say."""
    if sys.platform == 'win32':
        status = _Status()
        status.dwLength = ctypes.sizeof(_Status)
        if ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(status)):
            return int(status.ullTotalPhys), int(status.ullAvailPhys)
        return None, None
    found = {}
    try:
        with open('/proc/meminfo', encoding='ascii') as info:
            for line in info:
                key, _, rest = line.partition(':')
                if key in ('MemTotal', 'MemAvailable'):
                    found[key] = int(rest.split()[0]) * 1024
    except (OSError, ValueError, IndexError):
        pass
    return found.get('MemTotal'), found.get('MemAvailable')
