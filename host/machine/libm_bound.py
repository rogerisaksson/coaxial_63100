"""MuJoCo as the robot's processes load it: its libm imports bound to the robot's own (wbc_libm).

    mujoco = libm_bound.mujoco()       # imported once a process, its sin, cos, ... the robot's before any model is built

MuJoCo's shared library calls the system's libm for its compiler's turns (fromto, zaxis, euler) and for every step's -
its hinges' and free joints' turns, its contacts' impedance (pow): Windows' UCRT and Linux's glibc round a last bit
apart on a few inputs in a thousand, and the robot's walk parted between the systems from a step's sine on. MuJoCo's
binary, built as it is, calls the robot's libm instead: on Windows the slots of mujoco.dll's import table of
api-ms-win-crt-math, on Linux those of libmujoco.so's GOT of libm.so.6, each written once with the robot's function
(`tools.cores.libm_quick`'s addresses) - floor and sqrt left, exact on every system.
"""
import ctypes
import functools
import glob
import importlib
import mmap
import os
import struct
import sys

#: The functions MuJoCo's library imports that round otherwise on each system's libm.
NAMES = ('sin', 'cos', 'tan', 'asin', 'acos', 'atan2', 'exp', 'log', 'pow')


@functools.lru_cache(maxsize=None)
def mujoco():
    """MuJoCo, imported and its libm bound (`bound`) once a process."""
    mj = importlib.import_module('mujoco')
    BOUND.extend(bind(mj))
    return mj


#: The names bound in this process.
BOUND: list = []


def _ours():
    from tools.cores import libm_quick
    quick = libm_quick.module()
    if quick is None:
        raise RuntimeError("the robot's libm extension cannot be built here: MuJoCo keeps the system's")
    return {name: int(address) for name, address in quick.addresses().items()}


def bind(mj):
    """[names] of NAMES whose imports in `mj`'s shared library now call the robot's libm."""
    here = os.path.dirname(mj.__file__)
    if sys.platform == 'win32':
        return _pe(os.path.join(here, 'mujoco.dll'), _ours())
    found = sorted(glob.glob(os.path.join(here, 'libmujoco.so*')))
    if sys.platform.startswith('linux') and found:
        return _elf(os.path.realpath(found[0]), _ours())
    raise RuntimeError("MuJoCo's libm is bound on Windows and Linux alone, not on %s" % sys.platform)


def _pe(path, ours):
    """The import table of the loaded DLL at `path`: each of its math imports named in `ours` written with ours."""
    k32 = ctypes.WinDLL('kernel32', use_last_error=True)
    k32.GetModuleHandleW.restype, k32.GetModuleHandleW.argtypes = ctypes.c_void_p, [ctypes.c_wchar_p]
    k32.VirtualProtect.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_uint32, ctypes.POINTER(ctypes.c_uint32)]
    base = k32.GetModuleHandleW(path)
    if not base:
        raise RuntimeError('%s is not loaded' % path)
    u16, u32, u64 = (lambda a, t=t: t.from_address(a).value for t in (ctypes.c_uint16, ctypes.c_uint32, ctypes.c_uint64))
    opt = base + u32(base + 0x3C) + 24
    imports = base + u32(opt + (112 if u16(opt) == 0x20B else 96) + 8)
    bound = []
    while u32(imports + 12):
        dll = ctypes.string_at(base + u32(imports + 12)).decode().lower()
        names, slots = base + u32(imports), base + u32(imports + 16)
        while ('api-ms-win-crt-math' in dll or 'ucrtbase' in dll) and u64(names):
            entry = u64(names)
            name = '' if entry >> 63 else ctypes.string_at(base + (entry & 0x7FFFFFFF) + 2).decode()
            if name in ours:
                was = ctypes.c_uint32()
                if not k32.VirtualProtect(slots, 8, 0x04, ctypes.byref(was)):
                    raise ctypes.WinError(ctypes.get_last_error())
                ctypes.c_uint64.from_address(slots).value = ours[name]
                k32.VirtualProtect(slots, 8, was.value, ctypes.byref(was))
                bound.append(name)
            names, slots = names + 8, slots + 8
        imports += 20
    return bound


def _elf(path, ours):
    """The GOT of the loaded shared object at `path`: each relocation of a function named in `ours` written with ours."""
    with open(path, 'rb') as f:
        data = f.read()
    machine, shoff, (shentsize, shnum) = struct.unpack_from('<H', data, 0x12)[0], struct.unpack_from('<Q', data, 0x28)[0], \
        struct.unpack_from('<HH', data, 0x3A)
    kinds = {62: (6, 7), 183: (1025, 1026)}[machine]
    sections = [struct.unpack_from('<IIQQQQIIQQ', data, shoff + k * shentsize) for k in range(shnum)]
    symbols = next(s for s in sections if s[1] == 11)
    strings = sections[symbols[6]][4]
    base, bound = _base(path, data), []
    for sec in (s for s in sections if s[1] == 4):
        for k in range(sec[5] // 24):
            offset, info, _addend = struct.unpack_from('<QQq', data, sec[4] + 24 * k)
            if info & 0xFFFFFFFF in kinds:
                at = strings + struct.unpack_from('<I', data, symbols[4] + 24 * (info >> 32))[0]
                name = data[at:data.index(b'\0', at)].decode()
                if name in ours:
                    _poke(base + offset, ours[name])
                    bound.append(name)
    return bound


def _maps():
    with open('/proc/self/maps', encoding='utf-8') as f:
        for line in f:
            cut = line.split()
            lo, hi = (int(v, 16) for v in cut[0].split('-'))
            yield lo, hi, cut[1], int(cut[2], 16), cut[5] if len(cut) > 5 else ''


def _base(path, data):
    """Where the shared object at `path` is laid in this process: its first mapping less its first segment's address."""
    phoff, (phentsize, phnum) = struct.unpack_from('<Q', data, 0x20)[0], struct.unpack_from('<HH', data, 0x36)
    first = min(struct.unpack_from('<IIQQ', data, phoff + k * phentsize)[3] for k in range(phnum)
                if struct.unpack_from('<I', data, phoff + k * phentsize)[0] == 1)
    lo = min(lo for lo, _hi, _perms, offset, name in _maps() if name == path and offset == 0)
    return lo - (first & ~0xFFF)


def _poke(address, value):
    """`value` written at `address` (a GOT's slot), its page writable for the write and as it was after."""
    libc = ctypes.CDLL(None, use_errno=True)
    libc.mprotect.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int]
    page = address & ~(mmap.PAGESIZE - 1)
    perms = next(p for lo, hi, p, _o, _n in _maps() if lo <= page < hi)
    was = (1 if perms[0] == 'r' else 0) | (2 if perms[1] == 'w' else 0) | (4 if perms[2] == 'x' else 0)
    if libc.mprotect(page, mmap.PAGESIZE, was | 3):
        raise OSError(ctypes.get_errno(), 'mprotect')
    ctypes.c_uint64.from_address(address).value = value
    libc.mprotect(page, mmap.PAGESIZE, was)
