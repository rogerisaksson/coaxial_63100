#!/usr/bin/env python3
"""What a page's keys read in the pseudoconsole a terminal writes to, as VS Code runs a page.

    python -X utf8 tools/dev/conpty_input.py                  # the system's pseudoconsole
    python -X utf8 tools/dev/conpty_input.py --conpty DLL     # VS Code's: node-pty's conpty.dll

VS Code's terminal (xterm.js) writes the mouse in SGR to the program's pseudoconsole, which parses it into the
console's records and hands a program that reads VT its own encoding of them. Here a child holds the mouse with
`terminal.ui.console.Keys` and VT output on, as a page does, inside a pseudoconsole; this process writes what
VS Code would (`STEPS`), a step at a time, and prints what the page's keys made of it. VS Code pastes the
clipboard on a right press (terminal.integrated.rightClickBehavior copyPaste, Windows' default): the system's
pseudoconsole hands the paste on in its bracketed-paste markers, which the keys drop whole; VS Code's own
(terminal.integrated.windowsUseConptyDll, OpenConsole 1.25) without them, and the paste is typed. Windows only.
"""
import argparse
import ctypes
import json
import os
import subprocess
import sys
import tempfile
import threading
import time
from ctypes import wintypes

ESC = chr(27)
#: What VS Code sends for a chord of both buttons, then a drag of the left alone and the wheel - xterm.js's
#: _sendEvent: a press its button, a move the lowest held (left, middle, right), a release its button - with
#: its paste of the clipboard on the right press, bracketed as the page asks; and the pause after each.
STEPS = (('left down', ESC + '[<0;40;10M'), ('right down', ESC + '[<2;40;10M'),
         ('right press pastes', ESC + '[200~ab 4 /x' + ESC + '[201~'), ('both held, moved', ESC + '[<32;42;11M'),
         ('right up', ESC + '[<2;42;11m'), ('left held, moved', ESC + '[<32;46;13M'), ('left up', ESC + '[<0;46;13m'),
         ('left down again', ESC + '[<0;50;10M'), ('left dragged', ESC + '[<32;54;12M'),
         ('left up', ESC + '[<0;54;12m'), ('wheel up', ESC + '[<64;50;10M'))
PAUSE_S = 0.4
#: The child: VT output on and the mouse held, as a page has them; a line for each poll that made anything.
CHILD = r'''
import json, sys, time
sys.path.insert(0, sys.argv[1])
from terminal.ui import console
from terminal.ui.stage import _vt_on
_vt_on()
log = open(sys.argv[2], 'w', encoding='utf-8')
with console.Keys(True, mouse=True, select=frozenset(), pan=True) as keys:
    end = time.monotonic() + float(sys.argv[3])
    while time.monotonic() < end:
        leave, zoom = keys.poll()
        made = {'leave': leave, 'typed': keys.taken(), 'spun': keys.dragged(), 'panned': keys.panned(),
                'clicked': keys.clicked(), 'zoom': zoom}
        if leave or zoom or made['typed'] or made['clicked'] or made['spun'] != (0.0, 0.0) \
                or made['panned'] != (0.0, 0.0):
            log.write(json.dumps(made) + '\n')
            log.flush()
        time.sleep(0.01)
'''


class _Coord(ctypes.Structure):
    _fields_ = (('X', ctypes.c_short), ('Y', ctypes.c_short))


class _StartupInfo(ctypes.Structure):
    _fields_ = (('cb', wintypes.DWORD), ('lpReserved', wintypes.LPWSTR), ('lpDesktop', wintypes.LPWSTR),
                ('lpTitle', wintypes.LPWSTR), ('dwX', wintypes.DWORD), ('dwY', wintypes.DWORD),
                ('dwXSize', wintypes.DWORD), ('dwYSize', wintypes.DWORD), ('dwXCountChars', wintypes.DWORD),
                ('dwYCountChars', wintypes.DWORD), ('dwFillAttribute', wintypes.DWORD), ('dwFlags', wintypes.DWORD),
                ('wShowWindow', wintypes.WORD), ('cbReserved2', wintypes.WORD), ('lpReserved2', ctypes.c_void_p),
                ('hStdInput', wintypes.HANDLE), ('hStdOutput', wintypes.HANDLE), ('hStdError', wintypes.HANDLE))


class _StartupInfoEx(ctypes.Structure):
    _fields_ = (('StartupInfo', _StartupInfo), ('lpAttributeList', ctypes.c_void_p))


class _ProcessInfo(ctypes.Structure):
    _fields_ = (('hProcess', wintypes.HANDLE), ('hThread', wintypes.HANDLE),
                ('dwProcessId', wintypes.DWORD), ('dwThreadId', wintypes.DWORD))


def _kernel(conpty):
    """(kernel32, its CreatePseudoConsole, its ClosePseudoConsole) - the two taken from `conpty` when told (a
    conpty.dll, `Conpty` before each)."""
    k = ctypes.WinDLL('kernel32', use_last_error=True)
    maker = ctypes.WinDLL(conpty, use_last_error=True) if conpty else k
    prefix = 'Conpty' if conpty else ''
    create, close = getattr(maker, prefix + 'CreatePseudoConsole'), getattr(maker, prefix + 'ClosePseudoConsole')
    create.argtypes = (_Coord, wintypes.HANDLE, wintypes.HANDLE, wintypes.DWORD, ctypes.POINTER(ctypes.c_void_p))
    create.restype = ctypes.c_long
    close.argtypes = (ctypes.c_void_p,)
    k.InitializeProcThreadAttributeList.argtypes = (ctypes.c_void_p, wintypes.DWORD, wintypes.DWORD,
                                                    ctypes.POINTER(ctypes.c_size_t))
    k.UpdateProcThreadAttribute.argtypes = (ctypes.c_void_p, wintypes.DWORD, ctypes.c_size_t, ctypes.c_void_p,
                                            ctypes.c_size_t, ctypes.c_void_p, ctypes.c_void_p)
    k.CreateProcessW.argtypes = (wintypes.LPCWSTR, wintypes.LPWSTR, ctypes.c_void_p, ctypes.c_void_p, wintypes.BOOL,
                                 wintypes.DWORD, ctypes.c_void_p, wintypes.LPCWSTR, ctypes.c_void_p, ctypes.c_void_p)
    return k, create, close


def run(steps=STEPS, conpty=None):
    """What the child's keys made of `steps`, written to its pseudoconsole (the system's, or `conpty`'s) a
    PAUSE_S apart: a dict a poll that made anything."""
    (k, create, close), seconds = _kernel(conpty), 3.0 + PAUSE_S * len(steps)
    in_read, in_write, out_read, out_write = (wintypes.HANDLE() for _ in range(4))
    k.CreatePipe(ctypes.byref(in_read), ctypes.byref(in_write), None, 0)
    k.CreatePipe(ctypes.byref(out_read), ctypes.byref(out_write), None, 0)
    console = ctypes.c_void_p()
    if create(_Coord(120, 40), in_read, out_write, 0, ctypes.byref(console)):
        raise OSError('CreatePseudoConsole failed')
    k.CloseHandle(in_read)
    k.CloseHandle(out_write)
    size = ctypes.c_size_t()
    k.InitializeProcThreadAttributeList(None, 1, 0, ctypes.byref(size))
    attributes = ctypes.create_string_buffer(size.value)
    k.InitializeProcThreadAttributeList(attributes, 1, 0, ctypes.byref(size))
    k.UpdateProcThreadAttribute(attributes, 0, 0x00020016, console, ctypes.sizeof(ctypes.c_void_p), None, None)
    info = _StartupInfoEx()
    info.StartupInfo.cb, info.lpAttributeList = ctypes.sizeof(info), ctypes.addressof(attributes)
    # The pseudoconsole's own handles, not this process's: with its stdio redirected (a pipe, a shell's
    # capture) a child takes those unless told to use none (STARTF_USESTDHANDLES, all null).
    info.StartupInfo.dwFlags = 0x00000100

    def write(text):
        data, done = text.encode('utf-8'), wintypes.DWORD()
        k.WriteFile(in_write, data, len(data), ctypes.byref(done), None)

    def drain():
        """The pseudoconsole's output read away, its cursor-position question answered."""
        buf, got = ctypes.create_string_buffer(65536), wintypes.DWORD()
        while k.ReadFile(out_read, buf, 65536, ctypes.byref(got), None) and got.value:
            if ESC + '[6n' in buf.raw[:got.value].decode('utf-8', 'replace'):
                write(ESC + '[1;1R')
    with tempfile.TemporaryDirectory() as folder:
        log, script = os.path.join(folder, 'keys.jsonl'), os.path.join(folder, 'child.py')
        with open(script, 'w', encoding='utf-8') as f:
            f.write(CHILD)
        root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        line = subprocess.list2cmdline([sys.executable, '-X', 'utf8', script, root, log, str(seconds)])
        started = _ProcessInfo()
        if not k.CreateProcessW(None, ctypes.create_unicode_buffer(line), None, None, False, 0x00080000, None,
                                root, ctypes.byref(info), ctypes.byref(started)):
            raise OSError('CreateProcessW failed: %d' % ctypes.get_last_error())
        threading.Thread(target=drain, daemon=True).start()
        time.sleep(2.0)
        for _name, sent in steps:
            write(sent)
            time.sleep(PAUSE_S)
        k.WaitForSingleObject(started.hProcess, int(1000 * (seconds + 5)))
        for handle in (started.hProcess, started.hThread, in_write):
            k.CloseHandle(handle)
        close(console)
        with open(log, encoding='utf-8') as f:
            return [json.loads(text) for text in f.read().splitlines()]


def main():
    parser = argparse.ArgumentParser(description=(__doc__ or '').split('\n\n')[0])
    parser.add_argument('--conpty', help="a conpty.dll to make the pseudoconsole with (OpenConsole beside it)")
    args = parser.parse_args()
    for step, (name, sent) in enumerate(STEPS):
        print('%2d  %-20s %r' % (step, name, sent))
    print("-- the page's keys in %s pseudoconsole made of it:" % ('that' if args.conpty else "the system's"))
    for made in run(conpty=args.conpty):
        print('    %s' % ', '.join('%s %r' % kv for kv in made.items() if kv[1] not in (None, [], 0.0, (0.0, 0.0),
                                                                                       [0.0, 0.0], False)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
