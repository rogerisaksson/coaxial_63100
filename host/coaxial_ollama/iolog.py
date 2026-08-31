"""The hidden transcript a session leaves behind.

Every question, every call - including the ones `_trace` skips - and
every answer, for reading back when there is no terminal scrollback to
paste in. Overwritten each session, hidden on Windows so it does not
clutter the working tree.
"""
import json
import os
import sys
import time

from .sandbox import clip


# host/prompt_io.tmp - resolved from this file's own location, not the
# caller's cwd, so `python dbg.py` from host/ and a task that starts
# somewhere else both land in the same place, at the same fixed name a
# later debugging session can just open without knowing a timestamp.
IO_LOG_PATH = os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'prompt_io.tmp')


def _set_attributes(path, value):
    """Windows file attributes, best-effort. Not security - a file with the
    raw questions and answers of a bench session is not secret, it is just
    not something that belongs in an ordinary directory listing next to the
    files this project is actually about."""
    if sys.platform != 'win32':
        return
    try:
        import ctypes
        ctypes.windll.kernel32.SetFileAttributesW(str(path), value)
    except Exception:                                        # noqa: BLE001
        pass


def _unhide(path):
    """Clear the hidden attribute before (re)opening a session's log for
    writing. Measured directly : `open(path, 'w')` on an
    already-hidden file raised a plain PermissionError, not the OSError
    IOLog already expected and swallowed - the truncate that mode implies
    is what Windows refuses on a hidden file, not the open itself. 0x80 is
    FILE_ATTRIBUTE_NORMAL; nothing to do if the file does not exist yet."""
    if os.path.exists(path):
        _set_attributes(path, 0x80)


def _hide(path):
    _set_attributes(path, 0x02)                     # FILE_ATTRIBUTE_HIDDEN


class IOLog:
    """A hidden per-session log of every question, call and answer - for
    debugging this loop afterwards, not for the operator.

    Overwritten each session, not appended: a log covering three runs ago is
    worse than none when what matters is this one. It keeps more than the
    screen does - a refused afe_power call is hidden from the trace and kept
    here, because that is what answers "why did that turn cost four calls".
    """

    def __init__(self, path=IO_LOG_PATH, enabled=True):
        self.handle = None
        if not enabled:
            return
        _unhide(path)
        try:
            self.handle = open(path, 'w', encoding='utf-8', errors='replace')
            _hide(path)
        except OSError:
            self.handle = None

    def write(self, text):
        if self.handle is None:
            return
        try:
            self.handle.write(text)
            self.handle.flush()
        except OSError:
            self.handle = None

    def turn(self, question):
        self.write('=== %s ===\nQ: %s\n' % (time.strftime('%H:%M:%S'),
                                             question))

    def call(self, name, args, result):
        self.write('  %s %s\n  -> %s\n'
                   % (name, json.dumps(args, default=str)[:300],
                      clip(str(result), 500)))

    def answer(self, text):
        self.write('A: %s\n\n' % text)

    def close(self):
        if self.handle is not None:
            try:
                self.handle.close()
            except OSError:
                pass
            self.handle = None
