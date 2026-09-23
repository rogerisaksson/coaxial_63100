"""The hidden transcript a session leaves behind."""
import ctypes
import json
import os
import sys
import time

from .sandbox import clip
from contextlib import suppress


# host/prompt_io.tmp - resolved from this file's own location, not the caller's
# cwd, so `python dbg.py` from host/ and a task that starts somewhere else both
# land in the same place, at the same fixed name a later debugging session can
# just open without knowing a timestamp.
IO_LOG_PATH = os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'prompt_io.tmp')


def _set_attributes(path, value):
    """Windows file attributes, best-effort."""
    if sys.platform != 'win32':
        return
    with suppress(Exception):
        ctypes.windll.kernel32.SetFileAttributesW(str(path), value)


def _unhide(path):
    """Clear the hidden attribute before (re)opening a session's log for
    writing.
    """
    if os.path.exists(path):
        _set_attributes(path, 0x80)


def _hide(path):
    _set_attributes(path, 0x02)                     # FILE_ATTRIBUTE_HIDDEN


class IOLog:
    """A hidden per-session log of every question, call and answer - for
    debugging this loop afterwards, not for the operator.
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
            with suppress(OSError):
                self.handle.close()
            self.handle = None
