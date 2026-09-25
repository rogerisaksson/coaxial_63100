#!/usr/bin/env python3
"""The board's MCU emulated on Renode, its console a `socket://` port the library opens.

The application's ELF on Renode's STM32H753 as board/emu describes the board.

    python tools/emu/emulator.py                  # until Ctrl+C; prints the URL
    python tools/emu/emulator.py --elf build/Release/coaxial_63100.elf

    with Emulator() as emu:
        rig = Coaxial63100(port=emu.url, own_image=False).open()

Renode: $RENODE, `renode` on PATH, or the newest portable build under
%LOCALAPPDATA%/renode (setup: docs/ARCHITECTURE.md).
"""
import argparse
import glob
import os
import re
import shutil
import socket
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools import REPO  # noqa: E402

SCRIPT = 'board/emu/coaxial_63100.resc'
#: The image: COAXIAL_ELF, else the Debug build.
ELF = os.environ.get('COAXIAL_ELF') or os.path.join(REPO, 'build', 'Debug', 'coaxial_63100.elf')

#: Renode's start, the ADC class compiled, the image booted: 5.6 s measured on the laptop
#: (2026-09-25); the wait allows ten times that.
READY_S = 60.0

#: The monitor's prompt, `(machine)` after a line's end - Renode ends lines \n\r or \r\r\n -
#: and the colour codes around it.
PROMPT = re.compile(r'[\r\n]\([^)\r\n]*\)\s*$')
ANSI = re.compile(r'\x1b\[[0-9;?]*[A-Za-z]')


def find_renode():
    """Renode's executable, or None."""
    named = os.environ.get('RENODE') or shutil.which('renode')
    if named:
        return named
    local = os.path.join(os.environ.get('LOCALAPPDATA', ''), 'renode', 'renode_*', 'renode.exe')
    found = sorted(glob.glob(local))
    return found[-1] if found else None


def free_port():
    """A TCP port nothing listens on, as the system hands one out."""
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        return probe.getsockname()[1]


class Emulator:
    """One Renode process running `elf`, its console at `url` once `start()` returns."""

    def __init__(self, elf=ELF, port=None, log=None, monitor=None):
        """`monitor`: a TCP port for Renode's monitor, True for a free one, None for none."""
        self.elf = os.path.abspath(elf)
        self.port = port or free_port()
        self.monitor = free_port() if monitor is True else monitor
        self.console = None
        self.url = 'socket://127.0.0.1:%d' % self.port
        self.log = log
        self.process = None

    def start(self):
        renode = find_renode()
        if renode is None:
            raise RuntimeError('no Renode: set RENODE, put renode on PATH, or unpack the '
                               'portable build under %LOCALAPPDATA%/renode')
        if not os.path.exists(self.elf):
            raise RuntimeError('no image at %s - build it first' % self.elf)
        sink = open(self.log, 'w', encoding='utf-8') if self.log else subprocess.DEVNULL
        commands = '$port=%d; $elf=@%s; include @%s; start' % (
            self.port, self.elf.replace(os.sep, '/'), SCRIPT)
        self.process = subprocess.Popen(
            [renode, '--disable-gui', '--plain', '-P', str(self.monitor or -1), '-e', commands],
            cwd=REPO, stdout=sink, stderr=subprocess.STDOUT)
        self._ready(self.process)
        return self

    def _ready(self, process):
        """Until the console answers its help key: Renode up, the image through main()'s
        init and polling the console."""
        deadline = time.time() + READY_S
        while time.time() < deadline:
            if process.poll() is not None:
                raise RuntimeError('Renode exited with %d' % process.returncode)
            try:
                with socket.create_connection(('127.0.0.1', self.port), timeout=1.0) as s:
                    s.settimeout(0.5)
                    for _ in range(10):
                        s.sendall(b'?')
                        if b'commands:' in _heard(s, 0.5):
                            return
            except OSError:
                time.sleep(0.5)
        self.stop()
        raise RuntimeError('the emulated board did not answer in %.0f s' % READY_S)

    def command(self, text):
        """One monitor command's answer (`afe DcBusVolts 24`): needs `monitor`. One connection
        for the emulator's life; its greeting, wrapped in telnet's negotiation, drained once."""
        if not self.monitor:
            raise RuntimeError('the emulator runs without a monitor port')
        if self.console is None:
            self.console = socket.create_connection(('127.0.0.1', self.monitor), timeout=2.0)
            self.console.settimeout(0.1)
            _heard_quiet(self.console, 0.5, 10.0)
        self.console.sendall(text.encode() + b'\n')
        said = _heard_until(self.console, PROMPT, 30.0).decode('utf-8', 'replace')
        lines = [line.strip() for line in ANSI.sub('', said).splitlines()]
        return '\n'.join(line for line in lines[1:-1] if line)

    def wait(self):
        """Until Renode exits."""
        if self.process:
            self.process.wait()

    def stop(self):
        if self.console is not None:
            self.console.close()
            self.console = None
        if self.process and self.process.poll() is None:
            self.process.kill()
            self.process.wait()

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()


def _heard_until(s, pattern, seconds):
    """Bytes until `pattern` matches the tail, or `seconds` pass."""
    got = b''
    end = time.time() + seconds
    while time.time() < end and not pattern.search(ANSI.sub('', got.decode('utf-8', 'replace'))):
        try:
            chunk = s.recv(65536)
        except socket.timeout:
            continue
        if not chunk:
            break
        got += chunk
    return got


def _heard_quiet(s, quiet, seconds):
    """Bytes until none arrive for `quiet` seconds after the first, or `seconds` pass."""
    got = b''
    end = time.time() + seconds
    last = end
    while time.time() < end and not (got and time.time() - last > quiet):
        try:
            chunk = s.recv(65536)
        except socket.timeout:
            continue
        if not chunk:
            break
        got += chunk
        last = time.time()
    return got


def _heard(s, seconds):
    got = b''
    end = time.time() + seconds
    while time.time() < end:
        try:
            chunk = s.recv(4096)
        except socket.timeout:
            continue
        if not chunk:
            break
        got += chunk
    return got


def main():
    parser = argparse.ArgumentParser(description='The board\'s MCU emulated on Renode.')
    parser.add_argument('--elf', default=ELF, help='the application image (default: Debug)')
    parser.add_argument('--port', type=int, help='the console\'s TCP port (default: a free one)')
    parser.add_argument('--log', help='Renode\'s output into this file')
    parser.add_argument('--monitor', type=int, help='Renode\'s monitor on this TCP port')
    args = parser.parse_args()
    with Emulator(args.elf, args.port, args.log, args.monitor) as emu:
        print(emu.url, flush=True)
        try:
            emu.wait()
        except KeyboardInterrupt:
            pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
