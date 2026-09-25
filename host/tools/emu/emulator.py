#!/usr/bin/env python3
"""The board's MCU emulated on Renode, its console a `socket://` port the library opens.

The application's ELF on Renode's STM32H753 as board/emu describes the board; a limb is N of
them on one RS485 bus, the host's adapter on it.

    python tools/emu/emulator.py                  # until Ctrl+C; prints the URL
    python tools/emu/emulator.py --elf build/Release/coaxial_63100.elf
    python tools/emu/emulator.py --nodes 4        # a limb: the bus's URL, then each console

    with Emulator() as emu:
        rig = Coaxial63100(port=emu.url, own_image=False).open()
    with Limb(4) as limb:
        rig = Coaxial63100(port=limb.url, unit=3, own_image=False).open()

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
LIMB_REPL = 'board/emu/coaxial_63100_limb.repl'
#: Where a composed script is written: a limb's outgrows a command line.
WORK = os.path.join(REPO, 'build', 'emu')
#: The image: COAXIAL_ELF, else the Debug build.
ELF = os.environ.get('COAXIAL_ELF') or os.path.join(REPO, 'build', 'Debug', 'coaxial_63100.elf')

#: Renode's start, the ADC class compiled, the image booted: 5.6 s measured on the laptop
#: (2026-09-25); the wait allows ten times that, and as long again a node.
READY_S = 60.0

#: The monitor's prompt, `(machine)` after a line's end - Renode ends lines \n\r or \r\r\n -
#: and the colour codes around it.
PROMPT = re.compile(r'[\r\n]\([^)\r\n]*\)\s*$')
ANSI = re.compile(r'\x1b\[[0-9;?]*[A-Za-z]')

#: The handover slot a bootloader leaves (boot/inc/boot.h boot_hand_t, the top 32 B of DTCM):
#: magic, stay, then unit | position << 8 | flags << 16, the image's size and CRC - zero, as
#: a debugger leaves them.
HAND_AT = 0x2001FFE0
HAND_MAGIC = 0x444E4148
FLAG_TERMINATE = 0x01
#: The 96-bit unique id (UID_BASE): its first word told apart per node.
UID_AT = 0x1FF1E800


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
        self.monitor_socket = None
        self.url = 'socket://127.0.0.1:%d' % self.port
        self.consoles = [self.port]
        self.log = log
        self.process = None

    def script(self):
        """The monitor's commands that build and start the emulation."""
        return ['$port=%d' % self.port, '$elf=@%s' % self.elf.replace(os.sep, '/'),
                'include @%s' % SCRIPT, 'start']

    def start(self):
        renode = find_renode()
        if renode is None:
            raise RuntimeError('no Renode: set RENODE, put renode on PATH, or unpack the '
                               'portable build under %LOCALAPPDATA%/renode')
        if not os.path.exists(self.elf):
            raise RuntimeError('no image at %s - build it first' % self.elf)
        os.makedirs(WORK, exist_ok=True)
        composed = os.path.join(WORK, 'run_%d.resc' % self.port)
        with open(composed, 'w', encoding='utf-8') as f:
            f.write('\n'.join(self.script()) + '\n')
        sink = open(self.log, 'w', encoding='utf-8') if self.log else subprocess.DEVNULL
        self.process = subprocess.Popen(
            [renode, '--disable-gui', '--plain', '-P', str(self.monitor or -1), '-e',
             'include @%s' % composed.replace(os.sep, '/')],
            cwd=REPO, stdout=sink, stderr=subprocess.STDOUT)
        self._ready(self.process)
        return self

    def _ready(self, process):
        """Until every console answers its help key: Renode up, each image through main()'s
        init and polling its console."""
        deadline = time.time() + READY_S * len(self.consoles)
        for port in self.consoles:
            while not _answers(port):
                if process.poll() is not None:
                    raise RuntimeError('Renode exited with %d' % process.returncode)
                if time.time() > deadline:
                    self.stop()
                    raise RuntimeError('the emulated board on %d did not answer' % port)
                time.sleep(0.5)

    def command(self, text):
        """One monitor command's answer (`afe DcBusVolts 24`): needs `monitor`. One connection
        for the emulator's life; its greeting, wrapped in telnet's negotiation, drained once."""
        if not self.monitor:
            raise RuntimeError('the emulator runs without a monitor port')
        if self.monitor_socket is None:
            self.monitor_socket = socket.create_connection(('127.0.0.1', self.monitor),
                                                           timeout=2.0)
            self.monitor_socket.settimeout(0.1)
            _heard_quiet(self.monitor_socket, 0.5, 10.0)
        self.monitor_socket.sendall(text.encode() + b'\n')
        said = _heard_until(self.monitor_socket, PROMPT, 30.0).decode('utf-8', 'replace')
        lines = [line.strip() for line in ANSI.sub('', said).splitlines()]
        return '\n'.join(line for line in lines[1:-1] if line)

    def wait(self):
        """Until Renode exits."""
        if self.process:
            self.process.wait()

    def stop(self):
        if self.monitor_socket is not None:
            self.monitor_socket.close()
            self.monitor_socket = None
        if self.process and self.process.poll() is None:
            self.process.kill()
            self.process.wait()

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()


class Limb(Emulator):
    """`nodes` boards in one Renode, on one RS485 limb as a bootloader leaves them: node i is
    unit i at position i, the last closing the termination. `url` is the host's adapter on
    the bus, `consoles` each node's console port; the nodes' machines are node1..nodeN."""

    def __init__(self, nodes, elf=ELF, log=None, monitor=None):
        super().__init__(elf, log=log, monitor=monitor)
        self.nodes = nodes
        self.consoles = [free_port() for _ in range(nodes)]

    def script(self):
        """The board's script once a node, its name, port and console written in: Renode
        does not see variables set between two includes of one script (2026-09-25)."""
        with open(os.path.join(REPO, SCRIPT), encoding='utf-8') as f:
            board = [line for line in f.read().splitlines() if not line.startswith(('$', ':'))]
        out = ['$elf=@%s' % self.elf.replace(os.sep, '/')]
        for unit, port in enumerate(self.consoles, 1):
            flags = FLAG_TERMINATE if unit == self.nodes else 0
            out += [line.replace('$name', '"node%d"' % unit).replace('$port', str(port))
                    .replace('$console', '"node%d-console"' % unit) for line in board]
            out += ['sysbus WriteDoubleWord 0x%08X 0x%08X' % (HAND_AT, HAND_MAGIC),
                    'sysbus WriteDoubleWord 0x%08X 0x%08X' % (HAND_AT + 8,
                                                              unit | unit << 8 | flags << 16),
                    'sysbus WriteDoubleWord 0x%08X 0x%08X' % (UID_AT, 0x63100000 | unit)]
        out += ['emulation CreateUARTHub "limb"']
        for unit in range(1, self.nodes + 1):
            out += ['mach set "node%d"' % unit,
                    'connector Connect sysbus.gpioPortA.transceiverA limb']
        out += ['mach set "node1"', 'machine LoadPlatformDescription @%s' % LIMB_REPL,
                'connector Connect sysbus.gpioPortK.adapterBus limb',
                'emulation CreateServerSocketTerminal %d "limb-host" false' % self.port,
                'connector Connect sysbus.gpioPortK.adapterHost limb-host', 'start']
        return out


def _answers(port):
    """Whether the console on `port` answers its help key."""
    try:
        with socket.create_connection(('127.0.0.1', port), timeout=1.0) as s:
            s.settimeout(0.5)
            for _ in range(10):
                s.sendall(b'?')
                if b'commands:' in _heard(s, 0.5):
                    return True
    except OSError:
        pass
    return False


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
    parser.add_argument('--nodes', type=int, default=0, help='a limb of this many boards')
    parser.add_argument('--log', help='Renode\'s output into this file')
    parser.add_argument('--monitor', type=int, help='Renode\'s monitor on this TCP port')
    args = parser.parse_args()
    emu = (Limb(args.nodes, args.elf, args.log, args.monitor) if args.nodes
           else Emulator(args.elf, args.port, args.log, args.monitor))
    with emu:
        print(emu.url, flush=True)
        if args.nodes:
            for unit, port in enumerate(emu.consoles, 1):
                print('node%d console socket://127.0.0.1:%d' % (unit, port), flush=True)
        try:
            emu.wait()
        except KeyboardInterrupt:
            pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
