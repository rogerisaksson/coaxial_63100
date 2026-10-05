#!/usr/bin/env python3
"""The board's MCU emulated on Renode, its console a `frames://` port the library opens.

Each write to it a frame, its length ahead of it (protocol_frames.py).

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
import threading
import time

import serial

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools import REPO  # noqa: E402
from coaxial.devices.boot import HAND_AT, HAND_MAGIC  # noqa: E402
from coaxial.simulated.sto import PILOT_HZ  # noqa: E402
from tools.emu import world as worlds  # noqa: E402
from tools.emu.probes import (ANSI, answers, boot_answers, heard_quiet, heard_until,  # noqa: E402
                              own_temp, tied)

# frames://, the URL every emulator here hands out.
if 'tools.emu' not in serial.protocol_handler_packages:
    serial.protocol_handler_packages.append('tools.emu')

SCRIPT = 'board/emu/coaxial_63100.resc'
LIMB_REPL = 'board/emu/coaxial_63100_limb.repl'
#: Where a composed script is written: a limb's outgrows a command line.
WORK = os.path.join(REPO, 'build', 'emu')
#: The image: COAXIAL_ELF, else the Debug build; the bootloader beside it.
ELF = os.environ.get('COAXIAL_ELF') or os.path.join(REPO, 'build', 'Debug', 'coaxial_63100.elf')
BOOT_ELF = os.path.join(os.path.dirname(ELF), 'coaxial_63100_boot.elf')
#: Where each runs from: the bootloader from flash, the image from D2 SRAM (docs/BOOT.md).
BOOT_VTOR = 0x08000000
IMAGE_VTOR = 0x30000000
#: The bootloader's RS485 rate (docs/BOOT.md).
BOOT_BAUD = 10_000_000

#: Renode's start, the ADC class compiled, the image booted: 5.6 s on the laptop
#: (2026-09-25); the wait allows ten times that, and as long again a node.
READY_S = 60.0

#: The monitor's prompt, `(machine)` after a line's end - Renode ends lines \n\r or \r\r\n.
PROMPT = re.compile(r'[\r\n]\([^)\r\n]*\)\s*$')

#: assign's flag in the handover slot: the last node closes the termination.
FLAG_TERMINATE = 0x01
#: The 96-bit unique id (UID_BASE): its first word told apart per node.
UID_AT = 0x1FF1E800

#: The core's instructions a virtual second, the default: the part's 475 M. At Renode's own 100 M a
#: 240 B echo blast at 10 Mbit lost 12 of 200, and the drive's ISR (2 922 cycles) outran its
#: 20 us period and starved the link (2026-09-25).
FAITHFUL_MIPS = 475

#: The core's speed while no ADC waits on TRGO2 - no drive runs to the part's budget. The
#: polls' register accesses, 17 a pass, cost the same at any rate: idle with the AFE on 2.0
#: wall s a virtual s at 100, 1.2 at 50, 1.4 at 25 (the quantum's round trips left), 475's
#: 5.2 (2026-09-27). The link's ring holds a frame at any pass rate.
IDLE_MIPS = 50

#: How far a limb's boards run apart before they wait for each other, s: 8 idle boards run
#: 390 M instructions a wall second in all at 500 us, 387 M at 100 us, 367 M at 1 ms (MPU off,
#: 2026-09-25). The bus's bytes cross at
#: these boundaries, so it stays under RTU's t1.5 of 750 us inside a frame: 1 ms broke every
#: frame longer than a quantum's bytes. One board's too: Renode's 100 us cost it 0.13 wall s
#: a virtual s idle (2026-09-27).
QUANTUM = '0.0005'

#: main()'s time between the handlers while an ADC waits on TRGO2, us, the rest skipped
#: (Coaxial63100_Plant.LoopSlice): under the drive 2.5 wall s a virtual s against 16.5 whole,
#: 0.25 us 2.4, 1 us 5.3 (2026-09-25). Paced emulators only; the suites run main() whole.
LOOP_SLICE_US = 0.5

#: MPU_CTRL.ENABLE masked on the bus, the image's MPU off: tlib keeps no TLB entry for a page
#: inside an enabled region's span whose subregion is disabled, and walks the MPU on every
#: access there - CubeMX's 4 GB region, SRD 0x87, spans ITCM, DTCM and D2 SRAM. Idle at
#: 100 MIPS 8.2 -> 1.4 wall s a virtual s (2026-09-25). The suites run it on (`mpu`).
MPU_OFF = 'sysbus SetHookBeforePeripheralWrite sysbus.nvic "value = value & ~1" <0xD94, 0xD97>'

#: Wall s the emulation's speed is taken over once up.
SCALE_S = 1.0

#: Renode above the host's other apps on Windows, which stretched its pace between two
#: measures; not high: eight limbs' processes would starve the desktop.
PRIORITY = getattr(subprocess, 'ABOVE_NORMAL_PRIORITY_CLASS', 0)


def find_renode():
    """Renode's executable, or None."""
    named = os.environ.get('RENODE') or shutil.which('renode')
    if named and os.path.exists(named):
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

    def __init__(self, elf=ELF, port=None, log=None, monitor=None, world=None,
                 mips: int | None = FAITHFUL_MIPS, boot=False,
                 idle_mips: int | None = IDLE_MIPS, mpu=False):
        """`monitor`: a TCP port for Renode's monitor, a free one if None or True;
        `world`: a world's name (board/emu/worlds), its first node this board; `mips`: the
        core's instructions a virtual second, millions, the part's own by default - None for
        Renode's 100 - while an ADC waits on TRGO2, `idle_mips` else (None: `mips` throughout);
        `boot`: the bootloader from flash, blank, waiting for the host to load the image
        over Modbus (docs/BOOT.md) - host and target then run one build; `mpu`: the image's
        MPU on, as on the part, at a seventh of the speed (MPU_OFF)."""
        self.boot = boot
        self.mpu = mpu
        self.elf = os.path.abspath(BOOT_ELF if boot and elf == ELF else elf)
        self.mips = mips
        self.idle_mips = idle_mips
        self.world = worlds.load(world) if world else None
        self.port = port or free_port()
        self.monitor = free_port() if monitor in (None, True) else monitor
        self.monitor_socket = None
        self._monitor_lock = threading.Lock()
        #: Wall seconds a virtual second, as last measured: the host's waits are stretched by
        #: it (coaxial.comm.transport). The image sets it - at 475 MIPS the app 5, 18-22
        #: under the drive, its bootloader 9 (2026-09-25).
        self.time_scale = 1.0
        #: The same with the core kept awake, WFI a no-op: a request's pace, which an idle
        #: core's load understates - asleep it keeps real time, and replies came after the
        #: host had given up (2026-09-25). The least the host waits by.
        self.awake_scale = 1.0
        self.url = 'frames://127.0.0.1:%d' % self.port
        self.consoles = [self.port]
        #: The units its images answer to: the app's 1, none while blank in the bootloader.
        self.units = () if boot else (1,)
        self.log = log
        self._sink = None
        self.process = None

    def script(self):
        """The monitor's commands that build and start the emulation."""
        return (['$port=%d' % self.port, '$elf=@%s' % self.elf.replace(os.sep, '/')]
                + (['$vtor=0x%08X' % BOOT_VTOR] if self.boot else [])
                + ['include @%s' % SCRIPT] + self.planted(0) + self.guarded() + self.paced()
                + ['emulation SetGlobalQuantum "%s"' % QUANTUM, 'start'])

    def guarded(self):
        """The MPU masked off for the machine last created, unless `mpu`."""
        return [] if self.mpu else [MPU_OFF]

    def paced(self):
        """The core's speed, if not Renode's own, for the machine last created: IDLE_MIPS until
        an ADC waits on TRGO2 and `mips` while one does, or `mips` throughout where `idle_mips`
        is None or the bootloader runs."""
        if not self.mips:
            return []
        if self.idle_mips and not self.boot:
            return ['cpu PerformanceInMips %d' % self.idle_mips,
                    'sysbus.gpioPortE.plant BusyMips %d' % self.mips,
                    'sysbus.gpioPortE.plant IdleMips %d' % self.idle_mips,
                    'sysbus.gpioPortE.plant LoopSlice %g' % LOOP_SLICE_US]
        return ['cpu PerformanceInMips %d' % self.mips]

    def planted(self, node):
        """The board at `node`'s STO chain on the world library, and its world's commands if it
        has a world."""
        library = worlds.library()
        chain = ['%s Library "%s"' % (worlds.STO, library.replace(os.sep, '/')),
                 '%s Node %d' % (worlds.STO, node)]
        if self.world is None:
            return chain + ['%s DcBusVolts %s' % (worlds.AFE, worlds._decimal(worlds.LINK_VOLTS))]
        return chain + worlds.commands(self.world, node, library, first=node == 0)

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
        # Its own config and monitor history: Renode rewrites the history after every command,
        # and a body's limbs sharing %APPDATA%'s collided - an IOException, the limb gone
        # (2026-09-25).
        config = os.path.join(WORK, 'renode_%d.config' % self.port)
        with open(config, 'w', encoding='utf-8') as f:
            f.write('[general]\nhistory-path = %s\n'
                    % os.path.join(WORK, 'history_%d' % self.port))
        # Its log kept beside them: what Renode said before it went, where it goes (CI's
        # Release runs, 2026-09-27).
        self.log = self.log or os.path.join(WORK, 'renode_%d.log' % self.port)
        self._sink = open(self.log, 'w', encoding='utf-8')
        self.process = subprocess.Popen(
            [renode, '--disable-gui', '--plain', '--config', config, '-P', str(self.monitor),
             '-e', 'include @%s' % composed.replace(os.sep, '/')],
            cwd=REPO, stdout=self._sink, stderr=subprocess.STDOUT, creationflags=PRIORITY,
            env=own_temp(os.path.join(WORK, 'temp_%d' % self.port)))
        self._job = tied(self.process)
        self._ready(self.process)
        self.awake_scale = self.awake()
        return self

    def virtual_seconds(self):
        """The emulation's elapsed virtual time, s: the board's clock on the host."""
        info = self.command('emulation GetTimeSourceInfo')
        said = re.search(r'Elapsed Virtual Time: (\S+)', info)
        if said is None:
            raise RuntimeError('Renode gave no virtual time: %r' % info[:200])
        h, m, s = said.group(1).split(':')
        return (int(h) * 60 + int(m)) * 60 + float(s)

    def load(self):
        """Renode's Current load - wall seconds a virtual second, lately - at least 1: the
        Transport's time scale, asked each transaction."""
        said = re.search(r'Current load: (\S+)', self.command('emulation GetTimeSourceInfo'))
        return max(self.awake_scale, float(said.group(1))) if said else self.time_scale

    def awake(self, seconds=SCALE_S):
        """`time_scale` over `seconds` with every core kept awake: WFI a no-op, taken up once
        the translations holding it are cleared."""
        self._each_cpu('cpu WfiAsNop true; cpu ClearTranslationCache')
        try:
            time.sleep(seconds)                 # the image translated afresh first
            return self.measure(seconds)
        finally:
            self._each_cpu('cpu WfiAsNop false; cpu ClearTranslationCache')

    def _each_cpu(self, text):
        """Monitor commands, `;` between them, on the machine's CPU."""
        for part in text.split('; '):
            self.command(part)

    def _set(self, device, **values):
        """A world device's properties on every machine, as the monitor reads numbers."""
        self._each_cpu('; '.join('%s %s %s' % (device, name, worlds._decimal(value))
                                 for name, value in values.items()))

    def heat_clock(self, haste):
        """Every plant's heat on `haste` thermal s a virtual s, as the rig sets its boards'
        observers (Coaxial63100._in_its_world)."""
        self._set(worlds.PLANT, Haste=haste)

    def room(self, ambient, air=1.0, capacity=1.0):
        """Every plant's world in a room (coaxial.model.rooms): ambient, C, air path and
        capacity scaled."""
        self._set(worlds.PLANT, Ambient=ambient, Air=air, Capacity=capacity)

    def drag(self, k_drag, torque):
        """Every plant's load laid live (native.Limb.drag): drag, N m per (rad/s)^2, torque."""
        self._set(worlds.PLANT, Drag=k_drag, LoadTorque=torque)

    def pilot(self, volts, hz=PILOT_HZ, noise=0.0):
        """The master's common-mode pilot on the bus, every board's STO chain on it: its
        amplifier's amplitude, V (0 none), and Hz; the far end's 100 kHz common mode, V."""
        self._set(worlds.STO, PilotVolts=volts, PilotHz=hz, NoiseVolts=noise)

    def measure(self, seconds=SCALE_S):
        """`time_scale` over `seconds` of wall time, at least 1."""
        virtual, wall = self.virtual_seconds(), time.monotonic()
        time.sleep(seconds)
        virtual2, wall2 = self.virtual_seconds(), time.monotonic()
        self.time_scale = max(1.0, (wall2 - wall) / max(virtual2 - virtual, 1e-9))
        return self.time_scale

    def _ready(self, process):
        """Until every console answers its help key: Renode up, each image through main()'s
        init and polling its console."""
        deadline = time.time() + READY_S * len(self.consoles)
        for port in self.consoles:
            while not (boot_answers(port) if self.boot else answers(port)):
                if process.poll() is not None:
                    raise RuntimeError('Renode exited with %d%s' % (process.returncode, self.said()))
                if time.time() > deadline:
                    self.stop()
                    raise RuntimeError('the emulated board on %d did not answer%s'
                                       % (port, self.said()))
                time.sleep(0.5)

    def said(self):
        """Renode's errors from its log, the last eight, as a message's tail."""
        if not self.log:
            return ''
        try:
            with open(self.log, encoding='utf-8', errors='replace') as f:
                lines = [line.strip() for line in f
                         if 'ERROR' in line or 'xception' in line or 'abort' in line.lower()]
        except OSError:
            return ''
        return (':\n  ' + '\n  '.join(lines[-8:])) if lines else ''

    def command(self, text):
        """One monitor command's answer (`afe DcBusVolts 24`). One connection for the
        emulator's life; its greeting, wrapped in telnet's negotiation, drained once."""
        with self._monitor_lock:
            return self._command(text)

    def _command(self, text):
        if self.monitor_socket is None:
            self.monitor_socket = socket.create_connection(('127.0.0.1', self.monitor),
                                                           timeout=2.0)
            self.monitor_socket.settimeout(0.1)
            heard_quiet(self.monitor_socket, 0.5, 10.0)
        self.monitor_socket.sendall(text.encode() + b'\n')
        said = heard_until(self.monitor_socket, PROMPT, 30.0).decode('utf-8', 'replace')
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
        if self._sink is not None:
            self._sink.close()
            self._sink = None

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()


class Limb(Emulator):
    """`nodes` boards in one Renode, on one RS485 limb as a bootloader leaves them: node i is
    unit i at position i, the last closing the termination. `url` is the host's adapter on
    the bus, `consoles` each node's console port; the nodes' machines are node1..nodeN."""

    def __init__(self, nodes, elf=ELF, log=None, monitor=None, world=None,
                 mips: int | None = FAITHFUL_MIPS,
                 baud=None, boot=False, idle_mips: int | None = IDLE_MIPS, mpu=False):
        """`baud`: the bus's rate, bits a second - the app starts at the record's, 115 200, the
        bootloader (`boot`, each node blank until the host loads it) at BOOT_BAUD."""
        super().__init__(elf, log=log, monitor=monitor, world=world, mips=mips, boot=boot,
                         idle_mips=idle_mips, mpu=mpu)
        self.baud = baud or (BOOT_BAUD if boot else None)
        self.nodes = nodes
        self.units = () if boot else tuple(range(1, nodes + 1))
        self.consoles = [free_port() for _ in range(nodes)]

    def script(self):
        """The board's script once a node, its name, port and console written in: Renode
        does not see variables set between two includes of one script (2026-09-25)."""
        with open(os.path.join(REPO, SCRIPT), encoding='utf-8') as f:
            board = [line for line in f.read().splitlines() if not line.startswith(('$', ':'))]
        out = ['$elf=@%s' % self.elf.replace(os.sep, '/')]
        for unit, port in enumerate(self.consoles, 1):
            flags = FLAG_TERMINATE if unit == self.nodes else 0
            vtor = '0x%08X' % (BOOT_VTOR if self.boot else IMAGE_VTOR)
            out += [line.replace('$name', '"node%d"' % unit).replace('$port', str(port))
                    .replace('$console', '"node%d-console"' % unit).replace('$vtor', vtor)
                    for line in board]
            out += self.planted(unit - 1) + self.guarded() + self.paced()
            if not self.boot:
                # As a bootloader leaves it; a node in boot mode is blank, the host assigns it.
                out += ['sysbus WriteDoubleWord 0x%08X 0x%08X' % (HAND_AT, HAND_MAGIC),
                        'sysbus WriteDoubleWord 0x%08X 0x%08X' % (HAND_AT + 8,
                                                                  unit | unit << 8 | flags << 16)]
            out += ['sysbus WriteDoubleWord 0x%08X 0x%08X' % (UID_AT, 0x63100000 | unit),
                    # Its own board within the tolerances: the front end's errors drawn from
                    # its UID.
                    '%s NoiseSeed %d' % (worlds.AFE, 0x63100000 | unit)]
        out += ['emulation CreateUARTHub "limb"']
        for unit in range(1, self.nodes + 1):
            out += ['mach set "node%d"' % unit,
                    'connector Connect sysbus.gpioPortA.transceiverA limb']
        out += ['emulation SetGlobalQuantum "%s"' % QUANTUM,
                'mach set "node1"', 'machine LoadPlatformDescription @%s' % LIMB_REPL,
                'connector Connect sysbus.gpioPortK.adapterBus limb']
        out += ['sysbus.gpioPortK.adapterBus BaudRate %d' % self.baud] if self.baud else []
        out += [
                'emulation CreateServerSocketTerminal %d "limb-host" false' % self.port,
                'connector Connect sysbus.gpioPortK.adapterHost limb-host', 'start']
        return out


    def bus_rate(self, baud):
        """The host's adapter at `baud`, bits a second, as its port sets it."""
        if baud and baud != self.baud:
            self.command('mach set "node1"')
            self.command('sysbus.gpioPortK.adapterBus BaudRate %d' % baud)
            self.baud = baud

    def _each_cpu(self, text):
        """A monitor command on every node's CPU, node1 the machine after."""
        for unit in range(1, self.nodes + 1):
            self.command('mach set "node%d"' % unit)
            for part in text.split('; '):
                self.command(part)
        self.command('mach set "node1"')


class Body:
    """A machine's limbs, a Renode process each so they run on the host's cores side by side -
    one process's machines wait for each other every quantum, eight boards in one doing 5 a
    board's work (2026-09-25). Each limb is its own RS485 segment, as on the machine:
    `urls[name]` is its bus. `limbs` {name: boards}, `worlds` {name: a world's name}."""

    def __init__(self, limbs, elf=ELF, worlds=None, mips: int | None = FAITHFUL_MIPS,
                 mpu=False):
        worlds = worlds or {}
        self.limbs = {name: Limb(n, elf, world=worlds.get(name), mips=mips, mpu=mpu)
                      for name, n in limbs.items()}
        self.urls = {name: limb.url for name, limb in self.limbs.items()}

    def start(self):
        failed = []

        def up(limb):
            try:
                limb.start()
            except RuntimeError as exc:
                failed.append(str(exc))

        threads = [threading.Thread(target=up, args=(limb,)) for limb in self.limbs.values()]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        if failed:
            self.stop()
            raise RuntimeError('; '.join(failed))
        return self

    def stop(self):
        for limb in self.limbs.values():
            limb.stop()

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()


def check(emu):
    """The image up on Renode and answering the library: its version in one line and 0, or the
    reason and 1 - setup.ps1's last stage."""
    from coaxial import Coaxial63100

    try:
        with emu:
            rig = Coaxial63100(port=emu.url, own_image=False).open()
            try:
                v = rig.board.version_info
            finally:
                rig.close()
            print('firmware %s, protocol %d.%d, built %s; awake %.1f wall s a virtual s'
                  % (v.get('firmware'), v.get('proto_major', 0), v.get('proto_minor', 0),
                     v.get('build', '?'), emu.awake_scale), flush=True)
        return 0
    except Exception as exc:              # the reason is the answer
        print('%s: %s' % (type(exc).__name__, exc), flush=True)
        return 1


def main():
    parser = argparse.ArgumentParser(description='The board\'s MCU emulated on Renode.')
    parser.add_argument('--elf', default=ELF, help='the application image (default: Debug)')
    parser.add_argument('--port', type=int, help='the console\'s TCP port (default: a free one)')
    parser.add_argument('--nodes', type=int, default=0, help='a limb of this many boards')
    parser.add_argument('--world', choices=worlds.names(), help='what the motors turn')
    parser.add_argument('--boot', action='store_true',
                        help='the bootloader, blank, waiting for the host to load the image')
    parser.add_argument('--log', help='Renode\'s output into this file')
    parser.add_argument('--monitor', type=int, help='Renode\'s monitor on this TCP port')
    parser.add_argument('--check', action='store_true',
                        help='up, the image\'s version read through the library, down: one line')
    args = parser.parse_args()
    emu = (Limb(args.nodes, args.elf, args.log, args.monitor, args.world, boot=args.boot)
           if args.nodes
           else Emulator(args.elf, args.port, args.log, args.monitor, args.world,
                         boot=args.boot))
    if args.check:
        return check(emu)
    with emu:
        print('%s, %.1f wall s a virtual s' % (emu.url, emu.time_scale), flush=True)
        if args.nodes:
            for unit, port in enumerate(emu.consoles, 1):
                print('node%d console frames://127.0.0.1:%d' % (unit, port), flush=True)
        try:
            emu.wait()
        except KeyboardInterrupt:
            pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
