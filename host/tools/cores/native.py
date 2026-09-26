"""`native://`: the firmware's comms/ and board layer on this host, at the part's pace.

    import tools.cores.native                                           # registers the scheme
    Coaxial63100(port='native://').open()                               # one board, its console
    Coaxial63100(port='native://?world=bench').open()
    Coaxial63100(port='native://?nodes=4&world=humanoid_leg', unit=3).open()  # a limb's bus
    Coaxial63100(port='native://?body=humanoid&bus=LL', unit=2).open()  # the left knee
    Nodes.discover(port='native://?body=humanoid', execution_mode=EMULATED)  # the fleet

The real-time engine for SIL and HIL. board/src's stage, triple, meter, power, STO chain, A1335
and BNO085 drivers and drive, with comms/ over the fake board (tools.cores.fakeboard), on
board/native's chip - TIM1, ADC1-3, SPI2 and SPI4 with DMA1, the front end, the A1335, the
BNO085: Renode's models in C. Each board is a copy of the library, its statics its own; a
limb's boards share a world (world/, a library of its own) and one RS485 bus, and a thread
holds their clocks to the wall's in lockstep, limbs on the host's cores side by side. An
exchange takes its line time off the wall. Built and powered at a URL's first open in a
process, it runs on across sessions. Not the Cortex-M: validation is Renode's (emulator://).
"""
import ctypes
import glob
import itertools
import os
import re
import shutil
import threading
import time
import urllib.parse

import serial
from serial.serialutil import SerialBase

from tools import REPO
from tools.cores import fakeboard
from tools.cores.build import OUT, build, find_cc
from tools.emu import world as worlds
from tools.emu.protocol_emulator import BODIES

SOURCES = fakeboard.SOURCES + [
    os.path.join(REPO, 'board', 'src', name)
    for name in ('board_pwm.c', 'board_sync.c', 'board_adc.c', 'board_io.c', 'board_power.c',
                 'board_sto.c', 'board_angle.c', 'board_imu.c', 'board_drive.c', 'board_ctrl.c',
                 'board_log.c')] + [
    os.path.join(REPO, 'board', 'native', name)
    for name in ('native.c', 'native_io.c', 'native_a1335.c', 'native_bno085.c')] + [
    os.path.join(REPO, 'world', 'src', 'world_heat.c')]

#: board/native first - its stm32h7xx.h, main.h and board_irq.h stand in for the part's - then
#: board/inc, whose board_hw.h the board layer takes over board/fake's.
INCLUDES = ([os.path.join(REPO, 'board', 'native'), os.path.join(REPO, 'board', 'inc')]
            + fakeboard.INCLUDES + [os.path.join(REPO, 'world', 'inc')])

#: A DMA stream holds a buffer's low 32 bits (board_angle.c): native_io.c restores the rest.
EXTRA = ('-Wno-pointer-to-int-cast',)

#: How often a limb's thread brings its boards' clocks up to the wall's, and the most it runs
#: under one hold of the lock an exchange waits behind, s. A virtual second of one board runs
#: in 48 ms with the drive on: the default 15.6 ms timer and one 5 ms run a wake held it to a
#: third of real time (2026-09-25).
PACE_S = 0.001
BURST_S = 0.005

#: How far the board's clock stands from the wall's with a core for its limb: a wake, a burst,
#: a scheduler's slice (Windows' 15.6 ms), s.
TRAIL_S = 0.02

#: A limb's boards run to one time in steps of this, the world stepped between them, us.
LOCKSTEP_US = 1000

#: The host's ports on a board (fake_uart.c): USART3 the console, USART2 the RS485 bus.
CONSOLE_PORT = 0
BUS_PORT = 1

#: The handover's flag on the limb's last board: it closes the termination (boot.h).
TERMINATE = 0x01

#: The bus's silence between frames above 19 200 baud, us (modbus_rtu.c's t3.5).
T35_US = 1750

#: The front end's phase transfer and spread, LTspice's (tools/emu/afe_spice.py).
AFE_REPL = os.path.join(REPO, 'board', 'emu', 'coaxial_63100_afe.repl')
AFE_KEYS = ('PhaseVoltsPerAmp', 'PhaseZeroVolts', 'PhaseGainSigma', 'PhaseZeroSigmaVolts',
            'BusGainSigma')

#: A board's seed, its unit beside: tools.emu seeds each node's front end from its UID.
SEED = 0x63100000

_BUILT = {}
_COPIES = itertools.count()
_LIMBS = {}


def _libraries():
    """(the board's library, the world's), built at the first call in a process."""
    if not _BUILT:
        cc = find_cc()
        if cc is None:
            raise RuntimeError('no C compiler for the native board')
        for stale in glob.glob(os.path.join(OUT, 'native_*')):
            try:
                os.remove(stale)
            except OSError:
                pass                 # a live process holds it
        pid = os.getpid()
        _BUILT['board'] = build(cc, SOURCES, INCLUDES, 'native_%d' % pid, EXTRA)[0]
        _BUILT['world'] = build(cc, worlds.SOURCES, worlds.INCLUDES, 'native_world_%d' % pid)[0]
    return _BUILT['board'], _BUILT['world']


def _copy(path):
    """A library loaded under a name of its own: its statics its own."""
    base, ext = os.path.splitext(path)
    copy = '%s_%d%s' % (base, next(_COPIES), ext)
    shutil.copyfile(path, copy)
    return ctypes.CDLL(copy)


def _afe_values():
    """The front end's transfer and spread from the repl the emulator loads."""
    with open(AFE_REPL, encoding='utf-8') as f:
        text = f.read()
    values = []
    for key in AFE_KEYS:
        found = re.search(r'%s: ([0-9.eE+-]+)' % key, text)
        if found is None:
            raise RuntimeError('%s carries no %s - tools/emu/afe_spice.py writes it'
                               % (AFE_REPL, key))
        values.append(float(found.group(1)))
    return values


class World:
    """A limb's world (board/emu/worlds): the world core's library, a copy a limb, its nodes
    the limb's motors."""

    def __init__(self, name):
        self.spec = worlds.load(name)
        lib = self.lib = _copy(_libraries()[1])
        for call in ('emu_world_body', 'emu_world_load', 'emu_plant_attach', 'emu_world_motor'):
            getattr(lib, call).restype = None
        nodes = self.spec['nodes']
        body = self.spec.get('body', {})
        lib.emu_world_reset(len(nodes))
        lib.emu_world_body(worlds.BODIES[body.get('kind', 'ground')],
                           *[ctypes.c_float(v) for v in worlds._values(body, worlds.BODY)])
        for node, spec in enumerate(nodes):
            load = spec.get('load', {})
            lib.emu_world_load(node, worlds.LOADS[load.get('kind', 'free')],
                               *[ctypes.c_float(v) for v in worlds._values(load, worlds.LOAD)])
            lib.emu_plant_attach(node, *[ctypes.c_float(v) for v in worlds._motor(spec['motor'])])
        self.step = ctypes.cast(lib.emu_plant_step, ctypes.c_void_p).value
        self.shaft = ctypes.cast(lib.emu_plant_state, ctypes.c_void_p).value

    def pole_pairs(self, node):
        return worlds._motor(self.spec['nodes'][node]['motor'])[4]


class Board:
    """One board powered on: a copy of the library, on its world's `node` if it has one."""

    def __init__(self, unit, world=None, node=0, hand=None):
        """`hand`: (position, flags) as a bootloader leaves them, else none left."""
        lib = self.lib = _copy(_libraries()[0])
        lib.fake_hear.argtypes = [ctypes.c_uint8, ctypes.c_uint8]
        lib.fake_said.restype = ctypes.c_uint16
        lib.fake_said.argtypes = [ctypes.c_uint8, ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint16]
        lib.native_run.argtypes = [ctypes.c_uint32]
        lib.native_run_to.argtypes = [ctypes.c_uint64]
        lib.native_seconds.restype = ctypes.c_double
        lib.native_shaft_degrees.restype = ctypes.c_double
        lib.native_afe.argtypes = [ctypes.c_double] * 5 + [ctypes.c_uint32]
        lib.native_world.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int,
                                     ctypes.c_double]
        lib.native_hand.argtypes = [ctypes.c_uint8] * 3
        lib.native_angle.argtypes = [ctypes.c_double]
        lib.native_imu.argtypes = [ctypes.POINTER(ctypes.c_double)]
        if hand is not None:
            lib.native_hand(unit, *hand)
        lib.native_open()
        lib.native_afe(*_afe_values(), SEED | unit)
        if world is not None and node < len(world.spec['nodes']):
            lib.native_world(world.step, world.shaft, node, world.pole_pairs(node))
        self.unit = unit
        self._out = (ctypes.c_uint8 * fakeboard.ANSWER)()

    def said(self, port):
        """What the board sent on `port` since the last call."""
        n = self.lib.fake_said(port, self._out, fakeboard.ANSWER)
        return bytes(self._out[:n])

    def seconds(self):
        return self.lib.native_seconds()


class Limb:
    """Boards on one bus - unit i at position i, the last closing the termination - over one
    world, a thread holding their clocks to the wall's in lockstep. `nodes` 0: one board, its
    console the host's port."""

    def __init__(self, nodes, world=None):
        self.console = not nodes
        self.world = World(world) if world else None
        self.boards = [Board(unit, self.world, unit - 1,
                             None if self.console else (unit, TERMINATE if unit == nodes else 0))
                       for unit in range(1, max(1, nodes) + 1)]
        self.port = CONSOLE_PORT if self.console else BUS_PORT
        self.units = tuple(board.unit for board in self.boards)
        self.lock = threading.Lock()
        self._us = 0
        self._line = 0                 # the line's last activity, us of the limb's clock
        self._start = time.monotonic()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._pace, name='native limb', daemon=True)
        self._thread.start()

    def seconds(self):
        """The limb's clock, s."""
        return self.boards[0].seconds()

    def _run_to(self, us):
        for board in self.boards:
            board.lib.native_run_to(us)
        self._us = us

    def _pace(self):
        """The boards' clocks held to the wall's, a lockstep at a time."""
        timer = getattr(ctypes, 'windll', None)
        if timer is not None:
            timer.winmm.timeBeginPeriod(1)
        try:
            while not self._stop.wait(PACE_S):
                while not self._stop.is_set():
                    with self.lock:
                        wall = int((time.monotonic() - self._start) * 1e6)
                        if wall - self._us < PACE_S * 1e6:
                            break
                        target = min(wall, self._us + int(BURST_S * 1e6))
                        while self._us < target:
                            self._run_to(min(target, self._us + LOCKSTEP_US))
        finally:
            if timer is not None:
                timer.winmm.timeEndPeriod(1)

    def send(self, data):
        """`data` onto the limb's port at the line's pace, every board hearing it, the wall
        waited for the line's time. The frame starts at the wall's time and, as the bus's one
        adapter, no sooner than t3.5 after the line's last activity: every rig on a limb's URL
        has a transport of its own, none knowing the gap another owes, and the addressed board
        closes a frame at once, the others after t3.5 - a frame for unit 2 right behind unit
        1's ran into it and went unanswered (2026-09-26)."""
        with self.lock:
            start = max(int((time.monotonic() - self._start) * 1e6), self._line + T35_US)
            while self._us < start:
                self._run_to(min(start, self._us + LOCKSTEP_US))
            for byte in data:
                for board in self.boards:
                    board.lib.fake_hear(self.port, byte)
            self._us = max(self._us, int(max(board.seconds() for board in self.boards) * 1e6))
            self._line = self._us
            ahead = self._us / 1e6 - (time.monotonic() - self._start)
        if ahead > 0.0:
            time.sleep(ahead)

    def take(self):
        """What the boards said on the limb's port since the last take, the line's activity
        stamped at the limb's clock."""
        with self.lock:
            said = b''.join(board.said(self.port) for board in self.boards)
            if said:
                self._line = self._us
            return said

    def angle(self, unit, degrees):
        """The unit's magnet put at `degrees`, the plant's no longer: a test, a script."""
        with self.lock:
            self.boards[unit - 1].lib.native_angle(degrees)

    def imu(self, unit, readings):
        """The unit's BNO085 reads `readings` - accel xyz m/s^2, gyro xyz rad/s, mag xyz uT,
        quaternion i j k real - in place of its tumble: a SIL's body."""
        with self.lock:
            self.boards[unit - 1].lib.native_imu((ctypes.c_double * 13)(*readings))


def limb_for(url):
    """The limb a URL names, powered at its first open in the process: a body's, its first
    without `bus`; a bus of `nodes`; else one board on `world` (bench unless named, none by
    `world=none`)."""
    query = urllib.parse.parse_qs(urllib.parse.urlsplit(url).query)
    if 'body' in query:
        from coaxial.simulated.link import bus_nodes

        kind = query['body'][0]
        key = 'native://?body=%s' % kind
        if key not in _LIMBS:
            _LIMBS[key] = {name: Limb(len(bus_nodes(name)), world)
                             for name, world in BODIES[kind].items()}
        limbs = _LIMBS[key]
        return limbs[query.get('bus', [sorted(limbs)[0]])[0]]
    if url not in _LIMBS:
        nodes = int(query.get('nodes', ['0'])[0])
        world = query.get('world', [None if nodes else 'bench'])[0]
        _LIMBS[url] = Limb(nodes, None if world == 'none' else world)
    return _LIMBS[url]


class Serial(SerialBase):
    """`native://` as pyserial opens it, a UART's line: what is written every board on it
    hears at the line's pace; a read takes what they said, waiting its timeout while the limb
    runs on the wall's time."""

    #: Wall s a virtual s: the board keeps real time.
    time_scale = 1.0

    def open(self):
        try:
            self._limb = limb_for(self.portstr or 'native://')
        except RuntimeError as exc:           # no compiler: said as a port that fails
            raise serial.SerialException(str(exc)) from exc
        #: The units a scan probes, and whether the port is a board's console.
        self.units = self._limb.units
        self.console = self._limb.console
        self._answer = bytearray()
        self.is_open = True

    def virtual_seconds(self):
        """The limb's clock, s."""
        return self._limb.seconds()

    def close(self):
        self.is_open = False

    def _reconfigure_port(self, force_update=False):
        pass

    @property
    def in_waiting(self):
        self._answer += self._limb.take()
        return len(self._answer)

    def read(self, size=1):
        deadline = time.monotonic() + (self.timeout if self.timeout is not None else 1e9)
        while True:
            self._answer += self._limb.take()
            if len(self._answer) >= size or time.monotonic() >= deadline:
                got, self._answer = bytes(self._answer[:size]), self._answer[size:]
                return got
            time.sleep(PACE_S)

    def write(self, data):
        data = bytes(data)
        self._limb.send(data)
        return len(data)

    def reset_input_buffer(self):
        self._limb.take()
        self._answer = bytearray()

    def reset_output_buffer(self):
        pass

    def flush(self):
        pass


if 'tools.cores' not in serial.protocol_handler_packages:
    serial.protocol_handler_packages.append('tools.cores')
