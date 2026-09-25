"""`native://`: the firmware's comms/ and board layer on this host, at the part's pace.

    import tools.cores.native               # registers the scheme
    Coaxial63100(port='native://').open()
    Coaxial63100(port='native://?world=bench').open()

The stage, the triple, the meter and the drive over the chip and the world core's plant: the
fake board (tools.cores.fakeboard) with board/src's board_pwm.c, board_sync.c, board_adc.c
and board_drive.c on board/native/native.c's TIM1, ADC1-3 and front end - Renode's models in
C, the plant stepped between the timer's edges. Not the Cortex-M: the C built for this host,
the float math on its FPU. A thread holds the board's clock to the wall's - real time - and an
exchange takes its line time off the wall. The board powers up at the first open in a process
and runs on across sessions. Renode (emulator://) is the image itself; this is its code, fast.
"""
import ctypes
import os
import re
import threading
import time
import urllib.parse

import serial
from serial.serialutil import SerialBase

from tools import REPO
from tools.cores import fakeboard
from tools.cores.build import build, find_cc
from tools.emu import world as worlds

SOURCES = fakeboard.SOURCES + [
    os.path.join(REPO, 'board', 'src', name)
    for name in ('board_pwm.c', 'board_sync.c', 'board_adc.c', 'board_drive.c', 'board_ctrl.c',
                 'board_log.c')] + [
    os.path.join(REPO, 'world', 'src', name) for name in ('world.c', 'world_emu.c')] + [
    os.path.join(REPO, 'board', 'native', 'native.c')]

#: board/native first: its stm32h7xx.h, board_hw.h and board_irq.h stand in for the part's.
INCLUDES = ([os.path.join(REPO, 'board', 'native')] + fakeboard.INCLUDES
            + [os.path.join(REPO, 'world', 'inc')])

#: How often the pacing thread brings the board's clock up to the wall's, and the most it
#: runs under one hold of the lock an exchange waits behind, s. A virtual second runs in
#: 5 ms here: the default 15.6 ms timer and one 5 ms run a wake held it to a third of real
#: time (2026-09-25).
PACE_S = 0.001
BURST_S = 0.005

#: How far the board's clock stands from the wall's with a core for the pacer: a wake, a burst,
#: a scheduler's slice (Windows' 15.6 ms), s.
TRAIL_S = 0.02

#: The front end's phase transfer and spread, LTspice's (tools/emu/afe_spice.py).
AFE_REPL = os.path.join(REPO, 'board', 'emu', 'coaxial_63100_afe.repl')
AFE_KEYS = ('PhaseVoltsPerAmp', 'PhaseZeroVolts', 'PhaseGainSigma', 'PhaseZeroSigmaVolts',
            'BusGainSigma')

#: Unit 1's noise seed, as tools.emu seeds the emulator's front end.
AFE_SEED = 0x63100001

_BUILT = []
#: The one board in this process, and the lock its calls take.
_LOCK = threading.Lock()
_POWERED = []


def library():
    """The native board, built on first use."""
    if not _BUILT:
        cc = find_cc()
        if cc is None:
            raise RuntimeError('no C compiler for the native board')
        path, _warnings = build(cc, SOURCES, INCLUDES, 'native_%d' % os.getpid())
        lib = ctypes.CDLL(path)
        lib.fake_exchange.restype = ctypes.c_uint16
        lib.fake_exchange.argtypes = [ctypes.c_char_p, ctypes.c_uint16,
                                      ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint16]
        lib.native_run.argtypes = [ctypes.c_uint32]
        lib.native_seconds.restype = ctypes.c_double
        lib.native_afe.argtypes = [ctypes.c_double] * 5 + [ctypes.c_uint32]
        for name in ('emu_world_body', 'emu_world_load', 'emu_plant_attach'):
            getattr(lib, name).restype = None
        _BUILT.append(lib)
    return _BUILT[0]


def _afe(lib):
    """The front end's transfer and this board's errors, from the repl the emulator loads."""
    with open(AFE_REPL, encoding='utf-8') as f:
        text = f.read()
    values = []
    for key in AFE_KEYS:
        found = re.search(r'%s: ([0-9.eE+-]+)' % key, text)
        if found is None:
            raise RuntimeError('%s carries no %s - tools/emu/afe_spice.py writes it'
                               % (AFE_REPL, key))
        values.append(float(found.group(1)))
    lib.native_afe(*values, AFE_SEED)


def _plant(lib, name):
    """The world `name` (board/emu/worlds), its first node this board's motor."""
    world = worlds.load(name)
    body = world.get('body', {})
    lib.emu_world_reset(len(world['nodes']))
    lib.emu_world_body(worlds.BODIES[body.get('kind', 'ground')],
                       *[ctypes.c_float(v) for v in worlds._values(body, worlds.BODY)])
    for node, spec in enumerate(world['nodes']):
        load = spec.get('load', {})
        lib.emu_world_load(node, worlds.LOADS[load.get('kind', 'free')],
                           *[ctypes.c_float(v) for v in worlds._values(load, worlds.LOAD)])
        lib.emu_plant_attach(node, *[ctypes.c_float(v) for v in worlds._motor(spec['motor'])])


class Serial(SerialBase):
    """`native://` as pyserial opens it: what is written is one exchange with the board, its
    answer what the next reads take; between exchanges its clock keeps the wall's."""

    #: Wall s a virtual s: the board keeps real time.
    time_scale = 1.0
    #: The units a scan probes.
    units = (1,)
    #: The console port, binary as a host's 'm' leaves it.
    console = True

    def open(self):
        query = urllib.parse.parse_qs(urllib.parse.urlsplit(self.portstr or '').query)
        self._lib = library()
        self._lock = _LOCK
        with self._lock:
            if not _POWERED:
                self._lib.native_open()
                _afe(self._lib)
                _plant(self._lib, query.get('world', ['bench'])[0])
                _POWERED.append(True)
            self._start = time.monotonic() - self._lib.native_seconds()
        self._answer = bytearray()
        self._stop = threading.Event()
        self._pacer = threading.Thread(target=self._pace, name='native pacer', daemon=True)
        self.is_open = True
        self._pacer.start()

    def _pace(self):
        """The board's clock held to the wall's."""
        timer = getattr(ctypes, 'windll', None)
        if timer is not None:
            timer.winmm.timeBeginPeriod(1)
        try:
            while not self._stop.wait(PACE_S):
                while not self._stop.is_set():
                    with self._lock:
                        behind = (time.monotonic() - self._start) - self._lib.native_seconds()
                        if behind < PACE_S:
                            break
                        self._lib.native_run(int(min(behind, BURST_S) * 1e6))
        finally:
            if timer is not None:
                timer.winmm.timeEndPeriod(1)

    def virtual_seconds(self):
        """The board's clock, s."""
        return self._lib.native_seconds()

    def close(self):
        if getattr(self, '_stop', None) is not None:
            self._stop.set()
            self._pacer.join()
        self.is_open = False

    def _reconfigure_port(self, force_update=False):
        pass

    @property
    def in_waiting(self):
        return len(self._answer)

    def read(self, size=1):
        got, self._answer = bytes(self._answer[:size]), self._answer[size:]
        return got

    def write(self, data):
        data = bytes(data)
        out = (ctypes.c_uint8 * fakeboard.ANSWER)()
        with self._lock:
            got = self._lib.fake_exchange(data, len(data), out, fakeboard.ANSWER)
            ahead = self._lib.native_seconds() - (time.monotonic() - self._start)
        # The line's time and the board's answer, off the wall as a UART's are.
        if ahead > 0.0:
            time.sleep(ahead)
        self._answer += bytes(out[:got])
        return len(data)

    def reset_input_buffer(self):
        self._answer = bytearray()

    def reset_output_buffer(self):
        pass

    def flush(self):
        pass


if 'tools.cores' not in serial.protocol_handler_packages:
    serial.protocol_handler_packages.append('tools.cores')
