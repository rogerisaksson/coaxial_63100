#!/usr/bin/env python3
"""The host's bootloader client and front door against the C core: Boot's shapes, load, open()."""
import ctypes
import struct
import sys
import time
import zlib

from test_boot_core import (CHUNK, SEAL_MAGIC, SEALED, SESSION, STATE, STORE_BASE, TYPE, UID,
                            UNIT_BLANK, image, run)


class Wire:
    """A board for the host's own client, coaxial.devices.boot.Boot: each 0x6E
    request handed to boot_pdu as the bootloader's link hands it over."""

    unit = 2

    def __init__(self, lib):
        self.lib = lib

    def _pdu(self, payload):
        out = ctypes.create_string_buffer(253)
        n = self.lib.boot_h_pdu(bytes(payload), len(payload), out, 253)
        return n, out.raw[:max(n, 0)]

    def request(self, function, payload=b'', exact_payload=None, timeout=None,
                reply_shape=None):
        from coaxial import errors
        n, reply = self._pdu(payload)
        if n == -1:
            raise errors.NoReplyError('silence')
        if n < 0:
            raise errors.ModbusException(self.unit, function, 1 if n == -2 else 4)
        return reply

    def broadcast(self, function, payload=b'', settle=0.05):
        self._pdu(payload)


def test_the_hosts_client(report, node):
    """The master's client, byte for byte against the C: the replies carry
    the fields alone, as the application's do, and every shape parses."""
    from coaxial import errors
    from coaxial.devices.boot import Boot
    boot = Boot(Wire(node.lib))
    img = image(30 * CHUNK + 5)
    record = bytes(range(250))
    boot.hold(SESSION)
    who = boot.who()
    report.check('who parses: the uid, the type, held, the blank unit',
                 who == {'uid': UID.hex(), 'type': TYPE, 'state': 'held', 'unit': UNIT_BLANK}, who)
    report.check('a prefix that is not this node is silence, None to the client',
                 boot.who(8, b'\xff') is None)
    report.check('assign takes', boot.assign(UID.hex(), 2, 2) is True)
    state = boot.flash(TYPE, img, record, persist=True)
    report.check('the whole sequence through the client, persisted: sealed, valid, the image named',
                 state['state'] == 'sealed' and state['valid'] and state['unit'] == 2
                 and state['image'] == (len(img), zlib.crc32(img)) and state['flags'] == 0, state)
    report.check('missing parses a bitmap: nothing missing', boot.missing() == [])
    report.check('dump parses a page of the record', boot.dump(224) == (224, record[224:250] + b'\xff' * 198))
    try:
        boot.stay()
        said = None
    except errors.RigError as exc:
        said = str(exc)
    report.check('stay is refused in the node\'s own words', said == 'this node is in its bootloader already', said)


class Bench:
    """A transport with one node on it: its application answering `unit`
    (device 11 state and stay) until stay, then the C bootloader, and after
    go the application again, running what the bootloader verified."""

    port, baud, time_scale = 'bench', 115200, 1.0

    def __init__(self, lib, unit, running):
        self.lib, self.unit, self.running = lib, unit, running
        self.in_app, self.frames = True, 0

    def _app(self, unit, payload):
        from coaxial import errors
        if unit != self.unit or payload[:1] != b'\x0b':
            raise errors.NoReplyError('silence')
        if payload[1] == STATE:
            return (bytes([SEALED, TYPE, self.unit, 3]) + struct.pack('>II', 0, 0) + b'\x01'
                    + UID + struct.pack('>II', *self.running) + b'\x01')
        self.in_app = False                     # stay: the reset, RAM kept
        self.lib.boot_h_reboot()
        return b'\x01'

    def _core(self, payload):
        out = ctypes.create_string_buffer(253)
        return self.lib.boot_h_pdu(bytes(payload), len(payload), out, 253), out

    def request(self, unit, function, payload=b'', exact_payload=None, timeout=None,
                reply_shape=None):
        from coaxial import errors
        self.frames += 1
        if self.in_app:
            return self._app(unit, payload)
        if unit != self.lib.boot_h_unit():
            raise errors.NoReplyError('silence')
        n, out = self._core(payload)
        if n == -1:
            raise errors.NoReplyError('silence')
        if n < 0:
            raise errors.ModbusException(unit, function, 4)
        return out.raw[:n]

    def broadcast(self, function, payload=b'', settle=0.05):
        self.frames += 1
        if not self.in_app:
            self._core(payload)
            if self.lib.boot_h_go():
                got = (ctypes.c_uint32 * 2)()
                self.lib.boot_h_image(got)
                self.running, self.in_app = tuple(got), True

    @staticmethod
    def sleep(seconds):
        time.sleep(seconds)


def test_the_host_loads_its_image(report, node):
    """The old-firmware, new-host case: a node running another image takes
    the host's through its bootloader - its unit, position and termination
    given back, the store keeping it - and one running it already, or one
    a debugger started, is left alone."""
    from coaxial.devices import boot
    from coaxial.devices.board import Board
    boot.STAY_S, boot.GO_S = 0.0, 1.0
    img, old = image(20 * CHUNK + 3), image(20 * CHUNK + 3, version=6)
    bench = Bench(node.lib, 3, (len(old), zlib.crc32(old)))
    board = Board(bench, unit=3)
    report.check('another image running: loaded, and the application names the host\'s',
                 boot.ensure(board, img) is True
                 and bench.running == (len(img), zlib.crc32(img)) and bench.in_app)
    report.check('its unit, position and termination given back through assign',
                 node.unit == 3 and node.lib.boot_h_terminates() == 1
                 and node.op(STATE)[3] == 3)
    report.check('and the store keeps it, sealed',
                 struct.unpack('<III', node.read(STORE_BASE, 12))
                 == (SEAL_MAGIC, len(img), zlib.crc32(img)))
    frames = bench.frames
    report.check('running the host\'s image already: one state read, nothing loaded',
                 boot.ensure(board, img) is False and bench.frames == frames + 1)
    bench.running = (0, 0)
    report.check('started by a debugger (no image named): left alone',
                 boot.ensure(board, img) is False and bench.in_app)


def test_the_front_door_owns_the_image(report, node):
    """Coaxial63100.open()'s step on a real board: another image is loaded
    and said on stderr; a shared board is refused in words, not reset."""
    import contextlib
    import io as _io
    from coaxial import Coaxial63100
    from coaxial.devices import boot
    from coaxial.devices.board import Board
    from coaxial.errors import RigError
    from coaxial.comm.session import Origin
    boot.STAY_S, boot.GO_S = 0.0, 1.0
    img, old = image(12 * CHUNK), image(12 * CHUNK, version=5)
    real_host_image = boot.host_image
    boot.host_image = lambda: ('build/Debug/coaxial_63100.elf', img)
    try:
        for label, shared in (('COM9 - shared', True), ('COM9', False)):
            rig = Coaxial63100(port='COM9', unit=3)
            bench = Bench(node.lib, 3, (len(old), zlib.crc32(old)))
            rig._board = Board(bench, unit=3)
            rig._origin = Origin(True, 'COM9', 115200, None, label, 'debug probe', 3)
            said = _io.StringIO()
            try:
                with contextlib.redirect_stderr(said):
                    rig._board.probe = lambda tries=3: {}
                    rig._own_image()
                refused = None
            except RigError as exc:
                refused = str(exc)
            if shared:
                report.check('a shared board running another image: refused in words, not reset',
                             refused is not None and 'other sessions share' in refused
                             and bench.in_app and bench.running[1] == zlib.crc32(old), refused)
            else:
                report.check('an unshared one: loaded, said on stderr, recorded',
                             refused is None and rig.image_loaded == ('build/Debug/coaxial_63100.elf', True)
                             and bench.running == (len(img), zlib.crc32(img))
                             and 'loading this host\'s build' in said.getvalue(), said.getvalue())
    finally:
        boot.host_image = real_host_image


ROSTER = (test_the_hosts_client, test_the_host_loads_its_image,
          test_the_front_door_owns_the_image)


if __name__ == '__main__':
    sys.exit(run(ROSTER, 'bootclient'))
