"""Her boards on Renode: an extremity bus mirrored onto live STM32H743 nodes from a thread of its own.

The 60 Hz page never waits on a UART: `EmulatedLimb.sync` hands it her latest state, the thread
writes each joint's angle to its board in turn."""
import math
import threading
from contextlib import suppress

from coaxial import Coaxial63100
from coaxial.comm.hostclock import clock_of
from coaxial.errors import LINK_FAULTS, RigError
from machine import drives, edge
from machine.modes import EMULATED


class EmulatedLimb:

    """One extremity bus (`left_arm`, 4 nodes) on live boards in Renode (`emulator://`)."""

    def __init__(self, port='emulator://', limb='left_arm'):
        self.limb = limb
        self.joints = dict(edge.LIMBS)[limb]
        bus = 'emulator://?nodes=%d&world=humanoid_arm' % len(self.joints) if port == 'emulator://' else str(port)
        self.rigs = [Coaxial63100(port=bus, unit=u, execution_mode=EMULATED).open()
                     for u in range(1, len(self.joints) + 1)]
        self.origin = self.rigs[0].origin
        for rig, j in zip(self.rigs, self.joints):
            if not drives.passive(j):
                with suppress(RigError):
                    rig.board.afe.on()
                    clock_of(rig).sleep(0.05)
                    rig.gates.on(bypass_sto=True, ignore_interlock=True)
                    rig.board.drive.configure(source='model')
                    rig.board.drive.hold()
        self._now, self._stop = None, threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def sync(self, now):
        self._now = now

    def badge(self, crc_err):
        return '5x10M %d emu %d sil %2d crc' % (len(self.rigs), 27 - len(self.rigs), crc_err)

    def closed_note(self):
        return '%d Renode boards (%s) closed' % (len(self.rigs), self.limb)

    def _run(self):
        k = 0
        while not self._stop.is_set():
            now = self._now
            if now is not None and now.get('angles'):
                j = self.joints[k % len(self.joints)]
                rig = self.rigs[k % len(self.joints)]
                k += 1
                if not drives.passive(j):
                    with suppress(*LINK_FAULTS, RigError):
                        rig.board.drive.write(theta=math.radians(float(now['angles'].get(j, 0.0))))
            self._stop.wait(0.05)

    def close(self):
        self._stop.set()
        self._thread.join(timeout=2.0)
        for rig in self.rigs:
            with suppress(*LINK_FAULTS, RigError):
                rig.board.drive.off()
                rig.gates.off()
            with suppress(Exception):
                rig.close()

