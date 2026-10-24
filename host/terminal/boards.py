"""Her boards on Renode: one of her buses mirrored onto emulated boards from a thread of its own.

The page never waits on a UART: `EmulatedLimb.sync` hands it her latest state; the thread writes
each held joint's angle to its board in turn, at the pace the bus answers, through the line's noise
(`edge.LINE_BER`). A write lost is the next round's; a whole round lost is no longer sporadic: the
limb's drives off."""
import math
import threading
import urllib.parse
from contextlib import suppress

from coaxial import Coaxial63100
from coaxial.errors import LINK_FAULTS, RigError
from coaxial.node import arm_stage
from coaxial.simulated.link import SIMULATED_BUSES
from machine import drives, edge
from machine.modes import EMULATED
from tools.emu.protocol_emulator import BODIES


class EmulatedLimb:

    """One of her buses - a label of the stand-in's fleet (`coaxial.simulated.link`) - on boards
    of its own on Renode (`emulator://`), in the body's world for it; what `port` asks of the
    emulator laid over that."""

    def __init__(self, port='emulator://', bus='LA'):
        self.limb, nodes = SIMULATED_BUSES[bus]
        units = sorted(nodes)
        self.joints = [nodes[unit][2].replace(' ', '_') for unit in units]
        asked = dict(urllib.parse.parse_qsl(urllib.parse.urlsplit(str(port)).query))
        url = 'emulator://?' + urllib.parse.urlencode(
            {'nodes': len(units), 'world': BODIES['humanoid'][bus], 'ber': edge.LINE_BER, **asked})
        self.rigs = [Coaxial63100(port=url, unit=unit, execution_mode=EMULATED).open()
                     for unit in units]
        self.origin = self.rigs[0].origin
        self.held = [(rig, joint) for rig, joint in zip(self.rigs, self.joints)
                     if drives.passive(joint) is None]
        self.faults, self.shutdown, self._now = 0, None, {}
        for rig, _joint in self.held:
            try:
                arm_stage(rig)
                rig.board.drive.configure(source='model')
                rig.board.drive.hold()
            except RigError:
                self.faults += 1
        self._shown, self._stop = threading.Event(), threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def sync(self, now):
        """The frame the page shows: its angles the drives' next."""
        self._now = now
        self._shown.set()

    def badge(self, crc_err):
        """The BUSES box's modbus row: her buses, her drives on Renode, the CRC errors and apart
        the emulated link's faults - a write timed out or refused -, SHUTDOWN past a round lost."""
        return ('%dx10M %d emu crc %d lnk %d' % (len(edge.LIMBS), len(self.rigs), crc_err, self.faults)
                + ' SHUTDOWN' * bool(self.shutdown))

    def closed_note(self):
        return '%d Renode boards (%s) closed' % (len(self.rigs), self.limb)

    def _run(self):
        lost = 0
        while self._shown.wait() and not self._stop.is_set() and lost < len(self.held):
            self._shown.clear()
            angles = self._now['angles']
            for rig, joint in self.held:
                try:
                    rig.board.drive.write(theta=math.radians(angles[joint]))
                    lost = 0
                except LINK_FAULTS + (RigError,):
                    self.faults, lost = self.faults + 1, lost + 1
        if lost:
            self.shutdown = '%s: %d writes lost in a row, a round of its boards: drives off' % (self.limb, lost)
            self._off()

    def _off(self):
        for rig, _joint in self.held:
            with suppress(*LINK_FAULTS, RigError):
                rig.board.drive.off()
                rig.gates.off()

    def close(self):
        self._stop.set()
        self._shown.set()
        self._thread.join()
        self._off()
        for rig in self.rigs:
            rig.close()
