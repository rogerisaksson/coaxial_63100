"""The two SPI sensors: a BNO08X that tumbles and an A1335 that follows the
simulated shaft.
"""
import math
import random
import time

from .. import angle, imu
from ..sensor import PolledSensor
from ..scaling import KELVIN_AT_ZERO_C
from .values import _tumble
from typing import Any
from ..imu import CHANNELS, decode


class SimulatedImu(PolledSensor):
    """A BNO08X that was never soldered on."""

    #: Q8 counts for 9.81 m/s^2, which is what SCALE[0x01] divides by.
    ONE_G = 2511

    #: Q14 counts for 1.0, the scale a rotation vector is reported in.
    UNIT = 16384

    def __init__(self):
        self._seq = 0
        self._enabled = {}
        self._updates = 0
        self._held = False

    def product_id(self):
        return {
            'reset_cause': 1,
            'reset_cause_name': 'power on reset',
            'sw_version': 'simulated',
            'sw_part': 0,
            'sw_build': 0,
            'sw_patch': 0,
        }

    def read(self):
        """A timebase, then whatever has been enabled - framed the way a real
        cargo on channel 3 is, Figure 5-2.
        """
        self._seq = (self._seq + 1) & 0xFF
        cargo = bytes([0xFB, 0, 0, 0, 0])

        wanted = set(self._enabled) or {0x01}

        if 0x01 in wanted:
            cargo += bytes([0x01, self._seq, 0x03, 0])
            for value in (0, 0, self.ONE_G):
                cargo += int(value).to_bytes(2, 'little', signed=True)

        if 0x05 in wanted:
            cargo += bytes([0x05, self._seq, 0x03, 0])
            for value in _tumble(self._seq, self.UNIT) + (0,):
                cargo += int(value).to_bytes(2, 'little', signed=True)

        return {'channel': 3, 'channel_name': CHANNELS[3],
                'cargo': cargo, 'reports': decode(cargo)}

    def feature(self, report_id, interval_us):
        if not 0 <= report_id <= 0xFF:
            raise ValueError('report id %r is not a byte' % (report_id,))
        if not 0 <= interval_us <= 0xFFFFFFFF:
            raise ValueError('interval %r does not fit 32 bits'
                             % (interval_us,))
        if interval_us:
            self._enabled[report_id] = interval_us
        else:
            self._enabled.pop(report_id, None)

    def state(self):
        self._updates += 17
        got = {'loop': 'held' if self._held else 'running',
               'error': 'none', 'last_fault': 'none', 'last_fault_id': 0,
               # The board reports what it asked the part for; a stand-in
               # without it crashed the first view that read the field.
               'feature': {'report_id': 0x05, 'interval_us': 20000,
                           'pending': False},
               'updates': self._updates,
               'cargoes': self._updates, 'errors': 0}
        for report in self.read()['reports']:
            if 'quaternion' not in report:
                continue
            # The same shape the real state() builds: the counts the part sent
            # and the quaternion this host divided out of them.
            got.update({
                'report_id': report['report_id'],
                'name': report['name'],
                'accuracy': report.get('accuracy', 'unknown'),
                'counts': dict(zip(('i', 'j', 'k', 'real'), report['raw'])),
                'quaternion': report['quaternion'],
            })
            self._vectors(got)
            return got
        got['quaternion'] = None
        self._vectors(got)
        return got

    #: Q points the part uses, the same table `coaxial.imu` divides
    #: by: accelerometer Q8 in m/s^2, gyroscope Q9 in rad/s,
    #: magnetometer Q4 in uT.
    VECTORS = (('accelerometer', 0x01, 8, 'm/s^2', (0.0, 0.0, 9.81)),
               ('gyroscope', 0x02, 9, 'rad/s', (0.0, 0.0, 0.0)),
               ('magnetometer', 0x03, 4, 'uT', (22.0, -3.0, 41.0)))

    def _vectors(self, got):
        """The three vectors, present only when enabled."""

        for name, report, bits, unit, rest in self.VECTORS:
            if report not in self._enabled:
                got[name] = None
                continue
            value = [v + random.gauss(0.0, 0.02) for v in rest]
            got[name] = {
                'accuracy': 'high',
                'unit': unit,
                'counts': dict(zip('xyz',
                                   (int(v * (1 << bits))
                                    for v in value))),
                'value': dict(zip('xyz', value))}

    def latest(self):
        return self.state()['quaternion']

    def hold(self):
        self._held = True
        return 'held'

    def resume(self):
        self._held = False
        return 'running'

    def reset(self):
        return 3        # the advertisement and the two announcements

    def write(self, channel, payload):
        if not 0 <= channel <= 5:
            raise ValueError('channel %r is not one of the six' % (channel,))

    def probe(self, length=4, select=True):
        return {'kernel_hz': 190000000, 'bitrate_hz': 1484375,
                'raw': bytes(length)}

    def pins(self):
        return [{'pin': 'PB%d' % p, 'signal': name, 'bits': imu.ALL_BITS,
                 'held': False} for p, name in sorted(imu.SPI2_PINS.items())]

    def wake_test(self, ms=200):
        return 0


class SimulatedAngle(PolledSensor):
    """The A1335 without an A1335."""
    def __init__(self):
        self._at = time.monotonic()
        self._updates = 0
        self._reg = 0x20
        self._held = False
        #: The board wires these: the rotor the shaft follows, and the
        #: thermal stand-in the die is as warm as.
        self.drive: Any = None
        self.thermal: Any = None
    def _turn(self):
        """The shaft in counts: the virtual rotor's when one is turning,
        else one invented turn every twelve seconds - a stand-in that
        reports one angle for ever is indistinguishable from a dead link.
        """
        drive = self.drive
        if drive is not None and drive._source == 'model':
            drive.model()                      # advance to now
            return int(drive._mech
                       / (2.0 * math.pi) * 4096.0) % 4096
        return int(((time.monotonic() - self._at) / 12.0) * 4096.0) % 4096

    def _value(self, register):
        if register == angle.ANG:
            return 0x5000 | self._turn()
        if register == angle.TSEN:
            # The die sits on the board: its temperature is the thermal
            # stand-in's board node when the board wired one, else a room's 296
            # K.
            thermal = self.thermal
            kelvin = (KELVIN_AT_ZERO_C + thermal.state()['nodes']['board']
                      if thermal is not None else 296.0)
            return 0xF000 | (int(kelvin * 8.0) & 0x0FFF)
        if register == angle.FIELD:
            return 0xE000 | 380                # gauss, a magnet in place
        return 0x8000

    def state(self):
        self._updates += 37
        value = self._value(self._reg)
        got = {
            'loop': 'held' if self._held else 'running',
            'error': 'none', 'updates': self._updates, 'errors': 0,
            'register': self._reg,
            'register_name': angle.REGISTERS.get(self._reg,
                                                 '0x%02X' % self._reg),
            'value': value, 'crc': 0,
        }
        if self._reg == angle.ANG:
            got['degrees'] = angle.degrees(value)
            got['flags'] = value >> 12
        elif self._reg == angle.TSEN:
            got['kelvin'] = angle.kelvin(value)
        return got

    def read(self, register):
        if not 0 <= register <= 0x3F:
            raise ValueError('register %r is past the six address bits'
                             % (register,))
        return {'register': register,
                'register_name': angle.REGISTERS.get(register,
                                                     '0x%02X' % register),
                'value': self._value(register), 'crc': 0}

    def write(self, register, value):
        if not 0 <= register <= 0x3F:
            raise ValueError('register %r is past the six address bits'
                             % (register,))

    def poll_register(self, register=None):
        if register is not None:
            self._reg = register
        return {'register': self._reg,
                'register_name': angle.REGISTERS.get(self._reg,
                                                     '0x%02X' % self._reg)}

    def clock(self):
        return {'kernel_hz': 118750000, 'bitrate_hz': 1855468}

    def hold(self):
        self._held = True
        return 'held'

    def resume(self):
        self._held = False
        return 'running'
