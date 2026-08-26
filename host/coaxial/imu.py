"""The BNO08X on SPI2: what it says it is, and what it reports.

The board clocks SHTP cargoes and does not interpret them. This is where a
cargo becomes reports and a report's counts become a physical quantity, which
is the same division the ADC channels keep: the firmware reports raw
fixed-point integers, and the Q point belongs here.

Datasheet references are to BNO080_085 v1.17, in datasheets/.
"""
from . import protocol
from .errors import DeviceStateError
from .subsystem import Subsystem
from .wire import Reader

RESET_CAUSES = {
    0: 'not applicable', 1: 'power on reset', 2: 'internal system reset',
    3: 'watchdog timeout', 4: 'external reset', 5: 'other',
}
"""Byte 1 of the product id response, Figure 1-29."""

CHANNELS = {
    0: 'command', 1: 'executable', 2: 'control', 3: 'input',
    4: 'wake input', 5: 'gyro rotation vector',
}
"""The six SHTP channels, section 1.3.1."""

ACCURACY = {0: 'unreliable', 1: 'low', 2: 'medium', 3: 'high'}
"""Status bits 1:0 of an input report, section 1.3.5.2."""

REPORTS = {
    0x01: 'accelerometer', 0x02: 'gyroscope', 0x03: 'magnetic field',
    0x04: 'linear acceleration', 0x05: 'rotation vector', 0x06: 'gravity',
    0x08: 'game rotation vector', 0xFB: 'timebase',
}

# Q point per report: the fixed-point counts are divided by 2**q to give the
# unit named beside it. Only the reports whose length this repository's
# datasheet actually tabulates are here - see shtp.c on why the quaternion
# reports are absent rather than guessed.
SCALE = {
    0x01: (8, 'm/s^2'),
    0x02: (9, 'rad/s'),
    0x03: (4, 'uT'),
    0x04: (8, 'm/s^2'),
    0x06: (8, 'm/s^2'),
}


class Imu(Subsystem):
    """The BNO08X behind SPI2. Every call raises rather than returning a
    status: a reading that did not happen is not a reading of zero."""

    def product_id(self):
        """What the part says it is - the answer that proves the link.

        Raises DeviceStateError when the board reached SPI2 but the part
        never sent a product id response, which is what an unpowered or
        mis-strapped BNO08X looks like from here.
        """
        try:
            reply = self.request(protocol.IMU, bytes([protocol.IMU_OP_ID]))
        except Exception as exc:
            raise DeviceStateError(
                'the IMU did not answer a product id request: %s. The board '
                'drives NRSTN and PS0/WAKE and waits on H_INTN, so this is '
                'the part itself not answering - check it is populated and '
                'that PS1 is strapped high for SPI' % exc) from exc

        r = Reader(reply)
        cause = r.u8()
        return {
            'reset_cause': cause,
            'reset_cause_name': RESET_CAUSES.get(cause, 'reserved'),
            'sw_version': '%d.%d.%d' % (r.u8(), r.u8(), 0),
            'sw_part': r.u32(),
            'sw_build': r.u32(),
            'sw_patch': r.u16(),
        }

    def read(self):
        """One SHTP cargo, decoded as far as the datasheet allows.

        `cargo` is always the raw bytes. `reports` holds the ones whose
        layout is known; an unknown report id ends the walk rather than
        mis-framing what follows it, so a short `reports` beside a long
        `cargo` means exactly that.
        """
        reply = self.request(protocol.IMU, bytes([protocol.IMU_OP_READ]))
        r = Reader(reply)
        channel = r.u8()
        length = r.u8()
        cargo = bytes(r.take(length)) if length else b''

        return {
            'channel': channel,
            'channel_name': CHANNELS.get(channel, 'unknown'),
            'cargo': cargo,
            'reports': decode(cargo),
        }

    def feature(self, report_id, interval_us):
        """Enable a sensor report, or disable it with an interval of 0.

        The part may adopt a different period than the one asked for; it says
        so in a Get Feature Response, which arrives through read().
        """
        if not 0 <= report_id <= 0xFF:
            raise ValueError('report id %r is not a byte' % (report_id,))
        if not 0 <= interval_us <= 0xFFFFFFFF:
            raise ValueError('interval %r does not fit 32 bits'
                             % (interval_us,))
        # Big-endian: every integer on this board's wire is, and wire.c's
        # rd_u32 reads it that way. Sent little-endian, 60000 us arrived as
        # 0x60EA0000 - about 27 minutes between reports, which looks exactly
        # like a sensor that was never enabled.
        self.request(protocol.IMU,
                     bytes([protocol.IMU_OP_FEATURE, report_id])
                     + interval_us.to_bytes(4, 'big'))


def report_length(report_id):
    """Bytes one input report occupies, or 0 when it is not known here.

    The same table shtp.c keeps, for the same reason: reports are packed back
    to back and are not self-delimiting, so a length nobody checked
    mis-frames every byte after it.
    """
    if report_id == 0xFB:
        return 5
    if report_id in (0x01, 0x02, 0x03, 0x04, 0x06):
        return 10
    return 0


def decode(cargo):
    """Walk a cargo into reports, stopping at the first id without a length."""
    out = []
    at = 0
    while at < len(cargo):
        report_id = cargo[at]
        step = report_length(report_id)
        if step == 0 or at + step > len(cargo):
            break
        out.append(_one(cargo, at, report_id))
        at += step
    return out


def _one(cargo, at, report_id):
    """One report, with its counts and - where the Q point is known - a
    physical quantity beside them. The counts are always present; the scaled
    value is not, and its absence says the Q point is not established here."""
    named = REPORTS.get(report_id, 'unknown 0x%02X' % report_id)

    if report_id == 0xFB:
        delta = int.from_bytes(cargo[at + 1:at + 5], 'little', signed=True)
        return {'report_id': report_id, 'name': named,
                'base_delta_100us': delta}

    status = cargo[at + 2]
    axes = [int.from_bytes(cargo[at + 4 + 2 * i:at + 6 + 2 * i],
                           'little', signed=True) for i in range(3)]
    row = {
        'report_id': report_id,
        'name': named,
        'seq': cargo[at + 1],
        'accuracy': ACCURACY.get(status & 3, 'unknown'),
        'delay_100us': ((status >> 2) << 8) | cargo[at + 3],
        'raw': axes,
    }

    scale = SCALE.get(report_id)
    if scale is not None:
        q, unit = scale
        row['scaled'] = [v / float(1 << q) for v in axes]
        row['unit'] = unit
    return row
