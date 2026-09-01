"""The BNO08X on SPI2: what it says it is, and what it reports.

The board clocks SHTP cargoes and does not interpret them. This is where a
cargo becomes reports and a report's counts become a physical quantity, which
is the same division the ADC channels keep: the firmware reports raw
fixed-point integers, and the Q point belongs here.

Datasheet references are to BNO080_085 v1.17, in datasheets/.
"""
import contextlib

from . import protocol
from .errors import DeviceStateError, RigError
from .sensor import PolledSensor
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
    0x05: (14, ''),
    0x08: (14, ''),
}

LOOP_STATES = {0: 'off', 1: 'init', 2: 'running', 3: 'held'}
"""What the board's IMU poll loop is doing. 'off' means AFE_ON is low."""

LOOP_ERRORS = {
    0: 'none', 1: 'lost AFE_ON', 2: 'the part did not come up',
    3: 'a cargo read failed', 4: 'a report id with no length',
    5: 'wrote without an H_INTN acknowledge - the part never asked',
}
"""The last thing that went wrong in the poll loop, not a running tally."""


def _signed16(value):
    """A 16-bit count off the wire, which carries them unsigned."""
    return value - 0x10000 if value & 0x8000 else value


QUATERNIONS = (0x05, 0x08)
"""Reports whose four fields are i, j, k, real rather than three axes. Q14,
and unitless: a rotation vector is a direction, not a quantity."""


class Imu(Subsystem, PolledSensor):
    """The BNO08X behind SPI2. Every call raises rather than returning a
    status: a reading that did not happen is not a reading of zero."""

    def _op(self, op, payload=b''):
        """One 0x6E request for this device.

        The device byte lives here and nowhere else: 0x6E carries every
        peripheral, chosen by it, because the specification's user-defined
        function codes are spent.
        """
        try:
            return self.request(protocol.DEVICE,
                                bytes([protocol.DEVICE_IMU, op])
                                + bytes(payload))
        except RigError as exc:
            raise self._explain(op, exc) from exc

    #: Ops that do not drive the bus, so a running poll loop cannot be their
    #: problem: the shared record, and hold and resume themselves.
    FREE_OPS = frozenset((protocol.IMU_OP_LATEST, protocol.IMU_OP_HOLD,
                          protocol.IMU_OP_RESUME))

    def _explain(self, op, exc):
        """Turn a bare device failure into what to do about it.

        THE BOARD REFUSES A BUS OP UNLESS THE POLL LOOP IS HELD - two masters
        on one bus is a cargo split between them - and the refusal arrives as
        a plain device error with nothing said. Measured 2026-08-29: that read
        as a dead part for an hour, through a reflash of an older firmware to
        rule out a regression that was never there. The part was reporting the
        whole time; every call had simply been made with the loop running.

        The loop's state needs no hold to read, so the reason is one round
        trip away - and only on the path that has already failed.
        """
        if op in self.FREE_OPS:
            return exc
        try:
            loop = self.state()['loop']
        except Exception:                  # noqa: BLE001 - the first failure
            return exc                     # is the one worth reporting
        if loop == 'held':
            return exc
        return DeviceStateError(
            'the IMU poll loop owns SPI2, so this was refused - it is %r, not '
            'held. Two masters on one bus is a cargo split between them. '
            'hold(), then this call, then resume(). Reading the shared record '
            'through latest() needs no hold, which is what it is for. '
            'The board said: %s' % (loop, exc))

    def product_id(self):
        """What the part says it is - the answer that proves the link.

        Raises DeviceStateError when the board reached SPI2 but the part
        never sent a product id response, which is what an unpowered or
        mis-strapped BNO08X looks like from here.
        """
        try:
            reply = self._op(protocol.IMU_OP_ID)
        except Exception as exc:
            raise DeviceStateError(
                'the IMU did not answer a product id request: %s. '
                'AFE_ON powers this part - if it is off, that is the whole '
                'answer and afe.enable() is the fix. With it on, the board '
                'drives NRSTN and PS0/WAKE and waits on H_INTN, so a silent '
                'part is the part itself: check it is populated and that '
                'PS1 is strapped high for SPI' % exc) from exc

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
        reply = self._op(protocol.IMU_OP_READ)
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

    def state(self):
        """The poll loop's shared record: what it saw and what went wrong.

        The board polls the part from its main loop and writes here; a host
        only ever reads. One round trip, and no SPI in the command path -
        reading a cargo per request cost 45 ms each and caught one in eight.

        `updates` is monotonic, so the same reading read twice is telling
        rather than a guess from the values.
        """
        reply = self._op(protocol.IMU_OP_LATEST)
        r = Reader(reply)

        got = {
            'loop': LOOP_STATES.get(r.u8(), 'unknown'),
            'error': LOOP_ERRORS.get(r.u8(), 'unknown'),
            'updates': r.u32(),
            'cargoes': r.u32(),
            'errors': r.u32(),
        }

        if r.u8():
            report_id = r.u8()
            status = r.u8()
            counts = [_signed16(r.u16()) for _ in range(4)]
            divisor = float(1 << SCALE[report_id][0])

            got.update({
                'report_id': report_id,
                'name': REPORTS.get(report_id, 'unknown 0x%02X' % report_id),
                'accuracy': ACCURACY.get(status & 0x03, 'unknown'),
                'counts': dict(zip(('i', 'j', 'k', 'real'), counts)),
                'quaternion': dict(zip(('i', 'j', 'k', 'real'),
                                       (c / divisor for c in counts))),
            })
        else:
            got['quaternion'] = None

        # AFTER the report block, because the board writes it after. This was
        # read before it, so with a report present the interval came back as
        # 16616704 instead of 20000 - four bytes taken out of the middle of a
        # quaternion. One branch returning early is what let the two orders
        # disagree without either looking wrong.
        got['feature'] = {'report_id': r.u8(),
                          'interval_us': r.u32(),
                          'pending': bool(r.u8())}

        # The last error that was not 'none', kept by the board. `error`
        # above is whatever the most recent poll saw, which at 400 reports
        # a second is 'none' every time a host looks.
        got['last_fault'] = LOOP_ERRORS.get(r.u8(), 'unknown')
        got['last_fault_id'] = r.u8()

        # The three vectors, appended by MINOR 6 and read only if
        # they are there - a board older than that answers a reply
        # that stops above, and a decoder that assumed the bytes
        # would raise on one that is simply older.
        for name, report in (('accelerometer', 0x01),
                             ('gyroscope', 0x02),
                             ('magnetometer', 0x03)):
            got[name] = self._vector(r, report)
        return got

    @staticmethod
    def _vector(r, report_id):
        """One three-axis report, or None when it never arrived.

        EACH CARRIES ITS OWN `have`. A feature nobody enabled leaves
        zeros, and zero is a legal reading - so the flag is what
        tells a caller the difference, not the value.
        """
        if not r.remaining:
            return None
        have = bool(r.u8())
        status = r.u8()
        counts = [r.i16(), r.i16(), r.i16()]
        if not have:
            return None
        bits, unit = SCALE[report_id]
        divisor = float(1 << bits)
        return {'accuracy': ACCURACY.get(status & 0x03, 'unknown'),
                'unit': unit,
                'counts': dict(zip('xyz', counts)),
                'value': dict(zip('xyz',
                                  (c / divisor for c in counts)))}

    def latest(self):
        """The newest quaternion, or None when the loop has not seen one."""
        return self.state()['quaternion']

    def hold(self):
        """Stop the poll loop so the part can be configured.

        Every operation that drives SPI2 - feature, write, reset, product_id,
        probe - is refused while the loop runs, because both would be masters
        on one bus. Returns the loop state the board reports back.
        """
        reply = self._op(protocol.IMU_OP_HOLD)
        return LOOP_STATES.get(Reader(reply).u8(), 'unknown')

    def resume(self):
        """Start the poll loop again, through init - the usual reason to have
        held it was a reset, and the part needs bringing up after one."""
        reply = self._op(protocol.IMU_OP_RESUME)
        return LOOP_STATES.get(Reader(reply).u8(), 'unknown')

    @contextlib.contextmanager
    def configuring(self):
        """Hold the loop for the block, and resume it however the block ends.

        The sequence the board requires, written once: leaving the loop held
        because a call raised is an IMU that has silently stopped reporting.
        """
        self.hold()
        try:
            yield self
        finally:
            self.resume()

    def reset(self):
        """Pulse NRSTN and collect what the part says coming up.

        The way back from a part that has stopped streaming. Returns how
        many cargoes the reset produced - three is the advertisement and the
        two announcements, and nothing at all means it did not come up.
        """
        reply = self._op(protocol.IMU_OP_RESET)
        return Reader(reply).u8()

    def wake_test(self, ms=200):
        """Milliseconds for H_INTN to answer PS0/WAKE on a drained part.

        None when it never answered inside `ms`, which is a part that will
        not accept a write; 'busy' when it was still holding the line low and
        the question could not be put.
        """
        reply = self._op(protocol.IMU_OP_WAKE, ms.to_bytes(2, 'big'))
        got = Reader(reply).u16()
        if got == 0xFFFF:
            return None
        if got == 0xFFFE:
            return 'busy'
        return got

    def pins(self):
        """Drive and release each of SPI2's four pins, and say what read back.

        `held` names a pin something else is holding: it did not follow the
        MCU driving it, or it did not follow the MCU's own pull. Reads work
        and chip select is proven, so this is what is left to check from
        inside the firmware.
        """
        reply = self._op(protocol.IMU_OP_PINS)
        r = Reader(reply)
        names = {12: 'NSS/H_CSN', 13: 'SCK', 14: 'MISO', 15: 'MOSI'}
        out = []
        for _ in range(4):
            pin, bits = r.u8(), r.u8()
            out.append({'pin': 'PB%d' % pin, 'signal': names.get(pin, '?'),
                        'bits': bits, 'held': bits != 0x0F})
        return out

    def probe(self, length=4, select=True):
        """`length` raw bytes off SPI2, unframed and uninterpreted.

        The bring-up question the parser refuses both answers to: FF FF FF FF
        is a part that is absent or in reset, 00 00 00 00 one that is present
        and idle. Also the only way to see the header's true length field,
        which read() caps before the host sees it.
        """
        reply = self._op(protocol.IMU_OP_PROBE,
                         bytes([length, 1 if select else 0]))
        r = Reader(reply)
        kernel, bitrate = r.u32(), r.u32()
        return {'kernel_hz': kernel, 'bitrate_hz': bitrate,
                'raw': bytes(r.take(r.u8()))}

    def write(self, channel, payload):
        """Put `payload` on `channel` as one SHTP cargo, unparsed.

        The bring-up primitive: what feature() and product_id() are built on,
        exposed because a question with an answer nothing else produces is
        the only way to prove a write reached the part.
        """
        if not 0 <= channel <= 5:
            raise ValueError('channel %r is not one of the six' % (channel,))
        self._op(protocol.IMU_OP_WRITE, bytes([channel]) + bytes(payload))

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
        self._op(protocol.IMU_OP_FEATURE,
                 bytes([report_id]) + interval_us.to_bytes(4, 'big'))


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
    # The datasheet does not tabulate these two; CEVA's own decoder does -
    # github.com/ceva-dsp/sh2, sh2_SensorValue.c. The rotation vector carries
    # i, j, k, real and an accuracy estimate behind the common header; the
    # game rotation vector the same without the estimate.
    if report_id == 0x05:
        return 14
    if report_id == 0x08:
        return 12
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
    fields = 4 if report_id in QUATERNIONS else 3
    axes = [int.from_bytes(cargo[at + 4 + 2 * i:at + 6 + 2 * i],
                           'little', signed=True) for i in range(fields)]
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

    if report_id in QUATERNIONS:
        # i, j, k, real - the order the part sends them, which is not the
        # order most quaternion maths is written in. Named so a caller never
        # has to remember which end the scalar is on.
        row['quaternion'] = {'i': row['scaled'][0], 'j': row['scaled'][1],
                             'k': row['scaled'][2], 'real': row['scaled'][3]}
        if report_id == 0x05 and at + 14 <= len(cargo):
            row['accuracy_rad'] = int.from_bytes(
                cargo[at + 12:at + 14], 'little', signed=True) / float(1 << 12)
    return row
