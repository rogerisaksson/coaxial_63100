"""The BNO08X on SPI2: what it says it is, and what it reports."""
from . import protocol
from .errors import LINK_FAULTS, DeviceStateError, RigError
from .protocol import ImuOp
from .sensor import PolledSensor
from .subsystem import Device
from .wire import Reader, pack
from typing import Any

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
ACCURACY_MASK = 0x03

#: The input report ids this host decodes, section 1.3.5.
ACCELEROMETER = 0x01
GYROSCOPE = 0x02
MAGNETIC_FIELD = 0x03
LINEAR_ACCELERATION = 0x04
ROTATION_VECTOR = 0x05
GRAVITY = 0x06
GAME_ROTATION_VECTOR = 0x08
TIMEBASE = 0xFB

REPORTS = {
    ACCELEROMETER: 'accelerometer', GYROSCOPE: 'gyroscope',
    MAGNETIC_FIELD: 'magnetic field', LINEAR_ACCELERATION: 'linear acceleration',
    ROTATION_VECTOR: 'rotation vector', GRAVITY: 'gravity',
    GAME_ROTATION_VECTOR: 'game rotation vector', TIMEBASE: 'timebase',
}

# Q point per report: the fixed-point counts are divided by 2**q to give the
# unit named beside it.
SCALE = {
    ACCELEROMETER: (8, 'm/s^2'),
    GYROSCOPE: (9, 'rad/s'),
    MAGNETIC_FIELD: (4, 'uT'),
    LINEAR_ACCELERATION: (8, 'm/s^2'),
    GRAVITY: (8, 'm/s^2'),
    ROTATION_VECTOR: (14, ''),
    GAME_ROTATION_VECTOR: (14, ''),
}

#: Bytes one input report occupies, the same table shtp.c keeps and for
#: the same reason: reports are packed back to back and are not
#: self-delimiting, so a length nobody checked mis-frames every byte after
#: it. The datasheet does not tabulate the two rotation vectors; CEVA's own
#: decoder does - github.com/ceva-dsp/sh2, sh2_SensorValue.c. The rotation
#: vector carries i, j, k, real and an accuracy estimate behind the common
#: header; the game rotation vector the same without the estimate.
REPORT_LENGTHS = {
    TIMEBASE: 5,
    ACCELEROMETER: 10, GYROSCOPE: 10, MAGNETIC_FIELD: 10,
    LINEAR_ACCELERATION: 10, GRAVITY: 10,
    ROTATION_VECTOR: 14, GAME_ROTATION_VECTOR: 12,
}

#: An input report's common header: id, sequence, status, delay.
HEADER = 4
#: One axis, little-endian.
WORD = 2
#: The rotation vector's accuracy estimate, radians in Q12.
ACCURACY_Q = 12

LOOP_STATES = {0: 'off', 1: 'init', 2: 'running', 3: 'held'}
"""What the board's IMU poll loop is doing. 'off' means AFE_ON is low."""

LOOP_ERRORS = {
    0: 'none', 1: 'lost AFE_ON', 2: 'the part did not come up',
    3: 'a cargo read failed', 4: 'a report id with no length',
    5: 'wrote without an H_INTN acknowledge - the part never asked',
}
"""The last thing that went wrong in the poll loop, not a running tally."""

QUATERNIONS = (ROTATION_VECTOR, GAME_ROTATION_VECTOR)
"""Reports whose four fields are i, j, k, real rather than three axes. Q14,
and unitless: a rotation vector is a direction, not a quantity."""
QUATERNION_AXES = ('i', 'j', 'k', 'real')

#: The three vectors appended to the shared record by MINOR 6, in order.
VECTORS = (('accelerometer', ACCELEROMETER), ('gyroscope', GYROSCOPE),
           ('magnetometer', MAGNETIC_FIELD))

#: SPI2's four pins, as `pins()` names them.
SPI2_PINS = {12: 'NSS/H_CSN', 13: 'SCK', 14: 'MISO', 15: 'MOSI'}
#: What a pin that followed the MCU every way reads back as.
ALL_BITS = 0x0F

#: What `wake_test` answers for the two values that are not a time.
WAKE_ANSWERS = {0xFFFF: None, 0xFFFE: 'busy'}


def _report(r):
    """The newest quaternion report in the shared record, or none."""
    if not r.u8():
        return {'quaternion': None}
    report_id = r.u8()
    status = r.u8()
    counts = [r.i16() for _ in QUATERNION_AXES]
    divisor = float(1 << SCALE[report_id][0])
    return {
        'report_id': report_id,
        'name': REPORTS.get(report_id, 'unknown 0x%02X' % report_id),
        'accuracy': ACCURACY.get(status & ACCURACY_MASK, 'unknown'),
        'counts': dict(zip(QUATERNION_AXES, counts)),
        'quaternion': dict(zip(QUATERNION_AXES,
                               (c / divisor for c in counts))),
    }


def _vector(r, report_id):
    """One three-axis report, or None when it never arrived."""
    if not r.remaining:
        return None
    have = bool(r.u8())
    status = r.u8()
    counts = [r.i16(), r.i16(), r.i16()]
    if not have:
        return None
    q, unit = SCALE[report_id]
    divisor = float(1 << q)
    return {'accuracy': ACCURACY.get(status & ACCURACY_MASK, 'unknown'),
            'unit': unit,
            'counts': dict(zip('xyz', counts)),
            'value': dict(zip('xyz', (c / divisor for c in counts)))}


class Imu(Device, PolledSensor, device=protocol.DEVICE_IMU):
    """The BNO08X behind SPI2."""

    LOOP_STATES = LOOP_STATES

    #: Ops that do not drive the bus, so a running poll loop cannot be their
    #: problem: the shared record, and hold and resume themselves.
    FREE_OPS = frozenset((ImuOp.LATEST, ImuOp.HOLD, ImuOp.RESUME))

    def _op(self, op, payload=b'', **kwargs):
        """One request for this device, and the reason when it is refused."""
        try:
            return super()._op(op, payload, **kwargs)
        except RigError as exc:
            raise self._explain(op, exc) from exc

    def _explain(self, op, exc):
        """Turn a bare device failure into what to do about it."""
        if op in self.FREE_OPS:
            return exc
        try:
            loop = self.state()['loop']
        except LINK_FAULTS:                # the first failure
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
        """What the part says it is - the answer that proves the link."""
        try:
            reply = self._op(ImuOp.ID)
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
        """One SHTP cargo, decoded as far as the datasheet allows."""
        r = Reader(self._op(ImuOp.READ))
        channel = r.u8()
        cargo = bytes(r.take(r.u8()))

        return {
            'channel': channel,
            'channel_name': CHANNELS.get(channel, 'unknown'),
            'cargo': cargo,
            'reports': decode(cargo),
        }

    def state(self):
        """The poll loop's shared record: what it saw and what went wrong."""
        r = Reader(self._op(ImuOp.LATEST))
        got = {
            'loop': LOOP_STATES.get(r.u8(), 'unknown'),
            'error': LOOP_ERRORS.get(r.u8(), 'unknown'),
            'updates': r.u32(),
            'cargoes': r.u32(),
            'errors': r.u32(),
        }
        got.update(_report(r))

        # AFTER the report block, because the board writes it after.
        got['feature'] = {'report_id': r.u8(),
                          'interval_us': r.u32(),
                          'pending': bool(r.u8())}

        # The last error that was not 'none', kept by the board.
        got['last_fault'] = LOOP_ERRORS.get(r.u8(), 'unknown')
        got['last_fault_id'] = r.u8()

        # The three vectors, appended by MINOR 6 and read only if they are
        # there - a board older than that answers a reply that stops above, and
        # a decoder that assumed the bytes would raise on one that is simply
        # older.
        for name, report in VECTORS:
            got[name] = _vector(r, report)
        return got

    def latest(self):
        """The newest quaternion, or None when the loop has not seen one."""
        return self.state()['quaternion']

    def hold(self):
        """Stop the poll loop so the part can be configured."""
        return self._loop_state(self._op(ImuOp.HOLD))

    def resume(self):
        """Start the poll loop again, through init - the usual reason to have
        held it was a reset, and the part needs bringing up after one.
        """
        return self._loop_state(self._op(ImuOp.RESUME))

    def reset(self):
        """Pulse NRSTN and collect what the part says coming up."""
        return Reader(self._op(ImuOp.RESET)).u8()

    def wake_test(self, ms=200):
        """Milliseconds for H_INTN to answer PS0/WAKE on a drained part."""
        got = Reader(self._op(ImuOp.WAKE, pack(('u16', ms)))).u16()
        return WAKE_ANSWERS.get(got, got)

    def pins(self):
        """Drive and release each of SPI2's four pins, and say what read back.
        """
        r = Reader(self._op(ImuOp.PINS))
        return [self._pin(r) for _ in SPI2_PINS]

    @staticmethod
    def _pin(r):
        pin, bits = r.u8(), r.u8()
        return {'pin': 'PB%d' % pin, 'signal': SPI2_PINS.get(pin, '?'),
                'bits': bits, 'held': bits != ALL_BITS}

    def probe(self, length=4, select=True):
        """`length` raw bytes off SPI2, unframed and uninterpreted."""
        r = Reader(self._op(ImuOp.PROBE,
                            pack(('u8', length), ('u8', int(bool(select))))))
        kernel, bitrate = r.u32(), r.u32()
        return {'kernel_hz': kernel, 'bitrate_hz': bitrate,
                'raw': bytes(r.take(r.u8()))}

    def write(self, channel, payload):
        """Put `payload` on `channel` as one SHTP cargo, unparsed."""
        if channel not in CHANNELS:
            raise ValueError('channel %r is not one of the six' % (channel,))
        self._op(ImuOp.WRITE, pack(('u8', channel)) + bytes(payload))

    def feature(self, report_id, interval_us):
        """Enable a sensor report, or disable it with an interval of 0."""
        self._op(ImuOp.FEATURE,
                 pack(('u8', report_id), ('u32', interval_us)))


def report_length(report_id):
    """Bytes one input report occupies, or 0 when it is not known here."""
    return REPORT_LENGTHS.get(report_id, 0)


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


def _axis(cargo, at, index):
    """Axis `index` of the report at `at`: a little-endian signed word."""
    start = at + HEADER + WORD * index
    return int.from_bytes(cargo[start:start + WORD], 'little', signed=True)


def _timebase(cargo, at):
    """The timebase report: a signed delta in hundreds of microseconds."""
    return {'report_id': TIMEBASE, 'name': REPORTS[TIMEBASE],
            'base_delta_100us': int.from_bytes(
                cargo[at + 1:at + 1 + HEADER], 'little', signed=True)}


def _quaternion(scaled, cargo, at, report_id):
    """i, j, k, real - the order the part sends them, which is not the order
    most quaternion maths is written in.
    """
    got: dict[str, Any] = {'quaternion': dict(zip(QUATERNION_AXES, scaled))}
    estimate = at + HEADER + WORD * len(QUATERNION_AXES)
    if report_id == ROTATION_VECTOR and estimate + WORD <= len(cargo):
        got['accuracy_rad'] = int.from_bytes(
            cargo[estimate:estimate + WORD], 'little',
            signed=True) / float(1 << ACCURACY_Q)
    return got


def _one(cargo, at, report_id):
    """One report, with its counts and - where the Q point is known - a
    physical quantity beside them.
    """
    if report_id == TIMEBASE:
        return _timebase(cargo, at)

    status = cargo[at + 2]
    fields = len(QUATERNION_AXES) if report_id in QUATERNIONS else len('xyz')
    axes = [_axis(cargo, at, i) for i in range(fields)]
    row = {
        'report_id': report_id,
        'name': REPORTS.get(report_id, 'unknown 0x%02X' % report_id),
        'seq': cargo[at + 1],
        'accuracy': ACCURACY.get(status & ACCURACY_MASK, 'unknown'),
        'delay_100us': ((status >> 2) << 8) | cargo[at + 3],
        'raw': axes,
    }

    scale = SCALE.get(report_id)
    if scale is not None:
        q, unit = scale
        row['scaled'] = [v / float(1 << q) for v in axes]
        row['unit'] = unit

    if report_id in QUATERNIONS:
        row.update(_quaternion(row['scaled'], cargo, at, report_id))
    return row
