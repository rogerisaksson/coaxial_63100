"""Modbus RTU as her buses speak it: CRC-16, the frames a pass sends and a board answers.

    crc16(b'123456789') == CHECK_VALUE
    broadcast(1, [mdeg, ..]); poll(unit); gate(unit, op)     # the host's bytes
    reply(unit, mdeg, mdeg_s, centi_c, spent, derate, status); echo(frame)   # a board's
    requests(data), replies(data) -> [(unit, fc, body)], bytes not a frame

Holding registers: SETPOINT_REG unit 1's setpoint (i32 mdeg), two registers a unit up the bus -
one broadcast 0x10 sets every board's; STATE_REG a board's state, read by 0x03: angle and rate
(i32 mdeg, mdeg/s), its heat (`machine.heat`: the worst node, i16 0.01 C; the envelope spent and
the derate, u16 1/10000; the status word); GATE_REG by 0x06, echoed: GATE_ON its gates on again,
GATE_SHORT its phases shorted through the low sides. A register's words come high first, a
frame's CRC low byte first.
"""
import struct

#: CRC-16/MODBUS: 0x8005 reflected, initial 0xFFFF; a table a byte.
POLYNOMIAL, INITIAL = 0xA001, 0xFFFF
TABLE = []
for _byte in range(256):
    _crc = _byte
    for _ in range(8):
        _crc = (_crc >> 1) ^ POLYNOMIAL if _crc & 1 else _crc >> 1
    TABLE.append(_crc)


def crc16(data):
    """CRC-16/MODBUS over a bytes-like object."""
    crc = INITIAL
    for byte in data:
        crc = (crc >> 8) ^ TABLE[(crc ^ byte) & 0xFF]
    return crc


CHECK_VALUE = 0x4B37     # crc16(b'123456789'), from the CRC catalogue

if crc16(b'123456789') != CHECK_VALUE:
    raise RuntimeError('CRC-16/MODBUS implementation is broken: %#06x for the catalogue check, '
                       'not %#06x' % (crc16(b'123456789'), CHECK_VALUE))

READ, WRITE, WRITE_ONE, BROADCAST = 0x03, 0x10, 0x06, 0
SETPOINT_REG, STATE_REG, GATE_REG = 0x0100, 0x0200, 0x0300
GATE_ON, GATE_SHORT = 1, 2

#: A state's registers and bytes: angle, rate, heat.
STATE_FORMAT = '>iihHHH'
STATE_BYTES = struct.calcsize(STATE_FORMAT)

#: A poll's bytes and its reply's; a broadcast's around its setpoints, and a setpoint's; a gate
#: write's, and its echo's.
POLL_B, REPLY_B, FRAME_B, SETPOINT_B, GATE_B = 8, 5 + STATE_BYTES, 9, 4, 8


def framed(pdu):
    return pdu + struct.pack('<H', crc16(pdu))


def broadcast(first, values):
    """Setpoints for units `first` up, i32 mdeg each: 0x10 to every board."""
    data = struct.pack('>%di' % len(values), *values)
    return framed(struct.pack('>BBHHB', BROADCAST, WRITE, SETPOINT_REG + 2 * (first - 1),
                              2 * len(values), len(data)) + data)


def poll(unit):
    return framed(struct.pack('>BBHH', unit, READ, STATE_REG, STATE_BYTES // 2))


def gate(unit, op=GATE_ON):
    """`unit`'s gates: 0x06 GATE_REG `op`."""
    return framed(struct.pack('>BBHH', unit, WRITE_ONE, GATE_REG, op))


def gate_op(frame):
    """The op a gate write's frame carries."""
    return struct.unpack_from('>H', frame, 4)[0]


def reply(unit, angle, rate, centi_c, spent, derate, status):
    """A board's state: mdeg, mdeg/s, its worst node 0.01 C, its envelope spent and derate
    1/10000, its status word - each clamped to its register."""
    return framed(struct.pack('>BBB', unit, READ, STATE_BYTES) + struct.pack(
        STATE_FORMAT, angle, rate, max(-32768, min(32767, centi_c)),
        max(0, min(65535, spent)), max(0, min(65535, derate)), status & 0xFFFF))


def echo(frame):
    """A write's answer: its own bytes."""
    return bytes(frame)


def setpoints(body):
    """{unit: mdeg} a broadcast's body carries."""
    address, count, _size = struct.unpack_from('>HHB', body)
    first = (address - SETPOINT_REG) // 2 + 1
    return dict(zip(range(first, first + count // 2),
                    struct.unpack_from('>%di' % (count // 2), body, 5)))


def state(body):
    """(angle, rate, centi_c, spent, derate, status) a reply's body carries."""
    return struct.unpack_from(STATE_FORMAT, body, 1)


def _frames(data, length):
    """Whole frames of `data`, each `length(data, at)` bytes (0: not one): [(unit, fc, body)]
    of those whose CRC holds, and the bytes that were no frame."""
    out, bad, at = [], 0, 0
    while at + 4 <= len(data):
        n = length(data, at)
        if not n or at + n > len(data):
            bad += len(data) - at
            break
        frame = data[at:at + n]
        if crc16(frame[:-2]) == frame[-2] | frame[-1] << 8:
            out.append((frame[0], frame[1], frame[2:-2]))
        else:
            bad += n
        at += n
    return out, bad


def _request_length(data, at):
    if data[at + 1] in (READ, WRITE_ONE):
        return POLL_B
    if data[at + 1] == WRITE and at + 7 <= len(data):
        return FRAME_B + data[at + 6]
    return 0


def _reply_length(data, at):
    if data[at + 1] == WRITE_ONE:
        return GATE_B
    return 5 + data[at + 2] if data[at + 1] == READ else 0


def requests(data):
    """The host's frames in `data`: broadcasts and polls."""
    return _frames(data, _request_length)


def replies(data):
    """The boards' frames in `data`: replies to polls."""
    return _frames(data, _reply_length)
