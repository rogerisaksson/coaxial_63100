"""Modbus RTU as her buses speak it: CRC-16, the frames a pass sends and a board answers.

    crc16(b'123456789') == CHECK_VALUE
    broadcast(1, [mdeg, ..]); poll(unit); reply(unit, mdeg, mdeg_s)    # the bytes
    requests(data), replies(data) -> [(unit, fc, body)], bytes not a frame

Holding registers: SETPOINT_REG unit 1's setpoint (i32 mdeg), two registers a unit up the bus -
one broadcast 0x10 sets every board's; STATE_REG a board's angle and rate (i32 mdeg, mdeg/s),
read by 0x03. A register's words come high first, a frame's CRC low byte first.
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

READ, WRITE, BROADCAST = 0x03, 0x10, 0
SETPOINT_REG, STATE_REG = 0x0100, 0x0200

#: A poll's bytes and its reply's; a broadcast's around its setpoints, and a setpoint's.
POLL_B, REPLY_B, FRAME_B, SETPOINT_B = 8, 13, 9, 4


def framed(pdu):
    return pdu + struct.pack('<H', crc16(pdu))


def broadcast(first, values):
    """Setpoints for units `first` up, i32 mdeg each: 0x10 to every board."""
    data = struct.pack('>%di' % len(values), *values)
    return framed(struct.pack('>BBHHB', BROADCAST, WRITE, SETPOINT_REG + 2 * (first - 1),
                              2 * len(values), len(data)) + data)


def poll(unit):
    return framed(struct.pack('>BBHH', unit, READ, STATE_REG, 4))


def reply(unit, angle, rate):
    return framed(struct.pack('>BBBii', unit, READ, 8, angle, rate))


def setpoints(body):
    """{unit: mdeg} a broadcast's body carries."""
    address, count, _size = struct.unpack_from('>HHB', body)
    first = (address - SETPOINT_REG) // 2 + 1
    return dict(zip(range(first, first + count // 2),
                    struct.unpack_from('>%di' % (count // 2), body, 5)))


def state(body):
    """(angle, rate) a reply's body carries, mdeg and mdeg/s."""
    return struct.unpack_from('>ii', body, 1)


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
    if data[at + 1] == READ:
        return POLL_B
    if data[at + 1] == WRITE and at + 7 <= len(data):
        return FRAME_B + data[at + 6]
    return 0


def _reply_length(data, at):
    return 5 + data[at + 2] if data[at + 1] == READ else 0


def requests(data):
    """The host's frames in `data`: broadcasts and polls."""
    return _frames(data, _request_length)


def replies(data):
    """The boards' frames in `data`: replies to polls."""
    return _frames(data, _reply_length)
