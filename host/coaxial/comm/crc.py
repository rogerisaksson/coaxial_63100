"""CRC-16/MODBUS."""

POLYNOMIAL = 0xA001      # reflected 0x8005
INITIAL = 0xFFFF


def crc16(data):
    """CRC-16/MODBUS over a bytes-like object."""
    crc = INITIAL
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ POLYNOMIAL if crc & 1 else crc >> 1
    return crc


CHECK_VALUE = 0x4B37     # crc16(b'123456789'), from the CRC catalogue

if crc16(b'123456789') != CHECK_VALUE:
    raise RuntimeError('CRC-16/MODBUS implementation is broken: %#06x for the '
                       'catalogue check, not %#06x'
                       % (crc16(b'123456789'), CHECK_VALUE))
