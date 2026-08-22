"""CRC-16/MODBUS.

Bit-serial from the definition rather than a lookup table: this is a test host,
not a hot path, and a table is one more thing that can be transcribed wrong.
The catalogue check value is asserted at import so a broken edit fails loudly
instead of quietly corrupting every frame.

On the wire the CRC goes out LOW BYTE FIRST, the opposite of every other 16-bit
field in a Modbus frame. That asymmetry lives in transport.py, in one place.
"""

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

assert crc16(b'123456789') == CHECK_VALUE, 'CRC-16/MODBUS implementation is broken'
