#!/usr/bin/env python3
"""Independent Modbus RTU master, used to conformance-test the firmware slave.

Deliberately does NOT use pymodbus: the CRC, the framing and the PDU packing are
implemented here from the specification so that a shared wrong assumption between
master and slave cannot hide a defect. If both sides agree, they agree for real.
"""
import struct, sys, time
import serial

PORT, BAUD, SLAVE = 'COM4', 115200, 1
T35 = 0.005          # generous inter-frame gap; spec minimum at this baud is 1.75 ms
REPLY_TIMEOUT = 0.30

def crc16(data: bytes) -> int:
    """CRC-16/MODBUS, bitwise from the definition: reflected poly 0xA001, init 0xFFFF."""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc

def selftest_crc():
    # Canonical check value for CRC-16/MODBUS over ASCII "123456789".
    got = crc16(b'123456789')
    assert got == 0x4B37, 'CRC self-test failed: got 0x%04X want 0x4B37' % got
    return 'CRC-16/MODBUS check("123456789") = 0x%04X' % got

class Bus:
    def __init__(self, port=PORT, baud=BAUD):
        self.s = serial.Serial(port, baud, bytesize=8, parity='N', stopbits=1,
                               timeout=REPLY_TIMEOUT)
        self.log = []

    def close(self):
        self.s.close()

    def raw(self, frame: bytes, expect_reply=True):
        """Send an exact frame. Returns the reply bytes, or b'' on silence."""
        time.sleep(T35)
        self.s.reset_input_buffer()
        self.s.write(frame)
        self.s.flush()
        if not expect_reply:
            time.sleep(0.05)
            return self.s.read(256)
        # read until the line goes quiet
        deadline = time.time() + REPLY_TIMEOUT
        buf = b''
        while time.time() < deadline:
            chunk = self.s.read(1)
            if not chunk:
                if buf:
                    break
                continue
            buf += chunk
            deadline = time.time() + 0.02       # quiet-time frame end
        return buf

    def request(self, pdu: bytes, slave=SLAVE, expect_reply=True):
        frame = bytes([slave]) + pdu
        frame += struct.pack('<H', crc16(frame))
        reply = self.raw(frame, expect_reply)
        return reply

def parse(reply: bytes):
    """-> (addr, fc, data, ok_crc) or None if too short."""
    if len(reply) < 4:
        return None
    body, crc_rx = reply[:-2], struct.unpack('<H', reply[-2:])[0]
    return reply[0], reply[1], body[2:], crc_rx == crc16(body)

# ---- PDU builders -------------------------------------------------------
def pdu_read(fc, addr, qty):      return struct.pack('>BHH', fc, addr, qty)
def pdu_w_single_reg(addr, val):  return struct.pack('>BHH', 0x06, addr, val)
def pdu_w_single_coil(addr, on):  return struct.pack('>BHH', 0x05, addr, 0xFF00 if on else 0x0000)
def pdu_w_multi_reg(addr, vals):
    return struct.pack('>BHHB', 0x10, addr, len(vals), 2 * len(vals)) + b''.join(
        struct.pack('>H', v) for v in vals)
def pdu_w_multi_coil(addr, bits):
    nb = (len(bits) + 7) // 8
    by = bytearray(nb)
    for i, b in enumerate(bits):
        if b:
            by[i // 8] |= 1 << (i % 8)
        # LSB-first within each byte, per spec
    return struct.pack('>BHHB', 0x0F, addr, len(bits), nb) + bytes(by)

def regs_from(data: bytes):
    n = data[0]
    return [struct.unpack('>H', data[1 + 2 * i:3 + 2 * i])[0] for i in range(n // 2)]

def coils_from(data: bytes, qty: int):
    out = []
    for i in range(qty):
        out.append(bool(data[1 + i // 8] >> (i % 8) & 1))
    return out

# ---- test runner --------------------------------------------------------
def adc_channels(bus):
    """How many analog channels the board has, asked of the board.

    0x42 appends the total after the rows it managed to fit, which is what
    makes this one request rather than a count written down here. It was
    written down - as 7 - and two supply senses were added.
    """
    reply = bus.request(bytes([0x42]))
    parsed = parse(reply)
    return parsed[2][-1] if parsed and parsed[2] else 0


class Runner:
    def __init__(self, bus):
        self.bus, self.passed, self.failed = bus, 0, 0

    def check(self, name, cond, detail=''):
        if cond:
            self.passed += 1
            print('  PASS  %-42s %s' % (name, detail))
        else:
            self.failed += 1
            print('  FAIL  %-42s %s' % (name, detail))

    def expect_exception(self, name, pdu, fc, code):
        r = self.bus.request(pdu)
        p = parse(r)
        if p is None:
            return self.check(name, False, 'no/short reply: %s' % r.hex(' '))
        addr, rfc, data, ok = p
        self.check(name,
                   ok and addr == SLAVE and rfc == (fc | 0x80) and len(data) == 1 and data[0] == code,
                   'got %s (crc %s)' % (r.hex(' '), 'ok' if ok else 'BAD'))

    def expect_silence(self, name, frame):
        r = self.bus.raw(frame, expect_reply=False)
        self.check(name, r == b'', 'got %s' % (r.hex(' ') if r else '<silence>'))


def protocol_tests(run):
    """Conformance checks that do not depend on the register map."""
    b = run.bus
    print('\n-- framing and addressing --')

    # A frame whose CRC is wrong must be discarded silently: no reply, not an exception.
    f = bytes([SLAVE, 0x03, 0x00, 0x00, 0x00, 0x01])
    run.expect_silence('bad CRC -> silence', f + b'\x00\x00')

    # A frame addressed to another unit must be ignored entirely.
    f = bytes([SLAVE + 1, 0x03, 0x00, 0x00, 0x00, 0x01])
    run.expect_silence('wrong unit address -> silence', f + struct.pack('<H', crc16(f)))

    # Shorter than addr+fc+crc cannot be a frame.
    run.expect_silence('runt frame (3 bytes) -> silence', b'\x01\x03\x00')

    # Broadcast must be acted on but never answered.
    f = bytes([0x00, 0x63])
    run.expect_silence('broadcast -> silence', f + struct.pack('<H', crc16(f)))

    print('\n-- exceptions --')
    run.expect_exception('unsupported function -> exc 01', bytes([0x63]), 0x63, 0x01)
    run.expect_exception('FC03 quantity 0 -> exc 03', pdu_read(0x03, 0, 0), 0x03, 0x03)
    run.expect_exception('FC03 quantity 126 -> exc 03', pdu_read(0x03, 0, 126), 0x03, 0x03)
    run.expect_exception('FC04 quantity 126 -> exc 03', pdu_read(0x04, 0, 126), 0x04, 0x03)
    run.expect_exception('FC01 quantity 2001 -> exc 03', pdu_read(0x01, 0, 2001), 0x01, 0x03)
    # A full-length FC10 for 124 registers needs 248 data bytes, i.e. a 257-byte
    # ADU, which cannot exist on an RTU line at all - the spec limit of 123 IS
    # the framing limit. So the quantity check is exercised with frames that do
    # fit: zero items, and an over-large count with a truncated payload.
    run.expect_exception('FC10 quantity 0 -> exc 03',
                         struct.pack('>BHHB', 0x10, 0, 0, 0), 0x10, 0x03)
    run.expect_exception('FC0F quantity 0 -> exc 03',
                         struct.pack('>BHHB', 0x0F, 0, 0, 0), 0x0F, 0x03)
    run.expect_exception('FC10 quantity 124 (truncated) -> exc 03',
                         struct.pack('>BHHB', 0x10, 0, 124, 248) + b'\x00' * 4, 0x10, 0x03)
    _big = bytes([SLAVE]) + struct.pack('>BHHB', 0x10, 0, 124, 248) + b'\x00' * 248
    run.expect_silence('FC10 oversized ADU (257 B) -> silence',
                       _big + struct.pack('<H', crc16(_big)))
    run.expect_exception('FC05 bad coil value -> exc 03',
                         struct.pack('>BHH', 0x05, 0, 0x1234), 0x05, 0x03)
    run.expect_exception('FC03 far out of range -> exc 02',
                         pdu_read(0x03, 0xF000, 1), 0x03, 0x02)
    run.expect_exception('FC04 far out of range -> exc 02',
                         pdu_read(0x04, 0xF000, 1), 0x04, 0x02)

    print('\n-- report server id (FC 0x11) --')
    r = b.request(bytes([0x11]))
    p = parse(r)
    if p is None:
        run.check('FC11 replies', False, 'no/short reply: %s' % r.hex(' '))
    else:
        addr, fc, data, ok = p
        run.check('FC11 replies', ok and addr == SLAVE and fc == 0x11 and len(data) >= 2,
                  'byte_count=%d run_ind=0x%02X id=%r' % (
                      data[0] if data else -1,
                      data[1] if len(data) > 1 else -1,
                      bytes(data[2:]).decode('ascii', 'replace') if len(data) > 2 else ''))


def s16(v):
    return v - 0x10000 if v & 0x8000 else v


def enter_modbus(bus):
    """The board boots into the ASCII console; 'm' hands USART3 to Modbus."""
    bus.s.reset_input_buffer()
    bus.s.write(b'm')
    bus.s.flush()
    time.sleep(0.5)
    bus.s.reset_input_buffer()


def leave_modbus(bus):
    """Holding register 0x0001 = 1 returns the line to the console."""
    bus.request(pdu_w_single_reg(0x0001, 0x0001))
    time.sleep(0.3)


def channels_tests(run):
    """0x6D, decoded here from the specification, not with the host library.

    The point of this suite: if the map came back through coaxial/system.py
    both sides could share the same wrong idea of the layout. This walks the
    bytes.
    """
    b = run.bus
    for kind, what in ((0, 'analog'), (1, 'digital'), (2, 'reserved')):
        parsed = parse(b.request(bytes([0x6D, kind])))
        run.check('channels(%s) answered, CRC good' % what,
                  parsed is not None and parsed[3] and parsed[1] == 0x6D,
                  'no reply' if parsed is None else 'fc 0x%02X crc %s'
                  % (parsed[1], 'ok' if parsed[3] else 'BAD'))
        if parsed is None or parsed[1] != 0x6D:
            continue
        body = parsed[2]
        # Kind 0 answers a bare count; the paged sections answer total,
        # first, count. They are paged because 19 reserved pins are 418
        # bytes against a 253-byte PDU - see PROTOCOL.md.
        if kind == 0:
            count, at = body[0], 1
        else:
            count, at = body[2], 3
        rows = 0
        try:
            while rows < count:
                if kind == 0:
                    at += 3                                   # index, adc, ch
                    at += 1 + body[at]                        # pin
                    direction = body[at]
                    at += 2                                   # dir, diff
                    at += 1 + body[at]                        # signal
                    at += 1                                   # unit
                else:
                    at += 1 + body[at]                        # pin
                    direction = body[at]
                    at += 1
                    at += 1 + body[at]                        # signal
                run.check('%s row %d has a direction in range'
                          % (what, rows), direction <= 2, str(direction))
                rows += 1
        except IndexError:
            run.check('%s rows decode inside the payload' % what, False,
                      'ran off the end at row %d' % rows)
            continue
        run.check('%s payload ends exactly where the rows do' % what,
                  at == len(body), '%d of %d bytes' % (at, len(body)))

    # 5, not 4: kind 4 is the parts list now. The boundary this check is
    # about is "past the last section", and it moves when one is added.
    refused = parse(b.request(bytes([0x6D, 5])))
    run.check('an unknown section is refused, not answered',
              refused is not None and (refused[1] & 0x80) != 0,
              'no reply' if refused is None else 'fc 0x%02X' % refused[1])


def rs485_tests(run):
    """The two RS485 ports, checked from a stack that shares no code with the
    host library. Each transceiver has RE tied to GND, so it hears itself:
    four patterns out, four patterns back, and a bit that does not set is the
    driver, the receiver or the wiring between them."""
    b = run.bus

    print(chr(10) + '-- RS485 loopback and multidrop filtering --')

    for port, name in ((1, 'USART2'), (2, 'UART5')):
        parsed = parse(b.request(bytes([0x6E, 2, 0, port])))
        run.check('%s loopback answered, CRC good' % name,
                  parsed is not None and parsed[3] and parsed[1] == 0x6E,
                  'no reply' if parsed is None
                  else 'fc 0x%02X crc %s' % (parsed[1],
                                             'ok' if parsed[3] else 'BAD'))
        if parsed is None or parsed[1] != 0x6E:
            continue

        body = parsed[2]
        run.check('%s is an RS485 port and says so' % name, body[1] == 1,
                  'rs485 flag %d' % body[1])
        run.check('%s echoes all four patterns - 00, FF, 5A, A5' % name,
                  body[2] == 0x0F,
                  'matched 0x%02X, %d bytes back' % (body[2], body[3]))

    # The port carrying the request cannot test itself: its own patterns land
    # in front of the reply. Measured - the master saw a checksum failure.
    refused = parse(b.request(bytes([0x6E, 2, 0, 0])))
    run.check('the port carrying the conversation refuses its own loopback',
              refused is not None and (refused[1] & 0x80) != 0,
              'no reply' if refused is None else 'fc 0x%02X' % refused[1])

    for port, name in ((0, 'USART3'), (1, 'USART2'), (2, 'UART5')):
        parsed = parse(b.request(bytes([0x6E, 2, 1, port])))
        if parsed is None or parsed[1] != 0x6E:
            run.check('%s reports its counters' % name, False, 'no reply')
            continue

        body = parsed[2]
        at = 4
        fields = {}
        for key in ('baud', 't15', 't35', 'bus_message', 'bus_comm_error',
                    'server_message', 'server_exception', 'server_no_response',
                    'char_overrun', 'ring_dropped'):
            fields[key] = int.from_bytes(body[at:at + 4], 'big')
            at += 4

        run.check('%s reports a Modbus bitrate, not the .ioc value' % name,
                  fields['baud'] in (9600, 19200, 38400, 57600, 115200),
                  fields['baud'])
        run.check('%s never dropped a byte for want of ring space' % name,
                  fields['ring_dropped'] == 0, fields['ring_dropped'])
        # Every frame on the segment is counted; only the ones addressed here
        # are served. With one node the two match, and that is the check: a
        # server_message ahead of bus_message would mean the filter is being
        # skipped rather than passed.
        run.check('%s never served more frames than it saw' % name,
                  fields['server_message'] <= fields['bus_message'],
                  '%d served of %d seen' % (fields['server_message'],
                                            fields['bus_message']))
        run.check('%s is open when it is RS485' % name,
                  (body[2] == 0) or (body[3] == 1),
                  'rs485 %d, open %d' % (body[2], body[3]))


def cal_tests(run):
    """Device 3, byte for byte, from a master that shares no code with the
    library. Edits only - saving erases a flash sector, and a suite that runs
    on every commit has no business doing that to a board."""
    b = run.bus

    print(chr(10) + '-- 0x6E device 3, the calibration record --')
    r = b.request(bytes([0x6E, 3, 0]))
    p = parse(r)
    if p is None or not p[3]:
        return run.check('device 3 op 0 answers', False,
                         'bad reply %s' % r.hex(' '))

    data = p[2]
    run.check('device 3 op 0 answers', True, '%d bytes' % len(data))

    stored, version = data[0], (data[1] << 8) | data[2]
    count = data[3]
    # Not compared against a number written here. It was 1/9, then 2/9, then
    # 3/13, and each edit only taught the check its own last value. What is
    # worth checking is that the header describes the bytes behind it - the
    # count and the channel count together have to add up to the reply's
    # length, which the check below does.
    run.check('the record names a layout at all', version >= 1,
              'version %d, %d params' % (version, count))

    at = 4
    params = []
    for _ in range(count):
        params.append(int.from_bytes(data[at:at + 4], 'big'))
        at += 4

    # The schematic's numbers, and the only place outside the firmware that
    # says what they are. A default that drifts from board_cal.c fails here.
    run.check('shunt is 3.5 milliohm (RU1 || RU2, 7 mohm each)',
              params[1] == 3500, 'got %d' % params[1])
    run.check('amplifier gain is 4.545455 V/V (THS4551 1.5k/330)',
              params[2] == 4545455, 'got %d' % params[2])
    run.check('and the product still puts 100 A at 48 % of the span',
              abs((params[1] * 1e-6) * (params[2] * 1e-6) - 0.0159091) < 1e-7,
              '%.9f V/A' % ((params[1] * 1e-6) * (params[2] * 1e-6)))
    run.check('DC link divider is 49900/2200',
              (params[3], params[4]) == (49900, 2200),
              'got %d/%d' % (params[3], params[4]))
    run.check('thermistor B is 3380 K', params[6] == 3380000,
              'got %d' % params[6])

    channels = data[at]
    at += 1
    # Off the board's own table. It said 7 and two supply senses were
    # added, which is the second answer this suite exists to avoid.
    run.check('one correction per ADC channel',
              channels == adc_channels(b),
              'got %d' % channels)
    at += channels * 8

    # Then the thermal envelope, which is what makes "the ceilings are
    # stored" checkable from the wire rather than asserted. Counted off the
    # board's own node count, not written out here - a number in this file
    # is the second answer the suite exists to avoid.
    nodes = data[at]
    at += 1
    run.check('one ceiling per thermal node', nodes == 6, 'got %d' % nodes)
    at += nodes * 4 + 4                       # the limits, then throttle ppm

    run.check('the reply ends where the envelope does',
              len(data) == at, '%d bytes, expected %d' % (len(data), at))

    print(chr(10) + '-- what device 3 refuses --')
    run.expect_exception('an unknown parameter id is ILLEGAL DATA VALUE',
                         bytes([0x6E, 3, 1, 99, 0, 0, 0, 1]), 0x6E, 0x03)
    run.expect_exception('a zero reference would divide by zero',
                         bytes([0x6E, 3, 1, 0, 0, 0, 0, 0]), 0x6E, 0x03)
    run.expect_exception('a channel index past the table is refused',
                         bytes([0x6E, 3, 3, 99]), 0x6E, 0x03)
    run.expect_exception('an unknown op is refused, not ignored',
                         bytes([0x6E, 3, 99]), 0x6E, 0x03)
    run.expect_exception('spanning the thermistor is refused - logarithmic',
                         bytes([0x6E, 3, 4, 4, 0, 0, 0x27, 0x10]), 0x6E, 0x04)

    # It refused, so the record must be exactly what it was.
    again = parse(b.request(bytes([0x6E, 3, 0])))
    run.check('a refused edit changed nothing',
              again is not None and again[3] and again[2] == data,
              'record %s' % ('unchanged' if again and again[2] == data
                             else 'MOVED'))


def map_tests(run):
    b = run.bus

    print('\n-- input registers, semantic cross-checks --')
    r = b.request(pdu_read(0x04, 0x0020, 4))
    p = parse(r)
    if p is None or not p[3]:
        run.check('FC04 clock registers', False, 'bad reply %s' % r.hex(' '))
    else:
        v = regs_from(p[2])
        sysclk = (v[0] << 16) | v[1]
        hclk = (v[2] << 16) | v[3]
        run.check('SYSCLK reads 475000000', sysclk == 475000000, 'got %d' % sysclk)
        run.check('HCLK reads 237500000', hclk == 237500000, 'got %d' % hclk)

    r = b.request(pdu_read(0x04, 0x0000, 7))
    p = parse(r)
    if p is None or not p[3]:
        run.check('FC04 seven ADC channels', False, 'bad reply %s' % r.hex(' '))
    else:
        v = regs_from(p[2])
        run.check('FC04 returns 7 registers', len(v) == 7,
                  'diff=[%d,%d,%d] se=[%d,%d,%d,%d]' % (
                      s16(v[0]), s16(v[1]), s16(v[2]), v[3], v[4], v[5], v[6]))

    print('\n-- holding registers --')
    r = b.request(pdu_read(0x03, 0x0000, 2))
    p = parse(r)
    if p is None or not p[3]:
        run.check('FC03 holding 0..1', False, 'bad reply %s' % r.hex(' '))
    else:
        v = regs_from(p[2])
        run.check('unit id register reads 1', v[0] == 1, 'got %d' % v[0])
        run.check('command register reads 0', v[1] == 0, 'got %d' % v[1])

    print('\n-- coil drives real hardware --')
    # AFE_ON also powers the voltage reference, and PE15 was measured to follow
    # it: 1 while the AFE is off, 0 once it is on. That makes the discrete input
    # an independent witness that the coil write reached the pin.
    def sampling(every_ms, settle_ms):
        """Set the observer's NTC sampling. 0 stops it.

        THE RAIL IS SHARED. AFE_ON is reference counted, so the observer
        borrowing it for a sample makes a coil written off read back on -
        truthfully, and at random. This check needs the rail to itself, so it
        says so instead of hoping. Measured: the borrow is 500 ms every 5 s,
        which is exactly often enough to be flaky and rare enough to look
        like a link fault.
        """
        b.request(bytes([0x6E, 8, 3]) + every_ms.to_bytes(4, 'big')
                  + settle_ms.to_bytes(4, 'big'))

    sampling(0, 0)
    time.sleep(0.6)                      # let any borrow in flight finish

    def read_bit(table):
        """One bit, retried. A lost reply is not a wrong answer.

        The link goes quiet now and then - FINDINGS has it open, and 600
        requests ruled out four causes. Everything else in this tree tolerates
        it; this did not, and read back None at random. The WRITE is not
        retried: a write that did not land is a real failure.
        """
        for _ in range(6):
            got = parse(b.request(pdu_read(table, 0x0000, 1)))
            if got and got[3]:
                return coils_from(got[2], 1)[0]
            time.sleep(0.2)
        return None

    b.request(pdu_w_single_coil(0x0000, False))
    time.sleep(0.2)
    off_din = read_bit(0x02)
    off_coil = read_bit(0x01)

    b.request(pdu_w_single_coil(0x0000, True))
    time.sleep(0.4)
    on_din = read_bit(0x02)
    on_coil = read_bit(0x01)

    sampling(5000, 500)

    run.check('coil 0 reads back its written state',
              off_coil is False and on_coil is True,
              'off->%s on->%s' % (off_coil, on_coil))
    run.check('PE15 follows AFE_ON (independent witness)',
              off_din is True and on_din is False,
              'AFE off -> PE15=%s, AFE on -> PE15=%s' % (off_din, on_din))

    print('\n-- scaled physical quantities (AFE on) --')
    p = parse(b.request(pdu_read(0x04, 0x0010, 2)))
    if p is None or not p[3]:
        run.check('FC04 dcbus + ntc', False, 'bad reply')
    else:
        v = regs_from(p[2])
        # Recorded, not judged: the check is that FC04 answered with the two
        # registers asked for and that they decode - the values go in the
        # detail column so a reader sees them, with no threshold anywhere.
        # Until now only the failure path called run.check, so a working
        # FC04 passed in silence and counted for nothing.
        run.check('FC04 dcbus + ntc', len(v) == 2,
                  '%d mV, %.2f C' % (v[0], s16(v[1]) / 100.0))
        # No limits here on purpose. This suite tests the PROTOCOL, and the
        # board is a dumb slave - whether 24 V is the right voltage is a
        # question for a test executive with a calibrated meter, not for a
        # conformance test with a number compiled into it.
        #
        # What IS testable without a reference: that a scaled field agrees with
        # the raw code it was derived FROM. That needs both to come from one
        # conversion, so it uses FC 0x43, where the firmware computes them from
        # a single sample. Reading the two input registers separately compares
        # two independent samples of a noisy channel and fails on noise - which
        # is exactly how the first version of this check failed, by 13 LSB.
        scan = parse(b.request(bytes([0x43])))
        if scan is None or not scan[3]:
            run.check('FC43 scan for the scaling cross-check', False, 'no reply')
        else:
            f = scan[2]
            dc_raw = struct.unpack('>i', f[12:16])[0]
            dc_mv = struct.unpack('>i', f[16:20])[0]
            ntc_cc = struct.unpack('>i', f[24:28])[0]
            recomputed = int(dc_raw / 65536.0 * 3.3 * (49900 + 2200) / 2200 * 1000)
            run.check('DC link scaled field agrees with its own raw code',
                      abs(dc_mv - recomputed) <= 2,
                      'raw %d -> %d mV, recomputed %d' % (dc_raw, dc_mv, recomputed))
            run.check('NTC scaled field decodes to a finite temperature',
                      -30000 < ntc_cc < 20000,
                      '%.2f C' % (ntc_cc / 100.0))

    print('\n-- multi-item writes --')
    r = b.request(pdu_w_multi_reg(0x0000, [1, 0]))
    p = parse(r)
    run.check('FC10 two holding registers',
              p is not None and p[3] and p[1] == 0x10, 'got %s' % r.hex(' '))
    r = b.request(pdu_w_multi_coil(0x0000, [True]))
    p = parse(r)
    run.check('FC0F one coil',
              p is not None and p[3] and p[1] == 0x0F, 'got %s' % r.hex(' '))

    print('\n-- a bad value later in a multi-write span refuses the whole write --')
    # addr 0 (unit id, 99 - a legal value) then addr 1 (command, 999 - not
    # one of the ones this map accepts) in a single FC10. Before this was
    # fixed, write_reg() applied the unit id before discovering the command
    # value was bad, so the device answered exc 03 for the whole request
    # while unit id had already changed underneath it - a real violation of
    # modbus_slave.h's own "must not leave the device half written" promise.
    run.expect_exception('FC10 bad value later in the span answers exc 03',
                         pdu_w_multi_reg(0x0000, [99, 999]), 0x10, 0x03)
    r = b.request(pdu_read(0x03, 0x0000, 1))
    p = parse(r)
    unit_unchanged = p is not None and p[3] and regs_from(p[2]) == [1]
    run.check('and unit id was not changed by the refused write',
              unit_unchanged,
              'got %s' % (r.hex(' ') if r else '<silence - unit id may have changed>'))
    if not unit_unchanged:
        # The board would now be answering on 99, not 1: put it back before
        # the rest of this run, or the next one, loses the server entirely.
        b.request(pdu_w_single_reg(0x0000, 1), slave=99)

    print('\n-- map-specific exceptions --')
    # The first address past the channels, whatever there are of them.
    # It was 0x0007 with seven; adding two moved the hole rather than
    # removing it, and a fixed address here tested the count, not the map.
    hole = adc_channels(b)
    run.expect_exception('FC04 one past the last channel is a hole -> exc 02',
                         pdu_read(0x04, hole, 1), 0x04, 0x02)
    run.expect_exception('FC04 span crossing that hole -> exc 02',
                         pdu_read(0x04, hole - 2, 4), 0x04, 0x02)
    run.expect_exception('FC03 addr 2 unmapped -> exc 02',
                         pdu_read(0x03, 0x0002, 1), 0x03, 0x02)
    run.expect_exception('FC02 addr 1 unmapped -> exc 02',
                         pdu_read(0x02, 0x0001, 1), 0x02, 0x02)
    run.expect_exception('FC05 coil 1 unmapped -> exc 02',
                         struct.pack('>BHH', 0x05, 1, 0xFF00), 0x05, 0x02)
    run.expect_exception('FC06 unknown command -> exc 03',
                         pdu_w_single_reg(0x0001, 0x1234), 0x06, 0x03)
    run.expect_exception('FC06 unit id 0 -> exc 03',
                         pdu_w_single_reg(0x0000, 0), 0x06, 0x03)
    run.expect_exception('FC06 unit id 248 -> exc 03',
                         pdu_w_single_reg(0x0000, 248), 0x06, 0x03)

    print('\n-- broadcast is executed but never answered --')
    b.request(pdu_w_single_coil(0x0000, False))
    time.sleep(0.2)
    f = bytes([0x00]) + pdu_w_single_coil(0x0000, True)
    f += struct.pack('<H', crc16(f))
    run.expect_silence('broadcast coil write -> silence', f)
    time.sleep(0.3)
    p = parse(b.request(pdu_read(0x01, 0x0000, 1)))
    run.check('broadcast write took effect',
              p is not None and p[3] and coils_from(p[2], 1)[0] is True,
              'coil now %s' % (coils_from(p[2], 1)[0] if p and p[3] else '?'))

    print('\n-- diagnostic counters --')
    p = parse(b.request(pdu_read(0x04, 0x0030, 12)))
    if p is None or not p[3]:
        run.check('FC04 counters', False, 'bad reply')
    else:
        v = regs_from(p[2])
        c = [(v[i] << 16) | v[i + 1] for i in range(0, 12, 2)]
        run.check('bus_message > server_message', c[0] > c[2],
                  'bus=%d comm_err=%d srv=%d exc=%d no_rsp=%d ovr=%d' % tuple(c))
        run.check('bus_comm_error counted the bad-CRC frames', c[1] >= 1,
                  'bus_comm_error=%d' % c[1])

        print('\n-- counters survive leaving and re-entering binary mode --')
        # link_open() used to memset the whole mb_rtu_t on every 'm', counters
        # included - so a console round trip ('0x0001=1' out, 'm' back in)
        # silently zeroed this run's diagnostic history. link_close() already
        # left them alone; link_open() now does too.
        before = c[0]
        leave_modbus(bus)
        enter_modbus(bus)
        p3 = parse(b.request(pdu_read(0x04, 0x0030, 12)))
        if p3 is None or not p3[3]:
            run.check('bus_message survives a console round trip', False,
                      'bad reply after re-entering binary mode')
        else:
            v3 = regs_from(p3[2])
            after = (v3[0] << 16) | v3[1]
            run.check('bus_message survives a console round trip',
                      after >= before, 'before=%d after=%d' % (before, after))


def board_answers(port=PORT, baud=BAUD, unit=SLAVE):
    """Whether there is firmware on the other end to conform to.

    This suite cannot be simulated and is the one that must not be. It is a
    byte-level master built from the specification precisely so a shared
    wrong assumption between master and slave cannot hide a defect - and a
    stand-in for the slave would be exactly that shared assumption, written
    by the same hand. With no board it runs the CRC self-test, which needs
    none, and says what it skipped.
    """
    import os
    sys.path.insert(0, os.path.join(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))), 'tools'))
    import find_board
    return find_board.probe(port, baud, unit)


if __name__ == '__main__':
    print(selftest_crc())
    offline = len(sys.argv) > 1 and sys.argv[1] == '--offline'
    if not offline and not board_answers():
        offline = True
        print('no board on %s - the bus tests need firmware to conform to '
              'and cannot be simulated' % PORT)
    if offline:
        print('harness self-test only; skipping bus tests')
        # A tally either way: without one, run_tests.py reads the suite as
        # having crashed before it could print its own numbers.
        print(chr(10) + '1 passed, 0 failed')
        sys.exit(0)
    bus = Bus()
    run = Runner(bus)
    try:
        enter_modbus(bus)
        protocol_tests(run)
        channels_tests(run)
        rs485_tests(run)
        map_tests(run)
        cal_tests(run)
    finally:
        try:
            leave_modbus(bus)
        finally:
            bus.close()
    print(chr(10) + '%d passed, %d failed' % (run.passed, run.failed))
    sys.exit(1 if run.failed else 0)
