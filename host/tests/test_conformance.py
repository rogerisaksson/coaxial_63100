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
    b.request(pdu_w_single_coil(0x0000, False))
    time.sleep(0.2)
    p = parse(b.request(pdu_read(0x02, 0x0000, 1)))
    off_din = coils_from(p[2], 1)[0] if p and p[3] else None
    p = parse(b.request(pdu_read(0x01, 0x0000, 1)))
    off_coil = coils_from(p[2], 1)[0] if p and p[3] else None

    b.request(pdu_w_single_coil(0x0000, True))
    time.sleep(0.4)
    p = parse(b.request(pdu_read(0x02, 0x0000, 1)))
    on_din = coils_from(p[2], 1)[0] if p and p[3] else None
    p = parse(b.request(pdu_read(0x01, 0x0000, 1)))
    on_coil = coils_from(p[2], 1)[0] if p and p[3] else None

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
    run.expect_exception('FC04 addr 0x0007 is a hole -> exc 02',
                         pdu_read(0x04, 0x0007, 1), 0x04, 0x02)
    run.expect_exception('FC04 span crossing a hole -> exc 02',
                         pdu_read(0x04, 0x0005, 4), 0x04, 0x02)
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


if __name__ == '__main__':
    print(selftest_crc())
    if len(sys.argv) > 1 and sys.argv[1] == '--offline':
        print('harness self-test only; skipping bus tests')
        sys.exit(0)
    bus = Bus()
    run = Runner(bus)
    try:
        enter_modbus(bus)
        protocol_tests(run)
        map_tests(run)
    finally:
        try:
            leave_modbus(bus)
        finally:
            bus.close()
    print(chr(10) + '%d passed, %d failed' % (run.passed, run.failed))
    sys.exit(1 if run.failed else 0)
