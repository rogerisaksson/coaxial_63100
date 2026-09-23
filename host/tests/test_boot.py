"""The master's side of the bootloader against the stand-in (docs/BOOT.md):
the sequence on one node, the image kept when it is the one held, the
refusals in the node's words, and the interface holding both
implementations to one set of names.
"""
import os
import struct
import sys
import zlib

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial import Coaxial63100                            # noqa: E402
from coaxial.boot import Boot, BootControl, Master, chunks_of, enumerate_blank  # noqa: E402
from coaxial.errors import DeviceStateError                 # noqa: E402
from coaxial.simulated.boot import SimulatedBoot, SimulatedSegment  # noqa: E402
from test_modbus_core import Report                         # noqa: E402

TYPE = 1


def image(size, version=7):
    body = bytearray((i * 7 + 3) & 0xFF for i in range(size))
    struct.pack_into('<II', body, 0, 0x20010000, 0x08020000 + 0x1001)
    struct.pack_into('<IIII', body, 0x400, 0x50415843, size, version, TYPE)
    return bytes(body)


def test_one_node_flashed(report, boot):
    img, record = image(5 * 224 + 40), bytes(range(200))
    boot.hold(7)
    node = boot.who()
    report.check('who with no prefix answers with the blank node',
                 node is not None and node['state'] == 'held' and node['unit'] == 247)
    report.check('who with the wrong prefix is silence', boot.who(8, b'\xAA') is None)
    boot.assign(node['uid'], 2, 2, terminate=True)
    done = boot.flash(TYPE, img, record)
    report.check('the sequence seals a valid image at its unit and position',
                 done['state'] == 'sealed' and done['valid'] and done['unit'] == 2
                 and done['position'] == 2 and done['held'] == 6)
    report.check('dump reads the record back', boot.dump(0)[1][:200] == record)
    boot.go(7)
    report.check('go for the session jumps', boot.jumped)


def test_the_same_image_kept(report, boot):
    img, record = image(5 * 224 + 40), bytes(range(200))
    boot.hold(7)
    boot.assign(boot.who()['uid'], 2, 2)
    boot.flash(TYPE, img, record)
    boot.reboot()
    boot.hold(7)
    boot.assign(boot.who()['uid'], 2, 2)
    boot.erase(TYPE, img)
    report.check('an erase offering the image held keeps it: verified, nothing missing',
                 boot.state()['state'] == 'verified' and boot.missing() == [])
    report.check('and the sequence still seals', boot.flash(TYPE, img, record)['state'] == 'sealed')
    other = image(5 * 224 + 40, version=8)
    boot.reboot()
    boot.hold(7)
    boot.assign(boot.who()['uid'], 2, 2)
    boot.erase(TYPE, other)
    report.check('a different image of the same size is erased and wanted whole',
                 boot.state()['state'] == 'erased' and boot.missing() == list(range(6)))


def test_refusals(report, boot):
    img = image(6 * 224)
    boot.hold(7)
    uid = boot.who()['uid']
    for unit in (0, 247):
        try:
            boot.assign(uid, unit, 1)
            report.check('assign to unit %d is refused in words' % unit, False)
        except DeviceStateError as exc:
            report.check('assign to unit %d is refused in words' % unit,
                         'broadcast' in str(exc), str(exc))
    boot.assign(uid, 3, 3)
    boot.erase(TYPE, img)
    for index, piece in enumerate(chunks_of(img)):
        if index != 1:
            boot.chunk(index, piece)
    try:
        boot.seal()
        report.check('seal before verify is refused in words', False)
    except DeviceStateError as exc:
        report.check('seal before verify is refused in words', 'verify' in str(exc), str(exc))
    boot.chunk(1, b'\x00' * 224)
    ok, crc = boot.verify()
    report.check('a wrong chunk: verify says no and reports what it summed',
                 not ok and crc != zlib.crc32(img) and boot.state()['state'] == 'erased')
    try:
        boot.stay()
        report.check('stay from the bootloader is refused in words', False)
    except DeviceStateError as exc:
        report.check('stay from the bootloader is refused in words', 'already' in str(exc))


def test_a_bus_of_four(report, _boot):
    uids = [bytes([0x10 + k] + list(range(11))) for k in range(4)]
    uids[2] = bytes([0x10, 0x80] + [0] * 10)          # shares eight bits with the first
    segment = SimulatedSegment(SimulatedBoot(uid=uid) for uid in uids)
    found = enumerate_blank(segment.blank())
    report.check('the prefix search finds all four, splitting where two answer at once',
                 [n['uid'] for n in found] == sorted(u.hex() for u in uids))
    table = {u.hex(): {'unit': i + 1, 'position': i + 1, 'type': TYPE, 'terminate': i == 3}
             for i, u in enumerate(uids[:3])}
    master = Master(segment, table, {TYPE: image(6 * 224)}, {2: bytes(range(50))})
    states = master.run()
    report.check('three placed nodes are sealed, valid and jump; the fourth is named as unknown',
                 all(s['state'] == 'sealed' and s['valid'] for s in states.values())
                 and sorted(states) == [1, 2, 3] and master.unknown == [uids[3].hex()]
                 and [n.jumped for n in segment.nodes] == [True, True, True, False])
    report.check('the record went to unit 2 alone',
                 segment.at(2).dump(0)[1][:50] == bytes(range(50))
                 and segment.at(1).dump(0)[1][:4] == bytes([0xFF] * 4))


def test_the_two_implementations_share_their_names(report, _boot):
    names = {n for n in dir(BootControl) if not n.startswith('_')}
    report.check('Boot and SimulatedBoot both implement BootControl, every name',
                 issubclass(Boot, BootControl) and issubclass(SimulatedBoot, BootControl)
                 and names <= set(dir(Boot)) and names <= set(dir(SimulatedBoot)),
                 ', '.join(sorted(names)))
    with Coaxial63100(simulated_device=True) as device:
        report.check('the rig exposes it as board.boot', isinstance(device.board.boot, BootControl))


def main():
    report = Report()
    for test in (test_one_node_flashed, test_the_same_image_kept, test_refusals, test_a_bus_of_four,
                 test_the_two_implementations_share_their_names):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report, SimulatedBoot())
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
