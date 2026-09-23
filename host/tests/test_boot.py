"""The master's side of the bootloader against the stand-in (docs/BOOT.md).

The sequence on one node, the image kept when it is the one held, the
refusals in the node's words, and the interface holding both implementations
to one set of names.
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
    struct.pack_into('<II', body, 0, 0x20010000, 0x30000000 + 0x1001)   # runs from D2 SRAM
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


def test_the_store(report, _boot):
    """Master(persist=True): the store written once a new image, never for
    the same one, and a node whose RAM is gone takes the stored copy with
    nothing streamed."""
    img = image(9 * 224 + 3)
    node = SimulatedBoot()
    segment = SimulatedSegment([node])
    table = {node.uid.hex(): {'unit': 4, 'position': 1, 'type': TYPE}}
    states = Master(segment, table, {TYPE: img}, {}, persist=True).run()
    report.check('sealed with persist: the store written once, the image named in state',
                 node.stores == 1 and states[4]['image'] == (len(img), zlib.crc32(img)))
    node.reboot()
    node.unit = 247
    Master(segment, table, {TYPE: img}, {}, persist=True).run()
    report.check('the same image again writes no store', node.stores == 1)
    node.ram[:] = bytes(len(node.ram))                  # a power cycle
    node.reboot()
    node.unit = 247
    node.hold(1)
    node.assign(node.uid.hex(), 4, 1)
    node.erase(TYPE, img)
    report.check('RAM lost: erase copies the stored image, verified, nothing missing',
                 node.state()['state'] == 'verified' and node.missing() == []
                 and bytes(node.ram[:len(img)]) == img)


def elf_of(segments):
    """A minimal ELF32 LE: one PT_LOAD per (paddr, bytes), vaddr elsewhere."""
    head, phsize = 0x34, 0x20
    table, blobs, at = b'', b'', head + phsize * len(segments)
    for paddr, blob in segments:
        table += struct.pack('<8I', 1, at + len(blobs), 0x20000000, paddr, len(blob), len(blob), 5, 4)
        blobs += blob
    ident = b'\x7fELF\x01\x01\x01' + bytes(9)
    header = ident + struct.pack('<HHIIIIIHHHHHH', 2, 40, 1, 0, head, 0, 0, head, phsize,
                                 len(segments), 40, 0, 0)
    return header + table + blobs


def test_the_image_from_an_elf(report, _boot):
    """image_of cuts what the bootloader takes from the ELF's load segments;
    store_of puts the seal word in front of it."""
    import tempfile
    from pathlib import Path
    from coaxial.boot import SEAL_MAGIC, image_of, store_of
    img = image(7 * 224 + 5)
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / 'app.elf'
        path.write_bytes(elf_of([(0x30000000, img[:600]), (0x30000000 + 600, img[600:]),
                                 (0x24000000, b'\x00' * 64)]))
        cut = image_of(path)
    report.check('two segments in RAM joined at their load addresses, one elsewhere left out',
                 cut == img, '%d bytes' % len(cut))
    store = store_of(img)
    report.check('the store: seal word (magic, size, crc, type) padded to 32, then the image',
                 struct.unpack_from('<IIII', store) == (SEAL_MAGIC, len(img), zlib.crc32(img), TYPE)
                 and store[16:32] == b'\xff' * 16 and store[32:] == img)
    built = Path(__file__).resolve().parents[2] / 'build' / 'Debug' / 'coaxial_63100.elf'
    if built.exists():
        real = image_of(built)
        sp, reset = struct.unpack_from('<II', real)
        report.check('the build\'s own ELF: stack in DTCM, reset into RAM, the header\'s size',
                     0x20000000 < sp <= 0x20020000 and 0x30000000 < reset < 0x30048000
                     and struct.unpack_from('<I', real, 0x404)[0] == len(real))


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
                 test_the_store, test_the_image_from_an_elf,
                 test_the_two_implementations_share_their_names):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report, SimulatedBoot())
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
