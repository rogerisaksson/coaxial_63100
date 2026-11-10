"""The master's side of the bootloader against the stand-in (docs/BOOT.md).

The sequence on one node, the image kept when it is the one held, the
refusals in the node's words, and the interface holding both implementations
to one set of names.
"""
import struct
import sys
import zlib

from coaxial import Coaxial63100
from coaxial.devices.boot import Boot, BootControl, Master, chunks_of, enumerate_blank
from coaxial.errors import DeviceStateError
from coaxial.simulated.boot import SimulatedBoot, SimulatedSegment
from machine.rig.modes import SIMULATED

from test_modbus_core import Report

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
    if node is None:
        return
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
    from coaxial.devices.boot import SEAL_MAGIC, image_of, store_of
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
    with Coaxial63100(execution_mode=SIMULATED) as device:
        report.check('the rig exposes it as board.boot', isinstance(device.board.boot, BootControl))


def test_the_images_their_sources(report, _boot):
    """The terminal's check (tools/target/current.py): ninja's dry run read for what it would
    build, nothing where it has no work."""
    from tools.target.current import pending
    said = ("ninja: Entering directory `build/Release'\n"
            '[1/3] Building C object CMakeFiles/app.dir/comms/src/cmd_board.c.obj\n'
            '[2/3] Linking C executable coaxial_63100.elf\n')
    report.check('a tree behind says its steps and the first, a current one nothing',
                 pending(said) == '3 steps, the first Building C object '
                                  'CMakeFiles/app.dir/comms/src/cmd_board.c.obj'
                 and pending('ninja: no work to do.\n') == '',
                 repr(pending(said)))


def test_a_pull_runs_the_check(report, _boot):
    """Git's hooks (.githooks/post-merge, post-rewrite, post-checkout): a pull, a rebase and another commit checked
    out run the images' check in a tree that has built one - a stand-in tree, its check a file that counts its
    runs -, and nothing at a file checked out or at a commit amended; a tree that has built none runs it at its
    pull and its rebase too - they lay its images -, at a commit checked out not."""
    import os
    import shutil
    import subprocess
    import tempfile
    repo = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    env = {k: v for k, v in os.environ.items() if not k.startswith('GIT_')}
    with tempfile.TemporaryDirectory() as top:
        tree, hooks = os.path.join(top, 'tree'), os.path.join(top, 'hooks')
        target, papers = (os.path.join(tree, 'host', 'tools', name) for name in ('target', 'notebooks'))
        os.makedirs(target)
        os.makedirs(papers)
        os.makedirs(hooks)
        for name in ('post-merge', 'post-rewrite', 'post-checkout'):
            shutil.copy(os.path.join(repo, '.githooks', name), hooks)
        for folder in (os.path.dirname(target), target, papers):
            open(os.path.join(folder, '__init__.py'), 'w').close()
        for folder, name in ((target, 'current.py'), (papers, 'current.py')):
            with open(os.path.join(folder, name), 'w', encoding='utf-8') as f:
                f.write("import os\nopen(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ran'), 'a').write('.')\n")

        def git(*words):
            done = subprocess.run(['git', '-c', 'core.hooksPath=' + hooks, '-c', 'user.name=check',
                                   '-c', 'user.email=check@localhost', '-c', 'commit.gpgsign=false', *words],
                                  cwd=tree, env=env, capture_output=True, text=True, encoding='utf-8', errors='replace')
            assert done.returncode == 0, (words, done.stderr[-300:])
            return done.stdout.strip()

        def ran(folder=target):
            path = os.path.join(folder, 'ran')
            return os.path.getsize(path) if os.path.exists(path) else 0

        def laid(name, text):
            with open(os.path.join(tree, name), 'w', encoding='utf-8') as f:
                f.write(text)
            git('add', name)
            git('commit', '-q', '-m', text)
            return git('rev-parse', 'HEAD')

        git('init', '-q')
        git('add', 'host')
        one, two = laid('f', 'one'), laid('f', 'two')
        git('checkout', '-q', '--detach', one)
        bare = [ran()]
        git('merge', '-q', '--ff-only', two)
        bare.append(ran())
        git('reset', '-q', '--hard', one)
        laid('g', 'three')
        git('commit', '-q', '--amend', '-m', 'again')
        bare.append(ran())
        git('rebase', '-q', two)
        bare.append(ran())
        git('checkout', '-q', '--detach', one)
        bare.append(ran())
        was = bare[-1]
        os.makedirs(os.path.join(tree, 'build', 'Debug'))
        open(os.path.join(tree, 'build', 'Debug', 'build.ninja'), 'w').close()
        git('checkout', '-q', '--detach', two)
        moved = ran()
        git('checkout', '-q', '--detach', two)
        git('checkout', '-q', one, '--', 'f')
        git('checkout', '-q', two, '--', 'f')
        still = ran()
        git('reset', '-q', '--hard', one)
        git('merge', '-q', '--ff-only', two)
        merged = ran()
        git('reset', '-q', '--hard', one)
        laid('h', 'four')
        git('commit', '-q', '--amend', '-m', 'again')
        amended = ran()
        git('rebase', '-q', two)
        rebased, laid_too = ran(), ran(papers)
    report.check('no image built: no check at a commit checked out or amended, one at a fast-forward merge and '
                 'one at a rebase', bare == [0, 1, 1, 2, 2], bare)
    report.check('an image built: one a commit checked out, none at the same or a file',
                 (moved - was, still - was) == (1, 1), (moved - was, still - was))
    report.check('one more at a fast-forward merge, none at an amend, and more at a rebase',
                 (merged - was, amended - was) == (2, 2) and rebased > amended, (merged, amended, rebased))
    report.check("the papers' check (tools/notebooks/current.py) runs wherever the images' does",
                 laid_too == rebased > 0, (laid_too, rebased))


def main():
    report = Report()
    for test in (test_one_node_flashed, test_the_same_image_kept, test_refusals, test_a_bus_of_four,
                 test_the_store, test_the_image_from_an_elf,
                 test_the_two_implementations_share_their_names, test_the_images_their_sources,
                 test_a_pull_runs_the_check):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report, SimulatedBoot())
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
