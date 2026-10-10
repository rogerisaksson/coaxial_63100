"""The host mirrors the firmware: subsystem calls resolve, limits live in one file, the mirrors agree."""
import ast
import glob
import importlib
import io
import operator
import os
import re
import sys

from tools.dev.focus import chosen
from structure_kit import OP_CLASSES, REPO, Report, sources


#: `board.<name>.<method>()` is how every view and tool reaches the hardware.
#: The names come from Board itself, so adding a subsystem needs nothing here.
def _subsystems():
    from coaxial.devices.board import Board

    return Board.parts()                 # the declaration, no transport


#: The firmware's fixed numbers, and the one file that owns them. A #define
#: matching this outside it is the second answer board_limits.h exists to
#: prevent - they were one per file, in two layers, and IMU_CARGO being
#: smaller than IMU_BUF was invisible until a cargo arrived truncated.
OWNED = ('_MAX_HZ', '_POLL_HZ', '_BUF', '_CARGO', '_RING', '_BAUD',
         '_WAIT_MS', '_SETTLE_US', '_QUIET_MS', '_HOLD_MS', '_STEP_MS',
         '_LEASE_MS', '_EVERY_MS', '_SETTLE_MS', '_DTG_MAX', '_MIN_NS',
         '_BITS_PER_CHAR', '_MAX_ADDITIONS')


#: Where they live. Two files, one per layer, and everything else is a
#: duplicate: the drivers' numbers and the wire's, so the includes run one
#: way - comms/ may reach down into board/, and board/ never reaches up.
LIMITS = ('board_limits.h', 'comms_limits.h')


#: What the host says a firmware macro is worth, and where the macro lives:
#: (module, name, C file, macro, host units per macro unit). A mirror that
#: drifts is the second answer this tree keeps deleting, so they are held
#: to each other here rather than remembered.
MIRRORS = (
    ('coaxial.model.thermal', 'WINDING_INTO_IRON',
     'thermal/inc/thermal_run.h', 'THERMAL_WINDING_INTO_IRON', 1.0),
    ('coaxial.model.thermal', 'IDENT_MARGIN_FLOOR',
     'comms/inc/board/thermal.h', 'BOARD_SOA_MARGIN_FLOOR_PPM', 1e-6),
    ('coaxial.simulated.values', 'ACCUMULATE_MAX',
     'board/inc/board_limits.h', 'LIVE_MAX_ADDITIONS', 1.0),
    ('coaxial.acquire.daq', 'REPLY_ROOM', 'comms/src/cmd_daq.c', 'DAQ_REPLY_ROOM', 1.0),
    ('coaxial.acquire.bessel', 'MAX_BOXCAR',
     'board/inc/board_limits.h', 'LIVE_MAX_ADDITIONS', 1.0),
    ('coaxial.simulated.values', 'RING_BYTES',
     'board/inc/board_limits.h', 'DAQ_BYTES', 1.0),
    ('coaxial.devices.boot', 'CHUNK', 'boot/inc/boot.h', 'BOOT_CHUNK_BYTES', 1.0),
    ('coaxial.devices.boot', 'UID_BYTES', 'boot/inc/boot.h', 'BOOT_UID_BYTES', 1.0),
    ('coaxial.devices.boot', 'WORD', 'boot/inc/boot.h', 'BOOT_WORD_BYTES', 1.0),
    ('coaxial.devices.boot', 'HEADER', 'boot/inc/boot.h', 'BOOT_HEADER_OFFSET', 1.0),
    ('coaxial.devices.boot', 'MAGIC', 'boot/inc/boot.h', 'BOOT_HEADER_MAGIC', 1.0),
    ('coaxial.devices.boot', 'BLANK_UNIT', 'boot/inc/boot.h', 'BOOT_UNIT', 1.0),
    ('coaxial.devices.boot', 'RUN_BASE', 'boot/inc/boot.h', 'BOOT_RUN_BASE', 1.0),
    ('coaxial.devices.boot', 'RUN_BYTES', 'boot/inc/boot.h', 'BOOT_RUN_BYTES', 1.0),
    ('coaxial.devices.boot', 'STORE_BASE', 'boot/inc/boot.h', 'BOOT_STORE_BASE', 1.0),
    ('coaxial.devices.boot', 'STORE_BYTES', 'boot/inc/boot.h', 'BOOT_STORE_BYTES', 1.0),
    ('coaxial.devices.boot', 'SEAL_MAGIC', 'boot/inc/boot.h', 'BOOT_SEAL_MAGIC', 1.0),
    ('coaxial.devices.boot', 'PERSIST', 'boot/inc/boot.h', 'BOOT_SEAL_PERSIST', 1.0),
    ('coaxial.devices.boot', 'SECTOR', 'boot/inc/boot.h', 'BOOT_SECTOR_BYTES', 1.0),
    ('coaxial.devices.boot', 'RECORD_BASE', 'boot/inc/boot.h', 'BOOT_RECORD_BASE', 1.0),
    ('coaxial.devices.boot', 'RECORD_MAX', 'boot/inc/boot.h', 'BOOT_RECORD_MAX', 1.0),
    ('motor.pmsm', 'HALF_SQRT3',
     'drive/src/drive_math.c', 'HALF_SQRT3', 1.0),
    ('motor.pmsm', 'TWO_PI',
     'drive/src/drive_math.c', 'TWO_PI', 1.0),
)


_NUMBER = re.compile(r'\b(0[xX][0-9a-fA-F]+|\d+(?:\.\d*)?(?:[eE][-+]?\d+)?)[uUlL]*[fF]?\b')


_ARITH = {ast.Add: operator.add, ast.Sub: operator.sub, ast.Mult: operator.mul,
          ast.Div: operator.truediv, ast.UAdd: operator.pos, ast.USub: operator.neg}


def _arith(node):
    """A parsed expression of numbers and + - * /, evaluated; anything else raises."""
    if isinstance(node, ast.Expression):
        return _arith(node.body)
    if isinstance(node, ast.Constant) and type(node.value) in (int, float):
        return node.value
    if isinstance(node, ast.BinOp) and type(node.op) in _ARITH:
        return _ARITH[type(node.op)](_arith(node.left), _arith(node.right))
    if isinstance(node, ast.UnaryOp) and type(node.op) in _ARITH:
        return _ARITH[type(node.op)](_arith(node.operand))
    raise ValueError('not arithmetic: %s' % ast.dump(node))


def _defines(rel):
    """`#define NAME <number or arithmetic>` in one C file, evaluated -
    suffixes and a trailing comment stripped, anything else skipped."""
    found = {}
    for line in io.open(os.path.join(REPO, *rel.split('/')), encoding='utf-8'):
        m = re.match(r'#define\s+(\w+)\s+(.+)', line)
        if not m:
            continue
        expr = _NUMBER.sub(r'\1', m.group(2).split('/*')[0].split('//')[0])
        if re.fullmatch(r'(0[xX][0-9a-fA-F]+|[\d.eE+\-*/() ])+', expr.strip()):
            found[m.group(1)] = _arith(ast.parse(expr.strip(), mode='eval'))
    return found


def _agree(host, target):
    return abs(host - target) <= 1e-6 * max(1.0, abs(target))


def test_subsystem_calls_resolve(r):
    """Every `board.X.y()` in this tree names a method X has."""
    known = _subsystems()

    # The stand-in too, and by the same names.
    from coaxial.rig import Coaxial63100, DaqView
    from coaxial.simulated import SimulatedSession
    stand_in = SimulatedSession().board
    # On a rig, `.daq` is its DaqView, which hands the rest to the rig itself.
    rig_views = {'daq': (DaqView, Coaxial63100)}

    wrong, missing = [], []
    for path, _, tree in sources():
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            call = node.func
            if not isinstance(call, ast.Attribute):
                continue
            owner = call.value
            if not isinstance(owner, ast.Attribute) or owner.attr not in known:
                continue
            on_board = hasattr(known[owner.attr], call.attr)
            if not on_board and not any(hasattr(c, call.attr)
                                        for c in rig_views.get(owner.attr, ())):
                wrong.append('%s:%d %s.%s'
                             % (path, node.lineno, owner.attr, call.attr))
            elif on_board and not hasattr(getattr(stand_in, owner.attr, None), call.attr):
                missing.append('%s:%d %s.%s'
                               % (path, node.lineno, owner.attr, call.attr))

    r.check('every board.X.y() names a method X has',
            not wrong, '; '.join(wrong[:4]))
    r.check('and the stand-in answers it too',
            not missing, '; '.join(missing[:4]))


def test_limits_live_in_one_file(r):
    """No fixed number is defined outside the two limits headers."""
    import glob

    stray = []
    for where in ('board/src/*.c', 'board/inc/*.h', 'comms/src/*.c',
                  'comms/inc/*.h', 'thermal/src/*.c', 'thermal/inc/*.h'):
        for path in glob.glob(os.path.join(REPO, *where.split('/'))):
            if os.path.basename(path) in LIMITS:
                continue
            for line in io.open(path, encoding='utf-8'):
                if not line.startswith('#define '):
                    continue
                name = line.split()[1]
                if any(name.endswith(tail) for tail in OWNED):
                    stray.append('%s: %s' % (os.path.basename(path), name))

    r.check('every fixed number is defined in a limits header only',
            not stray, '; '.join(stray[:4]))

    # The layering, which is the reason there are two.
    up = [os.path.basename(path)
          for where in ('board/src/*.c', 'board/inc/*.h')
          for path in glob.glob(os.path.join(REPO, *where.split('/')))
          if '#include "comms_limits.h"'
          in io.open(path, encoding='utf-8').read()]
    r.check('nothing in board/ includes the comms limits',
            not up, ', '.join(up))


def test_mirrors_agree(r):
    """The host's copies of firmware constants are the firmware's."""
    off = []
    for module, name, rel, macro, scale in MIRRORS:
        host = getattr(importlib.import_module(module), name)
        target = _defines(rel).get(macro)
        if target is None or not _agree(host, target * scale):
            off.append('%s.%s = %r, %s %s = %r' % (module, name, host, rel,
                                                   macro, target))
    r.check('every named mirror is the macro it names', not off,
            '; '.join(off[:3]) or '%d pairs' % len(MIRRORS))

    from coaxial.comm import protocol
    header = dict(_defines('comms/inc/cmd.h'), **_defines('boot/inc/boot.h'))
    devices = {k: v for k, v in header.items() if k.startswith('DEVICE_')}
    wrong = ['%s: cmd.h %d, protocol %r' % (k, v, getattr(protocol, k, None))
             for k, v in sorted(devices.items()) if getattr(protocol, k, None) != v]
    r.check('protocol.py numbers the devices as cmd.h does',
            len(devices) >= 11 and not wrong,
            '; '.join(wrong[:3]) or '%d devices' % len(devices))

    ops = {}
    for k, v in header.items():
        m = re.fullmatch(r'(\w+?)_OP_(\w+)', k)
        if m:
            ops.setdefault(m.group(1), {})[m.group(2)] = v
    wrong = []
    for prefix, table in sorted(ops.items()):
        cls = getattr(protocol, OP_CLASSES.get(prefix, ''), None)
        members = {m.name: m.value for m in cls} if cls else {}
        wrong += ['%s_OP_%s: cmd.h %d, %s %r' % (prefix, op, v, OP_CLASSES.get(prefix),
                                                 members.get(op))
                  for op, v in sorted(table.items()) if members.get(op) != v]
        wrong += ['%s.%s: no %s_OP_%s in cmd.h' % (OP_CLASSES.get(prefix), op, prefix, op)
                  for op in sorted(set(members) - set(table))]
    r.check('and every op by name and number, both ways',
            len(ops) >= 10 and not wrong,
            '; '.join(wrong[:3]) or '%d op tables' % len(ops))

    from coaxial.kalman import thermal_ident as mirror
    named = dict(_defines('thermal/src/thermal_ident.c'),
                 **_defines('thermal/inc/thermal_ident.h'))
    pairs = [(k, v, k.split('IDENT_', 1)[1]) for k, v in named.items()
             if 'IDENT_' in k and hasattr(mirror, k.split('IDENT_', 1)[1])]
    off = ['%s %r != thermal_ident.%s %r' % (k, v, n, getattr(mirror, n))
           for k, v, n in pairs if not _agree(getattr(mirror, n), v)]
    r.check("the identifier's constants are thermal_ident.c's, by name",
            len(pairs) >= 10 and not off,
            '; '.join(off[:3]) or '%d by name' % len(pairs))


def test_dimensions_page_is_the_drives(r):
    """docs/DIMENSIONS.md is what `tools/sim/dimensions.py` writes from the drives as stacked."""
    from tools.sim import dimensions
    with open(dimensions.PAGE, encoding='utf-8') as f:
        page = f.read()
    r.check("the dimensions page is its tool's output", page == dimensions.text(),
            'python tools/sim/dimensions.py --write')


ROSTER = (test_subsystem_calls_resolve, test_limits_live_in_one_file, test_mirrors_agree,
          test_dimensions_page_is_the_drives)

def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed, %d skipped'
          % (report.passed, report.failed, getattr(report, 'skipped', 0)))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
