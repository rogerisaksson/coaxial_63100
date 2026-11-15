"""The host mirrors the firmware: subsystem calls resolve, limits live in one file, the mirrors agree."""
import ast
import glob
import importlib
import inspect
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
    ('coaxial.devices.angle', 'IDLE_HZ', 'board/inc/board_limits.h', 'ANGLE_IDLE_HZ', 1.0),
    ('coaxial.devices.angle', 'STALE_READS', 'board/src/board_angle.c', 'ANGLE_STALE', 1.0),
    ('tools.cores.native', 'HOST_QUIET_S', 'board/inc/board_limits.h', 'BOARD_POWER_HOST_QUIET_MS', 1e-3),
    ('machine.stack.wbc', 'MU', 'wbc/inc/wbc.h', 'WBC_MU', 1.0),
    ('machine.stack.wrench', 'ROUNDS', 'wbc/inc/wbc.h', 'WBC_ROUNDS', 1.0),
    ('machine.stack.wrench', 'PAST', 'wbc/inc/wbc.h', 'WBC_PAST', 1.0),
    ('machine.texts.ortho', 'N', 'wbc/inc/wbc_ortho.h', 'ORTHO_MODES', 1.0),
    ('machine.texts.ortho', 'P', 'wbc/inc/wbc_ortho.h', 'ORTHO_P', 1.0),
    ('machine.texts.ortho', 'Z', 'wbc/inc/wbc_ortho.h', 'ORTHO_Z', 1.0),
    ('machine.texts.ortho', 'K', 'wbc/inc/wbc_ortho.h', 'ORTHO_K', 1.0),
    ('machine.texts.ortho', 'D', 'wbc/inc/wbc_ortho.h', 'ORTHO_D', 1.0),
    ('machine.texts.ortho', 'AMP', 'wbc/inc/wbc_ortho.h', 'ORTHO_AMP', 1.0),
    ('machine.texts.ortho', 'PHASE', 'wbc/inc/wbc_ortho.h', 'ORTHO_PHASE', 1.0),
    ('machine.texts.ortho', 'GAIN', 'wbc/inc/wbc_ortho.h', 'ORTHO_GAIN', 1.0),
    ('machine.texts.ortho', 'HZ', 'wbc/inc/wbc_ortho.h', 'ORTHO_HZ', 1.0),
    ('machine.texts.ortho', 'ZMP', 'wbc/inc/wbc_ortho.h', 'ORTHO_ZMP', 1.0),
    ('machine.texts.ortho', 'YAW', 'wbc/inc/wbc_ortho.h', 'ORTHO_YAW', 1.0),
    ('machine.texts.ortho', 'SLIDE', 'wbc/inc/wbc_ortho.h', 'ORTHO_SLIDE', 1.0),
    ('machine.texts.ortho', 'REST', 'wbc/inc/wbc_ortho.h', 'ORTHO_REST', 1.0),
    ('machine.texts.ortho', 'REST_MS', 'wbc/inc/wbc_ortho.h', 'ORTHO_REST_MS', 1.0),
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
    ('tools.cores.sensed', 'P', 'wbc/inc/wbc_move.h', 'WBC_SENSED_PHASES', 1.0),
    ('tools.cores.sensed', 'Q', 'wbc/inc/wbc_move.h', 'WBC_SENSED_PARTS', 1.0),
    ('tools.cores.sensed', 'M', 'wbc/inc/wbc_move.h', 'WBC_SENSED_MOVED', 1.0),
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


def test_the_robots_manifest_is_its_build(r):
    """The robot's orthokinematic manifest is what `tools/cores/manifest.py` writes of its build's
    file and its figure, and its text compiles it; the robot's build names only the library's
    parts."""
    from machine.body import defrobot, mechatronics
    from machine.texts import ortho
    from machine.errors import MachineError
    from tools.cores import manifest
    with open(manifest.path(), encoding='utf-8') as f:
        page = f.read()
    r.check("the robot's manifest is its tool's output", page == manifest.text(), 'python tools/cores/manifest.py --write')
    read = defrobot.read()
    r.check('its text includes it: the robot\'s mass, its modes, its drives', read is not None
            and read['name'] == mechatronics.ROBOT and len(read['modes']) == ortho.N and 'left_knee' in read['drives'],
            read and (read['body'].get('mass'), len(read['drives'])))
    build, lib = mechatronics.robot(), mechatronics.library()
    r.check("each stack's motor, gearbox and inverter is the library's", all(
        named in lib[part] for stack in build['stacks'].values() for part, named in zip(mechatronics.PARTS, stack)))
    was = build['stacks']['knee']
    build['stacks']['knee'] = ['U9', 'B', 'coaxial_63100']
    try:
        mechatronics.robot()
        refused = ''
    except MachineError as exc:
        refused = str(exc)
    finally:
        build['stacks']['knee'] = was
    r.check('a stack naming a part the library lacks is refused, the parts named',
            'U9 is no motor' in refused and 'MN6007' in refused, refused)
    import subprocess
    host = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    for other in [n for n in mechatronics.robots() if n != mechatronics.ROBOT]:
        run = subprocess.run([sys.executable, '-X', 'utf8', '-c', 'from tools.cores import manifest\n'
                              'raise SystemExit(0 if open(manifest.path(), encoding="utf-8").read() == manifest.text() else 1)'],
                             cwd=host, env=dict(os.environ, COAXIAL_ROBOT=other, PYTHONPATH=host),
                             capture_output=True, text=True, timeout=300)
        r.check('%s\'s manifest is its tool\'s output under COAXIAL_ROBOT' % other, run.returncode == 0,
                run.stderr[-300:] or 'COAXIAL_ROBOT=%s python tools/cores/manifest.py --write' % other)
    import re

    def peaks(robot):
        with open(os.path.join(mechatronics.LISP, 'humanoid', 'builds', robot + '.manifest.lisp'), encoding='utf-8') as f:
            return {d: float(p) for d, p in re.findall(r'\(drive (\S+) .*?\(peak ([-\d.]+)\)', f.read())}
    robot_own, faster = peaks(mechatronics.ROBOT), peaks(mechatronics.ROBOT + '_performance')
    weaker = {d: (p, faster.get(d)) for d, p in robot_own.items() if faster.get(d, -1.0) < p}
    r.check('its performance build is no weaker than the robot\'s own at any drive', bool(robot_own) and not weaker, weaker)


def test_static_compiler_and_generators(r):
    """`mechatronics.compiled` caches by SHA-256 and rebuilds on change; generators honour per-box
    ratios and lay `term_rows.py`, `footprint_rows.py` and `wbc_model.h` verbatim."""
    import tempfile
    from machine.body import footprint_rows, mechatronics
    from machine.stack import wbc
    from tools.cores import footprint, model, terms
    from tools.sim import bom, dimensions
    key = '_test_gen_%d' % os.getpid()
    cache_file = os.path.join(mechatronics.CACHE, key + '.json')
    calls = 0
    try:
        with tempfile.TemporaryDirectory() as tmp:
            src = os.path.join(tmp, 'spec.toml')
            with open(src, 'w', encoding='utf-8') as f:
                f.write('k = 1\n')

            def build_it():
                nonlocal calls
                calls += 1
                return {'n': calls, 'pair': (1.5, 2.5)}

            first = mechatronics.compiled(key, (src,), build_it)
            second = mechatronics.compiled(key, (src,), build_it)
            mechatronics._COMPILED.pop(key, None)
            reloaded = mechatronics.compiled(key, (src,), build_it)
            with open(src, 'w', encoding='utf-8') as f:
                f.write('k = 22\n')
            rebuilt = mechatronics.compiled(key, (src,), build_it)
        r.check('mechatronics.compiled caches tuples on disk and rebuilds on SHA-256 change',
                calls == 2 and first == second == reloaded and isinstance(reloaded['pair'], tuple)
                and rebuilt['n'] == 2, (calls, reloaded, rebuilt))
    finally:
        mechatronics._COMPILED.pop(key, None)
        if os.path.exists(cache_file):
            os.remove(cache_file)

    lib = mechatronics.library()
    multi = dict(lib, gearboxes=dict(lib['gearboxes'],
                                     A=dict(lib['gearboxes']['A'], ratio=50.0, input_j_share=0.08),
                                     B=dict(lib['gearboxes']['B'], ratio=40.0, input_j_share=0.06)))
    dim_multi = dimensions.text(multi, [('U12', ('U12', 'A', 'coaxial_63100'), ['knee']),
                                        ('U8', ('U8', 'B', 'coaxial_63100'), ['hip_yaw'])])
    bom_multi = dict(bom.parts(multi))
    r.check('dimensions and bom honour each gearbox\'s own ratio and rotor inertia share',
            '84 mm, 1:50' in dim_multi and '64 mm, 1:40' in dim_multi
            and bom_multi.get('gearbox B, 1:40', 0) > 0, (dim_multi[:240], bom_multi.get('gearbox B, 1:40')))

    with open(terms.PATH, encoding='utf-8') as f:
        terms_disk = f.read()
    with open(footprint.PATH, encoding='utf-8') as f:
        fp_disk = f.read()
    fp_dict = {k: getattr(footprint_rows, k) for k in (
        'K', 'BAND_M', 'TURN_DEG', 'STEP_M', 'STAND', 'WALK', 'BACK', 'SIDE', 'LEFT', 'RIGHT',
        'CARRY_LEFT', 'SPREAD_LEFT', 'TOOK_LEFT', 'CARRY_RIGHT', 'SPREAD_RIGHT', 'TOOK_RIGHT')}
    b = wbc.Body()
    hdr, _src = model.render(model.links(b), b, stand=([0.0] * 12, [0.0] * (b.n - 6)))
    with open(model.HEADER, encoding='utf-8') as f:
        hdr_disk = f.read()
    r.check('terms, footprint and wbc_model.h match their generators\' output',
            terms.render(terms.rows()) == terms_disk and footprint.text(fp_dict) == fp_disk and hdr == hdr_disk)


def test_footprint_rows_hold_their_shape(r):
    """machine/body/footprint_rows.py as `tools/cores/footprint.py` lays it: K reaches a band, a turn each of
    TURN_DEG, each reach a length (its text is held by test_static_compiler_and_generators)."""
    from machine.body import footprint_rows as rows
    names = ('LEFT', 'RIGHT', 'CARRY_LEFT', 'CARRY_RIGHT', 'SPREAD_LEFT', 'SPREAD_RIGHT', 'TOOK_LEFT', 'TOOK_RIGHT')
    turns, bands = len(rows.TURN_DEG), ('STAND', 'WALK', 'BACK', 'SIDE')
    wide = all(len(row) == rows.K for name in bands for row in getattr(rows, name))
    turned = (all(len(getattr(rows, name)) == turns for name in names)
              and all(len(row) == rows.K for name in ('LEFT', 'RIGHT') for h in getattr(rows, name) for row in h))
    r.check('K reaches a band, a turn each of TURN_DEG', wide and turned, (rows.K, turns))
    reaches = [v for name in bands for row in getattr(rows, name) for v in row]
    reaches += [v for name in ('LEFT', 'RIGHT') for h in getattr(rows, name) for row in h for v in row]
    r.check('each reach a length, its bands and steps too', rows.BAND_M > 0.0 and rows.STEP_M > 0.0
            and all(v >= 0.0 for v in reaches) and all(t > 0.0 for t in rows.TOOK_LEFT + rows.TOOK_RIGHT),
            (min(reaches), rows.BAND_M, rows.STEP_M))


def test_generated_files_name_their_tools(r):
    """Every file a tool writes names the tool on its first line (`token_budget.GENERATED`), LF-only, and
    every tracked file saying so is one of theirs: the generators' outputs and the notices, one set."""
    import subprocess
    from machine.body import mechatronics
    from tools.cores import footprint, manifest, model, reach, terms
    from tools.dev import cover, pinned, token_budget
    from tools.sim import dimensions
    host = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    outputs = {footprint.PATH: footprint, reach.PATH: reach, terms.PATH: terms, model.HEADER: model,
               model.SOURCE: model, dimensions.PAGE: dimensions, str(pinned.LOCK): pinned, cover.PAGE: cover}
    outputs.update((manifest.path(robot), manifest) for robot in mechatronics.robots())

    def key(path):
        return os.path.normcase(os.path.relpath(os.path.abspath(path), REPO)).replace(os.sep, '/')
    unnamed, crlf = [], []
    for path, tool in outputs.items():
        named = os.path.relpath(inspect.getfile(tool), host).replace(os.sep, '/')
        with open(path, 'rb') as f:
            first = f.readline()
            crlf += [key(path)] if b'\r' in first + f.read() else []
        if token_budget.GENERATED.encode() not in first or named.encode() not in first:
            unnamed.append((key(path), named))
    r.check('%d outputs of %d tools, each naming its tool on its first line' % (len(outputs), len({
        t.__name__ for t in outputs.values()})), not unnamed, unnamed)
    r.check('each LF-only', not crlf, crlf)
    tracked = subprocess.run(['git', 'ls-files', '-z'], cwd=REPO, capture_output=True, check=True).stdout
    theirs, strays = {key(p) for p in outputs}, []
    files = (os.path.join(REPO, rel) for rel in filter(None, tracked.decode('utf-8').split('\0')))
    for full in filter(os.path.isfile, files):           # a submodule's gitlink is no file
        with open(full, 'rb') as f:
            if token_budget.GENERATED.encode() in f.readline() and key(full) not in theirs:
                strays.append(key(full))
    r.check('every tracked file saying a tool writes it is one of theirs', not strays, strays)


ROSTER = (test_subsystem_calls_resolve, test_limits_live_in_one_file, test_mirrors_agree,
          test_dimensions_page_is_the_drives, test_the_robots_manifest_is_its_build,
          test_static_compiler_and_generators, test_footprint_rows_hold_their_shape,
          test_generated_files_name_their_tools)

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
