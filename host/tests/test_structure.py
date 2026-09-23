#!/usr/bin/env python3
"""Does `host/` still hold together? No board, no model, no network.

Every check here is a defect that actually happened while moving code around:
a module that stopped importing, a name left behind in two files at once, a
re-export that pointed nowhere, an import nothing used any more. The
behavioural suites cannot see any of it - they import what they need and pass
while the rest of the package is broken.

Run it after editing anything under host/:

    .\\run_tests.ps1 -Structure
    python tests/test_structure.py
"""
import ast
import builtins
import importlib
import io
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
HOST = os.path.dirname(HERE)
REPO = os.path.dirname(HOST)
sys.path.insert(0, HOST)

# Packages this suite walks.
PACKAGES = ('coaxial', 'coaxial_mcp', 'coaxial_ollama', 'testline', 'terminal')
SCRIPTS = ('tools', 'examples')

#: Outside host/, and judged the same: a reader copies from these.
#: Notebooks included - their code cells are concatenated and parsed as
#: one module, so a rename that leaves `print(daq)` behind in a cell
#: fails here exactly as it did when the examples were .py files.
BESIDE = ('../notebook_examples',)

# The ceiling is the worst that survives a deliberate reading, not an ideal.
MAX_LINES = 130
MAX_DEPTH = 7

# Character-by-character state machines, where the nesting *is* the machine and
# flattening it would cost clarity rather than buy any.
DEEP_BY_NATURE = {'json_objects', '_gpus_registry'}

NESTS = (ast.If, ast.For, ast.While, ast.With, ast.Try, ast.ExceptHandler)


class Report:
    def __init__(self):
        self.passed = self.failed = 0

    def check(self, what, ok, detail=''):
        self.passed += bool(ok)
        self.failed += not ok
        print('  %s  %-52s %s' % ('PASS' if ok else 'FAIL', what,
                                  '' if ok else detail))


def modules():
    """Every importable module under the packages, as dotted names."""
    found = []
    for package in PACKAGES:
        root = os.path.join(HOST, package)
        for here, dirs, names in os.walk(root):
            dirs[:] = [d for d in dirs if d != '__pycache__']
            dotted = os.path.relpath(here, HOST).replace(os.sep, '.')
            for name in sorted(names):
                if name.endswith('.py') and not name.startswith('_'):
                    found.append('%s.%s' % (dotted, name[:-3]))
    return found


def sources(beside=True):
    """(path, tree) for everything this suite judges, scripts included."""
    out = []
    for where in PACKAGES + SCRIPTS + (BESIDE if beside else ()):
        root = (os.path.join(REPO, where[3:]) if where.startswith('../')
                else os.path.join(HOST, where))
        if not os.path.isdir(root):
            continue
        for here, dirs, names in os.walk(root):
            dirs[:] = [d for d in dirs if d != '__pycache__']
            for name in sorted(names):
                path = os.path.join(here, name)
                if name.endswith('.py'):
                    text = io.open(path, encoding='utf-8').read()
                elif name.endswith('.ipynb'):
                    text = notebook_source(path)
                else:
                    continue
                rel = os.path.relpath(path, root)
                out.append((os.path.join(where, rel), text, ast.parse(text)))
    return out


def notebook_source(path):
    """A notebook's code cells as one module, for the same AST checks."""
    import json
    with io.open(path, encoding='utf-8') as handle:
        cells = json.load(handle)['cells']
    return '\n\n'.join(''.join(c['source']) for c in cells
                       if c['cell_type'] == 'code')


def depth(node, at=0):
    worst = at
    for child in ast.iter_child_nodes(node):
        worst = max(worst, depth(child, at + isinstance(child, NESTS)))
    return worst


def test_imports(r):
    """Every module imports on its own, from a cold interpreter."""
    for name in modules():
        try:
            importlib.import_module(name)
            r.check('%s imports' % name, True)
        except Exception as exc:                              # noqa: BLE001
            r.check('%s imports' % name, False,
                    '%s: %s' % (type(exc).__name__, exc))


def test_no_cycles(r):
    """No package module imports another that imports it back."""
    edges = {}
    for path, _, tree in sources():
        if not path.startswith(PACKAGES):
            continue
        package, _, name = path.replace('\\', '/').partition('/')
        me = '%s.%s' % (package, name[:-3])
        edges[me] = set()
        for node in tree.body:                # top-level imports only
            if isinstance(node, ast.ImportFrom) and node.level == 1:
                edges[me].add('%s.%s' % (package, node.module))
            elif isinstance(node, ast.ImportFrom) and node.module:
                edges[me].add(node.module)
    back = [(a, b) for a, seen in edges.items() for b in seen
            if a in edges.get(b, ())]
    r.check('no module imports one that imports it back at the top',
            not back, repr(back))


def test_reexports(r):
    """Every name debug.py says lives elsewhere actually does."""
    from coaxial_ollama import debug
    for name, where in sorted(debug._ELSEWHERE.items()):
        try:
            got = getattr(debug, name)
            module = importlib.import_module('coaxial_ollama.' + where)
            r.check('debug.%s comes from %s' % (name, where),
                    getattr(module, name, None) is got)
        except Exception as exc:                              # noqa: BLE001
            r.check('debug.%s comes from %s' % (name, where), False, str(exc))
    try:
        debug.no_such_name_at_all
        r.check('and an unknown name still raises AttributeError', False)
    except AttributeError:
        r.check('and an unknown name still raises AttributeError', True)


def test_no_duplicate_definitions(r):
    """A name defined at the top level of two modules in one package."""
    seen = {}
    for path, text, tree in sources(beside=False):
        lines = text.split(chr(10))
        for node in tree.body:
            names = []
            if isinstance(node, (ast.FunctionDef, ast.ClassDef)):
                names = [node.name]
            elif isinstance(node, ast.Assign):
                names = [t.id for t in node.targets
                         if isinstance(t, ast.Name) and t.id.isupper()]
            # The *body*, not just the name.
            body = chr(10).join(lines[node.lineno - 1:node.end_lineno])
            for name in names:
                seen.setdefault((name, body.strip()), []).append(path)
    twice = {name: where for (name, _), where in seen.items()
             if len(where) > 1
             and len({os.path.dirname(p) for p in where}) == 1}
    r.check('no definition is copied into two files of one package',
            not twice, '; '.join('%s in %s' % (n, ', '.join(w))
                                 for n, w in sorted(twice.items())[:4]))


def test_no_unused_imports(r):
    """An import nothing references. Left behind by every move."""
    for path, text, tree in sources():
        if path.endswith('__init__.py'):
            continue        # re-exporting is what a package __init__ is for
        used = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Name):
                used.add(node.id)
            elif isinstance(node, ast.Attribute) and isinstance(node.value,
                                                                ast.Name):
                used.add(node.value.id)
        brought = []
        for node in tree.body:
            if isinstance(node, ast.Import):
                brought += [a.asname or a.name.split('.')[0]
                            for a in node.names]
            elif isinstance(node, ast.ImportFrom):
                brought += [a.asname or a.name for a in node.names]
        dead = [n for n in brought if n not in used and n not in text.split()]
        r.check('%s imports nothing it does not use' % path,
                not dead, ', '.join(dead))


def _names_imported(node):
    """The top-level package names one import statement brings in."""
    if isinstance(node, ast.Import):
        return [a.name.split('.')[0] for a in node.names]
    if isinstance(node, ast.ImportFrom) and node.module:
        return [node.module.split('.')[0]]
    return []


def _caps_openblas(node):
    """Whether `node` is `os.environ.setdefault('OPENBLAS_NUM_THREADS', ...)`."""
    call = getattr(node, 'value', None)
    return (isinstance(node, ast.Expr) and isinstance(call, ast.Call)
            and ast.unparse(call.func) == 'os.environ.setdefault'
            and bool(call.args)
            and getattr(call.args[0], 'value', None) == 'OPENBLAS_NUM_THREADS')


def test_numpy_enters_behind_the_thread_cap(r):
    """numpy is imported at module level in one package module, loop.py, and
    that module caps OpenBLAS's thread pool before importing it.
    """
    importers, capped = [], False
    for path, _text, tree in sources(beside=False):
        if not path.startswith(PACKAGES):
            continue
        for i, node in enumerate(tree.body):
            if 'numpy' in _names_imported(node):
                importers.append(path)
                capped = any(_caps_openblas(n) for n in tree.body[:i])
                break
    r.check('numpy enters the packages at module level in loop.py alone',
            importers == [os.path.join('coaxial', 'loop.py')],
            ', '.join(importers) or 'nowhere')
    r.check('and loop.py sets OPENBLAS_NUM_THREADS before importing it',
            capped)


def _bound(tree):
    """Every name a module binds, anywhere: imports, defs, targets, args."""
    names = set(vars(builtins)) | {'__file__', '__name__',
                                   '__doc__', '__package__'}
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names |= {a.asname or a.name.split('.')[0] for a in node.names}
        elif isinstance(node, ast.ImportFrom):
            names |= {a.asname or a.name for a in node.names}
        elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef,
                               ast.ClassDef)):
            names.add(node.name)
            args = getattr(node, 'args', None)
            if args:
                for group in (args.args, args.posonlyargs, args.kwonlyargs):
                    names |= {a.arg for a in group}
                for one in (args.vararg, args.kwarg):
                    if one:
                        names.add(one.arg)
        elif isinstance(node, ast.Lambda):
            for group in (node.args.args, node.args.kwonlyargs):
                names |= {a.arg for a in group}
        elif isinstance(node, ast.ExceptHandler) and node.name:
            names.add(node.name)
        elif isinstance(node, ast.Global):
            names |= set(node.names)
        elif isinstance(node, ast.Name) and isinstance(node.ctx,
                                                       (ast.Store, ast.Del)):
            names.add(node.id)
    return names


def test_no_undefined_names(r):
    """A name used that nothing in the module defines or imports."""
    for path, _, tree in sources():
        known = _bound(tree)
        used = {n.id for n in ast.walk(tree)
                if isinstance(n, ast.Name) and isinstance(n.ctx, ast.Load)}
        loose = sorted(used - known)
        r.check('%s uses only names it has' % path,
                not loose, ', '.join(loose[:6]))


def test_shape(r):
    """No function past the length or nesting a reader can hold."""
    long_ones, deep_ones = [], []
    for path, _, tree in sources():
        for node in ast.walk(tree):
            # AsyncFunctionDef too: it is not a subclass of FunctionDef, so the
            # three async handlers in coaxial_mcp/server.py were exempt from
            # both ceilings without anyone deciding they should be.
            if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                continue
            lines = (node.end_lineno or node.lineno) - node.lineno + 1
            if lines > MAX_LINES:
                long_ones.append('%s:%s %d lines' % (path, node.name, lines))
            if depth(node) > MAX_DEPTH and node.name not in DEEP_BY_NATURE:
                deep_ones.append('%s:%s %d deep'
                                 % (path, node.name, depth(node)))
    r.check('no function is longer than %d lines' % MAX_LINES,
            not long_ones, '; '.join(long_ones[:3]))
    r.check('no function nests deeper than %d' % MAX_DEPTH,
            not deep_ones, '; '.join(deep_ones[:3]))


def test_documented(r):
    """Every module, class and public function says what it is for."""
    # Modules and classes only.
    from tools.host_map import brief
    missing, long_ = [], []
    for path, _, tree in sources(beside=False):
        if not ast.get_docstring(tree):
            missing.append(path + ' (module)')
        elif brief(ast.get_docstring(tree)).endswith('...'):
            long_.append(path)
        # Top level only: a ctypes Structure declared inside a function is a
        # field list, and a docstring on it would say less than its name.
        for node in tree.body:
            if not isinstance(node, ast.ClassDef):
                continue
            if node.name.startswith('_') or ast.get_docstring(node):
                continue
            missing.append('%s:%s' % (path, node.name))
    r.check('every module and class says what it is for',
            not missing, '; '.join(missing[:4]))
    r.check('every module opens on a one-line brief (tools/host_map.py)',
            not long_, '; '.join(long_[:4]))


def test_no_escaping_scars(r):
    """chr(10) and chr(92) where a literal belongs."""
    for path, text, tree in sources():
        if path.endswith(('replies.py', 'sandbox.py')):
            continue        # these are *about* escaping: the first two by
                            # their tests, the third because a backslash is
                            # what a PDF literal string escapes with
        scars = [w for w in ('chr(10)', 'chr(92)') if w in text]
        r.check('%s has no heredoc scars' % path, not scars, ', '.join(scars))


#: Prose that quotes a suite size. Each drifts the moment a check is added,
#: and each was wrong at once: CLAUDE.md said 1679 with structure at 300
#: when it was 349, and three more files carried a third total.
COUNTED = ('CLAUDE.md', 'host/run_tests.ps1', 'docs/ARCHITECTURE.md',
           'docs/TODO.md')


def test_counts_are_measured(r):
    """Every suite size written in prose matches `.counts.json`."""
    import glob
    import json

    try:
        suites = json.loads(
            open(os.path.join(HERE, '.counts.json'), encoding='utf-8').read()
        )['suites']
    except (OSError, ValueError, KeyError):
        suites = {}
    if not suites:
        # A fresh clone has measured nothing - .counts.json is per-machine by
        # design.
        r.check('no suite sizes measured on this machine yet - the '
                'documents keep the last measured totals', True)
        return
    total = sum(suites.values())

    # A total needs every suite, and a tier runs a subset by design - the
    # default one runs two.
    everything = set(os.path.basename(p)
                     for p in glob.glob(os.path.join(HERE, 'test_*.py')))
    complete = not (everything - set(suites))

    for name in COUNTED:
        path = os.path.join(REPO, *name.split('/'))
        text = open(path, encoding='utf-8').read()

        wrong = ['%s (%d)' % (suite, said)
                 for suite, said in re.findall(
                     r'`(test_\w+\.py)`\s*\((\d+)', text)
                 for said in [int(said)]
                 if suites.get(suite, said) != said]
        r.check('%s quotes every suite at its size' % name,
                not wrong, ', '.join(wrong))

        totals = set(int(n) for n in re.findall(r"tree's (\d+) checks", text))
        totals |= set(int(n) for n in
                      re.findall(r'suites, (\d+) checks', text))
        totals |= set(int(n) for n in re.findall(r'\| (\d+) checks,', text))
        if complete:
            r.check('%s quotes the total as %d' % (name, total),
                    totals <= {total},
                    ', '.join(str(n) for n in sorted(totals)))
        else:
            r.check('%s keeps its total until all %d suites have run here'
                    % (name, len(everything)), True)


#: `board.<name>.<method>()` is how every view and tool reaches the hardware.
#: The names come from Board itself, so adding a subsystem needs nothing here.
def _subsystems():
    from coaxial.board import Board

    return Board.parts()                 # the declaration, no transport


def test_subsystem_calls_resolve(r):
    """Every `board.X.y()` in this tree names a method X actually has."""
    known = _subsystems()

    # The stand-in too, and by the same names.
    from coaxial.simulated import SimulatedSession
    stand_in = SimulatedSession().board

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
            if not hasattr(known[owner.attr], call.attr):
                wrong.append('%s:%d %s.%s'
                             % (path, node.lineno, owner.attr, call.attr))
            elif not hasattr(getattr(stand_in, owner.attr, None), call.attr):
                missing.append('%s:%d %s.%s'
                               % (path, node.lineno, owner.attr, call.attr))

    r.check('every board.X.y() names a method X has',
            not wrong, '; '.join(wrong[:4]))
    r.check('and the stand-in answers it too',
            not missing, '; '.join(missing[:4]))


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


#: What the host says a firmware macro is worth, and where the macro lives:
#: (module, name, C file, macro, host units per macro unit). A mirror that
#: drifts is the second answer this tree keeps deleting, so they are held
#: to each other here rather than remembered.
MIRRORS = (
    ('coaxial.thermal', 'WINDING_INTO_IRON',
     'board/src/board_thermal.c', 'WINDING_INTO_IRON', 1.0),
    ('coaxial.thermal', 'IDENT_MARGIN_FLOOR',
     'comms/inc/board/thermal.h', 'BOARD_SOA_MARGIN_FLOOR_PPM', 1e-6),
    ('coaxial.simulated.values', 'ACCUMULATE_MAX',
     'board/inc/board_limits.h', 'LIVE_MAX_ADDITIONS', 1.0),
    ('coaxial.bessel', 'MAX_BOXCAR',
     'board/inc/board_limits.h', 'LIVE_MAX_ADDITIONS', 1.0),
    ('coaxial.simulated.values', 'RING_BYTES',
     'board/inc/board_limits.h', 'DAQ_BYTES', 1.0),
    ('coaxial.boot', 'CHUNK', 'boot/inc/boot.h', 'BOOT_CHUNK_BYTES', 1.0),
    ('coaxial.boot', 'UID_BYTES', 'boot/inc/boot.h', 'BOOT_UID_BYTES', 1.0),
    ('coaxial.boot', 'WORD', 'boot/inc/boot.h', 'BOOT_WORD_BYTES', 1.0),
    ('coaxial.boot', 'HEADER', 'boot/inc/boot.h', 'BOOT_HEADER_OFFSET', 1.0),
    ('coaxial.boot', 'MAGIC', 'boot/inc/boot.h', 'BOOT_HEADER_MAGIC', 1.0),
    ('coaxial.boot', 'BLANK_UNIT', 'boot/inc/boot.h', 'BOOT_UNIT', 1.0),
    ('coaxial.boot', 'APP_BASE', 'boot/inc/boot.h', 'BOOT_APP_BASE', 1.0),
    ('coaxial.boot', 'APP_BYTES', 'boot/inc/boot.h', 'BOOT_APP_BYTES', 1.0),
    ('coaxial.boot', 'RECORD_BASE', 'boot/inc/boot.h', 'BOOT_RECORD_BASE', 1.0),
    ('coaxial.boot', 'RECORD_MAX', 'boot/inc/boot.h', 'BOOT_RECORD_MAX', 1.0),
    ('coaxial.sensorless', 'HALF_SQRT3',
     'drive/src/drive_math.c', 'HALF_SQRT3', 1.0),
    ('coaxial.sensorless', 'TWO_PI',
     'drive/src/drive_math.c', 'TWO_PI', 1.0),
)

#: The command header's op prefixes and the protocol module's enums. The
#: rails have one op and no enum on the C side.
OP_CLASSES = {'IMU': 'ImuOp', 'ANGLE': 'AngleOp', 'LINK': 'LinkOp',
              'CAL': 'CalOp', 'GATEDRIVERS': 'GateOp', 'LOG': 'LogOp',
              'DAQ': 'DaqOp', 'TIME': 'TimeOp', 'THERMAL': 'ThermalOp',
              'DRIVE': 'DriveOp', 'POWER': 'PowerOp', 'BOOT': 'BootOp'}

_NUMBER = re.compile(r'\b(0[xX][0-9a-fA-F]+|\d+(?:\.\d*)?(?:[eE][-+]?\d+)?)[uUlL]*[fF]?\b')


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
            found[m.group(1)] = eval(expr)          # noqa: S307 - numbers only
    return found


def _agree(host, target):
    return abs(host - target) <= 1e-6 * max(1.0, abs(target))


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

    from coaxial import protocol
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


#: A fixed-shape reply written field by field in C and read field by field
#: in Python, and the two sequences held to each other: the C file and
#: handler, the Python module, class and method. PROTOCOL.md describes the
#: same shape in prose, which no parser holds; these two are what a wire
#: actually crosses, and the C's own comments say what one moved offset
#: costs every decoder.
WIRE_SHAPES = (
    ('comms/src/cmd_gate_drivers.c', 'h_gate_drivers_state',
     'coaxial.gate_drivers', 'GateDrivers', 'state'),
    ('comms/src/cmd_drive.c', 'h_drive_state', 'coaxial.drive', 'Drive', 'state'),
    ('comms/src/cmd_daq.c', 'h_daq_state', 'coaxial.daq', 'Daq', 'state'),
    ('comms/src/cmd_drive.c', 'h_drive_setpoints', 'coaxial.drive', 'Drive', 'setpoints'),
    ('comms/src/cmd_drive.c', 'h_drive_window', 'coaxial.drive', 'Drive', 'window'),
    ('comms/src/cmd_drive.c', 'h_drive_moments', 'coaxial.drive', 'Drive', 'moments'),
    ('comms/src/cmd_drive.c', 'h_drive_model', 'coaxial.drive', 'Drive', 'model'),
    ('comms/src/cmd_drive.c', 'h_drive_observers', 'coaxial.drive', 'Drive', 'observers'),
    ('comms/src/cmd_log.c', 'h_log_state', 'coaxial.capture', 'Capture', 'state'),
    ('comms/src/cmd_power.c', 'h_power_state', 'coaxial.power', 'Power', 'state'),
    ('comms/src/cmd_thermal.c', 'h_thermal_state', 'coaxial.thermal_device', 'Thermal', 'state'),
    ('comms/src/cmd_thermal.c', 'h_thermal_budget', 'coaxial.thermal_device', 'Thermal', 'budget'),
    ('comms/src/cmd_thermal.c', 'h_thermal_edges', 'coaxial.thermal_device', 'Thermal', 'network'),
    ('comms/src/cmd_time.c', 'h_time_read', 'coaxial.clock', 'Clock', 'read_latch'),
)

#: What each Reader method takes off the wire; the scaled ones take a width
#: as their first argument when it is not the default.
_READS = {'u8': 'u8', 'i8': 'i8', 'u16': 'u16', 'i16': 'i16', 'u32': 'u32',
          'i32': 'i32', 'q16': 'u32', 'flags': 'u8', 'fraction': 'u8',
          'centi': 'i32', 'milli': 'i32', 'micro': 'i32', 'nano': 'u32'}
_WIDTH_ARG = ('centi', 'milli', 'micro', 'nano')
_C_TOKENS = re.compile(
    r'(?P<open>\{)|(?P<close>\})'
    r'|(?P<leave>\b(?:return|continue)\b[^;]*;)'
    r'|for\s*\([^;]*;[^<]*<=?\s*(?:\([^)]*\)\s*)?(?P<bound>\w+)[^)]*\)'
    r'|\b(?P<callee>\w+)\s*\(\s*out\b')
#: The command files whose helpers any handler may call - `wr_took` in
#: the wire writes the byte every acknowledging op answers with.
_SHARED = ('comms/src/cmd_device.c', 'comms/src/cmd.c', 'comms/src/wire.c')

#: Every file that dispatches ops the way a command file does: `case
#: PREFIX_OP_NAME: return handler(in, out)`. The bootloader's core is one
#: outside comms/, served by a node in its bootloader.
def _command_files():
    folder = os.path.join(REPO, 'comms', 'src')
    files = ['comms/src/' + name for name in sorted(os.listdir(folder))
             if name.startswith('cmd_') and name != 'cmd_length.c']
    return files + ['boot/src/boot_core.c']
_HEADERS = ('comms/inc', 'board/inc', 'drive/inc', 'thermal/inc', 'daq/inc', 'boot/inc',
            'filter/inc', 'shtp/inc', 'modbus/inc')


def _c_defines(extra_text=''):
    """Every `#define NAME <int>` in the tree's headers, and in the text
    given: what a loop bound may be."""
    found = {}
    texts = [extra_text]
    for rel in _HEADERS:
        folder = os.path.join(REPO, *rel.split('/'))
        for name in sorted(os.listdir(folder)) if os.path.isdir(folder) else ():
            if name.endswith('.h'):
                texts.append(io.open(os.path.join(folder, name), encoding='utf-8').read())
    for text in texts:
        for m in re.finditer(r'#define\s+(\w+)\s+\(?(\d+)U?\)?\s*(?:/|$)', text, re.M):
            found[m.group(1)] = int(m.group(2))
    return found


def _c_function(text, name):
    """The braces of `name` in `text`, or None when it is not defined there."""
    m = re.search(r'^(?:static\s+)?\w+\s+%s\s*\(\s*wr_t\s*\*\s*out' % re.escape(name), text, re.M)
    return None if m is None else text[text.index('{', m.end()):]


def _c_writes_in(text, body, defines, depth_limit=4):
    """The widths a body writes, in order."""
    stack = [[[], None, False]]      # [writes, repeat-or-'*'-or-None, ends-in-leave]
    depth, bound = 0, None
    for m in _C_TOKENS.finditer(body):
        if m.group('open'):
            depth += 1
            if bound is not None:
                repeat = (int(bound.rstrip('U')) if bound.rstrip('U').isdigit()
                          else defines.get(bound, '*'))
                stack.append([[], repeat, False])
                bound = None
            elif depth > 1:
                stack.append([[], None, False])
            continue
        if m.group('close'):
            depth -= 1
            if depth == 0:
                break
            writes, repeat, left = stack.pop()
            if repeat == '*':
                if writes:
                    stack[-1][0].append(('*', tuple(writes)))
            elif repeat is not None:
                stack[-1][0].extend(writes * repeat)
            elif not left:
                stack[-1][0].extend(writes)
            stack[-1][2] = False
            continue
        if m.group('leave'):
            if depth > 1 and stack[-1][0] and re.search(r'\breturn\b(?![^;]*CMD_ERR)', m.group('leave')):
                return [w for level in stack for w in level[0]]
            stack[-1][2] = True
            continue
        stack[-1][2] = False
        if m.group('bound'):
            bound = m.group('bound')
            continue
        callee = m.group('callee')
        w = re.fullmatch(r'wr_(u8|i8|u16|i16|u32|i32|str|bytes)', callee)
        if w and w.group(1) in ('str', 'bytes'):
            stack[-1][0].append(('rest',))
        elif w:
            stack[-1][0].append(w.group(1))
        elif depth_limit and _c_function(text, callee) is not None:
            stack[-1][0].extend(_c_writes_in(text, _c_function(text, callee), defines,
                                             depth_limit - 1))
    return stack[0][0]


def _c_source(rel):
    """One command file's text with the shared files' after it, comments
    stripped, so a helper is found whichever file defines it."""
    texts = []
    for each in (rel,) + _SHARED:
        path = os.path.join(REPO, *each.split('/'))
        if os.path.exists(path):
            texts.append(io.open(path, encoding='utf-8').read())
    return re.sub(r'/\*.*?\*/', ' ', '\n'.join(texts), flags=re.S)


def _c_writes(rel, name):
    text = _c_source(rel)
    body = _c_function(text, name)
    assert body is not None, '%s has no %s taking out first' % (rel, name)
    return _c_writes_in(text, body, _c_defines(text))


def _py_reads(module, cls, method):
    """The widths a decoder reads, in order, the same way: a loop over a
    module's tuple or a constant is its body that many times, a loop over
    a count read off the wire is `('*', body)`, a helper handed the
    reader is followed.
    """
    mod = importlib.import_module(module)
    owner = getattr(mod, cls)

    def count(node):
        """A loop's repeat when a constant or the module knows it, else None."""
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                and node.func.id == 'range'):
            arg = node.args[-1]
            if isinstance(arg, ast.Constant):
                return arg.value
            if isinstance(arg, ast.Name) and hasattr(mod, arg.id):
                return getattr(mod, arg.id)
            return None
        if isinstance(node, (ast.Tuple, ast.List)):
            return len(node.elts)
        if isinstance(node, ast.Name) and hasattr(mod, node.id):
            return len(getattr(mod, node.id))
        return None

    def helper(node):
        """The function a call hands the reader to, or None."""
        if isinstance(node.func, ast.Name):
            return getattr(mod, node.func.id, None)
        if (isinstance(node.func, ast.Attribute) and isinstance(node.func.value, ast.Name)
                and node.func.value.id == 'self'):
            return getattr(owner, node.func.attr, None)
        return None

    def function_reads(fn, reader_at, out, depth):
        tree = ast.parse(textwrap.dedent(inspect.getsource(fn)))
        params = [a.arg for a in tree.body[0].args.args]
        if params and params[0] == 'self':
            params = params[1:]
        visit(tree.body[0], out, params[reader_at], depth + 1)

    def visit(node, out, reader, depth=0):
        if isinstance(node, (ast.For, ast.GeneratorExp, ast.ListComp, ast.DictComp)):
            source_ = node.iter if isinstance(node, ast.For) else node.generators[0].iter
            body = node.body if isinstance(node, ast.For) else (
                [node.key, node.value] if isinstance(node, ast.DictComp) else [node.elt])
            inner = []
            for child in body:
                visit(child, inner, reader, depth)
            heads = []
            visit(source_, heads, reader, depth)
            out.extend(heads)
            repeat = None if heads else count(source_)
            if inner:
                out.extend(inner * repeat if repeat else [('*', tuple(inner))])
            return
        if isinstance(node, ast.Call):
            if (isinstance(node.func, ast.Attribute) and isinstance(node.func.value, ast.Name)
                    and node.func.value.id == reader):
                name = node.func.attr
                if name == 'maybe':
                    out.append(node.args[0].value)
                elif name in _READS:
                    width = _READS[name]
                    if name in _WIDTH_ARG and node.args and isinstance(node.args[0], ast.Constant):
                        width = node.args[0].value
                    out.append(width)
                else:
                    raise ValueError('a read this check cannot size: %s.%s' % (reader, name))
            handed = [k for k, a in enumerate(node.args)
                      if isinstance(a, ast.Name) and a.id == reader]
            fn = helper(node) if handed else None
            if fn is not None and depth < 4:
                function_reads(fn, handed[0], out, depth)
                return
        for child in ast.iter_child_nodes(node):
            visit(child, out, reader, depth)

    out = []
    tree = ast.parse(textwrap.dedent(inspect.getsource(getattr(owner, method))))
    visit(tree, out, 'r')
    return out


def _shapes_agree(wrote, read, i=0, j=0):
    """Whether the reader's sequence is the writer's."""
    if i == len(wrote) and j == len(read):
        return True, i
    if i >= len(wrote) or j >= len(read):
        rest = wrote[i:] if i < len(wrote) else read[j:]
        return all(isinstance(x, tuple) and x[0] in ('?', 'rest') for x in rest), i
    a, b = wrote[i], read[j]
    if (isinstance(a, tuple) and a[0] == 'rest') or (isinstance(b, tuple) and b[0] == 'rest'):
        return True, len(wrote)
    if isinstance(a, tuple) and a[0] == '?':
        if isinstance(b, tuple) and b[0] == '?' and a[1] == b[1]:
            return _shapes_agree(wrote, read, i + 1, j + 1)
        taken = _shapes_agree(wrote, read, i + 1, j + 1) if b == a[1] else (False, i)
        return taken if taken[0] else _shapes_agree(wrote, read, i + 1, j)
    if isinstance(b, tuple) and b[0] == '?':
        taken = _shapes_agree(wrote, read, i + 1, j + 1) if a == b[1] else (False, i)
        return taken if taken[0] else _shapes_agree(wrote, read, i, j + 1)
    if isinstance(a, tuple) and isinstance(b, tuple):
        return _shapes_agree(wrote, read, i + 1, j + 1) if a[1] == b[1] else (False, i)
    if isinstance(a, tuple) or isinstance(b, tuple):
        # the side with the repeat, and the side laying the run out
        laid, body = ((read, list(a[1])) if isinstance(a, tuple)
                      else (wrote, list(b[1])))
        at = j if isinstance(a, tuple) else i
        n = 0
        while body and laid[at + n * len(body):at + (n + 1) * len(body)] == body:
            n += 1
        furthest = i
        for k in range(n, 0, -1):
            ni, nj = ((i + 1, j + k * len(body)) if isinstance(a, tuple)
                      else (i + k * len(body), j + 1))
            ok, reached = _shapes_agree(wrote, read, ni, nj)
            if ok:
                return True, reached
            furthest = max(furthest, reached)
        return False, furthest
    if a != b:
        return False, i
    return _shapes_agree(wrote, read, i + 1, j + 1)


def test_wire_shapes_agree(r):
    """A reply's shape is one sequence of widths, written in C and read in
    Python, and the two are held to each other field by field.
    """
    import inspect
    import textwrap
    globals()['inspect'], globals()['textwrap'] = inspect, textwrap
    for rel, func, module, cls, method in WIRE_SHAPES:
        wrote = _c_writes(rel, func)
        read = _py_reads(module, cls, method)
        same, at = _shapes_agree(wrote, read)
        where = ('%d fields both ways' % len(wrote) if same else
                 'from field %d: C %s, Python %s' % (
                     at, ' '.join(str(w) for w in wrote[at:at + 4]) or 'nothing',
                     ' '.join(str(w) for w in read[at:at + 4]) or 'nothing'))
        r.check('%s writes what %s.%s reads, field for field' % (func, cls, method),
                same, where)


#: The requests' side of the same check. Every `case PREFIX_OP_NAME: return
#: handler(in, out);` names the handler that reads an op's request, and
#: every `self._op(XOp.NAME, pack(...))` names what the decoder sends; the
#: enums are already held to cmd.h by name, so the two pair up by the op's
#: name and the reads are held to the widths.
_RD_TOKENS = re.compile(
    r'(?P<open>\{)|(?P<close>\})|(?P<semi>;)'
    r'|(?P<leave>\b(?:return|continue)\b[^;]*;)'
    r'|(?P<guard>\brd_left\s*\(\s*in\s*\)|\?)'
    r'|for\s*\([^;]*;[^<]*<=?\s*(?:\([^)]*\)\s*)?(?P<bound>\w+)[^)]*\)'
    r'|\brd_(?P<width>u8|i8|u16|i16|u32|i32|bytes)\s*\(\s*in\b')
_DISPATCH = re.compile(r'case\s+(\w+?)_OP_(\w+)\s*:\s*return\s+(\w+)\s*\(\s*(in|out)')


def _c_reads(text, name, defines):
    """The widths a handler reads off its request, in order: a field read
    under an `rd_left` guard - a ternary, or a block the guard opens - is
    `('?', width)`, taken when it is there; a loop over a count is its
    body that many times or `('*', body)`; `rd_bytes` is `('rest',)`,
    whatever is left.
    """
    m = re.search(r'^static\s+\w+\s+%s\s*\(\s*rd_t\s*\*\s*in' % re.escape(name), text, re.M)
    if m is None:
        return []
    body = text[text.index('{', m.end()):]
    stack = [[[], None, False, False]]   # [reads, repeat, ends-in-leave, optional]
    depth, bound, armed = 0, None, False
    for t in _RD_TOKENS.finditer(body):
        if t.group('open'):
            depth += 1
            optional = armed or stack[-1][3]
            if bound is not None:
                repeat = (int(bound.rstrip('U')) if bound.rstrip('U').isdigit()
                          else defines.get(bound, '*'))
                stack.append([[], repeat, False, optional])
                bound = None
            elif depth > 1:
                stack.append([[], None, False, optional])
            armed = False
            continue
        if t.group('close'):
            depth -= 1
            if depth == 0:
                break
            reads, repeat, left, _ = stack.pop()
            if repeat == '*':
                if reads:
                    stack[-1][0].append(('*', tuple(reads)))
            elif repeat is not None:
                stack[-1][0].extend(reads * repeat)
            elif not left:
                stack[-1][0].extend(reads)
            stack[-1][2] = False
            continue
        if t.group('semi'):
            armed = False
            continue
        if t.group('leave'):
            stack[-1][2] = True
            armed = False
            continue
        stack[-1][2] = False
        if t.group('guard'):
            armed = True
            continue
        if t.group('bound'):
            bound = t.group('bound')
            continue
        width = t.group('width')
        if width == 'bytes':
            stack[-1][0].append(('rest',))
        elif armed or stack[-1][3]:
            stack[-1][0].append(('?', width))
        else:
            stack[-1][0].append(width)
    return stack[0][0]


def _c_requests():
    """{(PREFIX, OP): reads} for every op a command file dispatches."""
    found = {}
    for rel in _command_files():
        text = io.open(os.path.join(REPO, *rel.split('/')), encoding='utf-8').read()
        text = re.sub(r'/\*.*?\*/', ' ', text, flags=re.S)
        defines = _c_defines(text)
        for prefix, op, handler, first in _DISPATCH.findall(text):
            found[(prefix, op)] = _c_reads(text, handler, defines) if first == 'in' else []
    return found


def _py_requests():
    """{(EnumName, OP): widths} for every request the library sends whose
    payload is a literal `pack(...)`, nothing, or a name bound to one in
    the same function; None where it is built some other way."""
    found = {}
    folder = os.path.join(HOST, 'coaxial')

    def widths_of(node, scope):
        if node is None:
            return []
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                and node.func.id == 'pack'):
            out = []
            for arg in node.args:
                if not (isinstance(arg, ast.Tuple) and arg.elts
                        and isinstance(arg.elts[0], ast.Constant)):
                    return None
                out.append(arg.elts[0].value)
            return out
        if isinstance(node, ast.Name) and node.id in scope:
            return widths_of(scope[node.id], {})
        return None

    for name in sorted(os.listdir(folder)):
        if not name.endswith('.py'):
            continue
        tree = ast.parse(io.open(os.path.join(folder, name), encoding='utf-8').read())
        for fn in [n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)]:
            scope = {t.id: s.value for s in fn.body if isinstance(s, ast.Assign)
                     for t in s.targets if isinstance(t, ast.Name)}
            for call in ast.walk(fn):
                if not (isinstance(call, ast.Call) and isinstance(call.func, ast.Attribute)
                        and call.func.attr in ('_op', '_ack') and call.args):
                    continue
                op = call.args[0]
                if not (isinstance(op, ast.Attribute) and isinstance(op.value, ast.Name)
                        and op.value.id.endswith('Op')):
                    continue
                payload = call.args[1] if len(call.args) > 1 else None
                key = (op.value.id, op.attr)
                widths = widths_of(payload, scope)
                if key not in found or found[key] is None:
                    found[key] = widths
    return found


def _request_agrees(reads, widths):
    """Whether packed `widths` are what the handler reads: an optional field
    taken when it is there, the rest whatever is left, a repeat once or
    more."""
    def go(i, j):
        if i == len(reads):
            return j == len(widths)
        a = reads[i]
        if isinstance(a, tuple) and a[0] == 'rest':
            return True
        if isinstance(a, tuple) and a[0] == '?':
            return (j < len(widths) and widths[j] == a[1] and go(i + 1, j + 1)) or go(i + 1, j)
        if isinstance(a, tuple) and a[0] == '*':
            body = list(a[1])
            n = 0
            while body and widths[j + n * len(body):j + (n + 1) * len(body)] == body:
                n += 1
            return any(go(i + 1, j + k * len(body)) for k in range(n, 0, -1))
        return j < len(widths) and widths[j] == a and go(i + 1, j + 1)
    return go(0, 0)


def test_wire_requests_agree(r):
    """What the library packs into a request is what the handler reads."""
    enum_of = {prefix: cls for prefix, cls in OP_CLASSES.items()}
    reads = _c_requests()
    packs = _py_requests()
    for prefix in sorted({p for p, _ in reads}):
        cls = enum_of.get(prefix)
        wrong, compared, skipped = [], 0, []
        for (p, op), sequence in sorted(reads.items()):
            if p != prefix:
                continue
            widths = packs.get((cls, op), 'absent')
            if widths == 'absent':
                continue                      # no request from the library
            if widths is None:
                skipped.append(op)
                continue
            compared += 1
            if not _request_agrees(sequence, widths):
                wrong.append('%s: C reads %s, Python packs %s' % (
                    op, ' '.join(str(w) for w in sequence) or 'nothing',
                    ' '.join(widths) or 'nothing'))
        r.check("%s's requests are packed as its handlers read them" % prefix,
                not wrong, '; '.join(wrong[:3]) or '%d ops%s' % (
                    compared, (', not literal: ' + ', '.join(skipped)) if skipped else ''))


#: PROTOCOL.md's third answer to a reply's shape, held to the code. A row
#: `| N name | request | reply |` under `### N DEVICE` carries the widths in
#: backticks - `u8 on`, `u16 x3 ticks [, u32 periods]`, `i32 x5 x count`,
#: `u8 channel, bytes` - and a reply of `below` is the `Op N:` paragraph
#: after the table. Prose is prose; only the backticked spans are read, a
#: span after "per", "rows" or "of" is a body repeated an unknown number of
#: times, and a cell whose prose carries widths outside backticks is skipped
#: and named rather than half-read.
_DOC_SECTION = re.compile(r'^### (\d+) ([A-Z_]+),', re.M)
_DOC_ROW = re.compile(r'^\| (\d+) [a-z_ ]+ \| (.*?) \| (.*?) \|\s*$', re.M)
_DOC_SPAN = re.compile(r'`([^`]*)`')
_DOC_WIDTH = re.compile(r'\b([ui](?:8|16|32)|bytes|str)\b')
_DOC_REPEATED = re.compile(r'(?:\bper(?: \w+){1,2}|\brows|\bof)\s*$')


def _doc_widths(cell):
    """The widths a document cell states, or None when its prose carries
    widths the backticks do not."""
    outside = _DOC_SPAN.sub(' ', cell)
    if _DOC_WIDTH.search(outside) or 'below' in cell:
        return None
    out = []
    for m in _DOC_SPAN.finditer(cell):
        before = cell[max(0, m.start() - 24):m.start()]
        body = []
        optional = False
        for item in m.group(1).split(','):
            w = _DOC_WIDTH.search(item)
            if w is None:
                optional = optional or '[' in item
                continue
            # a bracket before the width opens the optional part here; one
            # after it opens it for the fields that follow
            opened = item.find('[')
            optional = optional or (0 <= opened < w.start())
            later = opened > w.start()
            item = item.replace('[', ' ').replace(']', ' ')
            width = w.group(1)
            if width in ('bytes', 'str'):
                body.append(('rest',))
                continue
            times = re.search(r'\bx(\d+)\b', item)
            counted = re.search(r'\bx count\b|\bx n\b', item)
            fields = [('?', width) if optional else width] * (int(times.group(1)) if times else 1)
            if counted:
                body.append(('*', tuple(fields)))
            else:
                body.extend(fields)
            optional = optional or later
        if _DOC_REPEATED.search(before) and body:
            out.append(('*', tuple(body)))
        else:
            out.extend(body)
    return out


def _doc_ops():
    """{(PREFIX, op number): (request widths, reply widths)} off PROTOCOL.md,
    None for a cell this reader cannot take whole."""
    text = io.open(os.path.join(REPO, 'docs', 'PROTOCOL.md'), encoding='utf-8').read()
    found = {}
    sections = list(_DOC_SECTION.finditer(text))
    for k, sec in enumerate(sections):
        end = sections[k + 1].start() if k + 1 < len(sections) else len(text)
        chunk = text[sec.start():end]
        prefix = sec.group(2).replace('_', '')
        paragraphs = {int(m.group(1)): m.group(2) for m in re.finditer(
            r'^Op (\d+): (.*?)(?:\n\n|\Z)', chunk, re.M | re.S)}
        for m in _DOC_ROW.finditer(chunk):
            op = int(m.group(1))
            request = [] if m.group(2).strip() == '-' else _doc_widths(m.group(2))
            reply_cell = m.group(3)
            if 'below' in reply_cell:
                reply = _doc_widths(paragraphs[op]) if op in paragraphs else None
            else:
                reply = [] if reply_cell.strip() == '-' else _doc_widths(reply_cell)
            found[(prefix, op)] = (request, reply)
    return found


def _c_ops():
    """{(PREFIX, op number): [(reads, writes), ...]} off the dispatch
    tables.
    """
    found = {}
    for rel in _command_files():
        own = re.sub(r'/\*.*?\*/', ' ',
                     io.open(os.path.join(REPO, *rel.split('/')), encoding='utf-8').read(), flags=re.S)
        text = _c_source(rel)
        defines = _c_defines(text)
        for prefix, op, handler, first in _DISPATCH.findall(own):
            number = defines.get('%s_OP_%s' % (prefix, op))
            if number is None:
                continue
            reads = _c_reads(text, handler, defines) if first == 'in' else []
            body = _c_function(text, handler)
            writes = _c_writes_in(text, body, defines) if body is not None else []
            if body is None:
                # a handler reading and writing: the writes parser wants `out`
                # first, so read its writes off the `in, out` form
                m = re.search(r'^static\s+\w+\s+%s\s*\(\s*rd_t\s*\*\s*in\s*,\s*wr_t\s*\*\s*out'
                              % re.escape(handler), text, re.M)
                if m is not None:
                    writes = _c_writes_in(text, text[text.index('{', m.end()):], defines)
            found.setdefault((prefix, number), []).append((reads, writes))
    return found


def _same_shape(code, doc):
    """A document's widths against the code's, either side repeating."""
    return _shapes_agree(list(code), list(doc))[0]


def test_protocol_agrees(r):
    """PROTOCOL.md's op tables say what the handlers read and write."""
    doc = _doc_ops()
    code = _c_ops()
    for prefix in sorted({p for p, _ in doc if any(q == p for q, _ in code)}):
        wrong, compared, skipped = [], 0, []
        for (p, op), (request, reply) in sorted(doc.items()):
            if p != prefix or (p, op) not in code:
                continue
            for reads, writes in code[(p, op)]:
                for side, stated, actual in (('request', request, reads), ('reply', reply, writes)):
                    if stated is None:
                        skipped.append('%d %s' % (op, side))
                        continue
                    compared += 1
                    if not _same_shape(actual, stated):
                        wrong.append('op %d %s: C %s, document %s' % (
                            op, side, ' '.join(str(w) for w in actual) or 'nothing',
                            ' '.join(str(w) for w in stated) or 'nothing'))
        r.check("PROTOCOL.md's %s table says what the handlers read and write" % prefix,
                not wrong, '; '.join(wrong[:3]) or '%d cells%s' % (
                    compared, (', prose: ' + ', '.join(skipped)) if skipped else ''))


#: The notebooks' shape, as `tools/notebooks/parts.paper` lays it out:
#: what a reader finds in every one, in this order. Held here so a cell
#: edited by hand, or a module that stops using the builder, fails a run
#: later rather than reading differently from the other eight.
_PAPER_KEYS = ('# ', '*', '**Abstract.** ', '## 1 Setup', 'SIMULATED = ',
               'Coaxial63100(', 'device.close()', 'Conclusions',
               '**At the bench.** ', '## References')
_FORBIDDEN = (('plt.subplots(', 'a figure outside coaxial.figures'),
              ('figsize=', 'a figure size of its own'),
              ('!', 'an exclamation mark'))


def _paper(path):
    """The cells of one notebook, and every way it departs from the shape."""
    import json
    with io.open(path, encoding='utf-8') as handle:
        cells = json.load(handle)['cells']
    text = lambda c: ''.join(c['source'])
    kinds = [c['cell_type'] for c in cells]
    said = [text(c) for c in cells]
    wrong = []
    head = said[0].splitlines() if said else []
    if kinds[:2] != ['markdown', 'markdown'] or not head or not head[0].startswith('# '):
        wrong.append('no title cell')
    elif len(head) < 5 or not head[2].startswith('*') or not head[4].startswith('**Abstract.** '):
        wrong.append('the title cell is not title, subtitle, abstract')
    if len(cells) < 4 or said[1] != '## 1 Setup' or 'SIMULATED = ' not in said[2] \
            or 'Coaxial63100(' not in said[3]:
        wrong.append('Setup is not the knob and the open cell')
    headings = [s for k, s in zip(kinds, said) if k == 'markdown' and s.startswith('## ')]
    numbers = [h.split()[1] for h in headings if h[3].isdigit()]
    if numbers != [str(n) for n in range(1, len(numbers) + 1)]:
        wrong.append('sections not numbered 1.. in order: %s' % numbers)
    if not headings or not headings[-1].startswith('## References'):
        wrong.append('References is not the last section')
    if len(headings) < 2 or not headings[-2].endswith(' Conclusions'):
        wrong.append('Conclusions is not the section before References')
    if not any(s.startswith('**At the bench.** ') for s in said):
        wrong.append('no At the bench paragraph')
    if not any(k == 'code' and 'device.close()' in s for k, s in zip(kinds, said)):
        wrong.append('the device is never closed')
    for k, c in zip(kinds, cells):
        if k != 'code':
            continue
        if not c.get('outputs') and 'print(' in text(c):
            wrong.append('a cell that prints has no output - not executed')
            break
        if any(o.get('output_type') == 'error' for o in c['outputs']):
            wrong.append('a cell raised: %s' % next(
                o['ename'] for o in c['outputs'] if o.get('output_type') == 'error'))
            break
    code = '\n'.join(s for k, s in zip(kinds, said) if k == 'code')
    prose = '\n'.join(s for k, s in zip(kinds, said) if k == 'markdown')
    wrong += ['%s in the code' % why for what, why in _FORBIDDEN[:2] if what in code]
    wrong += [why for what, why in _FORBIDDEN[2:] if what in prose]
    return wrong


def test_notebooks_are_papers(r):
    """Every example notebook is one paper in the builder's shape, executed."""
    folder = os.path.join(REPO, 'notebook_examples')
    names = sorted(n for n in os.listdir(folder) if n.endswith('.ipynb'))
    for name in names:
        wrong = _paper(os.path.join(folder, name))
        r.check('%s is a paper in the builder\'s shape, executed' % name,
                not wrong, '; '.join(wrong[:3]))
    r.check('and there is one notebook per functional area',
            names == ['%s.ipynb' % a for a in sorted(
                ('acquisition', 'link', 'sensors', 'power_stage', 'thermal',
                 'drive', 'motion', 'applications', 'commissioning'))],
            ', '.join(names))


ROSTER = (test_imports, test_no_undefined_names, test_no_cycles,
          test_reexports,
          test_no_duplicate_definitions, test_no_unused_imports,
          test_numpy_enters_behind_the_thread_cap,
          test_shape, test_documented, test_no_escaping_scars,
          test_counts_are_measured, test_subsystem_calls_resolve,
          test_limits_live_in_one_file, test_mirrors_agree,
          test_wire_shapes_agree, test_wire_requests_agree,
          test_protocol_agrees, test_notebooks_are_papers)


def main():
    report = Report()
    for test in ROSTER:
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
