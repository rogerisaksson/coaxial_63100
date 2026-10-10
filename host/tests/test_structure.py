"""Does host/ still hold together? Its imports, cycles, names, shape and docstrings, the token budget."""
import ast
import builtins
import importlib
import os
import sys

from tools.dev.focus import chosen
from structure_kit import HOST, PACKAGES, Report, depth, sources


# The ceiling is the worst that survives a deliberate reading, not an ideal.
MAX_LINES = 130


MAX_DEPTH = 7


# Character-by-character state machines, where the nesting *is* the machine and
# flattening it would cost clarity rather than buy any.
DEEP_BY_NATURE = {'json_objects', '_gpus_registry'}


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


#: Every device on the board, and its stand-in: the stand-in answers every
#: public call the device does (docs: add a method to both or neither).
TWINS = (
    ('coaxial.devices.afe:Afe', 'coaxial.simulated.analog:SimulatedAfe'),
    ('coaxial.devices.analog:Analog', 'coaxial.simulated.analog:SimulatedAnalog'),
    ('coaxial.devices.gpio:Gpio', 'coaxial.simulated.system:SimulatedGpio'),
    ('coaxial.devices.power:Power', 'coaxial.simulated.power:SimulatedPower'),
    ('coaxial.devices.gate_drivers:GateDrivers', 'coaxial.simulated.power:SimulatedGateDrivers'),
    ('coaxial.devices.imu:Imu', 'coaxial.simulated.sensors:SimulatedImu'),
    ('coaxial.devices.angle:Angle', 'coaxial.simulated.sensors:SimulatedAngle'),
    ('coaxial.acquire.capture:Capture', 'coaxial.simulated.acquire.capture:SimulatedCapture'),
    ('coaxial.acquire.clock:Clock', 'coaxial.simulated.acquire.clock:SimulatedClock'),
    ('coaxial.acquire.daq:Daq', 'coaxial.simulated.acquire.daq:SimulatedDaq'),
    ('coaxial.devices.drive:Drive', 'coaxial.simulated.drive.device:SimulatedDrive'),
    ('coaxial.devices.thermal:Thermal',
     'coaxial.simulated.thermal.observer:SimulatedThermal'),
    ('coaxial.devices.calibration:Calibration', 'coaxial.simulated.analog:SimulatedCalibration'),
    ('coaxial.devices.system:System', 'coaxial.simulated.system:SimulatedSystem'),
    ('coaxial.devices.link:Link', 'coaxial.simulated.link:SimulatedLink'),
    ('coaxial.devices.boot:Boot', 'coaxial.simulated.boot:SimulatedBoot'),
    ('coaxial.devices.ctrl:Ctrl', 'coaxial.simulated.ctrl:SimulatedCtrl'),
)


#: The wire's own plumbing, which a stand-in has no wire to carry.
WIRE_ONLY = {'board', 'request', 'took'}


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


def test_every_file_fits_a_reading(r):
    """No file past the token budget a model reads a file whole by, and none of those still
    over it grown past its cap (tools/dev/token_budget.py)."""
    from tools.dev import token_budget
    past = token_budget.breaches()
    r.check('every file within %.0f k tokens, or its cap while it is split'
            % (token_budget.BUDGET / 1e3), not past,
            ', '.join('%s %.1f k of %.1f' % (p, t / 1e3, cap / 1e3) for p, t, cap in past))


def test_imports(r):
    """Every module imports on its own, from a cold interpreter."""
    for name in modules():
        try:
            importlib.import_module(name)
            r.check('%s imports' % name, True)
        except Exception as exc:    # a module body can raise anything: each is a FAIL here
            r.check('%s imports' % name, False,
                    '%s: %s' % (type(exc).__name__, exc))


def test_no_undefined_names(r):
    """A name used that nothing in the module defines or imports."""
    for path, _, tree in sources():
        known = _bound(tree)
        used = {n.id for n in ast.walk(tree)
                if isinstance(n, ast.Name) and isinstance(n.ctx, ast.Load)}
        loose = sorted(used - known)
        r.check('%s uses only names it has' % path,
                not loose, ', '.join(loose[:6]))


def test_no_cycles(r):
    """No package module imports another that imports it back."""
    def is_module(dotted):
        base = os.path.join(HOST, *dotted.split('.'))
        return os.path.isfile(base + '.py') or os.path.isfile(
            os.path.join(base, '__init__.py'))

    edges = {}
    for path, _, tree in sources():
        if not path.startswith(PACKAGES):
            continue
        me = path.replace('\\', '/')[:-3].replace('/', '.').removesuffix('.__init__')
        here = me if path.endswith('__init__.py') else me.rpartition('.')[0]
        edges[me] = set()
        for node in tree.body:                # top-level imports only
            if isinstance(node, ast.Import):
                edges[me].update(a.name for a in node.names)
            elif isinstance(node, ast.ImportFrom):
                base = here.rsplit('.', node.level - 1)[0] if node.level else ''
                module = '.'.join(filter(None, (base, node.module)))
                for alias in node.names:
                    sub = '%s.%s' % (module, alias.name)
                    edges[me].add(sub if is_module(sub) else module)
    back = [(a, b) for a, seen in edges.items() for b in seen
            if a in edges.get(b, ())]
    r.check('no module imports one that imports it back at the top',
            not back, repr(back))


def test_reexports(r):
    """Every name debug.py says lives elsewhere does."""
    from coaxial_ollama import debug
    for name, where in sorted(debug._ELSEWHERE.items()):
        try:
            got = getattr(debug, name)
            module = importlib.import_module('coaxial_ollama.' + where)
            r.check('debug.%s comes from %s' % (name, where),
                    getattr(module, name, None) is got)
        except Exception as exc:    # a module body can raise anything: each is a FAIL here
            r.check('debug.%s comes from %s' % (name, where), False, str(exc))
    try:
        debug.no_such_name_at_all
        r.check('and an unknown name still raises AttributeError', False)
    except AttributeError:
        r.check('and an unknown name still raises AttributeError', True)


def test_stand_ins_answer_every_call(r):
    """A device and its stand-in: the same public calls."""
    def load(spec):
        module, _, name = spec.partition(':')
        return getattr(importlib.import_module(module), name)

    def public(cls):
        return {n for n in dir(cls) if not n.startswith('_') and callable(getattr(cls, n, None))}

    for real, stand_in in TWINS:
        missing = sorted(public(load(real)) - public(load(stand_in)) - WIRE_ONLY)
        r.check('the stand-in answers every call %s does' % real.rpartition(':')[2],
                not missing, ', '.join(missing))


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
             and len({p.split(os.sep)[0] for p in where}) == 1}
    r.check('no definition is copied into two files of one package',
            not twice, '; '.join('%s in %s' % (n, ', '.join(w))
                                 for n, w in sorted(twice.items())[:4]))


def test_no_unused_imports(r):
    """An import nothing references. Left behind by every move."""
    for path, _, tree in sources():
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
        dead = [n for n in brought if n not in used]
        r.check('%s imports nothing it does not use' % path,
                not dead, ', '.join(dead))


def test_numpy_enters_behind_the_thread_cap(r):
    """numpy is imported at module level by the modules that step arrays - blocks.py,
    machine/capture.py, machine/cyclic.py and the whole-body law's qp, wbc, mpc, balance - each
    capping OpenBLAS's thread pool before importing it.
    """
    importers, uncapped = [], []
    for path, _text, tree in sources(beside=False):
        if not path.startswith(PACKAGES):
            continue
        for i, node in enumerate(tree.body):
            if 'numpy' in _names_imported(node):
                importers.append(path)
                if not any(_caps_openblas(n) for n in tree.body[:i]):
                    uncapped.append(path)
                break
    r.check('numpy enters the packages at module level in blocks.py, capture.py, cyclic.py and '
            'the whole-body law alone',
            sorted(importers) == sorted([os.path.join('coaxial', 'model', 'blocks.py')]
                                        + [os.path.join('machine', m + '.py') for m in (
                                            'capture', 'cyclic', 'qp', 'wbc', 'mpc', 'balance')]),
            ', '.join(importers) or 'nowhere')
    r.check('each sets OPENBLAS_NUM_THREADS before importing it', not uncapped,
            ', '.join(uncapped))


def test_machine_imports_no_board(r):
    """machine/ names a board family only as a string in FAMILIES: no import of one,
    anywhere in a module, so the family can leave for its own repository."""
    importers = sorted({path for path, _text, tree in sources(beside=False)
                        if path.startswith('machine' + os.sep)
                        for node in ast.walk(tree)
                        if any(n.startswith('coaxial') for n in _names_imported(node))})
    r.check('machine imports no board family', not importers, ', '.join(importers))


def test_motor_imports_no_board(r):
    """motor/ is a PMSM and its loads: no board family, no machine."""
    importers = sorted({path for path, _text, tree in sources(beside=False)
                        if path.startswith('motor' + os.sep)
                        for node in ast.walk(tree)
                        if any(n.startswith(('coaxial', 'machine'))
                               for n in _names_imported(node))})
    r.check('motor imports no board family and no machine', not importers,
            ', '.join(importers))


def test_shape(r):
    """No function past the length or nesting a reader can hold."""
    long_ones, deep_ones = [], []
    for path, _, tree in sources():
        for node in ast.walk(tree):
            # AsyncFunctionDef too: it is not a subclass of FunctionDef, and
            # coaxial_mcp/server.py has three async handlers.
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
    from tools.dev.host_map import brief
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
    r.check('every module opens on a one-line brief (tools/dev/host_map.py)',
            not long_, '; '.join(long_[:4]))


def test_no_escaping_scars(r):
    """chr(10) and chr(92) where a literal belongs."""
    for path, text, tree in sources():
        if path.endswith(('replies.py', 'sandbox.py')):
            continue        # both are about escaping
        scars = [w for w in ('chr(10)', 'chr(92)') if w in text]
        r.check('%s has no heredoc scars' % path, not scars, ', '.join(scars))


ROSTER = (test_every_file_fits_a_reading, test_imports, test_no_undefined_names, test_no_cycles,
          test_reexports, test_stand_ins_answer_every_call, test_no_duplicate_definitions,
          test_no_unused_imports, test_numpy_enters_behind_the_thread_cap,
          test_machine_imports_no_board, test_motor_imports_no_board, test_shape,
          test_documented, test_no_escaping_scars)

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
