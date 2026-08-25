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
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
HOST = os.path.dirname(HERE)
sys.path.insert(0, HOST)

# Packages this suite walks. `tests` is deliberately out: a suite that
# imported every suite would run them. Everything else under host/ is in,
# including testline/ and examples/ - they were left out for no reason and
# had never been checked at all, which is how three undocumented classes
# and 750 unchecked lines sat there.
PACKAGES = ('coaxial', 'coaxial_mcp', 'coaxial_ollama', 'testline')
SCRIPTS = ('tools', 'examples')

# The ceiling is the worst that survives a deliberate reading, not an ideal.
# It exists to stop the next 250-line function, not to condemn the scanners
# that are genuinely one state machine - see the exemptions.
MAX_LINES = 130
MAX_DEPTH = 7

# Character-by-character state machines, where the nesting *is* the machine
# and flattening it would cost clarity rather than buy any.
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
        for name in sorted(os.listdir(root)):
            if name.endswith('.py') and not name.startswith('_'):
                found.append('%s.%s' % (package, name[:-3]))
    return found


def sources():
    """(path, tree) for everything this suite judges, scripts included."""
    out = []
    for where in PACKAGES + SCRIPTS:
        root = os.path.join(HOST, where)
        for name in sorted(os.listdir(root)):
            if not name.endswith('.py'):
                continue
            path = os.path.join(root, name)
            text = io.open(path, encoding='utf-8').read()
            out.append((os.path.join(where, name), text, ast.parse(text)))
    return out


def depth(node, at=0):
    worst = at
    for child in ast.iter_child_nodes(node):
        worst = max(worst, depth(child, at + isinstance(child, NESTS)))
    return worst


def test_imports(r):
    """Every module imports on its own, from a cold interpreter.

    The one that keeps breaking: code moves to a new file, the name it used
    goes with it, and nothing notices until a suite that happens to touch
    that path runs. Measured five times in one afternoon - IOLog, clip,
    PROMPT, render, re - each found by a behavioural test failing somewhere
    unrelated.
    """
    for name in modules():
        try:
            importlib.import_module(name)
            r.check('%s imports' % name, True)
        except Exception as exc:                              # noqa: BLE001
            r.check('%s imports' % name, False,
                    '%s: %s' % (type(exc).__name__, exc))


def test_no_cycles(r):
    """No package module imports another that imports it back.

    `cli` imports `Chat` from `debug`, and `debug` re-exports `cli`'s names -
    which is a cycle unless the re-export is lazy. It is, through a
    module-level __getattr__; this is what says so.
    """
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
    """A name defined at the top level of two modules in one package.

    What splitting a file gets wrong: the block is copied out and left in.
    Measured - ERR_CLASS, LINK_TOOLS and CONTACT_LOST all lived in two files
    at once for as long as it took to notice.
    """
    seen = {}
    for path, text, tree in sources():
        lines = text.split(chr(10))
        for node in tree.body:
            names = []
            if isinstance(node, (ast.FunctionDef, ast.ClassDef)):
                names = [node.name]
            elif isinstance(node, ast.Assign):
                names = [t.id for t in node.targets
                         if isinstance(t, ast.Name) and t.id.isupper()]
            # The *body*, not just the name. Two modules may both define
            # SYSTEM and mean two different prompts; what is a defect is the
            # same block living in two files, which is what a split leaves.
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
    """A name used that nothing in the module defines or imports.

    The check the behavioural suites cannot make and importing cannot either:
    a module imports perfectly well with a name that only fails when the line
    using it runs. Measured - `_printable` moved to another file and `Chat`
    kept calling it; every fixture passed `out=`, so no suite ever reached
    the line, and it failed at the first real prompt.
    """
    for path, _, tree in sources():
        known = _bound(tree)
        used = {n.id for n in ast.walk(tree)
                if isinstance(n, ast.Name) and isinstance(n.ctx, ast.Load)}
        loose = sorted(used - known)
        r.check('%s uses only names it has' % path,
                not loose, ', '.join(loose[:6]))


def test_shape(r):
    """No function past the length or nesting a reader can hold.

    Both ceilings are the worst that survives a deliberate reading, so this
    can only ratchet down. It exists to stop the next 250-line, five-deep
    turn loop, which is what this file was written alongside splitting.
    """
    long_ones, deep_ones = [], []
    for path, _, tree in sources():
        for node in ast.walk(tree):
            # AsyncFunctionDef too: it is not a subclass of FunctionDef, so
            # the three async handlers in coaxial_mcp/server.py were exempt
            # from both ceilings without anyone deciding they should be.
            if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                continue
            lines = node.end_lineno - node.lineno + 1
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
    # Modules and classes only. A function's name plus its signature
    # often says everything, and the MCP handlers are documented by their
    # schema - one fact in one place, which is the rule this suite keeps.
    missing = []
    for path, _, tree in sources():
        if not ast.get_docstring(tree):
            missing.append(path + ' (module)')
        # Top level only: a ctypes Structure declared inside a function is
        # a field list, and a docstring on it would say less than its name.
        for node in tree.body:
            if not isinstance(node, ast.ClassDef):
                continue
            if node.name.startswith('_') or ast.get_docstring(node):
                continue
            missing.append('%s:%s' % (path, node.name))
    r.check('every module and class says what it is for',
            not missing, '; '.join(missing[:4]))


def test_no_escaping_scars(r):
    """chr(10) and chr(92) where a literal belongs.

    A workaround for writing files through a shell heredoc, which has no
    business in the source it produced. Left in ten places once already.
    """
    for path, text, tree in sources():
        if path.endswith(('replies.py', 'sandbox.py', 'pdfwriter.py')):
            continue        # these are *about* escaping: the first two by
                            # their tests, the third because a backslash is
                            # what a PDF literal string escapes with
        scars = [w for w in ('chr(10)', 'chr(92)') if w in text]
        r.check('%s has no heredoc scars' % path, not scars, ', '.join(scars))


ROSTER = (test_imports, test_no_undefined_names, test_no_cycles,
          test_reexports,
          test_no_duplicate_definitions, test_no_unused_imports,
          test_shape, test_documented, test_no_escaping_scars)


def main():
    report = Report()
    for test in ROSTER:
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
