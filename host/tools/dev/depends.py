#!/usr/bin/env python3
"""A new machine's needs, each written where setup reads it: modules, requirements, programs.

    python tools/dev/depends.py                  # the tree's faults, one a line; 1 where any
    python tools/dev/depends.py --absent FILE    # a requirements file's distributions absent here

A new machine learns what it lacks by lacking it. Asked of the tree's own text instead:

    unwritten   a module imported - by a statement, or by its name told `import_module`, `find_spec` or
                `__import__` - whose distribution, as this interpreter holds it, no host/requirements*.txt
                writes: a dependency that is real is a dependency that is written
    unread      a host/requirements*.txt no setup script names
    unnamed     a program looked up (`shutil.which`) or run (a subprocess's first word) that setup.ps1,
                setup/*.ps1 and env.ps1 name nowhere - installed, checked, or said to come with what is

`--absent` is setup's own probe (setup/python.ps1), the same lines the pre-commit runs: the standard
library alone, so a bare python runs it. A distribution is asked of the interpreter's metadata by the name
the file writes, never by a module name guessed from it - pillow's is PIL, fonttools' fontTools, and a
probe that guessed said both missing on a machine that held them. An extra asked (`cupy-cuda13x[ctk]`) is
its own requirements asked in turn. A module no distribution here holds is not judged: said, not a fault.
"""
import ast
import glob
import importlib.metadata as metadata
import json
import os
import re
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

#: The calls told a module's name; a subprocess's calls, their first word a program.
NAMED, RUNS = ('import_module', 'find_spec', '__import__'), ('run', 'Popen', 'check_output', 'check_call', 'call')
#: A file that holds none of these words holds none of those calls: its statements alone are read.
SOUGHT = NAMED + ('which', 'subprocess')
#: One requirement: its distribution, its extras, what it asks of its version.
ASKED = re.compile(r'\s*([A-Za-z0-9][A-Za-z0-9._-]*)\s*(?:\[([^\]]*)\])?(.*)$')


def _name(text):
    """A distribution's name as an index compares it."""
    return re.sub(r'[-_.]+', '-', text).lower()


def _release(text):
    """A version's release numbers: 2.14.1 of 2.14.1+cu130, 13 of 13.*."""
    m = re.match(r'\s*v?(\d+(?:\.\d+)*)', text)
    return tuple(int(n) for n in m.group(1).split('.')) if m else ()


def met(have, spec):
    """Whether the version `have` is what `spec` asks: each of its comparisons held, 3.14 the same as 3.14.0."""
    got = _release(have)
    for op, want in re.findall(r'(==|!=|~=|>=|<=|>|<)\s*([0-9][0-9A-Za-z.*+!-]*)', spec):
        asks = _release(want)
        width = max(len(got), len(asks))
        a, b = got + (0,) * (width - len(got)), asks + (0,) * (width - len(asks))
        same = got[:len(asks)] == asks if '*' in want else a == b
        held = {'==': same, '!=': not same, '>=': a >= b, '<=': a <= b, '>': a > b, '<': a < b,
                '~=': a >= b and got[:len(asks) - 1] == asks[:-1]}[op]
        if not held:
            return False
    return True


def _extras(text):
    """The extras a requirement's brackets name."""
    return tuple(e.strip() for e in (text or '').split(',') if e.strip())


def asked(path):
    """[(distribution, its extras, what it asks of its version)] of a requirements file: a line's comment
    cut, blank lines and pip's own options (`--extra-index-url`) left out."""
    found = []
    with open(path, encoding='utf-8') as f:
        for line in f:
            m = ASKED.match(line.split('#')[0])
            if m:
                found.append((m.group(1), _extras(m.group(2)), m.group(3).strip()))
    return found


def _under(marker, extra):
    """Whether a requirement's `marker` holds here under `extra`: by `packaging`, which the installers of
    requirements.txt leave here; without it by the extra's clause alone, its other clauses taken to hold."""
    if _name(extra) not in {_name(e) for e in re.findall(r'''extra\s*==\s*['"]([^'"]+)['"]''', marker)}:
        return False
    try:
        from packaging.markers import Marker
    except ImportError:
        return True
    return any(Marker(marker).evaluate({'extra': e}) for e in (extra, _name(extra)))


def lacking(dist, extras=(), spec='', seen=()):
    """Whether `dist` is absent here, is not the version `spec` asks, or lacks a requirement one of `extras`
    brings - each asked in turn, one already on the way (`seen`) not asked again."""
    try:
        have = metadata.version(dist)
    except metadata.PackageNotFoundError:
        return True
    if not met(have, spec):
        return True
    way = seen + (_name(dist),)
    for line in (metadata.requires(dist) or ()) if extras else ():
        need, _, marker = line.partition(';')
        m = ASKED.match(need)
        if (m and _name(m.group(1)) not in way and any(_under(marker, extra) for extra in extras)
                and lacking(m.group(1), _extras(m.group(2)), m.group(3), way)):
            return True
    return False


def absent(path):
    """A requirements file's distributions absent here, of another version than it asks, or short of an extra
    it asks: setup's probe, by the names the file writes."""
    return [dist + ('[%s]' % ','.join(extras) if extras else '')
            for dist, extras, spec in asked(path) if lacking(dist, extras, spec)]


def files(root):
    """`root`'s own files by their paths from it: git's where it is a checkout - a hook's own GIT_ variables
    are another tree's, so left out -, else every file under it but a hidden folder's and a cache's."""
    env = {k: v for k, v in os.environ.items() if not k.startswith('GIT_')}
    try:
        done = subprocess.run(['git', '-C', root, 'ls-files', '-z'], capture_output=True, env=env)
        if done.returncode == 0 and done.stdout:
            return [p for p in done.stdout.decode('utf-8', 'replace').split(chr(0)) if p]
    except OSError:
        pass
    found = []
    for folder, folders, names in os.walk(root):
        folders[:] = [d for d in folders if not d.startswith('.') and d != '__pycache__']
        found += [os.path.relpath(os.path.join(folder, n), root).replace(os.sep, '/') for n in names]
    return found


def _parsed(source):
    """`source`'s syntax tree - a notebook cell's that an editor's own lines (`%matplotlib`, `!pip`) keep from
    parsing, without them -, None where it parses neither way."""
    plain = '\n'.join(line for line in source.splitlines() if not line.lstrip().startswith(('%', '!')))
    for text in (source, plain):
        try:
            return compile(text, '<source>', 'exec', ast.PyCF_ONLY_AST | ast.PyCF_ALLOW_TOP_LEVEL_AWAIT)
        except SyntaxError:
            continue
    return None


def _sought(node):
    """([a module named], [a program named]) by one call: a name told NAMED; `which`'s; the first word of a
    list `subprocess`'s RUNS start."""
    call, first = getattr(node.func, 'attr', getattr(node.func, 'id', '')), node.args[0]
    word = first.elts[0] if isinstance(first, (ast.List, ast.Tuple)) and first.elts else None
    text = first.value if isinstance(first, ast.Constant) and isinstance(first.value, str) else None
    if call in NAMED and text:
        return [text], []
    if call == 'which' and text:
        return [], [text]
    if (call in RUNS and getattr(getattr(node.func, 'value', None), 'id', call) in ('subprocess', call)
            and isinstance(word, ast.Constant) and isinstance(word.value, str)):
        return [], [word.value]
    return [], []


def sources(root):
    """[(its path from `root`, its text)] of the python files and notebooks under `root`."""
    found = []
    for path in files(root):
        if path.endswith(('.py', '.ipynb')):
            with open(os.path.join(root, path), encoding='utf-8') as f:
                found.append((path, f.read()))
    return found


def _statements(tree):
    """A syntax tree's statements, nested ones too: every import is one, no expression is walked."""
    todo = [tree]
    while todo:
        node = todo.pop()
        yield node
        for field in ('body', 'orelse', 'finalbody', 'handlers', 'cases'):
            todo.extend(getattr(node, field, ()))


def told(texts):
    """({a module of another's: its files}, {a program: its files}, [a file not parsed]) of python files and
    notebooks, each (its path, its text): the modules imported by a statement or named to NAMED, the tree's
    own and the standard library's left out; the programs `which` looks up and `subprocess`'s RUNS start."""
    own = {part for p, _text in texts if p.endswith('.py') for part in p[:-3].split('/')}
    own |= set(sys.stdlib_module_names)
    modules, programs, unparsed = {}, {}, set()
    for path, text in texts:
        cells = ([''.join(c.get('source', '')) for c in json.loads(text).get('cells', [])
                  if c.get('cell_type') == 'code'] if path.endswith('.ipynb') else [text])
        for source in cells:
            tree = _parsed(source)
            if tree is None:
                unparsed.add(path)
                continue
            for node in ast.walk(tree) if any(word in source for word in SOUGHT) else _statements(tree):
                names, tools = [], []
                if isinstance(node, ast.Import):
                    names = [a.name for a in node.names]
                elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
                    names = [node.module]
                elif isinstance(node, ast.Call) and node.args:
                    names, tools = _sought(node)
                for name in (n.split('.')[0] for n in names):
                    if name and name not in own:
                        modules.setdefault(name, set()).add(path)
                for tool in tools:
                    if '/' not in tool and os.sep not in tool:
                        programs.setdefault(tool, set()).add(path)
    return modules, programs, sorted(unparsed)


def _where(paths):
    """The first of a set of files, and how many more."""
    return sorted(paths)[0] + (' and %d more' % (len(paths) - 1) if len(paths) > 1 else '')


def faults(root=REPO, texts=None):
    """([fault], what was counted) of the tree under `root`: its files not parsed, its modules unwritten, its
    requirements files unread, its programs unnamed - its python files and notebooks `texts` where told (a
    commit's, as staged), else as they stand under it."""
    t = time.perf_counter()
    modules, programs, unparsed = told(sources(root) if texts is None else texts)
    requirements = sorted(glob.glob(os.path.join(root, 'host', 'requirements*.txt')))
    written = {_name(dist) for path in requirements for dist, _extras_, _spec in asked(path)}
    setup = ''
    for path in [os.path.join(root, n) for n in ('setup.ps1', 'env.ps1')] + sorted(
            glob.glob(os.path.join(root, 'setup', '*.ps1'))):
        if os.path.exists(path):
            with open(path, encoding='utf-8', errors='replace') as f:
                setup += f.read() + '\n'
    here = metadata.packages_distributions()
    found = ['%s: parses neither whole nor without an editor\'s lines - its imports unread' % p for p in unparsed]
    apart = []
    for module in sorted(modules):
        come = sorted({_name(d) for d in here.get(module, ())})
        if not come:
            apart.append(module)
        elif not written.intersection(come):
            found.append('%s (%s) comes with %s: written in no host/requirements*.txt'
                         % (module, _where(modules[module]), ', '.join(come)))
    found += ['host/%s: no setup script names it - a new machine never reads it' % os.path.basename(path)
              for path in requirements if os.path.basename(path) not in setup]
    found += ['%s (%s): setup.ps1, setup/*.ps1 and env.ps1 name it nowhere - install it, check it, or say '
              'what it comes with' % (program, _where(programs[program])) for program in sorted(programs)
              if not re.search(r'(?<![\w-])%s(?![\w-])' % re.escape(program), setup, re.I)]
    counted = '%d modules of %d distributions written, %d programs named%s (%.1f s)' % (
        len(modules) - len(apart), len(written), len(programs),
        '; not here, not judged: ' + ', '.join(apart) if apart else '', time.perf_counter() - t)
    return found, counted


def main(argv=None):
    """`--absent FILE`: its distributions absent here, comma-joined, 0; else the tree's faults, 1 where any."""
    argv = sys.argv[1:] if argv is None else argv
    if argv[:1] == ['--absent']:
        print(','.join(absent(argv[1])))
        return 0
    found, counted = faults(argv[0] if argv else REPO)
    for fault in found:
        print(fault)
    print('depends: ' + ('%d faults' % len(found) if found else counted))
    return 1 if found else 0


if __name__ == '__main__':
    sys.exit(main())
